#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平衡小车机械、编码器与 IMU 调试上位机。

这个工具专用于轮子悬空的方向/PWM 特性试验、地面直线编码器比例
试验，以及静止 IMU 标定。它不参与任何 PID 闭环控制；所有电机测试
均由主板的 SafetyManager 限幅并在 1 秒未续命时自动断电。

需要固件支持：T,10 遥测、D,1 原始编码器诊断包，以及
C,<seq>,MOTOR,<left>,<right> 和 C,<seq>,IMU_CAL 命令。
"""

from __future__ import annotations

import csv
import math
import socket
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


STATE_NAMES = ("BOOT", "SELF_TESTING", "STANDBY", "MANUAL_TEST", "BALANCING", "FAULT")
FAULT_NAMES = ("NONE", "SELF_TEST_FAILED", "IMU_UNHEALTHY", "PITCH_LIMIT_EXCEEDED", "AIRBORNE_LANDING_FAILED")
T10_FIELD_COUNT = 64
DIAGNOSTIC_FIELD_COUNT = 10
LOW_SPEED_QUANTIZATION_MPS = 0.03


@dataclass(frozen=True)
class DiagnosticSample:
    sequence: int
    timestamp_ms: int
    left_ticks: int
    right_ticks: int
    left_tick_delta: int
    right_tick_delta: int
    left_ticks_per_second: float
    right_ticks_per_second: float
    received_at: float


@dataclass(frozen=True)
class EncoderTrial:
    direction: str
    distance_m: float
    left_delta: int
    right_delta: int
    left_counts_per_m: float
    right_counts_per_m: float
    left_effective_diameter_m: float
    right_effective_diameter_m: float
    left_counts_per_revolution: float
    right_counts_per_revolution: float


@dataclass(frozen=True)
class PwmResult:
    side: str
    direction: str
    duty: float
    left_speed_mps: float
    right_speed_mps: float
    sample_count: int


def median(values: Sequence[float]) -> float:
    return statistics.median(values) if values else 0.0


def population_stddev(values: Sequence[float]) -> float:
    return statistics.pstdev(values) if len(values) >= 2 else 0.0


def linear_fit(points: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """Return intercept/slope, or None when a curve has insufficient variation."""
    if len(points) < 2:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    denominator = sum((point[0] - mean_x) ** 2 for point in points)
    if denominator <= 1e-10:
        return None
    slope = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points) / denominator
    return mean_y - slope * mean_x, slope


def calculate_encoder_trial(start: DiagnosticSample, end: DiagnosticSample, distance_m: float,
                            counts_per_revolution: float, wheel_diameter_m: float,
                            direction: str) -> EncoderTrial:
    if distance_m <= 0.0 or counts_per_revolution <= 0.0 or wheel_diameter_m <= 0.0:
        raise ValueError("距离、每圈计数和轮径都必须大于 0。")
    left_delta = end.left_ticks - start.left_ticks
    right_delta = end.right_ticks - start.right_ticks
    left_abs, right_abs = abs(left_delta), abs(right_delta)
    if left_abs == 0 or right_abs == 0:
        raise ValueError("至少一侧编码器 tick 为 0；请确认小车实际走完标尺距离后再结束记录。")
    circumference = math.pi * wheel_diameter_m
    left_counts_per_m = left_abs / distance_m
    right_counts_per_m = right_abs / distance_m
    return EncoderTrial(
        direction=direction,
        distance_m=distance_m,
        left_delta=left_delta,
        right_delta=right_delta,
        left_counts_per_m=left_counts_per_m,
        right_counts_per_m=right_counts_per_m,
        left_effective_diameter_m=distance_m * counts_per_revolution / (math.pi * left_abs),
        right_effective_diameter_m=distance_m * counts_per_revolution / (math.pi * right_abs),
        left_counts_per_revolution=left_abs * circumference / distance_m,
        right_counts_per_revolution=right_abs * circumference / distance_m,
    )


class BoardWorker(QThread):
    """UDP subscription, T,10/D,1 parsing, and serialized ACK commands."""

    telemetry_ready = pyqtSignal(dict)
    diagnostic_ready = pyqtSignal(object)
    ack_received = pyqtSignal(str)
    command_failed = pyqtSignal(int, str)
    error_occurred = pyqtSignal(str)
    info_updated = pyqtSignal(str)

    def __init__(self, local_port: int, board_ip: str, command_port: int, parent=None):
        super().__init__(parent)
        self.local_port = local_port
        self.board_ip = board_ip
        self.command_port = command_port
        self.running = False
        self.sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._queue: Deque[Tuple[int, bytes, float]] = deque()
        self._inflight: Optional[Dict[str, object]] = None

    def queue_command(self, sequence: int, payload: str, timeout_s: float = 1.0):
        with self._lock:
            self._queue.append((sequence, payload.encode("ascii"), max(0.5, timeout_s)))

    def stop(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

    @staticmethod
    def _parse_t10(text: str) -> Optional[dict]:
        parts = text.split(",")
        if len(parts) != T10_FIELD_COUNT or parts[:2] != ["T", "10"]:
            return None
        try:
            values = [float(value) for value in parts[7:]]
            return {
                "sequence": int(parts[2]), "timestamp_ms": int(parts[3]),
                "state": int(parts[4]), "fault": int(parts[5]), "imu_valid": bool(int(parts[6])),
                "pitch": values[0], "pitch_rate": values[1], "accel_pitch": values[2],
                "accel_x": values[3], "accel_y": values[4], "accel_z": values[5],
                "gyro_x": values[6], "gyro_y": values[7], "gyro_z": values[8],
                "target_speed": values[9], "actual_speed": values[10],
                "target_diff": values[13], "actual_diff": values[14],
                "left_motor": values[18], "right_motor": values[19],
                "wheel_left": values[28], "wheel_right": values[29],
                "requested_pitch": values[30], "balance_error": values[31],
                "balance_raw_motor": values[35], "speed_inverted": bool(int(values[36])),
                "turn_inverted": bool(int(values[40])),
                "encoder_valid": bool(int(values[53])), "imu_calibrated": bool(int(values[54])),
                "balance_period_ms": values[55], "velocity_period_ms": values[56],
            }
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _parse_diagnostic(text: str) -> Optional[DiagnosticSample]:
        parts = text.split(",")
        if len(parts) != DIAGNOSTIC_FIELD_COUNT or parts[:2] != ["D", "1"]:
            return None
        try:
            return DiagnosticSample(
                sequence=int(parts[2]), timestamp_ms=int(parts[3]),
                left_ticks=int(parts[4]), right_ticks=int(parts[5]),
                left_tick_delta=int(parts[6]), right_tick_delta=int(parts[7]),
                left_ticks_per_second=float(parts[8]), right_ticks_per_second=float(parts[9]),
                received_at=time.monotonic(),
            )
        except ValueError:
            return None

    def _service_command_queue(self, sock: socket.socket, now: float):
        if self._inflight is not None:
            elapsed = now - float(self._inflight["sent_at"])
            timeout_s = float(self._inflight["timeout_s"])
            if elapsed < timeout_s:
                return
            if int(self._inflight["retries"]) >= 2:
                sequence = int(self._inflight["sequence"])
                self._inflight = None
                self.command_failed.emit(sequence, "ACK_TIMEOUT")
                return
            try:
                sock.sendto(self._inflight["payload"], (self.board_ip, self.command_port))  # type: ignore[arg-type]
                self._inflight["sent_at"] = now
                self._inflight["retries"] = int(self._inflight["retries"]) + 1
            except OSError as error:
                self.error_occurred.emit(f"命令重发失败：{error}")
            return
        with self._lock:
            item = self._queue.popleft() if self._queue else None
        if item is None:
            return
        sequence, payload, timeout_s = item
        try:
            sock.sendto(payload, (self.board_ip, self.command_port))
            self._inflight = {
                "sequence": sequence, "payload": payload, "timeout_s": timeout_s,
                "sent_at": now, "retries": 0,
            }
        except OSError as error:
            self.command_failed.emit(sequence, "SEND_FAILED")
            self.error_occurred.emit(f"命令发送失败：{error}")

    def _accept_ack(self, text: str):
        if self._inflight is None:
            return
        fields = text.split(",", 3)
        try:
            sequence = int(fields[1])
        except (IndexError, ValueError):
            return
        if sequence == int(self._inflight["sequence"]):
            self._inflight = None

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.08)
            self.sock = sock
        except OSError as error:
            self.error_occurred.emit(f"UDP 端口绑定失败：{error}")
            return

        self.running = True
        last_subscription = 0.0
        try:
            while self.running:
                now = time.monotonic()
                if now - last_subscription >= 1.0:
                    try:
                        sock.sendto(b"H\n", (self.board_ip, self.command_port))
                    except OSError as error:
                        self.error_occurred.emit(f"遥测订阅失败：{error}")
                    last_subscription = now
                self._service_command_queue(sock, now)
                try:
                    data, _ = sock.recvfrom(768)
                except socket.timeout:
                    continue
                except OSError:
                    break
                text = data.decode("ascii", errors="replace").strip()
                if text.startswith("T,"):
                    telemetry = self._parse_t10(text)
                    if telemetry is not None:
                        self.telemetry_ready.emit(telemetry)
                elif text.startswith("D,"):
                    diagnostic = self._parse_diagnostic(text)
                    if diagnostic is not None:
                        self.diagnostic_ready.emit(diagnostic)
                elif text.startswith("A,"):
                    self._accept_ack(text)
                    self.ack_received.emit(text)
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self.sock = None
            self.info_updated.emit("主板 UDP 调试连接已停止")


class CharacterizationWindow(QMainWindow):
    """Four guided workflows; no configuration is written to the board."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("平衡小车机械与 IMU 调试上位机")
        self.resize(1260, 860)
        self.worker: Optional[BoardWorker] = None
        self.sequence = int(time.time_ns() & 0x7FFFFFFF)
        self.pending: Dict[int, str] = {}
        self.latest_telemetry: Optional[dict] = None
        self.latest_diagnostic: Optional[DiagnosticSample] = None
        self.encoder_start: Optional[DiagnosticSample] = None
        self.encoder_trials: List[EncoderTrial] = []
        self.pwm_results: List[PwmResult] = []
        self.pwm_state: Optional[dict] = None
        self.imu_recording: Optional[dict] = None
        self.last_motor_command = (0.0, 0.0)
        self.telemetry_file = self.telemetry_writer = self.diagnostic_file = self.diagnostic_writer = None
        self.records_directory = Path(__file__).resolve().parent / "records"

        self.pwm_timer = QTimer(self)
        self.pwm_timer.setInterval(80)
        self.pwm_timer.timeout.connect(self._advance_pwm_sweep)
        self.imu_timer = QTimer(self)
        self.imu_timer.setInterval(100)
        self.imu_timer.timeout.connect(self._advance_imu_recording)

        self._build_ui()

    @staticmethod
    def _double(value: float, minimum: float, maximum: float, step: float, decimals: int = 3) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(decimals)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        connection = QGroupBox("Wi-Fi 连接与安全状态")
        grid = QGridLayout(connection)
        self.ip_edit = QLineEdit("192.168.4.1")
        self.local_port = QSpinBox(); self.local_port.setRange(1024, 65535); self.local_port.setValue(9000)
        self.command_port = QSpinBox(); self.command_port.setRange(1, 65535); self.command_port.setValue(9001)
        self.connect_button = QPushButton("连接并订阅")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setEnabled(False)
        self.stop_button = QPushButton("停止测试 / 断电")
        self.stop_button.setEnabled(False)
        self.link_label = QLabel("未连接")
        self.live_label = QLabel("等待 T,10 / D,1")
        grid.addWidget(QLabel("主板 IP:"), 0, 0); grid.addWidget(self.ip_edit, 0, 1)
        grid.addWidget(QLabel("本地 UDP:"), 0, 2); grid.addWidget(self.local_port, 0, 3)
        grid.addWidget(QLabel("命令端口:"), 0, 4); grid.addWidget(self.command_port, 0, 5)
        grid.addWidget(self.connect_button, 0, 6); grid.addWidget(self.disconnect_button, 0, 7)
        grid.addWidget(self.stop_button, 0, 8); grid.addWidget(self.link_label, 1, 0, 1, 3)
        grid.addWidget(self.live_label, 1, 3, 1, 6)
        self.connect_button.clicked.connect(self.connect_board)
        self.disconnect_button.clicked.connect(self.disconnect_board)
        self.stop_button.clicked.connect(self.stop_motors)
        root.addWidget(connection)

        self.tabs = QTabWidget()
        self._build_direction_tab()
        self._build_encoder_tab()
        self._build_pwm_tab()
        self._build_imu_tab()
        root.addWidget(self.tabs, 1)

        log_group = QGroupBox("通信与试验日志")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit(); self.log_view.setReadOnly(True); self.log_view.setMaximumHeight(130)
        self.log_view.document().setMaximumBlockCount(800)
        log_layout.addWidget(self.log_view)
        root.addWidget(log_group)

    def _build_direction_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        warning = QLabel(
            "<b>仅限车轮悬空。</b>每次开环命令都受主板 ±0.35 与 1 秒超时保护；松开/超时后请确认轮子停止。"
        )
        warning.setWordWrap(True); layout.addWidget(warning)
        self.direction_suspended = QCheckBox("我已确认车轮悬空、周围无人接触轮子")
        layout.addWidget(self.direction_suspended)

        test_group = QGroupBox("开环电机与编码器方向检查")
        test_grid = QGridLayout(test_group)
        self.direction_power = self._double(0.15, 0.02, 0.35, 0.01)
        test_grid.addWidget(QLabel("测试占空比:"), 0, 0); test_grid.addWidget(self.direction_power, 0, 1)
        buttons = [
            ("左轮 +", 1, 0, 1.0, 0.0), ("左轮 −", 1, 1, -1.0, 0.0),
            ("右轮 +", 1, 2, 0.0, 1.0), ("右轮 −", 1, 3, 0.0, -1.0),
            ("双轮 +（应前进）", 2, 0, 1.0, 1.0), ("双轮 −（应后退）", 2, 1, -1.0, -1.0),
            ("Δv>0：右轮更快", 2, 2, 0.55, 1.0),
        ]
        for label, row, column, left_scale, right_scale in buttons:
            button = QPushButton(label)
            button.clicked.connect(lambda _=False, l=left_scale, r=right_scale: self.run_direction_test(l, r))
            test_grid.addWidget(button, row, column)
        self.direction_status = QLabel("D,1 到达后会显示当前 signed tick / wheel speed。")
        self.direction_status.setWordWrap(True)
        test_grid.addWidget(self.direction_status, 3, 0, 1, 4)
        layout.addWidget(test_group)

        verify_group = QGroupBox("人工观察确认（用于冻结符号，不让 PID 补偿方向错误）")
        verify_layout = QVBoxLayout(verify_group)
        self.confirm_left_motor = QCheckBox("左轮正向命令（+）对应小车前进")
        self.confirm_right_motor = QCheckBox("右轮正向命令（+）对应小车前进")
        self.confirm_encoder = QCheckBox("小车前进时，左右编码器速度同号且为正")
        self.confirm_diff = QCheckBox("正转差目标 Δv=vR−vL>0 时，右轮确实比左轮快")
        self.confirm_balance_output = QCheckBox("已在低风险平衡测试确认 motorOutputInverted，不依赖 PID 反向补偿")
        self.confirm_loop_outputs = QCheckBox("已确认速度/转差环 outputInverted，不依赖 PID 反向补偿")
        for check in (self.confirm_left_motor, self.confirm_right_motor, self.confirm_encoder,
                      self.confirm_diff, self.confirm_balance_output, self.confirm_loop_outputs):
            verify_layout.addWidget(check)
        actions = QHBoxLayout()
        self.freeze_button = QPushButton("生成并冻结方向确认记录")
        self.freeze_button.clicked.connect(self.freeze_direction_record)
        self.export_direction_button = QPushButton("导出完整调试报告")
        self.export_direction_button.clicked.connect(self.export_report)
        actions.addWidget(self.freeze_button); actions.addWidget(self.export_direction_button); actions.addStretch()
        verify_layout.addLayout(actions)
        self.direction_result = QTextEdit(); self.direction_result.setReadOnly(True); self.direction_result.setMinimumHeight(125)
        verify_layout.addWidget(self.direction_result)
        layout.addWidget(verify_group)
        layout.addStretch()
        self.tabs.addTab(page, "1. 电机 / 编码器方向")

    def _build_encoder_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        instructions = QLabel(
            "在地面直线标尺上至少测试 2 m：先在起点点击“记录起点”，匀速走到终点后点击“记录终点”。"
            " 前进、后退各记录一次。计算使用 D,1 原始累计 tick，不受 40 ms 速度量化影响。"
        )
        instructions.setWordWrap(True); layout.addWidget(instructions)
        config = QGroupBox("当前机械参数（仅用于计算建议，不会写入固件）")
        form = QFormLayout(config)
        self.encoder_distance = self._double(2.0, 0.2, 100.0, 0.1)
        self.encoder_counts = self._double(466.0, 1.0, 20000.0, 1.0, 2)
        self.encoder_diameter = self._double(0.064, 0.005, 0.3, 0.001, 4)
        self.encoder_direction = QComboBox(); self.encoder_direction.addItems(["前进", "后退"])
        form.addRow("实际标尺距离 (m):", self.encoder_distance)
        form.addRow("当前 countsPerWheelRevolution:", self.encoder_counts)
        form.addRow("当前轮径 (m):", self.encoder_diameter)
        form.addRow("本次方向:", self.encoder_direction)
        layout.addWidget(config)
        actions = QHBoxLayout()
        self.encoder_start_button = QPushButton("记录起点 tick")
        self.encoder_end_button = QPushButton("记录终点并计算")
        self.encoder_end_button.setEnabled(False)
        self.encoder_start_button.clicked.connect(self.capture_encoder_start)
        self.encoder_end_button.clicked.connect(self.capture_encoder_end)
        actions.addWidget(self.encoder_start_button); actions.addWidget(self.encoder_end_button); actions.addStretch()
        layout.addLayout(actions)
        self.encoder_status = QLabel("尚未记录起点。")
        layout.addWidget(self.encoder_status)
        self.encoder_table = QTableWidget(0, 10)
        self.encoder_table.setHorizontalHeaderLabels([
            "方向", "距离 m", "Δtick 左", "Δtick 右", "左 tick/m", "右 tick/m",
            "左有效轮径 m", "右有效轮径 m", "左建议 CPR", "右建议 CPR",
        ])
        self.encoder_table.setAlternatingRowColors(True)
        layout.addWidget(self.encoder_table)
        self.encoder_summary = QTextEdit(); self.encoder_summary.setReadOnly(True); self.encoder_summary.setMinimumHeight(110)
        layout.addWidget(self.encoder_summary)
        self.tabs.addTab(page, "2. 编码器比例")

    def _build_pwm_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        prompt = QLabel(
            "扫描在 STANDBY / MANUAL_TEST 状态执行，自动以 0.5 s 心跳续命；结束、取消、连接中断都会发送 STOP。"
            " 低于约 0.03 m/s 的结果会标记为量化噪声区，不能据此调大 PI。"
        )
        prompt.setWordWrap(True); layout.addWidget(prompt)
        self.pwm_suspended = QCheckBox("我已确认车轮悬空，允许自动 PWM 扫描")
        layout.addWidget(self.pwm_suspended)
        controls = QGroupBox("占空比—稳态轮速扫描")
        grid = QGridLayout(controls)
        self.pwm_side = QComboBox(); self.pwm_side.addItems(["左轮", "右轮", "双轮"])
        self.pwm_direction = QComboBox(); self.pwm_direction.addItems(["正向", "反向"])
        self.pwm_start = self._double(0.04, 0.02, 0.35, 0.01)
        self.pwm_end = self._double(0.30, 0.02, 0.35, 0.01)
        self.pwm_step = self._double(0.03, 0.01, 0.15, 0.01)
        self.pwm_hold = self._double(1.5, 0.8, 6.0, 0.1)
        fields = [("测试侧:", self.pwm_side), ("方向:", self.pwm_direction),
                  ("起始占空比:", self.pwm_start), ("终止占空比:", self.pwm_end),
                  ("步长:", self.pwm_step), ("每级保持 s:", self.pwm_hold)]
        for index, (label, field) in enumerate(fields):
            row, column = divmod(index, 3)
            grid.addWidget(QLabel(label), row, column * 2); grid.addWidget(field, row, column * 2 + 1)
        self.pwm_start_button = QPushButton("开始扫描")
        self.pwm_cancel_button = QPushButton("取消并停止")
        self.pwm_cancel_button.setEnabled(False)
        self.pwm_start_button.clicked.connect(self.start_pwm_sweep)
        self.pwm_cancel_button.clicked.connect(self.cancel_pwm_sweep)
        grid.addWidget(self.pwm_start_button, 2, 0, 1, 2); grid.addWidget(self.pwm_cancel_button, 2, 2, 1, 2)
        self.pwm_status = QLabel("未开始扫描。")
        grid.addWidget(self.pwm_status, 3, 0, 1, 6)
        layout.addWidget(controls)
        self.pwm_table = QTableWidget(0, 6)
        self.pwm_table.setHorizontalHeaderLabels(["侧", "方向", "占空比", "左轮 m/s", "右轮 m/s", "有效样本"])
        self.pwm_table.setAlternatingRowColors(True)
        layout.addWidget(self.pwm_table)
        self.pwm_summary = QTextEdit(); self.pwm_summary.setReadOnly(True); self.pwm_summary.setMinimumHeight(105)
        layout.addWidget(self.pwm_summary)
        self.tabs.addTab(page, "3. PWM / 死区")

    def _build_imu_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        overview = QLabel(
            "先在 STANDBY 静止完成陀螺仪零偏标定，再使用静态记录校核 pitchAxis、角度/陀螺仪反相和加速度角偏置。"
            " 本页只生成建议和记录；pitchAxis、反相、滤波时间常数均是固件编译期配置。"
        )
        overview.setWordWrap(True); layout.addWidget(overview)
        action_group = QGroupBox("静止 IMU 标定与已知角度校核")
        grid = QGridLayout(action_group)
        self.imu_stationary = QCheckBox("车体已静止放稳，电机已停止")
        self.imu_calibrate_button = QPushButton("执行陀螺仪零偏标定（约 1 秒）")
        self.imu_calibrate_button.clicked.connect(self.calibrate_imu)
        self.imu_known_angle = self._double(0.0, -45.0, 45.0, 0.1)
        self.imu_current_offset = self._double(1.5, -30.0, 30.0, 0.1)
        self.imu_record_seconds = self._double(3.0, 1.0, 20.0, 0.5)
        self.imu_record_button = QPushButton("记录静态样本")
        self.imu_record_button.clicked.connect(self.start_imu_recording)
        grid.addWidget(self.imu_stationary, 0, 0, 1, 3)
        grid.addWidget(self.imu_calibrate_button, 0, 3, 1, 2)
        grid.addWidget(QLabel("已知机械角 (°):"), 1, 0); grid.addWidget(self.imu_known_angle, 1, 1)
        grid.addWidget(QLabel("当前 accel offset (°):"), 1, 2); grid.addWidget(self.imu_current_offset, 1, 3)
        grid.addWidget(QLabel("记录秒数:"), 2, 0); grid.addWidget(self.imu_record_seconds, 2, 1)
        grid.addWidget(self.imu_record_button, 2, 2, 1, 2)
        self.imu_status = QLabel("等待 T,10 IMU 遥测。")
        grid.addWidget(self.imu_status, 3, 0, 1, 5)
        layout.addWidget(action_group)
        balance_group = QGroupBox("目标平衡角与互补滤波验收记录")
        form = QFormLayout(balance_group)
        self.imu_target_pitch = self._double(-2.09, -20.0, 20.0, 0.01, 3)
        self.imu_filter_tau = self._double(0.25, 0.05, 2.0, 0.01, 3)
        self.imu_balance_seconds = self._double(5.0, 2.0, 30.0, 0.5)
        self.imu_balance_record_button = QPushButton("记录零速度平衡窗口")
        self.imu_balance_record_button.clicked.connect(self.start_balance_recording)
        form.addRow("当前 targetPitchDegrees (°):", self.imu_target_pitch)
        form.addRow("当前互补滤波时间常数 τ (s):", self.imu_filter_tau)
        form.addRow("记录窗口 (s):", self.imu_balance_seconds)
        form.addRow(self.imu_balance_record_button)
        layout.addWidget(balance_group)
        self.imu_result = QTextEdit(); self.imu_result.setReadOnly(True); self.imu_result.setMinimumHeight(235)
        layout.addWidget(self.imu_result)
        self.tabs.addTab(page, "4. IMU / 姿态")

    def log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.append(f"[{timestamp}] {message}")
        self.statusBar().showMessage(message, 5000)

    def _next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0x7FFFFFFF
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence

    def connect_board(self):
        self.disconnect_board()
        self.worker = BoardWorker(self.local_port.value(), self.ip_edit.text().strip(), self.command_port.value())
        self.worker.telemetry_ready.connect(self.on_telemetry)
        self.worker.diagnostic_ready.connect(self.on_diagnostic)
        self.worker.ack_received.connect(self.on_ack)
        self.worker.command_failed.connect(self.on_command_failed)
        self.worker.error_occurred.connect(lambda message: self.log(f"错误：{message}"))
        self.worker.info_updated.connect(self.log)
        self.worker.start()
        self.start_session_log()
        self.connect_button.setEnabled(False); self.disconnect_button.setEnabled(True); self.stop_button.setEnabled(True)
        self.ip_edit.setEnabled(False); self.local_port.setEnabled(False); self.command_port.setEnabled(False)
        self.link_label.setText("正在等待主板 T,10 与 D,1 …")
        self.log("已开始订阅；请烧录支持 D,1 / MOTOR / IMU_CAL 的固件。")

    def disconnect_board(self):
        self.cancel_pwm_sweep(silent=True)
        self.imu_timer.stop(); self.imu_recording = None
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1500)
            self.worker = None
        self.close_session_log()
        self.pending.clear()
        if hasattr(self, "connect_button"):
            self.connect_button.setEnabled(True); self.disconnect_button.setEnabled(False); self.stop_button.setEnabled(False)
            self.ip_edit.setEnabled(True); self.local_port.setEnabled(True); self.command_port.setEnabled(True)
            self.link_label.setText("未连接")

    def send_control(self, action: str, *values: float, timeout_s: float = 1.0):
        if self.worker is None:
            self.log("错误：尚未连接主板")
            return
        sequence = self._next_sequence()
        suffix = "".join(f",{value:.4f}" for value in values)
        text = f"C,{sequence},{action}{suffix}\n"
        self.pending[sequence] = text.strip()
        self.worker.queue_command(sequence, text, timeout_s)
        self.log(f"TX {text.strip()}")

    def send_motor(self, left: float, right: float, timeout_s: float = 1.0):
        self.last_motor_command = (left, right)
        self.send_control("MOTOR", left, right, timeout_s=timeout_s)

    def stop_motors(self):
        self.pwm_timer.stop()
        if self.pwm_state is not None:
            self.pwm_state = None
            self.pwm_start_button.setEnabled(True); self.pwm_cancel_button.setEnabled(False)
            self.pwm_status.setText("已停止 PWM 扫描。")
        self.last_motor_command = (0.0, 0.0)
        self.send_control("STOP")

    def on_ack(self, text: str):
        self.log(f"RX {text}")
        fields = text.split(",", 3)
        try:
            sequence = int(fields[1])
        except (ValueError, IndexError):
            return
        description = self.pending.pop(sequence, "未知命令")
        if len(fields) < 4 or fields[2] != "OK":
            self.log(f"命令被拒绝：{description}；{fields[3] if len(fields) >= 4 else text}")

    def on_command_failed(self, sequence: int, reason: str):
        description = self.pending.pop(sequence, "未知命令")
        self.log(f"命令失败 #{sequence}：{reason}；{description}")

    def on_telemetry(self, telemetry: dict):
        self.latest_telemetry = telemetry
        state_index = telemetry["state"]
        state = STATE_NAMES[state_index] if 0 <= state_index < len(STATE_NAMES) else "UNKNOWN"
        fault_index = telemetry["fault"]
        fault = FAULT_NAMES[fault_index] if 0 <= fault_index < len(FAULT_NAMES) else "UNKNOWN"
        self.link_label.setText(f"已连接：{state} / {fault}")
        self.live_label.setText(
            f"T,10 #{telemetry['sequence']}  pitch={telemetry['pitch']:.3f}°  "
            f"gyro(X/Y/Z)={telemetry['gyro_x']:.3f}/{telemetry['gyro_y']:.3f}/{telemetry['gyro_z']:.3f} °/s  "
            f"轮速 L/R={telemetry['wheel_left']:.3f}/{telemetry['wheel_right']:.3f} m/s"
        )
        self.imu_status.setText(
            f"状态={state}；IMU={'有效' if telemetry['imu_valid'] else '无效'}；"
            f"gyro 已标定={'是' if telemetry['imu_calibrated'] else '否'}；"
            f"pitch/accel={telemetry['pitch']:.3f}/{telemetry['accel_pitch']:.3f}°。"
        )
        self._write_telemetry(telemetry, state, fault)
        if self.imu_recording is not None:
            self.imu_recording["samples"].append(telemetry)

    def on_diagnostic(self, diagnostic: DiagnosticSample):
        self.latest_diagnostic = diagnostic
        self._write_diagnostic(diagnostic)
        meters_per_tick = self.meters_per_tick()
        left_speed = diagnostic.left_ticks_per_second * meters_per_tick
        right_speed = diagnostic.right_ticks_per_second * meters_per_tick
        command_left, command_right = self.last_motor_command
        self.direction_status.setText(
            f"D,1 #{diagnostic.sequence}：ticks L/R={diagnostic.left_ticks}/{diagnostic.right_ticks}；"
            f"Δtick={diagnostic.left_tick_delta}/{diagnostic.right_tick_delta}；"
            f"估计轮速={left_speed:.4f}/{right_speed:.4f} m/s；"
            f"最近电机命令={command_left:.2f}/{command_right:.2f}。"
        )

    def meters_per_tick(self) -> float:
        return math.pi * self.encoder_diameter.value() / self.encoder_counts.value()

    # ---- 1. Direction -------------------------------------------------
    def run_direction_test(self, left_scale: float, right_scale: float):
        if not self.direction_suspended.isChecked():
            self.log("请先确认车轮悬空，才允许电机方向测试。")
            return
        power = self.direction_power.value()
        self.send_motor(power * left_scale, power * right_scale)
        # The board has its own 1-second watchdog.  The host's stop is an
        # additional guard; a fresh button press supersedes this brief pulse.
        QTimer.singleShot(850, self.stop_motors)

    def freeze_direction_record(self):
        checks = [
            self.confirm_left_motor, self.confirm_right_motor, self.confirm_encoder,
            self.confirm_diff, self.confirm_balance_output, self.confirm_loop_outputs,
        ]
        if not all(check.isChecked() for check in checks):
            self.direction_result.setPlainText(
                "尚未冻结：六项符号确认必须全部完成。请先用悬空测试确认机械/编码器符号，"
                "再用受保护的低速平衡测试确认三个控制输出符号。"
            )
            return
        timestamp = datetime.now().isoformat(timespec="seconds")
        self.direction_result.setPlainText(
            f"方向确认已记录：{timestamp}\n\n"
            "保持 include/config/vehicle_config.h 中已经确认的 leftDirectionInverted、"
            "rightDirectionInverted、motorOutputInverted、速度/转差 outputInverted。\n"
            "后续 PID 只能调响应与误差，不能用 Ki/Kp 去掩盖任何方向错误。\n"
            "本记录只冻结确认结果；该上位机不会偷偷改写固件方向配置。"
        )
        self.log("方向确认已冻结到本次报告。")

    # ---- 2. Encoder scale --------------------------------------------
    def capture_encoder_start(self):
        if self.latest_diagnostic is None:
            self.encoder_status.setText("尚未收到 D,1 原始 tick 包，无法记录。")
            return
        self.encoder_start = self.latest_diagnostic
        self.encoder_end_button.setEnabled(True)
        self.encoder_status.setText(
            f"起点已记录：L={self.encoder_start.left_ticks}，R={self.encoder_start.right_ticks}。"
            "沿标尺直线行驶后在终点点击计算。"
        )

    def capture_encoder_end(self):
        if self.encoder_start is None or self.latest_diagnostic is None:
            self.encoder_status.setText("请先记录起点并等待 D,1。")
            return
        try:
            trial = calculate_encoder_trial(
                self.encoder_start, self.latest_diagnostic, self.encoder_distance.value(),
                self.encoder_counts.value(), self.encoder_diameter.value(), self.encoder_direction.currentText(),
            )
        except ValueError as error:
            self.encoder_status.setText(f"计算失败：{error}")
            return
        self.encoder_trials.append(trial)
        row = self.encoder_table.rowCount(); self.encoder_table.insertRow(row)
        values = [
            trial.direction, f"{trial.distance_m:.3f}", str(trial.left_delta), str(trial.right_delta),
            f"{trial.left_counts_per_m:.2f}", f"{trial.right_counts_per_m:.2f}",
            f"{trial.left_effective_diameter_m:.5f}", f"{trial.right_effective_diameter_m:.5f}",
            f"{trial.left_counts_per_revolution:.2f}", f"{trial.right_counts_per_revolution:.2f}",
        ]
        for column, value in enumerate(values):
            self.encoder_table.setItem(row, column, QTableWidgetItem(value))
        self.encoder_start = None; self.encoder_end_button.setEnabled(False)
        self.encoder_status.setText("本次已计算。请切换前进/后退后重复测试，或记录新的起点。")
        self.update_encoder_summary()

    def update_encoder_summary(self):
        if not self.encoder_trials:
            return
        left_counts_per_m = median([trial.left_counts_per_m for trial in self.encoder_trials])
        right_counts_per_m = median([trial.right_counts_per_m for trial in self.encoder_trials])
        mean_counts_per_m = 0.5 * (left_counts_per_m + right_counts_per_m)
        left_cpr = median([trial.left_counts_per_revolution for trial in self.encoder_trials])
        right_cpr = median([trial.right_counts_per_revolution for trial in self.encoder_trials])
        left_diameter = median([trial.left_effective_diameter_m for trial in self.encoder_trials])
        right_diameter = median([trial.right_effective_diameter_m for trial in self.encoder_trials])
        mismatch = abs(left_counts_per_m - right_counts_per_m) / max(mean_counts_per_m, 1e-9) * 100.0
        self.encoder_summary.setPlainText(
            f"{len(self.encoder_trials)} 次试验的中位数：左/右 tick/m={left_counts_per_m:.2f}/{right_counts_per_m:.2f}；"
            f"差异={mismatch:.2f}%\n"
            f"固定 {self.encoder_counts.value():.2f} CPR 时的有效轮径：左/右={left_diameter:.5f}/{right_diameter:.5f} m\n"
            f"固定轮径 {self.encoder_diameter.value():.5f} m 时建议 CPR：左/右={left_cpr:.2f}/{right_cpr:.2f}\n"
            f"若保留单一比例，使用约 {mean_counts_per_m * math.pi * self.encoder_diameter.value():.2f} CPR；"
            f"若差异明显，优先在固件引入左右比例，左/右速度比例系数可先取 "
            f"{mean_counts_per_m / left_counts_per_m:.5f}/{mean_counts_per_m / right_counts_per_m:.5f}。"
        )

    # ---- 3. PWM / dead zone ------------------------------------------
    def start_pwm_sweep(self):
        if self.pwm_state is not None:
            return
        if not self.pwm_suspended.isChecked():
            self.pwm_status.setText("请先确认车轮悬空。")
            return
        if self.latest_diagnostic is None:
            self.pwm_status.setText("等待 D,1 后才能扫描。")
            return
        start, end, step = self.pwm_start.value(), self.pwm_end.value(), self.pwm_step.value()
        if end < start:
            self.pwm_status.setText("终止占空比必须不小于起始占空比。")
            return
        levels: List[float] = []
        value = start
        while value <= end + step * 0.25:
            levels.append(min(value, end))
            value += step
        if len(levels) > 40:
            self.pwm_status.setText("扫描级数超过 40，请加大步长或缩小范围。")
            return
        self.pwm_state = {
            "side": self.pwm_side.currentText(), "direction": self.pwm_direction.currentText(),
            "levels": levels, "index": 0, "started": time.monotonic(), "last_refresh": 0.0,
            "samples": [], "settle_s": min(0.50, self.pwm_hold.value() * 0.4),
        }
        self.pwm_start_button.setEnabled(False); self.pwm_cancel_button.setEnabled(True)
        self._start_pwm_level()
        self.pwm_timer.start()

    def _pwm_command(self, duty: float) -> Tuple[float, float]:
        if self.pwm_state is None:
            return 0.0, 0.0
        sign = 1.0 if self.pwm_state["direction"] == "正向" else -1.0
        power = duty * sign
        side = self.pwm_state["side"]
        return (power, 0.0) if side == "左轮" else (0.0, power) if side == "右轮" else (power, power)

    def _start_pwm_level(self):
        if self.pwm_state is None:
            return
        duty = self.pwm_state["levels"][self.pwm_state["index"]]
        self.pwm_state["started"] = time.monotonic()
        self.pwm_state["last_refresh"] = 0.0
        self.pwm_state["samples"] = []
        left, right = self._pwm_command(duty)
        self.send_motor(left, right)
        self.pwm_state["last_refresh"] = time.monotonic()
        self.pwm_status.setText(
            f"扫描 {self.pwm_state['side']} {self.pwm_state['direction']}："
            f"第 {self.pwm_state['index'] + 1}/{len(self.pwm_state['levels'])} 级，duty={duty:.3f}。"
        )

    def _advance_pwm_sweep(self):
        if self.pwm_state is None:
            self.pwm_timer.stop()
            return
        now = time.monotonic()
        duty = self.pwm_state["levels"][self.pwm_state["index"]]
        if now - self.pwm_state["last_refresh"] >= 0.50:
            left, right = self._pwm_command(duty)
            self.send_motor(left, right)
            self.pwm_state["last_refresh"] = now
        elapsed = now - self.pwm_state["started"]
        if elapsed >= self.pwm_state["settle_s"] and self.latest_diagnostic is not None:
            self.pwm_state["samples"].append(self.latest_diagnostic)
        if elapsed < self.pwm_hold.value():
            return
        samples: List[DiagnosticSample] = self.pwm_state["samples"]
        if samples:
            meters_per_tick = self.meters_per_tick()
            left_speed = median([sample.left_ticks_per_second * meters_per_tick for sample in samples])
            right_speed = median([sample.right_ticks_per_second * meters_per_tick for sample in samples])
        else:
            left_speed = right_speed = 0.0
        result = PwmResult(self.pwm_state["side"], self.pwm_state["direction"], duty,
                           left_speed, right_speed, len(samples))
        self.pwm_results.append(result)
        row = self.pwm_table.rowCount(); self.pwm_table.insertRow(row)
        values = [result.side, result.direction, f"{result.duty:.3f}",
                  f"{result.left_speed:.4f}", f"{result.right_speed:.4f}", str(result.sample_count)]
        for column, value in enumerate(values):
            self.pwm_table.setItem(row, column, QTableWidgetItem(value))
        self.pwm_state["index"] += 1
        if self.pwm_state["index"] >= len(self.pwm_state["levels"]):
            self._finish_pwm_sweep("扫描完成。")
            return
        self._start_pwm_level()

    def _finish_pwm_sweep(self, status: str):
        self.pwm_timer.stop()
        self.pwm_state = None
        self.pwm_start_button.setEnabled(True); self.pwm_cancel_button.setEnabled(False)
        self.pwm_status.setText(status)
        self.last_motor_command = (0.0, 0.0)
        self.send_control("STOP")
        self.update_pwm_summary()

    def cancel_pwm_sweep(self, silent: bool = False):
        if self.pwm_state is None:
            return
        self.pwm_timer.stop()
        self.pwm_state = None
        self.pwm_start_button.setEnabled(True); self.pwm_cancel_button.setEnabled(False)
        if not silent:
            self.pwm_status.setText("扫描已取消，已请求停止电机。")
        self.last_motor_command = (0.0, 0.0)
        if self.worker is not None:
            self.send_control("STOP")

    def update_pwm_summary(self):
        if not self.pwm_results:
            return
        lines = ["曲线摘要（按侧和方向分别计算；速度绝对值低于 0.03 m/s 属于量化噪声区）："]
        groups: Dict[Tuple[str, str], List[PwmResult]] = {}
        for result in self.pwm_results:
            groups.setdefault((result.side, result.direction), []).append(result)
        for (side, direction), rows in groups.items():
            speed_of = (lambda row: abs(row.left_speed_mps)) if side == "左轮" else \
                       (lambda row: abs(row.right_speed_mps)) if side == "右轮" else \
                       (lambda row: 0.5 * (abs(row.left_speed_mps) + abs(row.right_speed_mps)))
            moving = [row for row in rows if speed_of(row) >= LOW_SPEED_QUANTIZATION_MPS]
            start_duty = min((row.duty for row in moving), default=None)
            fit = linear_fit([(row.duty, speed_of(row)) for row in moving])
            if start_duty is None:
                lines.append(f"{side} {direction}：本次未超过 {LOW_SPEED_QUANTIZATION_MPS:.2f} m/s，无法可靠识别起转点。")
                continue
            description = f"{side} {direction}：最小可靠起转 duty≈{start_duty:.3f}"
            if fit is not None and fit[1] > 1e-6:
                estimated_dead_zone = max(0.0, min(0.35, -fit[0] / fit[1]))
                description += f"；近似斜率={fit[1]:.3f} m/s/duty；线性外推死区≈{estimated_dead_zone:.3f}"
            lines.append(description)
        lines.append("若左右曲线差异明显，应采用每侧死区补偿 + 速度前馈；不要把 PWM 频率当作常规 PID 扫描参数。")
        self.pwm_summary.setPlainText("\n".join(lines))

    # ---- 4. IMU -------------------------------------------------------
    def calibrate_imu(self):
        if not self.imu_stationary.isChecked():
            self.imu_status.setText("请先确认车体静止、轮子不转。")
            return
        if self.latest_telemetry is not None and self.latest_telemetry["state"] != 2:
            self.imu_status.setText("陀螺仪标定仅允许 STANDBY；请先停止平衡/测试。")
            return
        self.imu_status.setText("正在标定，约 1 秒内请勿触碰车体…")
        self.send_control("IMU_CAL", timeout_s=2.0)

    def start_imu_recording(self):
        if not self.imu_stationary.isChecked():
            self.imu_status.setText("请先确认静止，才记录 IMU 样本。")
            return
        if self.latest_telemetry is None:
            self.imu_status.setText("等待 T,10。")
            return
        self.imu_recording = {
            "kind": "static", "started": time.monotonic(), "duration": self.imu_record_seconds.value(), "samples": [],
        }
        self.imu_record_button.setEnabled(False); self.imu_timer.start()
        self.imu_status.setText("正在记录静态 IMU 样本…")

    def start_balance_recording(self):
        if self.latest_telemetry is None or self.latest_telemetry["state"] != 4:
            self.imu_result.setPlainText("零速度平衡窗口只能在 BALANCING 状态记录；请先确认安全，且 DRIVE/ TURN 均为 0。")
            return
        self.imu_recording = {
            "kind": "balance", "started": time.monotonic(), "duration": self.imu_balance_seconds.value(), "samples": [],
        }
        self.imu_balance_record_button.setEnabled(False); self.imu_timer.start()
        self.imu_result.setPlainText("正在记录零速度平衡窗口…")

    def _advance_imu_recording(self):
        if self.imu_recording is None:
            self.imu_timer.stop()
            return
        elapsed = time.monotonic() - self.imu_recording["started"]
        duration = self.imu_recording["duration"]
        kind = self.imu_recording["kind"]
        if elapsed < duration:
            if kind == "static":
                self.imu_status.setText(f"正在记录静态样本：{elapsed:.1f}/{duration:.1f} s")
            return
        samples: List[dict] = self.imu_recording["samples"]
        self.imu_recording = None; self.imu_timer.stop()
        self.imu_record_button.setEnabled(True); self.imu_balance_record_button.setEnabled(True)
        if kind == "static":
            self.finish_static_imu_recording(samples)
        else:
            self.finish_balance_recording(samples)

    def finish_static_imu_recording(self, samples: Sequence[dict]):
        if len(samples) < 10:
            self.imu_status.setText("样本不足；请检查 Wi-Fi 遥测。")
            return
        accel_pitch = [sample["accel_pitch"] for sample in samples]
        pitch = [sample["pitch"] for sample in samples]
        gyro_x = [sample["gyro_x"] for sample in samples]
        gyro_y = [sample["gyro_y"] for sample in samples]
        gyro_z = [sample["gyro_z"] for sample in samples]
        proposed_offset = self.imu_current_offset.value() + self.imu_known_angle.value() - median(accel_pitch)
        self.imu_status.setText("静态 IMU 记录完成。")
        self.imu_result.setPlainText(
            f"静态记录 {len(samples)} 个样本\n"
            f"pitch 中位数={median(pitch):.4f}°；accelerometerPitch 中位数={median(accel_pitch):.4f}°\n"
            f"gyro 均值 X/Y/Z={sum(gyro_x)/len(gyro_x):.4f}/{sum(gyro_y)/len(gyro_y):.4f}/{sum(gyro_z)/len(gyro_z):.4f} °/s\n"
            f"gyro 标准差 X/Y/Z={population_stddev(gyro_x):.4f}/{population_stddev(gyro_y):.4f}/{population_stddev(gyro_z):.4f} °/s\n\n"
            f"已知机械角={self.imu_known_angle.value():.3f}°、当前 accelerometerAngleOffsetDegrees="
            f"{self.imu_current_offset.value():.3f}° 时，建议候选偏置={proposed_offset:.4f}°。\n"
            "将车头缓慢前/后倾，确认 pitch 与 pitchRate 均朝物理预期方向变化；若不一致，先检查 "
            "pitchAxis / pitchAngleInverted / pitchGyroInverted，而不是用 PID 增益掩盖。"
        )

    def finish_balance_recording(self, samples: Sequence[dict]):
        if len(samples) < 10:
            self.imu_result.setPlainText("平衡窗口样本不足。")
            return
        motors = [sample["balance_raw_motor"] for sample in samples]
        pitches = [sample["pitch"] for sample in samples]
        speeds = [sample["actual_speed"] for sample in samples]
        self.imu_result.setPlainText(
            f"零速度平衡记录 {len(samples)} 个样本\n"
            f"targetPitchDegrees={self.imu_target_pitch.value():.4f}°；实际 pitch 中位数={median(pitches):.4f}°；"
            f"实际速度中位数={median(speeds):.4f} m/s\n"
            f"内环原始输出均值={sum(motors)/len(motors):.5f}，标准差={population_stddev(motors):.5f}\n\n"
            "目标是零速度下不持续漂移，且内环平均输出接近 0。每次只小幅调整 targetPitchDegrees 后重测。"
            f"互补滤波 τ 当前记录为 {self.imu_filter_tau.value():.3f} s：先保持 0.25 s；"
            "仅当加速时俯仰误差明显时考虑加大，不要为“更快”盲目减小。"
        )

    # ---- Persistent records ------------------------------------------
    def start_session_log(self):
        try:
            self.records_directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            telemetry_path = self.records_directory / f"characterization_telemetry_{stamp}.csv"
            diagnostic_path = self.records_directory / f"characterization_ticks_{stamp}.csv"
            self.telemetry_file = telemetry_path.open("w", encoding="utf-8-sig", newline="")
            self.telemetry_writer = csv.writer(self.telemetry_file)
            self.telemetry_writer.writerow([
                "host_time", "board_timestamp_ms", "sequence", "state", "fault", "imu_valid", "pitch_deg",
                "pitch_rate_dps", "accelerometer_pitch_deg", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                "wheel_left_mps", "wheel_right_mps", "left_motor", "right_motor", "target_speed_mps",
                "actual_speed_mps", "target_diff_mps", "actual_diff_mps", "balance_raw_motor",
            ])
            self.diagnostic_file = diagnostic_path.open("w", encoding="utf-8-sig", newline="")
            self.diagnostic_writer = csv.writer(self.diagnostic_file)
            self.diagnostic_writer.writerow([
                "host_time", "board_timestamp_ms", "sequence", "left_ticks", "right_ticks", "left_tick_delta",
                "right_tick_delta", "left_ticks_per_second", "right_ticks_per_second",
            ])
            self.log(f"会话 CSV：{telemetry_path.name}；{diagnostic_path.name}")
        except OSError as error:
            self.telemetry_file = self.telemetry_writer = self.diagnostic_file = self.diagnostic_writer = None
            self.log(f"无法建立会话 CSV：{error}")

    def close_session_log(self):
        for file in (self.telemetry_file, self.diagnostic_file):
            if file is not None:
                try:
                    file.close()
                except OSError:
                    pass
        self.telemetry_file = self.telemetry_writer = self.diagnostic_file = self.diagnostic_writer = None

    def _write_telemetry(self, telemetry: dict, state: str, fault: str):
        if self.telemetry_writer is None or self.telemetry_file is None:
            return
        try:
            self.telemetry_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), telemetry["timestamp_ms"], telemetry["sequence"],
                state, fault, int(telemetry["imu_valid"]), telemetry["pitch"], telemetry["pitch_rate"],
                telemetry["accel_pitch"], telemetry["gyro_x"], telemetry["gyro_y"], telemetry["gyro_z"],
                telemetry["wheel_left"], telemetry["wheel_right"], telemetry["left_motor"], telemetry["right_motor"],
                telemetry["target_speed"], telemetry["actual_speed"], telemetry["target_diff"], telemetry["actual_diff"],
                telemetry["balance_raw_motor"],
            ])
            self.telemetry_file.flush()
        except OSError as error:
            self.log(f"遥测 CSV 写入失败：{error}")
            self.close_session_log()

    def _write_diagnostic(self, diagnostic: DiagnosticSample):
        if self.diagnostic_writer is None or self.diagnostic_file is None:
            return
        try:
            self.diagnostic_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), diagnostic.timestamp_ms, diagnostic.sequence,
                diagnostic.left_ticks, diagnostic.right_ticks, diagnostic.left_tick_delta, diagnostic.right_tick_delta,
                diagnostic.left_ticks_per_second, diagnostic.right_ticks_per_second,
            ])
            self.diagnostic_file.flush()
        except OSError as error:
            self.log(f"tick CSV 写入失败：{error}")
            self.close_session_log()

    def export_report(self):
        try:
            self.records_directory.mkdir(parents=True, exist_ok=True)
            path = self.records_directory / f"characterization_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            direction_checks = [
                ("leftDirectionInverted / 左电机正向", self.confirm_left_motor.isChecked()),
                ("rightDirectionInverted / 右电机正向", self.confirm_right_motor.isChecked()),
                ("左右编码器前进同号", self.confirm_encoder.isChecked()),
                ("正 Δv 右轮更快", self.confirm_diff.isChecked()),
                ("motorOutputInverted", self.confirm_balance_output.isChecked()),
                ("速度/转差 outputInverted", self.confirm_loop_outputs.isChecked()),
            ]
            lines = [
                "平衡小车机械、编码器和 IMU 调试报告", f"生成时间：{datetime.now().isoformat(timespec='seconds')}",
                "", "[方向冻结确认]",
            ]
            lines.extend(f"- {name}: {'已确认' if value else '未确认'}" for name, value in direction_checks)
            lines.extend(["", "[编码器比例]"])
            if self.encoder_trials:
                for trial in self.encoder_trials:
                    lines.append(
                        f"- {trial.direction} {trial.distance_m:.3f}m: Δtick L/R={trial.left_delta}/{trial.right_delta}; "
                        f"CPR 建议 L/R={trial.left_counts_per_revolution:.2f}/{trial.right_counts_per_revolution:.2f}"
                    )
            else:
                lines.append("- 未记录")
            lines.extend(["", "[PWM / 死区]"])
            if self.pwm_results:
                for result in self.pwm_results:
                    lines.append(
                        f"- {result.side} {result.direction} duty={result.duty:.3f}: "
                        f"vL/vR={result.left_speed_mps:.5f}/{result.right_speed_mps:.5f} m/s, n={result.sample_count}"
                    )
            else:
                lines.append("- 未记录")
            lines.extend(["", "[IMU / 姿态记录]", self.imu_result.toPlainText() or "- 未记录", "", "[原则]",
                          "方向错误必须先修改并冻结配置，不能由 PID 补偿；低速量化区不用于放大 PI；"
                          "PWM 20 kHz / 10 bit 仅检查驱动器允许和发热，不作为常规 PID 扫描参数。"])
            path.write_text("\n".join(lines), encoding="utf-8")
            self.log(f"完整报告已导出：{path}")
        except OSError as error:
            self.log(f"导出报告失败：{error}")

    def closeEvent(self, event):  # noqa: N802
        self.disconnect_board()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Balance Car Characterizer")
    window = CharacterizationWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
