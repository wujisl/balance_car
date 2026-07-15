#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESP32 平衡小车速度环 / 转差环 Wi-Fi 调参上位机。

本工具只使用主板已实现的 UDP 线协议：
  H\n                         订阅遥测
  C,<seq>,DRIVE,<m/s>\n        设定目标车速
  C,<seq>,TURN,<m/s>\n         设定目标右减左轮速差
  C,<seq>,ARM|STOP|RESET\n      安全控制
  P,<seq>,speed,kp|ki|max_pitch,<value>\n
  P,<seq>,turn,kp|ki|max,<value>\n
主板发送：T,9,... 遥测、A,<seq>,OK|ERR,<reason> ACK、L,... 诊断行。
不把上位机放入控制闭环；PID 的采样、滤波、抗积分饱和及安全停机仍在小车主板。
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
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


STATE_NAMES = ("BOOT", "SELF_TESTING", "STANDBY", "MANUAL_TEST", "BALANCING", "FAULT")
FAULT_NAMES = ("NONE", "SELF_TEST_FAILED", "IMU_UNHEALTHY", "PITCH_LIMIT_EXCEEDED")


@dataclass(frozen=True)
class LoopSample:
    """一项调参实验的单个遥测样本。"""

    time_s: float
    target: float
    measured: float
    error: float


@dataclass(frozen=True)
class LoopMetrics:
    """使用鲁棒平滑数据计算的阶跃响应指标。"""

    status: str
    target: Optional[float] = None
    steady_value: Optional[float] = None
    steady_error: Optional[float] = None
    overshoot_percent: Optional[float] = None
    settling_time_s: Optional[float] = None
    noise_sigma: Optional[float] = None
    tolerance: Optional[float] = None
    step_time_s: Optional[float] = None


def _median(values: Sequence[float]) -> float:
    return statistics.median(values) if values else 0.0


def _mad_sigma(values: Sequence[float], center: Optional[float] = None) -> float:
    """以 MAD 估计标准差，对离群值和轮速量化噪声更稳健。"""
    if len(values) < 2:
        return 0.0
    center = _median(values) if center is None else center
    return 1.4826 * _median([abs(value - center) for value in values])


def smooth_values(samples: Sequence[LoopSample], method: str, alpha: float, median_window: int) -> List[float]:
    """仅用于显示与分析的鲁棒平滑，不改变任何主板控制参数。"""
    raw = [sample.measured for sample in samples]
    if not raw or method == "原始数据":
        return raw

    if method == "中位数 + EMA":
        radius = max(1, int(median_window) // 2)
        staged = [
            _median(raw[max(0, index - radius): min(len(raw), index + radius + 1)])
            for index in range(len(raw))
        ]
    else:
        staged = raw

    alpha = min(1.0, max(0.01, alpha))
    filtered = [staged[0]]
    for value in staged[1:]:
        filtered.append(alpha * value + (1.0 - alpha) * filtered[-1])
    return filtered


def calculate_metrics(
    samples: Sequence[LoopSample], method: str, alpha: float, median_window: int,
    steady_window_s: float, absolute_band: float, relative_band_percent: float,
) -> LoopMetrics:
    """计算稳态值、稳态误差、超调和调节时间。

    稳态值使用尾部窗口中位数；容差会自动不小于 3σ(MAD)，避免噪声使
    "调节时间"长期无法收敛。指标用于辅助调参，非安全控制依据。
    """
    if len(samples) < 12:
        return LoopMetrics("采样不足（至少需要 12 个遥测样本）")

    values = smooth_values(samples, method, alpha, median_window)
    end_time = samples[-1].time_s
    tail_indices = [index for index, sample in enumerate(samples)
                    if sample.time_s >= end_time - max(0.2, steady_window_s)]
    if len(tail_indices) < 5:
        return LoopMetrics("等待稳态窗口积累")

    tail_values = [values[index] for index in tail_indices]
    tail_targets = [samples[index].target for index in tail_indices]
    steady_value = _median(tail_values)
    target = _median(tail_targets)
    noise_sigma = _mad_sigma(tail_values, steady_value)

    # 寻找最后一次目标改变的位置；有助于一次日志中连续做多个阶跃实验。
    target_epsilon = max(0.0005, abs(target) * 0.002)
    step_index = 0
    for index in range(len(samples) - 1, -1, -1):
        if abs(samples[index].target - target) > target_epsilon:
            step_index = min(index + 1, len(samples) - 1)
            break
    baseline_end = max(1, step_index)
    baseline_values = values[:baseline_end]
    if not baseline_values:
        baseline_values = values[:min(len(values), 10)]
    baseline = _median(baseline_values)
    step_delta = target - baseline
    relative_band = abs(step_delta) * max(0.0, relative_band_percent) / 100.0
    tolerance = max(max(0.0001, absolute_band), relative_band, 3.0 * noise_sigma)

    overshoot_percent: Optional[float]
    if abs(step_delta) <= max(absolute_band, 0.002):
        overshoot_percent = None
    elif step_delta > 0.0:
        overshoot_percent = max(0.0, (max(values[step_index:]) - target) / abs(step_delta) * 100.0)
    else:
        overshoot_percent = max(0.0, (target - min(values[step_index:])) / abs(step_delta) * 100.0)

    # 最后一个越出容差带的点之后，即为截至当前记录的调节时间。
    last_outside = -1
    for index in range(step_index, len(samples)):
        if abs(values[index] - target) > tolerance:
            last_outside = index
    if last_outside == len(samples) - 1:
        settling_time_s = None
        status = "仍在调节或尚未进入稳态带"
    else:
        settled_index = max(step_index, last_outside + 1)
        settling_time_s = samples[settled_index].time_s - samples[step_index].time_s
        status = "已进入稳态带"

    return LoopMetrics(
        status=status,
        target=target,
        steady_value=steady_value,
        steady_error=steady_value - target,
        overshoot_percent=overshoot_percent,
        settling_time_s=settling_time_s,
        noise_sigma=noise_sigma,
        tolerance=tolerance,
        step_time_s=samples[step_index].time_s,
    )


class BalanceTelemetryWorker(QThread):
    """与现有 WifiDebugServer 完全兼容的 UDP 接收、订阅及命令串行器。"""

    telemetry_ready = pyqtSignal(dict)
    ack_received = pyqtSignal(str)
    console_line_received = pyqtSignal(str)
    command_failed = pyqtSignal(int, str)
    info_updated = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, local_port: int, board_ip: str, command_port: int, parent=None):
        super().__init__(parent)
        self.local_port = local_port
        self.board_ip = board_ip
        self.command_port = command_port
        self.running = False
        self.sock: Optional[socket.socket] = None
        self._queue_lock = threading.Lock()
        self._queued_commands: Deque[Tuple[int, bytes]] = deque()
        self._inflight: Optional[Dict[str, object]] = None

    def queue_command(self, sequence: int, payload: bytes):
        if not payload:
            return
        with self._queue_lock:
            self._queued_commands.append((sequence, payload))

    def stop(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

    def _service_command_queue(self, sock: socket.socket, now: float):
        if self._inflight is not None:
            elapsed = now - float(self._inflight["sent_at"])
            if elapsed < 0.50:
                return
            if int(self._inflight["retries"]) >= 3:
                sequence = int(self._inflight["sequence"])
                self.command_failed.emit(sequence, "TIMEOUT")
                self.error_occurred.emit(f"主板命令超时：序号 {sequence}")
                self._inflight = None
                return
            try:
                sock.sendto(self._inflight["payload"], (self.board_ip, self.command_port))  # type: ignore[arg-type]
                self._inflight["retries"] = int(self._inflight["retries"]) + 1
                self._inflight["sent_at"] = now
            except OSError as error:
                self.error_occurred.emit(f"主板 UDP 命令重发失败：{error}")
            return

        with self._queue_lock:
            queued = self._queued_commands.popleft() if self._queued_commands else None
        if queued is None:
            return
        sequence, payload = queued
        try:
            sock.sendto(payload, (self.board_ip, self.command_port))
            self._inflight = {"sequence": sequence, "payload": payload, "retries": 0, "sent_at": now}
        except OSError as error:
            self.command_failed.emit(sequence, "SEND_FAILED")
            self.error_occurred.emit(f"主板 UDP 命令发送失败：{error}")

    def _accept_ack(self, text: str):
        if self._inflight is None:
            return
        fields = text.split(",", 3)
        try:
            sequence = int(fields[1])
        except (IndexError, ValueError):
            return
        if sequence == self._inflight["sequence"]:
            self._inflight = None

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.10)
            self.sock = sock
        except OSError as error:
            self.error_occurred.emit(f"主板 UDP 绑定失败：{error}")
            return

        self.running = True
        last_subscription = 0.0
        rate_started = time.monotonic()
        received_count = 0
        try:
            while self.running:
                now = time.monotonic()
                if now - last_subscription >= 1.0:
                    try:
                        sock.sendto(b"H\n", (self.board_ip, self.command_port))
                    except OSError as error:
                        self.error_occurred.emit(f"遥测订阅发送失败：{error}")
                    last_subscription = now

                self._service_command_queue(sock, now)
                try:
                    data, _ = sock.recvfrom(700)
                except socket.timeout:
                    continue
                except OSError:
                    break

                text = data.decode("ascii", errors="replace").strip()
                if text.startswith("T,"):
                    telemetry = self._parse_telemetry(text)
                    if telemetry is None:
                        continue
                    received_count += 1
                    now = time.monotonic()
                    if now - rate_started >= 1.0:
                        telemetry["rx_hz"] = received_count / (now - rate_started)
                        received_count = 0
                        rate_started = now
                    self.telemetry_ready.emit(telemetry)
                elif text.startswith("A,"):
                    self._accept_ack(text)
                    self.ack_received.emit(text)
                elif text.startswith("L,"):
                    parts = text.split(",", 2)
                    if len(parts) == 3:
                        self.console_line_received.emit(parts[2])
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self.sock = None
            self.info_updated.emit("主板调参 UDP 已停止")

    @staticmethod
    def _parse_telemetry(text: str) -> Optional[dict]:
        """解析现有固件 T,5 ~ T,9；本调参工具需要 T,9 的完整转差字段。"""
        parts = text.split(",")
        if len(parts) < 2 or parts[0] != "T":
            return None
        expected_fields = {"5": 50, "6": 53, "7": 54, "8": 55, "9": 57}
        version = parts[1]
        if version not in expected_fields or len(parts) != expected_fields[version]:
            return None
        try:
            values = [float(value) for value in parts[7:]]
            return {
                "version": int(version),
                "sequence": int(parts[2]),
                "timestamp_ms": int(parts[3]),
                "state": int(parts[4]),
                "fault": int(parts[5]),
                "imu_valid": bool(int(parts[6])),
                "pitch": values[0],
                "pitch_rate": values[1],
                "target_speed": values[9],
                "actual_speed": values[10],
                "speed_error": values[11],
                "target_diff": values[13],
                "actual_diff": values[14],
                "diff_error": values[15],
                "left_motor": values[18],
                "right_motor": values[19],
                "speed_kp": values[24],
                "speed_ki": values[25],
                "max_pitch": values[27],
                "wheel_left": values[28],
                "wheel_right": values[29],
                "turn_kp": values[37],
                "turn_ki": values[38],
                "turn_max": values[39],
                "heading": values[41],
                "yaw_rate": values[42],
            }
        except (IndexError, ValueError):
            return None


class TimeSeriesPlot(QFrame):
    """不引入额外绘图库的实时目标/实际曲线控件。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples: Sequence[LoopSample] = ()
        self._filtered: Sequence[float] = ()
        self._show_raw = True
        self._view_seconds = 20.0
        self.setMinimumHeight(310)
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)

    def set_data(self, samples: Sequence[LoopSample], filtered: Sequence[float], show_raw: bool, view_seconds: float):
        self._samples = samples
        self._filtered = filtered
        self._show_raw = show_raw
        self._view_seconds = max(2.0, view_seconds)
        self.update()

    def _draw_series(self, painter: QPainter, points: Iterable[Tuple[float, float]], pen: QPen):
        path = QPainterPath()
        first = True
        for x, y in points:
            if first:
                path.moveTo(x, y)
                first = False
            else:
                path.lineTo(x, y)
        if not first:
            painter.setPen(pen)
            painter.drawPath(path)

    def paintEvent(self, event):  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#101820"))
        if not self._samples:
            painter.setPen(QColor("#b8c7d1"))
            painter.drawText(self.rect(), Qt.AlignCenter, "点击“清空并从 0 s 开始记录”后显示本次实验曲线")
            return

        left, right, top, bottom = 62, 18, 22, 38
        plot = self.rect().adjusted(left, top, -right, -bottom)
        if plot.width() <= 1 or plot.height() <= 1:
            return

        end_t = self._samples[-1].time_s
        start_t = max(0.0, end_t - self._view_seconds)
        visible = [(index, sample) for index, sample in enumerate(self._samples) if sample.time_s >= start_t]
        if not visible:
            return
        values = [sample.target for _, sample in visible] + [sample.measured for _, sample in visible]
        values.extend(self._filtered[index] for index, _ in visible if index < len(self._filtered))
        low, high = min(values), max(values)
        span = max(0.02, high - low)
        low -= span * 0.16
        high += span * 0.16

        painter.setPen(QPen(QColor("#31414f"), 1))
        for fraction in range(6):
            y = plot.top() + plot.height() * fraction / 5.0
            painter.drawLine(plot.left(), int(y), plot.right(), int(y))
            value = high - (high - low) * fraction / 5.0
            painter.setPen(QColor("#a8b8c4"))
            painter.drawText(3, int(y) + 4, f"{value:.3f}")
            painter.setPen(QPen(QColor("#31414f"), 1))
        for fraction in range(6):
            x = plot.left() + plot.width() * fraction / 5.0
            painter.drawLine(int(x), plot.top(), int(x), plot.bottom())
            value = start_t + (end_t - start_t) * fraction / 5.0
            painter.setPen(QColor("#a8b8c4"))
            painter.drawText(int(x) - 12, plot.bottom() + 20, f"{value:.1f}")
            painter.setPen(QPen(QColor("#31414f"), 1))

        time_span = max(0.001, end_t - start_t)
        def point(sample: LoopSample, value: float) -> Tuple[float, float]:
            x = plot.left() + (sample.time_s - start_t) / time_span * plot.width()
            y = plot.bottom() - (value - low) / (high - low) * plot.height()
            return x, y

        target_pen = QPen(QColor("#4fc3f7"), 2, Qt.DashLine)
        self._draw_series(painter, (point(sample, sample.target) for _, sample in visible), target_pen)
        if self._show_raw:
            self._draw_series(painter, (point(sample, sample.measured) for _, sample in visible),
                              QPen(QColor("#8796a3"), 1))
        self._draw_series(
            painter,
            (point(sample, self._filtered[index]) for index, sample in visible if index < len(self._filtered)),
            QPen(QColor("#ffab40"), 2),
        )
        painter.setPen(QColor("#d9e4ec"))
        painter.drawText(plot.left(), 16, "目标（蓝虚线）  原始实际（灰）  分析曲线（橙）")
        painter.drawText(plot.right() - 48, plot.bottom() + 34, "时间 / s")


class LoopTuningPage(QWidget):
    """速度环或转差环页面：控制、参数、曲线、鲁棒分析和日志导出。"""

    request_command = pyqtSignal(str, object, object)  # domain/action, parameter/value, payload metadata
    log_message = pyqtSignal(str)

    def __init__(self, kind: str, parent=None):
        super().__init__(parent)
        if kind not in ("speed", "turn"):
            raise ValueError(kind)
        self.kind = kind
        self.is_speed = kind == "speed"
        self.title = "转速环调节" if self.is_speed else "转差环调节"
        self.target_name = "目标速度" if self.is_speed else "目标转差（右轮 − 左轮）"
        self.actual_name = "实际速度" if self.is_speed else "实际转差（右轮 − 左轮）"
        self._samples: List[LoopSample] = []
        self._record_started_monotonic: Optional[float] = None
        self._last_metrics_time = 0.0
        self._last_parameters: Dict[str, float] = {}
        self._build_ui()
        self.clear_experiment(initial=True)

    @staticmethod
    def _spin(minimum: float, maximum: float, value: float, decimals: int = 4, step: float = 0.01) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setMinimumWidth(110)
        return spin

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(9)

        status_group = QGroupBox("实时闭环状态")
        status_grid = QGridLayout(status_group)
        self.status_values: Dict[str, QLabel] = {}
        for index, (key, title) in enumerate((
            ("target", self.target_name + " (m/s)"),
            ("actual", self.actual_name + " (m/s)"),
            ("error", "误差（目标 − 实际）"),
            ("state", "安全状态"),
            ("output", "控制输出 / 限幅"),
            ("wheels", "左右轮速度 (m/s)"),
        )):
            row, column = divmod(index, 3)
            status_grid.addWidget(QLabel(title + ":"), row, column * 2)
            label = QLabel("-")
            label.setMinimumWidth(150)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.status_values[key] = label
            status_grid.addWidget(label, row, column * 2 + 1)
        root.addWidget(status_group)

        setting_group = QGroupBox("设定值与在线 PI 参数")
        setting_grid = QGridLayout(setting_group)
        setting_grid.addWidget(QLabel(self.target_name + " (m/s):"), 0, 0)
        target_limit = 0.250 if self.is_speed else 0.200
        self.target_spin = self._spin(-target_limit, target_limit, 0.0, 3, 0.010)
        setting_grid.addWidget(self.target_spin, 0, 1)
        self.btn_set_target = QPushButton("设定目标")
        self.btn_set_target.clicked.connect(self.send_target)
        setting_grid.addWidget(self.btn_set_target, 0, 2)
        self.btn_zero = QPushButton("目标清零")
        self.btn_zero.clicked.connect(self.send_zero)
        setting_grid.addWidget(self.btn_zero, 0, 3)

        setting_grid.addWidget(QLabel("Kp:"), 1, 0)
        self.kp_spin = self._spin(0.0, 100.0 if self.is_speed else 10.0,
                                  3.0 if self.is_speed else 1.0, 5, 0.001)
        setting_grid.addWidget(self.kp_spin, 1, 1)
        setting_grid.addWidget(QLabel("Ki:"), 1, 2)
        self.ki_spin = self._spin(0.0, 100.0 if self.is_speed else 10.0, 0.15 if self.is_speed else 0.0, 5, 0.001)
        setting_grid.addWidget(self.ki_spin, 1, 3)
        self.limit_name = "最大俯仰偏置 (°)" if self.is_speed else "最大转向输出 (0–1)"
        limit_max = 15.0 if self.is_speed else 1.0
        limit_default = 6.0 if self.is_speed else 0.20
        setting_grid.addWidget(QLabel(self.limit_name + ":"), 1, 4)
        self.limit_spin = self._spin(0.0, limit_max, limit_default, 3, 0.01)
        setting_grid.addWidget(self.limit_spin, 1, 5)
        self.btn_apply_pi = QPushButton("应用 PI 与限幅")
        self.btn_apply_pi.clicked.connect(self.apply_pi)
        setting_grid.addWidget(self.btn_apply_pi, 1, 6)
        info = QLabel("主板会在 ACK 中返回实际生效值；参数包串行发送，避免 UDP 突发丢失。")
        info.setWordWrap(True)
        setting_grid.addWidget(info, 2, 0, 1, 7)
        root.addWidget(setting_group)

        analysis_group = QGroupBox("阶跃曲线、稳态指标与高噪声分析")
        analysis_layout = QVBoxLayout(analysis_group)
        toolbar = QHBoxLayout()
        self.btn_clear = QPushButton("清空并从 0 s 开始记录")
        self.btn_clear.clicked.connect(self.clear_experiment)
        toolbar.addWidget(self.btn_clear)
        self.btn_export = QPushButton("导出当前曲线 CSV")
        self.btn_export.clicked.connect(self.export_curve)
        toolbar.addWidget(self.btn_export)
        toolbar.addWidget(QLabel("显示窗口 (s):"))
        self.view_seconds_spin = self._spin(2.0, 600.0, 20.0, 1, 2.0)
        self.view_seconds_spin.valueChanged.connect(lambda: self.refresh_plot())
        toolbar.addWidget(self.view_seconds_spin)
        self.show_raw_check = QCheckBox("显示原始噪声曲线")
        self.show_raw_check.setChecked(True)
        self.show_raw_check.toggled.connect(lambda: self.refresh_plot())
        toolbar.addWidget(self.show_raw_check)
        toolbar.addStretch(1)
        analysis_layout.addLayout(toolbar)

        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("分析滤波:"))
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["中位数 + EMA", "EMA", "原始数据"])
        self.filter_combo.currentTextChanged.connect(lambda _: self.refresh_plot())
        filter_row.addWidget(self.filter_combo)
        filter_row.addWidget(QLabel("EMA α:"))
        self.alpha_spin = self._spin(0.01, 1.0, 0.25, 2, 0.05)
        self.alpha_spin.valueChanged.connect(lambda: self.refresh_plot())
        filter_row.addWidget(self.alpha_spin)
        filter_row.addWidget(QLabel("中位数窗口:"))
        self.median_window_spin = QSpinBox()
        self.median_window_spin.setRange(3, 31)
        self.median_window_spin.setSingleStep(2)
        self.median_window_spin.setValue(5)
        self.median_window_spin.valueChanged.connect(lambda: self.refresh_plot())
        filter_row.addWidget(self.median_window_spin)
        filter_row.addWidget(QLabel("稳态窗口 (s):"))
        self.steady_window_spin = self._spin(0.5, 20.0, 2.0, 1, 0.5)
        self.steady_window_spin.valueChanged.connect(lambda: self.refresh_plot())
        filter_row.addWidget(self.steady_window_spin)
        filter_row.addWidget(QLabel("容差下限 (m/s):"))
        self.band_spin = self._spin(0.001, 0.100, 0.010, 3, 0.002)
        self.band_spin.valueChanged.connect(lambda: self.refresh_plot())
        filter_row.addWidget(self.band_spin)
        filter_row.addWidget(QLabel("相对容差 (%):"))
        self.relative_band_spin = self._spin(0.0, 30.0, 5.0, 1, 1.0)
        self.relative_band_spin.valueChanged.connect(lambda: self.refresh_plot())
        filter_row.addWidget(self.relative_band_spin)
        filter_row.addStretch(1)
        analysis_layout.addLayout(filter_row)

        self.plot = TimeSeriesPlot()
        analysis_layout.addWidget(self.plot)
        self.metrics_label = QLabel("尚未记录实验数据")
        self.metrics_label.setWordWrap(True)
        self.metrics_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.metrics_label.setStyleSheet("font-family: Consolas, 'Microsoft YaHei'; padding: 5px; background: #f4f7fa;")
        analysis_layout.addWidget(self.metrics_label)
        root.addWidget(analysis_group)

        advisor_group = QGroupBox("PI 优化助手（只生成建议，须人工确认后才下发）")
        advisor_layout = QVBoxLayout(advisor_group)
        advisor_toolbar = QHBoxLayout()
        advisor_toolbar.addWidget(QLabel("策略:"))
        self.advisor_combo = QComboBox()
        self.advisor_combo.addItems(["保守阶跃 PI", "高噪声稳健 PI", "两段 PI 设计说明"])
        advisor_toolbar.addWidget(self.advisor_combo)
        self.btn_advise = QPushButton("基于当前曲线生成建议")
        self.btn_advise.clicked.connect(self.generate_advice)
        advisor_toolbar.addWidget(self.btn_advise)
        self.btn_use_advice = QPushButton("将建议填入参数框")
        self.btn_use_advice.clicked.connect(self.use_advice)
        self.btn_use_advice.setEnabled(False)
        advisor_toolbar.addWidget(self.btn_use_advice)
        advisor_toolbar.addStretch(1)
        advisor_layout.addLayout(advisor_toolbar)
        self.advice_text = QTextEdit()
        self.advice_text.setReadOnly(True)
        self.advice_text.setMaximumHeight(122)
        advisor_layout.addWidget(self.advice_text)
        root.addWidget(advisor_group)
        root.addStretch(1)
        self._advised_values: Optional[Tuple[float, float]] = None

    def set_connected(self, connected: bool):
        for button in (self.btn_set_target, self.btn_zero, self.btn_apply_pi):
            button.setEnabled(connected)

    def _filtered(self) -> List[float]:
        return smooth_values(self._samples, self.filter_combo.currentText(), self.alpha_spin.value(),
                             self.median_window_spin.value())

    def _metrics(self) -> LoopMetrics:
        return calculate_metrics(
            self._samples,
            self.filter_combo.currentText(),
            self.alpha_spin.value(),
            self.median_window_spin.value(),
            self.steady_window_spin.value(),
            self.band_spin.value(),
            self.relative_band_spin.value(),
        )

    def clear_experiment(self, checked=False, initial=False):
        self._samples.clear()
        self._record_started_monotonic = time.monotonic()
        self._last_metrics_time = 0.0
        self._advised_values = None
        if hasattr(self, "btn_use_advice"):
            self.btn_use_advice.setEnabled(False)
        if hasattr(self, "advice_text") and not initial:
            self.advice_text.clear()
            self.log_message.emit(f"{self.title}：已清空曲线，下一包遥测从 0 s 开始记录")
        self.refresh_plot(force_metrics=True)

    def consume_telemetry(self, telemetry: dict, state_name: str):
        target_key = "target_speed" if self.is_speed else "target_diff"
        actual_key = "actual_speed" if self.is_speed else "actual_diff"
        error_key = "speed_error" if self.is_speed else "diff_error"
        target = telemetry[target_key]
        actual = telemetry[actual_key]
        error = telemetry[error_key]
        self.status_values["target"].setText(f"{target:.4f}")
        self.status_values["actual"].setText(f"{actual:.4f}")
        self.status_values["error"].setText(f"{error:.4f}")
        self.status_values["state"].setText(state_name)
        self.status_values["wheels"].setText(f"{telemetry['wheel_left']:.4f}, {telemetry['wheel_right']:.4f}")
        if self.is_speed:
            self.status_values["output"].setText(f"俯仰限幅 {telemetry['max_pitch']:.3f}°")
            parameters = {"kp": telemetry["speed_kp"], "ki": telemetry["speed_ki"], "limit": telemetry["max_pitch"]}
        else:
            self.status_values["output"].setText(f"转向限幅 {telemetry['turn_max']:.3f}")
            parameters = {"kp": telemetry["turn_kp"], "ki": telemetry["turn_ki"], "limit": telemetry["turn_max"]}
        self._load_parameters_once(parameters)

        if self._record_started_monotonic is None:
            self._record_started_monotonic = time.monotonic()
        relative_time = time.monotonic() - self._record_started_monotonic
        self._samples.append(LoopSample(relative_time, target, actual, error))
        if len(self._samples) > 15000:  # 约 5 min @ 50 Hz；保留内存边界。
            del self._samples[:1000]
        if relative_time - self._last_metrics_time >= 0.20:
            self._last_metrics_time = relative_time
            self.refresh_plot(force_metrics=True)
        else:
            self.refresh_plot(force_metrics=False)

    def _load_parameters_once(self, parameters: Dict[str, float]):
        if self._last_parameters:
            return
        self._last_parameters = parameters.copy()
        self.kp_spin.setValue(parameters["kp"])
        self.ki_spin.setValue(parameters["ki"])
        self.limit_spin.setValue(parameters["limit"])
        self.log_message.emit(f"{self.title}：已从主板读取当前 PI 参数")

    def confirm_parameter(self, parameter: str, value: float):
        mapping = {"kp": self.kp_spin, "ki": self.ki_spin,
                   "max_pitch" if self.is_speed else "max": self.limit_spin}
        spin = mapping.get(parameter)
        if spin is not None:
            spin.setValue(value)
        self._last_parameters[parameter] = value

    def send_target(self):
        action = "DRIVE" if self.is_speed else "TURN"
        self.request_command.emit("control", action, self.target_spin.value())

    def send_zero(self):
        self.target_spin.setValue(0.0)
        self.send_target()

    def apply_pi(self):
        domain = "speed" if self.is_speed else "turn"
        limit_parameter = "max_pitch" if self.is_speed else "max"
        self.request_command.emit("parameter", domain, ("kp", self.kp_spin.value()))
        self.request_command.emit("parameter", domain, ("ki", self.ki_spin.value()))
        self.request_command.emit("parameter", domain, (limit_parameter, self.limit_spin.value()))

    def refresh_plot(self, force_metrics: bool = True):
        filtered = self._filtered()
        self.plot.set_data(self._samples, filtered, self.show_raw_check.isChecked(), self.view_seconds_spin.value())
        if force_metrics:
            metrics = self._metrics()
            self.metrics_label.setText(self._format_metrics(metrics))

    @staticmethod
    def _format_metrics(metrics: LoopMetrics) -> str:
        if metrics.target is None:
            return metrics.status
        overshoot = "不适用（非有效阶跃）" if metrics.overshoot_percent is None else f"{metrics.overshoot_percent:.2f}%"
        settling = "未收敛" if metrics.settling_time_s is None else f"{metrics.settling_time_s:.3f} s"
        return (
            f"状态：{metrics.status}    阶跃起点：{metrics.step_time_s:.3f} s\n"
            f"目标稳态值：{metrics.target:.5f} m/s    实际稳态值（尾部中位数）：{metrics.steady_value:.5f} m/s    "
            f"稳态误差（实际−目标）：{metrics.steady_error:.5f} m/s\n"
            f"超调量：{overshoot}    调节时间：{settling}    "
            f"噪声 σ（MAD）：{metrics.noise_sigma:.5f} m/s    有效稳态带：±{metrics.tolerance:.5f} m/s"
        )

    def generate_advice(self):
        metrics = self._metrics()
        if metrics.target is None or metrics.steady_error is None:
            self.advice_text.setPlainText("需要至少 12 个样本并积累稳态窗口后，才能生成建议。")
            self.btn_use_advice.setEnabled(False)
            return
        kp, ki = self.kp_spin.value(), self.ki_spin.value()
        suggested_kp, suggested_ki = kp, ki
        strategy = self.advisor_combo.currentText()
        lines = [
            "建议只会填入参数框，不会自动下发。请在车轮悬空、旁路急停可用时逐项验证。",
            f"当前 Kp={kp:.5f}, Ki={ki:.5f}；噪声 σ≈{metrics.noise_sigma:.5f} m/s。",
        ]
        if strategy == "高噪声稳健 PI":
            suggested_kp = max(0.0, kp * (0.85 if (metrics.noise_sigma or 0.0) > self.band_spin.value() / 2 else 1.0))
            suggested_ki = max(0.0, ki * (0.75 if (metrics.noise_sigma or 0.0) > self.band_spin.value() / 2 else 0.90))
            lines.extend([
                "高噪声策略：用“中位数 + EMA”评估，避免针对单个量化尖峰增加增益。",
                "若速度环仍有静差，先每次只增加 Ki 5%~10%；若振荡或超调，先降低 Ki，再小步降低 Kp。",
                f"建议试验值：Kp={suggested_kp:.5f}, Ki={suggested_ki:.5f}。",
            ])
        elif strategy == "两段 PI 设计说明":
            coarse_kp = kp * 0.75
            coarse_ki = ki * 0.55
            fine_kp = kp
            fine_ki = ki * 0.85
            lines.extend([
                "当前主板协议只提供固定 Kp/Ki；上位机不能通过 Wi-Fi 在闭环周期内可靠地切换增益。",
                "两段 PI 应在主板控制器中实现：|误差|较大用较保守的粗调增益，进入阈值后切换细调增益，"
                "并保证积分状态连续与切换滞回。",
                f"可作为固件初始设计：粗调 Kp={coarse_kp:.5f}, Ki={coarse_ki:.5f}；细调 Kp={fine_kp:.5f}, Ki={fine_ki:.5f}。",
                "本工具保持现有协议，因此“填入参数框”只采用细调建议；不要把它当作已启用的分段 PI。",
            ])
            suggested_kp, suggested_ki = fine_kp, fine_ki
        else:
            if metrics.overshoot_percent is not None and metrics.overshoot_percent > 10.0:
                suggested_kp *= 0.88
                suggested_ki *= 0.75
                lines.append("超调大于 10%：先降低 Ki，再小幅降低 Kp，避免积分累积加剧振荡。")
            elif abs(metrics.steady_error) > max(self.band_spin.value(), (metrics.noise_sigma or 0.0) * 3.0):
                suggested_ki *= 1.08
                lines.append("稳态误差超过噪声带：小幅增加 Ki，并观察是否出现慢振荡。")
            elif metrics.settling_time_s is None:
                suggested_kp *= 1.05
                lines.append("尚未进入稳态带且超调不明显：可小幅增加 Kp；每次变化不超过 5%。")
            else:
                lines.append("当前响应已进入稳态带；优先保持参数，换工况复测而不是继续追求单次曲线。")
            lines.append(f"建议试验值：Kp={suggested_kp:.5f}, Ki={suggested_ki:.5f}。")
        self._advised_values = (suggested_kp, suggested_ki)
        self.btn_use_advice.setEnabled(True)
        self.advice_text.setPlainText("\n".join(lines))

    def use_advice(self):
        if self._advised_values is None:
            return
        self.kp_spin.setValue(self._advised_values[0])
        self.ki_spin.setValue(self._advised_values[1])
        self.log_message.emit(f"{self.title}：已将优化建议填入参数框，尚未下发")

    def export_curve(self):
        if not self._samples:
            QMessageBox.information(self, "导出曲线", "当前没有可导出的实验数据。")
            return
        default_name = f"{self.kind}_step_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "导出当前曲线", default_name, "CSV 文件 (*.csv)")
        if not path:
            return
        filtered = self._filtered()
        metrics = self._metrics()
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["loop", self.kind])
                writer.writerow(["analysis_filter", self.filter_combo.currentText()])
                writer.writerow(["steady_value", metrics.steady_value, "steady_error", metrics.steady_error,
                                 "overshoot_percent", metrics.overshoot_percent, "settling_time_s", metrics.settling_time_s,
                                 "noise_sigma", metrics.noise_sigma, "tolerance", metrics.tolerance])
                writer.writerow(["time_s", "target_mps", "actual_mps", "error_mps", "analysis_filtered_mps"])
                for index, sample in enumerate(self._samples):
                    writer.writerow([f"{sample.time_s:.6f}", f"{sample.target:.6f}", f"{sample.measured:.6f}",
                                     f"{sample.error:.6f}", f"{filtered[index]:.6f}"])
            self.log_message.emit(f"{self.title}：曲线已导出到 {path}")
        except OSError as error:
            QMessageBox.critical(self, "导出失败", str(error))


class TuningMainWindow(QMainWindow):
    """独立上位机主窗口；不会导入相机标定模块。"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("平衡小车速度环 / 转差环 Wi-Fi 调参上位机")
        self.resize(1450, 980)
        self.worker: Optional[BalanceTelemetryWorker] = None
        self.command_sequence = int(time.time_ns() & 0x7FFFFFFF)
        self.pending: Dict[int, Dict[str, object]] = {}
        self.last_rx_monotonic: Optional[float] = None
        self.last_rx_hz = 0.0
        self.last_sequence: Optional[int] = None
        self.telemetry_file = None
        self.telemetry_writer = None
        self.telemetry_path: Optional[Path] = None
        self._build_ui()
        self.age_timer = QTimer(self)
        self.age_timer.setInterval(100)
        self.age_timer.timeout.connect(self.update_packet_age)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)

        connection = QGroupBox("主板 Wi-Fi / UDP 连接（与当前固件一致）")
        grid = QGridLayout(connection)
        grid.addWidget(QLabel("主板 IP:"), 0, 0)
        self.ip_edit = QLineEdit("192.168.4.1")
        self.ip_edit.setFixedWidth(150)
        grid.addWidget(self.ip_edit, 0, 1)
        grid.addWidget(QLabel("本地遥测端口:"), 0, 2)
        self.local_port_spin = QSpinBox()
        self.local_port_spin.setRange(1024, 65535)
        self.local_port_spin.setValue(9000)
        grid.addWidget(self.local_port_spin, 0, 3)
        grid.addWidget(QLabel("主板命令端口:"), 0, 4)
        self.command_port_spin = QSpinBox()
        self.command_port_spin.setRange(1024, 65535)
        self.command_port_spin.setValue(9001)
        grid.addWidget(self.command_port_spin, 0, 5)
        self.btn_connect = QPushButton("连接并订阅")
        self.btn_connect.clicked.connect(self.connect_board)
        grid.addWidget(self.btn_connect, 0, 6)
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.clicked.connect(self.disconnect_board)
        self.btn_disconnect.setEnabled(False)
        grid.addWidget(self.btn_disconnect, 0, 7)
        self.link_label = QLabel("未连接")
        self.link_label.setStyleSheet("color: #666;")
        grid.addWidget(self.link_label, 1, 0, 1, 8)
        self.btn_arm = QPushButton("启动平衡")
        self.btn_arm.clicked.connect(self.arm_board)
        self.btn_arm.setEnabled(False)
        grid.addWidget(self.btn_arm, 2, 0, 1, 2)
        self.btn_stop = QPushButton("停止平衡（急停）")
        self.btn_stop.setStyleSheet("background:#c62828; color:white; font-weight:bold;")
        self.btn_stop.clicked.connect(lambda: self.send_control("STOP"))
        self.btn_stop.setEnabled(False)
        grid.addWidget(self.btn_stop, 2, 2, 1, 2)
        self.packet_label = QLabel("包：-")
        grid.addWidget(self.packet_label, 2, 4, 1, 4)
        layout.addWidget(connection)

        self.tabs = QTabWidget()
        self.speed_page = LoopTuningPage("speed")
        self.turn_page = LoopTuningPage("turn")
        for page in (self.speed_page, self.turn_page):
            page.request_command.connect(self.handle_page_request)
            page.log_message.connect(self.log)
            page.set_connected(False)
        for page, title in ((self.speed_page, "转速环调节"), (self.turn_page, "转差环调节")):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, title)
        layout.addWidget(self.tabs, 1)

        log_group = QGroupBox("通信与调参日志")
        log_layout = QVBoxLayout(log_group)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumHeight(150)
        self.log_edit.document().setMaximumBlockCount(1000)
        log_layout.addWidget(self.log_edit)
        layout.addWidget(log_group)
        self.statusBar().showMessage("就绪：连接主板热点后点击“连接并订阅”")

    def connect_board(self):
        self.disconnect_board()
        self.worker = BalanceTelemetryWorker(self.local_port_spin.value(), self.ip_edit.text().strip(),
                                             self.command_port_spin.value())
        self.worker.telemetry_ready.connect(self.on_telemetry)
        self.worker.ack_received.connect(self.on_ack)
        self.worker.console_line_received.connect(lambda line: self.log(f"[主板] {line}"))
        self.worker.command_failed.connect(self.on_command_failed)
        self.worker.info_updated.connect(self.log)
        self.worker.error_occurred.connect(self.on_error)
        self.command_sequence = int(time.time_ns() & 0x7FFFFFFF)
        self.pending.clear()
        self.start_session_log()
        self.worker.start()
        self.age_timer.start()
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.btn_stop.setEnabled(True)
        for page in (self.speed_page, self.turn_page):
            page.set_connected(True)
        for field in (self.ip_edit, self.local_port_spin, self.command_port_spin):
            field.setEnabled(False)
        self.link_label.setText("等待主板 T,9 遥测…")
        self.link_label.setStyleSheet("color:#b36b00;")
        self.log("开始订阅主板遥测；本次会话 CSV 已打开")

    def disconnect_board(self):
        self.age_timer.stop() if hasattr(self, "age_timer") else None
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1200)
            self.worker = None
        self.close_session_log()
        self.pending.clear()
        self.last_rx_monotonic = None
        if hasattr(self, "btn_connect"):
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.btn_arm.setEnabled(False)
            self.btn_stop.setEnabled(False)
            for page in (self.speed_page, self.turn_page):
                page.set_connected(False)
            for field in (self.ip_edit, self.local_port_spin, self.command_port_spin):
                field.setEnabled(True)
            self.link_label.setText("未连接")
            self.link_label.setStyleSheet("color:#666;")

    def _next_sequence(self) -> int:
        self.command_sequence = (self.command_sequence + 1) & 0x7FFFFFFF
        if self.command_sequence == 0:
            self.command_sequence = 1
        return self.command_sequence

    def queue_command(self, payload: str, pending: Optional[Dict[str, object]] = None):
        if self.worker is None:
            self.on_error("尚未连接主板")
            return
        sequence = self._next_sequence()
        text = payload.format(sequence=sequence)
        metadata = pending or {"kind": "control", "description": text.strip()}
        self.pending[sequence] = metadata
        self.worker.queue_command(sequence, text.encode("ascii"))
        self.log(f"TX {text.strip()}")

    def handle_page_request(self, request_kind: str, first: object, second: object):
        if request_kind == "control":
            self.send_control(str(first), float(second))
        elif request_kind == "parameter":
            domain = str(first)
            parameter, value = second  # type: ignore[misc]
            self.send_parameter(domain, str(parameter), float(value))

    def send_control(self, action: str, value: Optional[float] = None):
        suffix = "" if value is None else f",{value:.3f}"
        self.queue_command(
            f"C,{{sequence}},{action}{suffix}\n",
            {"kind": "control", "action": action, "value": value},
        )

    def send_parameter(self, domain: str, parameter: str, value: float):
        self.queue_command(
            f"P,{{sequence}},{domain},{parameter},{value:.7f}\n",
            {"kind": "parameter", "domain": domain, "parameter": parameter, "value": value},
        )

    def arm_board(self):
        answer = QMessageBox.question(
            self, "确认启动平衡",
            "确认车轮状态安全、车辆接近直立，且可以随时按“停止平衡”？\n主板仍会执行自检、IMU 与姿态安全校验。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.send_control("ARM")

    def on_telemetry(self, telemetry: dict):
        sequence = telemetry["sequence"]
        if self.last_sequence is not None and sequence <= self.last_sequence:
            return
        self.last_sequence = sequence
        self.last_rx_monotonic = time.monotonic()
        if "rx_hz" in telemetry:
            self.last_rx_hz = telemetry["rx_hz"]
        state = STATE_NAMES[telemetry["state"]] if 0 <= telemetry["state"] < len(STATE_NAMES) else "UNKNOWN"
        self.speed_page.consume_telemetry(telemetry, state)
        self.turn_page.consume_telemetry(telemetry, state)
        self.write_session_telemetry(telemetry, state)
        self.link_label.setText("已收到主板遥测")
        self.link_label.setStyleSheet("color:#16803c;")
        self.btn_arm.setEnabled(self.worker is not None and state == "STANDBY" and telemetry["imu_valid"])
        self.update_packet_age(sequence)

    def on_ack(self, text: str):
        self.log(f"RX {text}")
        fields = text.split(",", 3)
        try:
            sequence = int(fields[1])
        except (IndexError, ValueError):
            return
        pending = self.pending.pop(sequence, None)
        if pending is None:
            return
        if len(fields) < 4 or fields[2] != "OK":
            self.on_error(f"主板拒绝命令 #{sequence}：{fields[3] if len(fields) >= 4 else text}")
            return
        if pending.get("kind") != "parameter":
            return
        result = fields[3].split(",")
        if len(result) != 4 or result[0] != "APPLIED":
            self.on_error(f"参数 ACK 格式异常：{fields[3]}")
            return
        try:
            actual_value = float(result[3])
        except ValueError:
            self.on_error(f"参数 ACK 数值异常：{fields[3]}")
            return
        domain, parameter = result[1], result[2]
        page = self.speed_page if domain == "speed" else self.turn_page if domain == "turn" else None
        if page is not None:
            page.confirm_parameter(parameter, actual_value)
        self.log(f"参数已确认：{domain}.{parameter}={actual_value:.5f}")

    def on_command_failed(self, sequence: int, reason: str):
        pending = self.pending.pop(sequence, None)
        description = pending if pending is not None else "未知命令"
        self.on_error(f"命令 #{sequence} 失败：{reason}；{description}")

    def update_packet_age(self, sequence: Optional[int] = None):
        if self.last_rx_monotonic is None:
            return
        age_ms = (time.monotonic() - self.last_rx_monotonic) * 1000.0
        shown_sequence = self.last_sequence if sequence is None else sequence
        self.packet_label.setText(f"包：{shown_sequence} / {age_ms:.0f} ms / {self.last_rx_hz:.1f} Hz")
        if age_ms > 1000.0:
            self.link_label.setText("遥测超时：检查 Wi-Fi、IP 与端口")
            self.link_label.setStyleSheet("color:#c62828;")

    def start_session_log(self):
        records = Path(__file__).resolve().parent / "records"
        try:
            records.mkdir(parents=True, exist_ok=True)
            self.telemetry_path = records / f"speed_turn_tuning_{datetime.now():%Y%m%d_%H%M%S}.csv"
            self.telemetry_file = self.telemetry_path.open("w", encoding="utf-8-sig", newline="")
            self.telemetry_writer = csv.writer(self.telemetry_file)
            self.telemetry_writer.writerow([
                "host_timestamp", "board_timestamp_ms", "sequence", "state", "fault", "imu_valid",
                "target_speed_mps", "actual_speed_mps", "speed_error_mps", "speed_kp", "speed_ki", "max_pitch_deg",
                "target_diff_mps", "actual_diff_mps", "diff_error_mps", "turn_kp", "turn_ki", "turn_max",
                "wheel_left_mps", "wheel_right_mps", "pitch_deg", "pitch_rate_dps", "left_motor", "right_motor",
            ])
        except OSError as error:
            self.telemetry_file = self.telemetry_writer = self.telemetry_path = None
            self.on_error(f"无法创建会话日志：{error}")

    def write_session_telemetry(self, telemetry: dict, state: str):
        if self.telemetry_writer is None or self.telemetry_file is None:
            return
        try:
            fault = FAULT_NAMES[telemetry["fault"]] if 0 <= telemetry["fault"] < len(FAULT_NAMES) else "UNKNOWN"
            self.telemetry_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), telemetry["timestamp_ms"], telemetry["sequence"],
                state, fault, int(telemetry["imu_valid"]), telemetry["target_speed"], telemetry["actual_speed"],
                telemetry["speed_error"], telemetry["speed_kp"], telemetry["speed_ki"], telemetry["max_pitch"],
                telemetry["target_diff"], telemetry["actual_diff"], telemetry["diff_error"], telemetry["turn_kp"],
                telemetry["turn_ki"], telemetry["turn_max"], telemetry["wheel_left"], telemetry["wheel_right"],
                telemetry["pitch"], telemetry["pitch_rate"], telemetry["left_motor"], telemetry["right_motor"],
            ])
            self.telemetry_file.flush()
        except (OSError, KeyError) as error:
            self.on_error(f"写入会话日志失败：{error}")
            self.close_session_log()

    def close_session_log(self):
        if self.telemetry_file is not None:
            path = self.telemetry_path
            try:
                self.telemetry_file.close()
                self.log(f"会话日志已保存：{path}")
            except OSError as error:
                self.on_error(f"关闭会话日志失败：{error}")
        self.telemetry_file = self.telemetry_writer = self.telemetry_path = None

    def log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_edit.append(f"[{timestamp}] {message}")
        self.statusBar().showMessage(message, 4000)

    def on_error(self, message: str):
        self.log(f"[错误] {message}")

    def closeEvent(self, event):  # noqa: N802 - Qt API
        self.disconnect_board()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Balance Car Speed/Turn Tuner")
    window = TuningMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
