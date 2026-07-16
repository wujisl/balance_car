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
主板发送：T,10,... 遥测、A,<seq>,OK|ERR,<reason> ACK、L,... 诊断行。
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
# Match the 40 ms speed/differential controller period while still bounding
# GUI work.  Faster incoming packets replace the mailbox value.
GUI_TELEMETRY_INTERVAL_MS = 40
PLOT_REFRESH_INTERVAL_SECONDS = 0.20
METRICS_REFRESH_INTERVAL_SECONDS = 0.50
CSV_FLUSH_INTERVAL_SECONDS = 1.0
CSV_FLUSH_ROW_LIMIT = 50
# Applied after the first valid telemetry frame on every new connection.  The
# balance derivative gain and all safety/output limits are intentionally not
# part of this group because they were not specified as baseline values.
CONNECTION_DEFAULT_PARAMETERS: Tuple[Tuple[str, str, float], ...] = (
    ("balance", "kp", 0.15),
    ("balance", "ki", 0.002),
    ("balance", "trim", -1.19),
    ("speed", "kp", 14.0),
    ("speed", "ki", 0.003),
    ("turn", "kp", 1.1),
    ("turn", "ki", 0.001),
)


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


@dataclass(frozen=True)
class VisibleDeviationStatistics:
    """当前图窗内，分析实际值相对目标值的带符号偏差统计。"""

    sample_count: int = 0
    maximum_positive: Optional[float] = None
    maximum_negative: Optional[float] = None
    average_positive: Optional[float] = None
    average_negative: Optional[float] = None


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


def calculate_visible_deviation_statistics(
    samples: Sequence[LoopSample], values: Sequence[float], view_seconds: float,
) -> VisibleDeviationStatistics:
    """统计当前曲线图窗中 ``分析实际值 - 目标值`` 的正、负偏差。"""
    if not samples:
        return VisibleDeviationStatistics()
    start_time = max(0.0, samples[-1].time_s - max(0.0, view_seconds))
    deviations = [
        (values[index] if index < len(values) else sample.measured) - sample.target
        for index, sample in enumerate(samples)
        if sample.time_s >= start_time
    ]
    positives = [value for value in deviations if value > 0.0]
    negatives = [value for value in deviations if value < 0.0]
    return VisibleDeviationStatistics(
        sample_count=len(deviations),
        maximum_positive=max(positives) if positives else None,
        maximum_negative=min(negatives) if negatives else None,
        average_positive=sum(positives) / len(positives) if positives else None,
        average_negative=sum(negatives) / len(negatives) if negatives else None,
    )


def format_visible_deviation_statistics(statistics: VisibleDeviationStatistics, unit: str) -> str:
    def signed(value: Optional[float]) -> str:
        return "—" if value is None else f"{value:+.5f} {unit}"

    return (
        f"当前图窗偏差（分析实际 − 目标，N={statistics.sample_count}）："
        f"最大正/负 {signed(statistics.maximum_positive)} / {signed(statistics.maximum_negative)}；"
        f"平均正/负 {signed(statistics.average_positive)} / {signed(statistics.average_negative)}"
    )


def calculate_metrics(
    samples: Sequence[LoopSample], method: str, alpha: float, median_window: int,
    steady_window_s: float, absolute_band: float, relative_band_percent: float,
    filtered_values: Optional[Sequence[float]] = None,
) -> LoopMetrics:
    """计算稳态值、稳态误差、超调和调节时间。

    稳态值使用尾部窗口中位数；容差会自动不小于 3σ(MAD)，避免噪声使
    "调节时间"长期无法收敛。指标用于辅助调参，非安全控制依据。
    """
    if len(samples) < 12:
        return LoopMetrics("采样不足（至少需要 12 个遥测样本）")

    values = (filtered_values if filtered_values is not None and len(filtered_values) == len(samples)
              else smooth_values(samples, method, alpha, median_window))
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
        # A queued Qt signal for every packet grows without a bound whenever
        # repainting is slower than reception.  The GUI polls this one-slot
        # mailbox instead, so stale telemetry is discarded deliberately.
        self._telemetry_lock = threading.Lock()
        self._latest_telemetry: Optional[dict] = None

    def _publish_latest_telemetry(self, telemetry: dict):
        with self._telemetry_lock:
            self._latest_telemetry = telemetry

    def take_latest_telemetry(self) -> Optional[dict]:
        with self._telemetry_lock:
            telemetry = self._latest_telemetry
            self._latest_telemetry = None
            return telemetry

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
                    self._publish_latest_telemetry(telemetry)
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
        """解析现有固件 T,5 ~ T,10；完整调参诊断由 T,10 提供。"""
        parts = text.split(",")
        if len(parts) < 2 or parts[0] != "T":
            return None
        expected_fields = {"5": 50, "6": 53, "7": 54, "8": 55, "9": 57, "10": 64}
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
                "turn_motor_requested": values[16],
                "turn_motor_applied": values[17],
                "left_motor": values[18],
                "right_motor": values[19],
                "balance_kp": values[20],
                "balance_ki": values[21],
                "balance_kd": values[22],
                "balance_trim": values[23],
                "speed_kp": values[24],
                "speed_ki": values[25],
                "max_motor": values[26],
                "max_pitch": values[27],
                "wheel_left": values[28],
                "wheel_right": values[29],
                "requested_pitch": values[30],
                "balance_error": values[31],
                "balance_p_term": values[32],
                "balance_i_term": values[33],
                "balance_d_term": values[34],
                "balance_motor_raw": values[35],
                "speed_inverted": bool(int(values[36])),
                "turn_kp": values[37],
                "turn_ki": values[38],
                "turn_max": values[39],
                "turn_inverted": bool(int(values[40])),
                "heading": values[41],
                "yaw_rate": values[42],
                # T,6 起开始回传视觉接管状态；T,10 追加闭环诊断。
                "vision_tracking": bool(int(values[43])) if version in ("6", "7", "8", "9", "10") else None,
                "vision_sample_fresh": bool(int(values[44])) if version in ("6", "7", "8", "9", "10") else None,
                "vision_command_accepted": bool(int(values[45])) if version in ("7", "8", "9", "10") else None,
                "vision_delta_speed": (values[46] if version in ("7", "8", "9", "10")
                                       else values[45] if version == "6" else None),
                "vision_period": values[47] if version in ("8", "9", "10") else None,
                "vision_filter": bool(int(values[48])) if version in ("9", "10") else None,
                "vision_max_step": values[49] if version in ("9", "10") else None,
                "balance_saturated": bool(int(values[50])) if version == "10" else None,
                "speed_saturated": bool(int(values[51])) if version == "10" else None,
                "turn_saturated": bool(int(values[52])) if version == "10" else None,
                "encoder_valid": bool(int(values[53])) if version == "10" else None,
                "imu_calibrated": bool(int(values[54])) if version == "10" else None,
                "balance_period_ms": values[55] if version == "10" else None,
                "velocity_period_ms": values[56] if version == "10" else None,
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
        # Samples are chronological.  Scanning from the tail makes repainting
        # cost depend on the visible time window, not on hours of history.
        start_index = len(self._samples) - 1
        while start_index > 0 and self._samples[start_index - 1].time_s >= start_t:
            start_index -= 1
        visible = list(enumerate(self._samples[start_index:], start_index))
        if not visible:
            return
        # Drawing more points than pixels does not add information, but can
        # monopolize the GUI thread after a long experiment.
        max_points = max(300, plot.width() * 2)
        if len(visible) > max_points:
            stride = math.ceil(len(visible) / max_points)
            decimated = visible[::stride]
            if decimated[-1][0] != visible[-1][0]:
                decimated.append(visible[-1])
            visible = decimated
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
        self._last_plot_refresh_time = 0.0
        self._plot_active = False
        self._last_parameters: Dict[str, float] = {}
        # 当前固件尚未完成所有积分环节的 dt/抗饱和前，不允许工具意外启用 Ki。
        self._integral_enabled = False
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
            ("diagnostics", "闭环诊断"),
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
        target_limit = 0.600 if self.is_speed else 0.200
        self.target_spin = self._spin(-target_limit, target_limit, 0.0, 3, 0.010)
        if self.is_speed:
            self.target_spin.setToolTip(
                "上位机输入范围为 ±0.600 m/s；实际可接受范围仍由当前固件的安全限幅决定。"
            )
        setting_grid.addWidget(self.target_spin, 0, 1)
        self.btn_set_target = QPushButton("设定目标")
        self.btn_set_target.clicked.connect(self.send_target)
        setting_grid.addWidget(self.btn_set_target, 0, 2)
        self.btn_zero = QPushButton("目标清零")
        self.btn_zero.clicked.connect(self.send_zero)
        setting_grid.addWidget(self.btn_zero, 0, 3)

        setting_grid.addWidget(QLabel("Kp:"), 1, 0)
        self.kp_spin = self._spin(0.0, 100.0 if self.is_speed else 10.0,
                                  14.0 if self.is_speed else 1.1, 5, 0.001)
        setting_grid.addWidget(self.kp_spin, 1, 1)
        setting_grid.addWidget(QLabel("Ki:"), 1, 2)
        self.ki_spin = self._spin(
            0.0, 100.0 if self.is_speed else 10.0,
            0.003 if self.is_speed else 0.001, 5, 0.001,
        )
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

    def reset_parameter_load(self):
        """下一帧完整遥测重新载入参数，避免换车连接时沿用旧显示值。"""
        self._last_parameters.clear()

    def set_integral_enabled(self, enabled: bool):
        self._integral_enabled = enabled

    def _filtered(self) -> List[float]:
        return smooth_values(self._samples, self.filter_combo.currentText(), self.alpha_spin.value(),
                             self.median_window_spin.value())

    def _metrics(self, filtered_values: Optional[Sequence[float]] = None) -> LoopMetrics:
        return calculate_metrics(
            self._samples,
            self.filter_combo.currentText(),
            self.alpha_spin.value(),
            self.median_window_spin.value(),
            self.steady_window_spin.value(),
            self.band_spin.value(),
            self.relative_band_spin.value(),
            filtered_values,
        )

    def clear_experiment(self, checked=False, initial=False):
        self._samples.clear()
        self._record_started_monotonic = time.monotonic()
        self._last_metrics_time = 0.0
        self._last_plot_refresh_time = 0.0
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
            if telemetry.get("speed_saturated") is None:
                self.status_values["diagnostics"].setText("等待 T,10 诊断遥测")
            else:
                period = telemetry.get("velocity_period_ms")
                self.status_values["diagnostics"].setText(
                    f"速度环饱和：{'是' if telemetry['speed_saturated'] else '否'}；"
                    f"周期：{period:.0f} ms；编码器：{'有效' if telemetry['encoder_valid'] else '无效'}"
                )
            parameters = {"kp": telemetry["speed_kp"], "ki": telemetry["speed_ki"], "limit": telemetry["max_pitch"]}
        else:
            self.status_values["output"].setText(
                f"请求/实际 {telemetry['turn_motor_requested']:.3f} / {telemetry['turn_motor_applied']:.3f}，限幅 {telemetry['turn_max']:.3f}"
            )
            if telemetry.get("turn_saturated") is None:
                self.status_values["diagnostics"].setText("等待 T,10 诊断遥测")
            else:
                period = telemetry.get("velocity_period_ms")
                self.status_values["diagnostics"].setText(
                    f"转差环饱和：{'是' if telemetry['turn_saturated'] else '否'}；"
                    f"周期：{period:.0f} ms；编码器：{'有效' if telemetry['encoder_valid'] else '无效'}"
                )
            parameters = {"kp": telemetry["turn_kp"], "ki": telemetry["turn_ki"], "limit": telemetry["turn_max"]}
        self._load_parameters_once(parameters)

        if self._record_started_monotonic is None:
            self._record_started_monotonic = time.monotonic()
        relative_time = time.monotonic() - self._record_started_monotonic
        self._samples.append(LoopSample(relative_time, target, actual, error))
        if len(self._samples) > 15000:  # 约 5 min @ 50 Hz；保留内存边界。
            del self._samples[:1000]
        if not self._plot_active:
            return
        if relative_time - self._last_metrics_time >= METRICS_REFRESH_INTERVAL_SECONDS:
            self._last_metrics_time = relative_time
            self._last_plot_refresh_time = relative_time
            self.refresh_plot(force_metrics=True)
        elif relative_time - self._last_plot_refresh_time >= PLOT_REFRESH_INTERVAL_SECONDS:
            self._last_plot_refresh_time = relative_time
            self.refresh_plot(force_metrics=False)

    def set_plot_active(self, active: bool):
        self._plot_active = active
        if active:
            self._last_plot_refresh_time = 0.0
            self._last_metrics_time = 0.0
            self.refresh_plot(force_metrics=True)

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
        if abs(self.ki_spin.value()) <= 1e-9 or self._integral_enabled:
            self.request_command.emit("parameter", domain, ("ki", self.ki_spin.value()))
        else:
            self.log_message.emit(
                f"{self.title}：未下发 Ki={self.ki_spin.value():.5f}；请先在“整车整定 / 视觉”页确认积分前置修正已烧录"
            )
        self.request_command.emit("parameter", domain, (limit_parameter, self.limit_spin.value()))

    def refresh_plot(self, force_metrics: bool = True):
        if not self._plot_active:
            return
        filtered = self._filtered()
        self.plot.set_data(self._samples, filtered, self.show_raw_check.isChecked(), self.view_seconds_spin.value())
        if force_metrics:
            metrics = self._metrics(filtered)
            statistics = calculate_visible_deviation_statistics(
                self._samples, filtered, self.view_seconds_spin.value()
            )
            self.metrics_label.setText(
                self._format_metrics(metrics) + "\n" + format_visible_deviation_statistics(statistics, "m/s")
            )

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


class BalanceTuningPage(QWidget):
    """平衡角内环整定。实际控制器支持 PD，保留 Ki 作为高级可选项。"""

    request_command = pyqtSignal(str, object, object)
    log_message = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._parameters_loaded = False
        self._integral_enabled = False
        self._angle_samples: List[LoopSample] = []
        self._balance_trace: List[Tuple[float, float, float, float, float, bool]] = []
        self._experiment_started_at: Optional[float] = None
        self._last_balance_sample_time: Optional[float] = None
        self._saturated_seconds = 0.0
        self._last_analysis_time = 0.0
        self._experiment_plot_active = False
        self._build_ui()
        self.clear_experiment(initial=True)

    @staticmethod
    def _spin(minimum: float, maximum: float, value: float, decimals: int, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setMinimumWidth(120)
        return spin

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(9)

        status_group = QGroupBox("平衡角环实时状态（主板 200 Hz 内环）")
        status_grid = QGridLayout(status_group)
        self.status_values: Dict[str, QLabel] = {}
        status_fields = (
            ("state", "安全状态"), ("pitch", "当前俯仰角 (°)"), ("rate", "俯仰角速度 (°/s)"),
            ("requested", "请求俯仰角 (°)"), ("error", "角度误差 (°)"), ("terms", "P / I / D 项"),
            ("raw", "角环原始输出"), ("motor", "混控后左右电机"), ("trim", "平衡点 Trim (°)"),
            ("diagnostics", "内环诊断"),
        )
        for index, (key, title) in enumerate(status_fields):
            row, column = divmod(index, 3)
            status_grid.addWidget(QLabel(title + ":"), row, column * 2)
            value = QLabel("-")
            value.setMinimumWidth(150)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.status_values[key] = value
            status_grid.addWidget(value, row, column * 2 + 1)
        root.addWidget(status_group)

        tuning_group = QGroupBox("平衡角环参数与电机输出限幅")
        tuning_grid = QGridLayout(tuning_group)
        tuning_grid.addWidget(QLabel("角度 Kp:"), 0, 0)
        self.kp_spin = self._spin(0.0, 100.0, 0.14, 5, 0.001)
        tuning_grid.addWidget(self.kp_spin, 0, 1)
        tuning_grid.addWidget(QLabel("角度 Kd:"), 0, 2)
        self.kd_spin = self._spin(0.0, 100.0, 0.02, 5, 0.001)
        tuning_grid.addWidget(self.kd_spin, 0, 3)
        tuning_grid.addWidget(QLabel("平衡点 Trim (°):"), 0, 4)
        self.trim_spin = self._spin(-20.0, 20.0, -1.19, 3, 0.05)
        self.trim_spin.setToolTip("机械平衡点；小步修改。它直接改变平衡目标角，不是传感器零偏。")
        tuning_grid.addWidget(self.trim_spin, 0, 5)

        tuning_grid.addWidget(QLabel("最大电机输出 (0–1):"), 1, 0)
        self.max_motor_spin = self._spin(0.0, 1.0, 0.45, 3, 0.01)
        self.max_motor_spin.setToolTip("归一化 PWM 占空比上限；这是安全的在线限幅，不改变 PWM 频率或位宽。")
        tuning_grid.addWidget(self.max_motor_spin, 1, 1)
        tuning_grid.addWidget(QLabel("角度 Ki（高级，默认 0）:"), 1, 2)
        self.ki_spin = self._spin(0.0, 100.0, 0.0, 5, 0.0005)
        self.ki_spin.setToolTip("平衡内环通常先使用 PD。仅在机械平衡点已正确、且确认存在稳定静差时才小步加入 Ki。")
        tuning_grid.addWidget(self.ki_spin, 1, 3)
        self.btn_apply_pd = QPushButton("应用 PD / Trim / 电机限幅")
        self.btn_apply_pd.clicked.connect(self.apply_pd)
        tuning_grid.addWidget(self.btn_apply_pd, 1, 4, 1, 2)
        self.btn_apply_all = QPushButton("应用含 Ki 的全部平衡参数")
        self.btn_apply_all.clicked.connect(self.apply_all)
        tuning_grid.addWidget(self.btn_apply_all, 1, 6)

        warning = QLabel(
            "建议顺序：先让车轮悬空，以较低最大电机输出确认方向；调 Trim，再调 Kp、Kd；"
            "最后才考虑 Ki。平衡状态下修改会立即生效，请每次只改一项并保留停止平衡通道。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#8a4b00; background:#fff7e5; padding:6px;")
        tuning_grid.addWidget(warning, 2, 0, 1, 7)
        root.addWidget(tuning_group)

        experiment_group = QGroupBox("平衡角环实验曲线与饱和统计")
        experiment_layout = QVBoxLayout(experiment_group)
        experiment_toolbar = QHBoxLayout()
        self.btn_clear_experiment = QPushButton("清空并从 0 s 开始记录")
        self.btn_clear_experiment.clicked.connect(self.clear_experiment)
        experiment_toolbar.addWidget(self.btn_clear_experiment)
        self.btn_export_experiment = QPushButton("导出平衡角环 CSV")
        self.btn_export_experiment.clicked.connect(self.export_experiment)
        experiment_toolbar.addWidget(self.btn_export_experiment)
        experiment_toolbar.addWidget(QLabel("分析采用中位数 + EMA，仅用于显示与指标，不改变主板控制。"))
        experiment_toolbar.addStretch(1)
        experiment_layout.addLayout(experiment_toolbar)
        self.pitch_plot = TimeSeriesPlot()
        self.pitch_plot.setMinimumHeight(280)
        experiment_layout.addWidget(self.pitch_plot)
        self.experiment_metrics_label = QLabel("尚未记录平衡角环实验")
        self.experiment_metrics_label.setWordWrap(True)
        self.experiment_metrics_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.experiment_metrics_label.setStyleSheet("font-family: Consolas, 'Microsoft YaHei'; padding: 5px; background: #f4f7fa;")
        experiment_layout.addWidget(self.experiment_metrics_label)
        root.addWidget(experiment_group)

        notes_group = QGroupBox("与平衡性能有关但不应在线随意修改的项目")
        notes_layout = QVBoxLayout(notes_group)
        notes = QLabel(
            "PWM 频率（当前 20 kHz）、PWM 位宽（10 bit）、电机方向、编码器每圈计数/轮径、"
            "IMU 轴向和互补滤波时间常数、控制周期及倾倒安全角均为固件启动或机械标定配置。"
            "它们改变后需要重新验证方向、自检和安全故障处理，因此不通过运行中的 Wi-Fi 调参开放。"
        )
        notes.setWordWrap(True)
        notes_layout.addWidget(notes)
        root.addWidget(notes_group)
        root.addStretch(1)

    def set_connected(self, connected: bool):
        self.btn_apply_pd.setEnabled(connected)
        self.btn_apply_all.setEnabled(connected)

    def reset_parameter_load(self):
        self._parameters_loaded = False

    def set_integral_enabled(self, enabled: bool):
        self._integral_enabled = enabled

    def consume_telemetry(self, telemetry: dict, state: str):
        values = self.status_values
        values["state"].setText(state)
        values["pitch"].setText(f"{telemetry['pitch']:.4f}")
        values["rate"].setText(f"{telemetry['pitch_rate']:.3f}")
        values["requested"].setText(f"{telemetry['requested_pitch']:.4f}")
        values["error"].setText(f"{telemetry['balance_error']:.4f}")
        values["terms"].setText(
            f"{telemetry['balance_p_term']:.4f} / {telemetry['balance_i_term']:.4f} / {telemetry['balance_d_term']:.4f}"
        )
        values["raw"].setText(f"{telemetry['balance_motor_raw']:.4f}")
        values["motor"].setText(f"{telemetry['left_motor']:.4f}, {telemetry['right_motor']:.4f}")
        values["trim"].setText(f"{telemetry['balance_trim']:.4f}")
        if telemetry.get("balance_saturated") is None:
            values["diagnostics"].setText("等待 T,10 诊断遥测")
        else:
            values["diagnostics"].setText(
                f"饱和：{'是' if telemetry['balance_saturated'] else '否'}；"
                f"周期：{telemetry['balance_period_ms']:.0f} ms；"
                f"IMU：{'已校准' if telemetry['imu_calibrated'] else '未校准'}"
            )
        if not self._parameters_loaded:
            self.kp_spin.setValue(telemetry["balance_kp"])
            self.ki_spin.setValue(telemetry["balance_ki"])
            self.kd_spin.setValue(telemetry["balance_kd"])
            self.trim_spin.setValue(telemetry["balance_trim"])
            self.max_motor_spin.setValue(telemetry["max_motor"])
            self._parameters_loaded = True
            self.log_message.emit("平衡角环：已从主板读取 PD、Trim、输出限幅与高级 Ki 参数")

        if self._experiment_started_at is None:
            self._experiment_started_at = time.monotonic()
        relative_time = time.monotonic() - self._experiment_started_at
        raw_motor = telemetry["balance_motor_raw"]
        applied_balance = (telemetry["left_motor"] + telemetry["right_motor"]) * 0.5
        saturated = telemetry.get("balance_saturated")
        if saturated is None:
            saturated = abs(raw_motor) > telemetry["max_motor"] + 1e-5
        if self._last_balance_sample_time is not None and saturated:
            self._saturated_seconds += max(0.0, relative_time - self._last_balance_sample_time)
        self._last_balance_sample_time = relative_time
        self._angle_samples.append(LoopSample(
            relative_time, telemetry["requested_pitch"], telemetry["pitch"], telemetry["balance_error"]
        ))
        self._balance_trace.append((
            relative_time, telemetry["pitch_rate"], raw_motor, applied_balance, telemetry["max_motor"], saturated
        ))
        if len(self._angle_samples) > 15000:
            del self._angle_samples[:1000]
            del self._balance_trace[:1000]
        if (self._experiment_plot_active
                and relative_time - self._last_analysis_time >= METRICS_REFRESH_INTERVAL_SECONDS):
            self._last_analysis_time = relative_time
            self.refresh_experiment_plot()

    def set_experiment_plot_active(self, active: bool):
        self._experiment_plot_active = active
        if active:
            self._last_analysis_time = 0.0
            self.refresh_experiment_plot()

    def confirm_parameter(self, parameter: str, value: float):
        mapping = {
            "kp": self.kp_spin,
            "ki": self.ki_spin,
            "kd": self.kd_spin,
            "trim": self.trim_spin,
            "max_motor": self.max_motor_spin,
        }
        spin = mapping.get(parameter)
        if spin is not None:
            spin.setValue(value)

    def _send(self, fields: Sequence[Tuple[str, float]]):
        for parameter, value in fields:
            self.request_command.emit("parameter", "balance", (parameter, value))

    def apply_pd(self):
        self._send((
            ("kp", self.kp_spin.value()),
            ("kd", self.kd_spin.value()),
            ("trim", self.trim_spin.value()),
            ("max_motor", self.max_motor_spin.value()),
        ))

    def apply_all(self):
        fields: List[Tuple[str, float]] = [
            ("kp", self.kp_spin.value()),
            ("kd", self.kd_spin.value()),
            ("trim", self.trim_spin.value()),
            ("max_motor", self.max_motor_spin.value()),
        ]
        if abs(self.ki_spin.value()) <= 1e-9 or self._integral_enabled:
            fields.insert(1, ("ki", self.ki_spin.value()))
        else:
            self.log_message.emit(
                f"平衡角环：未下发 Ki={self.ki_spin.value():.5f}；请先确认积分前置修正已烧录"
            )
        self._send(fields)

    def clear_experiment(self, checked=False, initial=False):
        self._angle_samples.clear()
        self._balance_trace.clear()
        self._experiment_started_at = time.monotonic()
        self._last_balance_sample_time = None
        self._saturated_seconds = 0.0
        self._last_analysis_time = 0.0
        if hasattr(self, "experiment_metrics_label"):
            self.experiment_metrics_label.setText("正在等待平衡角环遥测…")
        if not initial:
            self.log_message.emit("平衡角环：已清空实验曲线，下一包遥测从 0 s 开始记录")
        self.refresh_experiment_plot()

    def refresh_experiment_plot(self):
        if not self._experiment_plot_active:
            return
        filtered = smooth_values(self._angle_samples, "中位数 + EMA", 0.25, 5)
        self.pitch_plot.set_data(self._angle_samples, filtered, True, 20.0)
        deviation_statistics = calculate_visible_deviation_statistics(self._angle_samples, filtered, 20.0)
        deviation_text = format_visible_deviation_statistics(deviation_statistics, "°")
        metrics = calculate_metrics(
            self._angle_samples, "中位数 + EMA", 0.25, 5, 2.0, 0.25, 5.0, filtered
        )
        if metrics.target is None:
            self.experiment_metrics_label.setText(metrics.status + "\n" + deviation_text)
            return
        output_mismatch = 0.0
        if self._balance_trace:
            output_mismatch = max(abs(raw - applied) for _, _, raw, applied, _, _ in self._balance_trace)
        settling = "未收敛" if metrics.settling_time_s is None else f"{metrics.settling_time_s:.3f} s"
        overshoot = "不适用" if metrics.overshoot_percent is None else f"{metrics.overshoot_percent:.2f}%"
        latest_rate = self._balance_trace[-1][1] if self._balance_trace else 0.0
        self.experiment_metrics_label.setText(
            f"状态：{metrics.status}    目标/实际稳态俯仰：{metrics.target:.4f} / {metrics.steady_value:.4f} °    "
            f"稳态误差：{metrics.steady_error:.4f} °\n"
            f"超调：{overshoot}    调节时间：{settling}    噪声 σ：{metrics.noise_sigma:.4f} °    "
            f"当前角速度：{latest_rate:.3f} °/s\n"
            f"内环饱和累计：{self._saturated_seconds:.3f} s    原始/实际平衡输出最大差：{output_mismatch:.4f}\n"
            f"{deviation_text}"
        )

    def export_experiment(self):
        if not self._angle_samples:
            QMessageBox.information(self, "导出平衡角环曲线", "当前没有可导出的实验数据。")
            return
        default_name = f"balance_step_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "导出平衡角环曲线", default_name, "CSV 文件 (*.csv)")
        if not path:
            return
        filtered = smooth_values(self._angle_samples, "中位数 + EMA", 0.25, 5)
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow([
                    "time_s", "requested_pitch_deg", "pitch_deg", "pitch_error_deg", "analysis_filtered_pitch_deg",
                    "pitch_rate_dps", "balance_motor_raw", "applied_balance_estimate", "max_motor", "inner_saturated",
                ])
                for index, sample in enumerate(self._angle_samples):
                    _, rate, raw, applied, maximum, saturated = self._balance_trace[index]
                    writer.writerow([
                        f"{sample.time_s:.6f}", f"{sample.target:.6f}", f"{sample.measured:.6f}",
                        f"{sample.error:.6f}", f"{filtered[index]:.6f}", f"{rate:.6f}", f"{raw:.6f}",
                        f"{applied:.6f}", f"{maximum:.6f}", int(saturated),
                    ])
            self.log_message.emit(f"平衡角环：曲线已导出到 {path}")
        except OSError as error:
            QMessageBox.critical(self, "导出失败", str(error))


class VehicleTuningOverview(QWidget):
    """统一整定顺序与视觉循迹相关在线参数。"""

    request_command = pyqtSignal(str, object, object)
    log_message = pyqtSignal(str)
    integral_permission_changed = pyqtSignal(bool)
    experiment_marker = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._vision_loaded = False
        self._build_ui()

    @staticmethod
    def _spin(minimum: float, maximum: float, value: float, decimals: int, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setMinimumWidth(120)
        return spin

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(9)

        order_group = QGroupBox("整车统一整定顺序")
        order_layout = QVBoxLayout(order_group)
        order = QLabel(
            "1. 先完成机械、传感器方向和 PWM/编码器标定；2. 车轮悬空，低限幅整定平衡角环的 Trim → Kp → Kd；"
            "3. 地面低速整定速度环 Kp、Ki、最大俯仰偏置；4. 直线低速时整定转差环 Kp、Ki、最大转向输出；"
            "5. 最后接入摄像头循迹并调整视觉给定的更新/滤波。任何阶段出现振荡、异常方向或遥测失联，立即停止平衡。"
        )
        order.setWordWrap(True)
        order_layout.addWidget(order)
        self.integral_ready_check = QCheckBox(
            "已确认当前固件已烧录：启动目标速度为 0、所有积分按 dt 计算且具备抗积分饱和、静止陀螺零偏校准有效"
        )
        self.integral_ready_check.setToolTip(
            "未勾选时，上位机仍可整定 P/D 与限幅，但会阻止把非零 Ki 写入平衡、速度或转差环。"
        )
        self.integral_ready_check.toggled.connect(self.integral_permission_changed)
        order_layout.addWidget(self.integral_ready_check)
        self.firmware_diagnostics_label = QLabel("固件诊断：等待 T,10 遥测，以确认静止 IMU 校准、编码器和各环饱和状态。")
        self.firmware_diagnostics_label.setWordWrap(True)
        self.firmware_diagnostics_label.setStyleSheet("color:#4b5b6b; background:#f4f7fa; padding:6px;")
        order_layout.addWidget(self.firmware_diagnostics_label)
        tracking_note = QLabel(
            "摄像头循迹已开放：在主板处于 BALANCING 时，可直接用顶部按钮开启。"
            "进行单环阶跃实验时，建议操作者自行关闭循迹，以便区分人工给定与视觉给定。"
        )
        tracking_note.setWordWrap(True)
        tracking_note.setStyleSheet("color:#8a4b00; background:#fff7e5; padding:6px;")
        order_layout.addWidget(tracking_note)
        root.addWidget(order_group)

        experiment_group = QGroupBox("实验批次标记（写入命令/ACK/事件日志）")
        experiment_grid = QGridLayout(experiment_group)
        experiment_grid.addWidget(QLabel("重复次数/批次:"), 0, 0)
        self.trial_spin = QSpinBox()
        self.trial_spin.setRange(1, 99)
        self.trial_spin.setValue(1)
        experiment_grid.addWidget(self.trial_spin, 0, 1)
        experiment_grid.addWidget(QLabel("电池电压 (V):"), 0, 2)
        self.battery_spin = self._spin(0.0, 100.0, 0.0, 2, 0.1)
        self.battery_spin.setSpecialValueText("未记录")
        experiment_grid.addWidget(self.battery_spin, 0, 3)
        experiment_grid.addWidget(QLabel("地面:"), 0, 4)
        self.ground_edit = QLineEdit("未记录")
        experiment_grid.addWidget(self.ground_edit, 0, 5)
        experiment_grid.addWidget(QLabel("轮胎/负载/温度:"), 1, 0)
        self.vehicle_condition_edit = QLineEdit("未记录")
        experiment_grid.addWidget(self.vehicle_condition_edit, 1, 1, 1, 3)
        experiment_grid.addWidget(QLabel("本轮说明:"), 1, 4)
        self.experiment_note_edit = QLineEdit()
        self.experiment_note_edit.setPlaceholderText("例如：0→0.10 m/s 阶跃，Ki=0")
        experiment_grid.addWidget(self.experiment_note_edit, 1, 5)
        self.btn_mark_experiment = QPushButton("开始/标记本轮实验")
        self.btn_mark_experiment.clicked.connect(self.mark_experiment)
        experiment_grid.addWidget(self.btn_mark_experiment, 2, 0, 1, 6)
        root.addWidget(experiment_group)

        vision_group = QGroupBox("摄像头循迹给定整形（影响转向，不替代转差 PI）")
        vision_grid = QGridLayout(vision_group)
        vision_grid.addWidget(QLabel("目标转差更新间隔 (ms):"), 0, 0)
        self.vision_period_spin = self._spin(0.0, 60000.0, 40.0, 0, 10.0)
        self.vision_period_spin.setSpecialValueText("每个新包")
        self.vision_period_spin.setToolTip(
            "0 表示每个新 camera 包都更新；40 ms 与当前转差环周期一致。"
        )
        vision_grid.addWidget(self.vision_period_spin, 0, 1)
        vision_grid.addWidget(QLabel("单次转差最大变化 (mm/s，0=不限):"), 0, 2)
        self.vision_max_step_spin = self._spin(0.0, 200.0, 0.0, 0, 5.0)
        vision_grid.addWidget(self.vision_max_step_spin, 0, 3)
        self.vision_filter_check = QCheckBox("启用相机转差加权滑动滤波")
        self.vision_filter_check.setChecked(True)
        vision_grid.addWidget(self.vision_filter_check, 1, 0, 1, 2)
        self.vision_curve_hold_check = QCheckBox("弯道失线后保持确认方向的最大转差")
        self.vision_curve_hold_check.setChecked(True)
        vision_grid.addWidget(self.vision_curve_hold_check, 1, 2, 1, 2)
        vision_grid.addWidget(QLabel("失线保持转差 (mm/s):"), 2, 0)
        self.vision_curve_hold_spin = self._spin(20.0, 200.0, 120.0, 0, 10.0)
        vision_grid.addWidget(self.vision_curve_hold_spin, 2, 1)
        self.btn_apply_vision = QPushButton("应用视觉循迹参数")
        self.btn_apply_vision.clicked.connect(self.apply_vision)
        vision_grid.addWidget(self.btn_apply_vision, 2, 2, 1, 2)
        vision_note = QLabel(
            "当前 T,10 会回读更新间隔、加权滤波和最大变化；失线保持开关/幅值只会在本次下发 ACK 后确认。"
            "失线保持可能让车辆在黑线长时间丢失时继续转向，必须测试后再用于实际场地。"
        )
        vision_note.setWordWrap(True)
        vision_note.setStyleSheet("color:#8a4b00; background:#fff7e5; padding:6px;")
        vision_grid.addWidget(vision_note, 3, 0, 1, 4)
        root.addWidget(vision_group)

        hardware_group = QGroupBox("硬件与固件配置：需要重新编译烧录后验证")
        hardware_layout = QVBoxLayout(hardware_group)
        hardware = QLabel(
            "• 电机 PWM：20 kHz、10 bit；运行中改变频率/分辨率会重新初始化 PWM，可能造成电机瞬断。\n"
            "• 编码器：每圈计数、轮径、左右方向决定速度与转差的物理单位；修改后必须复核正负方向。\n"
            "• 姿态：IMU 轴向、互补滤波时间常数、加速度角偏置影响平衡角测量；不应在车辆运行时修改。\n"
            "• 安全：启动姿态限制、倾倒故障角及控制周期属于安全边界；保持固件配置并通过实物验证。\n"
            "这些参数位于 include/config/vehicle_config.h，故不放入在线下发接口。"
        )
        hardware.setWordWrap(True)
        hardware_layout.addWidget(hardware)
        root.addWidget(hardware_group)
        root.addStretch(1)

    def set_connected(self, connected: bool):
        self.btn_apply_vision.setEnabled(connected)
        self.btn_mark_experiment.setEnabled(connected)

    def reset_parameter_load(self):
        self._vision_loaded = False

    def consume_telemetry(self, telemetry: dict):
        if telemetry.get("balance_saturated") is None:
            self.firmware_diagnostics_label.setText(
                "固件诊断：当前为旧版遥测；请烧录支持 T,10 的固件后再进行完整三环验收。"
            )
        else:
            self.firmware_diagnostics_label.setText(
                "固件诊断："
                f"IMU {'已校准' if telemetry['imu_calibrated'] else '未校准'}；"
                f"编码器 {'有效' if telemetry['encoder_valid'] else '无效'}；"
                f"饱和（平衡/速度/转差）={'是' if telemetry['balance_saturated'] else '否'}/"
                f"{'是' if telemetry['speed_saturated'] else '否'}/"
                f"{'是' if telemetry['turn_saturated'] else '否'}；"
                f"周期（内环/外环）={telemetry['balance_period_ms']:.0f}/{telemetry['velocity_period_ms']:.0f} ms。"
            )
        if self._vision_loaded:
            return
        if telemetry.get("vision_period") is not None:
            self.vision_period_spin.setValue(telemetry["vision_period"])
        if telemetry.get("vision_max_step") is not None:
            self.vision_max_step_spin.setValue(telemetry["vision_max_step"])
        if telemetry.get("vision_filter") is not None:
            self.vision_filter_check.setChecked(bool(telemetry["vision_filter"]))
        self._vision_loaded = True
        self.log_message.emit("整车整定：已读取可回读的视觉循迹整形参数")

    def confirm_parameter(self, parameter: str, value: float):
        if parameter == "period_ms":
            self.vision_period_spin.setValue(value)
        elif parameter == "max_step_mmps":
            self.vision_max_step_spin.setValue(value)
        elif parameter == "filter":
            self.vision_filter_check.setChecked(value >= 0.5)
        elif parameter == "curve_hold":
            self.vision_curve_hold_check.setChecked(value >= 0.5)
        elif parameter == "curve_hold_mmps":
            self.vision_curve_hold_spin.setValue(value)

    def apply_vision(self):
        fields = (
            ("period_ms", self.vision_period_spin.value()),
            ("max_step_mmps", self.vision_max_step_spin.value()),
            ("filter", 1.0 if self.vision_filter_check.isChecked() else 0.0),
            ("curve_hold", 1.0 if self.vision_curve_hold_check.isChecked() else 0.0),
            ("curve_hold_mmps", self.vision_curve_hold_spin.value()),
        )
        for parameter, value in fields:
            self.request_command.emit("parameter", "vision", (parameter, value))

    def mark_experiment(self):
        metadata = {
            "trial": self.trial_spin.value(),
            "battery_v": self.battery_spin.value() if self.battery_spin.value() > 0.0 else "",
            "ground": self.ground_edit.text().strip(),
            "vehicle_condition": self.vehicle_condition_edit.text().strip(),
            "note": self.experiment_note_edit.text().strip(),
        }
        self.experiment_marker.emit(metadata)
        self.log_message.emit(f"已标记第 {metadata['trial']} 轮实验：{metadata['note'] or '无说明'}")


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
        self.event_file = None
        self.event_writer = None
        self.event_path: Optional[Path] = None
        self.latest_telemetry: Optional[dict] = None
        self.experiment_metadata: Dict[str, object] = {}
        self._connection_defaults_pending = False
        self._telemetry_rows_since_flush = 0
        self._telemetry_last_flush_monotonic = 0.0
        self._build_ui()
        self.age_timer = QTimer(self)
        self.age_timer.setInterval(100)
        self.age_timer.timeout.connect(self.update_packet_age)
        self.telemetry_render_timer = QTimer(self)
        self.telemetry_render_timer.setInterval(GUI_TELEMETRY_INTERVAL_MS)
        self.telemetry_render_timer.timeout.connect(self._drain_telemetry)

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
        self.vision_tracking = None
        self.btn_vision_tracking = QPushButton("开启摄像头循迹")
        self.btn_vision_tracking.setToolTip(
            "仅在 BALANCING 状态可用。启用后，主板以相机 I²C 给出的目标转差控制转向；"
            "手动设置目标转差会自动关闭循迹。按钮文字以主板 T,10 遥测回读为准。"
        )
        self.btn_vision_tracking.clicked.connect(self.toggle_vision_tracking)
        self.btn_vision_tracking.setEnabled(False)
        grid.addWidget(self.btn_vision_tracking, 2, 4, 1, 2)
        self.packet_label = QLabel("包：-")
        grid.addWidget(self.packet_label, 2, 6, 1, 2)
        layout.addWidget(connection)

        self.tabs = QTabWidget()
        self.balance_page = BalanceTuningPage()
        self.speed_page = LoopTuningPage("speed")
        self.turn_page = LoopTuningPage("turn")
        self.vehicle_page = VehicleTuningOverview()
        for page in (self.balance_page, self.speed_page, self.turn_page, self.vehicle_page):
            page.request_command.connect(self.handle_page_request)
            page.log_message.connect(self.log)
            page.set_connected(False)
        self.vehicle_page.integral_permission_changed.connect(self.set_integral_permission)
        self.vehicle_page.experiment_marker.connect(self.record_experiment_marker)
        self.set_integral_permission(False)
        for page, title in (
            (self.balance_page, "平衡角环整定"),
            (self.speed_page, "转速环调节"),
            (self.turn_page, "转差环调节"),
            (self.vehicle_page, "整车整定 / 视觉"),
        ):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, title)
        layout.addWidget(self.tabs, 1)
        self.tabs.currentChanged.connect(self._on_tuning_tab_changed)
        self._on_tuning_tab_changed(self.tabs.currentIndex())

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
        self.worker.ack_received.connect(self.on_ack)
        self.worker.console_line_received.connect(lambda line: self.log(f"[主板] {line}"))
        self.worker.command_failed.connect(self.on_command_failed)
        self.worker.info_updated.connect(self.log)
        self.worker.error_occurred.connect(self.on_error)
        self.command_sequence = int(time.time_ns() & 0x7FFFFFFF)
        self.pending.clear()
        self.last_sequence = None
        self.latest_telemetry = None
        self._connection_defaults_pending = True
        self.vehicle_page.integral_ready_check.setChecked(False)
        self.start_session_log()
        self.worker.start()
        self.age_timer.start()
        self.telemetry_render_timer.start()
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.btn_stop.setEnabled(True)
        for page in (self.balance_page, self.speed_page, self.turn_page, self.vehicle_page):
            page.reset_parameter_load()
            page.set_connected(True)
        for field in (self.ip_edit, self.local_port_spin, self.command_port_spin):
            field.setEnabled(False)
        self.link_label.setText("等待主板 T,10 遥测…")
        self.link_label.setStyleSheet("color:#b36b00;")
        self.log("开始订阅主板遥测；本次会话 CSV 已打开")

    def disconnect_board(self):
        self.age_timer.stop() if hasattr(self, "age_timer") else None
        self.telemetry_render_timer.stop() if hasattr(self, "telemetry_render_timer") else None
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1200)
            self.worker = None
        self.close_session_log()
        self.pending.clear()
        self.last_rx_monotonic = None
        self.last_sequence = None
        self.latest_telemetry = None
        self._connection_defaults_pending = False
        if hasattr(self, "btn_connect"):
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.btn_arm.setEnabled(False)
            self.btn_stop.setEnabled(False)
            self.vision_tracking = None
            self.btn_vision_tracking.setEnabled(False)
            self.btn_vision_tracking.setText("开启摄像头循迹")
            for page in (self.balance_page, self.speed_page, self.turn_page, self.vehicle_page):
                page.set_connected(False)
            for field in (self.ip_edit, self.local_port_spin, self.command_port_spin):
                field.setEnabled(True)
            self.link_label.setText("未连接")
            self.link_label.setStyleSheet("color:#666;")

    def _on_tuning_tab_changed(self, index: int):
        """Only redraw the expensive plot on its visible tab."""
        self.balance_page.set_experiment_plot_active(index == 0)
        self.speed_page.set_plot_active(index == 1)
        self.turn_page.set_plot_active(index == 2)

    def _drain_telemetry(self):
        """Consume one newest packet; old display packets are intentionally dropped."""
        worker = self.worker
        if worker is None:
            return
        telemetry = worker.take_latest_telemetry()
        if telemetry is not None:
            self.on_telemetry(telemetry)

    def _apply_connection_default_parameters(self):
        """Set the requested baseline group after telemetry confirms the board link."""
        if self.worker is None:
            return
        fields = {
            ("balance", "kp"): self.balance_page.kp_spin,
            ("balance", "ki"): self.balance_page.ki_spin,
            ("balance", "trim"): self.balance_page.trim_spin,
            ("speed", "kp"): self.speed_page.kp_spin,
            ("speed", "ki"): self.speed_page.ki_spin,
            ("turn", "kp"): self.turn_page.kp_spin,
            ("turn", "ki"): self.turn_page.ki_spin,
        }
        for domain, parameter, value in CONNECTION_DEFAULT_PARAMETERS:
            fields[(domain, parameter)].setValue(value)
            # This baseline explicitly includes all three Ki values.  It is
            # intentionally sent directly instead of relying on the manual
            # integral-permission checkbox.
            self.send_parameter(domain, parameter, value)
        self.log("已自动下发连接基准参数组（7 项），等待主板 ACK 回读确认")

    def _next_sequence(self) -> int:
        self.command_sequence = (self.command_sequence + 1) & 0x7FFFFFFF
        if self.command_sequence == 0:
            self.command_sequence = 1
        return self.command_sequence

    def set_integral_permission(self, enabled: bool):
        """积分项必须由操作者确认已具备固件侧 dt 与抗饱和保护后才可下发。"""
        for page in (self.balance_page, self.speed_page, self.turn_page):
            page.set_integral_enabled(enabled)
        if hasattr(self, "log_edit"):
            self.log("积分参数下发已" + ("解锁（前置修正已确认）" if enabled else "锁定（仅允许 Ki=0）"))

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
        self.write_event("TX", text.strip())

    def handle_page_request(self, request_kind: str, first: object, second: object):
        if request_kind == "control":
            self.send_control(str(first), float(second))
        elif request_kind == "parameter":
            domain = str(first)
            parameter, value = second  # type: ignore[misc]
            self.send_parameter(domain, str(parameter), float(value))

    def send_control(self, action: str, value: Optional[float] = None):
        action = action.upper()
        # 主板 TRACK 协议严格要求字符 `0` 或 `1`，不能发送 0.000 / 1.000。
        if action == "TRACK" and value is not None:
            suffix = ",1" if value >= 0.5 else ",0"
        else:
            suffix = "" if value is None else f",{value:.3f}"
        self.queue_command(
            f"C,{{sequence}},{action}{suffix}\n",
            {"kind": "control", "action": action, "value": value},
        )

    def toggle_vision_tracking(self):
        if self.worker is None:
            self.on_error("尚未连接主板")
            return
        if self.vision_tracking is None:
            self.on_error("当前固件未回传摄像头循迹状态；需要 T,6 或更新版本遥测")
            return
        requested = not self.vision_tracking
        self.send_control("TRACK", 1.0 if requested else 0.0)
        self.log(f"已请求{'开启' if requested else '关闭'}摄像头循迹，等待主板 ACK 与遥测回读")

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
        self.latest_telemetry = telemetry
        self.last_rx_monotonic = time.monotonic()
        if "rx_hz" in telemetry:
            self.last_rx_hz = telemetry["rx_hz"]
        state = STATE_NAMES[telemetry["state"]] if 0 <= telemetry["state"] < len(STATE_NAMES) else "UNKNOWN"
        self.balance_page.consume_telemetry(telemetry, state)
        self.speed_page.consume_telemetry(telemetry, state)
        self.turn_page.consume_telemetry(telemetry, state)
        self.vehicle_page.consume_telemetry(telemetry)
        if self._connection_defaults_pending:
            self._connection_defaults_pending = False
            self._apply_connection_default_parameters()
        self.write_session_telemetry(telemetry, state)
        self.link_label.setText("已收到主板遥测")
        self.link_label.setStyleSheet("color:#16803c;")
        self.btn_arm.setEnabled(self.worker is not None and state == "STANDBY" and telemetry["imu_valid"])
        tracking = telemetry.get("vision_tracking")
        if tracking is None:
            self.vision_tracking = None
            self.btn_vision_tracking.setEnabled(False)
            self.btn_vision_tracking.setText("摄像头循迹（需 T,6+）")
        else:
            self.vision_tracking = bool(tracking)
            self.btn_vision_tracking.setEnabled(self.worker is not None and state == "BALANCING")
            self.btn_vision_tracking.setText("关闭摄像头循迹" if self.vision_tracking else "开启摄像头循迹")
        self.update_packet_age(sequence)

    def on_ack(self, text: str):
        self.log(f"RX {text}")
        self.write_event("ACK", text)
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
        page = {
            "balance": self.balance_page,
            "speed": self.speed_page,
            "turn": self.turn_page,
            "vision": self.vehicle_page,
        }.get(domain)
        if page is not None:
            page.confirm_parameter(parameter, actual_value)
        self.log(f"参数已确认：{domain}.{parameter}={actual_value:.5f}")

    def on_command_failed(self, sequence: int, reason: str):
        pending = self.pending.pop(sequence, None)
        description = pending if pending is not None else "未知命令"
        self.write_event("COMMAND_FAILED", f"#{sequence},{reason},{description}")
        self.on_error(f"命令 #{sequence} 失败：{reason}；{description}")

    def record_experiment_marker(self, metadata: dict):
        self.experiment_metadata = metadata.copy()
        note = str(metadata.get("note", ""))
        self.write_event("EXPERIMENT_MARK", note)

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
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.telemetry_path = records / f"speed_turn_tuning_{timestamp}.csv"
            self.telemetry_file = self.telemetry_path.open("w", encoding="utf-8-sig", newline="")
            self.telemetry_writer = csv.writer(self.telemetry_file)
            self.telemetry_writer.writerow([
                "host_timestamp", "board_timestamp_ms", "sequence", "state", "fault", "imu_valid",
                "pitch_deg", "pitch_rate_dps", "requested_pitch_deg", "balance_error_deg",
                "balance_kp", "balance_ki", "balance_kd", "balance_trim_deg", "max_motor_command",
                "target_speed_mps", "actual_speed_mps", "speed_error_mps", "speed_kp", "speed_ki", "max_pitch_deg",
                "target_diff_mps", "actual_diff_mps", "diff_error_mps", "turn_kp", "turn_ki", "turn_max",
                "wheel_left_mps", "wheel_right_mps", "left_motor", "right_motor",
                "vision_tracking", "vision_period_ms", "vision_filter", "vision_max_step_mmps",
                "balance_inner_saturated", "speed_loop_saturated", "turn_loop_saturated",
                "encoder_valid", "imu_calibrated", "balance_period_ms", "velocity_period_ms",
            ])
            self.event_path = records / f"speed_turn_tuning_events_{timestamp}.csv"
            self.event_file = self.event_path.open("w", encoding="utf-8-sig", newline="")
            self.event_writer = csv.writer(self.event_file)
            self.event_writer.writerow([
                "host_timestamp", "event", "detail", "trial", "battery_v", "ground", "vehicle_condition", "note",
                "board_sequence", "state", "target_speed_mps", "actual_speed_mps", "target_diff_mps", "actual_diff_mps",
                "balance_kp", "balance_ki", "balance_kd", "speed_kp", "speed_ki", "turn_kp", "turn_ki",
            ])
            self._telemetry_rows_since_flush = 0
            self._telemetry_last_flush_monotonic = time.monotonic()
        except OSError as error:
            self.telemetry_file = self.telemetry_writer = self.telemetry_path = None
            self.event_file = self.event_writer = self.event_path = None
            self.on_error(f"无法创建会话日志：{error}")

    def write_session_telemetry(self, telemetry: dict, state: str):
        if self.telemetry_writer is None or self.telemetry_file is None:
            return
        try:
            fault = FAULT_NAMES[telemetry["fault"]] if 0 <= telemetry["fault"] < len(FAULT_NAMES) else "UNKNOWN"
            self.telemetry_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), telemetry["timestamp_ms"], telemetry["sequence"],
                state, fault, int(telemetry["imu_valid"]),
                telemetry["pitch"], telemetry["pitch_rate"], telemetry["requested_pitch"], telemetry["balance_error"],
                telemetry["balance_kp"], telemetry["balance_ki"], telemetry["balance_kd"], telemetry["balance_trim"],
                telemetry["max_motor"], telemetry["target_speed"], telemetry["actual_speed"], telemetry["speed_error"],
                telemetry["speed_kp"], telemetry["speed_ki"], telemetry["max_pitch"],
                telemetry["target_diff"], telemetry["actual_diff"], telemetry["diff_error"], telemetry["turn_kp"],
                telemetry["turn_ki"], telemetry["turn_max"], telemetry["wheel_left"], telemetry["wheel_right"],
                telemetry["left_motor"], telemetry["right_motor"], telemetry["vision_tracking"],
                telemetry["vision_period"], telemetry["vision_filter"], telemetry["vision_max_step"],
                telemetry.get("balance_saturated"), telemetry.get("speed_saturated"), telemetry.get("turn_saturated"),
                telemetry.get("encoder_valid"), telemetry.get("imu_calibrated"), telemetry.get("balance_period_ms"),
                telemetry.get("velocity_period_ms"),
            ])
            self._telemetry_rows_since_flush += 1
            self._flush_session_telemetry_if_due()
        except (OSError, KeyError) as error:
            self.on_error(f"写入会话日志失败：{error}")
            self.close_session_log()

    def _flush_session_telemetry_if_due(self, force: bool = False):
        if self.telemetry_file is None or self._telemetry_rows_since_flush == 0:
            return
        now = time.monotonic()
        if (not force and self._telemetry_rows_since_flush < CSV_FLUSH_ROW_LIMIT
                and now - self._telemetry_last_flush_monotonic < CSV_FLUSH_INTERVAL_SECONDS):
            return
        self.telemetry_file.flush()
        self._telemetry_rows_since_flush = 0
        self._telemetry_last_flush_monotonic = now

    def write_event(self, event: str, detail: str):
        if self.event_writer is None or self.event_file is None:
            return
        telemetry = self.latest_telemetry or {}
        metadata = self.experiment_metadata
        try:
            state_index = telemetry.get("state", -1)
            state = STATE_NAMES[state_index] if isinstance(state_index, int) and 0 <= state_index < len(STATE_NAMES) else "-"
            self.event_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), event, detail,
                metadata.get("trial", ""), metadata.get("battery_v", ""), metadata.get("ground", ""),
                metadata.get("vehicle_condition", ""), metadata.get("note", ""),
                telemetry.get("sequence", ""), state, telemetry.get("target_speed", ""), telemetry.get("actual_speed", ""),
                telemetry.get("target_diff", ""), telemetry.get("actual_diff", ""), telemetry.get("balance_kp", ""),
                telemetry.get("balance_ki", ""), telemetry.get("balance_kd", ""), telemetry.get("speed_kp", ""),
                telemetry.get("speed_ki", ""), telemetry.get("turn_kp", ""), telemetry.get("turn_ki", ""),
            ])
            self.event_file.flush()
        except OSError as error:
            self.on_error(f"写入实验事件日志失败：{error}")

    def close_session_log(self):
        if self.telemetry_file is not None:
            path = self.telemetry_path
            try:
                self._flush_session_telemetry_if_due(force=True)
                self.telemetry_file.close()
                self.log(f"会话日志已保存：{path}")
            except OSError as error:
                self.on_error(f"关闭会话日志失败：{error}")
        self.telemetry_file = self.telemetry_writer = self.telemetry_path = None
        self._telemetry_rows_since_flush = 0
        self._telemetry_last_flush_monotonic = 0.0
        if self.event_file is not None:
            path = self.event_path
            try:
                self.event_file.close()
                self.log(f"实验事件日志已保存：{path}")
            except OSError as error:
                self.on_error(f"关闭实验事件日志失败：{error}")
        self.event_file = self.event_writer = self.event_path = None

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
