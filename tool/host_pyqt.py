#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 + GC2145 平衡小车上位机（PyQt5）

功能：
- 通过 HTTP 或 UDP 连接 ESP32 Wi-Fi AP
- 实时显示 MJPEG/UDP 图像流
- 显示 FPS、帧大小、分辨率、UDP 包统计
- 保存快照
- 将 C++ 算法参数（流模式、灰度阈值、ROI）下发给 ESP32，由 ESP32 端执行算法
- 显示 ESP32 C++ 算法处理后的效果（二值化掩膜 + 赛道中线）


依赖：
    PyQt5, opencv-python, numpy, requests
"""

import csv
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from urllib.parse import urljoin

import cv2
import numpy as np
import requests
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox, QTabWidget,
    QTextEdit, QGroupBox, QGridLayout, QFileDialog, QMessageBox,
    QSplitter, QComboBox, QCheckBox
)


# ---------------------------------------------------------------------------
# UDP 协议常量（与 ESP32 端保持一致）
# ---------------------------------------------------------------------------
UDP_MAGIC = 0x55445043          # "UDPC"
UDP_HEADER_FMT = "<IIHHHH"      # magic, frame_id, pkt_id, pkt_cnt, payload_len, reserved
UDP_HEADER_LEN = struct.calcsize(UDP_HEADER_FMT)


def encode_udp_command(quality: int = None, interval_ms: int = None, stream_divider: int = None,
                        mode: int = None, threshold: int = None,
                        roi: tuple = None, lookahead_y: int = None, line_width: tuple = None,
                        contrast100: int = None, otsu: bool = None,
                        otsu_range: tuple = None, otsu_max_step: int = None,
                        otsu_alpha: int = None, foreground_range: tuple = None,
                        edge_threshold: int = None, smooth_filter: bool = None,
                        morph_clean: bool = None, smooth_alpha: int = None,
                        max_row_gap: int = None, max_hold_frames: int = None) -> bytes:
    """生成发送给 ESP32 的 ASCII 命令。"""
    if quality is not None:
        return f"Q={quality}\n".encode("ascii")
    if interval_ms is not None:
        return f"I={interval_ms}\n".encode("ascii")
    if stream_divider is not None:
        return f"N={stream_divider}\n".encode("ascii")
    if mode is not None:
        return f"M={mode}\n".encode("ascii")
    if threshold is not None:
        return f"T={threshold}\n".encode("ascii")
    if roi is not None:
        return f"R={roi[0]},{roi[1]}\n".encode("ascii")
    if lookahead_y is not None:
        return f"Y={lookahead_y}\n".encode("ascii")
    if line_width is not None:
        return f"W={line_width[0]},{line_width[1]}\n".encode("ascii")
    if contrast100 is not None:
        return f"G={contrast100}\n".encode("ascii")
    if otsu is not None:
        return f"O={1 if otsu else 0}\n".encode("ascii")
    if otsu_range is not None:
        return f"L={otsu_range[0]},{otsu_range[1]}\n".encode("ascii")
    if otsu_max_step is not None:
        return f"J={otsu_max_step}\n".encode("ascii")
    if otsu_alpha is not None:
        return f"A={otsu_alpha}\n".encode("ascii")
    if foreground_range is not None:
        return f"P={foreground_range[0]},{foreground_range[1]}\n".encode("ascii")
    if edge_threshold is not None:
        return f"E={edge_threshold}\n".encode("ascii")
    if smooth_filter is not None:
        return f"F={1 if smooth_filter else 0}\n".encode("ascii")
    if morph_clean is not None:
        return f"C={1 if morph_clean else 0}\n".encode("ascii")
    if smooth_alpha is not None:
        return f"S={smooth_alpha}\n".encode("ascii")
    if max_row_gap is not None:
        return f"D={max_row_gap}\n".encode("ascii")
    if max_hold_frames is not None:
        return f"H={max_hold_frames}\n".encode("ascii")
    return b""


# ---------------------------------------------------------------------------
# 后台工作线程：HTTP MJPEG 流
# ---------------------------------------------------------------------------
class StreamWorker(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    fps_updated = pyqtSignal(float)
    info_updated = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, base_url: str, parent=None):
        super().__init__(parent)
        self.base_url = base_url
        self.stream_url = urljoin(base_url + "/", "stream")
        self.running = False

    def stop(self):
        self.running = False
        self.wait(1000)

    def run(self):
        self.running = True
        self.info_updated.emit(f"Connecting to {self.stream_url} ...")

        try:
            resp = requests.get(self.stream_url, stream=True, timeout=5.0)
            if resp.status_code != 200:
                self.error_occurred.emit(f"HTTP {resp.status_code}")
                return
        except Exception as e:
            self.error_occurred.emit(f"Connect failed: {e}")
            return

        self.info_updated.emit("Stream connected")

        buffer = b""
        frame_count = 0
        last_time = time.time()

        for chunk in resp.iter_content(chunk_size=4096):
            if not self.running:
                break

            buffer += chunk

            while self.running:
                soi = buffer.find(b"\xff\xd8")
                eoi = buffer.find(b"\xff\xd9")

                if soi != -1 and eoi != -1 and eoi > soi:
                    jpeg_data = buffer[soi:eoi + 2]
                    buffer = buffer[eoi + 2:]

                    img = self._decode_jpeg(jpeg_data)
                    if img is not None:
                        self.frame_ready.emit(img)
                        frame_count += 1

                        now = time.time()
                        if now - last_time >= 1.0:
                            fps = frame_count / (now - last_time)
                            self.fps_updated.emit(fps)
                            frame_count = 0
                            last_time = now
                else:
                    break

        resp.close()
        self.info_updated.emit("Stream stopped")

    @staticmethod
    def _decode_jpeg(data: bytes):
        if not data:
            return None
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img


# ---------------------------------------------------------------------------
# 后台工作线程：HTTP 单帧轮询
# ---------------------------------------------------------------------------
class SingleFrameWorker(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    fps_updated = pyqtSignal(float)
    info_updated = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, base_url: str, interval_ms: int = 100, parent=None):
        super().__init__(parent)
        self.base_url = base_url
        self.frame_url = urljoin(base_url + "/", "frame")
        self.interval_ms = interval_ms
        self.running = False

    def stop(self):
        self.running = False
        self.wait(1000)

    def run(self):
        self.running = True
        frame_count = 0
        last_time = time.time()

        while self.running:
            t0 = time.time()
            try:
                resp = requests.get(self.frame_url, timeout=2.0)
                if resp.status_code == 200:
                    img = self._decode_jpeg(resp.content)
                    if img is not None:
                        self.frame_ready.emit(img)
                        frame_count += 1

                        now = time.time()
                        if now - last_time >= 1.0:
                            fps = frame_count / (now - last_time)
                            self.fps_updated.emit(fps)
                            frame_count = 0
                            last_time = now

                        self.info_updated.emit(
                            f"Frame: {len(resp.content)} bytes, {img.shape[1]}x{img.shape[0]}"
                        )
                    else:
                        self.error_occurred.emit("Decode JPEG failed")
                else:
                    self.error_occurred.emit(f"HTTP {resp.status_code}")
            except Exception as e:
                self.error_occurred.emit(f"Request failed: {e}")

            elapsed = (time.time() - t0) * 1000
            sleep_ms = max(10, self.interval_ms - elapsed)
            time.sleep(sleep_ms / 1000.0)

        self.info_updated.emit("Single-frame worker stopped")

    @staticmethod
    def _decode_jpeg(data: bytes):
        if not data:
            return None
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img


# ---------------------------------------------------------------------------
# 后台工作线程：UDP 裸二进制 JPEG 流
# ---------------------------------------------------------------------------
class UdpStreamWorker(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    latest_frame_available = pyqtSignal()
    fps_updated = pyqtSignal(float)
    info_updated = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    stats_updated = pyqtSignal(str)  # 包统计信息

    def __init__(self, local_port: int = 8888, esp_ip: str = "192.168.4.1",
                 cmd_port: int = 8889, parent=None):
        super().__init__(parent)
        self.local_port = local_port
        self.esp_ip = esp_ip
        self.cmd_port = cmd_port
        self.running = False
        self.cmd_socket = None
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_notification_pending = False
        self._init_cmd_socket()

    def _publish_latest_frame(self, img: np.ndarray):
        """只保留最新解码帧，避免 GUI 事件队列积压造成显示延迟。"""
        notify = False
        with self._frame_lock:
            self._latest_frame = img
            if not self._frame_notification_pending:
                self._frame_notification_pending = True
                notify = True
        if notify:
            self.latest_frame_available.emit()

    def take_latest_frame(self):
        with self._frame_lock:
            img = self._latest_frame
            self._latest_frame = None
            self._frame_notification_pending = False
            return img

    def _init_cmd_socket(self):
        try:
            self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except Exception as e:
            self.error_occurred.emit(f"UDP cmd socket failed: {e}")

    def send_command(self, cmd_bytes: bytes):
        if self.cmd_socket is None or not cmd_bytes:
            return
        try:
            self.cmd_socket.sendto(cmd_bytes, (self.esp_ip, self.cmd_port))
        except Exception as e:
            self.error_occurred.emit(f"Send cmd failed: {e}")

    def stop(self):
        self.running = False
        self.wait(1000)
        if self.cmd_socket is not None:
            try:
                self.cmd_socket.close()
            except Exception:
                pass
            self.cmd_socket = None

    def run(self):
        self.running = True
        self._init_cmd_socket()

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.2)
        except Exception as e:
            self.error_occurred.emit(f"UDP bind failed: {e}")
            return

        self.info_updated.emit(f"Listening UDP on 0.0.0.0:{self.local_port}")

        current_fid = None
        expected_cnt = None
        packets = {}
        first_seen = 0.0
        decoded_frames = 0
        dropped_frames = 0
        last_stats_time = time.time()
        last_frame_info_time = 0.0

        while self.running:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                # 检查当前帧是否超时未收齐
                if current_fid is not None and len(packets) < expected_cnt:
                    if time.time() - first_seen > 0.15:
                        dropped_frames += 1
                        current_fid = None
                        expected_cnt = None
                        packets.clear()
                continue
            except OSError:
                break

            if len(data) < UDP_HEADER_LEN:
                continue

            magic, fid, pid, pcnt, plen, _ = struct.unpack(
                UDP_HEADER_FMT, data[:UDP_HEADER_LEN]
            )
            if magic != UDP_MAGIC:
                continue
            if pcnt == 0:
                continue

            payload = data[UDP_HEADER_LEN:UDP_HEADER_LEN + plen]
            if len(payload) != plen:
                continue

            # 新帧号到达，丢弃旧帧
            if fid != current_fid:
                if current_fid is not None and len(packets) < expected_cnt:
                    dropped_frames += 1
                current_fid = fid
                expected_cnt = pcnt
                packets = {pid: payload}
                first_seen = time.time()
            else:
                packets[pid] = payload

            # 收齐一包
            if len(packets) == expected_cnt:
                jpeg = b"".join(packets[i] for i in range(expected_cnt))
                current_fid = None
                expected_cnt = None
                packets.clear()

                img = self._decode_jpeg(jpeg)
                if img is not None:
                    decoded_frames += 1
                    self._publish_latest_frame(img)
                    now = time.time()
                    if now - last_frame_info_time >= 1.0:
                        self.info_updated.emit(
                            f"UDP latest: {len(jpeg)} bytes, {img.shape[1]}x{img.shape[0]}, "
                            f"packets {pcnt}"
                        )
                        last_frame_info_time = now

            # 每秒刷新一次统计
            now = time.time()
            if now - last_stats_time >= 1.0:
                total = decoded_frames + dropped_frames
                loss = (dropped_frames / total * 100.0) if total > 0 else 0.0
                self.stats_updated.emit(
                    f"decoded={decoded_frames} dropped={dropped_frames} loss={loss:.1f}%"
                )
                # 不重置计数，保持累计
                last_stats_time = now

        try:
            sock.close()
        except Exception:
            pass
        self.info_updated.emit("UDP worker stopped")

    @staticmethod
    def _decode_jpeg(data: bytes):
        if not data:
            return None
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img


# ---------------------------------------------------------------------------
# 主板 Wi-Fi 遥测：独立端口与线程，不影响相机 UDP 图像流。
# ---------------------------------------------------------------------------
class BalanceTelemetryWorker(QThread):
    telemetry_ready = pyqtSignal(dict)
    ack_received = pyqtSignal(str)
    console_line_received = pyqtSignal(str)
    command_failed = pyqtSignal(int, str)
    info_updated = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, local_port: int, board_ip: str, command_port: int):
        super().__init__()
        self.local_port = local_port
        self.board_ip = board_ip
        self.command_port = command_port
        self.running = False
        self.sock = None
        self._command_lock = threading.Lock()
        self._pending_commands = []
        self._inflight_command = None

    def queue_command(self, command: bytes):
        if not command:
            return
        with self._command_lock:
            self._pending_commands.append(command)

    def stop(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

    @staticmethod
    def _command_sequence(command: bytes):
        try:
            fields = command.decode("ascii").strip().split(",")
            return int(fields[1]) if len(fields) >= 2 else None
        except (UnicodeDecodeError, ValueError):
            return None

    def _service_command_queue(self, sock, now: float):
        """串行下发命令并等待 ACK，避免 UDP 突发包被主板丢弃。"""
        if self._inflight_command is not None:
            elapsed = now - self._inflight_command["sent_at"]
            if elapsed < 0.50:
                return
            if self._inflight_command["retries"] >= 3:
                sequence = self._inflight_command["sequence"]
                self.error_occurred.emit(f"主板命令超时：序号 {sequence}")
                self.command_failed.emit(sequence if sequence is not None else -1, "TIMEOUT")
                self._inflight_command = None
                return
            try:
                sock.sendto(self._inflight_command["payload"], (self.board_ip, self.command_port))
                self._inflight_command["retries"] += 1
                self._inflight_command["sent_at"] = now
            except OSError as e:
                self.error_occurred.emit(f"主板 UDP 命令重发失败：{e}")
            return

        with self._command_lock:
            command = self._pending_commands.pop(0) if self._pending_commands else None
        if command is None:
            return
        try:
            sock.sendto(command, (self.board_ip, self.command_port))
            self._inflight_command = {
                "sequence": self._command_sequence(command),
                "payload": command,
                "retries": 0,
                "sent_at": now,
            }
        except OSError as e:
            self.error_occurred.emit(f"主板 UDP 命令发送失败：{e}")

    def _accept_ack(self, text: str):
        if self._inflight_command is None:
            return
        fields = text.split(",", 3)
        if len(fields) < 3:
            return
        try:
            sequence = int(fields[1])
        except ValueError:
            return
        if sequence == self._inflight_command["sequence"]:
            self._inflight_command = None

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
        except OSError as e:
            self.error_occurred.emit(f"主板 UDP 绑定失败：{e}")
            return

        self.running = True
        try:
            # Register this bound UDP port before waiting for telemetry.
            # Unicast avoids broadcast packets being filtered by some Wi-Fi drivers.
            sock.sendto(b"H\n", (self.board_ip, self.command_port))
        except OSError as e:
            self.error_occurred.emit(f"主板 UDP 订阅发送失败：{e}")
        self.info_updated.emit(
            f"主板调试已监听 UDP {self.local_port}，命令发送至 {self.board_ip}:{self.command_port}"
        )
        received_count = 0
        rate_started = time.monotonic()
        last_subscription_time = 0.0
        has_received_telemetry = False
        try:
            while self.running:
                now = time.monotonic()
                if now - last_subscription_time >= 1.0:
                    try:
                        sock.sendto(b"H\n", (self.board_ip, self.command_port))
                    except OSError as e:
                        self.error_occurred.emit(f"主板 UDP 订阅发送失败：{e}")
                    last_subscription_time = now
                self._service_command_queue(sock, now)

                try:
                    data, _ = sock.recvfrom(600)
                except socket.timeout:
                    continue
                except OSError:
                    break

                text = data.decode("ascii", errors="replace").strip()
                if text.startswith("T,"):
                    telemetry = self._parse_telemetry(text)
                    if telemetry is None:
                        continue
                    if not has_received_telemetry:
                        has_received_telemetry = True
                        self.info_updated.emit(
                            f"已收到首个主板遥测包，序号 {telemetry['sequence']}"
                        )
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
            self.info_updated.emit("主板调试 UDP 已停止")

    @staticmethod
    def _parse_telemetry(text: str):
        parts = text.split(",")
        if parts[0:2] not in (["T", "1"], ["T", "2"], ["T", "3"], ["T", "4"], ["T", "5"]):
            return None
        expected_fields = {"1": 31, "2": 33, "3": 40, "4": 48, "5": 50}[parts[1]]
        if len(parts) != expected_fields:
            return None
        try:
            values = [float(value) for value in parts[7:]]
            if parts[1] in ("4", "5"):
                return {
                    "sequence": int(parts[2]), "timestamp_ms": int(parts[3]),
                    "state": int(parts[4]), "fault": int(parts[5]), "imu_valid": int(parts[6]),
                    "pitch": values[0], "pitch_rate": values[1], "accel_pitch": values[2],
                    "accel_x": values[3], "accel_y": values[4], "accel_z": values[5],
                    "gyro_x": values[6], "gyro_y": values[7], "gyro_z": values[8],
                    "target_speed": values[9], "filtered_speed": values[10], "speed_error": values[11],
                    "pitch_offset": values[12], "turn": values[13],
                    "differential_speed": values[14], "differential_speed_error": values[15],
                    "turn_motor_command": values[16], "applied_turn_motor_command": values[17],
                    "motor_left": values[18], "motor_right": values[19],
                    "balance_kp": values[20], "balance_ki": values[21], "balance_kd": values[22],
                    "balance_trim": values[23], "speed_kp": values[24], "speed_ki": values[25],
                    "max_motor": values[26], "max_pitch": values[27],
                    "wheel_left": values[28], "wheel_right": values[29],
                    "requested_pitch": values[30], "balance_pitch_error": values[31],
                    "balance_p_term": values[32], "balance_i_term": values[33],
                    "balance_d_term": values[34], "balance_motor_raw": values[35],
                    "speed_invert": int(values[36]), "turn_kp": values[37], "turn_ki": values[38],
                    "max_turn": values[39], "turn_invert": int(values[40]),
                    "heading": values[41] if parts[1] == "5" else None,
                    "yaw_rate": values[42] if parts[1] == "5" else None,
                }
            return {
                "sequence": int(parts[2]), "timestamp_ms": int(parts[3]),
                "state": int(parts[4]), "fault": int(parts[5]), "imu_valid": int(parts[6]),
                "pitch": values[0], "pitch_rate": values[1], "accel_pitch": values[2],
                "accel_x": values[3], "accel_y": values[4], "accel_z": values[5],
                "gyro_x": values[6], "gyro_y": values[7], "gyro_z": values[8],
                "target_speed": values[9], "filtered_speed": values[10], "speed_error": values[11],
                "pitch_offset": values[12], "turn": values[13],
                "motor_left": values[14], "motor_right": values[15],
                "balance_kp": values[16], "balance_ki": values[17], "balance_kd": values[18],
                "balance_trim": values[19], "speed_kp": values[20], "speed_ki": values[21],
                "max_motor": values[22], "max_pitch": values[23],
                "wheel_left": values[24] if parts[1] in ("2", "3") else None,
                "wheel_right": values[25] if parts[1] in ("2", "3") else None,
                "requested_pitch": values[26] if parts[1] == "3" else None,
                "balance_pitch_error": values[27] if parts[1] == "3" else None,
                "balance_p_term": values[28] if parts[1] == "3" else None,
                "balance_i_term": values[29] if parts[1] == "3" else None,
                "balance_d_term": values[30] if parts[1] == "3" else None,
                "balance_motor_raw": values[31] if parts[1] == "3" else None,
                "speed_invert": int(values[32]) if parts[1] == "3" else None,
                "differential_speed": None, "differential_speed_error": None,
                "turn_motor_command": None, "applied_turn_motor_command": None,
                "turn_kp": None, "turn_ki": None, "max_turn": None, "turn_invert": None,
            }
        except ValueError:
            return None


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-S3 平衡小车上位机")
        self.setGeometry(100, 100, 1300, 850)

        self.worker = None
        self.last_frame = None

        self._build_ui()
        self._apply_styles()
        self._on_protocol_changed(0)

    # -----------------------------------------------------------------------
    # 界面构建
    # -----------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ---- 顶部连接控制栏 ----
        control_group = QGroupBox("连接设置")
        control_layout = QGridLayout(control_group)

        control_layout.addWidget(QLabel("协议:"), 0, 0)
        self.cmb_protocol = QComboBox()
        self.cmb_protocol.addItems(["HTTP MJPEG", "HTTP 单帧", "UDP 裸二进制"])
        self.cmb_protocol.currentIndexChanged.connect(self._on_protocol_changed)
        control_layout.addWidget(self.cmb_protocol, 0, 1)

        control_layout.addWidget(QLabel("ESP32 IP:"), 0, 2)
        self.ip_edit = QLineEdit("192.168.4.1")
        self.ip_edit.setFixedWidth(140)
        control_layout.addWidget(self.ip_edit, 0, 3)

        control_layout.addWidget(QLabel("HTTP Port:"), 0, 4)
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(80)
        self.port_spin.setFixedWidth(80)
        control_layout.addWidget(self.port_spin, 0, 5)

        control_layout.addWidget(QLabel("本地 UDP 端口:"), 0, 6)
        self.udp_local_port = QSpinBox()
        self.udp_local_port.setRange(1024, 65535)
        self.udp_local_port.setValue(8888)
        self.udp_local_port.setFixedWidth(80)
        control_layout.addWidget(self.udp_local_port, 0, 7)

        control_layout.addWidget(QLabel("命令端口:"), 0, 8)
        self.udp_cmd_port = QSpinBox()
        self.udp_cmd_port.setRange(1, 65535)
        self.udp_cmd_port.setValue(8889)
        self.udp_cmd_port.setFixedWidth(80)
        control_layout.addWidget(self.udp_cmd_port, 0, 9)

        self.btn_start = QPushButton("开始")
        self.btn_start.clicked.connect(self.start_stream)
        control_layout.addWidget(self.btn_start, 0, 10)

        self.btn_stop = QPushButton("停止")
        self.btn_stop.clicked.connect(self.stop_worker)
        self.btn_stop.setEnabled(False)
        control_layout.addWidget(self.btn_stop, 0, 11)

        self.btn_snapshot = QPushButton("保存快照")
        self.btn_snapshot.clicked.connect(self.save_snapshot)
        control_layout.addWidget(self.btn_snapshot, 0, 12)

        control_layout.setColumnStretch(13, 1)
        main_layout.addWidget(control_group)

        # ---- 主体：左侧图像 + 右侧参数 ----
        body_splitter = QSplitter(Qt.Horizontal)

        # 图像标签页
        self.tabs = QTabWidget()

        self.live_tab = QWidget()
        live_layout = QVBoxLayout(self.live_tab)
        self.live_label = QLabel("点击“开始”查看画面")
        self.live_label.setAlignment(Qt.AlignCenter)
        self.live_label.setMinimumSize(640, 480)
        self.live_label.setStyleSheet("background-color: #1a1a1a; color: #888;")
        live_layout.addWidget(self.live_label)

        status_layout = QHBoxLayout()
        self.lbl_fps = QLabel("FPS: 0.0")
        self.lbl_resolution = QLabel("Resolution: -")
        self.lbl_frame_size = QLabel("Frame size: -")
        self.lbl_udp_stats = QLabel("UDP: -")
        status_layout.addWidget(self.lbl_fps)
        status_layout.addWidget(self.lbl_resolution)
        status_layout.addWidget(self.lbl_frame_size)
        status_layout.addWidget(self.lbl_udp_stats)
        status_layout.addStretch()
        live_layout.addLayout(status_layout)

        self.tabs.addTab(self.live_tab, "实时画面")

        self.proc_tab = QWidget()
        proc_layout = QVBoxLayout(self.proc_tab)
        self.proc_label = QLabel("此处显示 ESP32 C++ 算法处理后的结果")
        self.proc_label.setAlignment(Qt.AlignCenter)
        self.proc_label.setMinimumSize(640, 480)
        self.proc_label.setStyleSheet("background-color: #1a1a1a; color: #888;")
        proc_layout.addWidget(self.proc_label)
        self.tabs.addTab(self.proc_tab, "算法处理")

        self._build_balance_debug_tab()
        body_splitter.addWidget(self.tabs)

        # 参数面板
        param_widget = QWidget()
        param_layout = QVBoxLayout(param_widget)

        # 图像传输参数
        tx_group = QGroupBox("图像传输参数（可下发到 ESP32）")
        tx_grid = QGridLayout(tx_group)

        tx_grid.addWidget(QLabel("JPEG 质量:"), 0, 0)
        self.spin_quality = QSpinBox()
        self.spin_quality.setRange(10, 100)
        self.spin_quality.setValue(30)
        self.spin_quality.setToolTip("越小体积越小、帧率越高；越大画质越好")
        tx_grid.addWidget(self.spin_quality, 0, 1)

        tx_grid.addWidget(QLabel("发送间隔 ms:"), 0, 2)
        self.spin_interval = QSpinBox()
        self.spin_interval.setRange(0, 1000)
        self.spin_interval.setValue(5)
        self.spin_interval.setToolTip("每帧编码后让出 CPU 的时间，0~20 即可")
        tx_grid.addWidget(self.spin_interval, 0, 3)

        tx_grid.addWidget(QLabel("图传间隔帧:"), 1, 0)
        self.spin_stream_divider = QSpinBox()
        self.spin_stream_divider.setRange(0, 30)
        self.spin_stream_divider.setValue(2)
        self.spin_stream_divider.setToolTip("0=关闭调试图传；1=每个视觉帧回传；2=每两帧回传一次")
        tx_grid.addWidget(self.spin_stream_divider, 1, 1)

        self.btn_apply_tx = QPushButton("应用并下发")
        self.btn_apply_tx.clicked.connect(self.apply_tx_params)
        tx_grid.addWidget(self.btn_apply_tx, 0, 4)

        tx_grid.addWidget(QLabel("单帧间隔 ms:"), 2, 0)
        self.spin_single_interval = QSpinBox()
        self.spin_single_interval.setRange(20, 2000)
        self.spin_single_interval.setValue(100)
        tx_grid.addWidget(self.spin_single_interval, 2, 1)

        tx_grid.addWidget(QLabel("UDP 单包大小:"), 2, 2)
        self.lbl_udp_payload = QLabel("1400 bytes")
        tx_grid.addWidget(self.lbl_udp_payload, 2, 3)

        param_layout.addWidget(tx_group)

        # C++ 算法参数（直接下发给 ESP32，由 ESP32 端算法处理）
        roi_group = QGroupBox("C++ 算法参数（ESP32 端执行）")
        roi_grid = QGridLayout(roi_group)

        roi_grid.addWidget(QLabel("流模式:"), 0, 0)
        self.cmb_vision_mode = QComboBox()
        self.cmb_vision_mode.addItems(["原图", "C++ 处理图", "C++ 二值化图", "C++ 边缘图",
                                        "C++ 灰度增强图", "C++ 原始灰度图"])
        self.cmb_vision_mode.setCurrentIndex(1)  # 默认显示 C++ 处理效果
        self.cmb_vision_mode.setToolTip(
            "原图=ESP32 原图；处理图=带绿色掩膜/中线/ROI/局部中心线；"
            "二值化图=白线黑底；边缘图=Sobel 白边黑底；"
            "灰度增强图=CLAHE 对比度增强后灰度；原始灰度图=未增强的灰度"
        )
        roi_grid.addWidget(self.cmb_vision_mode, 0, 1, 1, 3)

        roi_grid.addWidget(QLabel("灰度阈值:"), 1, 0)
        self.spin_threshold = QSpinBox()
        self.spin_threshold.setRange(0, 255)
        self.spin_threshold.setValue(100)
        self.spin_threshold.setToolTip("黑线白底建议 80~120；启用 Otsu 时作为 fallback")
        roi_grid.addWidget(self.spin_threshold, 1, 1)

        self.chk_otsu = QCheckBox("启用 Otsu")
        self.chk_otsu.setChecked(True)
        self.chk_otsu.setToolTip("自动根据 ROI 灰度直方图计算阈值；几乎不增加耗时")
        roi_grid.addWidget(self.chk_otsu, 1, 2, 1, 2)

        roi_grid.addWidget(QLabel("Otsu 阈值范围:"), 2, 0)
        self.spin_otsu_min = QSpinBox()
        self.spin_otsu_min.setRange(0, 255)
        self.spin_otsu_min.setValue(60)
        roi_grid.addWidget(self.spin_otsu_min, 2, 1)
        self.spin_otsu_max = QSpinBox()
        self.spin_otsu_max.setRange(0, 255)
        self.spin_otsu_max.setValue(150)
        roi_grid.addWidget(self.spin_otsu_max, 2, 2)
        self.spin_otsu_step = QSpinBox()
        self.spin_otsu_step.setRange(1, 50)
        self.spin_otsu_step.setValue(6)
        self.spin_otsu_step.setToolTip("Otsu 稳定阈值单帧最大变化量")
        roi_grid.addWidget(self.spin_otsu_step, 2, 3)

        roi_grid.addWidget(QLabel("前景占比 0.1%:"), 3, 0)
        self.spin_fg_min = QSpinBox()
        self.spin_fg_min.setRange(0, 1000)
        self.spin_fg_min.setValue(5)
        roi_grid.addWidget(self.spin_fg_min, 3, 1)
        self.spin_fg_max = QSpinBox()
        self.spin_fg_max.setRange(0, 1000)
        self.spin_fg_max.setValue(300)
        roi_grid.addWidget(self.spin_fg_max, 3, 2)
        self.spin_otsu_alpha = QSpinBox()
        self.spin_otsu_alpha.setRange(1, 100)
        self.spin_otsu_alpha.setValue(25)
        self.spin_otsu_alpha.setToolTip("Otsu 阈值当前帧权重 x100")
        roi_grid.addWidget(self.spin_otsu_alpha, 3, 3)

        roi_grid.addWidget(QLabel("对比度增强 x100:"), 4, 0)
        self.spin_contrast = QSpinBox()
        self.spin_contrast.setRange(0, 200)
        self.spin_contrast.setValue(130)
        self.spin_contrast.setToolTip("100=原始灰度；101~200=CLAHE 对比度增强，150 为常用起点")
        roi_grid.addWidget(self.spin_contrast, 4, 1)

        roi_grid.addWidget(QLabel("边缘阈值:"), 4, 2)
        self.spin_edge_threshold = QSpinBox()
        self.spin_edge_threshold.setRange(0, 255)
        self.spin_edge_threshold.setValue(80)
        self.spin_edge_threshold.setToolTip("Sobel 边缘图阈值，仅在边缘图模式生效")
        roi_grid.addWidget(self.spin_edge_threshold, 4, 3)

        roi_grid.addWidget(QLabel("平滑滤波:"), 5, 0)
        self.chk_smooth = QCheckBox("启用 3x3 高斯平滑")
        self.chk_smooth.setChecked(True)
        self.chk_smooth.setToolTip("在阈值前对灰度图做 3x3 高斯平滑，抑制随机噪声、柔和反光")
        roi_grid.addWidget(self.chk_smooth, 5, 1)

        roi_grid.addWidget(QLabel("形态学清理:"), 5, 2)
        self.chk_morph = QCheckBox("启用 3x3 清理")
        self.chk_morph.setChecked(True)
        self.chk_morph.setToolTip("对二值掩膜去孤点/填小洞，减少二值图噪点")
        roi_grid.addWidget(self.chk_morph, 5, 3)

        roi_grid.addWidget(QLabel("平滑系数 x100:"), 6, 0)
        self.spin_smooth = QSpinBox()
        self.spin_smooth.setRange(1, 100)
        self.spin_smooth.setValue(82)
        self.spin_smooth.setToolTip("1=最平滑（历史权重高），100=不平滑；推荐 70~90")
        roi_grid.addWidget(self.spin_smooth, 6, 1)

        roi_grid.addWidget(QLabel("断行 / 保持帧:"), 6, 2)
        self.spin_row_gap = QSpinBox()
        self.spin_row_gap.setRange(0, 30)
        self.spin_row_gap.setValue(4)
        roi_grid.addWidget(self.spin_row_gap, 6, 3)
        roi_grid.addWidget(QLabel("保持帧数:"), 7, 2)
        self.spin_hold_frames = QSpinBox()
        self.spin_hold_frames.setRange(0, 60)
        self.spin_hold_frames.setValue(5)
        self.spin_hold_frames.setToolTip("失线时保持最近有效结果的最大帧数")
        roi_grid.addWidget(self.spin_hold_frames, 7, 3)

        roi_grid.addWidget(QLabel("前瞻行 y:"), 8, 0)
        self.spin_lookahead_y = QSpinBox()
        self.spin_lookahead_y.setRange(0, 480)
        self.spin_lookahead_y.setValue(112)
        self.spin_lookahead_y.setToolTip("23 cm~1.1 m 视距下的初始值；应按实测距离标定")
        roi_grid.addWidget(self.spin_lookahead_y, 8, 1)

        roi_grid.addWidget(QLabel("ROI 上边界:"), 9, 0)
        self.spin_roi_y1 = QSpinBox()
        self.spin_roi_y1.setRange(0, 480)
        self.spin_roi_y1.setValue(60)
        roi_grid.addWidget(self.spin_roi_y1, 9, 1)

        roi_grid.addWidget(QLabel("ROI 下边界:"), 9, 2)
        self.spin_roi_y2 = QSpinBox()
        self.spin_roi_y2.setRange(0, 480)
        self.spin_roi_y2.setValue(236)
        roi_grid.addWidget(self.spin_roi_y2, 9, 3)

        roi_grid.addWidget(QLabel("最小线宽:"), 10, 0)
        self.spin_min_width = QSpinBox()
        self.spin_min_width.setRange(0, 320)
        self.spin_min_width.setValue(2)
        self.spin_min_width.setToolTip("最远 1.1m 黑线预计约 3~5 像素，保留 2 像素下限")
        roi_grid.addWidget(self.spin_min_width, 10, 1)

        roi_grid.addWidget(QLabel("最大线宽:"), 10, 2)
        self.spin_max_width = QSpinBox()
        self.spin_max_width.setRange(0, 320)
        self.spin_max_width.setValue(64)
        self.spin_max_width.setToolTip("过滤纸外阴影等大片黑区；若近端黑线被截断再增大")
        roi_grid.addWidget(self.spin_max_width, 10, 3)

        self.btn_apply_vision = QPushButton("下发算法参数")
        self.btn_apply_vision.setToolTip("将阈值、ROI、线宽、流模式、CLAHE 对比度增强、Otsu、边缘阈值、平滑滤波、形态学清理、平滑系数下发到 ESP32")
        self.btn_apply_vision.clicked.connect(self.apply_vision_params)
        roi_grid.addWidget(self.btn_apply_vision, 11, 0, 1, 4)

        roi_grid.setRowStretch(12, 1)
        param_layout.addWidget(roi_group)

        body_splitter.addWidget(param_widget)
        body_splitter.setSizes([900, 350])
        main_layout.addWidget(body_splitter)

        # ---- 日志区 ----
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumHeight(180)
        main_layout.addWidget(self.log_edit)

        # 状态栏
        self.statusBar().showMessage("Ready")

    # -----------------------------------------------------------------------
    # 样式
    # -----------------------------------------------------------------------
    def _build_balance_debug_tab(self):
        self.balance_worker = None
        self.balance_command_sequence = 0
        self.balance_pending_tuning = {}
        self.balance_pending_sequences = {}
        self.balance_tuning_loaded = False
        self.balance_last_received_monotonic = None
        self.balance_last_telemetry_sequence = None
        self.balance_last_board_timestamp_ms = None
        self.balance_rx_hz = 0.0
        self.speed_record_file = None
        self.speed_record_writer = None
        self.speed_record_path = None
        self.balance_age_timer = QTimer(self)
        self.balance_age_timer.setInterval(100)
        self.balance_age_timer.timeout.connect(self._update_balance_packet_age)
        self.balance_tab = QWidget()
        layout = QVBoxLayout(self.balance_tab)

        connection_group = QGroupBox("主板 Wi-Fi / UDP 连接")
        connection_grid = QGridLayout(connection_group)
        connection_grid.addWidget(QLabel("主板 IP:"), 0, 0)
        self.balance_ip_edit = QLineEdit("192.168.4.1")
        self.balance_ip_edit.setFixedWidth(140)
        connection_grid.addWidget(self.balance_ip_edit, 0, 1)
        connection_grid.addWidget(QLabel("本地遥测端口:"), 0, 2)
        self.balance_local_port = QSpinBox()
        self.balance_local_port.setRange(1024, 65535)
        self.balance_local_port.setValue(9000)
        connection_grid.addWidget(self.balance_local_port, 0, 3)
        connection_grid.addWidget(QLabel("主板命令端口:"), 0, 4)
        self.balance_command_port = QSpinBox()
        self.balance_command_port.setRange(1024, 65535)
        self.balance_command_port.setValue(9001)
        connection_grid.addWidget(self.balance_command_port, 0, 5)
        self.btn_balance_connect = QPushButton("连接主板")
        self.btn_balance_connect.clicked.connect(self.start_balance_debug)
        connection_grid.addWidget(self.btn_balance_connect, 0, 6)
        self.btn_balance_disconnect = QPushButton("断开")
        self.btn_balance_disconnect.clicked.connect(self.stop_balance_debug)
        self.btn_balance_disconnect.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_disconnect, 0, 7)
        self.lbl_balance_link = QLabel("未连接")
        self.lbl_balance_link.setStyleSheet("color: #666;")
        connection_grid.addWidget(self.lbl_balance_link, 1, 0, 1, 8)
        self.btn_balance_arm = QPushButton("启动平衡")
        self.btn_balance_arm.clicked.connect(self.request_balance_arm)
        self.btn_balance_arm.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_arm, 2, 0, 1, 3)
        self.btn_balance_stop = QPushButton("停止平衡")
        self.btn_balance_stop.setObjectName("stop")
        self.btn_balance_stop.clicked.connect(self.request_balance_stop)
        self.btn_balance_stop.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_stop, 2, 3, 1, 3)
        connection_grid.addWidget(QLabel("启动仍需通过主板自检、IMU 与姿态角安全校验。"), 2, 6, 1, 2)

        self.btn_balance_reset = QPushButton("重启主板")
        self.btn_balance_reset.setObjectName("stop")
        self.btn_balance_reset.setToolTip("等同按下主板 RESET：主板将立即重启，Wi-Fi 连接会暂时断开。")
        self.btn_balance_reset.clicked.connect(self.request_board_reset)
        self.btn_balance_reset.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_reset, 3, 0, 1, 2)

        connection_grid.addWidget(QLabel("前进速度给定 (m/s):"), 3, 2)
        self.spin_balance_drive_speed = QDoubleSpinBox()
        self.spin_balance_drive_speed.setDecimals(3)
        self.spin_balance_drive_speed.setRange(0.0, 0.250)
        self.spin_balance_drive_speed.setSingleStep(0.010)
        self.spin_balance_drive_speed.setValue(0.0)
        self.spin_balance_drive_speed.setEnabled(False)
        self.spin_balance_drive_speed.setToolTip("仅在 BALANCING 状态下可下发；主板会按 0.10 m/s² 斜坡改变给定")
        connection_grid.addWidget(self.spin_balance_drive_speed, 3, 3)
        self.btn_balance_drive = QPushButton("设置前进速度")
        self.btn_balance_drive.clicked.connect(self.request_balance_drive_speed)
        self.btn_balance_drive.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_drive, 3, 4, 1, 2)
        self.btn_balance_drive_zero = QPushButton("回到原地平衡")
        self.btn_balance_drive_zero.clicked.connect(self.request_balance_drive_zero)
        self.btn_balance_drive_zero.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_drive_zero, 3, 6, 1, 2)

        connection_grid.addWidget(QLabel("转向差速度给定 (右-左, m/s):"), 4, 0, 1, 2)
        self.spin_balance_turn_speed = QDoubleSpinBox()
        self.spin_balance_turn_speed.setDecimals(3)
        self.spin_balance_turn_speed.setRange(-0.200, 0.200)
        self.spin_balance_turn_speed.setSingleStep(0.010)
        self.spin_balance_turn_speed.setValue(0.0)
        self.spin_balance_turn_speed.setEnabled(False)
        self.spin_balance_turn_speed.setToolTip(
            "仅在 BALANCING 状态下可下发；正值表示右轮比左轮快，负值表示左轮比右轮快"
        )
        connection_grid.addWidget(self.spin_balance_turn_speed, 4, 2)
        self.btn_balance_turn = QPushButton("设置转向差速度")
        self.btn_balance_turn.clicked.connect(self.request_balance_turn_speed)
        self.btn_balance_turn.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_turn, 4, 3, 1, 2)
        self.btn_balance_turn_zero = QPushButton("转向差速度清零")
        self.btn_balance_turn_zero.clicked.connect(self.request_balance_turn_zero)
        self.btn_balance_turn_zero.setEnabled(False)
        connection_grid.addWidget(self.btn_balance_turn_zero, 4, 5, 1, 3)

        connection_grid.addWidget(QLabel("自动路线:"), 5, 0)
        self.cmb_route_direction = QComboBox()
        self.cmb_route_direction.addItems(["左转（两次半圆）", "右转（两次半圆）"])
        connection_grid.addWidget(self.cmb_route_direction, 5, 1, 1, 2)
        self.spin_route_turn = QDoubleSpinBox()
        self.spin_route_turn.setRange(0.01, 0.20)
        self.spin_route_turn.setDecimals(3)
        self.spin_route_turn.setSingleStep(0.01)
        self.spin_route_turn.setValue(0.060)
        self.spin_route_turn.setToolTip("目标右减左轮速差（m/s）；需通过实际转弯半径校准")
        connection_grid.addWidget(self.spin_route_turn, 5, 3)
        self.btn_route_start = QPushButton("执行 2m-半圆-2m-半圆")
        self.btn_route_start.clicked.connect(self.start_route)
        self.btn_route_start.setEnabled(False)
        connection_grid.addWidget(self.btn_route_start, 5, 4, 1, 2)
        self.btn_route_cancel = QPushButton("取消路线")
        self.btn_route_cancel.clicked.connect(self.cancel_route)
        self.btn_route_cancel.setEnabled(False)
        connection_grid.addWidget(self.btn_route_cancel, 5, 6)
        self.lbl_route_status = QLabel("路线未运行")
        connection_grid.addWidget(self.lbl_route_status, 6, 0, 1, 8)
        layout.addWidget(connection_group)

        live_group = QGroupBox("实时状态")
        live_grid = QGridLayout(live_group)
        self.balance_value_labels = {}
        status_fields = [
            ("state", "安全状态"), ("fault", "故障"), ("imu", "IMU"),
            ("pitch", "俯仰角 (°)"), ("pitch_rate", "俯仰角速度 (°/s)"),
            ("accel_pitch", "加速度姿态角 (°)"),
            ("accel", "加速度 g (X,Y,Z)"), ("gyro", "陀螺仪 °/s (X,Y,Z)"),
            ("speed", "目标 / 实际速度 (m/s)"), ("speed_error", "速度误差 (m/s)"),
            ("wheel_speed", "左右车轮线速度 (m/s)"),
            ("heading", "相对航向 / 转向角速度 (° / °/s)"),
            ("pitch_offset", "速度环俯角输出 (°)"), ("turn", "目标/实际差速度与转向输出"),
            ("motor", "左右电机输出"), ("packet", "包序号 / 包龄 / 频率"),
        ]
        for index, (key, title) in enumerate(status_fields):
            row, column = divmod(index, 3)
            live_grid.addWidget(QLabel(f"{title}:"), row, column * 2)
            value_label = QLabel("-")
            value_label.setMinimumWidth(145)
            self.balance_value_labels[key] = value_label
            live_grid.addWidget(value_label, row, column * 2 + 1)
        layout.addWidget(live_group)

        tuning_group = QGroupBox("在线 PID / PI 参数（仅调参，不含启动、急停、速度或转向命令）")
        tuning_grid = QGridLayout(tuning_group)
        self.balance_tuning_spins = {}
        tuning_fields = [
            ("balance_kp", "角度 Kp", 0.12, 5),
            ("balance_ki", "角度 Ki", 0.0, 5),
            ("balance_kd", "角度 Kd", 0.003, 5),
            ("balance_trim", "平衡点 Trim (°)", -2.09, 3),
            ("speed_kp", "速度 Kp", 11.0, 5),
            ("speed_ki", "速度 Ki", 0.15, 6),
            ("turn_kp", "差速度 Kp", 1.0, 5),
            ("turn_ki", "差速度 Ki", 0.0, 6),
            ("max_turn", "最大转向输出 (0–1)", 0.20, 3),
            ("max_motor", "最大电机输出 (0–1)", 0.45, 3),
            ("max_pitch", "最大俯仰偏置 (°)", 6.0, 2),
        ]
        for index, (key, title, value, decimals) in enumerate(tuning_fields):
            row, column = divmod(index, 3)
            tuning_grid.addWidget(QLabel(title + ":"), row, column * 2)
            spin = QDoubleSpinBox()
            spin.setDecimals(decimals)
            if key == "balance_trim":
                spin.setRange(-20.0, 20.0)
                spin.setSingleStep(0.05)
            elif key in ("max_motor", "max_turn"):
                spin.setRange(0.0, 1.0)
                spin.setSingleStep(0.01)
            elif key == "max_pitch":
                spin.setRange(0.0, 15.0)
                spin.setSingleStep(0.1)
            else:
                spin.setRange(0.0, 100.0)
                spin.setSingleStep(0.0001 if key == "speed_ki" else 0.001)
            spin.setValue(value)
            spin.setMinimumWidth(120)
            self.balance_tuning_spins[key] = spin
            tuning_grid.addWidget(spin, row, column * 2 + 1)
        self.btn_apply_balance_loop_tuning = QPushButton("应用平衡环参数")
        self.btn_apply_balance_loop_tuning.setToolTip(
            "仅下发角度 Kp/Ki/Kd、平衡点 Trim 与最大电机输出，不改动速度环参数"
        )
        self.btn_apply_balance_loop_tuning.clicked.connect(self.apply_balance_loop_tuning)
        self.btn_apply_balance_loop_tuning.setEnabled(False)
        tuning_grid.addWidget(self.btn_apply_balance_loop_tuning, 4, 0, 1, 3)

        self.btn_apply_balance_tuning = QPushButton("应用全部参数（含速度环）")
        self.btn_apply_balance_tuning.clicked.connect(self.apply_balance_tuning)
        self.btn_apply_balance_tuning.setEnabled(False)
        tuning_grid.addWidget(self.btn_apply_balance_tuning, 4, 3, 1, 3)
        layout.addWidget(tuning_group)

        console_group = QGroupBox("主板串口日志镜像")
        console_layout = QVBoxLayout(console_group)
        self.balance_console_log = QTextEdit()
        self.balance_console_log.setReadOnly(True)
        self.balance_console_log.setMaximumHeight(200)
        self.balance_console_log.document().setMaximumBlockCount(500)
        console_layout.addWidget(self.balance_console_log)
        layout.addWidget(console_group)
        layout.addStretch()
        self.tabs.addTab(self.balance_tab, "主板调试")

    def _apply_styles(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #ccc;
                border-radius: 5px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                padding: 6px 14px;
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:disabled {
                background-color: #bbb;
            }
            QPushButton#stop {
                background-color: #f44336;
            }
            QLabel {
                font-size: 13px;
            }
        """)

    # -----------------------------------------------------------------------
    # 事件回调
    # -----------------------------------------------------------------------
    def start_balance_debug(self):
        self.stop_balance_debug()
        self._start_speed_recording()
        self.balance_worker = BalanceTelemetryWorker(
            self.balance_local_port.value(), self.balance_ip_edit.text().strip(),
            self.balance_command_port.value()
        )
        self.balance_worker.telemetry_ready.connect(self.on_balance_telemetry)
        self.balance_worker.ack_received.connect(self.on_balance_ack)
        self.balance_worker.command_failed.connect(self.on_balance_command_failed)
        self.balance_worker.console_line_received.connect(self.on_balance_console_line)
        self.balance_worker.info_updated.connect(self.log)
        self.balance_worker.error_occurred.connect(self.on_error)
        self.balance_worker.start()
        self.balance_age_timer.start()
        self.balance_last_telemetry_sequence = None
        self.balance_last_board_timestamp_ms = None
        self.balance_command_sequence = int(time.time_ns() & 0x7fffffff)
        self.btn_balance_connect.setEnabled(False)
        self.btn_balance_disconnect.setEnabled(True)
        self.balance_tuning_loaded = False
        self.btn_apply_balance_loop_tuning.setEnabled(False)
        self.btn_apply_balance_tuning.setEnabled(False)
        self.btn_balance_stop.setEnabled(True)
        self.btn_balance_reset.setEnabled(True)
        self.btn_balance_arm.setEnabled(False)
        self.route_active = False
        self.route_stage = 0
        self.route_distance_m = 0.0
        self.route_last_timestamp_ms = None
        self.spin_balance_drive_speed.setEnabled(False)
        self.btn_balance_drive.setEnabled(False)
        self.btn_balance_drive_zero.setEnabled(False)
        self.spin_balance_turn_speed.setEnabled(False)
        self.btn_balance_turn.setEnabled(False)
        self.btn_balance_turn_zero.setEnabled(False)
        self.btn_route_start.setEnabled(False)
        self.btn_route_cancel.setEnabled(False)
        self.balance_ip_edit.setEnabled(False)
        self.balance_local_port.setEnabled(False)
        self.balance_command_port.setEnabled(False)
        self.lbl_balance_link.setText("等待主板遥测...")
        self.lbl_balance_link.setStyleSheet("color: #b36b00;")

    def stop_balance_debug(self):
        if self.balance_worker is not None:
            self.balance_worker.stop()
            self.balance_worker.wait(1000)
            self.balance_worker = None
        if hasattr(self, "balance_pending_tuning"):
            self.balance_pending_tuning.clear()
            self.balance_pending_sequences.clear()
        self.balance_tuning_loaded = False
        self.balance_last_telemetry_sequence = None
        self.balance_last_board_timestamp_ms = None
        self._stop_speed_recording()
        if hasattr(self, "balance_age_timer"):
            self.balance_age_timer.stop()
        if hasattr(self, "btn_balance_connect"):
            self.btn_balance_connect.setEnabled(True)
            self.btn_balance_disconnect.setEnabled(False)
            self.btn_apply_balance_loop_tuning.setEnabled(False)
            self.btn_apply_balance_tuning.setEnabled(False)
            self.btn_balance_arm.setEnabled(False)
            self.btn_balance_stop.setEnabled(False)
            self.btn_balance_reset.setEnabled(False)
            self.spin_balance_drive_speed.setEnabled(False)
            self.btn_balance_drive.setEnabled(False)
            self.btn_balance_drive_zero.setEnabled(False)
            self.spin_balance_turn_speed.setEnabled(False)
            self.btn_balance_turn.setEnabled(False)
            self.btn_balance_turn_zero.setEnabled(False)
            self.cancel_route(send_stop=False)
            self.btn_route_start.setEnabled(False)
            self.balance_ip_edit.setEnabled(True)
            self.balance_local_port.setEnabled(True)
            self.balance_command_port.setEnabled(True)
            self.lbl_balance_link.setText("未连接")
            self.lbl_balance_link.setStyleSheet("color: #666;")

    def _start_speed_recording(self):
        """为本次主板调试创建目标/实际速度 CSV 记录。"""
        records_dir = Path(__file__).resolve().parent / "records"
        try:
            records_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.speed_record_path = records_dir / f"balance_speed_{timestamp}.csv"
            self.speed_record_file = self.speed_record_path.open(
                "w", encoding="utf-8-sig", newline=""
            )
            self.speed_record_writer = csv.writer(self.speed_record_file)
            self.speed_record_writer.writerow([
                "timestamp", "board_timestamp_ms", "sequence",
                "safety_state", "fault_code", "imu_valid",
                "pitch_deg", "pitch_rate_dps", "accelerometer_pitch_deg",
                "accel_x_g", "accel_y_g", "accel_z_g",
                "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "requested_pitch_deg",
                "balance_pitch_error_deg", "balance_p_term", "balance_i_term", "balance_d_term",
                "balance_motor_raw", "left_motor_command", "right_motor_command",
                "left_wheel_mps", "right_wheel_mps",
                "balance_kp", "balance_ki", "balance_kd", "balance_trim",
                "speed_kp", "speed_ki", "speed_invert", "target_speed",
                "target_differential_speed_mps", "filtered_differential_speed_mps",
                "differential_speed_error_mps", "turn_motor_command", "applied_turn_motor_command",
                "turn_kp", "turn_ki", "max_turn", "turn_invert",
                "relative_heading_deg", "yaw_rate_deg_per_s",
            ])
            self.speed_record_file.flush()
            self.log(f"速度记录已开始：{self.speed_record_path}")
        except OSError as error:
            self.speed_record_file = None
            self.speed_record_writer = None
            self.speed_record_path = None
            self.log(f"[ERROR] 无法创建速度记录文件：{error}")

    def _stop_speed_recording(self):
        if self.speed_record_file is None:
            return
        record_path = self.speed_record_path
        try:
            self.speed_record_file.close()
            self.log(f"速度记录已保存：{record_path}")
        except OSError as error:
            self.log(f"[ERROR] 关闭速度记录文件失败：{error}")
        finally:
            self.speed_record_file = None
            self.speed_record_writer = None
            self.speed_record_path = None

    def _record_speed_telemetry(self, telemetry: dict):
        if self.speed_record_writer is None or self.speed_record_file is None:
            return
        try:
            def optional_float(key):
                value = telemetry.get(key)
                return "" if value is None else f"{value:.6f}"

            self.speed_record_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                telemetry["timestamp_ms"], telemetry["sequence"],
                telemetry["state"], telemetry["fault"], telemetry["imu_valid"],
                f"{telemetry['pitch']:.6f}", f"{telemetry['pitch_rate']:.6f}",
                f"{telemetry['accel_pitch']:.6f}",
                f"{telemetry['accel_x']:.6f}", f"{telemetry['accel_y']:.6f}", f"{telemetry['accel_z']:.6f}",
                f"{telemetry['gyro_x']:.6f}", f"{telemetry['gyro_y']:.6f}", f"{telemetry['gyro_z']:.6f}",
                optional_float("requested_pitch"), optional_float("balance_pitch_error"),
                optional_float("balance_p_term"), optional_float("balance_i_term"),
                optional_float("balance_d_term"), optional_float("balance_motor_raw"),
                f"{telemetry['motor_left']:.6f}", f"{telemetry['motor_right']:.6f}",
                optional_float("wheel_left"), optional_float("wheel_right"),
                f"{telemetry['balance_kp']:.6f}", f"{telemetry['balance_ki']:.6f}",
                f"{telemetry['balance_kd']:.6f}", f"{telemetry['balance_trim']:.6f}",
                f"{telemetry['speed_kp']:.6f}", f"{telemetry['speed_ki']:.6f}",
                "" if telemetry.get("speed_invert") is None else telemetry["speed_invert"],
                f"{telemetry['target_speed']:.6f}",
                f"{telemetry['turn']:.6f}", optional_float("differential_speed"),
                optional_float("differential_speed_error"), optional_float("turn_motor_command"),
                optional_float("applied_turn_motor_command"), optional_float("turn_kp"),
                optional_float("turn_ki"), optional_float("max_turn"),
                "" if telemetry.get("turn_invert") is None else telemetry["turn_invert"],
                optional_float("heading"), optional_float("yaw_rate"),
            ])
            # 每包落盘，异常关闭时也最多损失当前一行记录。
            self.speed_record_file.flush()
        except (OSError, KeyError) as error:
            self.log(f"[ERROR] 写入速度记录失败：{error}")
            self._stop_speed_recording()

    def apply_balance_tuning(self):
        command_fields = [
            ("balance", "kp", "balance_kp"), ("balance", "ki", "balance_ki"),
            ("balance", "kd", "balance_kd"), ("balance", "trim", "balance_trim"),
            ("speed", "kp", "speed_kp"), ("speed", "ki", "speed_ki"),
            ("turn", "kp", "turn_kp"), ("turn", "ki", "turn_ki"), ("turn", "max", "max_turn"),
            ("balance", "max_motor", "max_motor"), ("speed", "max_pitch", "max_pitch"),
        ]
        self._send_tuning_commands(
            command_fields, "已发送全部 PID / PI、平衡点 Trim 与输出限幅参数，等待主板 ACK 与遥测回读"
        )

    def apply_balance_loop_tuning(self):
        command_fields = [
            ("balance", "kp", "balance_kp"), ("balance", "ki", "balance_ki"),
            ("balance", "kd", "balance_kd"), ("balance", "trim", "balance_trim"),
            ("balance", "max_motor", "max_motor"),
        ]
        self._send_tuning_commands(
            command_fields, "已发送平衡环参数：Kp、Ki、Kd、Trim、最大电机输出；速度环参数未改动"
        )

    def _send_tuning_commands(self, command_fields, success_message: str):
        if self.balance_worker is None or not self.balance_tuning_loaded:
            self.log("请先连接主板并等待首次参数遥测完成")
            return
        self.btn_apply_balance_loop_tuning.setEnabled(False)
        self.btn_apply_balance_tuning.setEnabled(False)
        for domain, parameter, key in command_fields:
            self.balance_command_sequence += 1
            value = self.balance_tuning_spins[key].value()
            self.balance_pending_tuning[key] = value
            self.balance_pending_sequences[self.balance_command_sequence] = {
                "key": key, "domain": domain, "parameter": parameter, "value": value,
            }
            command = f"P,{self.balance_command_sequence},{domain},{parameter},{value:.7f}\n"
            self.balance_worker.queue_command(command.encode("ascii"))
        self.log(success_message)

    def request_balance_arm(self):
        if self.balance_worker is None:
            self.log("请先连接主板 Wi-Fi 调试端口")
            return
        answer = QMessageBox.question(
            self, "确认启动平衡",
            "确认小车已放在安全地面、周围无人且姿态接近直立？\n"
            "主板仍会执行自检、IMU 和姿态角校验。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return
        self._send_balance_control("ARM")

    def request_balance_stop(self):
        if self.balance_worker is None:
            return
        self.cancel_route(send_stop=False)
        self._send_balance_control("STOP")

    def request_board_reset(self):
        if self.balance_worker is None:
            return
        answer = QMessageBox.question(
            self, "确认重启主板",
            "这相当于按下主板 RESET：平衡、Wi-Fi 与全部控制状态会立即重启。\n确认继续？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if answer == QMessageBox.Yes:
            self._send_balance_control("RESET")

    def start_route(self):
        if self.balance_worker is None:
            return
        if QMessageBox.question(self, "确认自动路线", "确认道路畅通、有人在旁可随时按停止平衡？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self.route_active = True
        self.route_stage = 0
        self.route_distance_m = 0.0
        self.route_last_timestamp_ms = None
        self.btn_route_start.setEnabled(False)
        self.btn_route_cancel.setEnabled(True)
        self._enter_route_stage()

    def cancel_route(self, send_stop=True):
        was_active = getattr(self, "route_active", False)
        self.route_active = False
        self.route_last_timestamp_ms = None
        if was_active and send_stop and self.balance_worker is not None:
            self._send_balance_control("TURN", 0.0)
            self._send_balance_control("DRIVE", 0.0)
        if hasattr(self, "lbl_route_status"):
            self.lbl_route_status.setText("路线已取消" if was_active else "路线未运行")
        if hasattr(self, "btn_route_cancel"):
            self.btn_route_cancel.setEnabled(False)

    def _enter_route_stage(self):
        # 轮距 0.20 m、目标直径 0.50 m；弧长按 pi*0.25 m 用编码器闭环计量。
        stages = [("前进 2.000 m", 2.000, 0.0), ("半圆 0.785 m", 3.141592653589793 * 0.25, None),
                  ("前进 2.000 m", 2.000, 0.0), ("半圆 0.785 m", 3.141592653589793 * 0.25, None)]
        if self.route_stage >= len(stages):
            self.route_active = False
            self._send_balance_control("TURN", 0.0)
            self._send_balance_control("DRIVE", 0.0)
            self.lbl_route_status.setText("路线完成：已回到原地平衡")
            self.btn_route_cancel.setEnabled(False)
            self.btn_route_start.setEnabled(self.balance_worker is not None)
            return
        title, target, turn = stages[self.route_stage]
        if turn is None:
            turn = self.spin_route_turn.value() * (1.0 if self.cmb_route_direction.currentIndex() == 0 else -1.0)
        self.route_distance_m = 0.0
        self.route_last_timestamp_ms = None
        self._send_balance_control("TURN", turn)
        self._send_balance_control("DRIVE", 0.100)
        self.lbl_route_status.setText(f"路线 {self.route_stage + 1}/4：{title}，已行进 0.000 / {target:.3f} m")

    def _update_route(self, telemetry, state):
        if not self.route_active:
            return
        if state != "BALANCING" or not telemetry["imu_valid"]:
            self.cancel_route(send_stop=False)
            self.lbl_route_status.setText("路线已中止：平衡状态或 IMU 异常")
            return
        timestamp = telemetry["timestamp_ms"]
        if self.route_last_timestamp_ms is not None:
            delta_s = (timestamp - self.route_last_timestamp_ms) / 1000.0
            if 0.0 < delta_s <= 0.2 and telemetry["wheel_left"] is not None:
                self.route_distance_m += max(0.0, (telemetry["wheel_left"] + telemetry["wheel_right"]) * 0.5) * delta_s
        self.route_last_timestamp_ms = timestamp
        targets = [2.000, 3.141592653589793 * 0.25, 2.000, 3.141592653589793 * 0.25]
        target = targets[self.route_stage]
        self.lbl_route_status.setText(f"路线 {self.route_stage + 1}/4：已行进 {self.route_distance_m:.3f} / {target:.3f} m")
        if self.route_distance_m >= target:
            self.route_stage += 1
            self._enter_route_stage()

    def request_balance_drive_speed(self):
        if self.balance_worker is None:
            self.log("请先连接主板 Wi-Fi 调试端口")
            return
        self._send_balance_control("DRIVE", self.spin_balance_drive_speed.value())

    def request_balance_drive_zero(self):
        if self.balance_worker is None:
            return
        self.spin_balance_drive_speed.setValue(0.0)
        self._send_balance_control("DRIVE", 0.0)

    def request_balance_turn_speed(self):
        if self.balance_worker is None:
            self.log("请先连接主板 Wi-Fi 调试端口")
            return
        if self.route_active:
            self.cancel_route()
        self._send_balance_control("TURN", self.spin_balance_turn_speed.value())

    def request_balance_turn_zero(self):
        if self.balance_worker is None:
            return
        if self.route_active:
            self.cancel_route()
        self.spin_balance_turn_speed.setValue(0.0)
        self._send_balance_control("TURN", 0.0)

    def _send_balance_control(self, action: str, value: float = None):
        self.balance_command_sequence += 1
        command = f"C,{self.balance_command_sequence},{action}"
        if value is not None:
            command += f",{value:.3f}"
        command += "\n"
        self.balance_worker.queue_command(command.encode("ascii"))
        suffix = "" if value is None else f" {value:.3f} m/s"
        self.log(f"已发送主板控制命令：{action}{suffix}")

    def on_balance_telemetry(self, telemetry: dict):
        sequence = telemetry["sequence"]
        board_timestamp_ms = telemetry["timestamp_ms"]
        previous_sequence = self.balance_last_telemetry_sequence
        previous_timestamp_ms = self.balance_last_board_timestamp_ms
        if previous_sequence is not None and sequence <= previous_sequence:
            # UDP can deliver a queued, older telemetry frame after a newer
            # one. Never let that stale frame overwrite confirmed tuning.
            # A real MCU restart is identified by its millisecond clock
            # returning near zero after the board had already been running.
            board_restarted = (
                previous_timestamp_ms is not None and previous_timestamp_ms > 5000
                and board_timestamp_ms < 2000
            )
            if not board_restarted:
                return
            self.log("检测到主板重启，已重新开始接收遥测序号")
            self.balance_pending_tuning.clear()
            self.balance_pending_sequences.clear()

        self.balance_last_telemetry_sequence = sequence
        self.balance_last_board_timestamp_ms = board_timestamp_ms
        self.balance_last_received_monotonic = time.monotonic()
        self._record_speed_telemetry(telemetry)
        if "rx_hz" in telemetry:
            self.balance_rx_hz = telemetry["rx_hz"]
        state_names = ["BOOT", "SELF_TESTING", "STANDBY", "MANUAL_TEST", "BALANCING", "FAULT"]
        fault_names = ["NONE", "SELF_TEST_FAILED", "IMU_UNHEALTHY", "PITCH_LIMIT_EXCEEDED",
                       "AIRBORNE_LANDING_FAILED"]
        state = state_names[telemetry["state"]] if 0 <= telemetry["state"] < len(state_names) else "UNKNOWN"
        fault = fault_names[telemetry["fault"]] if 0 <= telemetry["fault"] < len(fault_names) else "UNKNOWN"
        value = self.balance_value_labels
        value["state"].setText(state)
        value["fault"].setText(fault)
        value["imu"].setText("有效" if telemetry["imu_valid"] else "无效")
        value["pitch"].setText(f"{telemetry['pitch']:.3f}")
        value["pitch_rate"].setText(f"{telemetry['pitch_rate']:.3f}")
        value["accel_pitch"].setText(f"{telemetry['accel_pitch']:.3f}")
        value["accel"].setText(
            f"{telemetry['accel_x']:.3f}, {telemetry['accel_y']:.3f}, {telemetry['accel_z']:.3f}"
        )
        value["gyro"].setText(
            f"{telemetry['gyro_x']:.2f}, {telemetry['gyro_y']:.2f}, {telemetry['gyro_z']:.2f}"
        )
        value["speed"].setText(f"{telemetry['target_speed']:.3f} / {telemetry['filtered_speed']:.3f}")
        left_wheel = telemetry.get("wheel_left")
        right_wheel = telemetry.get("wheel_right")
        value["wheel_speed"].setText(
            "-" if left_wheel is None else f"{left_wheel:.3f}, {right_wheel:.3f}"
        )
        heading = telemetry.get("heading")
        yaw_rate = telemetry.get("yaw_rate")
        value["heading"].setText(
            "-" if heading is None or yaw_rate is None else f"{heading:.2f} / {yaw_rate:.2f}"
        )
        value["speed_error"].setText(f"{telemetry['speed_error']:.3f}")
        value["pitch_offset"].setText(f"{telemetry['pitch_offset']:.3f}")
        differential_speed = telemetry.get("differential_speed")
        turn_motor_command = telemetry.get("turn_motor_command")
        applied_turn_motor_command = telemetry.get("applied_turn_motor_command")
        if differential_speed is None or turn_motor_command is None or applied_turn_motor_command is None:
            value["turn"].setText(f"目标 {telemetry['turn']:.3f}")
        else:
            value["turn"].setText(
                f"{telemetry['turn']:.3f} / {differential_speed:.3f} m/s，"
                f"{turn_motor_command:.3f} / {applied_turn_motor_command:.3f}"
            )
        value["motor"].setText(f"{telemetry['motor_left']:.3f}, {telemetry['motor_right']:.3f}")
        if not self.balance_tuning_loaded:
            # Only the first complete telemetry frame initializes the edit
            # controls. Periodic telemetry is display data, not an authority
            # to overwrite an operator's pending/manual tuning values.
            for key in self.balance_tuning_spins:
                if telemetry.get(key) is not None:
                    self.balance_tuning_spins[key].setValue(telemetry[key])
            self.balance_tuning_loaded = True
            self.btn_apply_balance_loop_tuning.setEnabled(True)
            self.btn_apply_balance_tuning.setEnabled(True)
            self.log("已从主板读取当前 PID / PI 参数；现在可安全下发修改")

        self.lbl_balance_link.setText("已收到主板遥测")
        self.lbl_balance_link.setStyleSheet("color: #16803c;")
        self.btn_balance_arm.setEnabled(
            self.balance_worker is not None and state == "STANDBY" and bool(telemetry["imu_valid"])
        )
        self.btn_balance_stop.setEnabled(self.balance_worker is not None)
        drive_enabled = self.balance_worker is not None and state == "BALANCING"
        self.spin_balance_drive_speed.setEnabled(drive_enabled)
        self.btn_balance_drive.setEnabled(drive_enabled)
        self.btn_balance_drive_zero.setEnabled(drive_enabled)
        turn_enabled = drive_enabled and not self.route_active
        self.spin_balance_turn_speed.setEnabled(turn_enabled)
        self.btn_balance_turn.setEnabled(turn_enabled)
        self.btn_balance_turn_zero.setEnabled(turn_enabled)
        self.btn_route_start.setEnabled(drive_enabled and not self.route_active)
        self._update_route(telemetry, state)
        self._update_balance_packet_age(sequence)

    def on_balance_ack(self, ack: str):
        self.log(f"主板参数响应：{ack}")
        fields = ack.split(",", 3)
        if len(fields) < 4:
            return
        try:
            sequence = int(fields[1])
        except ValueError:
            return
        pending = self.balance_pending_sequences.pop(sequence, None)
        if pending is None:
            return
        key = pending["key"]
        if fields[2] != "OK":
            self.balance_pending_tuning.pop(key, None)
            self.log(f"[ERROR] 参数 {key} 被主板拒绝：{fields[3]}")
            self._finish_tuning_transaction()
            return

        result = fields[3].split(",")
        if (len(result) != 4 or result[0] != "APPLIED" or
                result[1] != pending["domain"] or result[2] != pending["parameter"]):
            self.balance_pending_tuning.pop(key, None)
            self.log(f"[ERROR] 参数 {key} 的 ACK 格式无效：{fields[3]}")
            self._finish_tuning_transaction()
            return
        try:
            actual_value = float(result[3])
        except ValueError:
            self.balance_pending_tuning.pop(key, None)
            self.log(f"[ERROR] 参数 {key} 的 ACK 数值无效：{fields[3]}")
            self._finish_tuning_transaction()
            return

        self.balance_pending_tuning.pop(key, None)
        self.balance_tuning_spins[key].setValue(actual_value)
        self.log(f"参数已由主板 ACK 确认：{key}={actual_value:.5f}")
        self._finish_tuning_transaction()

    def on_balance_command_failed(self, sequence: int, reason: str):
        pending = self.balance_pending_sequences.pop(sequence, None)
        if pending is not None:
            self.balance_pending_tuning.pop(pending["key"], None)
            self.log(f"[ERROR] 参数 {pending['key']} 下发超时，输入值已保留：{reason}")
            self._finish_tuning_transaction()

    def _finish_tuning_transaction(self):
        if self.balance_worker is None or self.balance_pending_sequences:
            return
        self.btn_apply_balance_loop_tuning.setEnabled(self.balance_tuning_loaded)
        self.btn_apply_balance_tuning.setEnabled(self.balance_tuning_loaded)

    def on_balance_console_line(self, line: str):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.balance_console_log.append(f"[{timestamp}] {line}")

    def _update_balance_packet_age(self, sequence=None):
        if self.balance_last_received_monotonic is None:
            return
        age_ms = (time.monotonic() - self.balance_last_received_monotonic) * 1000.0
        if sequence is None:
            previous = self.balance_value_labels["packet"].text().split(" / ")[0]
            sequence = previous if previous != "-" else "-"
        self.balance_value_labels["packet"].setText(
            f"{sequence} / {age_ms:.0f} ms / {self.balance_rx_hz:.1f} Hz"
        )
        if age_ms > 1000.0:
            self.lbl_balance_link.setText("遥测超时：请检查 Wi-Fi 与端口")
            self.lbl_balance_link.setStyleSheet("color: #c62828;")

    def _on_protocol_changed(self, index: int):
        is_http = (index in (0, 1))
        is_udp = (index == 2)
        self.port_spin.setEnabled(is_http)
        self.udp_local_port.setEnabled(is_udp)
        self.udp_cmd_port.setEnabled(is_udp)

    def log(self, msg: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_edit.append(f"[{timestamp}] {msg}")

    def base_url(self) -> str:
        ip = self.ip_edit.text().strip()
        port = self.port_spin.value()
        if port == 80:
            return f"http://{ip}"
        return f"http://{ip}:{port}"

    # -----------------------------------------------------------------------
    # 启动 / 停止
    # -----------------------------------------------------------------------
    def start_stream(self):
        self.stop_worker()
        protocol = self.cmb_protocol.currentIndex()

        if protocol == 0:
            url = self.base_url()
            self.log(f"Start HTTP MJPEG stream from {url}")
            self.worker = StreamWorker(url)

        elif protocol == 1:
            url = self.base_url()
            interval = self.spin_single_interval.value()
            self.log(f"Start HTTP single-frame from {url}, interval={interval}ms")
            self.worker = SingleFrameWorker(url, interval)

        else:
            local_port = self.udp_local_port.value()
            esp_ip = self.ip_edit.text().strip()
            cmd_port = self.udp_cmd_port.value()
            self.log(f"Start UDP stream: listen {local_port}, cmd {esp_ip}:{cmd_port}")
            self.worker = UdpStreamWorker(local_port, esp_ip, cmd_port)

        if isinstance(self.worker, UdpStreamWorker):
            self.worker.latest_frame_available.connect(self.on_udp_latest_frame)
        else:
            self.worker.frame_ready.connect(self.on_frame)
        self.worker.fps_updated.connect(self.on_fps)
        self.worker.info_updated.connect(self.log)
        self.worker.error_occurred.connect(self.on_error)
        if isinstance(self.worker, UdpStreamWorker):
            self.worker.stats_updated.connect(self.on_udp_stats)
        else:
            self.lbl_udp_stats.setText("UDP: -")
        self.worker.start()

        # UDP 模式下启动后自动把当前算法参数下发给 ESP32
        if isinstance(self.worker, UdpStreamWorker):
            self.apply_vision_params()

        self._set_running_ui(True)

    def stop_worker(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self._set_running_ui(False)
        self.lbl_udp_stats.setText("UDP: -")

    def _set_running_ui(self, running: bool):
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        self.cmb_protocol.setEnabled(not running)
        self.ip_edit.setEnabled(not running)
        self.port_spin.setEnabled(not running and self.cmb_protocol.currentIndex() in (0, 1))
        self.udp_local_port.setEnabled(not running and self.cmb_protocol.currentIndex() == 2)
        self.udp_cmd_port.setEnabled(not running and self.cmb_protocol.currentIndex() == 2)

    # -----------------------------------------------------------------------
    # 下发参数
    # -----------------------------------------------------------------------
    def apply_tx_params(self):
        protocol = self.cmb_protocol.currentIndex()
        quality = self.spin_quality.value()
        interval = self.spin_interval.value()
        stream_divider = self.spin_stream_divider.value()

        if protocol == 2 and isinstance(self.worker, UdpStreamWorker):
            self.worker.send_command(encode_udp_command(quality=quality))
            self.worker.send_command(encode_udp_command(interval_ms=interval))
            self.worker.send_command(encode_udp_command(stream_divider=stream_divider))
            self.log(f"Sent UDP cmd: Q={quality}, I={interval}, N={stream_divider}")
        else:
            self.log(
                "Note: JPEG quality / send interval are currently only sent via UDP. "
                "For HTTP mode, restart ESP32 after changing the source code."
            )

    def apply_vision_params(self):
        if not isinstance(self.worker, UdpStreamWorker):
            self.log(
                "Note: C++ vision params are currently only sent via UDP. "
                "Switch to UDP mode first."
            )
            return

        mode = self.cmb_vision_mode.currentIndex()  # 0=raw, 1=processed, 2=binary, 3=edge, 4=gray, 5=gray_raw
        threshold = self.spin_threshold.value()
        y1 = self.spin_roi_y1.value()
        y2 = self.spin_roi_y2.value()
        lookahead_y = self.spin_lookahead_y.value()
        wmin = self.spin_min_width.value()
        wmax = self.spin_max_width.value()
        contrast100 = self.spin_contrast.value()
        otsu = self.chk_otsu.isChecked()
        otsu_min = self.spin_otsu_min.value()
        otsu_max = self.spin_otsu_max.value()
        otsu_step = self.spin_otsu_step.value()
        otsu_alpha = self.spin_otsu_alpha.value()
        fg_min = self.spin_fg_min.value()
        fg_max = self.spin_fg_max.value()
        edge_thr = self.spin_edge_threshold.value()
        smooth = self.chk_smooth.isChecked()
        morph = self.chk_morph.isChecked()
        smooth_alpha = self.spin_smooth.value()
        row_gap = self.spin_row_gap.value()
        hold_frames = self.spin_hold_frames.value()

        self.worker.send_command(encode_udp_command(mode=mode))
        self.worker.send_command(encode_udp_command(threshold=threshold))
        self.worker.send_command(encode_udp_command(roi=(y1, y2)))
        self.worker.send_command(encode_udp_command(lookahead_y=lookahead_y))
        self.worker.send_command(encode_udp_command(line_width=(wmin, wmax)))
        self.worker.send_command(encode_udp_command(contrast100=contrast100))
        self.worker.send_command(encode_udp_command(otsu=otsu))
        self.worker.send_command(encode_udp_command(otsu_range=(otsu_min, otsu_max)))
        self.worker.send_command(encode_udp_command(otsu_max_step=otsu_step))
        self.worker.send_command(encode_udp_command(otsu_alpha=otsu_alpha))
        self.worker.send_command(encode_udp_command(foreground_range=(fg_min, fg_max)))
        self.worker.send_command(encode_udp_command(edge_threshold=edge_thr))
        self.worker.send_command(encode_udp_command(smooth_filter=smooth))
        self.worker.send_command(encode_udp_command(morph_clean=morph))
        self.worker.send_command(encode_udp_command(smooth_alpha=smooth_alpha))
        self.worker.send_command(encode_udp_command(max_row_gap=row_gap))
        self.worker.send_command(encode_udp_command(max_hold_frames=hold_frames))
        self.log(
            f"Sent UDP cmd: M={mode}, T={threshold}, R={y1},{y2}, Y={lookahead_y}, "
            f"W={wmin},{wmax}, G={contrast100}, O={int(otsu)}, "
            f"L={otsu_min},{otsu_max}, J={otsu_step}, A={otsu_alpha}, P={fg_min},{fg_max}, "
            f"E={edge_thr}, F={int(smooth)}, C={int(morph)}, S={smooth_alpha}, "
            f"D={row_gap}, H={hold_frames}"
        )

    # -----------------------------------------------------------------------
    # 图像与处理
    # -----------------------------------------------------------------------
    def on_udp_latest_frame(self):
        """从 UDP 工作线程取最新帧；旧帧已被覆盖，不会累积显示延迟。"""
        if not isinstance(self.worker, UdpStreamWorker):
            return
        img = self.worker.take_latest_frame()
        if img is not None:
            self.on_frame(img)

    def on_frame(self, img: np.ndarray):
        self.last_frame = img
        self.lbl_resolution.setText(f"Resolution: {img.shape[1]}x{img.shape[0]}")
        self.lbl_frame_size.setText(f"Frame size: {img.nbytes // 1024} KB")

        # 实时画面：显示 ESP32 回传的画面（原图 或 C++ 处理后的效果）
        self._show_image(img, self.live_label)

        # 算法处理页：同样显示该画面，并叠加文字说明
        mode_name = self.cmb_vision_mode.currentText()
        annotated = img.copy()
        cv2.putText(
            annotated,
            f"Mode: {mode_name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        self._show_image(annotated, self.proc_label)

    def on_fps(self, fps: float):
        self.lbl_fps.setText(f"FPS: {fps:.1f}")
        self.statusBar().showMessage(f"Running | FPS: {fps:.1f}")

    def on_error(self, msg: str):
        self.log(f"[ERROR] {msg}")
        self.statusBar().showMessage(f"Error: {msg}")

    def on_udp_stats(self, msg: str):
        self.lbl_udp_stats.setText(f"UDP: {msg}")

    def save_snapshot(self):
        if self.last_frame is None:
            QMessageBox.warning(self, "警告", "没有可保存的画面")
            return

        default_name = datetime.now().strftime("snapshot_%Y%m%d_%H%M%S.jpg")
        path, _ = QFileDialog.getSaveFileName(
            self, "保存快照", default_name, "Images (*.jpg *.png)"
        )
        if path:
            cv2.imwrite(path, self.last_frame)
            self.log(f"Snapshot saved: {path}")

    @staticmethod
    def _show_image(img: np.ndarray, label: QLabel):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(qimg)
        scaled = pixmap.scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        label.setPixmap(scaled)

    def closeEvent(self, event):
        self.stop_worker()
        self.stop_balance_debug()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("ESP32-S3 Balance Car Host")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
