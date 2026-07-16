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
from calibration.calibration_core import (
    find_corners, load_intrinsic_result, render_checkerboard, save_intrinsic_result,
    load_track_alignment, save_flat_validation, save_track_alignment,
    solve_intrinsics, solve_manual_flat_plane, write_cpp_header, TrackAlignment,
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox, QTabWidget,
    QTextEdit, QGroupBox, QGridLayout, QFileDialog, QMessageBox, QDialog,
    QSplitter, QComboBox, QCheckBox, QScrollArea
)


# ---------------------------------------------------------------------------
# UDP 协议常量（与 ESP32 端保持一致）
# ---------------------------------------------------------------------------
UDP_MAGIC = 0x55445043          # "UDPC"
UDP_HEADER_FMT = "<IIHHHH"      # magic, frame_id, pkt_id, pkt_cnt, payload_len, reserved
UDP_HEADER_LEN = struct.calcsize(UDP_HEADER_FMT)
MAX_GUI_FRAME_RATE_HZ = 30.0
CSV_FLUSH_INTERVAL_SECONDS = 1.0
CSV_FLUSH_ROW_LIMIT = 50
MAIN_LOG_MAX_BLOCKS = 1000
TOOL_DIRECTORY = Path(__file__).resolve().parent
PROJECT_ROOT = TOOL_DIRECTORY.parent
CALIBRATION_OUTPUT_DIRECTORY = TOOL_DIRECTORY / "calibration" / "output"


def encode_udp_command(quality: int = None, interval_ms: int = None, stream_divider: int = None,
                        mode: int = None, threshold: int = None,
                        roi: tuple = None, lookahead_y: int = None, line_width: tuple = None,
                        contrast100: int = None, otsu: bool = None,
                        otsu_range: tuple = None, otsu_max_step: int = None,
                        otsu_alpha: int = None, foreground_range: tuple = None,
                        edge_threshold: int = None, smooth_filter: bool = None,
                        morph_clean: bool = None, smooth_alpha: int = None,
                        max_row_gap: int = None, max_hold_frames: int = None,
                        tracking_tuning: tuple = None) -> bytes:
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
    if tracking_tuning is not None:
        return (f"K={tracking_tuning[0]},{tracking_tuning[1]},"
                f"{tracking_tuning[2]},{tracking_tuning[3]}\n").encode("ascii")
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
        # Do not use a Qt signal for every decoded frame.  Signals crossing
        # threads are queued, so a GUI that is briefly busy would otherwise
        # retain every old image and become slower forever.
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._last_error_text = ""
        self._last_error_monotonic = 0.0

    def _publish_latest_frame(self, img: np.ndarray):
        with self._frame_lock:
            self._latest_frame = img

    def take_latest_frame(self):
        with self._frame_lock:
            img = self._latest_frame
            self._latest_frame = None
            return img

    def _report_error(self, text: str):
        now = time.monotonic()
        if text == self._last_error_text and now - self._last_error_monotonic < 1.0:
            return
        self._last_error_text = text
        self._last_error_monotonic = now
        self.error_occurred.emit(text)

    def stop(self):
        self.running = False
        self.wait(1000)

    def run(self):
        self.running = True
        self.info_updated.emit(f"Connecting to {self.stream_url} ...")

        try:
            resp = requests.get(self.stream_url, stream=True, timeout=5.0)
            if resp.status_code != 200:
                self._report_error(f"HTTP {resp.status_code}")
                return
        except Exception as e:
            self._report_error(f"Connect failed: {e}")
            return

        self.info_updated.emit("Stream connected")

        buffer = b""
        frame_count = 0
        last_time = time.monotonic()
        last_decode_time = 0.0

        for chunk in resp.iter_content(chunk_size=4096):
            if not self.running:
                break

            buffer += chunk
            # A malformed/partial MJPEG stream must not grow the receive
            # buffer without a bound and eventually consume all RAM.
            if len(buffer) > 4 * 1024 * 1024:
                buffer = buffer[-512 * 1024:]

            while self.running:
                soi = buffer.find(b"\xff\xd8")
                eoi = buffer.find(b"\xff\xd9")

                if soi != -1 and eoi != -1 and eoi > soi:
                    jpeg_data = buffer[soi:eoi + 2]
                    buffer = buffer[eoi + 2:]

                    now = time.monotonic()
                    # The GUI renders at 30 Hz.  Decoding every faster input
                    # frame only builds a queued backlog and makes the whole
                    # desktop sluggish, so discard frames the GUI cannot use.
                    if now - last_decode_time < 1.0 / MAX_GUI_FRAME_RATE_HZ:
                        continue
                    last_decode_time = now

                    img = self._decode_jpeg(jpeg_data)
                    if img is not None:
                        self._publish_latest_frame(img)
                        frame_count += 1

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
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._last_error_text = ""
        self._last_error_monotonic = 0.0

    def _publish_latest_frame(self, img: np.ndarray):
        with self._frame_lock:
            self._latest_frame = img

    def take_latest_frame(self):
        with self._frame_lock:
            img = self._latest_frame
            self._latest_frame = None
            return img

    def _report_error(self, text: str):
        now = time.monotonic()
        if text == self._last_error_text and now - self._last_error_monotonic < 1.0:
            return
        self._last_error_text = text
        self._last_error_monotonic = now
        self.error_occurred.emit(text)

    def stop(self):
        self.running = False
        self.wait(1000)

    def run(self):
        self.running = True
        frame_count = 0
        last_time = time.monotonic()
        last_info_time = 0.0

        while self.running:
            t0 = time.monotonic()
            try:
                resp = requests.get(self.frame_url, timeout=2.0)
                if resp.status_code == 200:
                    img = self._decode_jpeg(resp.content)
                    if img is not None:
                        self._publish_latest_frame(img)
                        frame_count += 1

                        now = time.monotonic()
                        if now - last_time >= 1.0:
                            fps = frame_count / (now - last_time)
                            self.fps_updated.emit(fps)
                            frame_count = 0
                            last_time = now

                        if now - last_info_time >= 1.0:
                            self.info_updated.emit(
                                f"Frame: {len(resp.content)} bytes, {img.shape[1]}x{img.shape[0]}"
                            )
                            last_info_time = now
                    else:
                        self._report_error("Decode JPEG failed")
                else:
                    self._report_error(f"HTTP {resp.status_code}")
            except Exception as e:
                self._report_error(f"Request failed: {e}")

            elapsed = (time.monotonic() - t0) * 1000
            # A single-frame HTTP request is far more expensive than a normal
            # draw operation.  Do not allow it to flood the GUI above 30 Hz.
            sleep_ms = max(int(1000.0 / MAX_GUI_FRAME_RATE_HZ), self.interval_ms - elapsed)
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
        self._stop_event = threading.Event()
        self.cmd_socket = None
        self._receive_socket = None
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._init_cmd_socket()

    def _publish_latest_frame(self, img: np.ndarray):
        """只保留最新解码帧，避免 GUI 事件队列积压造成显示延迟。"""
        with self._frame_lock:
            self._latest_frame = img

    def take_latest_frame(self):
        with self._frame_lock:
            img = self._latest_frame
            self._latest_frame = None
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
        self._stop_event.set()
        if self._receive_socket is not None:
            try:
                self._receive_socket.close()
            except OSError:
                pass
        self.wait(1500)
        if self.cmd_socket is not None:
            try:
                self.cmd_socket.close()
            except Exception:
                pass
            self.cmd_socket = None

    def run(self):
        self.running = True

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.05)
            self._receive_socket = sock
        except Exception as e:
            self.error_occurred.emit(f"UDP bind failed: {e}")
            return

        self.info_updated.emit(f"Listening UDP on 0.0.0.0:{self.local_port}")

        current_fid = None
        expected_cnt = None
        packets = {}
        first_seen = 0.0
        last_completed_fid = None
        decoded_frames = 0
        dropped_frames = 0
        malformed_packets = 0
        last_stats_time = time.monotonic()
        last_fps_time = last_stats_time
        fps_frames = 0
        last_frame_info_time = 0.0
        last_decode_time = 0.0
        skipped_decode_frames = 0

        while self.running and not self._stop_event.is_set():
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                # 检查当前帧是否超时未收齐
                if current_fid is not None and len(packets) < expected_cnt:
                    if time.monotonic() - first_seen > 0.075:
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
            if pcnt == 0 or pid >= pcnt or plen > len(data) - UDP_HEADER_LEN:
                malformed_packets += 1
                continue

            payload = data[UDP_HEADER_LEN:UDP_HEADER_LEN + plen]
            if len(payload) != plen:
                continue

            # 新帧号到达，丢弃旧帧
            if fid != current_fid:
                if last_completed_fid is not None:
                    delta_from_completed = (fid - last_completed_fid) & 0xFFFFFFFF
                    if delta_from_completed == 0 or delta_from_completed >= 0x80000000:
                        continue
                if current_fid is not None:
                    # Newer frame IDs are accepted; delayed old frames never
                    # replace the frame that is currently being assembled.
                    if ((fid - current_fid) & 0xFFFFFFFF) >= 0x80000000:
                        continue
                    if len(packets) < expected_cnt:
                        dropped_frames += 1
                current_fid = fid
                expected_cnt = pcnt
                packets = {pid: payload}
                first_seen = time.monotonic()
            else:
                if pcnt != expected_cnt:
                    malformed_packets += 1
                    continue
                packets[pid] = payload

            # 收齐一包
            if len(packets) == expected_cnt and all(index in packets for index in range(expected_cnt)):
                jpeg = b"".join(packets[i] for i in range(expected_cnt))
                completed_fid = current_fid
                current_fid = None
                expected_cnt = None
                packets.clear()

                now = time.monotonic()
                # A completed UDP image is useful only when the GUI can show
                # it.  JPEG decoding is CPU-heavy, so drop surplus complete
                # frames before decoding rather than creating a CPU backlog.
                last_completed_fid = completed_fid
                if now - last_decode_time < 1.0 / MAX_GUI_FRAME_RATE_HZ:
                    skipped_decode_frames += 1
                    continue
                last_decode_time = now
                img = self._decode_jpeg(jpeg)
                if img is not None:
                    decoded_frames += 1
                    fps_frames += 1
                    self._publish_latest_frame(img)
                    wall_now = time.time()
                    if wall_now - last_frame_info_time >= 1.0:
                        self.info_updated.emit(
                            f"UDP latest: {len(jpeg)} bytes, {img.shape[1]}x{img.shape[0]}, "
                            f"packets {pcnt}"
                        )
                        last_frame_info_time = wall_now

            # 每秒刷新一次统计
            now = time.monotonic()
            if now - last_stats_time >= 1.0:
                total = decoded_frames + dropped_frames
                loss = (dropped_frames / total * 100.0) if total > 0 else 0.0
                elapsed = now - last_fps_time
                self.fps_updated.emit(fps_frames / elapsed if elapsed > 0 else 0.0)
                fps_frames = 0
                last_fps_time = now
                self.stats_updated.emit(
                    f"decoded={decoded_frames} skipped={skipped_decode_frames} "
                    f"dropped={dropped_frames} bad={malformed_packets} loss={loss:.1f}%"
                )
                # 不重置计数，保持累计
                last_stats_time = now

        try:
            sock.close()
        except Exception:
            pass
        self._receive_socket = None
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
        # Telemetry is display/logging data, not a control input.  Keep only
        # the newest packet so temporary GUI stalls cannot create an unbounded
        # queued-signal backlog.
        self._telemetry_lock = threading.Lock()
        self._latest_telemetry = None

    def _publish_latest_telemetry(self, telemetry: dict):
        with self._telemetry_lock:
            self._latest_telemetry = telemetry

    def take_latest_telemetry(self):
        with self._telemetry_lock:
            telemetry = self._latest_telemetry
            self._latest_telemetry = None
            return telemetry

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
            self.info_updated.emit("主板调试 UDP 已停止")

    @staticmethod
    def _parse_telemetry(text: str):
        parts = text.split(",")
        if parts[0:2] not in (["T", "1"], ["T", "2"], ["T", "3"], ["T", "5"], ["T", "6"], ["T", "7"], ["T", "8"], ["T", "9"], ["T", "10"]):
            return None
        expected_fields = {"1": 31, "2": 33, "3": 40, "5": 50, "6": 53, "7": 54, "8": 55, "9": 57, "10": 64}[parts[1]]
        if len(parts) != expected_fields:
            return None
        try:
            values = [float(value) for value in parts[7:]]
            # T,7 retains the extended v5/v6 fields and appends the vision
            # hand-off status.  Treating it as the old compact layout makes
            # the UI display the target only and hides the real turn feedback.
            rich = parts[1] in ("5", "6", "7", "8", "9", "10")
            return {
                "telemetry_version": int(parts[1]),
                "sequence": int(parts[2]), "timestamp_ms": int(parts[3]),
                "state": int(parts[4]), "fault": int(parts[5]), "imu_valid": int(parts[6]),
                "pitch": values[0], "pitch_rate": values[1], "accel_pitch": values[2],
                "accel_x": values[3], "accel_y": values[4], "accel_z": values[5],
                "gyro_x": values[6], "gyro_y": values[7], "gyro_z": values[8],
                "target_speed": values[9], "filtered_speed": values[10], "speed_error": values[11],
                "pitch_offset": values[12],
                "turn": values[13],
                "motor_left": values[18] if rich else values[14],
                "motor_right": values[19] if rich else values[15],
                "balance_kp": values[20] if rich else values[16], "balance_ki": values[21] if rich else values[17],
                "balance_kd": values[22] if rich else values[18], "balance_trim": values[23] if rich else values[19],
                "speed_kp": values[24] if rich else values[20], "speed_ki": values[25] if rich else values[21],
                "max_motor": values[26] if rich else values[22], "max_pitch": values[27] if rich else values[23],
                "wheel_left": values[28] if rich else (values[24] if parts[1] in ("2", "3") else None),
                "wheel_right": values[29] if rich else (values[25] if parts[1] in ("2", "3") else None),
                "requested_pitch": values[30] if rich else (values[26] if parts[1] == "3" else None),
                "balance_pitch_error": values[31] if rich else (values[27] if parts[1] == "3" else None),
                "balance_p_term": values[32] if rich else (values[28] if parts[1] == "3" else None),
                "balance_i_term": values[33] if rich else (values[29] if parts[1] == "3" else None),
                "balance_d_term": values[34] if rich else (values[30] if parts[1] == "3" else None),
                "balance_motor_raw": values[35] if rich else (values[31] if parts[1] == "3" else None),
                "speed_invert": int(values[36]) if rich else (int(values[32]) if parts[1] == "3" else None),
                "diff_speed": values[14] if rich else None,
                "diff_error": values[15] if rich else None,
                "turn_motor": values[16] if rich else None,
                "turn_applied": values[17] if rich else None,
                "turn_kp": values[37] if rich else None,
                "turn_ki": values[38] if rich else None,
                "turn_max": values[39] if rich else None,
                "vision_tracking": bool(int(values[43])) if parts[1] in ("6", "7", "8", "9", "10") else None,
                "vision_fresh": bool(int(values[44])) if parts[1] in ("6", "7", "8", "9", "10") else None,
                "vision_accepted": bool(int(values[45])) if parts[1] in ("7", "8", "9", "10") else None,
                "vision_dv": values[46] if parts[1] in ("7", "8", "9", "10") else (values[45] if parts[1] == "6" else None),
                "vision_period": values[47] if parts[1] in ("8", "9", "10") else None,
                "vision_filter": bool(int(values[48])) if parts[1] in ("9", "10") else None,
                "vision_max_step": values[49] if parts[1] in ("9", "10") else None,
                "balance_saturated": bool(int(values[50])) if parts[1] == "10" else None,
                "speed_saturated": bool(int(values[51])) if parts[1] == "10" else None,
                "turn_saturated": bool(int(values[52])) if parts[1] == "10" else None,
                "encoder_valid": bool(int(values[53])) if parts[1] == "10" else None,
                "imu_calibrated": bool(int(values[54])) if parts[1] == "10" else None,
                "balance_period_ms": values[55] if parts[1] == "10" else None,
                "velocity_period_ms": values[56] if parts[1] == "10" else None,
            }
        except ValueError:
            return None


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class CalibrationImageLabel(QLabel):
    """Aspect-ratio-safe image widget that returns clicks in source pixels."""
    image_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._source_size = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(480, 300)
        self.setStyleSheet("background:#111; border:1px solid #888;")

    def set_source_size(self, width: int, height: int):
        self._source_size = (width, height)

    def mousePressEvent(self, event):
        if self._source_size is None:
            return
        source_w, source_h = self._source_size
        scale = min(self.width() / source_w, self.height() / source_h)
        shown_w, shown_h = source_w * scale, source_h * scale
        left, top = (self.width() - shown_w) * 0.5, (self.height() - shown_h) * 0.5
        x, y = event.pos().x(), event.pos().y()
        if left <= x <= left + shown_w and top <= y <= top + shown_h:
            self.image_clicked.emit((x - left) / scale, (y - top) / scale)


class VisionI2cDebugDialog(QDialog):
    """Dedicated view for the board's mirrored [I2C] diagnostics."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("循迹 I2C 状态")
        self.resize(760, 430)
        layout = QVBoxLayout(self)
        self.summary = QLabel("等待主板 [I2C] 状态；新版固件每 0.1 s 更新。")
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(300)
        layout.addWidget(self.log_view)
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._csv_rows_since_flush = 0
        self._csv_last_flush_monotonic = 0.0
        self._start_csv_log()
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self.log_view.clear)
        layout.addWidget(clear_button)

    def append_line(self, line: str):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.append(f"[{timestamp}] {line}")
        self._write_csv_log(timestamp, line)
        self.summary.setText(
            "dv=相机原始目标，conditioned_dv=限频滤波缓存，cmd_dv=当前真正写入差速环的目标，"
            "measured_dv=编码器实测右轮减左轮；单位均为 mm/s。"
        )

    def _start_csv_log(self):
        try:
            log_dir = Path(__file__).resolve().parent / "log"
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self._csv_path = log_dir / f"vision_i2c_{stamp}.csv"
            self._csv_file = self._csv_path.open("w", encoding="utf-8-sig", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "track_valid", "camera_delta_speed_mps",
                "applied_delta_speed_mps", "left_wheel_speed_mps",
                "right_wheel_speed_mps", "measured_delta_speed_mps",
                "vehicle_speed_mps", "target_speed_mps",
            ])
            self._csv_rows_since_flush = 0
            self._csv_last_flush_monotonic = time.monotonic()
        except OSError:
            self._csv_file = self._csv_writer = self._csv_path = None

    def _flush_csv_if_due(self, force: bool = False):
        if self._csv_file is None or self._csv_rows_since_flush == 0:
            return
        now = time.monotonic()
        if not force and self._csv_rows_since_flush < CSV_FLUSH_ROW_LIMIT and \
                now - self._csv_last_flush_monotonic < CSV_FLUSH_INTERVAL_SECONDS:
            return
        self._csv_file.flush()
        self._csv_rows_since_flush = 0
        self._csv_last_flush_monotonic = now

    def _write_csv_log(self, timestamp: str, line: str):
        if self._csv_writer is None or self._csv_file is None:
            return
        fields = {}
        for token in line.replace("[", "").replace("]", "").split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        try:
            def mmps_to_mps(name: str):
                try:
                    return f"{float(fields[name]) / 1000.0:.6f}"
                except (KeyError, ValueError):
                    return ""

            self._csv_writer.writerow([
                timestamp,
                fields.get("valid", ""),
                mmps_to_mps("dv"),
                mmps_to_mps("cmd_dv"),
                fields.get("vl", ""),
                fields.get("vr", ""),
                mmps_to_mps("measured_dv"),
                fields.get("vavg", ""),
                fields.get("vtarget", ""),
            ])
            self._csv_rows_since_flush += 1
            self._flush_csv_if_due()
        except OSError:
            pass

    def closeEvent(self, event):
        if self._csv_file is not None:
            try:
                self._flush_csv_if_due(force=True)
                self._csv_file.close()
            except OSError:
                pass
            self._csv_file = self._csv_writer = None
        if self.parent() is not None and hasattr(self.parent(), "vision_i2c_debug_dialog"):
            self.parent().vision_i2c_debug_dialog = None
        super().closeEvent(event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-S3 平衡小车上位机")
        self.setGeometry(100, 100, 1300, 850)

        self.worker = None
        self.last_frame = None
        self._pending_display_frame = None
        self._stream_generation = 0
        self._display_timer = QTimer(self)
        self._display_timer.setInterval(33)  # Render at most 30 Hz; never queue stale frames.
        self._display_timer.timeout.connect(self._render_latest_display_frame)
        self._last_calibration_preview_monotonic = 0.0
        self.intrinsic_samples = []
        self.intrinsic_result = None
        self.track_alignment = None
        self.calibration_frozen_frame = None
        self.calibration_points = []
        self.calibration_mode = "line"

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
        self._build_calibration_tab()
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
        self.spin_interval.setValue(0)
        self.spin_interval.setToolTip("每帧编码后让出 CPU 的时间，0~20 即可")
        tx_grid.addWidget(self.spin_interval, 0, 3)

        tx_grid.addWidget(QLabel("图传间隔帧:"), 1, 0)
        self.spin_stream_divider = QSpinBox()
        self.spin_stream_divider.setRange(0, 30)
        self.spin_stream_divider.setValue(1)
        self.spin_stream_divider.setToolTip("0=关闭调试图传；1=每个视觉帧回传；2=每两帧回传一次")
        tx_grid.addWidget(self.spin_stream_divider, 1, 1)

        self.btn_apply_tx = QPushButton("应用并下发")
        self.btn_apply_tx.clicked.connect(self.apply_tx_params)
        tx_grid.addWidget(self.btn_apply_tx, 0, 4)

        tx_grid.addWidget(QLabel("单帧间隔 ms:"), 2, 0)
        self.spin_single_interval = QSpinBox()
        self.spin_single_interval.setRange(int(1000.0 / MAX_GUI_FRAME_RATE_HZ), 2000)
        self.spin_single_interval.setValue(100)
        self.spin_single_interval.setToolTip("下限为 33 ms（30 Hz），避免 HTTP 请求和 GUI 队列长期积压。")
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
        self.spin_row_gap.setValue(8)
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
        self.spin_lookahead_y.setToolTip("优先前瞻行；弯道到不了该行时会自动退到最远可靠可见点")
        roi_grid.addWidget(self.spin_lookahead_y, 8, 1)

        roi_grid.addWidget(QLabel("ROI 上边界:"), 9, 0)
        self.spin_roi_y1 = QSpinBox()
        self.spin_roi_y1.setRange(0, 480)
        self.spin_roi_y1.setValue(4)
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
        self.spin_min_width.setToolTip("短视野远端仍保留 2 像素下限，避免漏掉弯道边缘黑线")
        roi_grid.addWidget(self.spin_min_width, 10, 1)

        roi_grid.addWidget(QLabel("最大线宽:"), 10, 2)
        self.spin_max_width = QSpinBox()
        self.spin_max_width.setRange(0, 320)
        self.spin_max_width.setValue(64)
        self.spin_max_width.setToolTip("过滤纸外阴影等大片黑区；若近端黑线被截断再增大")
        roi_grid.addWidget(self.spin_max_width, 10, 3)

        roi_grid.addWidget(QLabel("循迹横向增益 K_e x100:"), 11, 0)
        self.spin_track_lateral_gain = QSpinBox()
        self.spin_track_lateral_gain.setRange(0, 400)
        self.spin_track_lateral_gain.setValue(180)
        roi_grid.addWidget(self.spin_track_lateral_gain, 11, 1)

        roi_grid.addWidget(QLabel("循迹航向增益 Kθ x100:"), 11, 2)
        self.spin_track_heading_gain = QSpinBox()
        self.spin_track_heading_gain.setRange(0, 400)
        self.spin_track_heading_gain.setValue(200)
        roi_grid.addWidget(self.spin_track_heading_gain, 11, 3)

        roi_grid.addWidget(QLabel("目标转差限幅 (mm/s):"), 12, 0)
        self.spin_track_max_delta = QSpinBox()
        self.spin_track_max_delta.setRange(0, 200)
        self.spin_track_max_delta.setValue(120)
        roi_grid.addWidget(self.spin_track_max_delta, 12, 1)

        roi_grid.addWidget(QLabel("实测转差不足补偿 x100:"), 12, 2)
        self.spin_track_speed_feedback = QSpinBox()
        self.spin_track_speed_feedback.setRange(0, 200)
        self.spin_track_speed_feedback.setValue(75)
        self.spin_track_speed_feedback.setToolTip(
            "按视觉目标与编码器实测转差之差提高给定；0=关闭，75=补偿0.75倍")
        roi_grid.addWidget(self.spin_track_speed_feedback, 12, 3)

        self.btn_apply_vision = QPushButton("下发算法参数")
        self.btn_apply_vision.setToolTip("将阈值、ROI、线宽、流模式、CLAHE 对比度增强、Otsu、边缘阈值、平滑滤波、形态学清理、平滑系数下发到 ESP32")
        self.btn_apply_vision.clicked.connect(self.apply_vision_params)
        roi_grid.addWidget(self.btn_apply_vision, 13, 0, 1, 4)

        roi_grid.setRowStretch(14, 1)
        param_layout.addWidget(roi_group)

        body_splitter.addWidget(param_widget)
        body_splitter.setSizes([900, 350])
        main_layout.addWidget(body_splitter)

        # ---- 日志区 ----
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumHeight(180)
        self.log_edit.document().setMaximumBlockCount(MAIN_LOG_MAX_BLOCKS)
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
        self.balance_render_timer = QTimer(self)
        self.balance_render_timer.setInterval(50)
        self.balance_render_timer.timeout.connect(self._drain_balance_telemetry)
        # This page contains several control rows.  Keep their natural height
        # in full-screen/small-height windows instead of letting QGridLayout
        # compress rows until widgets overlap.
        self.balance_tab = QScrollArea()
        self.balance_tab.setWidgetResizable(True)
        balance_content = QWidget()
        self.balance_tab.setWidget(balance_content)
        layout = QVBoxLayout(balance_content)

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
        self.spin_balance_drive_speed.setRange(0.0, 0.600)
        self.spin_balance_drive_speed.setSingleStep(0.010)
        self.spin_balance_drive_speed.setValue(0.0)
        self.spin_balance_drive_speed.setToolTip(
            "上位机输入上限为 0.600 m/s；实际可接受范围仍由当前固件的安全限幅决定。"
        )
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

        self.btn_vision_tracking = QPushButton("启用摄像头循迹")
        self.btn_vision_tracking.clicked.connect(self.toggle_vision_tracking)
        self.btn_vision_tracking.setEnabled(False)
        self.btn_vision_tracking.setToolTip("启用后，将相机 I2C 的目标左右轮速度差送给主板差速环；手动 TURN 会自动关闭。")
        connection_grid.addWidget(self.btn_vision_tracking, 4, 0, 1, 3)
        self.btn_vision_i2c_debug = QPushButton("查看循迹 I2C 状态")
        self.btn_vision_i2c_debug.clicked.connect(self.show_vision_i2c_debug)
        connection_grid.addWidget(self.btn_vision_i2c_debug, 4, 3, 1, 3)

        connection_grid.addWidget(QLabel("自动路线:"), 5, 0)
        self.cmb_route_direction = QComboBox()
        self.cmb_route_direction.addItems(["左转（两次半圆）", "右转（两次半圆）"])
        connection_grid.addWidget(self.cmb_route_direction, 5, 1, 1, 2)
        self.spin_route_turn = QDoubleSpinBox()
        self.spin_route_turn.setRange(0.01, 0.20)
        self.spin_route_turn.setDecimals(3)
        self.spin_route_turn.setSingleStep(0.01)
        self.spin_route_turn.setValue(0.060)
        self.spin_route_turn.setToolTip("转向混控量，需通过实际半径校准；轮距 0.20 m、目标半径 0.25 m")
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
            ("pitch_offset", "速度环俯角输出 (°)"),
            ("motor", "左右电机输出"), ("packet", "包序号 / 包龄 / 频率"),
        ]
        for index, (key, title) in enumerate(status_fields):
            row, column = divmod(index, 3)
            live_grid.addWidget(QLabel(f"{title}:"), row, column * 2)
            value_label = QLabel("-")
            value_label.setMinimumWidth(145)
            self.balance_value_labels[key] = value_label
            live_grid.addWidget(value_label, row, column * 2 + 1)
        # The vision hand-off chain is intentionally shown in one full-width
        # row. A normal 145 px status cell would truncate the important
        # camera/accepted/target/measured/applied values.
        turn_row = (len(status_fields) + 2) // 3
        live_grid.addWidget(QLabel("视觉差速闭环 (m/s):"), turn_row, 0)
        turn_label = QLabel("-")
        turn_label.setWordWrap(True)
        turn_label.setMinimumHeight(42)
        turn_label.setMinimumWidth(520)
        turn_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.balance_value_labels["turn"] = turn_label
        live_grid.addWidget(turn_label, turn_row, 1, 1, 5)
        layout.addWidget(live_group)

        tuning_group = QGroupBox("在线 PID / PI 参数（仅调参，不含启动、急停、速度或转向命令）")
        tuning_grid = QGridLayout(tuning_group)
        self.balance_tuning_spins = {}
        tuning_fields = [
            ("balance_kp", "角度 Kp", 0.15, 5),
            ("balance_ki", "角度 Ki", 0.002, 5),
            ("balance_kd", "角度 Kd", 0.003, 5),
            ("balance_trim", "平衡点 Trim (°)", -1.19, 3),
            ("speed_kp", "速度 Kp", 14.0, 5),
            ("speed_ki", "速度 Ki", 0.003, 6),
            ("max_motor", "最大电机输出 (0–1)", 0.45, 3),
            ("max_pitch", "最大俯仰偏置 (°)", 6.0, 2),
            ("turn_kp", "转差环 Kp", 1.1, 4),
            ("turn_ki", "转差环 Ki", 0.001, 4),
            ("turn_max", "最大转向输出 (0–1)", 0.20, 3),
            ("vision_period", "视觉转差更新间隔 (ms)", 400.0, 0),
            ("vision_max_step", "单次转差最大变化 (mm/s, 0=不限)", 0.0, 0),
            ("vision_curve_hold_mmps", "弯道锁存测试转差 (mm/s)", 120.0, 0),
        ]
        for index, (key, title, value, decimals) in enumerate(tuning_fields):
            row, column = divmod(index, 3)
            tuning_grid.addWidget(QLabel(title + ":"), row, column * 2)
            spin = QDoubleSpinBox()
            spin.setDecimals(decimals)
            if key == "balance_trim":
                spin.setRange(-20.0, 20.0)
                spin.setSingleStep(0.05)
            elif key in ("max_motor", "turn_max"):
                spin.setRange(0.0, 1.0)
                spin.setSingleStep(0.01)
            elif key == "turn_kp":
                spin.setRange(0.0, 10.0)
                spin.setSingleStep(0.1)
            elif key == "turn_ki":
                spin.setRange(0.0, 2.0)
                spin.setSingleStep(0.01)
            elif key == "vision_period":
                # 仅限制相机转差给定的更新频率，不影响 I2C 50 Hz 状态交换。
                # 默认 400 ms；I2C 仍以 50 Hz 向加权滑动窗口提供样本。
                spin.setRange(100.0, 5000.0)
                spin.setSingleStep(100.0)
            elif key == "vision_max_step":
                spin.setRange(0.0, 200.0)
                spin.setSingleStep(5.0)
            elif key == "vision_curve_hold_mmps":
                spin.setRange(20.0, 200.0)
                spin.setSingleStep(10.0)
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
        self.chk_vision_filter = QCheckBox("启用视觉转差加权滑动滤波")
        self.chk_vision_filter.setChecked(True)
        self.chk_vision_filter.setToolTip("最近5个有效相机dv按1:2:3:4:5加权，最新样本权重最高；关闭后采用最新dv")
        tuning_grid.addWidget(self.chk_vision_filter, 5, 0, 1, 3)
        self.chk_vision_curve_hold = QCheckBox("弯道失线后保持最大转差")
        self.chk_vision_curve_hold.setChecked(True)
        self.chk_vision_curve_hold.setToolTip(
            "连续两帧确认方向后，valid=0 时按该方向保持最大转差；valid=1 后立即恢复视觉给定")
        tuning_grid.addWidget(self.chk_vision_curve_hold, 5, 3, 1, 3)
        tuning_grid.addWidget(self.btn_apply_balance_loop_tuning, 6, 0, 1, 3)

        self.btn_apply_balance_tuning = QPushButton("应用全部参数（含速度 / 转差环）")
        self.btn_apply_balance_tuning.clicked.connect(self.apply_balance_tuning)
        self.btn_apply_balance_tuning.setEnabled(False)
        tuning_grid.addWidget(self.btn_apply_balance_tuning, 6, 3, 1, 3)
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

    def _build_calibration_tab(self):
        """Manual calibration page.  Every click is made on one frozen raw frame."""
        self.calibration_tab = QWidget()
        layout = QVBoxLayout(self.calibration_tab)
        intro = QLabel(
            "当前页直接显示相机原始图。先请求 M=0 原始图，再定格。黑线双点只校正像素零点；棋盘 A-F 用于平地物理验证，不用于坡道控制。"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        preview_group = QGroupBox("标定画面（点选前必须定格）")
        preview_layout = QVBoxLayout(preview_group)
        self.calib_preview = CalibrationImageLabel()
        self.calib_preview.image_clicked.connect(self._on_calibration_image_clicked)
        preview_layout.addWidget(self.calib_preview)
        preview_buttons = QHBoxLayout()
        self.btn_request_raw = QPushButton("请求原始画靪 M=0")
        self.btn_request_raw.clicked.connect(self.request_calibration_raw_frame)
        self.btn_freeze_calib = QPushButton("定格当前画面")
        self.btn_freeze_calib.clicked.connect(self.freeze_calibration_frame)
        self.btn_resume_calib = QPushButton("恢复实时画面")
        self.btn_resume_calib.clicked.connect(self.resume_calibration_frame)
        self.btn_undo_calib = QPushButton("撤销点")
        self.btn_undo_calib.clicked.connect(self.undo_calibration_point)
        self.btn_clear_calib = QPushButton("清空点")
        self.btn_clear_calib.clicked.connect(self.clear_calibration_points)
        self.lbl_calib_frame = QLabel("等待相机图传…")
        for button in (self.btn_request_raw, self.btn_freeze_calib, self.btn_resume_calib,
                       self.btn_undo_calib, self.btn_clear_calib):
            preview_buttons.addWidget(button)
        preview_buttons.addWidget(self.lbl_calib_frame, 1)
        preview_layout.addLayout(preview_buttons)
        layout.addWidget(preview_group)

        settings = QGroupBox("棋盘参数（与手机/平板上实际显示的格宽一致）")
        grid = QGridLayout(settings)
        grid.addWidget(QLabel("内角点列数:"), 0, 0)
        self.calib_cols = QSpinBox(); self.calib_cols.setRange(3, 20); self.calib_cols.setValue(9)
        grid.addWidget(self.calib_cols, 0, 1)
        grid.addWidget(QLabel("内角点行数:"), 0, 2)
        self.calib_rows = QSpinBox(); self.calib_rows.setRange(3, 20); self.calib_rows.setValue(6)
        grid.addWidget(self.calib_rows, 0, 3)
        grid.addWidget(QLabel("实际格宽 (mm):"), 0, 4)
        self.calib_square_mm = QDoubleSpinBox(); self.calib_square_mm.setRange(1.0, 200.0)
        self.calib_square_mm.setValue(20.0); self.calib_square_mm.setDecimals(2)
        grid.addWidget(self.calib_square_mm, 0, 5)
        self.btn_export_pattern = QPushButton("导出手机/平板棋盘 PNG")
        self.btn_export_pattern.clicked.connect(self.export_calibration_pattern)
        grid.addWidget(self.btn_export_pattern, 0, 6)
        layout.addWidget(settings)

        intrinsic = QGroupBox("步骤 1：相机内参（每张样本都先定格）")
        igrid = QGridLayout(intrinsic)
        self.btn_capture_intrinsic = QPushButton("采集当前原始画面")
        self.btn_capture_intrinsic.clicked.connect(self.capture_intrinsic_sample)
        igrid.addWidget(self.btn_capture_intrinsic, 0, 0)
        self.btn_solve_intrinsic = QPushButton("计算内参（至少 20 张）")
        self.btn_solve_intrinsic.clicked.connect(self.solve_intrinsic_calibration)
        igrid.addWidget(self.btn_solve_intrinsic, 0, 1)
        self.lbl_intrinsic_status = QLabel("尚未采集样本")
        self.lbl_intrinsic_status.setWordWrap(True)
        igrid.addWidget(self.lbl_intrinsic_status, 1, 0, 1, 2)
        layout.addWidget(intrinsic)

        line = QGroupBox("步骤 2：人工黑线零点对齐（不需测距离）")
        lgrid = QGridLayout(line)
        self.btn_line_mode = QPushButton("选择黑线双点")
        self.btn_line_mode.clicked.connect(lambda: self._set_calibration_mode("line"))
        self.btn_save_alignment = QPushButton("保存零点并生成固件配置")
        self.btn_save_alignment.clicked.connect(self.save_manual_track_alignment)
        lgrid.addWidget(self.btn_line_mode, 0, 0)
        lgrid.addWidget(self.btn_save_alignment, 0, 1)
        lgrid.addWidget(QLabel("K_e:"), 0, 2)
        self.spin_align_ke = QDoubleSpinBox(); self.spin_align_ke.setRange(0.0, 2.0); self.spin_align_ke.setDecimals(3); self.spin_align_ke.setValue(0.40)
        lgrid.addWidget(self.spin_align_ke, 0, 3)
        lgrid.addWidget(QLabel("K_θ:"), 0, 4)
        self.spin_align_kh = QDoubleSpinBox(); self.spin_align_kh.setRange(0.0, 2.0); self.spin_align_kh.setDecimals(3); self.spin_align_kh.setValue(0.60)
        lgrid.addWidget(self.spin_align_kh, 0, 5)
        self.lbl_line_status = QLabel("先定格，然后点击远端辅助线与黑线中心的交点，再点击近端交点。")
        self.lbl_line_status.setWordWrap(True)
        lgrid.addWidget(self.lbl_line_status, 1, 0, 1, 6)
        layout.addWidget(line)

        ground = QGroupBox("步骤 3：人工平地物理验证（不用于循迹控制）")
        ggrid = QGridLayout(ground)
        ggrid.addWidget(QLabel("A 内角点 X (m，右正):"), 0, 0)
        self.ground_x = QDoubleSpinBox(); self.ground_x.setRange(-5.0, 5.0); self.ground_x.setDecimals(3)
        ggrid.addWidget(self.ground_x, 0, 1)
        ggrid.addWidget(QLabel("A 内角点 Y (m，前正):"), 0, 2)
        self.ground_y = QDoubleSpinBox(); self.ground_y.setRange(-1.0, 10.0); self.ground_y.setDecimals(3)
        ggrid.addWidget(self.ground_y, 0, 3)
        fixed_direction = QLabel("固定摆放：屏幕顶边朝车头，A 在左上角（无需测角）")
        fixed_direction.setWordWrap(True)
        ggrid.addWidget(fixed_direction, 0, 4, 1, 2)
        self.btn_flat_mode = QPushButton("选择棋盘 A-F 点选")
        self.btn_flat_mode.clicked.connect(lambda: self._set_calibration_mode("flat"))
        ggrid.addWidget(self.btn_flat_mode, 1, 0, 1, 3)
        self.btn_solve_ground = QPushButton("验证并保存平地结果")
        self.btn_solve_ground.clicked.connect(self.solve_manual_flat_validation)
        ggrid.addWidget(self.btn_solve_ground, 1, 3, 1, 3)
        self.lbl_ground_status = QLabel(
            "定格后按 A→B→C→D→E→F 点选棋盘内角点。A-D 必须是相邻一格的四个角（顺时针），"
            "E/F 选其右侧相邻两角作验证；需实测 A 的 X/Y 与格宽。结果仅作平地验证，不参与控制。"
        )
        self.lbl_ground_status.setWordWrap(True)
        ggrid.addWidget(self.lbl_ground_status, 2, 0, 1, 6)
        layout.addWidget(ground)
        layout.addStretch()
        self.tabs.addTab(self.calibration_tab, "相机标定")
        self._restore_intrinsic_calibration()
        self._restore_track_alignment()

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
                min-height: 34px;
                min-width: 108px;
                max-width: 300px;
                padding: 7px 14px;
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
        self.balance_worker.ack_received.connect(self.on_balance_ack)
        self.balance_worker.command_failed.connect(self.on_balance_command_failed)
        self.balance_worker.console_line_received.connect(self.on_balance_console_line)
        self.balance_worker.info_updated.connect(self.log)
        self.balance_worker.error_occurred.connect(self.on_error)
        self.balance_worker.start()
        self.balance_age_timer.start()
        self.balance_render_timer.start()
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
        if hasattr(self, "balance_render_timer"):
            self.balance_render_timer.stop()
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
                "target_differential_speed_mps", "measured_differential_speed_mps",
                "differential_speed_error_mps", "turn_motor_command", "applied_turn_motor_command",
                "vision_tracking_enabled", "vision_sample_fresh", "vision_command_accepted", "camera_delta_speed_mps",
                "balance_inner_saturated", "velocity_loop_saturated", "turn_loop_saturated",
                "encoder_valid", "imu_calibrated", "balance_period_ms", "velocity_period_ms",
            ])
            self._speed_record_rows_since_flush = 0
            self._speed_record_last_flush_monotonic = time.monotonic()
            self.log(f"速度记录已开始：{self.speed_record_path}")
        except OSError as error:
            self.speed_record_file = None
            self.speed_record_writer = None
            self.speed_record_path = None
            self.log(f"[ERROR] 无法创建速度记录文件：{error}")

    def _flush_speed_record_if_due(self, force: bool = False):
        if self.speed_record_file is None or self._speed_record_rows_since_flush == 0:
            return
        now = time.monotonic()
        if not force and self._speed_record_rows_since_flush < CSV_FLUSH_ROW_LIMIT and \
                now - self._speed_record_last_flush_monotonic < CSV_FLUSH_INTERVAL_SECONDS:
            return
        self.speed_record_file.flush()
        self._speed_record_rows_since_flush = 0
        self._speed_record_last_flush_monotonic = now

    def _stop_speed_recording(self):
        if self.speed_record_file is None:
            return
        record_path = self.speed_record_path
        try:
            self._flush_speed_record_if_due(force=True)
            self.speed_record_file.close()
            self.log(f"速度记录已保存：{record_path}")
        except OSError as error:
            self.log(f"[ERROR] 关闭速度记录文件失败：{error}")
        finally:
            self.speed_record_file = None
            self.speed_record_writer = None
            self.speed_record_path = None
            self._speed_record_rows_since_flush = 0
            self._speed_record_last_flush_monotonic = 0.0

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
                optional_float("turn"), optional_float("diff_speed"), optional_float("diff_error"),
                optional_float("turn_motor"), optional_float("turn_applied"),
                "" if telemetry.get("vision_tracking") is None else int(telemetry["vision_tracking"]),
                "" if telemetry.get("vision_fresh") is None else int(telemetry["vision_fresh"]),
                "" if telemetry.get("vision_accepted") is None else int(telemetry["vision_accepted"]),
                optional_float("vision_dv"),
                "" if telemetry.get("balance_saturated") is None else int(telemetry["balance_saturated"]),
                "" if telemetry.get("speed_saturated") is None else int(telemetry["speed_saturated"]),
                "" if telemetry.get("turn_saturated") is None else int(telemetry["turn_saturated"]),
                "" if telemetry.get("encoder_valid") is None else int(telemetry["encoder_valid"]),
                "" if telemetry.get("imu_calibrated") is None else int(telemetry["imu_calibrated"]),
                optional_float("balance_period_ms"), optional_float("velocity_period_ms"),
            ])
            self._speed_record_rows_since_flush += 1
            self._flush_speed_record_if_due()
        except (OSError, KeyError) as error:
            self.log(f"[ERROR] 写入速度记录失败：{error}")
            self._stop_speed_recording()

    def apply_balance_tuning(self):
        command_fields = [
            ("balance", "kp", "balance_kp"), ("balance", "ki", "balance_ki"),
            ("balance", "kd", "balance_kd"), ("balance", "trim", "balance_trim"),
            ("speed", "kp", "speed_kp"), ("speed", "ki", "speed_ki"),
            ("balance", "max_motor", "max_motor"), ("speed", "max_pitch", "max_pitch"),
            ("turn", "kp", "turn_kp"), ("turn", "ki", "turn_ki"),
            ("turn", "max", "turn_max"),
            ("vision", "period_ms", "vision_period"),
            ("vision", "max_step_mmps", "vision_max_step"),
            ("vision", "filter", "vision_filter"),
            ("vision", "curve_hold_mmps", "vision_curve_hold_mmps"),
            ("vision", "curve_hold", "vision_curve_hold"),
        ]
        self._send_tuning_commands(
            command_fields, "已发送平衡、速度、转差环参数与输出限幅，等待主板 ACK 与遥测回读"
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

    def _apply_connection_default_pid_group(self):
        """每次连接确认首包遥测后，下发用户指定的三环基准参数。"""
        defaults = {
            "balance_kp": 0.15,
            "balance_ki": 0.002,
            "balance_trim": -1.19,
            "speed_kp": 14.0,
            "speed_ki": 0.003,
            "turn_kp": 1.1,
            "turn_ki": 0.001,
        }
        for key, value in defaults.items():
            self.balance_tuning_spins[key].setValue(value)
        self._send_tuning_commands([
            ("balance", "kp", "balance_kp"),
            ("balance", "ki", "balance_ki"),
            ("balance", "trim", "balance_trim"),
            ("speed", "kp", "speed_kp"),
            ("speed", "ki", "speed_ki"),
            ("turn", "kp", "turn_kp"),
            ("turn", "ki", "turn_ki"),
        ], "已自动下发连接基准参数组（7 项），等待主板 ACK 与遥测回读")

    def _send_tuning_commands(self, command_fields, success_message: str):
        if self.balance_worker is None or not self.balance_tuning_loaded:
            self.log("请先连接主板并等待首次参数遥测完成")
            return
        self.btn_apply_balance_loop_tuning.setEnabled(False)
        self.btn_apply_balance_tuning.setEnabled(False)
        for domain, parameter, key in command_fields:
            self.balance_command_sequence += 1
            if key == "vision_filter":
                value = 1.0 if self.chk_vision_filter.isChecked() else 0.0
            elif key == "vision_curve_hold":
                value = 1.0 if self.chk_vision_curve_hold.isChecked() else 0.0
            else:
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

    def toggle_vision_tracking(self):
        if self.balance_worker is None:
            return
        enabled = self.btn_vision_tracking.property("tracking_enabled") is not True
        self._send_balance_control("TRACK", 1 if enabled else 0)
        self.btn_vision_tracking.setProperty("tracking_enabled", enabled)
        self.btn_vision_tracking.setText("关闭摄像头循迹" if enabled else "启用摄像头循迹")

    def show_vision_i2c_debug(self):
        if getattr(self, "vision_i2c_debug_dialog", None) is None:
            self.vision_i2c_debug_dialog = VisionI2cDebugDialog(self)
        self.vision_i2c_debug_dialog.show()
        self.vision_i2c_debug_dialog.raise_()
        self.vision_i2c_debug_dialog.activateWindow()

    def _send_balance_control(self, action: str, value: float = None):
        self.balance_command_sequence += 1
        command = f"C,{self.balance_command_sequence},{action}"
        if value is not None:
            # TRACK is deliberately a Boolean wire field, not a floating
            # value.  The mainboard protocol accepts exactly `0` or `1`.
            if action.upper() == "TRACK":
                command += ",1" if float(value) >= 0.5 else ",0"
            else:
                command += f",{value:.3f}"
        command += "\n"
        self.balance_worker.queue_command(command.encode("ascii"))
        if value is None:
            suffix = ""
        elif action.upper() == "TRACK":
            suffix = " 1" if float(value) >= 0.5 else " 0"
        else:
            suffix = f" {value:.3f} m/s"
        self.log(f"已发送主板控制命令：{action}{suffix}（线协议：{command.strip()}）")

    def _drain_balance_telemetry(self):
        """Render only the newest worker packet at a bounded GUI rate."""
        worker = self.balance_worker
        if worker is None:
            return
        telemetry = worker.take_latest_telemetry()
        if telemetry is not None:
            self.on_balance_telemetry(telemetry)

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
        fault_names = ["NONE", "SELF_TEST_FAILED", "IMU_UNHEALTHY", "PITCH_LIMIT_EXCEEDED"]
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
        value["speed_error"].setText(f"{telemetry['speed_error']:.3f}")
        value["pitch_offset"].setText(f"{telemetry['pitch_offset']:.3f}")
        if telemetry.get("vision_dv") is not None:
            accepted = telemetry.get("vision_accepted")
            accepted_text = "—" if accepted is None else str(int(accepted))
            value["turn"].setText(
                f"相机 Δv {telemetry['vision_dv']:.3f} / 接管 {accepted_text} / "
                f"主板目标 Δv {telemetry['turn']:.3f}\n"
                f"实测 Δv {telemetry['diff_speed']:.3f} / 差速环输出 {telemetry['turn_motor']:.3f} / "
                f"混控已施加 {telemetry['turn_applied']:.3f}"
            )
        elif telemetry.get("telemetry_version", 0) >= 5:
            value["turn"].setText(
                f"主板遥测 T{telemetry['telemetry_version']} 未回传相机接管状态；请烧录支持 T7 的主板固件。"
            )
        elif telemetry.get("diff_speed") is None:
            value["turn"].setText(f"{telemetry['turn']:.3f}")
        else:
            value["turn"].setText(
                f"目标 Δv {telemetry['turn']:.3f} / 实测 Δv {telemetry['diff_speed']:.3f} / "
                f"差速环输出 {telemetry['turn_motor']:.3f} / 混控已施加 {telemetry['turn_applied']:.3f}"
            )
        value["motor"].setText(f"{telemetry['motor_left']:.3f}, {telemetry['motor_right']:.3f}")
        if not self.balance_tuning_loaded:
            # Only the first complete telemetry frame initializes the edit
            # controls. Periodic telemetry is display data, not an authority
            # to overwrite an operator's pending/manual tuning values.
            for key in self.balance_tuning_spins:
                if telemetry.get(key) is not None:
                    self.balance_tuning_spins[key].setValue(telemetry[key])
            if telemetry.get("vision_filter") is not None:
                self.chk_vision_filter.setChecked(telemetry["vision_filter"])
            self.balance_tuning_loaded = True
            self.btn_apply_balance_loop_tuning.setEnabled(True)
            self.btn_apply_balance_tuning.setEnabled(True)
            self.log("已从主板读取当前 PID / PI 参数；现在可安全下发修改")
            self._apply_connection_default_pid_group()

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
        tracking_reported = telemetry.get("vision_tracking")
        tracking_enabled = (self.btn_vision_tracking.property("tracking_enabled") is True
                            if tracking_reported is None else bool(tracking_reported))
        self.btn_vision_tracking.setEnabled(drive_enabled)
        self.btn_vision_tracking.setProperty("tracking_enabled", tracking_enabled)
        self.btn_vision_tracking.setText("关闭摄像头循迹" if tracking_enabled else "启用摄像头循迹")
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
        if key == "vision_filter":
            self.chk_vision_filter.setChecked(actual_value >= 0.5)
        elif key == "vision_curve_hold":
            self.chk_vision_curve_hold.setChecked(actual_value >= 0.5)
        else:
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
        if line.startswith("[I2C]") and getattr(self, "vision_i2c_debug_dialog", None) is not None:
            # I²C CSV logging is opt-in through the dedicated dialog.  Do not
            # silently create a window and keep flushing a file in the
            # background merely because the board emits diagnostics.
            self.vision_i2c_debug_dialog.append_line(line)

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
        # Calibration restore happens while the UI is being built, before the
        # visible log widget exists.  Never turn a recoverable old-file error
        # into a startup failure.
        if hasattr(self, "log_edit"):
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
        self._stream_generation += 1
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

        worker = self.worker
        # Every callback is tied to this exact worker.  Decoded image frames
        # are deliberately *not* sent through Qt signals: the display timer
        # pulls one newest frame from the worker, preventing event-queue
        # accumulation after a transient slow repaint.
        worker.fps_updated.connect(lambda fps, source=worker: self._on_worker_fps(source, fps))
        worker.info_updated.connect(lambda text, source=worker: self._on_worker_info(source, text))
        worker.error_occurred.connect(lambda text, source=worker: self._on_worker_error(source, text))
        if isinstance(self.worker, UdpStreamWorker):
            self.worker.stats_updated.connect(
                lambda text, source=worker: self._on_worker_udp_stats(source, text))
        else:
            self.lbl_udp_stats.setText("UDP: -")
        self.worker.start()
        self._display_timer.start()

        # UDP 模式下启动后自动把当前算法参数下发给 ESP32
        if isinstance(self.worker, UdpStreamWorker):
            self.apply_vision_params()

        self._set_running_ui(True)

    def stop_worker(self):
        old_worker = self.worker
        self.worker = None
        self._pending_display_frame = None
        self._display_timer.stop()
        if old_worker is not None:
            old_worker.stop()
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
        track_lateral_gain = self.spin_track_lateral_gain.value()
        track_heading_gain = self.spin_track_heading_gain.value()
        track_max_delta = self.spin_track_max_delta.value()
        track_speed_feedback = self.spin_track_speed_feedback.value()

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
        self.worker.send_command(encode_udp_command(
            tracking_tuning=(track_lateral_gain, track_heading_gain,
                             track_max_delta, track_speed_feedback)))
        self.log(
            f"Sent UDP cmd: M={mode}, T={threshold}, R={y1},{y2}, Y={lookahead_y}, "
            f"W={wmin},{wmax}, G={contrast100}, O={int(otsu)}, "
            f"L={otsu_min},{otsu_max}, J={otsu_step}, A={otsu_alpha}, P={fg_min},{fg_max}, "
            f"E={edge_thr}, F={int(smooth)}, C={int(morph)}, S={smooth_alpha}, "
            f"D={row_gap}, H={hold_frames}, "
            f"K={track_lateral_gain},{track_heading_gain},{track_max_delta},{track_speed_feedback}"
        )

    # -----------------------------------------------------------------------
    # 图像与处理
    # -----------------------------------------------------------------------
    def _raw_calibration_frame(self):
        if self.calibration_frozen_frame is None:
            QMessageBox.warning(self, "标定", "请先在本页点击“定格当前画面”。")
            return None
        return self.calibration_frozen_frame.copy()

    def request_calibration_raw_frame(self):
        if self.last_frame is None:
            QMessageBox.warning(self, "标定", "请先启动相机图传并获得一帧画面。")
            return None
        if self.cmb_vision_mode.currentIndex() != 0:
            self.cmb_vision_mode.setCurrentIndex(0)
            self.apply_vision_params()
        self.resume_calibration_frame()
        self.log("已请求 M=0 原始图；等待标定预览更新后点击定格。")

    def freeze_calibration_frame(self):
        if self.last_frame is None:
            QMessageBox.warning(self, "标定", "没有可定格的相机画面。")
            return
        if self.cmb_vision_mode.currentIndex() != 0:
            QMessageBox.warning(self, "标定", "请先点击“请求原始画靪 M=0”，等待新画面后再定格。")
            return
        self.calibration_frozen_frame = self.last_frame.copy()
        self.calibration_points.clear()
        self.lbl_calib_frame.setText(f"已定格 {self.last_frame.shape[1]}x{self.last_frame.shape[0]}")
        self._refresh_calibration_preview()

    def resume_calibration_frame(self):
        self.calibration_frozen_frame = None
        self.calibration_points.clear()
        self.lbl_calib_frame.setText("实时画面（未定格）")
        self._refresh_calibration_preview()

    def undo_calibration_point(self):
        if self.calibration_points:
            self.calibration_points.pop()
            self._refresh_calibration_preview()

    def clear_calibration_points(self):
        self.calibration_points.clear()
        self._refresh_calibration_preview()

    def _set_calibration_mode(self, mode: str):
        self.calibration_mode = mode
        self.calibration_points.clear()
        message = "黑线模式：请依次点远端、近端黑线中心。" if mode == "line" else "平地模式：请依次点 A→B→C→D→E→F。"
        self.lbl_line_status.setText(message) if mode == "line" else self.lbl_ground_status.setText(message)
        self._refresh_calibration_preview()

    def _calibration_guide_rows(self):
        top, bottom = self.spin_roi_y1.value(), self.spin_roi_y2.value()
        height = max(1, bottom - top)
        # The far point must be above the default lookahead row (112), so the
        # reference is interpolated rather than extrapolated at the target.
        return int(round(top + 0.25 * height)), int(round(top + 0.80 * height))

    def _on_calibration_image_clicked(self, x: float, y: float):
        if self.calibration_frozen_frame is None:
            QMessageBox.information(self, "标定", "请先定格画面，再进行手动点选。")
            return
        required = 2 if self.calibration_mode == "line" else 6
        if len(self.calibration_points) >= required:
            return
        if self.calibration_mode == "line":
            far_y, near_y = self._calibration_guide_rows()
            expected_y = far_y if not self.calibration_points else near_y
            if abs(y - expected_y) > 14:
                QMessageBox.warning(self, "黑线零点", f"请点击距离当前辅助线 14 像素以内的黑线中心（y={expected_y}）。")
                return
            y = float(expected_y)
        self.calibration_points.append((float(x), float(y)))
        if self.calibration_mode == "line":
            names = ("远端", "近端")
            self.lbl_line_status.setText(f"已选 {names[len(self.calibration_points)-1]} {x:.1f}, {y:.1f}。")
        else:
            names = "ABCDEF"
            self.lbl_ground_status.setText(f"已选 {names[len(self.calibration_points)-1]} {x:.1f}, {y:.1f}。")
        self._refresh_calibration_preview()

    def _refresh_calibration_preview(self):
        image = self.calibration_frozen_frame if self.calibration_frozen_frame is not None else self.last_frame
        if image is None or not hasattr(self, "calib_preview"):
            return
        annotated = image.copy()
        if self.calibration_frozen_frame is not None:
            cv2.putText(annotated, "FROZEN", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if self.calibration_mode == "line":
                far_y, near_y = self._calibration_guide_rows()
                for name, row in (("FAR", far_y), ("NEAR", near_y)):
                    cv2.line(annotated, (0, row), (annotated.shape[1] - 1, row), (0, 220, 220), 1)
                    cv2.putText(annotated, name, (5, max(14, row - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)
        labels = ("远", "近") if self.calibration_mode == "line" else tuple("ABCDEF")
        for index, point in enumerate(self.calibration_points):
            px, py = int(round(point[0])), int(round(point[1]))
            cv2.circle(annotated, (px, py), 5, (0, 0, 255), -1)
            cv2.putText(annotated, labels[index], (px + 7, py - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        if len(self.calibration_points) >= 2 and self.calibration_mode == "line":
            cv2.line(annotated, tuple(map(lambda a: int(round(a)), self.calibration_points[0])),
                     tuple(map(lambda a: int(round(a)), self.calibration_points[1])), (0, 255, 0), 2)
        self.calib_preview.set_source_size(annotated.shape[1], annotated.shape[0])
        self._show_image(annotated, self.calib_preview)

    def export_calibration_pattern(self):
        image = render_checkerboard(self.calib_cols.value(), self.calib_rows.value())
        default = str(Path(__file__).resolve().parent / "calibration" / "checkerboard.png")
        path, _ = QFileDialog.getSaveFileName(self, "导出棋盘图案", default, "PNG 图片 (*.png)")
        if not path:
            return
        if cv2.imwrite(path, image):
            self.log("已导出棋盘图。请在手机/平板全屏显示，并用直尺测量一个格子的实际边长后填入本页。")
        else:
            QMessageBox.critical(self, "标定", "棋盘图保存失败。")

    def capture_intrinsic_sample(self):
        image = self._raw_calibration_frame()
        if image is None:
            return
        corners = find_corners(image, self.calib_cols.value(), self.calib_rows.value())
        if corners is None:
            self.lbl_intrinsic_status.setText("未识别到完整棋盘：请保证图案清晰、完整并避免反光。")
            return
        self.intrinsic_samples.append(image)
        self.lbl_intrinsic_status.setText(
            f"已接受 {len(self.intrinsic_samples)} 张；请覆盖画面中心、四角和不同倾角，至少 20 张。")

    def solve_intrinsic_calibration(self):
        try:
            result = solve_intrinsics(self.intrinsic_samples, self.calib_cols.value(),
                                      self.calib_rows.value(), self.calib_square_mm.value())
        except ValueError as exc:
            QMessageBox.warning(self, "内参标定", str(exc))
            return
        self.intrinsic_result = result
        save_intrinsic_result(
            CALIBRATION_OUTPUT_DIRECTORY / "intrinsic_calibration.json", result)
        message = (f"内参完成：重投影误差 {result.reprojection_error_px:.3f} px，"
                   f"分辨率 {result.image_size[0]}x{result.image_size[1]}。")
        if result.reprojection_error_px > 1.0:
            message += " 超过 1 px，不会允许生成固件标定头文件；请补充更分散的样本。"
        self.lbl_intrinsic_status.setText(message + " 已保存到 intrinsic_calibration.json。")
        self.log(message + " 内参已保存；关闭上位机不会丢失。")

    def _restore_intrinsic_calibration(self):
        path = CALIBRATION_OUTPUT_DIRECTORY / "intrinsic_calibration.json"
        if not path.exists():
            return
        try:
            self.intrinsic_result = load_intrinsic_result(path)
            result = self.intrinsic_result
            self.lbl_intrinsic_status.setText(
                f"已恢复保存的内参：{result.image_size[0]}x{result.image_size[1]}，"
                f"重投影误差 {result.reprojection_error_px:.3f} px。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.log(f"[ERROR] 无法读取保存的内参：{exc}")

    def _restore_track_alignment(self):
        path = CALIBRATION_OUTPUT_DIRECTORY / "track_alignment.json"
        if not path.exists():
            return
        try:
            self.track_alignment = load_track_alignment(path)
            result = self.track_alignment
            self.spin_align_ke.setValue(result.gain_lateral)
            self.spin_align_kh.setValue(result.gain_heading)
            self.lbl_line_status.setText(
                f"已恢复黑线零点：x={result.x_zero:.1f}px，θ={result.theta_zero_deg:.2f}°，"
                f"{result.image_size[0]}x{result.image_size[1]}。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.log(f"[ERROR] 无法读取保存的黑线零点：{exc}")

    def save_manual_track_alignment(self):
        if self.intrinsic_result is None or self.intrinsic_result.reprojection_error_px > 1.0:
            QMessageBox.warning(self, "黑线零点", "请先完成且通过 1 px 内参校验。")
            return
        if self.calibration_mode != "line" or len(self.calibration_points) != 2 or self.calibration_frozen_frame is None:
            QMessageBox.warning(self, "黑线零点", "请定格后依次选择远端、近端两点。")
            return
        far, near = self.calibration_points
        if near[1] <= far[1] + 5:
            QMessageBox.warning(self, "黑线零点", "近端点必须位于远端点下方。")
            return
        lookahead = self.spin_lookahead_y.value()
        x_zero = far[0] + (near[0] - far[0]) * (lookahead - far[1]) / (near[1] - far[1])
        theta_zero = np.degrees(np.arctan2(near[0] - far[0], near[1] - far[1]))
        frame = self.calibration_frozen_frame
        self.track_alignment = TrackAlignment((frame.shape[1], frame.shape[0]), self.spin_roi_y1.value(),
                                              self.spin_roi_y2.value(), lookahead, x_zero, theta_zero,
                                              self.spin_align_ke.value(), self.spin_align_kh.value(), 0.08)
        root = PROJECT_ROOT
        save_track_alignment(CALIBRATION_OUTPUT_DIRECTORY / "track_alignment.json", self.track_alignment)
        try:
            write_cpp_header(root / "include" / "config" / "vision_calibration.h", self.intrinsic_result, self.track_alignment)
        except ValueError as exc:
            QMessageBox.warning(self, "黑线零点", str(exc)); return
        self.lbl_line_status.setText(f"已保存：x_zero={x_zero:.2f}px，theta_zero={theta_zero:.2f}°。请重新编译烧录相机。")
        self.log("已保存 track_alignment.json 并生成 include/config/vision_calibration.h。")

    def solve_manual_flat_validation(self):
        if self.intrinsic_result is None or self.intrinsic_result.reprojection_error_px > 1.0:
            QMessageBox.warning(self, "平地验证", "请先完成且通过 1 px 内参校验。"); return
        if self.calibration_mode != "flat" or len(self.calibration_points) != 6:
            QMessageBox.warning(self, "平地验证", "请定格后依次选择 A→B→C→D→E→F 六个内角点。"); return
        try:
            h, errors = solve_manual_flat_plane(np.asarray(self.calibration_points[:4]), np.asarray(self.calibration_points[4:]),
                                                self.intrinsic_result, self.calib_cols.value(), self.calib_rows.value(),
                                                self.calib_square_mm.value(), self.ground_x.value(), self.ground_y.value())
        except (ValueError, cv2.error) as exc:
            QMessageBox.warning(self, "平地验证", str(exc)); return
        save_flat_validation(CALIBRATION_OUTPUT_DIRECTORY / "flat_plane_validation.json",
                             self.intrinsic_result, h, errors, self.calib_cols.value(), self.calib_rows.value(),
                             self.calib_square_mm.value(), self.ground_x.value(), self.ground_y.value())
        self.lbl_ground_status.setText(f"E/F 验证误差：{errors[0]*1000:.1f} / {errors[1]*1000:.1f} mm（仅保存验证，不参与循迹）。")
        self.log("已保存 flat_plane_validation.json；它不会更改相机循迹固件配置。")

    def _queue_worker_frame(self, source, img: np.ndarray):
        if source is self.worker:
            self._pending_display_frame = img

    def _on_worker_fps(self, source, fps: float):
        if source is self.worker:
            self.on_fps(fps)

    def _on_worker_info(self, source, text: str):
        if source is self.worker:
            self.log(text)

    def _on_worker_error(self, source, text: str):
        if source is self.worker:
            self.on_error(text)

    def _on_worker_udp_stats(self, source, text: str):
        if source is self.worker:
            self.on_udp_stats(text)

    def _render_latest_display_frame(self):
        worker = self.worker
        if worker is not None and hasattr(worker, "take_latest_frame"):
            image = worker.take_latest_frame()
            if image is not None:
                self._pending_display_frame = image
        image = self._pending_display_frame
        self._pending_display_frame = None
        if image is not None:
            self._render_frame(image)

    def on_udp_latest_frame(self):
        # Kept for compatibility with older signal connections.
        self._render_latest_display_frame()

    def on_frame(self, img: np.ndarray):
        """Compatibility entry point: queue rather than rendering in a worker signal."""
        self._pending_display_frame = img

    def _render_frame(self, img: np.ndarray):
        self.last_frame = img
        self.lbl_resolution.setText(f"Resolution: {img.shape[1]}x{img.shape[0]}")
        self.lbl_frame_size.setText(f"Frame size: {img.nbytes // 1024} KB")

        # Calibration preview needs another full image conversion and scale.
        # It is unnecessary while another tab is visible, and 5 Hz is ample
        # for manually placing calibration points.
        now = time.monotonic()
        if (hasattr(self, "calib_preview") and self.calibration_frozen_frame is None
                and self.tabs.currentWidget() is self.calibration_tab
                and now - self._last_calibration_preview_monotonic >= 0.20):
            self._last_calibration_preview_monotonic = now
            self._refresh_calibration_preview()

        # Rendering only the visible image tab halves conversion/scaling load.
        if self.tabs.currentWidget() is self.proc_tab:
            annotated = img.copy()
            cv2.putText(annotated, f"Mode: {self.cmb_vision_mode.currentText()}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self._show_image(annotated, self.proc_label)
        elif self.tabs.currentWidget() is self.live_tab:
            self._show_image(img, self.live_label)

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
            label.size(), Qt.KeepAspectRatio, Qt.FastTransformation
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
