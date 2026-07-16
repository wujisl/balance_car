"""Self-contained camera-calibration helpers for the desktop host.

The host must be runnable directly with ``python tool/host_pyqt.py``.  Keep
the calibration implementation local to the repository rather than relying on
an untracked ``calibration`` package from a developer's machine.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class IntrinsicCalibration:
    image_size: Tuple[int, int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    reprojection_error_px: float
    square_size_mm: float
    valid_sample_count: int


@dataclass
class TrackAlignment:
    image_size: Tuple[int, int]
    roi_y1: int
    roi_y2: int
    lookahead_y: int
    x_zero: float
    theta_zero_deg: float
    gain_lateral: float
    gain_heading: float
    speed_feedback: float


def _as_path(path: Path | str) -> Path:
    result = Path(path)
    result.parent.mkdir(parents=True, exist_ok=True)
    return result


def _image_points(image: np.ndarray, columns: int, rows: int) -> np.ndarray | None:
    if image is None or image.ndim not in (2, 3):
        return None
    if columns < 3 or rows < 3:
        raise ValueError("棋盘内角点行列数均不得小于 3。")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    pattern_size = (int(columns), int(rows))
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray, pattern_size, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        )
        if found:
            return corners.astype(np.float32)

    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def find_corners(image: np.ndarray, columns: int, rows: int) -> np.ndarray | None:
    """Return the detected chessboard corners, or ``None`` when absent."""
    return _image_points(image, columns, rows)


def render_checkerboard(columns: int, rows: int, square_pixels: int = 96) -> np.ndarray:
    """Render a black/white chessboard with the requested inner-corner count."""
    if columns < 3 or rows < 3:
        raise ValueError("棋盘内角点行列数均不得小于 3。")
    square_pixels = max(16, int(square_pixels))
    height = (int(rows) + 1) * square_pixels
    width = (int(columns) + 1) * square_pixels
    image = np.full((height, width), 255, dtype=np.uint8)
    for row in range(int(rows) + 1):
        for column in range(int(columns) + 1):
            if (row + column) % 2 == 0:
                y0, x0 = row * square_pixels, column * square_pixels
                image[y0:y0 + square_pixels, x0:x0 + square_pixels] = 0
    return image


def solve_intrinsics(images: Sequence[np.ndarray], columns: int, rows: int,
                     square_size_mm: float) -> IntrinsicCalibration:
    """Calibrate from frozen chessboard images and return a validated result."""
    if square_size_mm <= 0.0:
        raise ValueError("棋盘格宽必须大于 0。")
    if len(images) < 20:
        raise ValueError("请至少采集 20 张覆盖中心、四角和不同倾角的棋盘图。")

    object_template = np.zeros((int(columns) * int(rows), 3), np.float32)
    object_template[:, :2] = np.mgrid[0:int(columns), 0:int(rows)].T.reshape(-1, 2)
    object_template[:, :2] *= float(square_size_mm)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: Tuple[int, int] | None = None

    for image in images:
        corners = _image_points(image, columns, rows)
        if corners is None:
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            raise ValueError("所有棋盘样本必须使用同一相机分辨率。")
        object_points.append(object_template.copy())
        image_points.append(corners)

    if image_size is None or len(object_points) < 20:
        raise ValueError("有效棋盘样本不足 20 张；请重新采集清晰、完整的棋盘画面。")

    rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    if not np.isfinite(rms) or not np.isfinite(camera_matrix).all() or not np.isfinite(distortion).all():
        raise ValueError("OpenCV 标定结果无效，请检查棋盘格宽和样本覆盖范围。")
    return IntrinsicCalibration(
        image_size=image_size,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion.reshape(-1),
        reprojection_error_px=float(rms),
        square_size_mm=float(square_size_mm),
        valid_sample_count=len(object_points),
    )


def save_intrinsic_result(path: Path | str, result: IntrinsicCalibration) -> None:
    target = _as_path(path)
    payload = {
        "image_size": list(result.image_size),
        "camera_matrix": result.camera_matrix.tolist(),
        "distortion_coefficients": result.distortion_coefficients.reshape(-1).tolist(),
        "reprojection_error_px": result.reprojection_error_px,
        "square_size_mm": result.square_size_mm,
        "valid_sample_count": result.valid_sample_count,
    }
    target.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def load_intrinsic_result(path: Path | str) -> IntrinsicCalibration:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    image_size = tuple(int(value) for value in payload["image_size"])
    camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(payload["distortion_coefficients"], dtype=np.float64).reshape(-1)
    if len(image_size) != 2 or min(image_size) <= 0 or camera_matrix.shape != (3, 3) or distortion.size < 4:
        raise ValueError("内参文件格式无效。")
    return IntrinsicCalibration(
        image_size=(image_size[0], image_size[1]),
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion,
        reprojection_error_px=float(payload["reprojection_error_px"]),
        square_size_mm=float(payload["square_size_mm"]),
        valid_sample_count=int(payload.get("valid_sample_count", 0)),
    )


def save_track_alignment(path: Path | str, result: TrackAlignment) -> None:
    target = _as_path(path)
    target.write_text(json.dumps({
        "image_size": list(result.image_size),
        "roi_y1": result.roi_y1,
        "roi_y2": result.roi_y2,
        "lookahead_y": result.lookahead_y,
        "x_zero": result.x_zero,
        "theta_zero_deg": result.theta_zero_deg,
        "gain_lateral": result.gain_lateral,
        "gain_heading": result.gain_heading,
        "speed_feedback": result.speed_feedback,
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def load_track_alignment(path: Path | str) -> TrackAlignment:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    image_size = tuple(int(value) for value in payload["image_size"])
    if len(image_size) != 2 or min(image_size) <= 0:
        raise ValueError("黑线零点文件的图像尺寸无效。")
    return TrackAlignment(
        image_size=(image_size[0], image_size[1]),
        roi_y1=int(payload["roi_y1"]), roi_y2=int(payload["roi_y2"]),
        lookahead_y=int(payload["lookahead_y"]), x_zero=float(payload["x_zero"]),
        theta_zero_deg=float(payload["theta_zero_deg"]),
        gain_lateral=float(payload["gain_lateral"]), gain_heading=float(payload["gain_heading"]),
        speed_feedback=float(payload.get("speed_feedback", 0.0)),
    )


def solve_manual_flat_plane(fit_points_px: np.ndarray, validation_points_px: np.ndarray,
                            intrinsic: IntrinsicCalibration, columns: int, rows: int,
                            square_size_mm: float, origin_x_m: float, origin_y_m: float):
    """Fit a local one-square ground homography and validate the next two points.

    A-D must be four adjacent inner corners in clockwise order (A top-left,
    B top-right, C bottom-right, D bottom-left); E/F are the next adjacent
    corners selected for validation.  This result is diagnostic only and is
    never sent to the vehicle controller.
    """
    if square_size_mm <= 0.0:
        raise ValueError("棋盘格宽必须大于 0。")
    fit = np.asarray(fit_points_px, dtype=np.float64).reshape(-1, 2)
    validation = np.asarray(validation_points_px, dtype=np.float64).reshape(-1, 2)
    if fit.shape != (4, 2) or validation.shape != (2, 2):
        raise ValueError("需要 A-D 四个拟合点和 E/F 两个验证点。")
    square_m = float(square_size_mm) / 1000.0
    world_fit = np.array([
        [origin_x_m, origin_y_m],
        [origin_x_m + square_m, origin_y_m],
        [origin_x_m + square_m, origin_y_m + square_m],
        [origin_x_m, origin_y_m + square_m],
    ], dtype=np.float64)
    homography, _ = cv2.findHomography(fit, world_fit, method=0)
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("无法拟合平地映射；请检查 A-D 是否为相邻棋盘内角点。")
    validation_world = np.array([
        [origin_x_m + 2.0 * square_m, origin_y_m],
        [origin_x_m + 2.0 * square_m, origin_y_m + square_m],
    ], dtype=np.float64)
    predicted = cv2.perspectiveTransform(validation.reshape(1, -1, 2), homography).reshape(-1, 2)
    errors = np.linalg.norm(predicted - validation_world, axis=1)
    return homography, errors


def save_flat_validation(path: Path | str, intrinsic: IntrinsicCalibration, homography: np.ndarray,
                         errors_m: Iterable[float], columns: int, rows: int, square_size_mm: float,
                         origin_x_m: float, origin_y_m: float) -> None:
    target = _as_path(path)
    target.write_text(json.dumps({
        "image_size": list(intrinsic.image_size),
        "reprojection_error_px": intrinsic.reprojection_error_px,
        "homography_pixel_to_ground_m": np.asarray(homography).tolist(),
        "validation_errors_m": [float(value) for value in errors_m],
        "chessboard_inner_corners": [int(columns), int(rows)],
        "square_size_mm": float(square_size_mm),
        "origin_a_m": [float(origin_x_m), float(origin_y_m)],
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def write_cpp_header(path: Path | str, intrinsic: IntrinsicCalibration,
                     alignment: TrackAlignment) -> None:
    if intrinsic.reprojection_error_px > 1.0:
        raise ValueError("重投影误差超过 1 px，拒绝生成固件标定头文件。")
    target = _as_path(path)
    camera = intrinsic.camera_matrix.reshape(-1)
    distortion = intrinsic.distortion_coefficients.reshape(-1)
    camera_values = ", ".join(f"{float(value):.9g}F" for value in camera)
    distortion_values = ", ".join(f"{float(value):.9g}F" for value in distortion)
    target.write_text(
        "#pragma once\n\n"
        "// Generated by tool/host_pyqt.py.  Re-run calibration after changing camera resolution.\n"
        "namespace balance_car::config::vision_calibration\n{\n"
        f"constexpr int kImageWidth = {intrinsic.image_size[0]};\n"
        f"constexpr int kImageHeight = {intrinsic.image_size[1]};\n"
        f"constexpr float kCameraMatrix[] = {{{camera_values}}};\n"
        f"constexpr float kDistortion[] = {{{distortion_values}}};\n"
        f"constexpr float kTrackXZeroPixels = {alignment.x_zero:.9g}F;\n"
        f"constexpr float kTrackThetaZeroDegrees = {alignment.theta_zero_deg:.9g}F;\n"
        f"constexpr float kTrackLateralGain = {alignment.gain_lateral:.9g}F;\n"
        f"constexpr float kTrackHeadingGain = {alignment.gain_heading:.9g}F;\n"
        "} // namespace balance_car::config::vision_calibration\n",
        encoding="utf-8",
    )
