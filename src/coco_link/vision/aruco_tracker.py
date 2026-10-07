"""ArUco マーカーの検出とロボット姿勢の計算.

マーカーは「マーカーの上辺がロボットの前方」を向くように貼る（docs/vision.md）。
OpenCV のコーナー順は (左上, 右上, 右下, 左下) なので、
    前方ベクトル = (上辺の中点) - (下辺の中点)
をフィールド座標へ変換して向き θ を求める。
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from ..protocol import messages as m
from .calibration import FieldCalibration


class ArucoTracker:
    def __init__(self, dict_name: str = "DICT_4X4_50"):
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # サブピクセル精度
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

    def detect(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        """戻り値: ID → コーナー (4, 2) [px]."""
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners, strict=True)}


def marker_quality(c: np.ndarray) -> float:
    """マーカーの歪み具合から 0–1 の信頼度を作る（正方形に近いほど 1）."""
    sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    if min(sides) < 1e-6:
        return 0.0
    return float(min(sides) / max(sides))


def robot_poses(markers: dict[int, np.ndarray], calib: FieldCalibration, robot_ids: range,
                marker_height: float = 0.0) -> list[m.DetectedRobot]:
    out = []
    for mid, c in markers.items():
        if mid not in robot_ids:
            continue
        top = (c[0] + c[1]) / 2
        bottom = (c[2] + c[3]) / 2
        center = c.mean(axis=0)
        f = calib.to_field(np.array([center, top, bottom]), height=marker_height)
        th = math.atan2(f[1][1] - f[2][1], f[1][0] - f[2][0])
        out.append(m.DetectedRobot(id=mid, x=round(float(f[0][0]), 4), y=round(float(f[0][1]), 4),
                                   th=round(th, 4), conf=round(marker_quality(c), 2)))
    return out
