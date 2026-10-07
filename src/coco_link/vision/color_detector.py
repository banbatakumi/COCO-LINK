"""色領域の検出: 緑=人, 赤=障害物, 青=物資.

HSV 色空間で閾値処理 → モルフォロジーでノイズ除去 → 輪郭 → 床面積で足切り → 重心と等価半径.
HSV を使うのは、明るさ (V) が変わっても色相 (H) が変わりにくく、照明変化に強いから。
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from ..common.config import ColorRange
from ..protocol import messages as m
from .calibration import FieldCalibration


def color_mask(hsv: np.ndarray, c: ColorRange) -> np.ndarray:
    lo_s, hi_s = (c.s_min, c.v_min), (c.s_max, c.v_max)
    if c.h_min <= c.h_max:
        mask = cv2.inRange(hsv, (c.h_min, *lo_s), (c.h_max, *hi_s))
    else:   # 赤のように H が 0 をまたぐ
        mask = cv2.inRange(hsv, (c.h_min, *lo_s), (179, *hi_s)) | cv2.inRange(hsv, (0, *lo_s), (c.h_max, *hi_s))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def polygon_area(pts: np.ndarray) -> float:
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))


def detect_blobs(mask: np.ndarray, calib: FieldCalibration, min_area_m2: float,
                 max_count: int = 10) -> list[tuple[m.DetectedObject, np.ndarray]]:
    """戻り値: (検出物, 画像上の輪郭) のリスト（面積の大きい順）."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in contours:
        if cv2.contourArea(cnt) < 20:
            continue
        f = calib.to_field(cnt.reshape(-1, 2).astype(np.float64))
        area = polygon_area(f)
        if area < min_area_m2:
            continue
        mom = cv2.moments(cnt)
        if mom["m00"] == 0:
            continue
        cx, cy = calib.to_field(np.array([[mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]]]))[0]
        out.append((m.DetectedObject(x=round(float(cx), 4), y=round(float(cy), 4),
                                     r=round(math.sqrt(area / math.pi), 3)), cnt))
    out.sort(key=lambda t: -t[0].r)
    return out[:max_count]
