"""ビジョン処理の全体: 画像 1 枚 → world_state."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from ..common.clock import monotonic_ms
from ..common.config import FieldConfig
from ..protocol import messages as m
from .aruco_tracker import ArucoTracker, robot_poses
from .calibration import FieldCalibration
from .color_detector import color_mask, detect_blobs

KINDS = {"person": "persons", "obstacle": "obstacles", "cargo": "cargo"}


@dataclass
class VisionResult:
    world: m.WorldState
    markers: dict[int, np.ndarray] = field(default_factory=dict)
    blobs: dict[str, list[tuple[m.DetectedObject, np.ndarray]]] = field(default_factory=dict)
    process_ms: float = 0.0
    calibrated: bool = False


class VisionPipeline:
    def __init__(self, field_cfg: FieldConfig, calib: FieldCalibration | None = None,
                 robot_ids: range = range(0, 30), marker_height: float = 0.08, camera_height: float = 2.0,
                 robot_radius: float = 0.06, camera_latency_ms: float = 50.0, auto_calibrate: bool = True):
        self.field = field_cfg
        self.calib = calib
        self.robot_ids = robot_ids
        self.marker_height = marker_height
        self.camera_height = camera_height
        self.robot_radius = robot_radius
        self.camera_latency_ms = camera_latency_ms
        self.auto_calibrate = auto_calibrate
        self.tracker = ArucoTracker(field_cfg.aruco_dict)
        self.frame = 0

    def process(self, frame_bgr: np.ndarray, t_capture: float | None = None) -> VisionResult:
        t0 = time.perf_counter()
        t_capture = time.monotonic() if t_capture is None else t_capture
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        markers = self.tracker.detect(gray)
        h, w = gray.shape
        if self.auto_calibrate:
            c = FieldCalibration.from_corner_markers(markers, self.field, (w, h), self.camera_height)
            if c is not None:
                self.calib = c
        self.frame += 1
        ws = m.WorldState(frame=self.frame, field=m.FieldSize(self.field.width, self.field.height))
        result = VisionResult(world=ws, markers=markers, calibrated=self.calib is not None)
        if self.calib is not None:
            ws.robots = robot_poses(markers, self.calib, self.robot_ids, self.marker_height)
            # ロボット（LED などの色）を色検出から除外する
            exclude = np.zeros((h, w), np.uint8)
            for mid, c in markers.items():
                if mid in self.robot_ids:
                    side = float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]))
                    rad = side * self.robot_radius / self.field.robot_marker_size * 1.2
                    cv2.circle(exclude, tuple(map(int, c.mean(axis=0))), int(rad), 255, -1)
                cv2.fillConvexPoly(exclude, c.astype(np.int32), 255)
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            for kind, attr in KINDS.items():
                rng = self.field.colors.get(kind)
                if rng is None:
                    continue
                mask = color_mask(hsv, rng)
                mask[exclude > 0] = 0
                blobs = detect_blobs(mask, self.calib, self.field.min_blob_area_m2)
                result.blobs[kind] = blobs
                setattr(ws, attr, [b for b, _ in blobs])
        result.process_ms = (time.perf_counter() - t0) * 1000.0
        ws.latency_ms = round((time.monotonic() - t_capture) * 1000.0 + self.camera_latency_ms, 1)
        ws.stamp_ms = (monotonic_ms() - int(ws.latency_ms)) & 0xFFFFFFFF
        return result


def draw_overlay(frame: np.ndarray, res: VisionResult, calib: FieldCalibration | None) -> np.ndarray:
    """検出結果を画像に描く（GUI 表示用）."""
    img = frame.copy()
    colors = {"person": (0, 255, 0), "obstacle": (0, 0, 255), "cargo": (255, 0, 0)}
    for kind, blobs in res.blobs.items():
        for obj, cnt in blobs:
            cv2.drawContours(img, [cnt], -1, colors[kind], 2)
            if calib is not None:
                u, v = calib.to_image([[obj.x, obj.y]])[0]
                cv2.putText(img, f"{kind} ({obj.x:.2f},{obj.y:.2f})", (int(u) + 5, int(v)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colors[kind], 1, cv2.LINE_AA)
    robot_ids = {r.id: r for r in res.world.robots}
    for mid, c in res.markers.items():
        pts = c.astype(np.int32)
        col = (0, 200, 255) if mid in robot_ids else (255, 0, 255)
        cv2.polylines(img, [pts], True, col, 2)
        top = ((c[0] + c[1]) / 2).astype(int)
        center = c.mean(axis=0).astype(int)
        cv2.arrowedLine(img, tuple(center), tuple(center + (top - center) * 2), col, 2, tipLength=0.3)
        label = f"#{mid}"
        if mid in robot_ids:
            r = robot_ids[mid]
            label += f" ({r.x:.2f},{r.y:.2f},{np.degrees(r.th):.0f}deg)"
        cv2.putText(img, label, tuple(center + np.array([10, -10])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1,
                    cv2.LINE_AA)
    if calib is not None:   # フィールド枠
        f = res.world.field
        box = calib.to_image([[0, 0], [f.w, 0], [f.w, f.h], [0, f.h]]).astype(np.int32)
        cv2.polylines(img, [box], True, (0, 255, 255), 1)
    else:
        cv2.putText(img, "NOT CALIBRATED: show corner markers 40-43 or click 4 corners", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    return img
