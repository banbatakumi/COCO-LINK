"""画像の入力源: カメラ / 動画ファイル / 合成画像."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from ..common.geometry import Pose2D
from .synthetic import SyntheticScene, render


class CameraSource:
    def __init__(self, index: int = 0, width: int = 1280, height: int = 720):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.name = f"camera {index}"

    def read(self) -> tuple[bool, np.ndarray | None, float]:
        ok, frame = self.cap.read()
        return ok, frame, time.monotonic()

    def release(self) -> None:
        self.cap.release()


class VideoSource:
    def __init__(self, path: str, loop: bool = True):
        self.cap = cv2.VideoCapture(path)
        self.loop = loop
        self.name = path
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

    def read(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return ok, frame, time.monotonic()

    def release(self) -> None:
        self.cap.release()


class SyntheticSource:
    """ロボットが円を描いて動く合成映像（デモ・テスト用）."""

    def __init__(self, scene: SyntheticScene | None = None, n_robots: int = 3):
        self.scene = scene or SyntheticScene(tilt=0.04)
        self.n = n_robots
        self.t0 = time.monotonic()
        self.name = "synthetic"
        self.rng = np.random.default_rng(0)
        if not self.scene.objects:
            w, h = self.scene.field_cfg.width, self.scene.field_cfg.height
            self.scene.objects = [("person", w * 0.8, h * 0.7, 0.15), ("obstacle", w * 0.25, h * 0.3, 0.1),
                                  ("cargo", w * 0.7, h * 0.25, 0.06)]

    def read(self):
        t = time.monotonic() - self.t0
        w, h = self.scene.field_cfg.width, self.scene.field_cfg.height
        self.scene.robots = {}
        for i in range(self.n):
            a = 0.4 * t + 2 * math.pi * i / self.n
            self.scene.robots[i + 1] = Pose2D(w / 2 + 0.5 * math.cos(a), h / 2 + 0.5 * math.sin(a), a + math.pi / 2)
        img, _ = render(self.scene, self.rng)
        time.sleep(1 / 30)
        return True, img, time.monotonic()

    def release(self) -> None:
        pass
