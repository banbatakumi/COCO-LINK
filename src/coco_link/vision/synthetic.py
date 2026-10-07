"""合成カメラ画像の生成（カメラが無い環境での動作確認・テスト用）.

フィールドを真上から見た画像に、四隅マーカー・ロボット（ArUco 付き）・人/障害物/物資の色円を描き、
必要なら射影変換で「斜めから撮った」画像にする。生成に使った真値も返すので、検出精度を評価できる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from ..common.config import FieldConfig
from ..common.geometry import Pose2D

BGR = {"person": (60, 170, 40), "obstacle": (40, 40, 210), "cargo": (200, 90, 30)}


@dataclass
class SyntheticScene:
    field_cfg: FieldConfig = field(default_factory=FieldConfig)
    robots: dict[int, Pose2D] = field(default_factory=dict)
    objects: list[tuple[str, float, float, float]] = field(default_factory=list)   # (kind, x, y, r)
    robot_radius: float = 0.06
    px_per_m: float = 350.0
    margin_m: float = 0.15
    tilt: float = 0.0          # 射影の歪み量 (0 = 真上から)
    noise: float = 3.0         # 画素ノイズ σ
    blur: int = 3


def _marker_patch(dictionary, marker_id: int, px: int, border_cells: float = 1.0) -> np.ndarray:
    """白い余白付きのマーカー画像（ArUco は周囲に白い余白が必要）."""
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    pad = int(px / 6 * border_cells)
    return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def _paste(scene: np.ndarray, patch: np.ndarray, quad: np.ndarray) -> None:
    """patch (グレースケール) を scene 上の四角形 quad (左上,右上,右下,左下) へ貼る."""
    h, w = patch.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    M = cv2.getPerspectiveTransform(src, quad.astype(np.float32))
    size = (scene.shape[1], scene.shape[0])
    warped = cv2.warpPerspective(cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR), M, size, flags=cv2.INTER_LINEAR)
    mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), M, size, flags=cv2.INTER_NEAREST)
    scene[mask > 0] = warped[mask > 0]


def render(scene: SyntheticScene, rng: np.random.Generator | None = None) -> tuple[np.ndarray, np.ndarray]:
    """戻り値: (BGR 画像, 真上画像→最終画像 の射影行列)."""
    f, s, mg = scene.field_cfg, scene.px_per_m, scene.margin_m
    W, H = int((f.width + 2 * mg) * s), int((f.height + 2 * mg) * s)
    img = np.full((H, W, 3), (205, 210, 210), np.uint8)

    def px(x: float, y: float) -> tuple[float, float]:
        return (mg + x) * s, (mg + f.height - y) * s

    cv2.rectangle(img, tuple(map(int, px(0, f.height))), tuple(map(int, px(f.width, 0))), (235, 238, 238), -1)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f.aruco_dict))

    def square(cx: float, cy: float, size: float, th: float) -> np.ndarray:
        """中心 (cx,cy)・一辺 size・上辺が向き th を向く正方形の画像上の四隅（余白込みの patch 用に 8/6 倍）."""
        half = size / 2 * 8 / 6
        fwd = np.array([math.cos(th), math.sin(th)])
        left = np.array([-math.sin(th), math.cos(th)])
        c = np.array([cx, cy])
        corners = [c + half * fwd + half * left, c + half * fwd - half * left,
                   c - half * fwd - half * left, c - half * fwd + half * left]   # 左上, 右上, 右下, 左下
        return np.array([px(*p) for p in corners])

    # 四隅マーカー（中心がフィールドの角）
    corners = [(0, 0), (f.width, 0), (f.width, f.height), (0, f.height)]
    for mid, (x, y) in zip(f.corner_marker_ids, corners, strict=True):
        _paste(img, _marker_patch(dictionary, mid, 120), square(x, y, f.corner_marker_size, math.pi / 2))
    for kind, x, y, r in scene.objects:
        cv2.circle(img, tuple(map(int, px(x, y))), int(r * s), BGR[kind], -1, cv2.LINE_AA)
    for rid, p in scene.robots.items():
        cv2.circle(img, tuple(map(int, px(p.x, p.y))), int(scene.robot_radius * s), (50, 50, 50), -1, cv2.LINE_AA)
        _paste(img, _marker_patch(dictionary, rid, 120), square(p.x, p.y, f.robot_marker_size, p.th))

    # 斜めから撮影したような射影歪み
    src = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    d = scene.tilt * W
    dst = np.float32([[d, d * 0.5], [W - d, 0], [W, H], [0, H - d * 0.3]])
    M = cv2.getPerspectiveTransform(src, dst)
    img = cv2.warpPerspective(img, M, (W, H), borderValue=(90, 90, 90))
    if scene.blur:
        img = cv2.GaussianBlur(img, (scene.blur, scene.blur), 0)
    if scene.noise > 0:
        rng = rng or np.random.default_rng(0)
        img = np.clip(img.astype(np.float32) + rng.normal(0, scene.noise, img.shape), 0, 255).astype(np.uint8)
    return img, M
