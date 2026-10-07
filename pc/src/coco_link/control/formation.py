"""隊形の生成と、ロボットへのスロット割り当て（ハンガリアン法）. docs/control.md §3."""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import linear_sum_assignment

from ..common.geometry import Pose2D, Vec2

SHAPES = ("line", "column", "circle", "v", "grid", "heart")


def formation_offsets(shape: str, n: int, spacing: float) -> list[Vec2]:
    """隊形中心を原点・隊形の前方を +x としたスロット位置（n 個）."""
    if n <= 0:
        return []
    if shape == "line":          # 横一列（前方に対して垂直）
        return [Vec2(0.0, (i - (n - 1) / 2) * spacing) for i in range(n)]
    if shape == "column":        # 縦一列
        return [Vec2(-(i - (n - 1) / 2) * spacing, 0.0) for i in range(n)]
    if shape == "circle":
        if n == 1:
            return [Vec2()]
        radius = max(spacing / (2 * math.sin(math.pi / n)), spacing / 2)
        return [Vec2(radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if shape == "v":             # 先頭 1 台 + 左右に後退
        pts = [Vec2()]
        for i in range(1, n):
            k = (i + 1) // 2
            side = 1 if i % 2 else -1
            pts.append(Vec2(-k * spacing * 0.8, side * k * spacing * 0.8))
        cx = sum(p.x for p in pts) / n
        return [Vec2(p.x - cx, p.y) for p in pts]
    if shape == "grid":
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        return [Vec2(-((i // cols) - (rows - 1) / 2) * spacing, ((i % cols) - (cols - 1) / 2) * spacing)
                for i in range(n)]
    if shape == "heart":         # ハート曲線 x=16sin³t, y=13cost-5cos2t-2cos3t-cos4t を等間隔(道のり)に配置
        dense = []
        for i in range(720):
            t = 2 * math.pi * i / 720
            hx = 16 * math.sin(t) ** 3
            hy = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
            dense.append(Vec2(hy, -hx))   # 前方 (+x) にハートの上側
        seg = [(dense[(i + 1) % len(dense)] - dense[i]).norm() for i in range(len(dense))]
        perimeter = sum(seg)
        scale = spacing * max(n, 6) / perimeter      # 周長 = 台数 × 間隔（少数台では小さくしすぎない）
        pts, acc, k = [], 0.0, 0
        for i in range(n):
            s_target = perimeter * i / n
            while acc + seg[k] < s_target:
                acc += seg[k]
                k += 1
            a, b = dense[k], dense[(k + 1) % len(dense)]
            pts.append((a + (b - a) * ((s_target - acc) / seg[k])) * scale)
        cx, cy = sum(p.x for p in pts) / n, sum(p.y for p in pts) / n
        return [Vec2(p.x - cx, p.y - cy) for p in pts]
    raise ValueError(f"unknown shape: {shape}")


def slot_poses(shape: str, n: int, spacing: float, center: Pose2D) -> list[Pose2D]:
    """隊形中心姿勢 center に配置した各スロットの目標姿勢（向きは隊形の向き）."""
    return [Pose2D(*_xy(center.transform_point(o)), center.th) for o in formation_offsets(shape, n, spacing)]


def _xy(v: Vec2) -> tuple[float, float]:
    return v.x, v.y


def assign_slots(robot_positions: dict[int, Vec2], slots: list[Pose2D]) -> dict[int, int]:
    """移動距離の二乗和が最小になるようにロボット→スロットを割り当てる（ハンガリアン法 O(n³)）.

    距離の二乗和を最小化すると、直線移動したときに経路が交差しにくいという性質がある。
    """
    ids = sorted(robot_positions)
    if not ids or not slots:
        return {}
    cost = np.array([[(robot_positions[i].x - s.x) ** 2 + (robot_positions[i].y - s.y) ** 2 for s in slots]
                     for i in ids])
    rows, cols = linear_sum_assignment(cost)
    return {ids[r]: int(c) for r, c in zip(rows, cols, strict=True)}
