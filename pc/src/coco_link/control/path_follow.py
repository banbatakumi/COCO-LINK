"""経路追従: 軌跡 (breadcrumb) の記録と Pure Pursuit. docs/control.md §4."""

from __future__ import annotations

import math
from collections import deque

from ..common.geometry import Pose2D, Vec2, wrap_angle


class Trail:
    """移動体が通った点列を一定間隔で記録する."""

    def __init__(self, spacing: float = 0.02, max_points: int = 2000):
        self.spacing = spacing
        self.points: deque[Vec2] = deque(maxlen=max_points)

    def add(self, p: Vec2) -> None:
        if not self.points or (p - self.points[-1]).norm() >= self.spacing - 1e-9:
            self.points.append(p)

    def prune_before(self, pos: Vec2, radius: float) -> None:
        """pos から radius 以内に来た点より前の点を捨てる（通過済みとみなす）."""
        idx = -1
        for i, q in enumerate(self.points):
            if (q - pos).norm() < radius:
                idx = i
        for _ in range(idx + 1 if idx >= 0 else 0):
            self.points.popleft()

    def length_from(self, pos: Vec2, end: Vec2 | None = None) -> float:
        """pos → 先頭点 → … → 末尾点 (→ end) の道のり."""
        pts = list(self.points)
        if end is not None:
            pts.append(end)
        if not pts:
            return 0.0
        total = (pts[0] - pos).norm()
        for a, b in zip(pts, pts[1:], strict=False):
            total += (b - a).norm()
        return total

    def as_list(self) -> list[tuple[float, float]]:
        return [(p.x, p.y) for p in self.points]


def lookahead_point(pts: list[Vec2], pos: Vec2, lookahead: float) -> Vec2 | None:
    """点列上で pos から道のり lookahead だけ先の点."""
    if not pts:
        return None
    remaining = lookahead
    prev = pos
    for q in pts:
        d = (q - prev).norm()
        if d >= remaining and d > 1e-9:
            return prev + (q - prev) * (remaining / d)
        remaining -= d
        prev = q
    return pts[-1]


def pure_pursuit(pose: Pose2D, goal: Vec2, v: float) -> tuple[float, float]:
    """Pure Pursuit: 前方注視点 goal を通る円弧に乗る角速度.

    曲率 κ = 2 sin α / L （α: 注視点の方向, L: 注視点までの距離）,  ω = v κ
    """
    d = goal - pose.pos
    L = d.norm()
    if L < 1e-6:
        return 0.0, 0.0
    alpha = wrap_angle(d.angle() - pose.th)
    if abs(alpha) > math.pi / 2:          # 注視点が後ろ → その場で向きを変える
        return 0.0, math.copysign(2.5, alpha)
    kappa = 2.0 * math.sin(alpha) / L
    return v, v * kappa
