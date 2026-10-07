"""安全機能: 超音波による速度制限と、ロボット間の衝突回避. docs/control.md §5."""

from __future__ import annotations

import math
from collections.abc import Iterable

from ..common.geometry import Pose2D, Vec2, wrap_angle


def ultrasonic_speed_limit(v: float, us_front: float | None, us_rear: float | None,
                           stop_dist: float = 0.08, slow_dist: float = 0.30) -> float:
    """進行方向の超音波距離に応じて速度を線形に絞る. stop_dist 以下で 0."""
    d = us_front if v > 0 else us_rear
    if d is None:
        return v
    k = (d - stop_dist) / (slow_dist - stop_dist)
    return v * max(0.0, min(1.0, k))


def avoid_robots(pose: Pose2D, v: float, w: float, others: Iterable[Vec2], radius: float,
                 influence: float = 0.25, k_turn: float = 2.0) -> tuple[float, float]:
    """前方 ±60° にいる他ロボットとの距離に応じて減速し、相手と反対側へ曲がる（簡易ポテンシャル法）."""
    if v == 0.0:
        return v, w
    heading = pose.th if v > 0 else wrap_angle(pose.th + math.pi)
    scale = 1.0
    dw = 0.0
    for o in others:
        d = o - pose.pos
        dist = d.norm() - 2 * radius
        if dist > influence:
            continue
        bearing = wrap_angle(d.angle() - heading)
        if abs(bearing) > math.radians(60):
            continue
        k = max(0.0, dist) / influence
        scale = min(scale, k)
        dw += -math.copysign(k_turn * (1 - k), bearing) * (1 if v > 0 else -1)
    return v * scale, w + dw
