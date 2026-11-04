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


def avoid_circles(pose: Pose2D, v: float, w: float,
                  circles: Iterable[tuple[Vec2, float]],
                  influence: float = 0.25, k_turn: float = 2.0,
                  half_angle_deg: float = 60.0) -> tuple[float, float]:
    """前方 ±half_angle にいる円（人・障害物・他ロボット）を避ける（簡易ポテンシャル法）.

    circles は (中心, 機体半径を含めた衝突半径)。衝突半径は「相手の半径 + 自分の機体半径」。
    近いほど減速し、相手と反対側へ曲がる。経路計画の再計画までの間を埋める反応的な層。
    """
    if v == 0.0:
        return v, w
    heading = pose.th if v > 0 else wrap_angle(pose.th + math.pi)
    scale = 1.0
    dw = 0.0
    for center, combined_radius in circles:
        d = center - pose.pos
        dist = d.norm() - combined_radius
        if dist > influence:
            continue
        bearing = wrap_angle(d.angle() - heading)
        if abs(bearing) > math.radians(half_angle_deg):
            continue
        k = max(0.0, dist) / influence
        scale = min(scale, k)
        dw += -math.copysign(k_turn * (1 - k), bearing) * (1 if v > 0 else -1)
    return v * scale, w + dw


def avoid_robots(pose: Pose2D, v: float, w: float, others: Iterable[Vec2], radius: float,
                 influence: float = 0.25, k_turn: float = 2.0) -> tuple[float, float]:
    """前方 ±60° にいる他ロボットとの距離に応じて減速し、相手と反対側へ曲がる（簡易ポテンシャル法）."""
    circles = ((o, 2.0 * radius) for o in others)
    return avoid_circles(pose, v, w, circles, influence=influence, k_turn=k_turn)
