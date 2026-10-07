"""センサモデル: 超音波レイキャスト・IMU・電池 ADC."""

from __future__ import annotations

import math
import random
from collections.abc import Iterable

from ..common.geometry import Pose2D
from ..protocol import messages as m
from .entities import CircleBody, SimRobot

GRAVITY = 9.80665
SOUND_MIN_RANGE = 0.02


def ray_circle(ox: float, oy: float, dx: float, dy: float, cx: float, cy: float, r: float) -> float | None:
    """原点 o から単位方向 d へのレイが円と最初に交わる距離."""
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    t = -b - sq
    if t < 0:
        t = -b + sq
        if c < 0:          # 原点が円の内側 → 接触扱い
            return 0.0
    return t if t >= 0 else None


def ray_box(ox: float, oy: float, dx: float, dy: float, w: float, h: float) -> float | None:
    """フィールド [0,w]×[0,h] の内側から壁までの距離."""
    ts = []
    if dx > 1e-12:
        ts.append((w - ox) / dx)
    elif dx < -1e-12:
        ts.append(-ox / dx)
    if dy > 1e-12:
        ts.append((h - oy) / dy)
    elif dy < -1e-12:
        ts.append(-oy / dy)
    ts = [t for t in ts if t >= 0]
    return min(ts) if ts else None


def ultrasonic(pose: Pose2D, offset: float, facing: float, fov_deg: float, max_range: float,
               circles: Iterable[tuple[float, float, float]], field_wh: tuple[float, float] | None,
               rng: random.Random, noise: float = 0.004, n_rays: int = 7) -> float | None:
    """超音波距離センサ. facing は機体前方からの角度（前=0, 後=π）."""
    th = pose.th + facing
    ox = pose.x + offset * math.cos(th)
    oy = pose.y + offset * math.sin(th)
    half = math.radians(fov_deg) / 2.0
    circles = list(circles)
    best = math.inf
    for i in range(n_rays):
        a = th + (-half + 2 * half * i / (n_rays - 1) if n_rays > 1 else 0.0)
        dx, dy = math.cos(a), math.sin(a)
        for cx, cy, r in circles:
            t = ray_circle(ox, oy, dx, dy, cx, cy, r)
            if t is not None and t < best:
                best = t
        if field_wh is not None:
            t = ray_box(ox, oy, dx, dy, *field_wh)
            if t is not None and t < best:
                best = t
    if best > max_range:
        return None
    return round(max(SOUND_MIN_RANGE, best + rng.gauss(0.0, noise)), 4)


def imu(robot: SimRobot) -> m.Imu:
    p = robot.plant
    rng = robot.rng
    n = robot.gyro_noise
    return m.Imu(ax=p.ax + rng.gauss(0, 0.05), ay=p.vx * p.wz + rng.gauss(0, 0.05), az=GRAVITY + rng.gauss(0, 0.05),
                 gx=rng.gauss(0, n), gy=rng.gauss(0, n), gz=p.wz + robot.params.gyro_bias_z + rng.gauss(0, n))


def obstacles_for(robot: SimRobot, robots: Iterable[SimRobot], bodies: Iterable[CircleBody]):
    """robot 自身以外の円形物体 (x, y, r) の列."""
    for o in robots:
        if o is not robot:
            yield o.pose.x, o.pose.y, o.params.body_radius
    for b in bodies:
        yield b.x, b.y, b.r
