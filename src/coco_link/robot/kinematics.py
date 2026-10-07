"""差動2輪ロボットの運動学.

  vx = r (ω_R + ω_L) / 2          ω_L = (vx - wz b/2) / r
  wz = r (ω_R - ω_L) / b          ω_R = (vx + wz b/2) / r
"""

from __future__ import annotations

import math

from ..common.geometry import Pose2D, wrap_angle


def body_to_wheel(vx: float, wz: float, r: float, b: float) -> tuple[float, float]:
    """機体速度 → 車輪角速度 (ω_L, ω_R) [rad/s]."""
    return (vx - wz * b / 2.0) / r, (vx + wz * b / 2.0) / r


def wheel_to_body(wl: float, wr: float, r: float, b: float) -> tuple[float, float]:
    """車輪角速度 → 機体速度 (vx, wz)."""
    return r * (wr + wl) / 2.0, r * (wr - wl) / b


def limit_wheel_speeds(wl: float, wr: float, w_max: float) -> tuple[float, float]:
    """どちらかが w_max を超えたら両輪を同じ比率で縮小する（旋回半径を保つ）."""
    m = max(abs(wl), abs(wr))
    if m > w_max > 0:
        k = w_max / m
        return wl * k, wr * k
    return wl, wr


def integrate_unicycle(pose: Pose2D, vx: float, wz: float, dt: float) -> Pose2D:
    """一定の (vx, wz) で dt 秒進んだ姿勢（円弧で厳密積分）."""
    th = pose.th
    if abs(wz) < 1e-9:
        return Pose2D(pose.x + vx * dt * math.cos(th), pose.y + vx * dt * math.sin(th), th)
    th2 = th + wz * dt
    rad = vx / wz
    return Pose2D(pose.x + rad * (math.sin(th2) - math.sin(th)),
                  pose.y - rad * (math.cos(th2) - math.cos(th)),
                  wrap_angle(th2))
