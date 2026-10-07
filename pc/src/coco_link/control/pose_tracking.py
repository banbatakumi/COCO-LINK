"""姿勢制御: 目標点・目標姿勢へ向かう (v, ω) を計算する. docs/control.md §2."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..common.geometry import Pose2D, wrap_angle


@dataclass
class GoToPoseGains:
    k_rho: float = 1.2        # 距離ゲイン
    k_alpha: float = 4.0      # 目標方向への旋回ゲイン
    k_beta: float = -0.8      # 最終姿勢への旋回ゲイン (負)
    k_heading: float = 3.0    # 到着後の向き合わせゲイン
    pos_tol: float = 0.03     # 到着判定 [m]
    th_tol: float = math.radians(8)
    allow_reverse: bool = True


def saturate(v: float, w: float, v_max: float, w_max: float) -> tuple[float, float]:
    """(v, ω) を同じ比率で縮小して上限に収める（軌道の曲率を保つ）."""
    k = 1.0
    if abs(v) > v_max > 0:
        k = min(k, v_max / abs(v))
    if abs(w) > w_max > 0:
        k = min(k, w_max / abs(w))
    return v * k, w * k


def go_to_pose(pose: Pose2D, target: Pose2D, v_max: float, w_max: float,
               g: GoToPoseGains | None = None) -> tuple[float, float, bool]:
    """極座標フィードバック (Astolfi 型) による姿勢安定化. 戻り値 (v, ω, 到着したか).

    ρ: 目標までの距離, α: ロボット前方から見た目標方向, β: 目標方向から見た最終姿勢
        v = k_ρ ρ,  ω = k_α α + k_β β
    k_ρ > 0, k_β < 0, k_α - k_ρ > 0 で局所漸近安定（Siegwart "Autonomous Mobile Robots" 3.6）。
    後退も許す場合は |α| > π/2 なら前後を反転して扱う。
    """
    g = g or GoToPoseGains()
    dx, dy = target.x - pose.x, target.y - pose.y
    rho = math.hypot(dx, dy)
    if rho < g.pos_tol:
        e_th = wrap_angle(target.th - pose.th)
        if abs(e_th) < g.th_tol:
            return 0.0, 0.0, True
        return 0.0, max(-w_max, min(w_max, g.k_heading * e_th)), False
    alpha = wrap_angle(math.atan2(dy, dx) - pose.th)
    direction = 1.0
    if g.allow_reverse and abs(alpha) > math.pi / 2:
        direction = -1.0
        alpha = wrap_angle(alpha + math.pi)
    beta = wrap_angle(target.th - pose.th - alpha)
    v = direction * g.k_rho * rho
    w = g.k_alpha * alpha + g.k_beta * beta
    # 目標方向を大きく外れているときは前進を抑えて先に向きを変える
    v *= max(0.0, math.cos(alpha)) ** 2
    v, w = saturate(v, w, v_max, w_max)
    return v, w, False


def go_to_point(pose: Pose2D, x: float, y: float, v_max: float, w_max: float,
                k_v: float = 1.2, k_w: float = 4.0, tol: float = 0.03) -> tuple[float, float, bool]:
    """向きを問わず点へ向かう."""
    dx, dy = x - pose.x, y - pose.y
    rho = math.hypot(dx, dy)
    if rho < tol:
        return 0.0, 0.0, True
    alpha = wrap_angle(math.atan2(dy, dx) - pose.th)
    v = k_v * rho * max(0.0, math.cos(alpha)) ** 2
    return (*saturate(v, k_w * alpha, v_max, w_max), False)


def kanayama_tracking(pose: Pose2D, ref: Pose2D, v_ref: float, w_ref: float,
                      kx: float = 2.0, ky: float = 30.0, kth: float = 4.0) -> tuple[float, float]:
    """Kanayama の軌道追従則（参照軌道 ref が速度 v_ref, w_ref で動いているとき）.

    e = ref をロボット座標系で表したもの (ex, ey, eθ)
        v = v_ref cos eθ + kx ex
        ω = w_ref + v_ref (ky ey + kθ sin eθ)
    """
    e = ref.relative_to(pose)
    v = v_ref * math.cos(e.th) + kx * e.x
    w = w_ref + v_ref * (ky * e.y + kth * math.sin(e.th))
    return v, w
