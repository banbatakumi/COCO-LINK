"""協調物資運搬のための、物資の選択・把持隊形・押し出し目標の生成. docs/control.md §7.

把持隊形は「目標方向 (+x) を向いた物資の後方に広がる円弧」で表す。
  - 物資中心からロボット中心までの距離 R = 物資半径 + 機体半径 + 余白
  - 後方 (±x に対して π) を中心に、半角 spread の範囲へ n 台を等間隔に配置
  - 各ロボットは目標方向を向く（= 物資を押す向き）
台数が多いときに隣のロボットと重ならないよう、R は円弧上の弦長からも決める。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..common.geometry import Pose2D, Vec2, wrap_angle

if TYPE_CHECKING:
    from ..fleet.world_model import World


@dataclass(frozen=True)
class CargoObject:
    """ビジョンが見つけた物資（青）."""

    x: float
    y: float
    r: float

    @property
    def pos(self) -> Vec2:
        return Vec2(self.x, self.y)


def collect_cargo(world: World) -> list[CargoObject]:
    """World に含まれる物資を列挙する."""
    return [CargoObject(o.x, o.y, o.r) for o in world.cargo]


def nearest_cargo(x: float, y: float, cargoes: list[CargoObject],
                  max_surface_dist: float = 0.35) -> CargoObject | None:
    """クリック位置 (x, y) に最も近い物資を返す（物資の表面からの距離で判定）."""
    best: CargoObject | None = None
    best_d = math.inf
    for c in cargoes:
        d = max(0.0, math.hypot(x - c.x, y - c.y) - c.r)
        if d < best_d:
            best, best_d = c, d
    return best if best is not None and best_d <= max_surface_dist else None


def track_cargo(cargoes: list[CargoObject], last_pos: Vec2, gate: float = 0.8) -> CargoObject | None:
    """直前の位置に最も近い物資を返す（押している物資を見失わないため）."""
    if not cargoes:
        return None
    best = min(cargoes, key=lambda c: (c.pos - last_pos).norm())
    return best if (best.pos - last_pos).norm() <= gate else None


def grasp_radius(cargo_r: float, body_radius: float, margin: float, n: int,
                 spread: float, robot_gap: float = 0.015) -> float:
    """把持隊形の半径 [m]. n 台が円弧上で重ならない最小半径も考慮する."""
    radius = cargo_r + body_radius + margin
    if n <= 1 or spread <= 1e-6:
        return radius
    # 隣り合うスロットの角度差 = 2*spread/(n-1)。その弦長 >= 機体直径 + 隙間
    chord = 2.0 * body_radius + robot_gap
    min_radius = chord / (2.0 * math.sin(spread / (n - 1)))
    return max(radius, min_radius)


def grasp_offsets(n: int, radius: float, spread: float) -> list[Vec2]:
    """物資座標系（+x = 目標方向）での把持スロット位置.

    後方 (π) を中心に ±spread の範囲へ等間隔に並べる。
    """
    if n <= 0:
        return []
    if n == 1:
        deltas = [0.0]
    else:
        deltas = [-spread + 2.0 * spread * i / (n - 1) for i in range(n)]
    return [Vec2(-radius * math.cos(d), -radius * math.sin(d)) for d in deltas]


def grasp_slots(center: Pose2D, n: int, radius: float, spread: float) -> list[Pose2D]:
    """物資姿勢 center に把持隊形を置いたときの各ロボットの目標姿勢."""
    return [Pose2D(*_xy(center.transform_point(o)), center.th) for o in grasp_offsets(n, radius, spread)]


def _xy(v: Vec2) -> tuple[float, float]:
    return v.x, v.y


def push_pose(cargo: CargoObject, goal: Vec2, lead: float, last_heading: float,
              goal_tol: float) -> tuple[Pose2D, float]:
    """押し出し用の「仮想物資姿勢」と、物資から目標までの距離を返す.

    実測の物資位置から目標方向へ lead だけ先行した点を隊形中心にする。
    隊形中心が物資より先行するので、ロボットは物資に少し食い込み、押す力が生まれる。
    目標に十分近づいたら先行量を 0 にして停止する。
    """
    d = goal - cargo.pos
    dist = d.norm()
    if dist <= goal_tol or dist < 1e-9:
        return Pose2D(cargo.x, cargo.y, last_heading), dist
    heading = d.angle()
    ahead = min(lead, dist)
    return Pose2D(cargo.x + math.cos(heading) * ahead,
                  cargo.y + math.sin(heading) * ahead, heading), dist


def heading_to(cargo: CargoObject, goal: Vec2, last_heading: float, goal_tol: float) -> float:
    """物資から目標へ向かう向き。目標に近いときはノイズで暴れないよう直前値を保つ."""
    d = goal - cargo.pos
    if d.norm() <= goal_tol:
        return last_heading
    return wrap_angle(d.angle())
