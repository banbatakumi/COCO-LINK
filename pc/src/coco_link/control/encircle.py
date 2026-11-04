"""対象を円形に包囲するための目標姿勢生成と、クリック位置からの対象選択. docs/control.md §6."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..common.geometry import Pose2D, Vec2, wrap_angle

if TYPE_CHECKING:
    from ..fleet.world_model import World


@dataclass(frozen=True)
class TargetObject:
    """包囲対象の候補（ビジョンが見つけた人・物資・障害物）."""

    kind: str          # "person" / "cargo" / "obstacle"
    x: float
    y: float
    r: float

    @property
    def pos(self) -> Vec2:
        return Vec2(self.x, self.y)


def collect_targets(world: World) -> list[TargetObject]:
    """World に含まれる包囲対象の候補を列挙する."""
    out: list[TargetObject] = []
    for kind, objects in (("person", world.persons), ("cargo", world.cargo), ("obstacle", world.obstacles)):
        out.extend(TargetObject(kind, o.x, o.y, o.r) for o in objects)
    return out


def nearest_target(x: float, y: float, targets: list[TargetObject],
                   max_surface_dist: float = 0.25) -> TargetObject | None:
    """クリック位置 (x, y) に最も近い対象を返す（対象の表面からの距離で判定）.

    対象の中心ではなく表面からの距離を使うので、大きい障害物の端をクリックしても選べる。
    max_surface_dist より遠ければ None（何も選択しない）。
    """
    best: TargetObject | None = None
    best_d = math.inf
    for t in targets:
        d = max(0.0, math.hypot(x - t.x, y - t.y) - t.r)
        if d < best_d:
            best, best_d = t, d
    return best if best is not None and best_d <= max_surface_dist else None


def track_target(targets: list[TargetObject], kind: str, last_pos: Vec2,
                 gate: float = 1.0) -> TargetObject | None:
    """同じ種類の対象のうち、直前の位置に最も近いものを返す（追従用）.

    人や物資が動いても同じ対象を追い続ける。gate より離れていたら見失ったとみなす。
    """
    same = [t for t in targets if t.kind == kind]
    if not same:
        return None
    best = min(same, key=lambda t: (t.pos - last_pos).norm())
    return best if (best.pos - last_pos).norm() <= gate else None


def ring_radius(target_r: float, body_radius: float, margin: float, n_robots: int,
                robot_gap: float = 0.04, min_radius: float = 0.18) -> float:
    """包囲リングの半径 [m].

    対象の表面から body_radius + margin 離れた位置を基本とし、
    台数が多いときはリング上で機体同士が重ならないように広げる。
    """
    fit = n_robots * (2.0 * body_radius + robot_gap) / (2.0 * math.pi)
    return max(min_radius, target_r + body_radius + margin, fit)


def encircle_slots(center: Vec2, n: int, radius: float, phase: float = 0.0,
                   face_center: bool = True) -> list[Pose2D]:
    """中心のまわりに等間隔に並ぶ n 台分の目標姿勢を作る.

    phase はリングの回転角 [rad]。face_center=True なら各ロボットは中心を向く。
    """
    if n <= 0:
        return []
    slots: list[Pose2D] = []
    for i in range(n):
        a = phase + 2.0 * math.pi * i / n
        x = center.x + radius * math.cos(a)
        y = center.y + radius * math.sin(a)
        th = wrap_angle(a + math.pi) if face_center else wrap_angle(a)
        slots.append(Pose2D(x, y, th))
    return slots
