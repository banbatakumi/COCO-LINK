"""2次元の幾何計算.

座標系は docs/protocol.md §4 に従う（フィールド座標: x右, y上, θは反時計回り正）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    """角度を (-π, π] に正規化する."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


@dataclass(frozen=True, slots=True)
class Vec2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, o: Vec2) -> Vec2:
        return Vec2(self.x + o.x, self.y + o.y)

    def __sub__(self, o: Vec2) -> Vec2:
        return Vec2(self.x - o.x, self.y - o.y)

    def __mul__(self, k: float) -> Vec2:
        return Vec2(self.x * k, self.y * k)

    __rmul__ = __mul__

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def rotated(self, th: float) -> Vec2:
        c, s = math.cos(th), math.sin(th)
        return Vec2(c * self.x - s * self.y, s * self.x + c * self.y)


@dataclass(frozen=True, slots=True)
class Pose2D:
    """平面上の位置と向き."""

    x: float = 0.0
    y: float = 0.0
    th: float = 0.0

    @property
    def pos(self) -> Vec2:
        return Vec2(self.x, self.y)

    def compose(self, local: Pose2D) -> Pose2D:
        """self 座標系で表された local を、self の親座標系へ変換する (self ⊕ local)."""
        c, s = math.cos(self.th), math.sin(self.th)
        return Pose2D(
            self.x + c * local.x - s * local.y,
            self.y + s * local.x + c * local.y,
            wrap_angle(self.th + local.th),
        )

    def inverse(self) -> Pose2D:
        c, s = math.cos(self.th), math.sin(self.th)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, wrap_angle(-self.th))

    def relative_to(self, ref: Pose2D) -> Pose2D:
        """self を ref 座標系で表す (ref⁻¹ ⊕ self)."""
        return ref.inverse().compose(self)

    def distance_to(self, other: Pose2D | Vec2) -> float:
        return math.hypot(other.x - self.x, other.y - self.y)

    def transform_point(self, p: Vec2) -> Vec2:
        """self 座標系の点を親座標系へ."""
        return self.pos + p.rotated(self.th)
