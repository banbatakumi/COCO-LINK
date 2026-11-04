"""経路計画: 占有格子地図・A*・経路の平滑化・経路追従. docs/control.md §8.

ロボットを円で近似し、障害物・人・物資・他のロボットを「機体半径 + 安全余白」だけ
膨らませた占有格子にする。格子の上で A* を解き、見通しのよい直線で経路を平滑化する。
計画した経路は Pure Pursuit（`path_follow.py`）で追従する。
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Iterable
from dataclasses import dataclass

from ..common.geometry import Vec2
from .path_follow import lookahead_point

NEIGHBORS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
)


@dataclass(frozen=True, slots=True)
class CircleObstacle:
    """経路計画で避ける円（中心と半径）."""

    pos: Vec2
    r: float


def as_circle(item: CircleObstacle | tuple[Vec2, float]) -> CircleObstacle:
    """(中心, 半径) のタプルも受け付ける（`World.obstacle_points()` など）."""
    if isinstance(item, CircleObstacle):
        return item
    return CircleObstacle(Vec2(item[0].x, item[0].y), float(item[1]))


class OccupancyGrid:
    """フィールドを一定の解像度で区切った占有格子地図.

    `blocked[iy * nx + ix]` が 1 のセルにはロボットの中心を置けない。
    """

    def __init__(self, width: float, height: float, resolution: float = 0.05):
        if width <= 0.0 or height <= 0.0:
            raise ValueError("field size must be positive")
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.width = float(width)
        self.height = float(height)
        self.res = float(resolution)
        self.nx = max(1, math.ceil(self.width / self.res))
        self.ny = max(1, math.ceil(self.height / self.res))
        self.blocked = bytearray(self.nx * self.ny)

    # ------------------------------------------------------------ 座標変換
    def index(self, ix: int, iy: int) -> int:
        return iy * self.nx + ix

    def inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def world_to_cell(self, p: Vec2) -> tuple[int, int]:
        ix = min(self.nx - 1, max(0, int(p.x / self.res)))
        iy = min(self.ny - 1, max(0, int(p.y / self.res)))
        return ix, iy

    def cell_center(self, ix: int, iy: int) -> Vec2:
        return Vec2((ix + 0.5) * self.res, (iy + 0.5) * self.res)

    # ------------------------------------------------------------ 占有状態
    def block_cell(self, ix: int, iy: int) -> None:
        if self.inside(ix, iy):
            self.blocked[self.index(ix, iy)] = 1

    def is_blocked_cell(self, ix: int, iy: int) -> bool:
        return not self.inside(ix, iy) or bool(self.blocked[self.index(ix, iy)])

    def block_circle(self, center: Vec2, radius: float) -> None:
        """中心から半径 radius 以内のセルを占有にする（ロボット中心の進入禁止領域）."""
        r = max(0.0, float(radius))
        ix0 = max(0, math.floor((center.x - r) / self.res - 0.5))
        ix1 = min(self.nx - 1, math.ceil((center.x + r) / self.res - 0.5))
        iy0 = max(0, math.floor((center.y - r) / self.res - 0.5))
        iy1 = min(self.ny - 1, math.ceil((center.y + r) / self.res - 0.5))
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                if (self.cell_center(ix, iy) - center).norm() <= r:
                    self.blocked[self.index(ix, iy)] = 1

    def block_walls(self, margin: float) -> None:
        """壁から margin 以内のセルを占有にする."""
        for iy in range(self.ny):
            y = (iy + 0.5) * self.res
            near_y = y < margin or y > self.height - margin
            for ix in range(self.nx):
                x = (ix + 0.5) * self.res
                if near_y or x < margin or x > self.width - margin:
                    self.blocked[self.index(ix, iy)] = 1

    def is_free(self, p: Vec2) -> bool:
        ix, iy = self.world_to_cell(p)
        return not self.is_blocked_cell(ix, iy)

    def nearest_free(self, p: Vec2, max_radius: float = 1.0) -> tuple[int, int] | None:
        """p に最も近い空きセル. 見つからなければ None."""
        ix, iy = self.world_to_cell(p)
        if not self.is_blocked_cell(ix, iy):
            return ix, iy
        max_ring = max(1, math.ceil(max_radius / self.res))
        best: tuple[float, int, int] | None = None
        for ring in range(1, max_ring + 1):
            found = False
            for dy in range(-ring, ring + 1):
                for dx in range(-ring, ring + 1):
                    if max(abs(dx), abs(dy)) != ring:
                        continue
                    jx, jy = ix + dx, iy + dy
                    if self.is_blocked_cell(jx, jy):
                        continue
                    d = (self.cell_center(jx, jy) - p).norm()
                    if best is None or d < best[0]:
                        best = (d, jx, jy)
                    found = True
            if found:                      # 近いリングで見つかったらそこで打ち切る
                break
        return None if best is None else (best[1], best[2])

    def line_clear(self, a: Vec2, b: Vec2) -> bool:
        """線分 ab が占有セルを通らないか（半セル間隔でサンプリング）."""
        d = (b - a).norm()
        n = max(1, math.ceil(d / (self.res * 0.5)))
        for k in range(n + 1):
            p = a + (b - a) * (k / n)
            if not self.is_free(p):
                return False
        return True

    # ------------------------------------------------------------ 探索
    def astar(self, start: Vec2, goal: Vec2, max_expansions: int = 200_000) -> list[Vec2] | None:
        """8 近傍 A*. 見つからなければ None. 返り値はセル中心の点列."""
        s = self.nearest_free(start)
        g = self.nearest_free(goal)
        if s is None or g is None:
            return None
        if s == g:
            return [self.cell_center(*g)]

        sx, sy = s
        gx, gy = g

        def heuristic(ix: int, iy: int) -> float:
            dx, dy = abs(ix - gx), abs(iy - gy)
            return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

        counter = 0
        open_heap: list[tuple[float, int, int, int]] = [(heuristic(sx, sy), counter, sx, sy)]
        g_score = {s: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        closed: set[tuple[int, int]] = set()

        while open_heap and len(closed) < max_expansions:
            _, _, ix, iy = heapq.heappop(open_heap)
            cell = (ix, iy)
            if cell in closed:
                continue
            if cell == g:
                return self._reconstruct(came_from, cell)
            closed.add(cell)
            for dx, dy, cost in NEIGHBORS:
                jx, jy = ix + dx, iy + dy
                if self.is_blocked_cell(jx, jy):
                    continue
                if dx and dy and (self.is_blocked_cell(ix + dx, iy) or self.is_blocked_cell(ix, iy + dy)):
                    continue               # 角を斜めに通り抜けない
                nxt = (jx, jy)
                if nxt in closed:
                    continue
                tentative = g_score[cell] + cost
                if tentative < g_score.get(nxt, math.inf):
                    g_score[nxt] = tentative
                    came_from[nxt] = cell
                    counter += 1
                    heapq.heappush(open_heap, (tentative + heuristic(jx, jy), counter, jx, jy))
        return None

    def _reconstruct(self, came_from: dict[tuple[int, int], tuple[int, int]],
                     cell: tuple[int, int]) -> list[Vec2]:
        cells = [cell]
        while cell in came_from:
            cell = came_from[cell]
            cells.append(cell)
        cells.reverse()
        return [self.cell_center(ix, iy) for ix, iy in cells]


def smooth_path(path: list[Vec2], grid: OccupancyGrid) -> list[Vec2]:
    """見通しのよい所まで一気に進む折れ線にする（ greedy shortcut ）."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not grid.line_clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def densify(path: list[Vec2], step: float = 0.02) -> list[Vec2]:
    """点列を step 間隔に細かくする（追従・描画用）."""
    if not path:
        return []
    if step <= 0.0:
        return list(path)
    out = [path[0]]
    for a, b in zip(path, path[1:], strict=False):
        d = (b - a).norm()
        n = max(1, math.ceil(d / step))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return out


def path_length(path: list[Vec2]) -> float:
    """折れ線の道のり [m]."""
    return sum((b - a).norm() for a, b in zip(path, path[1:], strict=False))


def point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> float:
    """点 p から線分 ab までの最短距離."""
    ab = b - a
    denom = ab.x * ab.x + ab.y * ab.y
    if denom < 1e-12:
        return (p - a).norm()
    t = max(0.0, min(1.0, ((p - a).x * ab.x + (p - a).y * ab.y) / denom))
    return (p - (a + ab * t)).norm()


def circle_path_clear(path: list[Vec2], center: Vec2, radius: float) -> bool:
    """経路（折れ線）が円の内側を通らないか."""
    if not path:
        return True
    if len(path) == 1:
        return (path[0] - center).norm() >= radius
    return all(point_segment_distance(center, a, b) >= radius
               for a, b in zip(path, path[1:], strict=False))


def plan_path(start: Vec2, goal: Vec2,
              obstacles: Iterable[CircleObstacle | tuple[Vec2, float]],
              robot_radius: float, clearance: float = 0.08, resolution: float = 0.05,
              field_wh: tuple[float, float] = (3.0, 2.0), smooth: bool = True) -> list[Vec2] | None:
    """start から goal までの衝突しない経路を A* で計画する.

    obstacles は (中心, 半径) の列。ロボットの中心が「相手の半径 + 機体半径 + 安全余白」
    に入らないように格子を占有にする。返り値は始点・終点を含む点列（見つからなければ None）。
    """
    grid = OccupancyGrid(field_wh[0], field_wh[1], resolution)
    grid.block_walls(robot_radius + clearance)
    for item in obstacles:
        circle = as_circle(item)
        grid.block_circle(circle.pos, circle.r + robot_radius + clearance)
    path = grid.astar(start, goal)
    if path is None:
        return None
    if smooth:
        path = smooth_path(path, grid)
    if grid.is_free(start):
        path[0] = start
    if grid.is_free(goal):
        path[-1] = goal
    return path


class PathTracker:
    """計画した経路を Pure Pursuit で追うための状態（進行点と注視点）."""

    def __init__(self, lookahead: float = 0.20):
        self.lookahead = float(lookahead)
        self.path: list[Vec2] = []
        self.index = 0

    def set_path(self, path: list[Vec2], pos: Vec2 | None = None) -> None:
        self.path = list(path)
        self.index = 0
        if pos is not None:
            self.update(pos)

    def update(self, pos: Vec2) -> None:
        """経路上を前進した分だけ index を進める（後戻りしない）."""
        while self.index + 1 < len(self.path):
            d_now = (self.path[self.index] - pos).norm()
            d_next = (self.path[self.index + 1] - pos).norm()
            if d_next < d_now - 1e-9:
                self.index += 1
            else:
                break

    def target(self, pos: Vec2) -> Vec2 | None:
        """経路上で現在位置から lookahead だけ先の点."""
        if not self.path:
            return None
        return lookahead_point(self.path[self.index:], pos, self.lookahead)

    def remaining(self, pos: Vec2) -> float:
        """経路の残りの道のり [m]（現在位置から経路の終点まで）."""
        if not self.path:
            return math.inf
        if self.index >= len(self.path) - 1:
            return (self.path[-1] - pos).norm()
        nxt = self.index + 1
        return (self.path[nxt] - pos).norm() + path_length(self.path[nxt:])

    def finished(self, pos: Vec2, tol: float = 0.06) -> bool:
        return bool(self.path) and (self.path[-1] - pos).norm() <= tol
