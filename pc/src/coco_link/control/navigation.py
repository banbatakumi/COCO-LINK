"""全モード共通の経路計画ナビゲーション. docs/control.md §8.

各モードは「目標へ向かう速度指令」を返すだけでよい。このクラスが
  1. モードが返した目標点までの占有格子地図を作り（障害物・人・物資・他のロボット）
  2. A* で経路を計画し、Pure Pursuit で追従する速度指令に置き換える
  3. 目標が近い・経路が見つからないときはモードの指令に任せる（最終姿勢合わせなど）
という共通処理を担当する。モード側は `Mode.navigation_targets()` で目標点を返す。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..common.geometry import Vec2
from ..fleet.world_model import World
from ..robot.kinematics import integrate_unicycle
from .path_follow import pure_pursuit
from .planning import CircleObstacle, PathTracker, densify, plan_path
from .safety import avoid_circles

# 計画に使う円は (中心, 半径) のタプルでも CircleObstacle でもよい
Obstacle = CircleObstacle | tuple[Vec2, float]


@dataclass(frozen=True)
class NavigationConfig:
    """全モード共通のナビゲーション設定."""

    resolution: float = 0.05        # 占有格子の解像度 [m]
    clearance: float = 0.08         # 障害物との安全余白 [m]
    replan_s: float = 0.5           # 再計画の周期 [s]
    predict_s: float = 0.5          # 他のロボットの予測時間 [s]
    lookahead: float = 0.20         # Pure Pursuit の注視距離 [m]
    handover_distance: float = 0.12  # 目標がこの距離以内ならモードの指令に任せる [m]
    target_move_tol: float = 0.08   # 目標がこれ以上動いたら再計画 [m]
    avoid_influence: float = 0.25   # 反応的な回避を始める距離 [m]
    default_speed: float = 0.20     # モードが速度を指定しない場合の巡航速度 [m/s]


class Navigator:
    """モードの速度指令を、経路計画に基づく指令へ変換する."""

    def __init__(self, config: NavigationConfig | None = None):
        self.config = config or NavigationConfig()
        self._paths: dict[int, list[Vec2]] = {}
        self._trackers: dict[int, PathTracker] = {}
        self._targets: dict[int, Vec2] = {}
        self._planned_at: dict[int, float] = {}
        self.blocked: set[int] = set()          # 経路が見つからなかったロボット

    # ------------------------------------------------------------ 状態
    def reset(self) -> None:
        self._paths.clear()
        self._trackers.clear()
        self._targets.clear()
        self._planned_at.clear()
        self.blocked.clear()

    def paths(self) -> dict[int, list[Vec2]]:
        """ロボットごとの計画経路（GUI 描画用）."""
        return {rid: list(path) for rid, path in self._paths.items()}

    def _forget(self, rid: int) -> None:
        self._paths.pop(rid, None)
        self._trackers.pop(rid, None)
        self._targets.pop(rid, None)
        self._planned_at.pop(rid, None)

    # ------------------------------------------------------------ 適用
    def apply(self, world: World, cmds: dict, targets: dict[int, Vec2],
              speeds: dict[int, float] | None = None) -> dict:
        """cmds のうち targets に含まれるロボットを経路追従指令に置き換える.

        targets にないロボットの指令には触れない（停止・その場旋回・物資押しなど、
        モード固有の制御をそのまま使う）。`speeds` は経路追従中の巡航速度で、
        省略したロボットは `default_speed` を使う。

        モードの瞬間的な速度指令は使わない。姿勢制御が特異姿勢に近いときに
        指令速度がほぼ 0 になっても、計画経路に沿って進めるようにするため。
        """
        robots = world.usable_robots()
        speeds = speeds or {}
        self.blocked.clear()

        for rid, target in targets.items():
            cmd = cmds.get(rid)
            r = robots.get(rid)
            if cmd is None or r is None or cmd.wheel is not None:
                self._forget(rid)
                continue
            if (target - r.pose.pos).norm() <= self.config.handover_distance:
                self._forget(rid)
                continue

            speed = max(0.0, speeds.get(rid, self.config.default_speed))
            if speed <= 1e-3:                 # モードが明示的に停止を指示
                self._forget(rid)
                continue

            self._plan_if_needed(world, robots, rid, target)
            path = self._paths.get(rid)
            if not path:
                # 経路なし: モードの指令を反応的な回避だけ補助する
                cmd.vx, cmd.wz = avoid_circles(
                    r.pose, cmd.vx, cmd.wz, self._avoid_circles(world, robots, rid),
                    influence=self.config.avoid_influence)
                continue

            tracker = self._trackers[rid]
            tracker.update(r.pose.pos)
            goal = tracker.target(r.pose.pos)
            remaining = tracker.remaining(r.pose.pos)
            if (goal is None or remaining <= self.config.handover_distance
                    or tracker.finished(r.pose.pos, 0.04)):
                # 最後の数 cm と向き合わせはモードの制御に任せる
                self._forget(rid)
                continue

            v = min(speed, r.params.max_speed)
            v, w = pure_pursuit(r.pose, goal, v)
            v, w = avoid_circles(r.pose, v, w, self._avoid_circles(world, robots, rid),
                                 influence=self.config.avoid_influence)
            cmd.vx, cmd.wz = v, w

        for rid in list(self._paths):
            if rid not in targets:
                self._forget(rid)
        return cmds

    # ------------------------------------------------------------ 計画
    def _plan_if_needed(self, world: World, robots, rid: int, target: Vec2) -> None:
        last = self._planned_at.get(rid)
        old = self._targets.get(rid)
        moved = old is not None and (target - old).norm() > self.config.target_move_tol
        if last is not None and not moved and world.t - last < self.config.replan_s:
            return

        r = robots[rid]
        path = plan_path(
            r.pose.pos, target, self._plan_obstacles(world, robots, rid),
            robot_radius=r.params.body_radius,
            clearance=self.config.clearance,
            resolution=self.config.resolution,
            field_wh=(world.field_w, world.field_h),
        )
        self._planned_at[rid] = world.t
        self._targets[rid] = target
        if path is None:
            self._paths.pop(rid, None)
            self._trackers.pop(rid, None)
            self.blocked.add(rid)
            return

        dense = densify(path, 0.02)
        self._paths[rid] = dense
        tracker = self._trackers.get(rid)
        if tracker is None:
            tracker = PathTracker(self.config.lookahead)
            self._trackers[rid] = tracker
        tracker.lookahead = self.config.lookahead
        tracker.set_path(dense, r.pose.pos)

    def _plan_obstacles(self, world: World, robots, rid: int) -> list[Obstacle]:
        """経路計画で避ける円（自分以外のすべての物体）."""
        obstacles: list[Obstacle] = [
            (Vec2(o.x, o.y), o.r)
            for o in world.obstacles + world.persons + world.cargo
        ]
        for oid, other in robots.items():
            if oid == rid:
                continue
            obstacles.append((other.pose.pos, other.params.body_radius))
            if self.config.predict_s > 1e-6 and abs(other.vx) > 1e-3:
                ahead = integrate_unicycle(other.pose, other.vx, other.wz, self.config.predict_s)
                obstacles.append((ahead.pos, other.params.body_radius))
        return obstacles

    def _avoid_circles(self, world: World, robots, rid: int) -> list[tuple[Vec2, float]]:
        """反応的な回避に使う円（中心, 相手の半径 + 自分の機体半径）."""
        own = robots[rid].params.body_radius
        circles: list[tuple[Vec2, float]] = [
            (Vec2(o.x, o.y), o.r + own)
            for o in world.obstacles + world.persons + world.cargo
        ]
        for oid, other in robots.items():
            if oid != rid:
                circles.append((other.pose.pos, other.params.body_radius + own))
        return circles
