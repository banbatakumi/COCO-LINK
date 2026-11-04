"""協調物資運搬モード: 複数台で物資（青）を囲むように押し、目標地点まで運ぶ.

アルゴリズム (docs/control.md §7):
  1. フィールドクリックで物資を選び、次のクリックで目標地点を決める
     （auto_select=True なら開始時にフィールド中央に近い物資を自動選択）
  2. 物資の後方（目標と反対側）に、半円弧状の「把持隊形」を作る
  3. 隊形中心を物資位置より少しだけ目標方向へ先行させ、その分だけ押し込む
  4. 押し出し中は Kanayama 軌道追従で把持スロットを追い、safety=False で超音波減速を切る
     （物資を前方の障害物と誤検知して止まってしまうため）
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass

from ..common.geometry import Pose2D, Vec2
from ..control.formation import assign_slots
from ..control.planning import PathTracker, plan_path
from ..control.pose_tracking import GoToPoseGains, go_to_pose, kanayama_tracking, saturate
from ..control.transport import (
    CargoObject,
    collect_cargo,
    grasp_radius,
    grasp_slots,
    heading_to,
    nearest_cargo,
    push_pose,
    track_cargo,
)
from .base import Mode, RobotCommand, param
from .registry import register_mode

APPROACH_TIMEOUT_S = 12.0     # 把持隊形に揃わなくても、この時間で押し始める
LOST_TIMEOUT_S = 0.5          # 物資を見失ってから停止するまで
ROUTE_REPLAN_S = 1.0          # 物資の搬送経路を計画し直す周期 [s]
ROUTE_LOOKAHEAD = 0.35        # 物資の経路上で見る先読み距離 [m]
CARGO_CLEARANCE = 0.06        # 物資と障害物の安全余白 [m]


@register_mode
class CooperativeTransportMode(Mode):
    name = "協調物資運搬"
    description = ("物資（青）の後方に 1〜3 台が把持隊形を組み、協力して押しながら目標地点まで運ぶ。"
                   "開始後、物資をクリックして選択し、次に目標地点をクリック。")

    @dataclass
    class Params:
        goal_x: float = param(2.2, "目標 x [m]", min=0.0, max=10.0, step=0.05)
        goal_y: float = param(1.0, "目標 y [m]", min=0.0, max=10.0, step=0.05)
        max_speed: float = param(0.25, "最大速度 [m/s]", min=0.05, max=0.5, step=0.05)
        push_speed: float = param(0.10, "押し出し速度 [m/s]", min=0.02, max=0.30, step=0.01)
        max_robots: int = param(3, "最大台数", min=1, max=5, step=1)
        margin: float = param(0.01, "物資との余白 [m]", min=0.0, max=0.10, step=0.005)
        lead: float = param(0.05, "押し込み量 [m]", min=0.0, max=0.10, step=0.005)
        spread_deg: float = param(55.0, "把持の広がり [deg]", min=10.0, max=100.0, step=5.0)
        goal_tol: float = param(0.08, "目標許容誤差 [m]", min=0.02, max=0.30, step=0.01)
        click_tol: float = param(0.30, "クリック判定 [m]", min=0.05, max=0.80, step=0.05)
        auto_select: bool = param(True, "物資を自動選択")
        track: bool = param(True, "物資を追従")
        led_show: bool = param(True, "LED 演出")

    def __init__(self, params=None):
        super().__init__(params)
        self.cargo: CargoObject | None = None
        self.assignment: dict[int, int] = {}
        self._team: tuple[int, ...] = ()
        self._phase = "select"          # select / approach / push / done
        self._heading = 0.0
        self._candidates: list[CargoObject] = []
        self._lost = False
        self._last_seen = 0.0
        self._phase_t = 0.0
        self._now = 0.0
        self._done_announced = False
        self._nav_targets: dict[int, Vec2] = {}
        self._cargo_path: list[Vec2] = []
        self._cargo_tracker: PathTracker | None = None
        self._cargo_goal: Vec2 | None = None
        self._cargo_plan_t = -math.inf

    # ------------------------------------------------------------ ライフサイクル
    def on_start(self, world) -> None:
        self.cargo = None
        self.assignment = {}
        self._team = ()
        self._phase = "select"
        self._heading = 0.0
        self._candidates = collect_cargo(world)
        self._lost = False
        self._last_seen = world.t
        self._phase_t = world.t
        self._now = world.t
        self._done_announced = False
        self._nav_targets = {}
        self._cargo_path = []
        self._cargo_tracker = None
        self._cargo_goal = None
        self._cargo_plan_t = -math.inf
        if self.params.auto_select:
            cargo = self._initial_cargo(world)
            if cargo is not None:
                self._select(cargo)
        self.status = ("物資をクリックして選択してください" if self.cargo is None
                       else "物資を選択しました。次のクリックで目標地点を指定できます")

    def on_stop(self, world) -> dict[int, RobotCommand]:
        self.cargo = None
        self.assignment = {}
        self._team = ()
        self._phase = "select"
        return super().on_stop(world)

    def navigation_targets(self, world):
        return dict(self._nav_targets)

    def update_params(self, params) -> None:
        goal_changed = (params.goal_x, params.goal_y) != (self.params.goal_x, self.params.goal_y)
        super().update_params(params)
        if goal_changed:
            self._phase = "push" if self._phase == "done" else self._phase
            self._done_announced = False
            self._cargo_plan_t = -math.inf

    def on_field_click(self, x: float, y: float) -> None:
        if self.cargo is None:
            cargo = nearest_cargo(x, y, self._candidates, self.params.click_tol)
            if cargo is None:
                self.status = "クリック位置に物資がありません（青い物資の近くをクリック）"
                return
            self._select(cargo)
            self.status = "物資を選択しました。次のクリックで目標地点を指定できます"
            return
        self.params.goal_x, self.params.goal_y = x, y
        self._phase = "push" if self._phase == "done" else self._phase
        self._done_announced = False
        self._cargo_plan_t = -math.inf
        self.status = f"目標地点を ({x:.2f}, {y:.2f}) に設定しました"

    # ------------------------------------------------------------ 内部
    def _initial_cargo(self, world) -> CargoObject | None:
        if not self._candidates:
            return None
        center = Vec2(world.field_w / 2.0, world.field_h / 2.0)
        return min(self._candidates, key=lambda c: (c.pos - center).norm())

    def _select(self, cargo: CargoObject) -> None:
        self.cargo = cargo
        self.assignment = {}
        self._team = ()
        self._phase = "approach"
        self._phase_t = self._now
        self._last_seen = self._now
        self._lost = False
        self._done_announced = False
        self._cargo_path = []
        self._cargo_tracker = None
        self._cargo_goal = None
        self._cargo_plan_t = -math.inf
        goal = Vec2(self.params.goal_x, self.params.goal_y)
        self._heading = heading_to(cargo, goal, 0.0, self.params.goal_tol)

    def _retarget(self) -> None:
        """隊形中心とスロットを計算する."""
        p = self.params
        cargo = self.cargo
        assert cargo is not None
        goal = Vec2(p.goal_x, p.goal_y)
        dist = (goal - cargo.pos).norm()
        # 直線ではなく、障害物を避ける搬送経路の先読み点へ押し出す
        carrot = self._route_carrot(goal)
        if self._phase == "done":
            center = Pose2D(cargo.x, cargo.y, self._heading)
        elif self._phase == "push":
            center, _ = push_pose(cargo, carrot, p.lead, self._heading, p.goal_tol)
            self._heading = center.th
        else:
            self._heading = heading_to(cargo, carrot, self._heading, p.goal_tol)
            center = Pose2D(cargo.x, cargo.y, self._heading)
        n = len(self._team)
        body_r = 0.06
        if n:
            body_r = max(self._body_radius[rid] for rid in self._team)
        radius = grasp_radius(cargo.r, body_r, p.margin, n, math.radians(p.spread_deg))
        self._radius = radius
        self._center = center
        self._dist = dist
        self._slots = grasp_slots(center, n, radius, math.radians(p.spread_deg)) if n else []

    def _update_cargo_route(self, world, robots, team, goal: Vec2) -> None:
        """物資を押す経路を、障害物・人・他のロボットを避けて計画する."""
        cargo = self.cargo
        assert cargo is not None
        moved = self._cargo_goal is None or (goal - self._cargo_goal).norm() > 0.10
        if not moved and self._now - self._cargo_plan_t < ROUTE_REPLAN_S:
            return
        self._cargo_goal = goal
        self._cargo_plan_t = self._now

        obstacles: list[tuple[Vec2, float]] = []
        for o in world.obstacles + world.persons + world.cargo:
            if math.hypot(o.x - cargo.x, o.y - cargo.y) < 1e-6 and abs(o.r - cargo.r) < 1e-6:
                continue                      # 押している物資自身は除く
            obstacles.append((Vec2(o.x, o.y), o.r))
        for rid, r in robots.items():
            if rid not in team:               # 把持しているロボットは物資と一緒に動く
                obstacles.append((r.pose.pos, r.params.body_radius))

        path = plan_path(cargo.pos, goal, obstacles, robot_radius=cargo.r,
                         clearance=CARGO_CLEARANCE, field_wh=(world.field_w, world.field_h))
        self._cargo_path = path or []
        if self._cargo_path:
            self._cargo_tracker = PathTracker(ROUTE_LOOKAHEAD)
            self._cargo_tracker.set_path(self._cargo_path, cargo.pos)
        else:
            self._cargo_tracker = None

    def _route_carrot(self, goal: Vec2) -> Vec2:
        """搬送経路上で物資から ROUTE_LOOKAHEAD だけ先の点（無ければ最終目標）."""
        cargo = self.cargo
        if cargo is None or not self._cargo_path or self._cargo_tracker is None:
            return goal
        self._cargo_tracker.update(cargo.pos)
        carrot = self._cargo_tracker.target(cargo.pos)
        if carrot is None or (carrot - cargo.pos).norm() < 0.05:
            return goal
        return carrot

    # ------------------------------------------------------------ 制御周期
    def step(self, world, dt):
        p = self.params
        self._now = world.t
        goal = Vec2(p.goal_x, p.goal_y)
        self._candidates = collect_cargo(world)
        self.viz.targets.clear()
        self.viz.paths.clear()
        self.viz.points = []
        self._nav_targets.clear()

        if self.cargo is not None and p.track:
            cargo = track_cargo(self._candidates, self.cargo.pos)
            if cargo is not None:
                self.cargo = cargo
                self._last_seen = self._now
                self._lost = False
            else:
                self._lost = True
        elif self.cargo is None and p.auto_select:
            cargo = self._initial_cargo(world)
            if cargo is not None:
                self._select(cargo)

        robots = world.usable_robots()
        if not robots:
            self.status = "制御可能なロボットがいません（接続・ビジョン認識を確認）"
            return {}
        if self.cargo is None:
            self.status = "物資が見つかりません。青い物資の近くをクリックしてください"
            return {}
        if self._lost and self._now - self._last_seen > LOST_TIMEOUT_S:
            self.status = "物資を見失いました → 停止"
            return {rid: RobotCommand() for rid in robots}

        # 物資に近い順に最大 max_robots 台を搬送チームにする
        nearest = sorted(robots, key=lambda rid: (robots[rid].pose.pos - self.cargo.pos).norm())
        team = tuple(sorted(nearest[:max(1, p.max_robots)]))
        if team != self._team:
            self._team = team
            self._phase = "approach"
            self._phase_t = self._now
            self.assignment = {}
            self._done_announced = False
        self._body_radius = {rid: robots[rid].params.body_radius for rid in team}
        self._update_cargo_route(world, robots, team, goal)
        self._retarget()
        slots = self._slots
        if not self.assignment:
            self.assignment = assign_slots({rid: robots[rid].pose.pos for rid in team}, slots)

        cmds: dict[int, RobotCommand] = {rid: RobotCommand() for rid in robots if rid not in team}
        arrived = 0
        spread = math.radians(p.spread_deg)

        if self._phase == "approach":
            gains = GoToPoseGains(pos_tol=0.02)
            for rid in team:
                r = robots[rid]
                target = slots[self.assignment[rid]]
                v_max = min(p.max_speed, r.params.max_speed)
                v, w, ok = go_to_pose(r.pose, target, v_max, 3.0, gains)
                arrived += ok
                cmds[rid] = RobotCommand(vx=v, wz=w, safety=False)
                self._nav_targets[rid] = target.pos
            if arrived == len(team) or (self._now - self._phase_t > APPROACH_TIMEOUT_S
                                        and arrived >= max(1, len(team) - 1)):
                self._phase = "push"
                self._phase_t = self._now
                self._nav_targets.clear()      # 押し出し中はモード固有の制御に任せる
                for cmd in cmds.values():
                    cmd.buzzer = "ok"
            self.status = f"把持隊形へ移動: {arrived}/{len(team)} 台"

        elif self._phase == "push":
            v_ref = p.push_speed
            if self._dist < 0.25:                       # 目標の手前で減速
                v_ref *= max(0.25, self._dist / 0.25)
            for rid in team:
                r = robots[rid]
                target = slots[self.assignment[rid]]
                v, w = kanayama_tracking(r.pose, target, v_ref, 0.0)
                v, w = saturate(v, w, min(p.max_speed, r.params.max_speed), 3.0)
                cmds[rid] = RobotCommand(vx=v, wz=w, safety=False)
            self.status = f"搬送中: 目標まで {self._dist:.2f} m"
            if self._dist <= p.goal_tol:
                self._phase = "done"
                self._done_announced = False
                self.status = f"✓ 搬送完了（誤差 {self._dist:.2f} m）"

        else:                                            # done
            for rid in team:
                cmds[rid] = RobotCommand(safety=False)
            self.status = f"✓ 搬送完了（誤差 {self._dist:.2f} m）"
            if not self._done_announced:
                for cmd in cmds.values():
                    cmd.buzzer = "chime"
                self._done_announced = True

        # 目標を通過してしまった／目標を変えたときは搬送を再開
        if self._phase == "done" and self._dist > p.goal_tol + 0.02:
            self._phase = "push"
            self._done_announced = False

        # 表示と LED
        for rid in team:
            self.viz.targets[rid] = slots[self.assignment[rid]]
        self.viz.points = [(self.cargo.x, self.cargo.y, "物資"), (goal.x, goal.y, "目標")]
        self.viz.paths["搬送経路"] = ([(p.x, p.y) for p in self._cargo_path]
                                       if self._cargo_path
                                       else [(self.cargo.x, self.cargo.y), (goal.x, goal.y)])
        if self._slots:
            arc = [(self._center.x + self._radius * math.cos(math.pi + a),
                    self._center.y + self._radius * math.sin(math.pi + a))
                   for a in (-spread + 2.0 * spread * k / 24 for k in range(25))]
            self.viz.paths["把持隊形"] = arc
        if p.led_show:
            for i, rid in enumerate(team):
                hue = i / max(1, len(team))
                rgb = tuple(int(c * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, 1.0))
                if self._phase == "push":
                    cmds[rid].led = (*rgb, "solid")
                elif self._phase == "done":
                    cmds[rid].led = (0, 180, 80, "breath")
                else:
                    cmds[rid].led = (255, 180, 0, "blink")
        if self._lost:
            self.status += "（物資を見失い、最後の位置で継続）"
        return cmds
