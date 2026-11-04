"""包囲モード: クリックした人・物資・障害物を全ロボットで等間隔に取り囲む.

アルゴリズム (docs/control.md §6):
  1. フィールドクリック位置に最も近いビジョン対象（緑/青/赤）を選択する
  2. 対象のまわりに半径 R のリングを作り、n 台分のスロットを等間隔に配置する
  3. ロボット集合が変わったときだけハンガリアン法でスロットを割り当て直す
  4. 各ロボットは自分のスロットへ go_to_pose で向かう（既定では中心を向く）
  5. 対象が動く場合は同じ種類のうち直前位置に最も近いものを追い続ける
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass

from ..common.geometry import Vec2
from ..control.encircle import collect_targets, encircle_slots, nearest_target, ring_radius, track_target
from ..control.formation import assign_slots
from ..control.pose_tracking import GoToPoseGains, go_to_pose
from ..control.safety import avoid_robots
from .base import Mode, RobotCommand, param
from .registry import register_mode

TARGET_LABELS = {"person": "人", "cargo": "物資", "obstacle": "障害物"}


@register_mode
class EncircleMode(Mode):
    name = "包囲"
    description = ("フィールドでクリックした人（緑）・物資（青）・障害物（赤）を、"
                   "全ロボットで等間隔に取り囲む。開始後に対象の近くをクリック。")

    @dataclass
    class Params:
        margin: float = param(0.15, "対象との余白 [m]", min=0.03, max=0.8, step=0.01)
        max_speed: float = param(0.25, "最大速度 [m/s]", min=0.05, max=0.5, step=0.05)
        phase_deg: float = param(0.0, "配置の回転 [deg]", min=-180.0, max=180.0, step=5.0)
        track: bool = param(True, "対象を追従")
        face_center: bool = param(True, "中心を向く")
        click_tolerance: float = param(0.25, "クリック判定 [m]", min=0.05, max=0.6, step=0.05)
        led_show: bool = param(True, "LED 演出")

    def __init__(self, params=None):
        super().__init__(params)
        self.assignment: dict[int, int] = {}
        self._ids: tuple[int, ...] = ()
        self.target_kind: str | None = None
        self.target_pos: Vec2 | None = None
        self.target_r: float = 0.0
        self._candidates = []
        self._lost = False
        self._just_selected = False
        self._nav_targets: dict[int, Vec2] = {}
        self._nav_speeds: dict[int, float] = {}

    def on_start(self, world) -> None:
        self.assignment = {}
        self.nav.reset()
        self._ids = ()
        self.target_kind = None
        self.target_pos = None
        self.target_r = 0.0
        self._lost = False
        self._just_selected = False
        self._candidates = collect_targets(world)
        self.status = "対象をクリックしてください（人・物資・障害物の近く）"

    def on_field_click(self, x: float, y: float) -> None:
        target = nearest_target(x, y, self._candidates, self.params.click_tolerance)
        if target is None:
            self.status = "クリック位置に対象がありません（人・物資・障害物の近くをクリック）"
            return
        self.target_kind = target.kind
        self.target_pos = target.pos
        self.target_r = target.r
        self._ids = ()            # 対象が変わったので割り当て直す
        self._lost = False
        self._just_selected = True
        self.status = f"{TARGET_LABELS[target.kind]}を包囲します"

    def navigation_targets(self, world):
        return dict(self._nav_targets)

    def navigation_speeds(self, world):
        return dict(self._nav_speeds)

    def on_stop(self, world) -> dict[int, RobotCommand]:
        self.assignment = {}
        self._ids = ()
        return super().on_stop(world)

    def step(self, world, dt):
        p = self.params
        self._nav_targets.clear()
        self._nav_speeds.clear()
        self._candidates = collect_targets(world)

        if self.target_kind is None or self.target_pos is None:
            self.status = "対象をクリックしてください（人・物資・障害物の近く）"
            return {}

        if p.track:
            found = track_target(self._candidates, self.target_kind, self.target_pos)
            if found is not None:
                self.target_pos, self.target_r = found.pos, found.r
                self._lost = False
            else:
                self._lost = True

        robots = world.usable_robots()
        if not robots:
            self.status = "制御可能なロボットがいません（接続・ビジョン認識を確認）"
            return {}

        body_r = max(r.params.body_radius for r in robots.values())
        radius = ring_radius(self.target_r, body_r, p.margin, len(robots))
        slots = encircle_slots(self.target_pos, len(robots), radius,
                               math.radians(p.phase_deg), p.face_center)
        ids = tuple(sorted(robots))
        if ids != self._ids:
            self.assignment = assign_slots({rid: r.pose.pos for rid, r in robots.items()}, slots)
            self._ids = ids

        gains = GoToPoseGains()
        cmds: dict[int, RobotCommand] = {}
        arrived = 0
        self.viz.targets.clear()
        self.viz.paths.clear()
        self.viz.points = [(self.target_pos.x, self.target_pos.y,
                            TARGET_LABELS.get(self.target_kind, self.target_kind))]
        for rid, r in robots.items():
            target = slots[self.assignment[rid]]
            self.viz.targets[rid] = target
            v_max = min(p.max_speed, r.params.max_speed)
            self._nav_targets[rid] = target.pos
            self._nav_speeds[rid] = v_max
            v, w, ok = go_to_pose(r.pose, target, v_max, 3.0, gains)
            others = [o.pose.pos for oid, o in robots.items() if oid != rid]
            v, w = avoid_robots(r.pose, v, w, others, r.params.body_radius)
            arrived += ok
            cmd = RobotCommand(vx=v, wz=w)
            if p.led_show:
                hue = self.assignment[rid] / max(1, len(robots))
                rgb = tuple(int(c * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, 1.0))
                cmd.led = (*rgb, "solid") if ok else (255, 255, 255, "blink")
            if self._just_selected:
                cmd.buzzer = "ok"
            cmds[rid] = cmd
        self._just_selected = False

        # リング（見た目用）
        ring = [(self.target_pos.x + radius * math.cos(2.0 * math.pi * k / 48),
                 self.target_pos.y + radius * math.sin(2.0 * math.pi * k / 48))
                for k in range(48)]
        self.viz.paths["包囲リング"] = ring

        label = TARGET_LABELS.get(self.target_kind, self.target_kind)
        self.status = f"{label}を包囲: {arrived}/{len(robots)} 台配置" + (" ✓ 完成" if arrived == len(robots) else "")
        if self._lost:
            self.status += "（対象を見失い、最後の位置を維持）"
        return cmds
