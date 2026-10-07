"""フォーメーションモード: 指定した隊形にロボットを整列させる（回転させてパフォーマンスも可能）.

アルゴリズム (docs/control.md §3):
  1. 隊形中心姿勢 C(t) と隊形形状からスロット目標姿勢を生成（仮想構造法）
  2. ロボット集合が変わったときだけハンガリアン法でスロットを割り当て直す
  3. 各ロボットは自分のスロットへ極座標フィードバックで向かう。近くのロボットは避ける
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass

from ..common.geometry import Pose2D, wrap_angle
from ..control.formation import SHAPES, assign_slots, slot_poses
from ..control.pose_tracking import GoToPoseGains, go_to_pose
from ..control.safety import avoid_robots
from .base import Mode, RobotCommand, param
from .registry import register_mode


@register_mode
class FormationMode(Mode):
    name = "フォーメーション"
    description = "隊形（横一列・縦一列・円・V字・格子・ハート）に整列する。フィールドをクリックすると隊形の中心が移動。"

    @dataclass
    class Params:
        shape: str = param("circle", "隊形", choices=SHAPES)
        spacing: float = param(0.25, "間隔 [m]", min=0.12, max=1.0, step=0.01)
        center_x: float = param(1.5, "中心 x [m]", min=0.0, max=10.0, step=0.05)
        center_y: float = param(1.0, "中心 y [m]", min=0.0, max=10.0, step=0.05)
        heading_deg: float = param(90.0, "向き [deg]", min=-180.0, max=180.0, step=5.0)
        rotate_dps: float = param(0.0, "回転速度 [deg/s]", min=-60.0, max=60.0, step=5.0)
        max_speed: float = param(0.25, "最大速度 [m/s]", min=0.05, max=0.5, step=0.05)
        led_show: bool = param(True, "LED 演出")

    def __init__(self, params=None):
        super().__init__(params)
        self.assignment: dict[int, int] = {}
        self._ids: tuple[int, ...] = ()
        self._heading = 0.0
        self._arrived_all = False

    def on_start(self, world) -> None:
        self._heading = math.radians(self.params.heading_deg)
        self._ids = ()
        self._arrived_all = False

    def update_params(self, params) -> None:
        shape_changed = params.shape != self.params.shape
        if params.heading_deg != self.params.heading_deg:
            self._heading = math.radians(params.heading_deg)
        super().update_params(params)
        if shape_changed:
            self._ids = ()   # 再割り当て

    def on_field_click(self, x: float, y: float) -> None:
        self.params.center_x, self.params.center_y = x, y

    def center(self) -> Pose2D:
        return Pose2D(self.params.center_x, self.params.center_y, self._heading)

    def step(self, world, dt):
        p = self.params
        robots = world.usable_robots()
        if not robots:
            self.status = "制御可能なロボットがいません（接続・ビジョン認識を確認）"
            return {}
        if p.rotate_dps and self._arrived_all:
            self._heading = wrap_angle(self._heading + math.radians(p.rotate_dps) * dt)
        slots = slot_poses(p.shape, len(robots), p.spacing, self.center())
        ids = tuple(sorted(robots))
        if ids != self._ids:
            self.assignment = assign_slots({i: r.pose.pos for i, r in robots.items()}, slots)
            self._ids = ids

        gains = GoToPoseGains()
        cmds: dict[int, RobotCommand] = {}
        arrived = 0
        self.viz.targets.clear()
        for rid, r in robots.items():
            target = slots[self.assignment[rid]]
            self.viz.targets[rid] = target
            v_max = min(p.max_speed, r.params.max_speed)
            v, w, ok = go_to_pose(r.pose, target, v_max, 3.0, gains)
            if self._arrived_all and p.rotate_dps:
                ok = r.pose.distance_to(target) < 0.05     # 回転中は位置だけで判定
            others = [o.pose.pos for oid, o in robots.items() if oid != rid]
            v, w = avoid_robots(r.pose, v, w, others, r.params.body_radius)
            arrived += ok
            cmd = RobotCommand(vx=v, wz=w)
            if p.led_show:
                hue = self.assignment[rid] / max(1, len(robots))
                rgb = tuple(int(c * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, 1.0))
                cmd.led = (*rgb, "solid") if ok else (255, 255, 255, "blink")
            cmds[rid] = cmd
        if arrived == len(robots) and not self._arrived_all:
            self._arrived_all = True
            for c in cmds.values():
                c.buzzer = "chime"
        elif arrived < len(robots) * 0.5 and not p.rotate_dps:
            self._arrived_all = False
        self.status = f"{p.shape}: {arrived}/{len(robots)} 台到着" + (" ✓ 完成" if self._arrived_all else "")
        return cmds
