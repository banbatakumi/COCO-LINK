"""人追従（隊列）モード: 先頭のロボットが人の後ろを、後続は前のロボットの軌跡をたどって一列で追従する.

アルゴリズム (docs/control.md §4):
  - 先頭: 人の軌跡 (Trail) を記録し、人から道のり follow_distance 後ろを目標に Pure Pursuit
  - i 番目: (i-1) 番目のロボットの軌跡を、道のり spacing 後ろで追う
    → 「人が通った道をそのままたどる」ので、障害物を避けた人の後をついて行ける（車列/カルガモ）
  - 速度は「前との道のり誤差」に比例 (P 制御)、超音波で前方が近いと減速
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..common.geometry import Vec2, wrap_angle
from ..control.path_follow import Trail, lookahead_point, pure_pursuit
from .base import Mode, RobotCommand, param
from .registry import register_mode


@register_mode
class FollowPersonMode(Mode):
    name = "人追従（隊列）"
    description = "人（緑）の後ろに一列の隊列を作って追従する。順番は開始時に人に近い順。"

    @dataclass
    class Params:
        follow_distance: float = param(0.40, "人との距離 [m]", min=0.2, max=1.5, step=0.05)
        spacing: float = param(0.25, "ロボット間隔 [m]", min=0.15, max=1.0, step=0.05)
        max_speed: float = param(0.30, "最大速度 [m/s]", min=0.05, max=0.5, step=0.05)
        k_speed: float = param(1.5, "速度ゲイン [1/s]", min=0.1, max=5.0, step=0.1)
        lookahead: float = param(0.12, "注視距離 [m]", min=0.05, max=0.5, step=0.01)
        led_show: bool = param(True, "LED 演出")

    def __init__(self, params=None):
        super().__init__(params)
        self.order: list[int] = []
        self.trails: dict[int | str, Trail] = {}

    def on_start(self, world) -> None:
        self.order = []
        self.trails = {"person": Trail()}

    def _person(self, world) -> Vec2 | None:
        if not world.persons:
            return None
        # 前回位置に一番近い人を追う（複数人いても乗り換えにくくする）
        trail = self.trails.get("person")
        last = trail.points[-1] if trail and trail.points else None
        ps = [Vec2(o.x, o.y) for o in world.persons]
        return min(ps, key=lambda p: (p - last).norm()) if last else ps[0]

    def step(self, world, dt):
        p = self.params
        robots = world.usable_robots()
        person = self._person(world)
        if person is None:
            self.status = "人（緑）が見つかりません → 停止"
            return {rid: RobotCommand() for rid in robots}
        # 隊列の順番を決める（新しく加わったロボットは最後尾へ）
        self.order = [rid for rid in self.order if rid in robots]
        new = sorted((rid for rid in robots if rid not in self.order),
                     key=lambda rid: (robots[rid].pose.pos - person).norm())
        if not self.order and new:
            # 初回は人に近い順。各ロボットの軌跡を初期化
            self.order = new
        else:
            self.order += new
        for rid in self.order:
            self.trails.setdefault(rid, Trail())

        self.trails["person"].add(person)
        cmds: dict[int, RobotCommand] = {}
        self.viz.paths.clear()
        self.viz.targets.clear()
        leader_key: int | str = "person"
        leader_pos = person
        for idx, rid in enumerate(self.order):
            r = robots[rid]
            pos = r.pose.pos
            trail = self.trails[leader_key]
            trail.prune_before(pos, p.lookahead * 0.5)
            gap = p.follow_distance if idx == 0 else p.spacing
            path_len = trail.length_from(pos, leader_pos)
            err = path_len - gap
            v = max(0.0, min(min(p.max_speed, r.params.max_speed), p.k_speed * err))
            pts = list(trail.points) + [leader_pos]
            goal = lookahead_point(pts, pos, p.lookahead)
            if goal is None or err <= 0.0:
                vx, wz = 0.0, 0.0
                # 停止中は前を向く
                d = leader_pos - pos
                if d.norm() > 1e-3:
                    e = wrap_angle(d.angle() - r.pose.th)
                    wz = max(-2.0, min(2.0, 2.0 * e)) if abs(e) > math.radians(15) else 0.0
            else:
                vx, wz = pure_pursuit(r.pose, goal, v)
            cmd = RobotCommand(vx=vx, wz=wz)
            if p.led_show:
                cmd.led = (0, 200, 80, "breath") if vx > 0.02 else (0, 120, 255, "solid")
            cmds[rid] = cmd
            self.viz.paths[f"trail→#{rid}"] = trail.as_list()
            self.trails[rid].add(pos)
            leader_key, leader_pos = rid, pos
        self.status = "隊列: 人 → " + " → ".join(f"#{rid}" for rid in self.order)
        return cmds
