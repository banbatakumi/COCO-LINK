"""待機モード: 全ロボットを停止させ続ける."""

from __future__ import annotations

from .base import Mode, RobotCommand
from .registry import register_mode


@register_mode
class IdleMode(Mode):
    name = "待機"
    description = "全ロボットに停止指令を送り続ける。"

    def step(self, world, dt):
        self.status = f"{len(world.usable_robots())} 台待機中"
        return {rid: RobotCommand() for rid in world.usable_robots()}
