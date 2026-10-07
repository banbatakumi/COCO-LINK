"""テスト用ヘルパ: 通信を介さずにモード ⇄ シミュレータの閉ループを高速に回す.

モードの開発では、まずこれで収束を確かめ、その後 GUI で目視確認するとよい。
"""

from __future__ import annotations

from coco_link.control.safety import ultrasonic_speed_limit
from coco_link.fleet.world_model import RobotState, World
from coco_link.modes.base import Mode, RobotCommand
from coco_link.protocol import messages as m
from coco_link.sim.engine import PHYSICS_HZ, SimEngine


def world_from_engine(engine: SimEngine) -> World:
    """シミュレータの真値から World を作る（ビジョンが理想的な場合に相当）."""
    with engine.lock:
        w = World(t=engine.t, field_w=engine.config.field_w, field_h=engine.config.field_h, vision_age=0.0)
        for rid, r in engine.robots.items():
            tel = r.firmware.telemetry()
            w.robots[rid] = RobotState(robot_id=rid, connected=True, pose=r.pose, pose_source="vision",
                                       pose_age=0.0, vx=r.plant.vx, wz=r.plant.wz, telemetry=tel,
                                       params=engine.config.nominal)
        for b in engine.bodies:
            obj = m.DetectedObject(b.x, b.y, b.r)
            {"person": w.persons, "obstacle": w.obstacles, "cargo": w.cargo}[b.kind].append(obj)
    return w


def apply_commands(engine: SimEngine, world: World, cmds: dict[int, RobotCommand]) -> None:
    for rid, c in cmds.items():
        if c.wheel is not None:
            engine.deliver(rid, m.CmdWheel(*c.wheel))
            continue
        vx = c.vx
        r = world.robots.get(rid)
        if c.safety and r is not None:
            vx = ultrasonic_speed_limit(vx, r.us_front, r.us_rear)
        engine.deliver(rid, m.CmdVel(vx=vx, wz=c.wz))


def run_closed_loop(engine: SimEngine, mode: Mode, seconds: float, control_hz: float = 20.0,
                    on_step=None) -> World:
    """mode を control_hz で seconds 秒動かす. on_step(engine, t) で外乱（人の移動など）を与えられる."""
    steps_per_ctrl = round(PHYSICS_HZ / control_hz)
    mode.on_start(world_from_engine(engine))
    n = round(seconds * control_hz)
    for _ in range(n):
        world = world_from_engine(engine)
        apply_commands(engine, world, mode.step(world, 1.0 / control_hz))
        for _ in range(steps_per_ctrl):
            engine.step(1.0 / PHYSICS_HZ)
            if on_step:
                on_step(engine, engine.t)
    return world_from_engine(engine)
