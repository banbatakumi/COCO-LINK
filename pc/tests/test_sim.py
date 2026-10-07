import math
import random
import time

import pytest

from coco_link.common.config import NetworkConfig
from coco_link.common.geometry import Pose2D
from coco_link.net import UdpEndpoint
from coco_link.protocol import messages as m
from coco_link.sim import sensors
from coco_link.sim.engine import SimConfig, SimEngine
from coco_link.sim.scenario import load_scenario, save_scenario
from coco_link.sim.virtual_robot_node import VirtualRobotNode
from coco_link.sim.vision_emulator import VisionEmulator, VisionNoise


def test_ray_circle():
    assert sensors.ray_circle(0, 0, 1, 0, 2, 0, 0.5) == pytest.approx(1.5)
    assert sensors.ray_circle(0, 0, -1, 0, 2, 0, 0.5) is None
    assert sensors.ray_box(1, 1, 1, 0, 3, 2) == pytest.approx(2.0)


def test_ultrasonic_sees_wall_and_obstacle():
    rng = random.Random(0)
    pose = Pose2D(1.0, 1.0, 0.0)
    d = sensors.ultrasonic(pose, 0.06, 0.0, 30, 2.0, [], (3.0, 2.0), rng, noise=0)
    assert d == pytest.approx(3.0 - 1.06)
    d = sensors.ultrasonic(pose, 0.06, 0.0, 30, 2.0, [(1.6, 1.0, 0.1)], (3.0, 2.0), rng, noise=0)
    assert d == pytest.approx(0.44)
    assert sensors.ultrasonic(pose, 0.06, 0.0, 30, 0.5, [], (3.0, 2.0), rng) is None


def make_engine(**kw) -> SimEngine:
    return SimEngine(SimConfig(param_spread=0.0, **kw))


def drive(engine: SimEngine, rid: int, msg: m.Message, seconds: float):
    for i in range(int(seconds * 200)):
        if i % 10 == 0:
            engine.deliver(rid, msg)
        engine.step()


def test_robot_drives_forward():
    e = make_engine()
    e.add_robot(1, Pose2D(0.5, 1.0, 0.0))
    drive(e, 1, m.CmdVel(vx=0.2), 2.0)
    assert e.robots[1].pose.x == pytest.approx(0.5 + 0.2 * 2.0, abs=0.08)
    assert e.robots[1].pose.y == pytest.approx(1.0, abs=0.02)


def test_dragging_overrides_motion():
    e = make_engine()
    r = e.add_robot(1, Pose2D(0.5, 1.0, 0.0))
    r.dragging = True
    drive(e, 1, m.CmdVel(vx=0.2), 1.0)
    assert r.pose.x == pytest.approx(0.5)


def test_collisions_and_walls():
    e = make_engine()
    a = e.add_robot(1, Pose2D(1.0, 1.0, 0.0))
    b = e.add_robot(2, Pose2D(1.05, 1.0, 0.0))
    e.add_robot(3, Pose2D(-1.0, 5.0, 0.0))
    e.step()
    assert a.pose.distance_to(b.pose) >= 2 * a.params.body_radius - 1e-6
    p3 = e.robots[3].pose
    assert 0 < p3.x < 3.0 and 0 < p3.y < 2.0


def test_robot_pushes_cargo():
    e = make_engine()
    e.add_robot(1, Pose2D(0.5, 1.0, 0.0))
    cargo = e.add_body("cargo", 0.65, 1.0)
    drive(e, 1, m.CmdVel(vx=0.2), 1.5)
    assert cargo.x > 0.75


def test_individual_differences_are_deterministic():
    e1 = SimEngine(SimConfig(seed=3))
    e2 = SimEngine(SimConfig(seed=3))
    assert e1.add_robot(4).params == e2.add_robot(4).params
    assert e1.robots[4].params != e1.add_robot(5).params


def test_vision_emulator_latency_and_content():
    e = make_engine()
    e.add_robot(1, Pose2D(1.0, 1.0, 0.5))
    e.add_body("person", 2.0, 1.0)
    v = VisionEmulator(e, VisionNoise(dropout=0.0, latency_s=0.1))
    out = []
    t = 0.0
    for _ in range(40):
        t += 0.005
        out += v.update(0.005, t)
    # 30 Hz, 遅延 0.1 s → 0.2 s では最初の 1-3 フレーム
    assert 1 <= len(out) <= 3
    ws = out[0]
    assert ws.robots[0].id == 1 and ws.robots[0].x == pytest.approx(1.0, abs=0.02)
    assert len(ws.persons) == 1


def test_scenario_roundtrip(tmp_path):
    e = make_engine()
    load_scenario(e, "scenarios/demo_formation.yaml")
    assert len(e.robots) == 5
    save_scenario(e, tmp_path / "s.yaml")
    e2 = make_engine()
    load_scenario(e2, tmp_path / "s.yaml")
    assert set(e2.robots) == set(e.robots)
    assert len(e2.bodies) == len(e.bodies)


def test_virtual_robot_node_speaks_protocol():
    """仮想ロボットは実機と同じ手順で hello → 指令受信 → telemetry を返す."""
    received: list = []
    op = UdpEndpoint("op", "127.0.0.1", 0, on_message=lambda env, addr: received.append((env, addr))).start()
    net = NetworkConfig(operator_host="127.0.0.1", operator_robot_port=op.port, bind_host="127.0.0.1")
    e = make_engine()
    e.add_robot(7, Pose2D(1.0, 1.0, 0.0))
    node = VirtualRobotNode(e, 7, net, port=0).start()
    try:
        node.update(0.0)   # 最初の update で hello
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.01)
        env, addr = received[0]
        assert isinstance(env.msg, m.Hello) and env.msg.robot_id == 7 and env.src == "robot:7"
        op.send(m.Ping(nonce=5), addr)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not any(isinstance(x[0].msg, m.Pong) for x in received):
            time.sleep(0.01)
        for _ in range(30):
            e.step()
            node.update(0.005)
        time.sleep(0.1)
        types = {type(x[0].msg) for x in received}
        assert m.Pong in types and m.Telemetry in types
    finally:
        node.close()
        op.close()


def test_runner_realtime():
    from coco_link.sim.runner import SimRunner
    e = make_engine()
    e.add_robot(1, Pose2D(1.0, 1.0, 0.0))
    runner = SimRunner(e, network=False).start()
    time.sleep(0.5)
    runner.stop()
    assert e.t == pytest.approx(0.5, abs=0.15)
    assert math.isfinite(e.robots[1].pose.x)
