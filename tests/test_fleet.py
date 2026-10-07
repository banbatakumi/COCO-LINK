"""operator 側（FleetManager / WorldModel / ControlCore）とシミュレータを UDP で繋いだ結合テスト."""

import random
import time

import pytest

from coco_link.common.config import NetworkConfig
from coco_link.common.geometry import Pose2D
from coco_link.fleet.control_core import ControlCore
from coco_link.fleet.fleet_manager import FleetManager
from coco_link.fleet.world_model import WorldModel
from coco_link.protocol import messages as m
from coco_link.sim.engine import SimConfig, SimEngine
from coco_link.sim.runner import SimRunner


def wait_until(cond, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def system():
    """operator(ランダムポート) + シミュレータ(実時間) を起動."""
    op_net = NetworkConfig(bind_host="127.0.0.1", operator_robot_port=0, operator_vision_port=0, control_hz=20)
    core = ControlCore(op_net)
    core.start()
    sim_net = NetworkConfig(operator_host="127.0.0.1", bind_host="127.0.0.1",
                            operator_robot_port=core.fleet.endpoint.port,
                            operator_vision_port=core.world_model.endpoint.port,
                            robot_port=random.randint(40000, 45000))
    engine = SimEngine(SimConfig(seed=5, param_spread=0.05))
    engine.add_robot(1, Pose2D(1.0, 1.0, 0.0))
    engine.add_robot(2, Pose2D(2.0, 1.0, 3.14))
    engine.add_body("person", 1.5, 1.6)
    runner = SimRunner(engine, sim_net).start()
    yield core, engine
    runner.stop()
    core.close()


def test_discovery_telemetry_and_vision(system):
    core, engine = system
    assert wait_until(lambda: core.fleet.connected_ids() == [1, 2])
    assert wait_until(lambda: all(p.telemetry is not None for p in core.fleet.robots.values()))
    assert wait_until(lambda: all(p.sim_truth is not None for p in core.fleet.robots.values()), 3.0)
    world = core.world_model.snapshot()
    assert world.robots[1].pose is not None
    assert world.robots[1].pose.distance_to(engine.robots[1].pose) < 0.05
    assert len(world.persons) == 1
    assert wait_until(lambda: core.fleet.get(1).rtt_ms is not None)


def test_manual_drive_and_estop(system):
    core, engine = system
    assert wait_until(lambda: core.fleet.connected_ids() == [1, 2])
    x0 = engine.robots[1].pose.x
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        core.set_manual(1, 0.2, 0.0)
        time.sleep(0.05)
    assert engine.robots[1].pose.x > x0 + 0.1
    core.estop_all()
    assert wait_until(lambda: engine.robots[1].firmware.state == m.STATE_ESTOP)
    core.clear_estop_all()
    assert wait_until(lambda: engine.robots[1].firmware.state == m.STATE_IDLE)


def test_formation_over_udp(system):
    core, engine = system
    assert wait_until(lambda: len(core.world_model.snapshot().usable_robots()) == 2)
    mode_cls = core.modes["フォーメーション"]
    core.start_mode("フォーメーション", mode_cls.Params(shape="line", spacing=0.4, center_x=1.5, center_y=0.8))
    assert wait_until(lambda: "完成" in core.snapshot.mode_status, timeout=20.0), core.snapshot.mode_status
    core.stop_mode()


def test_world_model_extrapolates_latency():
    """ビジョン遅延をエンコーダ速度で補償する."""
    t = [100.0]
    net = NetworkConfig(bind_host="127.0.0.1", operator_robot_port=0, operator_vision_port=0)
    fleet = FleetManager(net, clock=lambda: t[0])
    wm = WorldModel(net, fleet, clock=lambda: t[0])
    try:
        ws = m.WorldState(latency_ms=100.0, field=m.FieldSize(3, 2), robots=[m.DetectedRobot(1, 1.0, 1.0, 0.0)])
        wm.inject(ws, now=100.0)
        from coco_link.fleet.robot_proxy import RobotProxy
        proxy = RobotProxy(1, ("127.0.0.1", 1), last_rx=100.0)
        r = wm.robot_params(1).wheel_radius
        proxy.telemetry = m.Telemetry(robot_id=1, state="RUNNING", enc=m.Encoders(wl=0.2 / r, wr=0.2 / r))
        fleet.robots[1] = proxy
        st = wm.snapshot().robots[1]
        assert st.pose.x == pytest.approx(1.0 + 0.2 * 0.1, abs=1e-6)
        assert st.pose_source == "vision"
        t[0] = 100.3      # ビジョンが途絶 → 推測航法
        st = wm.snapshot().robots[1]
        assert st.pose_source == "odom" and st.pose.x == pytest.approx(1.0 + 0.2 * 0.4, abs=1e-6)
        t[0] = 101.0      # 長く途絶 → 不明
        assert wm.snapshot().robots[1].pose is None
    finally:
        fleet.close()
        wm.close()
