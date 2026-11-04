import math

from coco_link.common.geometry import Pose2D, Vec2
from coco_link.control.transport import (
    CargoObject,
    grasp_offsets,
    grasp_radius,
    heading_to,
    push_pose,
)
from coco_link.modes import MODES, discover
from coco_link.modes.cooperative_transport import CooperativeTransportMode
from coco_link.sim.engine import SimConfig, SimEngine

from .helpers import run_closed_loop, world_from_engine


def _engine(seed: int, starts: list[tuple[float, float]], cargo=(0.90, 1.00, 0.05)):
    e = SimEngine(SimConfig(seed=seed, param_spread=0.05))
    for i, (x, y) in enumerate(starts, start=1):
        e.add_robot(i, Pose2D(x, y, 0.0))
    body = e.add_body("cargo", *cargo)
    return e, body


def test_cooperative_transport_is_registered():
    assert "協調物資運搬" in discover()
    assert MODES["協調物資運搬"] is CooperativeTransportMode


def test_click_selects_cargo_then_sets_goal():
    e, cargo = _engine(10, [(0.5, 0.5)])
    mode = CooperativeTransportMode(CooperativeTransportMode.Params(auto_select=False))
    world = world_from_engine(e)
    mode.on_start(world)
    assert mode.cargo is None
    assert mode.step(world, 0.05) == {}

    mode.on_field_click(cargo.x + 0.02, cargo.y - 0.02)
    assert mode.cargo is not None
    assert (mode.cargo.x, mode.cargo.y) == (cargo.x, cargo.y)

    mode.on_field_click(2.0, 0.6)
    assert (mode.params.goal_x, mode.params.goal_y) == (2.0, 0.6)


def test_click_far_from_cargo_selects_nothing():
    e, _ = _engine(11, [(0.5, 0.5)])
    mode = CooperativeTransportMode(CooperativeTransportMode.Params(auto_select=False))
    world = world_from_engine(e)
    mode.on_start(world)
    mode.step(world, 0.05)
    mode.on_field_click(2.5, 1.5)
    assert mode.cargo is None
    assert "物資がありません" in mode.status


def test_no_cargo_stops_with_message():
    e = SimEngine(SimConfig(seed=12))
    e.add_robot(1, Pose2D(0.5, 0.5, 0.0))
    e.step(0.05)                     # ファームウェアが BOOT → IDLE になるまで待つ
    mode = CooperativeTransportMode()
    world = world_from_engine(e)
    mode.on_start(world)
    assert mode.step(world, 0.05) == {}
    assert "物資が見つかりません" in mode.status


def test_transport_converges_with_three_robots():
    e, cargo = _engine(1, [(0.45, 0.70), (0.45, 1.00), (0.45, 1.30)])
    mode = CooperativeTransportMode(CooperativeTransportMode.Params(goal_x=2.30, goal_y=1.00))
    run_closed_loop(e, mode, 45.0)

    assert mode._team == (1, 2, 3)
    assert "完了" in mode.status
    assert math.hypot(cargo.x - 2.30, cargo.y - 1.00) < 0.12
    # 押している間は超音波減速を切る（物資を障害物と誤検知しないため）
    assert mode._phase == "done"
    for rid in mode._team:
        assert mode.viz.targets[rid] is not None


def test_transport_converges_with_two_robots():
    """2 台でも把持円弧の左右のロボットが物資に届くこと（余白の回帰テスト）."""
    e, cargo = _engine(2, [(0.45, 0.85), (0.45, 1.15)], cargo=(0.90, 1.00, 0.05))
    mode = CooperativeTransportMode(CooperativeTransportMode.Params(goal_x=2.30, goal_y=1.00))
    run_closed_loop(e, mode, 45.0)

    assert mode._team == (1, 2)
    assert "完了" in mode.status
    assert math.hypot(cargo.x - 2.30, cargo.y - 1.00) < 0.12


def test_at_most_max_robots_join_the_team():
    starts = [(0.30, 0.50), (0.35, 0.85), (0.35, 1.15), (0.30, 1.50)]
    e, _ = _engine(3, starts)
    e.step(0.05)                     # ファームウェアが BOOT → IDLE になるまで待つ
    mode = CooperativeTransportMode(CooperativeTransportMode.Params(max_robots=3))
    world = world_from_engine(e)
    mode.on_start(world)
    cmds = mode.step(world, 0.05)

    assert len(mode._team) == 3
    idle = set(cmds) - set(mode._team)
    assert len(idle) == 1
    for rid in idle:
        assert cmds[rid].vx == 0.0 and cmds[rid].wz == 0.0


# ---------------------------------------------------------------- control/transport.py
def test_grasp_offsets_are_behind_the_cargo():
    offsets = grasp_offsets(3, radius=0.20, spread=math.radians(50.0))
    assert len(offsets) == 3
    assert all(o.x < 0.0 for o in offsets)                  # 目標方向 (+x) の後方
    assert abs(offsets[1].y) < 1e-9                          # 中央は真後ろ
    assert offsets[0].y * offsets[2].y < 0.0                # 左右に分かれる


def test_grasp_radius_grows_when_robots_would_overlap():
    one = grasp_radius(0.05, 0.06, 0.01, n=1, spread=math.radians(55.0))
    four = grasp_radius(0.05, 0.06, 0.01, n=4, spread=math.radians(55.0))
    assert abs(one - 0.12) < 1e-9
    assert four > one


def test_push_pose_leads_toward_goal_and_stops_at_goal():
    cargo = CargoObject(1.0, 0.5, 0.05)
    pose, dist = push_pose(cargo, Vec2(2.0, 0.5), lead=0.05, last_heading=0.0, goal_tol=0.08)
    assert dist == 1.0
    assert pose.x == 1.05 and pose.y == 0.5 and pose.th == 0.0

    pose, dist = push_pose(cargo, Vec2(1.03, 0.5), lead=0.05, last_heading=0.3, goal_tol=0.08)
    assert dist < 0.08
    assert pose.x == cargo.x and pose.y == cargo.y and pose.th == 0.3


def test_heading_holds_last_value_near_goal():
    cargo = CargoObject(1.0, 0.5, 0.05)
    assert heading_to(cargo, Vec2(2.0, 0.5), 0.0, goal_tol=0.08) == 0.0
    assert heading_to(cargo, Vec2(1.01, 0.5), 1.1, goal_tol=0.08) == 1.1
