"""全モード共通の経路計画ナビゲーションのテスト."""

import math

from coco_link.common.geometry import Pose2D, Vec2
from coco_link.control.navigation import Navigator
from coco_link.control.planning import circle_path_clear
from coco_link.fleet.world_model import RobotState, World
from coco_link.modes import MODES, discover
from coco_link.modes.base import RobotCommand
from coco_link.modes.cooperative_transport import CooperativeTransportMode
from coco_link.modes.follow_person import FollowPersonMode
from coco_link.modes.formation import FormationMode
from coco_link.protocol import messages as m
from coco_link.robot.kinematics import integrate_unicycle
from coco_link.sim.engine import SimConfig, SimEngine

from .helpers import run_closed_loop, world_from_engine


def _world(pos: tuple[float, float], th: float = 0.0) -> World:
    w = World(t=0.0, field_w=3.0, field_h=2.0)
    w.robots[1] = RobotState(
        robot_id=1, connected=True, pose=Pose2D(*pos, th), pose_source="vision",
        telemetry=m.Telemetry(state=m.STATE_IDLE))
    return w


def test_path_planning_is_shared_by_modes_not_a_separate_mode():
    assert "経路計画走行" not in discover()
    assert {"待機", "フォーメーション", "人追従（隊列）", "包囲", "協調物資運搬"} <= set(MODES)


def test_navigator_ignores_robots_without_targets_and_near_goals():
    nav = Navigator()
    w = _world((1.0, 1.0))
    cmd = RobotCommand(vx=0.2, wz=0.1)
    assert nav.apply(w, {1: cmd}, {})[1] is cmd
    assert cmd.vx == 0.2 and cmd.wz == 0.1

    # 目標まで handover_distance 以内なら、モードの最終制御に任せる
    out = nav.apply(w, {1: cmd}, {1: Vec2(1.05, 1.0)})
    assert out[1] is cmd
    assert cmd.vx == 0.2 and cmd.wz == 0.1


def test_navigator_routes_around_an_obstacle():
    nav = Navigator()
    w = _world((0.4, 1.0))
    w.obstacles = [m.DetectedObject(1.5, 1.0, 0.25)]
    goal = Vec2(2.6, 1.0)
    pose = w.robots[1].pose
    min_dist = math.inf
    for k in range(600):
        w.t = k * 0.05
        cmd = RobotCommand(vx=0.25)
        nav.apply(w, {1: cmd}, {1: goal})
        pose = integrate_unicycle(pose, cmd.vx, cmd.wz, 0.05)
        w.robots[1].pose = pose
        min_dist = min(min_dist, (pose.pos - Vec2(1.5, 1.0)).norm())
        if (pose.pos - goal).norm() < 0.12:
            break
    assert (pose.pos - goal).norm() < 0.15
    assert min_dist > 0.25 + 0.06 - 0.02          # 機体が障害物に触れていない


def test_formation_routes_around_an_obstacle():
    e = SimEngine(SimConfig(seed=21, param_spread=0.05))
    for i, y in enumerate([0.75, 1.00, 1.25], start=1):
        e.add_robot(i, Pose2D(0.40, y, 0.0))
    obstacle = e.add_body("obstacle", 1.20, 1.00, 0.22)
    mode = FormationMode(FormationMode.Params(
        shape="circle", spacing=0.32, center_x=2.10, center_y=1.00))
    min_dist = math.inf

    def record(engine, _t):
        nonlocal min_dist
        for robot in engine.robots.values():
            p = robot.pose
            min_dist = min(min_dist, math.hypot(p.x - obstacle.x, p.y - obstacle.y))

    run_closed_loop(e, mode, 30.0, on_step=record)

    assert "完成" in mode.status
    assert min_dist > obstacle.r + e.robots[1].params.body_radius - 0.01


def test_follow_person_routes_around_an_obstacle():
    e = SimEngine(SimConfig(seed=22, param_spread=0.05))
    e.add_robot(1, Pose2D(0.40, 1.00, 0.0))
    e.add_robot(2, Pose2D(0.30, 0.75, 0.0))
    e.add_robot(3, Pose2D(0.30, 1.25, 0.0))
    person = e.add_body("person", 1.60, 1.00, 0.15)
    obstacle = e.add_body("obstacle", 0.90, 0.85, 0.18)
    mode = FollowPersonMode(FollowPersonMode.Params(
        follow_distance=0.40, spacing=0.25, max_speed=0.30))
    min_dist = math.inf

    def record(engine, _t):
        nonlocal min_dist
        for robot in engine.robots.values():
            p = robot.pose
            min_dist = min(min_dist, math.hypot(p.x - obstacle.x, p.y - obstacle.y))

    run_closed_loop(e, mode, 20.0, on_step=record)

    assert mode.order[0] == 1
    assert min_dist > obstacle.r + e.robots[1].params.body_radius - 0.01
    leader = e.robots[1].pose
    assert 0.20 < math.hypot(leader.x - person.x, leader.y - person.y) < 0.60


def test_transport_plans_a_cargo_route_around_an_obstacle():
    e = SimEngine(SimConfig(seed=23, param_spread=0.05))
    for i, y in enumerate([0.80, 1.00, 1.20], start=1):
        e.add_robot(i, Pose2D(0.45, y, 0.0))
    cargo = e.add_body("cargo", 0.90, 1.00, 0.05)
    obstacle = e.add_body("obstacle", 1.60, 1.00, 0.20)
    e.step(0.05)                     # ファームウェアが BOOT → IDLE になるまで待つ

    mode = CooperativeTransportMode(CooperativeTransportMode.Params(
        goal_x=2.40, goal_y=1.00))
    world = world_from_engine(e)
    mode.on_start(world)
    mode.step(world, 0.05)

    assert mode._cargo_path
    assert circle_path_clear(mode._cargo_path, Vec2(obstacle.x, obstacle.y),
                             obstacle.r + cargo.r)
    # 把持隊形へ向かう各ロボットも共通ナビゲーションの目標を持つ
    assert mode.navigation_targets(world)
