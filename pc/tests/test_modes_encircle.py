import math

from coco_link.common.geometry import Pose2D
from coco_link.modes import MODES, discover
from coco_link.modes.encircle import EncircleMode
from coco_link.sim.engine import SimConfig, SimEngine

from .helpers import run_closed_loop, world_from_engine


def _click_once(mode, x, y):
    """run_closed_loop の on_step 用: 候補が揃った最初のタイミングで 1 回だけクリックする."""
    clicked = False

    def click(_engine, _t):
        nonlocal clicked
        if not clicked:
            mode.on_field_click(x, y)
            clicked = True

    return click


def test_encircle_mode_is_registered():
    assert "包囲" in discover()
    assert MODES["包囲"] is EncircleMode


def test_click_selects_nearest_object():
    e = SimEngine(SimConfig(seed=1))
    e.add_robot(1, Pose2D(0.5, 0.5, 0.0))
    cargo = e.add_body("cargo", 1.5, 1.0, 0.05)
    e.add_body("obstacle", 2.5, 1.0, 0.10)
    mode = EncircleMode()
    world = world_from_engine(e)
    mode.on_start(world)
    mode.step(world, 0.05)

    mode.on_field_click(1.52, 1.02)
    assert mode.target_kind == "cargo"
    assert mode.target_pos.x == cargo.x
    assert mode.target_pos.y == cargo.y


def test_click_far_from_objects_selects_nothing():
    e = SimEngine(SimConfig(seed=2))
    e.add_robot(1, Pose2D(0.5, 0.5, 0.0))
    e.add_body("obstacle", 2.5, 1.5, 0.10)
    mode = EncircleMode()
    world = world_from_engine(e)
    mode.on_start(world)
    mode.step(world, 0.05)

    mode.on_field_click(0.5, 1.0)
    assert mode.target_kind is None
    assert "対象がありません" in mode.status
    assert mode.step(world, 0.05) == {}


def test_encircle_converges_around_obstacle():
    e = SimEngine(SimConfig(seed=3, param_spread=0.05))
    for i, (x, y, th) in enumerate(
        [(0.4, 0.4, 0.0), (0.8, 1.6, 1.57), (2.4, 1.5, 3.0), (2.6, 0.5, 2.0)], start=1
    ):
        e.add_robot(i, Pose2D(x, y, th))
    obstacle = e.add_body("obstacle", 1.5, 1.0, 0.10)

    mode = EncircleMode(EncircleMode.Params(margin=0.15, max_speed=0.25))
    run_closed_loop(e, mode, 25.0, on_step=_click_once(mode, obstacle.x, obstacle.y))

    assert mode.target_kind == "obstacle"
    assert "完成" in mode.status
    radius = mode.target_pos and math.hypot(
        mode.viz.targets[1].x - mode.target_pos.x, mode.viz.targets[1].y - mode.target_pos.y
    )
    assert radius is not None and radius > 0.2
    angles = []
    for rid, target in mode.viz.targets.items():
        pose = e.robots[rid].pose
        assert pose.distance_to(target) < 0.06, (rid, pose, target)
        # 向きも目標（中心向き）に合っている
        assert abs(math.atan2(math.sin(pose.th - target.th), math.cos(pose.th - target.th))) < 0.2
        angles.append(math.atan2(pose.y - mode.target_pos.y, pose.x - mode.target_pos.x))
    # 等間隔に並んでいる（隣り合う角度差が 2π/n）
    angles.sort()
    gaps = [angles[(i + 1) % len(angles)] - angles[i] for i in range(len(angles))]
    gaps = [g + 2 * math.pi if g <= 0 else g for g in gaps]
    for gap in gaps:
        assert abs(gap - 2 * math.pi / len(angles)) < 0.35, gaps


def test_encircle_tracks_a_moving_person():
    e = SimEngine(SimConfig(seed=4, param_spread=0.05))
    for i, (x, y) in enumerate([(0.4, 0.4), (0.4, 1.6), (2.6, 0.4), (2.6, 1.6)], start=1):
        e.add_robot(i, Pose2D(x, y, 0.0))
    person = e.add_body("person", 1.5, 1.0, 0.15)

    def walk(engine, t):
        person.vx, person.vy = (0.10, 0.0) if t < 4.0 else (0.0, 0.0)

    mode = EncircleMode(EncircleMode.Params(margin=0.15, max_speed=0.30))
    run_closed_loop(e, mode, 22.0, on_step=_click_once(mode, person.x, person.y))

    assert mode.target_kind == "person"
    assert abs(mode.target_pos.x - person.x) < 0.02
    assert abs(mode.target_pos.y - person.y) < 0.02
    assert "完成" in mode.status
    for rid, target in mode.viz.targets.items():
        assert e.robots[rid].pose.distance_to(target) < 0.06, (rid, e.robots[rid].pose, target)
