import math

import pytest

from coco_link.common.geometry import Pose2D, Vec2
from coco_link.control.formation import SHAPES, assign_slots, formation_offsets, slot_poses
from coco_link.control.path_follow import Trail, lookahead_point, pure_pursuit
from coco_link.control.pose_tracking import go_to_pose, kanayama_tracking, saturate
from coco_link.control.safety import avoid_robots, ultrasonic_speed_limit
from coco_link.robot.kinematics import integrate_unicycle


@pytest.mark.parametrize("shape", SHAPES)
@pytest.mark.parametrize("n", [1, 2, 5, 8])
def test_formation_offsets_count_and_separation(shape, n):
    pts = formation_offsets(shape, n, 0.25)
    assert len(pts) == n
    cx, cy = sum(p.x for p in pts) / n, sum(p.y for p in pts) / n
    if shape != "heart":
        assert math.hypot(cx, cy) < 0.2
    for i in range(n):
        for j in range(i + 1, n):
            assert (pts[i] - pts[j]).norm() > 0.12, (shape, n, i, j)


def test_assignment_is_optimal():
    slots = [Pose2D(0, 0), Pose2D(1, 0)]
    a = assign_slots({7: Vec2(1.1, 0), 3: Vec2(-0.1, 0)}, slots)
    assert a == {3: 0, 7: 1}


def test_slot_poses_rotate_with_center():
    s = slot_poses("line", 2, 0.5, Pose2D(1.0, 1.0, math.pi / 2))
    xs = sorted(round(p.x, 6) for p in s)
    assert xs == [0.75, 1.25]


@pytest.mark.parametrize("start", [Pose2D(0, 0, 0), Pose2D(0, 0, math.pi), Pose2D(1, 1, -2.0), Pose2D(0.5, -0.3, 1.0)])
def test_go_to_pose_converges(start):
    target = Pose2D(0.6, 0.4, math.pi / 2)
    p = start
    done = False
    for _ in range(2000):
        v, w, done = go_to_pose(p, target, 0.3, 3.0)
        if done:
            break
        p = integrate_unicycle(p, v, w, 0.01)
    assert done
    assert p.distance_to(target) < 0.04


def test_kanayama_zero_error_gives_feedforward():
    assert kanayama_tracking(Pose2D(1, 1, 0.3), Pose2D(1, 1, 0.3), 0.2, 0.5) == pytest.approx((0.2, 0.5))


def test_saturate_keeps_curvature():
    v, w = saturate(1.0, 4.0, 0.5, 3.0)
    assert w / v == pytest.approx(4.0) and abs(v) <= 0.5 and abs(w) <= 3.0


def test_trail_and_pure_pursuit():
    t = Trail(spacing=0.05)
    for i in range(21):
        t.add(Vec2(i * 0.05, 0.0))
    assert t.length_from(Vec2(0, 0)) == pytest.approx(1.0)
    t.prune_before(Vec2(0.5, 0.0), 0.03)
    assert t.points[0].x == pytest.approx(0.55)
    g = lookahead_point(list(t.points), Vec2(0.5, 0.0), 0.2)
    assert g.x == pytest.approx(0.7)
    v, w = pure_pursuit(Pose2D(0, 0, 0), Vec2(1.0, 1.0), 0.2)
    assert v == 0.2 and w > 0


def test_ultrasonic_limit():
    assert ultrasonic_speed_limit(0.3, 0.05, None) == 0.0
    assert ultrasonic_speed_limit(0.3, None, 0.05) == 0.3
    assert ultrasonic_speed_limit(-0.3, None, 0.05) == 0.0
    assert 0 < ultrasonic_speed_limit(0.3, 0.2, None) < 0.3


def test_avoid_robot_in_front():
    v, w = avoid_robots(Pose2D(0, 0, 0), 0.3, 0.0, [Vec2(0.2, 0.02)], 0.06)
    assert v < 0.3 and w < 0      # 左前の相手 → 右へ避ける
