import math

from coco_link.common.geometry import Vec2
from coco_link.control.planning import (
    OccupancyGrid,
    PathTracker,
    circle_path_clear,
    densify,
    path_length,
    plan_path,
    smooth_path,
)


def _free_grid() -> OccupancyGrid:
    return OccupancyGrid(3.0, 2.0, 0.05)


def test_astar_goes_around_an_obstacle():
    start, goal, obstacle = Vec2(0.3, 1.0), Vec2(2.7, 1.0), (Vec2(1.5, 1.0), 0.25)
    path = plan_path(start, goal, [obstacle], robot_radius=0.06, clearance=0.08,
                     resolution=0.05, field_wh=(3.0, 2.0))
    assert path is not None
    assert path[0] == start and path[-1] == goal
    assert path_length(path) > (goal - start).norm()          # 直線より遠回り
    assert circle_path_clear(path, obstacle[0], obstacle[1] + 0.06)   # 機体が障害物に触れない


def test_astar_returns_none_when_wall_blocks_the_goal():
    wall = [(Vec2(1.5, 0.05 + 0.10 * i), 0.12) for i in range(20)]
    path = plan_path(Vec2(0.3, 1.0), Vec2(2.7, 1.0), wall, robot_radius=0.06, clearance=0.05,
                     resolution=0.05, field_wh=(3.0, 2.0))
    assert path is None


def test_goal_inside_an_obstacle_moves_to_the_nearest_free_point():
    grid = _free_grid()
    grid.block_circle(Vec2(1.5, 1.0), 0.20)
    ix, iy = grid.nearest_free(Vec2(1.5, 1.0))
    p = grid.cell_center(ix, iy)
    assert not grid.is_free(Vec2(1.5, 1.0))
    assert grid.is_free(p)
    assert (p - Vec2(1.5, 1.0)).norm() >= 0.20 - 0.05


def test_smooth_path_replaces_detour_with_a_straight_line_in_free_space():
    grid = _free_grid()
    path = [Vec2(0.2, 0.2), Vec2(0.5, 0.2), Vec2(0.5, 0.5), Vec2(1.0, 0.5)]
    smoothed = smooth_path(path, grid)
    assert smoothed[0] == path[0] and smoothed[-1] == path[-1]
    assert len(smoothed) < len(path)
    assert all(grid.line_clear(a, b) for a, b in zip(smoothed, smoothed[1:], strict=False))


def test_densify_keeps_the_path_and_spacing():
    path = [Vec2(0.0, 0.0), Vec2(1.0, 0.0)]
    dense = densify(path, 0.02)
    assert dense[0] == path[0] and dense[-1] == path[-1]
    assert len(dense) == 51
    assert max((b - a).norm() for a, b in zip(dense, dense[1:], strict=False)) <= 0.02 + 1e-9


def test_path_tracker_advances_and_reports_the_remaining_distance():
    path = [Vec2(0.0, 0.0), Vec2(0.5, 0.0), Vec2(1.0, 0.0)]
    tracker = PathTracker(lookahead=0.2)
    tracker.set_path(path, Vec2(0.0, 0.0))
    assert not tracker.finished(Vec2(0.0, 0.0), tol=0.05)
    assert tracker.remaining(Vec2(0.0, 0.0)) == 1.0
    target = tracker.target(Vec2(0.0, 0.0))
    assert target is not None and 0.0 < target.x <= 0.25

    tracker.update(Vec2(0.6, 0.0))
    assert tracker.index == 1
    assert math.isclose(tracker.remaining(Vec2(0.6, 0.0)), 0.4, abs_tol=1e-9)
    tracker.update(Vec2(1.0, 0.0))
    assert tracker.finished(Vec2(1.0, 0.0), tol=0.05)
