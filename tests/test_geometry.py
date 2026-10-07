import math

import pytest

from coco_link.common.geometry import Pose2D, Vec2, wrap_angle


@pytest.mark.parametrize("a,expected", [(0, 0), (math.pi, math.pi), (-math.pi, math.pi),
                                        (3 * math.pi, math.pi), (2 * math.pi + 0.1, 0.1), (-0.1, -0.1)])
def test_wrap_angle(a, expected):
    assert wrap_angle(a) == pytest.approx(expected)


def test_pose_compose_inverse_roundtrip():
    a = Pose2D(1.0, 2.0, 0.7)
    b = Pose2D(-0.3, 0.5, -2.0)
    c = a.compose(b)
    back = c.relative_to(a)
    assert back.x == pytest.approx(b.x)
    assert back.y == pytest.approx(b.y)
    assert back.th == pytest.approx(b.th)


def test_transform_point():
    p = Pose2D(1.0, 0.0, math.pi / 2).transform_point(Vec2(1.0, 0.0))
    assert (p.x, p.y) == pytest.approx((1.0, 1.0))
