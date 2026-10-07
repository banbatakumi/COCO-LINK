"""ビジョン: 合成画像で ArUco 姿勢・色検出・キャリブレーションの精度を確認."""

import math

import numpy as np
import pytest

from coco_link.common.config import FieldConfig
from coco_link.common.geometry import Pose2D, wrap_angle
from coco_link.vision.calibration import FieldCalibration
from coco_link.vision.pipeline import VisionPipeline, draw_overlay
from coco_link.vision.synthetic import SyntheticScene, render

ROBOTS = {1: Pose2D(0.8, 0.6, 0.0), 2: Pose2D(1.6, 1.2, 2.0), 7: Pose2D(2.4, 0.5, -1.2)}
OBJECTS = [("person", 2.3, 1.5, 0.15), ("obstacle", 1.0, 1.4, 0.10), ("cargo", 1.8, 0.4, 0.06)]


@pytest.fixture(scope="module")
def field_cfg():
    return FieldConfig.load()


@pytest.mark.parametrize("tilt", [0.0, 0.05])
def test_robot_poses(field_cfg, tilt):
    img, _ = render(SyntheticScene(field_cfg=field_cfg, robots=ROBOTS, objects=OBJECTS, tilt=tilt))
    res = VisionPipeline(field_cfg, marker_height=0.0).process(img)
    assert res.calibrated
    found = {r.id: r for r in res.world.robots}
    assert set(found) == set(ROBOTS)
    for rid, truth in ROBOTS.items():
        r = found[rid]
        assert math.hypot(r.x - truth.x, r.y - truth.y) < 0.01, (rid, r, truth)
        assert abs(wrap_angle(r.th - truth.th)) < math.radians(3)


def test_color_objects(field_cfg):
    img, _ = render(SyntheticScene(field_cfg=field_cfg, robots=ROBOTS, objects=OBJECTS, tilt=0.03))
    ws = VisionPipeline(field_cfg, marker_height=0.0).process(img).world
    for kind, attr in (("person", "persons"), ("obstacle", "obstacles"), ("cargo", "cargo")):
        objs = getattr(ws, attr)
        assert len(objs) == 1, (kind, objs)
        _, x, y, r = next(o for o in OBJECTS if o[0] == kind)
        assert math.hypot(objs[0].x - x, objs[0].y - y) < 0.02
        assert objs[0].r == pytest.approx(r, rel=0.15)


def test_no_calibration_without_corners(field_cfg):
    img = np.full((480, 640, 3), 200, np.uint8)
    res = VisionPipeline(field_cfg).process(img)
    assert not res.calibrated and res.world.robots == []
    assert draw_overlay(img, res, None).shape == img.shape


def test_manual_calibration_and_parallax(tmp_path):
    c = FieldCalibration.from_points([[0, 0], [300, 0], [300, 200], [0, 200]], [[0, 0], [3, 0], [3, 2], [0, 2]],
                                     image_size=(300, 200), camera_height=2.0)
    assert c.to_field([[150, 100]])[0] == pytest.approx([1.5, 1.0])
    # 高さ 0.2 m の点は天底 (1.5, 1.0) に向かって 10 % 縮む
    assert c.to_field([[300, 200]], height=0.2)[0] == pytest.approx([1.5 + 1.5 * 0.9, 1.0 + 1.0 * 0.9])
    path = c.save(tmp_path / "calib.yaml")
    c2 = FieldCalibration.load(path)
    assert np.allclose(c2.H, c.H)


def test_publisher_reaches_operator(field_cfg):
    import time

    from coco_link.common.config import NetworkConfig
    from coco_link.fleet.fleet_manager import FleetManager
    from coco_link.fleet.world_model import WorldModel
    from coco_link.vision.publisher import VisionPublisher

    net = NetworkConfig(bind_host="127.0.0.1", operator_robot_port=0, operator_vision_port=0)
    fleet = FleetManager(net)
    wm = WorldModel(net, fleet).start()
    pub = VisionPublisher(NetworkConfig(operator_host="127.0.0.1", operator_vision_port=wm.endpoint.port,
                                        bind_host="127.0.0.1"))
    try:
        img, _ = render(SyntheticScene(field_cfg=field_cfg, robots=ROBOTS, objects=OBJECTS))
        pub.send(VisionPipeline(field_cfg).process(img).world)
        deadline = time.monotonic() + 2
        while wm.latest is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert wm.latest is not None and len(wm.latest.robots) == 3
    finally:
        pub.close()
        wm.close()
        fleet.close()
