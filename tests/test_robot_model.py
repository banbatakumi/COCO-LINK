import math

import pytest

from coco_link.common.geometry import Pose2D
from coco_link.protocol import messages as m
from coco_link.robot.firmware_model import ENC_COUNTS, EncoderUnwrapper, FirmwareModel, SensorInputs
from coco_link.robot.kinematics import body_to_wheel, integrate_unicycle, limit_wheel_speeds, wheel_to_body
from coco_link.robot.params import RobotParams
from coco_link.robot.plant import DiffDrivePlant

DT = 0.01


def test_kinematics_roundtrip():
    wl, wr = body_to_wheel(0.2, 1.5, 0.021, 0.09)
    assert wheel_to_body(wl, wr, 0.021, 0.09) == pytest.approx((0.2, 1.5))


def test_limit_preserves_ratio():
    wl, wr = limit_wheel_speeds(10.0, 30.0, 15.0)
    assert (wl, wr) == pytest.approx((5.0, 15.0))


def test_integrate_unicycle_full_circle():
    p = Pose2D()
    for _ in range(1000):
        p = integrate_unicycle(p, 0.2, 2 * math.pi / 10.0, 0.01)
    assert (p.x, p.y) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_encoder_unwrap_across_zero():
    u = EncoderUnwrapper()
    seq = [4090, 4095, 3, 10, 4000, 10]
    counts = [u.update(r) for r in seq]
    assert counts == [0, 5, 9, 16, -90, 16]


def run(fw: FirmwareModel, plant: DiffDrivePlant, seconds: float, batt_v: float = 4.8):
    for _ in range(round(seconds / DT)):
        el, er = plant.encoder_raw()
        duty = fw.tick(DT, SensorInputs(enc_raw_l=el, enc_raw_r=er, batt_adc_v=batt_v / 2.0))
        plant.step(*duty, batt_v=batt_v, dt=DT)


def make(params: RobotParams | None = None):
    params = params or RobotParams()
    fw = FirmwareModel(1, {"wheel_radius": params.wheel_radius, "tread": params.tread})
    plant = DiffDrivePlant(params)
    run(fw, plant, 0.05)  # BOOT -> IDLE
    return fw, plant


def test_boot_to_idle():
    fw, _ = make()
    assert fw.state == m.STATE_IDLE


def test_cmd_vel_tracks_speed():
    fw, plant = make()
    for _ in range(100):  # 1 s 間 20Hz で指令し続ける
        fw.handle_message(m.CmdVel(vx=0.2, wz=0.0))
        run(fw, plant, 0.05)
    assert fw.state == m.STATE_RUNNING
    assert plant.vx == pytest.approx(0.2, abs=0.02)
    assert abs(plant.wz) < 0.1


def test_cmd_vel_tracks_rotation():
    fw, plant = make()
    for _ in range(40):
        fw.handle_message(m.CmdVel(vx=0.0, wz=2.0))
        run(fw, plant, 0.05)
    assert plant.wz == pytest.approx(2.0, abs=0.2)


def test_watchdog_stops_robot():
    fw, plant = make()
    for _ in range(20):
        fw.handle_message(m.CmdVel(vx=0.2, wz=0.0))
        run(fw, plant, 0.05)
    run(fw, plant, 0.35)   # 指令途絶 > 300 ms
    assert fw.state == m.STATE_IDLE
    run(fw, plant, 0.5)
    assert abs(plant.vx) < 0.01


def test_estop_latches():
    fw, plant = make()
    fw.handle_message(m.CmdVel(vx=0.2))
    run(fw, plant, 0.1)
    fw.handle_message(m.EStop())
    assert fw.tick(DT, SensorInputs(batt_adc_v=2.4)) == (0.0, 0.0)
    fw.handle_message(m.CmdVel(vx=0.2))
    assert fw.state == m.STATE_ESTOP
    fw.handle_message(m.ClearEStop())
    assert fw.state == m.STATE_IDLE


def test_cmd_wheel_is_open_loop():
    fw, plant = make()
    fw.handle_message(m.CmdWheel(left=0.5, right=-0.3))
    assert fw.tick(DT, SensorInputs(batt_adc_v=2.4)) == (0.5, -0.3)


def test_low_battery():
    fw, plant = make()
    run(fw, plant, 3.0, batt_v=4.0)
    assert fw.state == m.STATE_LOW_BATT
    fw.handle_message(m.CmdVel(vx=0.2))
    assert fw.state == m.STATE_LOW_BATT
    run(fw, plant, 3.0, batt_v=4.8)
    assert fw.state == m.STATE_IDLE


def test_set_param_clamps_and_replies():
    fw, _ = make()
    replies = fw.handle_message(m.SetParam(key="kp", value=5.0))
    assert isinstance(replies[0], m.Params)
    assert fw.params["kp"] == 1.0
    assert fw.handle_message(m.Ping(nonce=42)) == [m.Pong(nonce=42)]


def test_telemetry_encoder_matches_plant():
    fw, plant = make()
    for _ in range(20):
        fw.handle_message(m.CmdVel(vx=0.2))
        run(fw, plant, 0.05)
    t = fw.telemetry()
    # telemetry は直前の tick 時点の値なので、最後の 1 ステップ分だけ遅れる
    expected = plant.theta_l - plant.omega_l * DT
    assert t.enc.left_rad == pytest.approx(expected, abs=2 * 2 * math.pi / ENC_COUNTS)


def test_randomized_params_reproducible():
    import random
    a = RobotParams().randomized(random.Random(1))
    b = RobotParams().randomized(random.Random(1))
    assert a == b and a != RobotParams()
