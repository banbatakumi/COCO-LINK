"""システム同定: シミュレータの隠れ真値を推定できるか."""

import random
import time

import numpy as np
import pytest

from coco_link.common.config import NetworkConfig
from coco_link.common.geometry import Pose2D
from coco_link.protocol import messages as m
from coco_link.robot.params import RobotParams
from coco_link.sim.engine import PHYSICS_HZ, SimConfig, SimEngine
from coco_link.sysid import estimators as est
from coco_link.sysid.experiments import EXPERIMENTS, SysIdContext, SysIdOptions
from coco_link.sysid.recording import Recording, TelemetrySample, VisionSample


def run_offline(engine: SimEngine, rid: int, names: list[str], seed: int = 0) -> tuple[SysIdContext, dict]:
    """通信を介さず、試験スケジュールどおりに仮想ロボットを動かしてデータを取り推定する."""
    rng = random.Random(seed)
    ctx = SysIdContext(params=RobotParams(), options=SysIdOptions(), fw_params=dict(engine.robots[rid].firmware.params))
    results = {}
    robot = engine.robots[rid]
    for name in names:
        exp = EXPERIMENTS[name]
        rec = Recording(rid)
        for seg in exp.segments(ctx):
            n_ctrl = round(seg.duration * 20)
            for k in range(n_ctrl):
                cmd = seg.command(k / 20)
                engine.deliver(rid, m.CmdWheel(*cmd.wheel) if cmd.wheel else m.CmdVel(cmd.vx, cmd.wz))
                for i in range(int(PHYSICS_HZ / 20)):
                    engine.step()
                    if i % 2 == 0:      # 同定中はテレメトリ 100 Hz（SysIdRunner が telemetry_hz=100 に設定する）
                        t = robot.firmware.telemetry()
                        rec.telemetry.append(TelemetrySample(robot.firmware.t, engine.t, seg.phase, t.enc.left_rad,
                                                             t.enc.right_rad, t.enc.wl, t.enc.wr, t.applied.left,
                                                             t.applied.right, t.imu.gz, t.batt_v))
                    if i % 7 == 0:      # ビジョン 約 30 Hz（ノイズ付き）
                        p = robot.pose
                        rec.vision.append(VisionSample(engine.t, seg.phase, p.x + rng.gauss(0, 0.003),
                                                       p.y + rng.gauss(0, 0.003), p.th + rng.gauss(0, 0.017)))
        res = exp.estimate(rec, ctx)
        assert res.ok, res.message
        results[name] = res
        from dataclasses import replace
        ctx.params = replace(ctx.params, **res.params)
    return ctx, results


@pytest.fixture(scope="module")
def identified():
    e = SimEngine(SimConfig(seed=11, param_spread=0.15))
    e.add_robot(1, Pose2D(1.5, 1.0, 0.0))
    ctx, results = run_offline(e, 1, ["gyro_bias", "deadzone", "step", "straight", "spin", "verify"])
    return e.robots[1].params, ctx.params, results


def rel(a, b):
    return abs(a - b) / abs(b)


def test_gyro_bias(identified):
    truth, est_p, _ = identified
    assert est_p.gyro_bias_z == pytest.approx(truth.gyro_bias_z, abs=0.002)


def test_deadzone(identified):
    truth, est_p, _ = identified
    assert est_p.deadzone_l == pytest.approx(truth.deadzone_l, abs=0.015)
    assert est_p.deadzone_r == pytest.approx(truth.deadzone_r, abs=0.015)


def test_motor_gain_and_tau(identified):
    truth, est_p, results = identified
    assert rel(est_p.motor_gain_l, truth.motor_gain_l) < 0.05
    assert rel(est_p.motor_gain_r, truth.motor_gain_r) < 0.05
    assert rel(est_p.motor_tau_l, truth.motor_tau_l) < 0.15
    assert rel(est_p.motor_tau_r, truth.motor_tau_r) < 0.15
    assert results["step"].metrics["r2_l"] > 0.95


def test_wheel_radius_and_tread(identified):
    truth, est_p, _ = identified
    assert rel(est_p.wheel_radius, truth.wheel_radius) < 0.02
    assert rel(est_p.tread, truth.tread) < 0.03


def test_verify_metrics_reported(identified):
    _, _, results = identified
    assert np.isfinite(results["verify"].metrics["rise_time_s"])


def test_first_order_on_synthetic_data():
    """既知の一次遅れ系の合成データから K, τ を回復する."""
    K, tau, u0, dt = 30.0, 0.08, 0.1, 0.02
    t = np.arange(0, 6, dt)
    u = np.where((t % 2) < 1, 0.5, 0.0)
    ueff = est.effective_input(u, u0, np.full_like(t, 4.8), 4.8)
    w, theta = 0.0, [0.0]
    for k in range(len(t) - 1):
        a = np.exp(-dt / tau)
        # 角度は区間で厳密積分
        theta.append(theta[-1] + K * ueff[k] * dt + (w - K * ueff[k]) * tau * (1 - a))
        w = K * ueff[k] + (w - K * ueff[k]) * a
    fit = est.first_order(t, np.array(theta), u, np.full_like(t, 4.8), 4.8, u0)
    assert pytest.approx(K, rel=1e-3) == fit.K
    assert fit.tau == pytest.approx(tau, rel=1e-3)


def test_imc_pi():
    kp, ki, kff = est.imc_pi(25.0, 0.06, 0.04)
    assert kp == pytest.approx(0.06) and ki == pytest.approx(1.0) and kff == pytest.approx(0.04)


def test_runner_over_udp():
    """ControlCore + SysIdRunner + シミュレータを UDP で繋いでジャイロバイアス試験を実行."""
    from coco_link.fleet.control_core import ControlCore
    from coco_link.sim.runner import SimRunner
    from coco_link.sysid.runner import SysIdRunner

    core = ControlCore(NetworkConfig(bind_host="127.0.0.1", operator_robot_port=0, operator_vision_port=0)).start()
    sim_net = NetworkConfig(operator_host="127.0.0.1", bind_host="127.0.0.1",
                            operator_robot_port=core.fleet.endpoint.port,
                            operator_vision_port=core.world_model.endpoint.port, robot_port=random.randint(45001, 49000))
    engine = SimEngine(SimConfig(seed=4))
    engine.add_robot(3, Pose2D(1, 1, 0))
    runner = SimRunner(engine, sim_net).start()
    try:
        sysid = SysIdRunner(core)
        deadline = time.monotonic() + 5
        while 3 not in core.fleet.connected_ids() and time.monotonic() < deadline:
            time.sleep(0.05)
        sysid.start(3, ["gyro_bias"])
        deadline = time.monotonic() + 8
        while sysid.active and time.monotonic() < deadline:
            time.sleep(0.1)
        assert not sysid.active
        assert sysid.results["gyro_bias"].ok
        assert sysid.ctx.params.gyro_bias_z == pytest.approx(engine.robots[3].params.gyro_bias_z, abs=0.003)
        assert sysid.truth_comparison()       # sim_truth と比較できる
        gains = sysid.suggested_gains()
        assert {"kp", "ki", "kff", "tau_ff", "u_deadzone", "wheel_radius", "tread"} <= set(gains)
    finally:
        runner.stop()
        core.close()
