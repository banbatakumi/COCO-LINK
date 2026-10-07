"""同定試験の定義. 各試験は「指令のスケジュール (segments)」と「推定 (estimate)」の組.

新しい試験を加えるときは Experiment を継承して EXPERIMENTS に登録する。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from ..modes.base import RobotCommand, param
from ..robot.params import RobotParams
from . import estimators as est
from .recording import Recording


@dataclass
class SysIdOptions:
    max_duty: float = param(0.5, "ランプ最大 duty", min=0.2, max=1.0, step=0.05)
    ramp_time: float = param(8.0, "ランプ時間 [s]", min=3.0, max=20.0, step=1.0)
    step_levels: str = param("0.3,0.45,0.6", "ステップ duty（カンマ区切り）")
    step_time: float = param(1.0, "ステップ保持 [s]", min=0.3, max=3.0, step=0.1)
    straight_speed: float = param(0.15, "直進速度 [m/s]", min=0.05, max=0.4, step=0.01)
    straight_dist: float = param(0.4, "直進距離 [m]", min=0.1, max=1.5, step=0.05)
    spin_rate: float = param(2.0, "旋回角速度 [rad/s]", min=0.5, max=5.0, step=0.1)
    spin_time: float = param(3.0, "旋回時間 [s]", min=1.0, max=8.0, step=0.5)
    lam: float = param(0.05, "PI 設計 閉ループ時定数 λ [s]", min=0.01, max=0.3, step=0.005)


@dataclass
class Segment:
    phase: str
    duration: float
    command: Callable[[float], RobotCommand]     # フェーズ開始からの経過時間 → 指令


@dataclass
class Plot:
    title: str
    xlabel: str
    ylabel: str
    series: list[tuple[str, np.ndarray, np.ndarray]] = field(default_factory=list)


@dataclass
class ExperimentResult:
    name: str
    ok: bool = True
    message: str = ""
    params: dict[str, float] = field(default_factory=dict)     # RobotParams のフィールド名 → 値
    metrics: dict[str, float] = field(default_factory=dict)    # 当てはまりの良さなど
    plots: list[Plot] = field(default_factory=list)


@dataclass
class SysIdContext:
    params: RobotParams                         # 現時点の最良推定（同定が進むと更新される）
    options: SysIdOptions
    fw_params: dict[str, float] = field(default_factory=dict)   # ロボットが今使っているパラメータ


def hold(phase: str, duration: float) -> Segment:
    return Segment(phase, duration, lambda t: RobotCommand())


def wheels(phase: str, duration: float, fn: Callable[[float], tuple[float, float]]) -> Segment:
    return Segment(phase, duration, lambda t: RobotCommand(wheel=fn(t), safety=False))


def coast(phase: str, duration: float) -> Segment:
    """開ループで duty 0（閉ループの停止制御を効かせず、自然に止まらせる）."""
    return wheels(phase, duration, lambda t: (0.0, 0.0))


def vel(phase: str, duration: float, vx: float, wz: float) -> Segment:
    return Segment(phase, duration, lambda t: RobotCommand(vx=vx, wz=wz))


class Experiment:
    name: str = ""
    label: str = ""
    description: str = ""
    needs_vision: bool = False

    def segments(self, ctx: SysIdContext) -> list[Segment]:
        raise NotImplementedError

    def estimate(self, rec: Recording, ctx: SysIdContext) -> ExperimentResult:
        raise NotImplementedError

    def duration(self, ctx: SysIdContext) -> float:
        return sum(s.duration for s in self.segments(ctx))


# ======================================================================== 1. ジャイロバイアス
class GyroBias(Experiment):
    name = "gyro_bias"
    label = "1. ジャイロバイアス"
    description = "3 秒静止してジャイロ z 軸の零点ずれとノイズを測る。"

    def segments(self, ctx):
        return [hold("hold", 3.0)]

    def estimate(self, rec, ctx):
        s = rec.tel_in("hold", tail=0.8)
        gz = np.array([x.gz for x in s])
        bias, std = est.gyro_bias(gz)
        t = np.array([x.t_robot for x in s])
        return ExperimentResult(self.name, params={"gyro_bias_z": bias}, metrics={"gyro_noise_std": std},
                                message=f"バイアス {bias:+.4f} rad/s, ノイズ σ={std:.4f} rad/s",
                                plots=[Plot("静止中のジャイロ z", "t [s]", "gz [rad/s]",
                                            [("gz", t - t[0] if len(t) else t, gz)])])


# ======================================================================== 2. 不感帯 + 静的ゲイン
class DeadzoneRamp(Experiment):
    name = "deadzone"
    label = "2. 不感帯（duty ランプ）"
    description = "その場旋回で duty をゆっくり増やし、回り始める duty（不感帯）と静的ゲインを測る。"

    def segments(self, ctx):
        o = ctx.options
        T, umax = o.ramp_time, o.max_duty
        return [coast("pre", 0.5),
                wheels("ramp_pos", T, lambda t: (umax * t / T, -umax * t / T)),
                coast("rest", 1.0),
                wheels("ramp_neg", T, lambda t: (-umax * t / T, umax * t / T)),
                coast("post", 0.5)]

    def estimate(self, rec, ctx):
        v_nom = ctx.params.batt_nominal_v
        out = ExperimentResult(self.name)
        plot = Plot("duty ランプ: 実効角速度 |ω|·Vnom/V vs |duty|", "|duty|", "|ω| [rad/s]")
        for side in ("l", "r"):
            s = rec.tel_in("ramp_pos") + rec.tel_in("ramp_neg")
            u = np.array([getattr(x, f"duty_{side}") for x in s])
            w = np.array([getattr(x, f"w{side}") for x in s])
            v = np.array([x.batt_v for x in s])
            fit = est.static_gain_deadzone(u, w, v, v_nom)
            # ランプ中の ω は入力より τ 遅れるので、u0 は (ランプ速度 × τ) だけ大きく出る。現時点の τ で補正する
            ramp_rate = ctx.options.max_duty / ctx.options.ramp_time
            fit.u0 -= ramp_rate * getattr(ctx.params, f"motor_tau_{side}")
            out.params[f"deadzone_{side}"] = fit.u0
            out.metrics[f"static_gain_{side}"] = fit.K
            out.metrics[f"r2_{side}"] = fit.r2
            plot.series.append((f"{side.upper()} 実測", np.abs(u), np.abs(w) * v_nom / v))
            xs = np.linspace(0, ctx.options.max_duty, 50)
            plot.series.append((f"{side.upper()} 当てはめ", xs, np.maximum(0, fit.K * (xs - fit.u0))))
        out.plots.append(plot)
        out.message = (f"不感帯 L={out.params['deadzone_l']:.3f} R={out.params['deadzone_r']:.3f} / "
                       f"静的ゲイン L={out.metrics['static_gain_l']:.1f} R={out.metrics['static_gain_r']:.1f} rad/s")
        return out


# ======================================================================== 3. ステップ応答 (K, τ)
class StepResponse(Experiment):
    name = "step"
    label = "3. モータ特性（duty ステップ）"
    description = "その場旋回で duty をステップ状に変え、一次遅れモデルのゲイン K と時定数 τ を推定する。"

    def levels(self, ctx) -> list[float]:
        return [float(x) for x in ctx.options.step_levels.split(",") if x.strip()]

    def segments(self, ctx):
        segs = [coast("pre", 0.5)]
        for i, u in enumerate(self.levels(ctx)):
            sgn = 1 if i % 2 == 0 else -1
            segs.append(wheels(f"step{i}", ctx.options.step_time, lambda t, u=u, s=sgn: (s * u, -s * u)))
            segs.append(coast(f"rest{i}", 0.8))   # duty 0 の自然減衰も同定に使う
        return segs

    def estimate(self, rec, ctx):
        with rec.lock:
            s = [x for x in rec.telemetry if x.phase.startswith(("pre", "step", "rest"))]
        t = np.array([x.t_robot for x in s])
        v = np.array([x.batt_v for x in s])
        v_nom = ctx.params.batt_nominal_v
        out = ExperimentResult(self.name)
        for side in ("l", "r"):
            th = np.array([getattr(x, f"theta_{side}") for x in s])
            u = np.array([getattr(x, f"duty_{side}") for x in s])
            u0 = getattr(ctx.params, f"deadzone_{side}")
            fit = est.first_order(t, th, u, v, v_nom, u0)
            out.params[f"motor_gain_{side}"] = fit.K
            out.params[f"motor_tau_{side}"] = fit.tau
            out.metrics[f"r2_{side}"] = fit.r2
            out.metrics[f"arx_K_{side}"] = fit.arx_K
            out.metrics[f"arx_tau_{side}"] = fit.arx_tau
            out.plots.append(Plot(f"ステップ応答（{'左' if side == 'l' else '右'}輪）", "t [s]", "ω [rad/s]",
                                  [("実測 Δθ/Δt", fit.t, fit.w_meas), ("モデル", fit.t, fit.w_model)]))
        p = out.params
        out.message = (f"K L={p['motor_gain_l']:.1f} R={p['motor_gain_r']:.1f} rad/s/duty, "
                       f"τ L={p['motor_tau_l'] * 1000:.0f} R={p['motor_tau_r'] * 1000:.0f} ms "
                       f"(R² {out.metrics['r2_l']:.3f}/{out.metrics['r2_r']:.3f})")
        return out


def _mean_pose(samples) -> tuple[float, float]:
    return float(np.mean([s.x for s in samples])), float(np.mean([s.y for s in samples]))


def _mean_theta(samples) -> tuple[float, float]:
    return float(np.mean([s.theta_l for s in samples])), float(np.mean([s.theta_r for s in samples]))


# ======================================================================== 4. 直進 (車輪半径)
class Straight(Experiment):
    name = "straight"
    label = "4. 車輪半径（直進）"
    description = "前進→後退し、ビジョンで測った移動距離とエンコーダ回転角から車輪半径を求める。前後 0.5 m の空間が必要。"
    needs_vision = True

    def segments(self, ctx):
        o = ctx.options
        T = o.straight_dist / o.straight_speed
        return [hold("hold0", 1.2), vel("fwd", T, o.straight_speed, 0.0), hold("hold1", 1.5),
                vel("back", T, -o.straight_speed, 0.0), hold("hold2", 1.5)]

    def estimate(self, rec, ctx):
        holds = ["hold0", "hold1", "hold2"]
        vis = [rec.vis_in(h, tail=0.5) for h in holds]
        tel = [rec.tel_in(h, tail=0.5) for h in holds]
        if any(len(v) < 3 for v in vis):
            return ExperimentResult(self.name, ok=False, message="ビジョンでロボットが見えていません")
        pos = [_mean_pose(v) for v in vis]
        ang = [_mean_theta(t) for t in tel]
        dists, wheel_angles = [], []
        for a, b, ta, tb in zip(pos, pos[1:], ang, ang[1:], strict=False):
            dists.append(float(np.hypot(b[0] - a[0], b[1] - a[1])))
            wheel_angles.append(((tb[0] - ta[0]) + (tb[1] - ta[1])) / 2)
        r = est.wheel_radius(dists, wheel_angles)
        # 直進中の向きの変化（左右の車輪半径差の目安）
        vs = rec.vis_between("hold0", "hold1")
        drift = np.degrees(est.unwrap_sum(np.array([s.th for s in vs]))) if vs else float("nan")
        xs = np.array([s.x for s in rec.vis_between("hold0", "hold2")])
        ys = np.array([s.y for s in rec.vis_between("hold0", "hold2")])
        return ExperimentResult(self.name, params={"wheel_radius": r},
                                metrics={"distance_fwd": dists[0], "distance_back": dists[1], "heading_drift_deg": drift},
                                message=f"車輪半径 r = {r * 1000:.2f} mm（移動 {dists[0]:.3f}/{dists[1]:.3f} m, "
                                        f"直進中の向き変化 {drift:+.1f}°）",
                                plots=[Plot("直進中の軌跡（ビジョン）", "x [m]", "y [m]", [("軌跡", xs, ys)])])


# ======================================================================== 5. その場旋回 (トレッド)
class Spin(Experiment):
    name = "spin"
    label = "5. トレッド幅（その場旋回）"
    description = "左右に旋回し、ビジョン（無ければジャイロ）の回転角とエンコーダの差から実効トレッド幅を求める。"

    def segments(self, ctx):
        o = ctx.options
        return [hold("hold0", 1.2), vel("ccw", o.spin_time, 0.0, o.spin_rate), hold("hold1", 1.5),
                vel("cw", o.spin_time, 0.0, -o.spin_rate), hold("hold2", 1.5)]

    def estimate(self, rec, ctx):
        r = ctx.params.wheel_radius
        bias = ctx.params.gyro_bias_z
        diffs, yaw_vis, yaw_gyro = [], [], []
        for a, b in (("hold0", "hold1"), ("hold1", "hold2")):
            ta, tb = _mean_theta(rec.tel_in(a, 0.5)), _mean_theta(rec.tel_in(b, 0.5))
            diffs.append((tb[1] - ta[1]) - (tb[0] - ta[0]))
            vs = rec.vis_between(a, b)
            yaw_vis.append(est.unwrap_sum(np.array([s.th for s in vs])) if len(vs) > 10 else float("nan"))
            ts = rec.tel_between(a, b)
            t = np.array([s.t_robot for s in ts])
            gz = np.array([s.gz for s in ts]) - bias
            yaw_gyro.append(float(np.sum(gz[:-1] * np.diff(t))) if len(t) > 2 else float("nan"))
        use_vision = all(np.isfinite(yaw_vis)) and all(abs(y) > 0.5 for y in yaw_vis)
        yaw = yaw_vis if use_vision else yaw_gyro
        b = est.tread(r, diffs, yaw)
        b_gyro = est.tread(r, diffs, yaw_gyro) if all(np.isfinite(yaw_gyro)) else float("nan")
        src = "ビジョン" if use_vision else "ジャイロ"
        return ExperimentResult(self.name, params={"tread": b},
                                metrics={"tread_gyro": b_gyro, "yaw_ccw_deg": float(np.degrees(yaw[0])),
                                         "yaw_cw_deg": float(np.degrees(yaw[1]))},
                                message=f"トレッド b = {b * 1000:.1f} mm（{src}基準。ジャイロ基準 {b_gyro * 1000:.1f} mm）")


# ======================================================================== 6. 閉ループ速度応答（検証）
class VelocityStep(Experiment):
    name = "verify"
    label = "6. 検証（速度ステップ）"
    description = "cmd_vel のステップ応答で、車輪速度制御の立ち上がり時間・オーバーシュート・定常偏差を評価する。"

    def segments(self, ctx):
        v = ctx.options.straight_speed
        return [hold("hold0", 0.5), vel("step", 1.5, v, 0.0), hold("hold1", 1.0), vel("back", 1.5, -v, 0.0),
                hold("hold2", 0.5)]

    def estimate(self, rec, ctx):
        r_fw = ctx.fw_params.get("wheel_radius", ctx.params.wheel_radius)
        target = ctx.options.straight_speed / r_fw
        s = rec.tel_in("step")
        if len(s) < 5:
            return ExperimentResult(self.name, ok=False, message="データ不足")
        t = np.array([x.t_robot for x in s])
        w = np.array([(x.wl + x.wr) / 2 for x in s])
        m = est.step_metrics(t, w, target)
        return ExperimentResult(self.name, metrics={"rise_time_s": m.rise_time, "overshoot_pct": m.overshoot,
                                                    "steady_error_pct": m.steady_error},
                                message=f"立ち上がり {m.rise_time * 1000:.0f} ms, オーバーシュート {m.overshoot:.1f} %, "
                                        f"定常偏差 {m.steady_error:+.1f} %",
                                plots=[Plot("速度ステップ応答", "t [s]", "ω [rad/s]",
                                            [("車輪速度 (平均)", t - t[0], w),
                                             ("目標", t - t[0], np.full_like(t, target))])])


EXPERIMENTS: dict[str, Experiment] = {e.name: e for e in
                                      (GyroBias(), DeadzoneRamp(), StepResponse(), Straight(), Spin(), VelocityStep())}
