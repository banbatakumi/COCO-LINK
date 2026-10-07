"""システム同定の実行: 試験スケジュールに従って指令を出し、データを記録し、推定する.

ControlCore の「オーバーライド」として動く（モードより優先、手動操作・E-STOP より劣後）。
"""

from __future__ import annotations

import datetime
import logging
from dataclasses import asdict, replace

from ..fleet.control_core import ControlCore
from ..fleet.world_model import World
from ..modes.base import RobotCommand
from ..protocol import messages as m
from ..robot.params import RobotParams
from .estimators import imc_pi
from .experiments import EXPERIMENTS, ExperimentResult, SysIdContext, SysIdOptions
from .recording import Recording, TelemetrySample, VisionSample

log = logging.getLogger(__name__)


class SysIdRunner:
    def __init__(self, core: ControlCore):
        self.core = core
        self.active = False
        self.robot_id: int | None = None
        self.queue: list[str] = []
        self.current: str | None = None
        self.phase = "idle"
        self.results: dict[str, ExperimentResult] = {}
        self.ctx: SysIdContext | None = None
        self.recording: Recording | None = None
        self.log: list[str] = []
        self._t = 0.0
        self._seg_idx = 0
        self._seg_t = 0.0
        self._saved_telemetry_hz = 50.0
        core.fleet.listeners.append(self._on_robot_msg)
        core.world_model.listeners.append(self._on_world)
        core.set_override(self)

    # ------------------------------------------------------------ 操作
    def start(self, robot_id: int, experiments: list[str], options: SysIdOptions | None = None) -> None:
        proxy = self.core.fleet.get(robot_id)
        params = RobotParams.load(robot_id)
        self.ctx = SysIdContext(params=params, options=options or SysIdOptions(),
                                fw_params=dict(proxy.params) if proxy else {})
        self.robot_id = robot_id
        self.queue = [e for e in experiments if e in EXPERIMENTS]
        self.results = {}
        self.log = [f"ロボット #{robot_id} の同定を開始: {', '.join(self.queue)}"]
        self.core.fleet.request_params(robot_id)
        # 制御周期 (100 Hz) と同じ周期でテレメトリを取る。50 Hz だと 1 サンプル区間に duty 更新が 2 回入り、
        # 入力と応答の対応がずれて時定数 τ を過小評価する (docs/system_identification.md §5)
        self._saved_telemetry_hz = self.ctx.fw_params.get("telemetry_hz", 50)
        self.core.send_direct(robot_id, m.SetParam(key="telemetry_hz", value=100))
        self._next_experiment()
        self.active = self.current is not None

    def abort(self, reason: str = "中断しました") -> None:
        if self.active:
            self.log.append(f"⚠ {reason}")
            self._restore_telemetry_rate()
        self.active = False
        self.current = None
        self.phase = "idle"

    @property
    def progress(self) -> float:
        if not self.active or self.ctx is None or self.current is None:
            return 0.0
        exp = EXPERIMENTS[self.current]
        return min(1.0, self._t / max(exp.duration(self.ctx), 1e-3))

    # ------------------------------------------------------------ 結果
    def identified_params(self) -> RobotParams | None:
        if self.ctx is None:
            return None
        return self.ctx.params

    def suggested_gains(self) -> dict[str, float]:
        """同定結果からファームウェアに書き込むパラメータを計算する."""
        p = self.identified_params()
        if p is None or self.ctx is None:
            return {}
        K = (p.motor_gain_l + p.motor_gain_r) / 2
        tau = (p.motor_tau_l + p.motor_tau_r) / 2
        kp, ki, kff = imc_pi(K, tau, self.ctx.options.lam)
        return {"kp": round(kp, 5), "ki": round(ki, 4), "kff": round(kff, 5), "tau_ff": round(tau, 4),
                "u_deadzone": round((p.deadzone_l + p.deadzone_r) / 2, 4),
                "wheel_radius": round(p.wheel_radius, 5), "tread": round(p.tread, 5)}

    def apply_to_robot(self) -> dict[str, float]:
        """同定結果をロボット (set_param) と operator の WorldModel に反映する."""
        if self.robot_id is None:
            return {}
        gains = self.suggested_gains()
        for k, v in gains.items():
            self.core.send_direct(self.robot_id, m.SetParam(key=k, value=v))
        self.core.world_model.set_robot_params(self.robot_id, self.identified_params())
        self.log.append("ロボットへ書き込み: " + ", ".join(f"{k}={v:g}" for k, v in gains.items()))
        return gains

    def save(self, root=None):
        if self.robot_id is None or self.ctx is None:
            return None
        meta = {"date": datetime.datetime.now().isoformat(timespec="seconds"),
                "experiments": {k: {"message": r.message, **{mk: round(mv, 6) for mk, mv in r.metrics.items()
                                                             if mv == mv}}
                                for k, r in self.results.items()},
                "suggested_firmware_params": self.suggested_gains()}
        path = self.ctx.params.save(self.robot_id, root, extra=meta)
        self.log.append(f"保存しました: {path}")
        return path

    def truth_comparison(self) -> list[tuple[str, float, float, float]]:
        """シミュレータの真値との比較 (名前, 推定, 真値, 誤差%). 実機では空."""
        if self.robot_id is None or self.ctx is None:
            return []
        proxy = self.core.fleet.get(self.robot_id)
        if proxy is None or not proxy.sim_truth:
            return []
        rows = []
        est = asdict(self.ctx.params)
        for name in {k for r in self.results.values() for k in r.params}:
            if name in proxy.sim_truth:
                truth = proxy.sim_truth[name]
                err = (est[name] - truth) / truth * 100 if abs(truth) > 1e-6 else float("nan")
                rows.append((name, est[name], truth, err))
        return sorted(rows)

    # ------------------------------------------------------------ Controller プロトコル
    def step(self, world: World, dt: float) -> dict[int, RobotCommand]:
        if not self.active or self.current is None or self.ctx is None or self.robot_id is None:
            return {}
        rs = world.robots.get(self.robot_id)
        if rs is None or not rs.connected:
            self.abort("ロボットとの通信が切れました")
            return {}
        if rs.state in (m.STATE_ESTOP, m.STATE_LOW_BATT, m.STATE_FAULT):
            self.abort(f"ロボットが {rs.state} になりました")
            return {}
        segs = EXPERIMENTS[self.current].segments(self.ctx)
        self._t += dt
        self._seg_t += dt
        while self._seg_idx < len(segs) and self._seg_t >= segs[self._seg_idx].duration:
            self._seg_t -= segs[self._seg_idx].duration
            self._seg_idx += 1
        if self._seg_idx >= len(segs):
            self._finish_experiment()
            return {self.robot_id: RobotCommand()}
        seg = segs[self._seg_idx]
        self.phase = seg.phase
        return {self.robot_id: seg.command(self._seg_t)}

    # ------------------------------------------------------------ 内部
    def _next_experiment(self) -> None:
        self.current = self.queue.pop(0) if self.queue else None
        self._t = self._seg_t = 0.0
        self._seg_idx = 0
        if self.current is not None and self.robot_id is not None:
            self.recording = Recording(self.robot_id)
            exp = EXPERIMENTS[self.current]
            self.log.append(f"▶ {exp.label}（約 {exp.duration(self.ctx):.0f} 秒）")

    def _finish_experiment(self) -> None:
        assert self.current is not None and self.ctx is not None and self.recording is not None
        exp = EXPERIMENTS[self.current]
        self.phase = "idle"
        try:
            res = exp.estimate(self.recording, self.ctx)
        except Exception as e:   # 推定失敗でも他の試験は続ける
            log.exception("estimation failed")
            res = ExperimentResult(exp.name, ok=False, message=f"推定に失敗: {e}")
        self.results[exp.name] = res
        if res.ok and res.params:
            self.ctx.params = replace(self.ctx.params, **res.params)
        self.log.append(("✓ " if res.ok else "✗ ") + res.message)
        self._next_experiment()
        if self.current is None:
            self.active = False
            self._restore_telemetry_rate()
            self.log.append("すべての試験が終了しました。結果を確認して「保存」「ロボットへ書き込み」を押してください。")

    def _restore_telemetry_rate(self) -> None:
        if self.robot_id is not None:
            self.core.send_direct(self.robot_id, m.SetParam(key="telemetry_hz", value=self._saved_telemetry_hz))

    def _on_robot_msg(self, direction: str, rid: int, msg: m.Message, now: float, t_ms: int | None) -> None:
        if (direction != "rx" or not self.active or rid != self.robot_id or self.recording is None
                or not isinstance(msg, m.Telemetry) or t_ms is None):
            return
        s = TelemetrySample(t_robot=t_ms / 1000.0, t_rx=now, phase=self.phase,
                            theta_l=msg.enc.left_rad, theta_r=msg.enc.right_rad, wl=msg.enc.wl, wr=msg.enc.wr,
                            duty_l=msg.applied.left, duty_r=msg.applied.right, gz=msg.imu.gz, batt_v=msg.batt_v)
        with self.recording.lock:
            self.recording.telemetry.append(s)

    def _on_world(self, ws: m.WorldState, now: float) -> None:
        if not self.active or self.recording is None:
            return
        for d in ws.robots:
            if d.id == self.robot_id:
                with self.recording.lock:
                    self.recording.vision.append(VisionSample(now - ws.latency_ms / 1000.0, self.phase, d.x, d.y, d.th))
