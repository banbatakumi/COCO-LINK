"""同定試験中に記録するデータ.

時刻の扱い:
  - テレメトリはロボット自身の時刻 t_ms（制御周期に同期しており、Wi-Fi の遅延揺らぎを含まない）で解析する
  - ビジョンとテレメトリの時刻合わせは行わず、「静止区間の平均」どうしを比べる設計にしている
    （時刻同期が不要になり、実機で頑健）
各サンプルには受信時点の試験フェーズ名 `phase` を付ける。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

import numpy as np


@dataclass
class TelemetrySample:
    t_robot: float      # ロボット時刻 [s]
    t_rx: float         # operator 受信時刻 [s]
    phase: str
    theta_l: float      # 車輪累積角 [rad]
    theta_r: float
    wl: float           # ファーム推定角速度 [rad/s]（LPF 付き）
    wr: float
    duty_l: float       # 実際に出していた duty
    duty_r: float
    gz: float
    batt_v: float


@dataclass
class VisionSample:
    t_rx: float
    phase: str
    x: float
    y: float
    th: float


@dataclass
class Recording:
    robot_id: int
    telemetry: list[TelemetrySample] = field(default_factory=list)
    vision: list[VisionSample] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def tel_array(self, attr: str, phase_prefix: str | None = None) -> np.ndarray:
        with self.lock:
            return np.array([getattr(s, attr) for s in self.telemetry
                             if phase_prefix is None or s.phase.startswith(phase_prefix)], dtype=float)

    def tel_in(self, phase: str, tail: float = 1.0) -> list[TelemetrySample]:
        """フェーズ phase のサンプルのうち、後半 tail（割合）だけ."""
        with self.lock:
            xs = [s for s in self.telemetry if s.phase == phase]
        return xs[int(len(xs) * (1 - tail)):]

    def vis_in(self, phase: str, tail: float = 1.0) -> list[VisionSample]:
        with self.lock:
            xs = [s for s in self.vision if s.phase == phase]
        return xs[int(len(xs) * (1 - tail)):]

    def vis_between(self, first_phase: str, last_phase: str) -> list[VisionSample]:
        """first_phase の開始から last_phase の終了まで（途中のフェーズを含む）."""
        with self.lock:
            xs = list(self.vision)
        idx = [i for i, s in enumerate(xs) if s.phase in (first_phase, last_phase)]
        return xs[idx[0]: idx[-1] + 1] if idx else []

    def tel_between(self, first_phase: str, last_phase: str) -> list[TelemetrySample]:
        with self.lock:
            xs = list(self.telemetry)
        idx = [i for i, s in enumerate(xs) if s.phase in (first_phase, last_phase)]
        return xs[idx[0]: idx[-1] + 1] if idx else []
