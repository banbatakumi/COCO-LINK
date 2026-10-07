"""ビジョンエミュレータ: シミュレーションの真値にノイズ・遅延・欠測を加えて world_state を作る.

実際のビジョンシステム (vision/publisher.py) と同じメッセージを出すので、操作GUIは区別できない。
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass

from ..common.clock import monotonic_ms
from ..common.geometry import wrap_angle
from ..protocol import messages as m
from .engine import SimEngine


@dataclass
class VisionNoise:
    pos_sigma: float = 0.003        # [m]
    th_sigma: float = math.radians(1.0)
    latency_s: float = 0.05         # 撮像〜送信の遅延
    dropout: float = 0.02           # 1 台が 1 フレーム見えない確率
    rate_hz: float = 30.0


class VisionEmulator:
    def __init__(self, engine: SimEngine, noise: VisionNoise | None = None, seed: int = 0):
        self.engine = engine
        self.noise = noise or VisionNoise()
        self.rng = random.Random(seed)
        self.frame = 0
        self._queue: deque[tuple[float, m.WorldState]] = deque()
        self._acc = 0.0

    def capture(self) -> m.WorldState:
        """現在の真値から 1 フレームを生成する."""
        n, rng, e = self.noise, self.rng, self.engine
        w, h = e.field_wh
        with e.lock:
            robots = []
            for r in e.robots.values():
                if rng.random() < n.dropout:
                    continue
                p = r.pose
                if not (0 <= p.x <= w and 0 <= p.y <= h):
                    continue
                robots.append(m.DetectedRobot(id=r.robot_id, x=round(p.x + rng.gauss(0, n.pos_sigma), 4),
                                              y=round(p.y + rng.gauss(0, n.pos_sigma), 4),
                                              th=round(wrap_angle(p.th + rng.gauss(0, n.th_sigma)), 4),
                                              conf=round(rng.uniform(0.85, 1.0), 2)))
            groups: dict[str, list[m.DetectedObject]] = {"person": [], "obstacle": [], "cargo": []}
            for b in e.bodies:
                groups[b.kind].append(m.DetectedObject(x=round(b.x + rng.gauss(0, n.pos_sigma), 4),
                                                       y=round(b.y + rng.gauss(0, n.pos_sigma), 4), r=round(b.r, 3)))
        self.frame += 1
        return m.WorldState(frame=self.frame, stamp_ms=monotonic_ms(), latency_ms=n.latency_s * 1000.0,
                            field=m.FieldSize(w, h), robots=robots, persons=groups["person"][:10],
                            obstacles=groups["obstacle"][:10], cargo=groups["cargo"][:10])

    def update(self, dt: float, now: float) -> list[m.WorldState]:
        """dt 経過させ、遅延を経て送るべきフレームを返す."""
        self._acc += dt
        if self._acc >= 1.0 / self.noise.rate_hz:
            self._acc -= 1.0 / self.noise.rate_hz
            self._queue.append((now + self.noise.latency_s, self.capture()))
        out = []
        while self._queue and self._queue[0][0] <= now:
            out.append(self._queue.popleft()[1])
        return out
