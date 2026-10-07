"""operator 側から見た 1 台のロボット（実機でも仮想でも同じ）."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from ..net import Address
from ..protocol import messages as m


@dataclass
class RobotProxy:
    robot_id: int
    addr: Address
    hello: m.Hello | None = None
    telemetry: m.Telemetry | None = None
    params: dict[str, float] = field(default_factory=dict)
    sim_truth: dict[str, float] | None = None     # シミュレータのみ
    last_rx: float = 0.0
    last_telemetry_t: float = 0.0
    rtt_ms: float | None = None
    _telem_times: deque = field(default_factory=lambda: deque(maxlen=50))

    @property
    def telemetry_hz(self) -> float:
        ts = self._telem_times
        if len(ts) < 2 or ts[-1] == ts[0]:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def is_connected(self, now: float, timeout: float = 2.0) -> bool:
        return now - self.last_rx < timeout

    @property
    def state(self) -> str:
        return self.telemetry.state if self.telemetry else "-"

    @property
    def is_simulated(self) -> bool:
        """表示用。制御ロジックではこの値で分岐しないこと（CLAUDE.md ルール1）."""
        return self.sim_truth is not None
