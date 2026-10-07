"""仮想ロボットノード: シミュレータ内の FirmwareModel を UDP で公開する.

実機 ESP32 の通信部 (docs/firmware_spec.md §7) と同じ振る舞い:
  - hello を 1 Hz で operator_host:operator_robot_port へ送る
  - 最後にメッセージをくれた送信元を operator として記憶し、telemetry をそこへ送る
"""

from __future__ import annotations

from ..common.config import NetworkConfig
from ..net import Address, UdpEndpoint
from ..protocol import Envelope
from ..protocol import messages as m
from ..protocol.ports import sim_robot_port
from .engine import SimEngine


class VirtualRobotNode:
    def __init__(self, engine: SimEngine, robot_id: int, net: NetworkConfig, port: int | None = None):
        self.engine = engine
        self.robot_id = robot_id
        self.net = net
        self.operator: Address | None = None
        self.endpoint = UdpEndpoint(f"robot:{robot_id}", net.bind_host,
                                    sim_robot_port(robot_id, net.robot_port) if port is None else port,
                                    on_message=self._on_message, name=f"vrobot{robot_id}")
        self._hello_acc = 1.0
        self._telem_acc = 0.0

    def start(self) -> VirtualRobotNode:
        self.endpoint.start()
        return self

    def close(self) -> None:
        self.endpoint.close()

    @property
    def port(self) -> int:
        return self.endpoint.port

    def _on_message(self, env: Envelope, addr: Address) -> None:
        self.operator = addr
        for reply in self.engine.deliver(self.robot_id, env.msg):
            self.endpoint.send(reply, addr)

    def update(self, dt: float) -> None:
        """シミュレーションループから呼ばれ、周期送信を行う."""
        with self.engine.lock:
            robot = self.engine.robots.get(self.robot_id)
            if robot is None:
                return
            fw = robot.firmware
            telem_hz = fw.params["telemetry_hz"]
            hello = fw.hello()
            self._telem_acc += dt
            telem = None
            if self._telem_acc >= 1.0 / telem_hz:
                self._telem_acc %= 1.0 / telem_hz
                telem = fw.telemetry()
            truth = m.SimTruth(values=robot.truth_dict())
            # 実機同様、t_ms はロボット自身の時計（テレメトリ値を取得した制御周期の時刻）
            t_ms = int(fw.t * 1000) & 0xFFFFFFFF
        self._hello_acc += dt
        if self._hello_acc >= 1.0:
            self._hello_acc = 0.0
            self.endpoint.send(hello, (self.net.operator_host, self.net.operator_robot_port))
            if self.operator is not None:
                self.endpoint.send(truth, self.operator)
        if telem is not None and self.operator is not None:
            self.endpoint.send(telem, self.operator, t_ms)
