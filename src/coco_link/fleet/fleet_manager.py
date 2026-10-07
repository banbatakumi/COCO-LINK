"""FleetManager: hello で接続ロボットを発見し、テレメトリを集め、指令を送る."""

from __future__ import annotations

import logging
import random
import threading
import time
from collections.abc import Callable

from ..common.config import NetworkConfig
from ..net import Address, UdpEndpoint
from ..protocol import Envelope
from ..protocol import messages as m
from .robot_proxy import RobotProxy

log = logging.getLogger(__name__)

# (direction "rx"/"tx", robot_id, message, monotonic time)
MessageListener = Callable[[str, int, m.Message, float], None]


class FleetManager:
    def __init__(self, net: NetworkConfig, port: int | None = None, clock: Callable[[], float] = time.monotonic):
        self.net = net
        self.clock = clock
        self.lock = threading.RLock()
        self.robots: dict[int, RobotProxy] = {}
        self.listeners: list[MessageListener] = []
        self.endpoint = UdpEndpoint("op", net.bind_host, net.operator_robot_port if port is None else port,
                                    on_message=self._on_message, name="fleet")
        self._pings: dict[int, tuple[int, float]] = {}

    def start(self) -> FleetManager:
        self.endpoint.start()
        return self

    def close(self) -> None:
        self.endpoint.close()

    # ------------------------------------------------------------ 受信
    def _on_message(self, env: Envelope, addr: Address) -> None:
        msg = env.msg
        rid = msg.robot_id if isinstance(msg, m.Hello | m.Telemetry) else env.robot_id
        if rid is None:
            return
        now = self.clock()
        with self.lock:
            proxy = self.robots.get(rid)
            if proxy is None:
                if not isinstance(msg, m.Hello | m.Telemetry):
                    return
                proxy = RobotProxy(rid, addr)
                self.robots[rid] = proxy
                log.info("robot %d discovered at %s:%d", rid, *addr)
            proxy.addr = addr       # IP が変わっても追従
            proxy.last_rx = now
            if isinstance(msg, m.Hello):
                proxy.hello = msg
            elif isinstance(msg, m.Telemetry):
                proxy.telemetry = msg
                proxy.last_telemetry_t = now
                proxy._telem_times.append(now)
            elif isinstance(msg, m.Params):
                proxy.params = dict(msg.values)
            elif isinstance(msg, m.SimTruth):
                proxy.sim_truth = dict(msg.values)
            elif isinstance(msg, m.Pong):
                sent = self._pings.get(rid)
                if sent and sent[0] == msg.nonce:
                    proxy.rtt_ms = (now - sent[1]) * 1000.0
        for fn in list(self.listeners):
            fn("rx", rid, msg, now)

    # ------------------------------------------------------------ 送信
    def send(self, robot_id: int, msg: m.Message) -> bool:
        with self.lock:
            proxy = self.robots.get(robot_id)
            addr = proxy.addr if proxy else None
        if addr is None:
            return False
        ok = self.endpoint.send(msg, addr)
        if ok:
            now = self.clock()
            for fn in list(self.listeners):
                fn("tx", robot_id, msg, now)
        return ok

    def send_all(self, msg: m.Message, only_connected: bool = True) -> None:
        for rid in self.connected_ids() if only_connected else list(self.robots):
            self.send(rid, msg)

    def ping_all(self) -> None:
        now = self.clock()
        for rid in self.connected_ids():
            nonce = random.getrandbits(31)
            self._pings[rid] = (nonce, now)
            self.send(rid, m.Ping(nonce=nonce))

    def request_params(self, robot_id: int) -> None:
        self.send(robot_id, m.GetParams())

    # ------------------------------------------------------------ 参照
    def connected_ids(self) -> list[int]:
        now = self.clock()
        with self.lock:
            return sorted(rid for rid, p in self.robots.items() if p.is_connected(now, self.net.disconnect_timeout_s))

    def get(self, robot_id: int) -> RobotProxy | None:
        with self.lock:
            return self.robots.get(robot_id)

    def forget_disconnected(self, older_than_s: float = 30.0) -> None:
        now = self.clock()
        with self.lock:
            for rid in [r for r, p in self.robots.items() if now - p.last_rx > older_than_s]:
                del self.robots[rid]
