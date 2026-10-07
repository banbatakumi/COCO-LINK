"""シミュレーションを実時間で回し、仮想ロボットノード・ビジョンエミュレータの送信を行うスレッド."""

from __future__ import annotations

import logging
import threading
import time

from ..common.config import NetworkConfig
from ..net import UdpEndpoint
from .engine import PHYSICS_HZ, SimEngine
from .virtual_robot_node import VirtualRobotNode
from .vision_emulator import VisionEmulator

log = logging.getLogger(__name__)


class SimRunner:
    """SimEngine を固定ステップ・実時間で進める.

    実時間より遅れた場合は最大 `max_catchup` ステップまで追いつき、それ以上は諦める（処理落ち）。
    """

    def __init__(self, engine: SimEngine, net: NetworkConfig | None = None, speed: float = 1.0,
                 network: bool = True, vision: VisionEmulator | None = None):
        self.engine = engine
        self.net = net or NetworkConfig.load()
        self.speed = speed
        self.paused = False
        self.network = network
        self.vision = vision or VisionEmulator(engine)
        self.nodes: dict[int, VirtualRobotNode] = {}
        self._vision_ep = UdpEndpoint("vision", self.net.bind_host, 0, name="sim-vision") if network else None
        self._thread: threading.Thread | None = None
        self._running = False
        self.real_time_factor = 1.0
        self.max_catchup = 20

    def start(self) -> SimRunner:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="sim", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        for n in self.nodes.values():
            n.close()
        self.nodes.clear()
        if self._vision_ep:
            self._vision_ep.close()

    def _sync_nodes(self) -> None:
        """エンジン内のロボットと UDP ノードを一致させる."""
        with self.engine.lock:
            ids = set(self.engine.robots)
        for rid in ids - set(self.nodes):
            try:
                self.nodes[rid] = VirtualRobotNode(self.engine, rid, self.net).start()
            except OSError as e:
                log.error("robot %d: port bind failed: %s", rid, e)
                self.engine.remove_robot(rid)
        for rid in set(self.nodes) - ids:
            self.nodes.pop(rid).close()

    def _loop(self) -> None:
        dt = 1.0 / PHYSICS_HZ
        next_t = time.perf_counter()
        wall_prev, sim_prev = next_t, self.engine.t
        while self._running:
            if self.network:
                self._sync_nodes()
            now = time.perf_counter()
            steps = 0
            while next_t <= now and steps < self.max_catchup:
                if not self.paused:
                    self.engine.step(dt)
                    for node in list(self.nodes.values()):
                        node.update(dt)
                    for ws in self.vision.update(dt, self.engine.t):
                        if self._vision_ep:
                            self._vision_ep.send(ws, (self.net.operator_host, self.net.operator_vision_port))
                next_t += dt / max(self.speed, 1e-3)
                steps += 1
            if steps >= self.max_catchup:
                next_t = now  # 処理落ち: 追いつくのを諦める
            if now - wall_prev >= 1.0:
                self.real_time_factor = (self.engine.t - sim_prev) / (now - wall_prev)
                wall_prev, sim_prev = now, self.engine.t
            time.sleep(max(0.0, min(0.005, next_t - time.perf_counter())))
