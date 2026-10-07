"""WorldModel: ビジョン (world_state) とテレメトリを統合し、制御に使う「世界のスナップショット」を作る.

姿勢推定の方針（シンプルで説明しやすいもの）:
  1. ビジョンの姿勢を基本とする
  2. ビジョンの遅延 (latency_ms + 受信からの経過時間) を、エンコーダから求めた (vx, wz) で外挿して補償
  3. ビジョンが一時的に途切れた (< odom_timeout_s) ときはオドメトリで推測航法、それ以上は「姿勢不明」
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from ..common.config import NetworkConfig
from ..common.geometry import Pose2D, Vec2
from ..net import Address, UdpEndpoint
from ..protocol import Envelope
from ..protocol import messages as m
from ..robot.kinematics import integrate_unicycle, wheel_to_body
from ..robot.params import RobotParams
from .fleet_manager import FleetManager


@dataclass
class RobotState:
    robot_id: int
    connected: bool = False
    pose: Pose2D | None = None
    pose_source: str = "none"          # "vision" / "odom" / "none"
    pose_age: float = float("inf")     # 最後にビジョンで見えてからの時間 [s]
    vx: float = 0.0                    # エンコーダから推定した機体速度
    wz: float = 0.0
    telemetry: m.Telemetry | None = None
    params: RobotParams = field(default_factory=RobotParams)

    @property
    def state(self) -> str:
        return self.telemetry.state if self.telemetry else "-"

    @property
    def us_front(self) -> float | None:
        return self.telemetry.us_front_m if self.telemetry else None

    @property
    def us_rear(self) -> float | None:
        return self.telemetry.us_rear_m if self.telemetry else None

    @property
    def usable(self) -> bool:
        """制御対象にできるか（接続中・姿勢既知・走行可能状態）."""
        return self.connected and self.pose is not None and self.state in (m.STATE_IDLE, m.STATE_RUNNING)


@dataclass
class World:
    """ある時刻の世界のスナップショット. モードやテストはこれだけを見る."""

    t: float = 0.0
    field_w: float = 3.0
    field_h: float = 2.0
    robots: dict[int, RobotState] = field(default_factory=dict)
    persons: list[m.DetectedObject] = field(default_factory=list)
    obstacles: list[m.DetectedObject] = field(default_factory=list)
    cargo: list[m.DetectedObject] = field(default_factory=list)
    vision_age: float = float("inf")

    def usable_robots(self) -> dict[int, RobotState]:
        return {rid: r for rid, r in self.robots.items() if r.usable}

    def obstacle_points(self) -> list[tuple[Vec2, float]]:
        return [(Vec2(o.x, o.y), o.r) for o in self.obstacles + self.cargo]


class WorldModel:
    def __init__(self, net: NetworkConfig, fleet: FleetManager, port: int | None = None,
                 clock: Callable[[], float] = time.monotonic, odom_timeout_s: float = 0.5):
        self.net = net
        self.fleet = fleet
        self.clock = clock
        self.odom_timeout_s = odom_timeout_s
        self.lock = threading.Lock()
        self.latest: m.WorldState | None = None
        self.latest_rx: float = -1e9
        self._last_vision_pose: dict[int, tuple[Pose2D, float]] = {}   # 推定に使った最終観測 (姿勢, 観測時刻)
        self._odom: dict[int, tuple[Pose2D, float]] = {}               # 推測航法の状態
        self._params: dict[int, RobotParams] = {}
        self.listeners: list[Callable[[m.WorldState, float], None]] = []
        self.endpoint = UdpEndpoint("op", net.bind_host, net.operator_vision_port if port is None else port,
                                    on_message=self._on_message, name="vision-rx")

    def start(self) -> WorldModel:
        self.endpoint.start()
        return self

    def close(self) -> None:
        self.endpoint.close()

    def robot_params(self, rid: int) -> RobotParams:
        """robot_XX.yaml（同定結果）があればそれを、無ければ公称値を使う."""
        if rid not in self._params:
            self._params[rid] = RobotParams.load(rid)
        return self._params[rid]

    def set_robot_params(self, rid: int, params: RobotParams) -> None:
        self._params[rid] = params

    def _on_message(self, env: Envelope, _addr: Address) -> None:
        if isinstance(env.msg, m.WorldState):
            now = self.clock()
            with self.lock:
                self.latest = env.msg
                self.latest_rx = now
            for fn in list(self.listeners):
                fn(env.msg, now)

    def inject(self, ws: m.WorldState, now: float | None = None) -> None:
        """テストやログ再生用: world_state を直接与える."""
        with self.lock:
            self.latest = ws
            self.latest_rx = self.clock() if now is None else now

    def snapshot(self) -> World:
        now = self.clock()
        with self.lock:
            ws, rx_t = self.latest, self.latest_rx
        world = World(t=now, vision_age=now - rx_t)
        seen: dict[int, m.DetectedRobot] = {}
        if ws is not None:
            world.field_w, world.field_h = ws.field.w or world.field_w, ws.field.h or world.field_h
            world.persons, world.obstacles, world.cargo = list(ws.persons), list(ws.obstacles), list(ws.cargo)
            # 観測時刻 = 受信時刻 - ビジョン内部の遅延
            obs_t = rx_t - ws.latency_ms / 1000.0
            for d in ws.robots:
                seen[d.id] = d
                prev = self._last_vision_pose.get(d.id)
                if prev is None or prev[1] < obs_t - 1e-9 or prev[0] != Pose2D(d.x, d.y, d.th):
                    self._last_vision_pose[d.id] = (Pose2D(d.x, d.y, d.th), obs_t)

        with self.fleet.lock:
            proxies = dict(self.fleet.robots)
        for rid in set(proxies) | set(seen):
            proxy = proxies.get(rid)
            params = self.robot_params(rid)
            st = RobotState(robot_id=rid, params=params)
            if proxy is not None:
                st.connected = proxy.is_connected(now, self.net.disconnect_timeout_s)
                st.telemetry = proxy.telemetry
                if proxy.telemetry:
                    e = proxy.telemetry.enc
                    st.vx, st.wz = wheel_to_body(e.wl, e.wr, params.wheel_radius, params.tread)
            vis = self._last_vision_pose.get(rid)
            if vis is not None:
                pose0, t0 = vis
                st.pose_age = now - t0
                if st.pose_age < self.odom_timeout_s:
                    # 遅延補償 / 一時欠測時の推測航法: 観測時刻から現在までを外挿
                    st.pose = integrate_unicycle(pose0, st.vx, st.wz, max(0.0, st.pose_age))
                    st.pose_source = "vision" if rid in seen and world.vision_age < 0.2 else "odom"
            world.robots[rid] = st
        return world
