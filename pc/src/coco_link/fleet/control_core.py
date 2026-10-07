"""ControlCore: 上位制御のメインループ（Qt 非依存スレッド）.

各周期で
  1. WorldModel からスナップショットを取得
  2. 実行中のモード / オーバーライド（システム同定）/ 手動操作 から指令を得る
  3. 優先度で調停:  E-STOP > 手動操作 > オーバーライド(同定) > モード
  4. 安全フィルタ（超音波による減速）を通して送信
GUI はこのクラスのメソッドを呼ぶだけで、ロボットへ直接は送らない。
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

from ..common.config import NetworkConfig
from ..control.safety import ultrasonic_speed_limit
from ..modes.base import Mode, RobotCommand, Visualization
from ..modes.registry import discover
from ..protocol import messages as m
from .fleet_manager import FleetManager
from .world_model import World, WorldModel

log = logging.getLogger(__name__)

MANUAL_TIMEOUT_S = 0.3
LED_REFRESH_S = 2.0


class Controller(Protocol):
    """モード以外に指令を出すもの（システム同定ランナーなど）."""

    active: bool

    def step(self, world: World, dt: float) -> dict[int, RobotCommand]: ...


@dataclass
class CoreSnapshot:
    world: World = field(default_factory=World)
    commands: dict[int, RobotCommand] = field(default_factory=dict)
    mode_name: str | None = None
    mode_status: str = ""
    viz: Visualization = field(default_factory=Visualization)
    estopped: bool = False
    loop_hz: float = 0.0


class ControlCore:
    def __init__(self, net: NetworkConfig | None = None, fleet: FleetManager | None = None,
                 world_model: WorldModel | None = None, clock: Callable[[], float] = time.monotonic):
        self.net = net or NetworkConfig.load()
        self.clock = clock
        self.fleet = fleet or FleetManager(self.net, clock=clock)
        self.world_model = world_model or WorldModel(self.net, self.fleet, clock=clock)
        self.modes = discover()
        self.lock = threading.RLock()
        self.mode: Mode | None = None
        self.override: Controller | None = None
        self.estopped = False
        self.safety_enabled = True
        self.snapshot = CoreSnapshot()
        self.event_listeners: list[Callable[[str, dict[str, Any]], None]] = []
        self.tick_listeners: list[Callable[[CoreSnapshot], None]] = []   # 制御周期ごと（ロガー等）
        self._manual: dict[int, tuple[float, float, float]] = {}    # rid -> (vx, wz, 時刻)
        self._last_led: dict[int, tuple[tuple, float]] = {}
        self._thread: threading.Thread | None = None
        self._running = False
        self._last_tick: float | None = None
        self._last_ping = 0.0
        self._last_estop_tx = 0.0

    # ------------------------------------------------------------ ライフサイクル
    def start(self, thread: bool = True) -> ControlCore:
        self.fleet.start()
        self.world_model.start()
        if thread:
            self._running = True
            self._thread = threading.Thread(target=self._loop, name="control", daemon=True)
            self._thread.start()
        return self

    def close(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        with contextlib.suppress(Exception):
            self.stop_mode()
        self.fleet.close()
        self.world_model.close()

    def _loop(self) -> None:
        period = 1.0 / self.net.control_hz
        next_t = time.monotonic()
        while self._running:
            try:
                self.tick()
            except Exception:
                log.exception("control tick failed")
            next_t += period
            delay = next_t - time.monotonic()
            if delay < -period:          # 大きく遅れたら追いつくのを諦める
                next_t = time.monotonic()
            time.sleep(max(0.0, delay))

    # ------------------------------------------------------------ GUI から呼ぶ API
    def start_mode(self, name: str, params: Any = None) -> Mode:
        with self.lock:
            if self.mode is not None:
                self.stop_mode()
            mode = self.modes[name](params)
            mode.on_start(self.world_model.snapshot())
            self.mode = mode
        self._emit("mode_start", {"mode": name, "params": _params_dict(mode.params)})
        return mode

    def stop_mode(self) -> None:
        with self.lock:
            mode, self.mode = self.mode, None
        if mode is not None:
            self._send(mode.on_stop(self.world_model.snapshot()))
            self._emit("mode_stop", {"mode": mode.name})

    def update_mode_params(self, params: Any) -> None:
        with self.lock:
            if self.mode is not None:
                self.mode.update_params(params)
                self._emit("mode_params", {"mode": self.mode.name, "params": _params_dict(params)})

    def field_click(self, x: float, y: float) -> None:
        with self.lock:
            if self.mode is not None:
                self.mode.on_field_click(x, y)

    def set_manual(self, robot_id: int, vx: float, wz: float) -> None:
        """手動操作. 0.3 s 以内に再度呼ばれないと失効する（キーを離したら止まる）."""
        with self.lock:
            self._manual[robot_id] = (vx, wz, self.clock())

    def send_direct(self, robot_id: int | None, msg: m.Message) -> None:
        """LED・ブザー・パラメータなどを直接送る. robot_id=None なら全台."""
        if robot_id is None:
            self.fleet.send_all(msg)
        else:
            self.fleet.send(robot_id, msg)

    def estop_all(self) -> None:
        with self.lock:
            self.estopped = True
            self.mode = None
            if self.override is not None:
                self.override.active = False
        self.fleet.send_all(m.EStop(), only_connected=False)
        self._last_estop_tx = self.clock()
        self._emit("estop", {})

    def clear_estop_all(self) -> None:
        with self.lock:
            self.estopped = False
        self.fleet.send_all(m.ClearEStop(), only_connected=False)
        self._emit("clear_estop", {})

    def set_override(self, controller: Controller | None) -> None:
        with self.lock:
            self.override = controller

    # ------------------------------------------------------------ 制御周期
    def tick(self) -> CoreSnapshot:
        now = self.clock()
        dt = (now - self._last_tick) if self._last_tick is not None else 1.0 / self.net.control_hz
        dt = min(max(dt, 1e-3), 0.5)
        self._last_tick = now
        world = self.world_model.snapshot()

        if now - self._last_ping > 1.0:
            self._last_ping = now
            self.fleet.ping_all()

        with self.lock:
            cmds: dict[int, RobotCommand] = {}
            mode = self.mode
            if self.estopped:
                if now - self._last_estop_tx > 0.5:      # 新しく繋がったロボットにも届くよう周期送信
                    self._last_estop_tx = now
                    self.fleet.send_all(m.EStop())
            else:
                if mode is not None:
                    cmds.update(mode.step(world, dt))
                if self.override is not None and self.override.active:
                    cmds.update(self.override.step(world, dt))
                for rid, (vx, wz, t) in list(self._manual.items()):
                    if now - t < MANUAL_TIMEOUT_S:
                        cmds[rid] = RobotCommand(vx=vx, wz=wz)
                    else:
                        del self._manual[rid]
                        cmds.setdefault(rid, RobotCommand())    # 離した瞬間に停止指令
                self._apply_safety(world, cmds)
                self._send(cmds)
            self.snapshot = CoreSnapshot(
                world=world, commands=cmds, mode_name=mode.name if mode else None,
                mode_status=mode.status if mode else "", viz=mode.viz if mode else Visualization(),
                estopped=self.estopped, loop_hz=1.0 / dt)
        for fn in list(self.tick_listeners):
            fn(self.snapshot)
        return self.snapshot

    def _apply_safety(self, world: World, cmds: dict[int, RobotCommand]) -> None:
        if not self.safety_enabled:
            return
        for rid, c in cmds.items():
            r = world.robots.get(rid)
            if r is not None and c.safety and c.wheel is None:
                c.vx = ultrasonic_speed_limit(c.vx, r.us_front, r.us_rear)

    def _send(self, cmds: dict[int, RobotCommand]) -> None:
        now = self.clock()
        for rid, c in cmds.items():
            if c.wheel is not None:
                self.fleet.send(rid, m.CmdWheel(left=c.wheel[0], right=c.wheel[1]))
            else:
                self.fleet.send(rid, m.CmdVel(vx=c.vx, wz=c.wz))
            if c.led is not None:
                last = self._last_led.get(rid)
                if last is None or last[0] != c.led or now - last[1] > LED_REFRESH_S:
                    r, g, b, pattern = c.led
                    self.fleet.send(rid, m.SetLed(r=r, g=g, b=b, pattern=pattern, period_ms=800))
                    self._last_led[rid] = (c.led, now)
            if c.buzzer is not None:
                if isinstance(c.buzzer, str):
                    self.fleet.send(rid, m.Buzzer(melody=c.buzzer))
                else:
                    self.fleet.send(rid, m.Buzzer(freq_hz=float(c.buzzer), dur_ms=150))

    def _emit(self, kind: str, data: dict[str, Any]) -> None:
        for fn in list(self.event_listeners):
            fn(kind, data)


def _params_dict(params: Any) -> dict[str, Any]:
    from dataclasses import asdict, is_dataclass
    return asdict(params) if is_dataclass(params) else {}
