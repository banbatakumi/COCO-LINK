"""メッセージ定義（docs/protocol.md §5 と 1対1 対応）.

★ここを変更したら docs/protocol.md, docs/firmware_spec.md, tests/test_protocol.py も更新すること。

各メッセージは dataclass で、クラス変数 TYPE がワイヤ上の "type" になる。
エンベロープ (v, type, src, seq, t_ms) は codec.py が付与する。
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field as dc_field  # "field" はメッセージのフィールド名として使うため別名
from typing import ClassVar

PROTOCOL_VERSION = 1

# 状態 (docs/firmware_spec.md §4)
STATE_BOOT = "BOOT"
STATE_IDLE = "IDLE"
STATE_RUNNING = "RUNNING"
STATE_ESTOP = "ESTOP"
STATE_LOW_BATT = "LOW_BATT"
STATE_FAULT = "FAULT"

LED_PATTERNS = ("solid", "blink", "breath", "rainbow")
MELODIES = ("startup", "ok", "error", "chime")


class Message:
    """全メッセージの基底."""

    TYPE: ClassVar[str] = ""


MESSAGE_TYPES: dict[str, type[Message]] = {}


def _register(cls):
    MESSAGE_TYPES[cls.TYPE] = cls
    return cls


# ---------------------------------------------------------------- operator -> robot
@_register
@dataclass
class CmdVel(Message):
    """機体速度指令（ロボット内で車輪速度 PI 閉ループ）."""

    TYPE: ClassVar[str] = "cmd_vel"
    vx: float = 0.0  # [m/s] 前進正
    wz: float = 0.0  # [rad/s] 反時計回り正


@_register
@dataclass
class CmdWheel(Message):
    """車輪 duty 直接指令（開ループ、システム同定用）."""

    TYPE: ClassVar[str] = "cmd_wheel"
    left: float = 0.0   # [-1, 1]
    right: float = 0.0  # [-1, 1]


@_register
@dataclass
class Stop(Message):
    TYPE: ClassVar[str] = "stop"


@_register
@dataclass
class EStop(Message):
    TYPE: ClassVar[str] = "estop"


@_register
@dataclass
class ClearEStop(Message):
    TYPE: ClassVar[str] = "clear_estop"


@_register
@dataclass
class SetLed(Message):
    TYPE: ClassVar[str] = "set_led"
    r: int = 0
    g: int = 0
    b: int = 0
    pattern: str = "solid"
    period_ms: int = 1000


@_register
@dataclass
class Buzzer(Message):
    """freq_hz/dur_ms か melody のどちらかを指定する."""

    TYPE: ClassVar[str] = "buzzer"
    freq_hz: float = 0.0
    dur_ms: int = 0
    melody: str | None = None


@_register
@dataclass
class SetParam(Message):
    TYPE: ClassVar[str] = "set_param"
    key: str = ""
    value: float = 0.0


@_register
@dataclass
class GetParams(Message):
    TYPE: ClassVar[str] = "get_params"


@_register
@dataclass
class Ping(Message):
    TYPE: ClassVar[str] = "ping"
    nonce: int = 0


# ---------------------------------------------------------------- robot -> operator
@_register
@dataclass
class Hello(Message):
    TYPE: ClassVar[str] = "hello"
    robot_id: int = 0
    fw: str = ""
    mac: str = ""
    caps: list[str] = dc_field(default_factory=list)


@dataclass
class Imu:
    ax: float = 0.0  # [m/s^2]
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0  # [rad/s]
    gy: float = 0.0
    gz: float = 0.0


@dataclass
class Encoders:
    left_rad: float = 0.0   # 累積回転角 [rad]
    right_rad: float = 0.0
    wl: float = 0.0         # 角速度 [rad/s]
    wr: float = 0.0


@dataclass
class WheelDuty:
    left: float = 0.0
    right: float = 0.0


@_register
@dataclass
class Telemetry(Message):
    TYPE: ClassVar[str] = "telemetry"
    robot_id: int = 0
    state: str = STATE_BOOT
    faults: list[str] = dc_field(default_factory=list)
    batt_v: float = 0.0
    us_front_m: float | None = None
    us_rear_m: float | None = None
    imu: Imu = dc_field(default_factory=Imu)
    enc: Encoders = dc_field(default_factory=Encoders)
    applied: WheelDuty = dc_field(default_factory=WheelDuty)
    rssi: int = 0
    loop_dt_us: int = 0
    rx_count: int = 0


@_register
@dataclass
class Params(Message):
    TYPE: ClassVar[str] = "params"
    values: dict[str, float] = dc_field(default_factory=dict)


@_register
@dataclass
class Pong(Message):
    TYPE: ClassVar[str] = "pong"
    nonce: int = 0


@_register
@dataclass
class Log(Message):
    TYPE: ClassVar[str] = "log"
    level: str = "info"
    msg: str = ""


@_register
@dataclass
class SimTruth(Message):
    """シミュレータ専用拡張: 仮想ロボットの真の物理パラメータ."""

    TYPE: ClassVar[str] = "sim_truth"
    values: dict[str, float] = dc_field(default_factory=dict)


# ---------------------------------------------------------------- vision -> operator
@dataclass
class DetectedRobot:
    id: int = 0
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    conf: float = 1.0


@dataclass
class DetectedObject:
    x: float = 0.0
    y: float = 0.0
    r: float = 0.0


@dataclass
class FieldSize:
    w: float = 0.0
    h: float = 0.0


@_register
@dataclass
class WorldState(Message):
    TYPE: ClassVar[str] = "world_state"
    frame: int = 0
    stamp_ms: int = 0
    latency_ms: float = 0.0
    field: FieldSize = dc_field(default_factory=FieldSize)
    robots: list[DetectedRobot] = dc_field(default_factory=list)
    persons: list[DetectedObject] = dc_field(default_factory=list)
    obstacles: list[DetectedObject] = dc_field(default_factory=list)
    cargo: list[DetectedObject] = dc_field(default_factory=list)


__all__ = [n for n in dir() if not n.startswith("_") and n not in ("annotations", "dataclass", "dc_field", "ClassVar")]
