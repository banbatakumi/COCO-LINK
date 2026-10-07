"""メッセージ ⇄ バイト列 の変換（JSON）.

エンコード方式を変えたい（MsgPack 等）ときはこのファイルだけを差し替える。
"""

from __future__ import annotations

import dataclasses
import json
import math
import types
import typing
from dataclasses import dataclass
from functools import cache
from typing import Any

from ..common.clock import monotonic_ms
from .messages import MESSAGE_TYPES, PROTOCOL_VERSION, Message

MAX_DATAGRAM = 1400  # byte (docs/protocol.md §1)


class ProtocolError(ValueError):
    """不正なメッセージ."""


@dataclass
class Envelope:
    """受信したメッセージとエンベロープ情報."""

    msg: Message
    src: str
    seq: int
    t_ms: int

    @property
    def robot_id(self) -> int | None:
        """src が "robot:<id>" なら id."""
        if self.src.startswith("robot:"):
            try:
                return int(self.src[6:])
            except ValueError:
                return None
        return None


def _to_jsonable(obj: Any) -> Any:
    if dataclasses.is_dataclass(obj):
        return {f.name: _to_jsonable(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, list | tuple):
        return [_to_jsonable(v) for v in obj]
    if isinstance(obj, dict):
        return {str(k): _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, float):
        if not math.isfinite(obj):
            return None
        return round(obj, 6)
    return obj


def encode(msg: Message, src: str, seq: int = 0, t_ms: int | None = None) -> bytes:
    body = _to_jsonable(msg)
    data = {"v": PROTOCOL_VERSION, "type": msg.TYPE, "src": src, "seq": seq & 0xFFFFFFFF,
            "t_ms": (monotonic_ms() if t_ms is None else t_ms) & 0xFFFFFFFF}
    # Message 自身の None (melody 等の省略可能フィールド) は送らない
    data.update({k: v for k, v in body.items() if v is not None or k.startswith("us_")})
    raw = json.dumps(data, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(raw) > MAX_DATAGRAM:
        raise ProtocolError(f"message too large: {len(raw)} bytes ({msg.TYPE})")
    return raw


@cache
def _hints(cls) -> dict[str, Any]:
    return typing.get_type_hints(cls)


def _strip_optional(tp):
    origin = typing.get_origin(tp)
    if origin in (typing.Union, types.UnionType):
        args = [a for a in typing.get_args(tp) if a is not type(None)]
        if len(args) == 1:
            return args[0]
    return tp


def _from_jsonable(tp, value):
    if value is None:
        return None
    tp = _strip_optional(tp)
    if dataclasses.is_dataclass(tp):
        if not isinstance(value, dict):
            raise ProtocolError(f"expected object for {tp.__name__}")
        hints = _hints(tp)
        kwargs = {}
        for f in dataclasses.fields(tp):
            if f.name in value:
                kwargs[f.name] = _from_jsonable(hints[f.name], value[f.name])
        return tp(**kwargs)
    origin = typing.get_origin(tp)
    if origin is list:
        (item_tp,) = typing.get_args(tp) or (Any,)
        if not isinstance(value, list):
            raise ProtocolError("expected array")
        return [_from_jsonable(item_tp, v) for v in value]
    if origin is dict:
        if not isinstance(value, dict):
            raise ProtocolError("expected object")
        return dict(value)
    if tp is float:
        if not isinstance(value, int | float) or isinstance(value, bool):
            raise ProtocolError(f"expected number, got {value!r}")
        return float(value)
    if tp is int:
        if not isinstance(value, int | float) or isinstance(value, bool):
            raise ProtocolError(f"expected integer, got {value!r}")
        return int(value)
    if tp is str:
        return str(value)
    return value


def decode(raw: bytes) -> Envelope:
    """バイト列を解析する. 不正なら ProtocolError、未知 type も ProtocolError."""
    try:
        data = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise ProtocolError(f"invalid json: {e}") from e
    if not isinstance(data, dict):
        raise ProtocolError("not an object")
    if data.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported version: {data.get('v')}")
    cls = MESSAGE_TYPES.get(data.get("type", ""))
    if cls is None:
        raise ProtocolError(f"unknown type: {data.get('type')}")
    msg = _from_jsonable(cls, {k: v for k, v in data.items() if k not in ("v", "type", "src", "seq", "t_ms")})
    return Envelope(msg=msg, src=str(data.get("src", "")), seq=int(data.get("seq", 0)), t_ms=int(data.get("t_ms", 0)))


class Sender:
    """送信者ごとの seq を管理してエンコードする."""

    def __init__(self, src: str):
        self.src = src
        self._seq = 0

    def encode(self, msg: Message, t_ms: int | None = None) -> bytes:
        raw = encode(msg, self.src, self._seq, t_ms)
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return raw
