"""MCAP ロガー: 通信メッセージと上位制御の状態を 1 ファイルに時系列で記録する.

MCAP (https://mcap.dev) はロボット向けの標準的なログ形式で、Foxglove などのツールでそのまま開いて
グラフ表示・再生できる。ROS がなくても使える。

トピック:
  /robot/<id>/<type>          ロボット → operator（telemetry, hello, params, pong, sim_truth）
  /robot/<id>/cmd/<type>      operator → ロボット（cmd_vel, cmd_wheel, set_led, ...）
  /vision/world_state         ビジョン
  /core/state                 制御周期ごとの推定姿勢・指令・モード
  /core/event                 モード開始/停止・E-STOP などのイベント
エンコーディングは JSON + JSON Schema（スキーマは dataclass の型ヒントから自動生成）。
"""

from __future__ import annotations

import dataclasses
import datetime
import json
import threading
import time
import types
import typing
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap.writer import Writer

from ..common.config import project_root
from ..protocol import messages as m
from ..protocol.codec import _to_jsonable

_JSON_TYPES = {float: "number", int: "integer", str: "string", bool: "boolean"}


def json_schema(tp: Any) -> dict[str, Any]:
    """dataclass / 型ヒントから JSON Schema を作る."""
    origin = typing.get_origin(tp)
    if origin in (typing.Union, types.UnionType):
        args = [a for a in typing.get_args(tp) if a is not type(None)]
        s = json_schema(args[0]) if len(args) == 1 else {}
        return {"anyOf": [s, {"type": "null"}]}
    if dataclasses.is_dataclass(tp):
        hints = typing.get_type_hints(tp)
        return {"type": "object", "title": tp.__name__,
                "properties": {f.name: json_schema(hints[f.name]) for f in dataclasses.fields(tp)}}
    if origin is list:
        (item,) = typing.get_args(tp) or (Any,)
        return {"type": "array", "items": json_schema(item)}
    if origin is dict:
        return {"type": "object"}
    if tp in _JSON_TYPES:
        return {"type": _JSON_TYPES[tp]}
    return {}


class McapLogger:
    def __init__(self, core=None):
        self.core = core
        self.lock = threading.Lock()
        self.path: Path | None = None
        self.count = 0
        self._writer: Writer | None = None
        self._file = None
        self._schemas: dict[str, int] = {}
        self._channels: dict[str, int] = {}
        if core is not None:
            core.fleet.listeners.append(self._on_robot_msg)
            core.world_model.listeners.append(self._on_world)
            core.tick_listeners.append(self._on_tick)
            core.event_listeners.append(self._on_event)

    @property
    def recording(self) -> bool:
        return self._writer is not None

    def start(self, path: str | Path | None = None) -> Path:
        if self.recording:
            self.stop()
        if path is None:
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = project_root() / "logs" / f"coco_{stamp}.mcap"
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self._file = path.open("wb")
            self._writer = Writer(self._file)
            self._writer.start(profile="", library="coco-link")
            self._schemas.clear()
            self._channels.clear()
            self.count = 0
            self.path = path
        return path

    def stop(self) -> Path | None:
        with self.lock:
            if self._writer is None:
                return None
            self._writer.finish()
            self._file.close()
            self._writer = None
            self._file = None
            return self.path

    # ------------------------------------------------------------ 書き込み
    def write(self, topic: str, schema_name: str, schema: dict[str, Any], data: dict[str, Any],
              t: float | None = None) -> None:
        """任意のデータを書く. t はモノトニック時刻ではなく UNIX 時刻 [s]（省略時は現在）."""
        with self.lock:
            if self._writer is None:
                return
            sid = self._schemas.get(schema_name)
            if sid is None:
                sid = self._writer.register_schema(schema_name, "jsonschema", json.dumps(schema).encode())
                self._schemas[schema_name] = sid
            cid = self._channels.get(topic)
            if cid is None:
                cid = self._writer.register_channel(topic, "json", sid)
                self._channels[topic] = cid
            ns = int((time.time() if t is None else t) * 1e9)
            self._writer.add_message(cid, ns, json.dumps(data, ensure_ascii=False).encode(), ns)
            self.count += 1

    def write_message(self, topic: str, msg: m.Message, extra: dict[str, Any] | None = None) -> None:
        data = _to_jsonable(msg)
        if extra:
            data.update(extra)
        cls = type(msg)
        self.write(topic, f"coco_link.{cls.TYPE}", json_schema(cls), data)

    # ------------------------------------------------------------ リスナ
    def _on_robot_msg(self, direction: str, rid: int, msg: m.Message, now: float, t_ms: int | None) -> None:
        if not self.recording:
            return
        topic = f"/robot/{rid}/{msg.TYPE}" if direction == "rx" else f"/robot/{rid}/cmd/{msg.TYPE}"
        self.write_message(topic, msg, {"robot_t_ms": t_ms} if t_ms is not None else None)

    def _on_world(self, ws: m.WorldState, now: float) -> None:
        if self.recording:
            self.write_message("/vision/world_state", ws)

    def _on_tick(self, snap) -> None:
        if not self.recording:
            return
        robots = []
        for rid, r in snap.world.robots.items():
            c = snap.commands.get(rid)
            tgt = snap.viz.targets.get(rid)
            robots.append({
                "id": rid, "connected": r.connected, "state": r.state, "pose_source": r.pose_source,
                "x": r.pose.x if r.pose else None, "y": r.pose.y if r.pose else None,
                "th": r.pose.th if r.pose else None, "vx": r.vx, "wz": r.wz,
                "cmd_vx": c.vx if c else None, "cmd_wz": c.wz if c else None,
                "target_x": tgt.x if tgt else None, "target_y": tgt.y if tgt else None,
                "target_err": (r.pose.distance_to(tgt) if (tgt and r.pose) else None)})
        data = {"mode": snap.mode_name, "status": snap.mode_status, "estopped": snap.estopped,
                "loop_hz": snap.loop_hz, "robots": robots}
        self.write("/core/state", "coco_link.core_state", {"type": "object"}, data)

    def _on_event(self, kind: str, data: dict[str, Any]) -> None:
        if self.recording:
            self.write("/core/event", "coco_link.event", {"type": "object"}, {"event": kind, **_to_jsonable(data)})


def read_mcap(path: str | Path, topics: list[str] | None = None) -> Iterator[tuple[str, float, dict[str, Any]]]:
    """MCAP を読み (topic, UNIX 時刻[s], データ) を順に返す."""
    with Path(path).open("rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            yield channel.topic, message.log_time / 1e9, json.loads(message.data)
