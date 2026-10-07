"""シナリオ（シミュレータの初期配置）の保存・読込."""

from __future__ import annotations

from pathlib import Path

from ..common.config import load_yaml, save_yaml
from ..common.geometry import Pose2D
from .engine import SimEngine


def load_scenario(engine: SimEngine, path: str | Path) -> None:
    data = load_yaml(path)
    with engine.lock:
        engine.clear()
        f = data.get("field", {})
        engine.config.field_w = float(f.get("w", engine.config.field_w))
        engine.config.field_h = float(f.get("h", engine.config.field_h))
        for r in data.get("robots", []):
            engine.add_robot(int(r["id"]), Pose2D(float(r["x"]), float(r["y"]), float(r.get("th", 0.0))))
        for kind, key in (("person", "persons"), ("obstacle", "obstacles"), ("cargo", "cargo")):
            for b in data.get(key, []):
                engine.add_body(kind, float(b["x"]), float(b["y"]), b.get("r"))


def save_scenario(engine: SimEngine, path: str | Path) -> None:
    with engine.lock:
        data = {
            "field": {"w": engine.config.field_w, "h": engine.config.field_h},
            "robots": [{"id": r.robot_id, "x": round(r.pose.x, 3), "y": round(r.pose.y, 3), "th": round(r.pose.th, 3)}
                       for r in engine.robots.values()],
        }
        for kind, key in (("person", "persons"), ("obstacle", "obstacles"), ("cargo", "cargo")):
            data[key] = [{"x": round(b.x, 3), "y": round(b.y, 3), "r": b.r} for b in engine.bodies if b.kind == kind]
    save_yaml(path, data)
