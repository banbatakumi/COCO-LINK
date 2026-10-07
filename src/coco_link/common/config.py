"""設定ファイル (config/*.yaml) の読み込み."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml


def project_root() -> Path:
    """リポジトリのルート（config/ がある場所）を返す.

    開発時は src/coco_link/common/config.py から3つ上。見つからなければカレントディレクトリ。
    """
    here = Path(__file__).resolve()
    for p in [*here.parents, Path.cwd()]:
        if (p / "config").is_dir() and (p / "pyproject.toml").exists():
            return p
    return Path.cwd()


def config_dir() -> Path:
    return project_root() / "config"


def load_yaml(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def save_yaml(path: str | Path, data: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def dataclass_from_dict(cls, data: dict[str, Any]):
    """dict から dataclass を作る（未知キーは無視、欠けたキーは既定値）."""
    names = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in data.items() if k in names})


@dataclass
class NetworkConfig:
    """ポート・ホスト設定. docs/protocol.md §2."""

    operator_host: str = "127.0.0.1"   # 仮想ロボット/ビジョンが送る先
    operator_robot_port: int = 50000
    operator_vision_port: int = 50001
    robot_port: int = 50100             # 実機の受信ポート。仮想ロボットは robot_port + id
    bind_host: str = "0.0.0.0"
    control_hz: float = 20.0
    disconnect_timeout_s: float = 2.0

    @classmethod
    def load(cls, path: str | Path | None = None) -> NetworkConfig:
        return dataclass_from_dict(cls, load_yaml(path or config_dir() / "network.yaml"))


@dataclass
class ColorRange:
    """HSV 閾値（OpenCV: H 0-179, S/V 0-255）。赤のように H が wrap する場合は h_min > h_max とする."""

    h_min: int = 0
    h_max: int = 179
    s_min: int = 80
    s_max: int = 255
    v_min: int = 60
    v_max: int = 255


@dataclass
class FieldConfig:
    """フィールド寸法とビジョン設定."""

    width: float = 3.0
    height: float = 2.0
    corner_marker_ids: list[int] = field(default_factory=lambda: [40, 41, 42, 43])
    corner_marker_size: float = 0.10
    robot_marker_size: float = 0.06
    aruco_dict: str = "DICT_4X4_50"
    min_blob_area_m2: float = 0.002
    colors: dict[str, ColorRange] = field(default_factory=dict)

    @classmethod
    def load(cls, path: str | Path | None = None) -> FieldConfig:
        data = load_yaml(path or config_dir() / "field.yaml")
        cfg = dataclass_from_dict(cls, data)
        colors = dict(data.get("colors") or {})
        if path is None:   # ビジョン GUI で調整・保存した閾値があれば優先
            colors.update(load_yaml(config_dir() / "vision_colors.yaml"))
        cfg.colors = {k: dataclass_from_dict(ColorRange, v) for k, v in colors.items()}
        return cfg

    def save_colors(self, path: str | Path | None = None) -> Path:
        path = Path(path or config_dir() / "vision_colors.yaml")
        save_yaml(path, {k: asdict(v) for k, v in self.colors.items()})
        return path
