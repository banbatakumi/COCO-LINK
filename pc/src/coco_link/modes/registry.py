"""モードの登録と自動探索."""

from __future__ import annotations

import importlib
import pkgutil

from .base import Mode

MODES: dict[str, type[Mode]] = {}


def register_mode(cls: type[Mode]) -> type[Mode]:
    """クラスデコレータ: モードを登録する."""
    if cls.name in MODES and MODES[cls.name] is not cls:
        raise ValueError(f"duplicate mode name: {cls.name}")
    MODES[cls.name] = cls
    return cls


def discover() -> dict[str, type[Mode]]:
    """coco_link.modes パッケージ内の全モジュールを import して登録させる."""
    from .. import modes as pkg

    for info in pkgutil.iter_modules(pkg.__path__):
        if info.name not in ("base", "registry"):
            importlib.import_module(f"{pkg.__name__}.{info.name}")
    return dict(MODES)
