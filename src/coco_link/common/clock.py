"""時刻ユーティリティ."""

from __future__ import annotations

import time

_T0 = time.monotonic()


def monotonic_ms() -> int:
    """プロセス起動からのモノトニック時刻 [ms] を uint32 で返す（ESP32 の millis() 相当）."""
    return int((time.monotonic() - _T0) * 1000.0) & 0xFFFFFFFF


def now() -> float:
    """制御用のモノトニック時刻 [s]."""
    return time.monotonic()
