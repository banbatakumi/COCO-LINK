"""入力 → 処理 → 送信 を回すスレッド（Qt 非依存）."""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

from .pipeline import VisionPipeline, VisionResult
from .publisher import VisionPublisher

log = logging.getLogger(__name__)


class VisionWorker:
    def __init__(self, source, pipeline: VisionPipeline, publisher: VisionPublisher | None):
        self.source = source
        self.pipeline = pipeline
        self.publisher = publisher
        self.sending = True
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.result: VisionResult | None = None
        self.fps = 0.0
        self.sent = 0
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> VisionWorker:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="vision", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.source.release()
        if self.publisher:
            self.publisher.close()

    def latest(self) -> tuple[np.ndarray | None, VisionResult | None]:
        with self.lock:
            return self.frame, self.result

    def _loop(self) -> None:
        t_prev = time.monotonic()
        while self._running:
            ok, frame, t_cap = self.source.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            try:
                res = self.pipeline.process(frame, t_cap)
            except Exception:
                log.exception("vision processing failed")
                continue
            if self.sending and self.publisher is not None and res.calibrated:
                self.publisher.send(res.world)
                self.sent += 1
            with self.lock:
                self.frame, self.result = frame, res
            now = time.monotonic()
            self.fps = 0.9 * self.fps + 0.1 / max(now - t_prev, 1e-3)
            t_prev = now
