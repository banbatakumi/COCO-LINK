"""Qt アプリケーション起動の共通処理."""

from __future__ import annotations

import logging
import signal
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication


def create_app(name: str) -> QApplication:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName(name)
    app.setStyle("Fusion")   # Mac/Windows で見た目を揃える
    # Ctrl+C で終了できるようにする（Qt のイベントループ中も Python のシグナル処理を回す）
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer(app)
    timer.timeout.connect(lambda: None)
    timer.start(200)
    app._sigint_timer = timer  # type: ignore[attr-defined]
    return app


def run_for(app: QApplication, seconds: float | None, screenshot=None) -> int:
    """seconds 指定時はその秒数後に（必要ならスクリーンショットを撮って）終了する. スモークテスト用."""
    if seconds is not None:
        def finish():
            if screenshot is not None:
                screenshot()
            app.quit()
        QTimer.singleShot(int(seconds * 1000), finish)
    return app.exec()
