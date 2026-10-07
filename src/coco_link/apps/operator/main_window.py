"""操作GUI メインウィンドウ."""

from __future__ import annotations

from PySide6.QtCore import QEvent, QObject, Qt, QTimer
from PySide6.QtGui import QAction, QKeyEvent
from PySide6.QtWidgets import QLabel, QMainWindow, QPushButton, QSplitter, QTabWidget, QToolBar, QWidget

from ...fleet.control_core import ControlCore
from .field_view import OperatorFieldView
from .mode_panel import ModePanel
from .robot_table import RobotTable
from .teleop_panel import TeleopPanel


class OperatorWindow(QMainWindow):
    def __init__(self, core: ControlCore):
        super().__init__()
        self.core = core
        self.logger = None
        self.setWindowTitle("COCO-LINK Operator")
        self.resize(1400, 860)

        self.field = OperatorFieldView()
        self.table = RobotTable(core.fleet)
        self.mode_panel = ModePanel(core)
        self.teleop = TeleopPanel(core)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.mode_panel, "モード")
        self.tabs.addTab(self.teleop, "手動操作")
        self.tabs.setMinimumWidth(380)

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self.field)
        top.addWidget(self.tabs)
        top.setStretchFactor(0, 3)
        top.setStretchFactor(1, 1)
        main = QSplitter(Qt.Orientation.Vertical)
        main.addWidget(top)
        main.addWidget(self.table)
        main.setStretchFactor(0, 3)
        main.setStretchFactor(1, 1)
        self.setCentralWidget(main)
        self._build_toolbar()
        self.status = QLabel()
        self.statusBar().addWidget(self.status)

        self.field.robot_selected.connect(self._select_robot)
        self.table.robot_selected.connect(self._select_robot)
        self.field.field_clicked.connect(core.field_click)
        # キー入力はウィンドウ全体で拾う（入力欄にフォーカスがある時は除く）
        self._key_filter = _KeyFilter(self)
        self.field.installEventFilter(self._key_filter)
        self.table.installEventFilter(self._key_filter)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)
        self._tick = 0

    def _build_toolbar(self) -> None:
        tb = QToolBar("main")
        tb.setMovable(False)
        self.addToolBar(tb)
        estop = QPushButton("■ 全台 E-STOP (Esc)")
        estop.setStyleSheet("background:#d00; color:white; font-weight:bold; padding:6px 16px; font-size:14px;")
        estop.clicked.connect(self.core.estop_all)
        tb.addWidget(estop)
        clear = QPushButton("E-STOP 解除")
        clear.setStyleSheet("padding:6px 10px;")
        clear.clicked.connect(self.core.clear_estop_all)
        tb.addWidget(clear)
        tb.addSeparator()
        safety = QAction("超音波セーフティ", self, checkable=True, checked=True)
        safety.toggled.connect(lambda v: setattr(self.core, "safety_enabled", v))
        tb.addAction(safety)
        trails = QAction("軌跡表示", self, checkable=True, checked=True)
        trails.toggled.connect(lambda v: setattr(self.field, "show_trails", v))
        tb.addAction(trails)
        self.toolbar = tb

    def attach_logger(self, logger) -> None:
        """MCAP ロガーと記録ボタンを付ける."""
        self.logger = logger
        act = QAction("● MCAP 記録", self, checkable=True)
        act.toggled.connect(self._toggle_record)
        self.toolbar.addSeparator()
        self.toolbar.addAction(act)
        self.record_action = act

    def _toggle_record(self, on: bool) -> None:
        if on:
            path = self.logger.start()
            self.statusBar().showMessage(f"記録開始: {path}", 5000)
        else:
            path = self.logger.stop()
            if path:
                self.statusBar().showMessage(f"保存しました: {path}", 8000)

    def add_tab(self, widget: QWidget, title: str) -> None:
        self.tabs.addTab(widget, title)

    def _select_robot(self, rid: int) -> None:
        self.field.selected = rid
        self.table.select_robot(rid)
        self.teleop.set_robot(rid)
        for i in range(self.tabs.count()):
            w = self.tabs.widget(i)
            if hasattr(w, "set_robot") and w is not self.teleop:
                w.set_robot(rid)

    def _refresh(self) -> None:
        snap = self.core.snapshot
        self.field.set_snapshot(snap)
        self._tick += 1
        if self._tick % 3 == 0:
            self.table.refresh(snap)
            self.mode_panel.refresh()
            n_conn = len(self.core.fleet.connected_ids())
            va = snap.world.vision_age
            vision = f"{va * 1000:.0f} ms 前" if va < 10 else "未受信"
            self.status.setText(
                f"制御 {snap.loop_hz:4.1f} Hz   接続 {n_conn} 台   ビジョン {vision}   "
                f"受信ポート robot:{self.core.fleet.endpoint.port} vision:{self.core.world_model.endpoint.port}"
                + ("   ⚠ E-STOP 中" if snap.estopped else "")
                + (f"   ● 記録中 {self.logger.count} msgs" if getattr(self, "logger", None) and self.logger.recording
                   else ""))

    def closeEvent(self, e) -> None:
        self.timer.stop()
        super().closeEvent(e)


class _KeyFilter(QObject):
    def __init__(self, win: OperatorWindow):
        super().__init__(win)
        self.win = win

    def eventFilter(self, obj, event) -> bool:
        if event.type() == QEvent.Type.KeyPress and isinstance(event, QKeyEvent):
            if not event.isAutoRepeat():
                return self.win.teleop.key_pressed(event.key())
            return True
        if event.type() == QEvent.Type.KeyRelease and isinstance(event, QKeyEvent):
            if not event.isAutoRepeat():
                self.win.teleop.key_released(event.key())
            return True
        return False
