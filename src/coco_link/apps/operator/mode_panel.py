"""モード選択・パラメータ・開始/停止."""

from __future__ import annotations

from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget

from ...fleet.control_core import ControlCore
from ..common.param_form import ParamForm


class ModePanel(QWidget):
    def __init__(self, core: ControlCore, parent=None):
        super().__init__(parent)
        self.core = core
        self.combo = QComboBox()
        self.combo.addItems(list(core.modes))
        self.desc = QLabel()
        self.desc.setWordWrap(True)
        self.form_area = QScrollArea()
        self.form_area.setWidgetResizable(True)
        self.form: ParamForm | None = None
        self.start_btn = QPushButton("▶ 開始")
        self.stop_btn = QPushButton("■ 停止")
        self.start_btn.setStyleSheet("font-weight: bold; padding: 6px;")
        self.stop_btn.setStyleSheet("padding: 6px;")
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #1a5fb4; font-weight: bold;")

        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("モード"))
        lay.addWidget(self.combo)
        lay.addWidget(self.desc)
        lay.addWidget(self.form_area, 1)
        btns = QHBoxLayout()
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        lay.addLayout(btns)
        lay.addWidget(self.status)

        self.combo.currentTextChanged.connect(self._on_mode_selected)
        self.start_btn.clicked.connect(self._start)
        self.stop_btn.clicked.connect(core.stop_mode)
        self._on_mode_selected(self.combo.currentText())

    def _on_mode_selected(self, name: str) -> None:
        cls = self.core.modes.get(name)
        if cls is None:
            return
        self.desc.setText(cls.description)
        self.form = ParamForm(cls.Params)
        self.form.changed.connect(self._params_changed)
        self.form_area.setWidget(self.form)

    def _start(self) -> None:
        if self.form is not None:
            self.core.start_mode(self.combo.currentText(), self.form.values())

    def _params_changed(self) -> None:
        mode = self.core.mode
        if mode is not None and mode.name == self.combo.currentText() and self.form is not None:
            self.core.update_mode_params(self.form.values())

    def refresh(self) -> None:
        snap = self.core.snapshot
        running = snap.mode_name is not None
        self.combo.setEnabled(not running)
        self.start_btn.setEnabled(not running and not snap.estopped)
        self.stop_btn.setEnabled(running)
        self.status.setText(f"実行中: {snap.mode_name}\n{snap.mode_status}" if running else "停止中")
        mode = self.core.mode
        if running and mode is not None and self.combo.currentText() != mode.name:
            self.combo.setCurrentText(mode.name)        # 他所から開始されたモードに表示を合わせる
        # フィールドクリックなどでモード側が変えたパラメータをフォームに反映
        if running and mode is not None and self.form is not None and mode.name == self.combo.currentText():
            if not self.form.hasFocus() and not any(w.hasFocus() for w in self.form.widgets.values()):
                self.form.set_values(mode.params)
