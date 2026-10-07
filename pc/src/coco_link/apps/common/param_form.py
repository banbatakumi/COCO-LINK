"""dataclass からパラメータ入力フォームを自動生成する.

フィールドの metadata (modes/base.py の param()) を見てウィジェットを選ぶ:
  float → QDoubleSpinBox, int → QSpinBox, bool → QCheckBox, choices 付き → QComboBox, str → QLineEdit
"""

from __future__ import annotations

import dataclasses
import typing

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QLineEdit, QSpinBox, QWidget


class ParamForm(QWidget):
    changed = Signal()

    def __init__(self, params_cls: type, parent=None):
        super().__init__(parent)
        self.params_cls = params_cls
        self.widgets: dict[str, QWidget] = {}
        layout = QFormLayout(self)
        hints = typing.get_type_hints(params_cls)
        defaults = params_cls()
        for f in dataclasses.fields(params_cls):
            meta = f.metadata
            tp = hints[f.name]
            value = getattr(defaults, f.name)
            if "choices" in meta:
                w = QComboBox()
                w.addItems([str(c) for c in meta["choices"]])
                w.setCurrentText(str(value))
                w.currentTextChanged.connect(self.changed)
            elif tp is bool:
                w = QCheckBox()
                w.setChecked(bool(value))
                w.toggled.connect(self.changed)
            elif tp is int:
                w = QSpinBox()
                w.setRange(int(meta.get("min", -10**6)), int(meta.get("max", 10**6)))
                w.setSingleStep(int(meta.get("step", 1)))
                w.setValue(int(value))
                w.valueChanged.connect(self.changed)
            elif tp is float:
                w = QDoubleSpinBox()
                step = float(meta.get("step", 0.01))
                w.setDecimals(max(0, len(f"{step:.6f}".rstrip("0").split(".")[1])) if step < 1 else 1)
                w.setRange(float(meta.get("min", -1e6)), float(meta.get("max", 1e6)))
                w.setSingleStep(step)
                w.setValue(float(value))
                w.setKeyboardTracking(False)
                w.valueChanged.connect(self.changed)
            else:
                w = QLineEdit(str(value))
                w.editingFinished.connect(self.changed)
            w.setToolTip(meta.get("help", ""))
            layout.addRow(meta.get("label", f.name), w)
            self.widgets[f.name] = w

    def values(self):
        kwargs = {}
        hints = typing.get_type_hints(self.params_cls)
        for name, w in self.widgets.items():
            if isinstance(w, QComboBox):
                kwargs[name] = hints[name](w.currentText()) if hints[name] in (int, float) else w.currentText()
            elif isinstance(w, QCheckBox):
                kwargs[name] = w.isChecked()
            elif isinstance(w, QSpinBox | QDoubleSpinBox):
                kwargs[name] = w.value()
            elif isinstance(w, QLineEdit):
                kwargs[name] = w.text()
        return self.params_cls(**kwargs)

    def set_values(self, params) -> None:
        for name, w in self.widgets.items():
            v = getattr(params, name)
            w.blockSignals(True)
            if isinstance(w, QComboBox):
                w.setCurrentText(str(v))
            elif isinstance(w, QCheckBox):
                w.setChecked(bool(v))
            elif isinstance(w, QSpinBox | QDoubleSpinBox):
                w.setValue(v)
            elif isinstance(w, QLineEdit):
                w.setText(str(v))
            w.blockSignals(False)
