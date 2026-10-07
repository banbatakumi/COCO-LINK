"""システム同定パネル: 試験の選択・実行・結果表示・保存・ロボットへの書き込み."""

from __future__ import annotations

import math

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ...robot.params import RobotParams
from ...sysid.experiments import EXPERIMENTS, SysIdContext, SysIdOptions
from ...sysid.runner import SysIdRunner
from ..common.param_form import ParamForm

PEN_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]
UNITS = {"wheel_radius": ("mm", 1000), "tread": ("mm", 1000), "motor_tau_l": ("ms", 1000), "motor_tau_r": ("ms", 1000),
         "gyro_bias_z": ("rad/s", 1), "motor_gain_l": ("rad/s/duty", 1), "motor_gain_r": ("rad/s/duty", 1),
         "deadzone_l": ("duty", 1), "deadzone_r": ("duty", 1)}


class SysIdPanel(QWidget):
    def __init__(self, runner: SysIdRunner, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.robot_id: int | None = None
        self._shown_results = -1

        self.robot_label = QLabel("対象: なし（一覧かフィールドでロボットを選択）")
        self.robot_label.setStyleSheet("font-weight: bold;")
        exp_box = QGroupBox("試験（上から順に実行）")
        ev = QVBoxLayout(exp_box)
        self.checks: dict[str, QCheckBox] = {}
        ctx = SysIdContext(params=RobotParams(), options=SysIdOptions())
        for name, exp in EXPERIMENTS.items():
            cb = QCheckBox(f"{exp.label}（約 {exp.duration(ctx):.0f} s）" + ("  ※ビジョン必須" if exp.needs_vision else ""))
            cb.setChecked(True)
            cb.setToolTip(exp.description)
            ev.addWidget(cb)
            self.checks[name] = cb
        self.options = ParamForm(SysIdOptions)
        opt_box = QGroupBox("試験条件")
        QVBoxLayout(opt_box).addWidget(self.options)
        opt_box.setCheckable(True)
        opt_box.setChecked(False)
        opt_box.toggled.connect(self.options.setVisible)
        self.options.setVisible(False)

        self.start_btn = QPushButton("▶ 同定開始")
        self.start_btn.setStyleSheet("font-weight: bold; padding: 6px;")
        self.abort_btn = QPushButton("■ 中断")
        self.progress = QProgressBar()
        self.phase = QLabel()
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)

        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["パラメータ", "推定値", "真値 (sim)", "誤差"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.gains = QLabel()
        self.gains.setWordWrap(True)
        self.gains.setStyleSheet("font-family: monospace;")
        self.save_btn = QPushButton("💾 保存 (config/robots/robot_XX.yaml)")
        self.apply_btn = QPushButton("⬆ ロボットへ書き込み (set_param)")

        self.plot_select = QComboBox()
        self.plot = pg.PlotWidget(background="w")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend()

        # レイアウト
        run_page = QWidget()
        rv = QVBoxLayout(run_page)
        rv.addWidget(self.robot_label)
        rv.addWidget(exp_box)
        rv.addWidget(opt_box)
        row = QHBoxLayout()
        row.addWidget(self.start_btn)
        row.addWidget(self.abort_btn)
        rv.addLayout(row)
        rv.addWidget(self.progress)
        rv.addWidget(self.phase)
        rv.addWidget(self.log, 1)
        result_page = QWidget()
        res_v = QVBoxLayout(result_page)
        res_split = QSplitter(Qt.Orientation.Vertical)
        top = QWidget()
        tv = QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)
        tv.addWidget(self.table)
        tv.addWidget(QLabel("ファームウェアへの推奨パラメータ（IMC 法）"))
        tv.addWidget(self.gains)
        b = QHBoxLayout()
        b.addWidget(self.save_btn)
        b.addWidget(self.apply_btn)
        tv.addLayout(b)
        bottom = QWidget()
        bv = QVBoxLayout(bottom)
        bv.setContentsMargins(0, 0, 0, 0)
        bv.addWidget(self.plot_select)
        bv.addWidget(self.plot)
        res_split.addWidget(top)
        res_split.addWidget(bottom)
        res_v.addWidget(res_split)
        tabs = QTabWidget()
        tabs.addTab(run_page, "実行")
        tabs.addTab(result_page, "結果")
        self.inner_tabs = tabs
        QVBoxLayout(self).addWidget(tabs)

        self.start_btn.clicked.connect(self._start)
        self.abort_btn.clicked.connect(lambda: self.runner.abort("ユーザが中断しました"))
        self.save_btn.clicked.connect(self._save)
        self.apply_btn.clicked.connect(self._apply)
        self.plot_select.currentIndexChanged.connect(self._draw_plot)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

    def set_robot(self, rid: int) -> None:
        if not self.runner.active:
            self.robot_id = rid
            self.robot_label.setText(f"対象: ロボット #{rid}")

    def _start(self) -> None:
        if self.robot_id is None:
            QMessageBox.information(self, "同定", "ロボットを選択してください。")
            return
        names = [n for n, cb in self.checks.items() if cb.isChecked()]
        if self.runner.core.mode is not None:
            self.runner.core.stop_mode()
        self.runner.start(self.robot_id, names, self.options.values())

    def _save(self) -> None:
        path = self.runner.save()
        if path:
            QMessageBox.information(self, "同定", f"保存しました:\n{path}")

    def _apply(self) -> None:
        gains = self.runner.apply_to_robot()
        if gains:
            QMessageBox.information(self, "同定", "書き込みました:\n" + "\n".join(f"{k} = {v:g}" for k, v in gains.items()))

    def refresh(self) -> None:
        r = self.runner
        self.start_btn.setEnabled(not r.active)
        self.abort_btn.setEnabled(r.active)
        self.progress.setValue(int(r.progress * 100))
        cur = EXPERIMENTS[r.current].label if r.active and r.current else "-"
        self.phase.setText(f"実行中: {cur} / フェーズ: {r.phase}" if r.active else "待機中")
        text = "\n".join(r.log)
        if self.log.toPlainText() != text:
            self.log.setPlainText(text)
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())
        has = bool(r.results)
        self.save_btn.setEnabled(has and not r.active)
        self.apply_btn.setEnabled(has and not r.active)
        if len(r.results) != self._shown_results:
            self._shown_results = len(r.results)
            self._update_results()

    def _update_results(self) -> None:
        r = self.runner
        p = r.identified_params()
        truth = {name: (t, e) for name, _, t, e in r.truth_comparison()}
        names = [k for res in r.results.values() for k in res.params]
        self.table.setRowCount(len(names))
        for i, name in enumerate(names):
            unit, k = UNITS.get(name, ("", 1))
            v = getattr(p, name)
            t, e = truth.get(name, (None, None))
            cells = [f"{name} [{unit}]", f"{v * k:.4g}", "-" if t is None else f"{t * k:.4g}",
                     "-" if e is None or not math.isfinite(e) else f"{e:+.1f} %"]
            for j, c in enumerate(cells):
                self.table.setItem(i, j, QTableWidgetItem(c))
        gains = r.suggested_gains()
        self.gains.setText("  ".join(f"{k}={v:g}" for k, v in gains.items()))
        self.plot_select.blockSignals(True)
        self.plot_select.clear()
        for res in r.results.values():
            for i, pl in enumerate(res.plots):
                self.plot_select.addItem(pl.title, (res.name, i))
        self.plot_select.blockSignals(False)
        if self.plot_select.count():
            self.plot_select.setCurrentIndex(self.plot_select.count() - 1)
            self._draw_plot()

    def _draw_plot(self) -> None:
        data = self.plot_select.currentData()
        self.plot.clear()
        if not data:
            return
        res = self.runner.results.get(data[0])
        if res is None or data[1] >= len(res.plots):
            return
        pl = res.plots[data[1]]
        self.plot.setLabel("bottom", pl.xlabel)
        self.plot.setLabel("left", pl.ylabel)
        self.plot.setTitle(pl.title)
        for i, (name, xs, ys) in enumerate(pl.series):
            color = PEN_COLORS[i % len(PEN_COLORS)]
            if "実測" in name and len(pl.series) > 1:
                self.plot.plot(xs, ys, pen=None, symbol="o", symbolSize=3, symbolBrush=color, symbolPen=None, name=name)
            else:
                self.plot.plot(xs, ys, pen=pg.mkPen(color, width=2), name=name)
