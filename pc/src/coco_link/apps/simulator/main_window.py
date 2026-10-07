"""シミュレータ GUI.

- ツールバーでロボット・人・障害物・物資を配置/削除
- ドラッグで移動（動作中の外乱）、ロボット上でホイール回転すると向きを変える
- 矢印キー / WASD で人を動かす（隊列追従の確認用）
"""

from __future__ import annotations

import contextlib
import math
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QActionGroup, QColor, QFont, QKeyEvent, QMouseEvent, QPainter, QWheelEvent
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from ...common.geometry import Pose2D, wrap_angle
from ...protocol import messages as m
from ...sim.engine import SimEngine
from ...sim.entities import CircleBody, SimRobot
from ...sim.runner import SimRunner
from ...sim.scenario import load_scenario, save_scenario
from ..common.field_canvas import COLORS, FieldCanvas

STATE_COLORS = {m.STATE_ESTOP: QColor(220, 0, 0), m.STATE_LOW_BATT: QColor(255, 140, 0),
                m.STATE_FAULT: QColor(160, 0, 255), m.STATE_RUNNING: QColor(0, 120, 220)}


class SimCanvas(FieldCanvas):
    def __init__(self, engine: SimEngine, parent=None):
        super().__init__(engine.config.field_w, engine.config.field_h, parent)
        self.engine = engine
        self.tool = "move"
        self._drag: SimRobot | CircleBody | None = None
        self._drag_offset = (0.0, 0.0)
        self.show_sensors = True
        self._keys: set[int] = set()

    # ------------------------------------------------------------ 描画
    def draw_world(self, p: QPainter) -> None:
        e = self.engine
        with e.lock:
            self.field_w, self.field_h = e.field_wh
            for b in e.bodies:
                self.draw_circle(p, b.x, b.y, b.r, QColor(COLORS[b.kind]).lighter(115))
            for r in e.robots.values():
                pose, prm = r.pose, r.params
                if self.show_sensors:
                    tel = r.firmware.last_sensors
                    for dist, facing in ((tel.us_front_m, 0.0), (tel.us_rear_m, math.pi)):
                        th = pose.th + facing
                        sx = pose.x + prm.us_offset * math.cos(th)
                        sy = pose.y + prm.us_offset * math.sin(th)
                        if dist is not None:   # エコーがある時だけ、測距距離までの扇形を描く
                            self.draw_cone(p, sx, sy, th, math.radians(prm.us_fov_deg), dist,
                                           QColor(255, 160, 0, 28))
                led = QColor(*r.firmware.led_color())
                state = r.firmware.state
                self.draw_robot(p, pose.x, pose.y, pose.th, prm.body_radius, led, f"#{r.robot_id}",
                                outline=STATE_COLORS.get(state, QColor(40, 40, 40)))
                if r.firmware.buzzer_freq() > 0:
                    c = self.to_screen(pose.x + prm.body_radius, pose.y + prm.body_radius)
                    p.setPen(QColor(0, 0, 0))
                    p.setFont(QFont("", 14))
                    p.drawText(c, "♪")
                if state not in (m.STATE_IDLE, m.STATE_RUNNING):
                    c = self.to_screen(pose.x - prm.body_radius, pose.y + prm.body_radius + 0.02)
                    p.setPen(STATE_COLORS.get(state, QColor(0, 0, 0)))
                    p.setFont(QFont("", 8))
                    p.drawText(c, state)

    # ------------------------------------------------------------ マウス
    def mousePressEvent(self, e: QMouseEvent) -> None:
        x, y = self.to_world(e.position())
        eng = self.engine
        if e.button() == Qt.MouseButton.RightButton or self.tool == "delete":
            obj = eng.pick(x, y)
            if isinstance(obj, SimRobot):
                eng.remove_robot(obj.robot_id)
            elif isinstance(obj, CircleBody):
                eng.remove_body(obj.uid)
            return
        if self.tool in ("person", "obstacle", "cargo"):
            eng.add_body(self.tool, x, y)
            return
        if self.tool == "robot":
            with contextlib.suppress(StopIteration, ValueError):  # 30 台を超えたら何もしない
                eng.add_robot(pose=Pose2D(x, y, math.pi / 2))
            return
        obj = eng.pick(x, y)
        if obj is not None:
            with eng.lock:
                obj.dragging = True
                ox, oy = (obj.pose.x, obj.pose.y) if isinstance(obj, SimRobot) else (obj.x, obj.y)
            self._drag = obj
            self._drag_offset = (ox - x, oy - y)

    def mouseMoveEvent(self, e: QMouseEvent) -> None:
        super().mouseMoveEvent(e)
        if self._drag is None:
            return
        x, y = self.to_world(e.position())
        x, y = x + self._drag_offset[0], y + self._drag_offset[1]
        with self.engine.lock:
            if isinstance(self._drag, SimRobot):
                self._drag.pose = Pose2D(x, y, self._drag.pose.th)
            else:
                self._drag.x, self._drag.y = x, y

    def mouseReleaseEvent(self, _e: QMouseEvent) -> None:
        if self._drag is not None:
            with self.engine.lock:
                self._drag.dragging = False
            self._drag = None

    def wheelEvent(self, e: QWheelEvent) -> None:
        x, y = self.to_world(e.position())
        obj = self.engine.pick(x, y)
        if isinstance(obj, SimRobot):
            d = math.radians(10) * (1 if e.angleDelta().y() > 0 else -1)
            with self.engine.lock:
                obj.pose = Pose2D(obj.pose.x, obj.pose.y, wrap_angle(obj.pose.th + d))

    # ------------------------------------------------------------ キーボードで人を動かす
    def keyPressEvent(self, e: QKeyEvent) -> None:
        self._keys.add(e.key())
        self._update_person_velocity()

    def keyReleaseEvent(self, e: QKeyEvent) -> None:
        if not e.isAutoRepeat():
            self._keys.discard(e.key())
        self._update_person_velocity()

    def _update_person_velocity(self) -> None:
        K = Qt.Key
        vx = (K.Key_Right in self._keys or K.Key_D in self._keys) - (K.Key_Left in self._keys or K.Key_A in self._keys)
        vy = (K.Key_Up in self._keys or K.Key_W in self._keys) - (K.Key_Down in self._keys or K.Key_S in self._keys)
        n = math.hypot(vx, vy) or 1.0
        speed = self.engine.config.person_speed
        with self.engine.lock:
            for person in self.engine.persons()[:1]:   # 最初に置いた人を操作
                person.vx, person.vy = speed * vx / n, speed * vy / n


class SimulatorWindow(QMainWindow):
    def __init__(self, engine: SimEngine, runner: SimRunner, scenario: Path | None = None):
        super().__init__()
        self.engine = engine
        self.runner = runner
        self.scenario_path = scenario
        self.setWindowTitle("COCO-LINK Simulator")
        self.resize(1200, 720)

        self.canvas = SimCanvas(engine)
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["ID", "状態", "電圧[V]", "duty L/R", "前[m]", "後[m]", "operator"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.truth_label = QLabel()
        self.truth_label.setWordWrap(True)
        self.truth_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        help_label = QLabel(
            "<b>操作</b><br>左ドラッグ: 移動（外乱）<br>ホイール: ロボットの向き<br>右クリック: 削除<br>"
            "矢印/WASD: 人を移動<br>ツールバーで配置する物を選択")
        help_label.setWordWrap(True)

        side = QWidget()
        sl = QVBoxLayout(side)
        sl.addWidget(QLabel("<b>仮想ロボット</b>"))
        sl.addWidget(self.table, 2)
        sl.addWidget(QLabel("<b>真のパラメータ（個体差, 同定の答え）</b>"))
        sl.addWidget(self.truth_label, 1)
        sl.addWidget(help_label)
        splitter = QSplitter()
        splitter.addWidget(self.canvas)
        splitter.addWidget(side)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        central = QWidget()
        QHBoxLayout(central).addWidget(splitter)
        self.setCentralWidget(central)
        self._build_toolbar()
        self.status = QLabel()
        self.statusBar().addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)
        self._table_tick = 0

    def _build_toolbar(self) -> None:
        tb = QToolBar("tools")
        self.addToolBar(tb)
        group = QActionGroup(self)
        for key, label in (("move", "✋ 移動"), ("robot", "🤖 ロボット"), ("person", "🟢 人"),
                           ("obstacle", "🔴 障害物"), ("cargo", "🔵 物資"), ("delete", "🗑 削除")):
            act = QAction(label, self, checkable=True)
            act.setData(key)
            act.triggered.connect(lambda _=False, k=key: setattr(self.canvas, "tool", k))
            group.addAction(act)
            tb.addAction(act)
            if key == "move":
                act.setChecked(True)
        tb.addSeparator()
        self.pause_act = QAction("⏸ 一時停止", self, checkable=True)
        self.pause_act.toggled.connect(lambda v: setattr(self.runner, "paused", v))
        tb.addAction(self.pause_act)
        tb.addWidget(QLabel(" 速度×"))
        speed = QDoubleSpinBox()
        speed.setRange(0.1, 5.0)
        speed.setSingleStep(0.5)
        speed.setValue(self.runner.speed)
        speed.valueChanged.connect(lambda v: setattr(self.runner, "speed", v))
        tb.addWidget(speed)
        sensors = QAction("超音波表示", self, checkable=True, checked=True)
        sensors.toggled.connect(lambda v: setattr(self.canvas, "show_sensors", v))
        tb.addAction(sensors)
        tb.addSeparator()
        for label, fn in (("📂 開く", self._open), ("💾 保存", self._save), ("↺ リセット", self._reset)):
            act = QAction(label, self)
            act.triggered.connect(fn)
            tb.addAction(act)

    def _open(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "シナリオを開く", "scenarios", "YAML (*.yaml *.yml)")
        if path:
            self.scenario_path = Path(path)
            load_scenario(self.engine, path)

    def _save(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "シナリオを保存", "scenarios/new.yaml", "YAML (*.yaml)")
        if path:
            save_scenario(self.engine, path)

    def _reset(self) -> None:
        if self.scenario_path:
            load_scenario(self.engine, self.scenario_path)

    def _refresh(self) -> None:
        self.canvas.update()
        self._table_tick += 1
        if self._table_tick % 5:
            return
        e = self.engine
        with e.lock:
            rows = []
            for r in sorted(e.robots.values(), key=lambda r: r.robot_id):
                fw = r.firmware
                s = fw.last_sensors
                node = self.runner.nodes.get(r.robot_id)
                op = f"{node.operator[0]}:{node.operator[1]}" if node and node.operator else "-"
                rows.append((str(r.robot_id), fw.state, f"{fw.batt_v:.2f}",
                             f"{fw.applied[0]:+.2f}/{fw.applied[1]:+.2f}",
                             "-" if s.us_front_m is None else f"{s.us_front_m:.2f}",
                             "-" if s.us_rear_m is None else f"{s.us_rear_m:.2f}", op))
            truth = []
            for r in sorted(e.robots.values(), key=lambda r: r.robot_id)[:6]:
                q = r.params
                truth.append(f"#{r.robot_id}: r={q.wheel_radius * 1000:.1f}mm b={q.tread * 1000:.1f}mm "
                             f"K={q.motor_gain_l:.1f}/{q.motor_gain_r:.1f} τ={q.motor_tau_l * 1000:.0f}/"
                             f"{q.motor_tau_r * 1000:.0f}ms u0={q.deadzone_l:.3f}/{q.deadzone_r:.3f} "
                             f"bias={q.gyro_bias_z:+.4f}")
            n_robots, t = len(e.robots), e.t
        self.table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for j, v in enumerate(row):
                item = QTableWidgetItem(v)
                if j == 1 and v in STATE_COLORS:
                    item.setForeground(STATE_COLORS[v])
                self.table.setItem(i, j, item)
        self.truth_label.setText("\n".join(truth))
        net = self.runner.net
        self.status.setText(f"t = {t:7.1f} s   実時間比 {self.runner.real_time_factor:.2f}   ロボット {n_robots} 台   "
                            f"→ operator {net.operator_host}:{net.operator_robot_port} / vision :{net.operator_vision_port}")

    def closeEvent(self, e) -> None:
        self.timer.stop()
        self.runner.stop()
        super().closeEvent(e)
