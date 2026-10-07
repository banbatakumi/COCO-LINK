"""動作試験用の手動操作パネル（キーボード走行・LED・ブザー・パラメータ）."""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox,
    QColorDialog,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ...fleet.control_core import ControlCore
from ...protocol import messages as m

KEY_HELP = ("<b>キー操作</b>（フィールドビューにフォーカス）<br>"
            "W/↑: 前進　S/↓: 後退　A/←: 左旋回　D/→: 右旋回<br>"
            "Space: 停止　Esc: 全台 E-STOP")


class TeleopPanel(QWidget):
    def __init__(self, core: ControlCore, parent=None):
        super().__init__(parent)
        self.core = core
        self.robot_id: int | None = None
        self.keys: set[int] = set()

        self.robot_label = QLabel("対象: なし（一覧かフィールドでロボットを選択）")
        self.robot_label.setStyleSheet("font-weight: bold;")
        self.enable = QCheckBox("キーボード操作を有効にする")
        self.enable.setChecked(True)
        self.v_max = QDoubleSpinBox()
        self.v_max.setRange(0.02, 0.6)
        self.v_max.setSingleStep(0.02)
        self.v_max.setValue(0.15)
        self.w_max = QDoubleSpinBox()
        self.w_max.setRange(0.2, 6.0)
        self.w_max.setSingleStep(0.2)
        self.w_max.setValue(1.5)
        drive = QGroupBox("走行")
        f = QFormLayout(drive)
        f.addRow(self.enable)
        f.addRow("並進 [m/s]", self.v_max)
        f.addRow("旋回 [rad/s]", self.w_max)
        f.addRow(QLabel(KEY_HELP))

        # LED
        self.led_color = QColor(0, 200, 255)
        self.color_btn = QPushButton("色を選ぶ")
        self.color_btn.clicked.connect(self._pick_color)
        self.pattern = QComboBox()
        self.pattern.addItems(list(m.LED_PATTERNS))
        led_send = QPushButton("LED 送信")
        led_send.clicked.connect(self._send_led)
        self.all_robots = QCheckBox("全台に送る")
        led = QGroupBox("LED / ブザー")
        g = QFormLayout(led)
        row = QHBoxLayout()
        row.addWidget(self.color_btn)
        row.addWidget(self.pattern)
        row.addWidget(led_send)
        g.addRow(row)
        self.melody = QComboBox()
        self.melody.addItems(list(m.MELODIES))
        play = QPushButton("♪ 鳴らす")
        play.clicked.connect(self._send_buzzer)
        row2 = QHBoxLayout()
        row2.addWidget(self.melody)
        row2.addWidget(play)
        g.addRow(row2)
        g.addRow(self.all_robots)

        # パラメータ
        params = QGroupBox("ロボットパラメータ (set_param)")
        h = QFormLayout(params)
        self.param_key = QComboBox()
        self.param_key.setEditable(True)
        self.param_key.addItems(["kp", "ki", "kff", "u_deadzone", "max_v", "max_w", "accel_limit", "wheel_radius",
                                 "tread", "watchdog_ms", "telemetry_hz", "batt_low_v"])
        self.param_value = QLineEdit("0.0")
        send_param = QPushButton("書き込み")
        send_param.clicked.connect(self._send_param)
        get_params = QPushButton("読み出し")
        get_params.clicked.connect(lambda: self._to_target(m.GetParams()))
        row3 = QHBoxLayout()
        row3.addWidget(self.param_key)
        row3.addWidget(self.param_value)
        row3.addWidget(send_param)
        row3.addWidget(get_params)
        h.addRow(row3)
        self.params_label = QLabel()
        self.params_label.setWordWrap(True)
        self.params_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        h.addRow(self.params_label)

        lay = QVBoxLayout(self)
        lay.addWidget(self.robot_label)
        lay.addWidget(drive)
        lay.addWidget(led)
        lay.addWidget(params)
        lay.addStretch(1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._drive)
        self.timer.start(50)

    def set_robot(self, rid: int) -> None:
        self.robot_id = rid
        self.robot_label.setText(f"対象: ロボット #{rid}")
        self.core.fleet.request_params(rid)

    def _targets(self) -> list[int | None]:
        return [None] if self.all_robots.isChecked() else ([self.robot_id] if self.robot_id is not None else [])

    def _to_target(self, msg: m.Message) -> None:
        for rid in self._targets():
            self.core.send_direct(rid, msg)

    def _pick_color(self) -> None:
        c = QColorDialog.getColor(self.led_color, self, "LED の色")
        if c.isValid():
            self.led_color = c
            self.color_btn.setStyleSheet(f"background-color: {c.name()};")

    def _send_led(self) -> None:
        c = self.led_color
        self._to_target(m.SetLed(r=c.red(), g=c.green(), b=c.blue(), pattern=self.pattern.currentText(),
                                 period_ms=800))

    def _send_buzzer(self) -> None:
        self._to_target(m.Buzzer(melody=self.melody.currentText()))

    def _send_param(self) -> None:
        try:
            value = float(self.param_value.text())
        except ValueError:
            return
        self._to_target(m.SetParam(key=self.param_key.currentText(), value=value))

    # ------------------------------------------------------------ キーボード
    def key_pressed(self, key: int) -> bool:
        K = Qt.Key
        if key == K.Key_Escape:
            self.core.estop_all()
            return True
        if key in (K.Key_W, K.Key_S, K.Key_A, K.Key_D, K.Key_Up, K.Key_Down, K.Key_Left, K.Key_Right, K.Key_Space):
            self.keys.add(key)
            return True
        return False

    def key_released(self, key: int) -> None:
        self.keys.discard(key)

    def _drive(self) -> None:
        if self.robot_id is not None:
            p = self.core.fleet.get(self.robot_id)
            if p and p.params:
                self.params_label.setText("  ".join(f"{k}={v:g}" for k, v in sorted(p.params.items())))
        if not self.enable.isChecked() or self.robot_id is None or not self.keys:
            return
        K = Qt.Key
        k = self.keys
        if K.Key_Space in k:
            self.core.set_manual(self.robot_id, 0.0, 0.0)
            return
        fwd = (K.Key_W in k or K.Key_Up in k) - (K.Key_S in k or K.Key_Down in k)
        turn = (K.Key_A in k or K.Key_Left in k) - (K.Key_D in k or K.Key_Right in k)
        self.core.set_manual(self.robot_id, fwd * self.v_max.value(), turn * self.w_max.value())
