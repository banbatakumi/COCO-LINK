"""接続中ロボットの一覧（電圧・センサ値など）."""

from __future__ import annotations

import math
import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem

from ...fleet.control_core import CoreSnapshot
from ...fleet.fleet_manager import FleetManager
from ...protocol import messages as m

COLUMNS = ["ID", "接続", "状態", "電圧[V]", "前[m]", "後[m]", "ωL/ωR[rad/s]", "gz[rad/s]", "姿勢 x,y,θ",
           "姿勢源", "受信[Hz]", "RTT[ms]", "RSSI", "異常"]

STATE_COLOR = {m.STATE_RUNNING: QColor(0, 110, 210), m.STATE_IDLE: QColor(0, 140, 0),
               m.STATE_ESTOP: QColor(220, 0, 0), m.STATE_LOW_BATT: QColor(230, 120, 0),
               m.STATE_FAULT: QColor(150, 0, 220)}


def _fmt(v, f="{:.2f}"):
    return "-" if v is None else f.format(v)


class RobotTable(QTableWidget):
    robot_selected = Signal(int)

    def __init__(self, fleet: FleetManager, parent=None):
        super().__init__(0, len(COLUMNS), parent)
        self.fleet = fleet
        self.setHorizontalHeaderLabels(COLUMNS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.verticalHeader().setVisible(False)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.itemSelectionChanged.connect(self._on_select)
        self._ids: list[int] = []

    def _on_select(self) -> None:
        rows = self.selectionModel().selectedRows()
        if rows and rows[0].row() < len(self._ids):
            self.robot_selected.emit(self._ids[rows[0].row()])

    def select_robot(self, rid: int) -> None:
        if rid in self._ids:
            self.blockSignals(True)
            self.selectRow(self._ids.index(rid))
            self.blockSignals(False)

    def refresh(self, snap: CoreSnapshot) -> None:
        now = time.monotonic()
        with self.fleet.lock:
            proxies = dict(self.fleet.robots)
        ids = sorted(set(proxies) | set(snap.world.robots))
        if ids != self._ids:
            self._ids = ids
            self.setRowCount(len(ids))
        for row, rid in enumerate(ids):
            px = proxies.get(rid)
            st = snap.world.robots.get(rid)
            t = px.telemetry if px else None
            connected = px is not None and px.is_connected(now, self.fleet.net.disconnect_timeout_s)
            pose = st.pose if st else None
            cells = [
                f"#{rid}" + (" (sim)" if px and px.is_simulated else ""),
                "✓" if connected else "×",
                t.state if t else "-",
                _fmt(t.batt_v if t else None),
                _fmt(t.us_front_m if t else None),
                _fmt(t.us_rear_m if t else None),
                f"{t.enc.wl:+.1f}/{t.enc.wr:+.1f}" if t else "-",
                _fmt(t.imu.gz if t else None, "{:+.3f}"),
                f"{pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.th):+.0f}°" if pose else "-",
                st.pose_source if st else "-",
                f"{px.telemetry_hz:.0f}" if px else "-",
                _fmt(px.rtt_ms if px else None, "{:.1f}"),
                str(t.rssi) if t else "-",
                ",".join(t.faults) if t and t.faults else "",
            ]
            for col, text in enumerate(cells):
                item = self.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.setItem(row, col, item)
                if item.text() != text:
                    item.setText(text)
                if col == 2:
                    item.setForeground(STATE_COLOR.get(text, QColor(0, 0, 0)))
                if col == 1:
                    item.setForeground(QColor(0, 150, 0) if connected else QColor(200, 0, 0))
                if col == 3 and t is not None:
                    item.setForeground(QColor(220, 0, 0) if t.batt_v < 4.4 else QColor(0, 0, 0))
