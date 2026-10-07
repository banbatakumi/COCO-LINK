"""操作GUIのフィールドビュー: ビジョンで見えている世界と、モードの目標・経路を描く."""

from __future__ import annotations

import math
from collections import deque

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont, QMouseEvent, QPainter, QPen

from ...fleet.control_core import CoreSnapshot
from ..common.field_canvas import COLORS, FieldCanvas

PALETTE = [QColor(c) for c in ("#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#46f0f0", "#f032e6",
                               "#bcf60c", "#fabebe", "#008080", "#9a6324", "#800000")]


def robot_color(rid: int) -> QColor:
    return PALETTE[rid % len(PALETTE)]


class OperatorFieldView(FieldCanvas):
    robot_selected = Signal(int)
    field_clicked = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.snapshot = CoreSnapshot()
        self.selected: int | None = None
        self.history: dict[int, deque] = {}
        self.show_trails = True

    def set_snapshot(self, snap: CoreSnapshot) -> None:
        self.snapshot = snap
        self.field_w, self.field_h = snap.world.field_w, snap.world.field_h
        for rid, r in snap.world.robots.items():
            if r.pose is not None:
                h = self.history.setdefault(rid, deque(maxlen=150))
                if not h or math.hypot(h[-1][0] - r.pose.x, h[-1][1] - r.pose.y) > 0.01:
                    h.append((r.pose.x, r.pose.y))
        self.update()

    def draw_world(self, p: QPainter) -> None:
        snap = self.snapshot
        w = snap.world
        for kind, objs in (("obstacle", w.obstacles), ("cargo", w.cargo), ("person", w.persons)):
            for o in objs:
                self.draw_circle(p, o.x, o.y, o.r, QColor(COLORS[kind]).lighter(120))
        # モードの補助表示
        for pts in snap.viz.paths.values():
            self.draw_path(p, pts, QColor(120, 120, 120), 1.5, dashed=True)
        for x, y, label in snap.viz.points:
            self.draw_circle(p, x, y, 0.02, COLORS["target"])
            p.drawText(self.to_screen(x + 0.03, y + 0.03), label)
        if self.show_trails:
            for rid, h in self.history.items():
                c = QColor(robot_color(rid))
                c.setAlpha(90)
                self.draw_path(p, list(h), c, 2)
        for rid, target in snap.viz.targets.items():
            r = w.robots.get(rid)
            c = robot_color(rid)
            self.draw_robot(p, target.x, target.y, target.th, r.params.body_radius if r else 0.06, c, alpha=60)
            if r and r.pose:
                self.draw_path(p, [(r.pose.x, r.pose.y), (target.x, target.y)], c, 1, dashed=True)
        for rid, r in sorted(w.robots.items()):
            if r.pose is None:
                continue
            outline = QColor(255, 200, 0) if rid == self.selected else None
            alpha = 255 if r.pose_source == "vision" else 140
            self.draw_robot(p, r.pose.x, r.pose.y, r.pose.th, r.params.body_radius, robot_color(rid),
                            f"#{rid}", outline=outline, alpha=alpha)
            if rid == self.selected:
                self.draw_circle(p, r.pose.x, r.pose.y, r.params.body_radius + 0.02, QColor(0, 0, 0, 0),
                                 QColor(255, 200, 0), 3)
            cmd = snap.commands.get(rid)
            if cmd is not None and cmd.wheel is None and abs(cmd.vx) > 1e-3:
                ex = r.pose.x + cmd.vx * 0.8 * math.cos(r.pose.th)
                ey = r.pose.y + cmd.vx * 0.8 * math.sin(r.pose.th)
                p.setPen(QPen(QColor(0, 0, 0, 120), 2))
                p.drawLine(self.to_screen(r.pose.x, r.pose.y), self.to_screen(ex, ey))

    def draw_overlay(self, p: QPainter) -> None:
        super().draw_overlay(p)
        snap = self.snapshot
        if snap.estopped:
            p.setPen(QColor(220, 0, 0))
            f = QFont("", 22)
            f.setBold(True)
            p.setFont(f)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter, "E-STOP")
        elif snap.world.vision_age > 1.0:
            p.setPen(QColor(200, 100, 0))
            p.setFont(QFont("", 11))
            p.drawText(self.rect().adjusted(0, 4, 0, 0), Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
                       "ビジョン(world_state)を受信していません")

    def mousePressEvent(self, e: QMouseEvent) -> None:
        x, y = self.to_world(e.position())
        for rid, r in self.snapshot.world.robots.items():
            if r.pose and math.hypot(r.pose.x - x, r.pose.y - y) < r.params.body_radius + 0.02:
                self.selected = rid
                self.robot_selected.emit(rid)
                return
        if e.button() == Qt.MouseButton.LeftButton:
            self.field_clicked.emit(x, y)
