"""フィールド (床面座標 [m]) を描画するキャンバスの基底クラス.

ワールド座標 (x右, y上) ⇄ 画面座標 (x右, y下) の変換、グリッド描画、マウス座標変換を提供する。
派生クラスは `draw_world(painter)` を実装する。
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QMouseEvent, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

COLORS = {
    "person": QColor(40, 170, 60),
    "obstacle": QColor(210, 50, 50),
    "cargo": QColor(40, 90, 220),
    "field": QColor(245, 245, 240),
    "grid": QColor(220, 220, 215),
    "border": QColor(90, 90, 90),
    "robot": QColor(60, 60, 60),
    "target": QColor(230, 140, 0),
}


class FieldCanvas(QWidget):
    world_clicked = Signal(float, float, object)   # x, y, Qt.MouseButton

    def __init__(self, field_w: float = 3.0, field_h: float = 2.0, parent=None):
        super().__init__(parent)
        self.field_w = field_w
        self.field_h = field_h
        self.margin_px = 24
        self.setMinimumSize(480, 320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.mouse_world: tuple[float, float] | None = None

    # ------------------------------------------------------------ 座標変換
    def scale(self) -> float:
        sx = (self.width() - 2 * self.margin_px) / max(self.field_w, 1e-3)
        sy = (self.height() - 2 * self.margin_px) / max(self.field_h, 1e-3)
        return max(1.0, min(sx, sy))

    def origin(self) -> QPointF:
        s = self.scale()
        ox = (self.width() - self.field_w * s) / 2
        oy = (self.height() + self.field_h * s) / 2
        return QPointF(ox, oy)

    def to_screen(self, x: float, y: float) -> QPointF:
        o, s = self.origin(), self.scale()
        return QPointF(o.x() + x * s, o.y() - y * s)

    def to_world(self, p: QPointF) -> tuple[float, float]:
        o, s = self.origin(), self.scale()
        return (p.x() - o.x()) / s, (o.y() - p.y()) / s

    # ------------------------------------------------------------ 描画
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(200, 200, 195))
        self._draw_field(p)
        self.draw_world(p)
        self.draw_overlay(p)
        p.end()

    def _draw_field(self, p: QPainter) -> None:
        tl, br = self.to_screen(0, self.field_h), self.to_screen(self.field_w, 0)
        p.fillRect(QRectF(tl, br), COLORS["field"])
        p.setPen(QPen(COLORS["grid"], 1))
        step = 0.5
        for i in range(1, int(self.field_w / step) + 1):
            x = i * step
            if x < self.field_w:
                p.drawLine(self.to_screen(x, 0), self.to_screen(x, self.field_h))
        for j in range(1, int(self.field_h / step) + 1):
            y = j * step
            if y < self.field_h:
                p.drawLine(self.to_screen(0, y), self.to_screen(self.field_w, y))
        p.setPen(QPen(COLORS["border"], 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(QRectF(tl, br))
        # 原点と軸
        o = self.to_screen(0, 0)
        p.setPen(QPen(QColor(200, 0, 0), 2))
        p.drawLine(o, self.to_screen(0.15, 0))
        p.setPen(QPen(QColor(0, 150, 0), 2))
        p.drawLine(o, self.to_screen(0, 0.15))

    def draw_world(self, p: QPainter) -> None:  # 派生クラスで実装
        pass

    def draw_overlay(self, p: QPainter) -> None:
        if self.mouse_world:
            p.setPen(QColor(60, 60, 60))
            p.setFont(QFont("", 9))
            x, y = self.mouse_world
            p.drawText(8, self.height() - 8, f"x={x:.2f} m  y={y:.2f} m")

    # ------------------------------------------------------------ 描画ヘルパ
    def draw_circle(self, p: QPainter, x: float, y: float, r: float, fill: QColor, pen: QColor | None = None,
                    width: float = 1.0) -> None:
        s = self.scale()
        p.setPen(QPen(pen or fill.darker(140), width))
        p.setBrush(QBrush(fill))
        p.drawEllipse(self.to_screen(x, y), r * s, r * s)

    def draw_robot(self, p: QPainter, x: float, y: float, th: float, r: float, fill: QColor, label: str = "",
                   outline: QColor | None = None, alpha: int = 255) -> None:
        fill = QColor(fill)
        fill.setAlpha(alpha)
        self.draw_circle(p, x, y, r, fill, outline or QColor(40, 40, 40, alpha), 2)
        # 向き（三角形）
        tip = self.to_screen(x + r * math.cos(th), y + r * math.sin(th))
        left = self.to_screen(x + 0.5 * r * math.cos(th + 2.4), y + 0.5 * r * math.sin(th + 2.4))
        rr = self.to_screen(x + 0.5 * r * math.cos(th - 2.4), y + 0.5 * r * math.sin(th - 2.4))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(255, 255, 255, int(alpha * 0.9)))
        p.drawPolygon(QPolygonF([tip, left, rr]))
        if label:
            p.setPen(QColor(0, 0, 0, alpha))
            f = QFont("", 9)
            f.setBold(True)
            p.setFont(f)
            c = self.to_screen(x, y)
            p.drawText(QRectF(c.x() - 30, c.y() + r * self.scale(), 60, 16), Qt.AlignmentFlag.AlignCenter, label)

    def draw_path(self, p: QPainter, pts: list[tuple[float, float]], color: QColor, width: float = 1.5,
                  dashed: bool = False) -> None:
        if len(pts) < 2:
            return
        path = QPainterPath(self.to_screen(*pts[0]))
        for q in pts[1:]:
            path.lineTo(self.to_screen(*q))
        pen = QPen(color, width)
        if dashed:
            pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(path)

    def draw_cone(self, p: QPainter, x: float, y: float, th: float, fov: float, dist: float, color: QColor) -> None:
        """超音波の測距範囲（扇形）."""
        pts = [self.to_screen(x, y)]
        for i in range(9):
            a = th - fov / 2 + fov * i / 8
            pts.append(self.to_screen(x + dist * math.cos(a), y + dist * math.sin(a)))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(color)
        p.drawPolygon(QPolygonF(pts))

    # ------------------------------------------------------------ マウス
    def mouseMoveEvent(self, e: QMouseEvent) -> None:
        self.mouse_world = self.to_world(e.position())
        self.update()

    def mousePressEvent(self, e: QMouseEvent) -> None:
        x, y = self.to_world(e.position())
        self.world_clicked.emit(x, y, e.button())
