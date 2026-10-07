"""ビジョン GUI: 映像と検出結果の表示、キャリブレーション、色閾値の調整."""

from __future__ import annotations

import math

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QMouseEvent, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ...common.config import ColorRange
from ...vision.calibration import FieldCalibration
from ...vision.color_detector import color_mask
from ...vision.pipeline import draw_overlay
from ...vision.worker import VisionWorker

KIND_LABEL = {"person": "人（緑）", "obstacle": "障害物（赤）", "cargo": "物資（青）"}


class VideoLabel(QLabel):
    """映像表示. クリック位置を画像座標で返す."""

    def __init__(self, on_click):
        super().__init__()
        self.on_click = on_click
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(640, 400)
        self.setStyleSheet("background:#333;")
        self.img_size = (1, 1)

    def show_image(self, bgr: np.ndarray) -> None:
        h, w = bgr.shape[:2]
        self.img_size = (w, h)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self.setPixmap(QPixmap.fromImage(qimg).scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                      Qt.TransformationMode.SmoothTransformation))

    def mousePressEvent(self, e: QMouseEvent) -> None:
        pm = self.pixmap()
        if pm is None or pm.isNull():
            return
        ox = (self.width() - pm.width()) / 2
        oy = (self.height() - pm.height()) / 2
        u = (e.position().x() - ox) * self.img_size[0] / pm.width()
        v = (e.position().y() - oy) * self.img_size[1] / pm.height()
        if 0 <= u < self.img_size[0] and 0 <= v < self.img_size[1]:
            self.on_click(u, v)


class VisionWindow(QMainWindow):
    def __init__(self, worker: VisionWorker):
        super().__init__()
        self.worker = worker
        self.pipeline = worker.pipeline
        self.setWindowTitle(f"COCO-LINK Vision — {worker.source.name}")
        self.resize(1300, 800)
        self.click_points: list[tuple[float, float]] = []

        self.video = VideoLabel(self._on_click)
        side = QWidget()
        side.setFixedWidth(360)
        sv = QVBoxLayout(side)

        # 状態
        self.info = QLabel()
        self.info.setStyleSheet("font-family: monospace;")
        sv.addWidget(self.info)

        # キャリブレーション
        cal = QGroupBox("床面キャリブレーション")
        cf = QFormLayout(cal)
        self.auto_cal = QCheckBox("四隅マーカー (ID 40–43) で自動")
        self.auto_cal.setChecked(self.pipeline.auto_calibrate)
        self.auto_cal.toggled.connect(lambda v: setattr(self.pipeline, "auto_calibrate", v))
        cf.addRow(self.auto_cal)
        click_btn = QPushButton("4 点クリックで校正（原点→x軸端→対角→y軸端）")
        click_btn.clicked.connect(self._start_click_calibration)
        cf.addRow(click_btn)
        save_btn = QPushButton("校正結果を保存")
        save_btn.clicked.connect(self._save_calibration)
        cf.addRow(save_btn)
        self.cam_h = QDoubleSpinBox()
        self.cam_h.setRange(0.0, 10.0)
        self.cam_h.setSingleStep(0.1)
        self.cam_h.setValue(self.pipeline.camera_height)
        self.cam_h.valueChanged.connect(self._heights_changed)
        self.mk_h = QDoubleSpinBox()
        self.mk_h.setRange(0.0, 0.5)
        self.mk_h.setSingleStep(0.01)
        self.mk_h.setValue(self.pipeline.marker_height)
        self.mk_h.valueChanged.connect(self._heights_changed)
        cf.addRow("カメラ高さ [m]", self.cam_h)
        cf.addRow("マーカー高さ [m]", self.mk_h)
        sv.addWidget(cal)

        # 色閾値
        col = QGroupBox("色の閾値 (HSV)")
        cv = QVBoxLayout(col)
        self.kind = QComboBox()
        for k, label in KIND_LABEL.items():
            self.kind.addItem(label, k)
        self.kind.currentIndexChanged.connect(self._load_sliders)
        self.show_mask = QCheckBox("マスク表示")
        row = QHBoxLayout()
        row.addWidget(self.kind)
        row.addWidget(self.show_mask)
        cv.addLayout(row)
        self.sliders: dict[str, QSlider] = {}
        form = QFormLayout()
        for key, mx in (("h_min", 179), ("h_max", 179), ("s_min", 255), ("s_max", 255), ("v_min", 255), ("v_max", 255)):
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(0, mx)
            s.valueChanged.connect(self._slider_changed)
            form.addRow(key, s)
            self.sliders[key] = s
        cv.addLayout(form)
        save_col = QPushButton("閾値を保存 (config/vision_colors.yaml)")
        save_col.clicked.connect(self._save_colors)
        cv.addWidget(save_col)
        sv.addWidget(col)

        self.send = QCheckBox("operator へ送信")
        self.send.setChecked(worker.sending)
        self.send.toggled.connect(lambda v: setattr(self.worker, "sending", v))
        sv.addWidget(self.send)
        self.detections = QPlainTextEdit()
        self.detections.setReadOnly(True)
        self.detections.setStyleSheet("font-family: monospace; font-size: 10px;")
        sv.addWidget(self.detections, 1)

        central = QWidget()
        h = QHBoxLayout(central)
        h.addWidget(self.video, 1)
        h.addWidget(side)
        self.setCentralWidget(central)
        self._load_sliders()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)

    # ------------------------------------------------------------ キャリブレーション
    def _start_click_calibration(self) -> None:
        self.click_points = []
        self.auto_cal.setChecked(False)
        self.statusBar().showMessage("フィールドの 原点(0,0) → (w,0) → (w,h) → (0,h) の順にクリック")

    def _on_click(self, u: float, v: float) -> None:
        if self.auto_cal.isChecked() or len(self.click_points) >= 4:
            return
        self.click_points.append((u, v))
        if len(self.click_points) == 4:
            f = self.pipeline.field
            self.pipeline.calib = FieldCalibration.from_points(
                self.click_points, [(0, 0), (f.width, 0), (f.width, f.height), (0, f.height)],
                self.video.img_size, self.cam_h.value(), source="click")
            self.statusBar().showMessage("校正しました", 5000)
        else:
            self.statusBar().showMessage(f"{len(self.click_points)}/4 点")

    def _save_calibration(self) -> None:
        if self.pipeline.calib is None:
            QMessageBox.warning(self, "校正", "まだ校正されていません")
            return
        path = self.pipeline.calib.save()
        self.statusBar().showMessage(f"保存しました: {path}", 5000)

    def _heights_changed(self) -> None:
        self.pipeline.camera_height = self.cam_h.value()
        self.pipeline.marker_height = self.mk_h.value()
        if self.pipeline.calib is not None:
            self.pipeline.calib.camera_height = self.cam_h.value()

    # ------------------------------------------------------------ 色
    def _current_range(self) -> ColorRange:
        k = self.kind.currentData()
        return self.pipeline.field.colors.setdefault(k, ColorRange())

    def _load_sliders(self) -> None:
        c = self._current_range()
        for key, s in self.sliders.items():
            s.blockSignals(True)
            s.setValue(getattr(c, key))
            s.blockSignals(False)

    def _slider_changed(self) -> None:
        c = self._current_range()
        for key, s in self.sliders.items():
            setattr(c, key, s.value())

    def _save_colors(self) -> None:
        path = self.pipeline.field.save_colors()
        self.statusBar().showMessage(f"保存しました: {path}", 5000)

    # ------------------------------------------------------------ 表示
    def _refresh(self) -> None:
        frame, res = self.worker.latest()
        if frame is None or res is None:
            return
        if self.show_mask.isChecked():
            mask = color_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self._current_range())
            img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            img = draw_overlay(frame, res, self.pipeline.calib)
            for u, v in self.click_points:
                cv2.circle(img, (int(u), int(v)), 6, (0, 255, 255), 2)
        self.video.show_image(img)
        ws = res.world
        cal = self.pipeline.calib.source if self.pipeline.calib else "未校正"
        self.info.setText(f"FPS {self.worker.fps:5.1f}   処理 {res.process_ms:5.1f} ms\n"
                          f"遅延(推定) {ws.latency_ms:5.1f} ms   送信 {self.worker.sent}\n"
                          f"校正: {cal}   マーカー {len(res.markers)} 個")
        lines = [f"robot #{r.id}: x={r.x:.3f} y={r.y:.3f} θ={math.degrees(r.th):+.1f}° conf={r.conf:.2f}"
                 for r in ws.robots]
        for name, objs in (("person", ws.persons), ("obstacle", ws.obstacles), ("cargo", ws.cargo)):
            lines += [f"{name}: x={o.x:.3f} y={o.y:.3f} r={o.r:.3f}" for o in objs]
        self.detections.setPlainText("\n".join(lines))

    def closeEvent(self, e) -> None:
        self.timer.stop()
        self.worker.stop()
        super().closeEvent(e)
