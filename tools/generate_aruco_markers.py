"""印刷用の ArUco マーカーを生成する（実寸の PDF と、個別 PNG）.

例:
    python tools/generate_aruco_markers.py                       # ロボット 1-8 と四隅 40-43
    python tools/generate_aruco_markers.py --robots 1 2 3 --robot-size 60 --corner-size 100

PDF は A4・実寸（mm 指定）なので、印刷時は「実際のサイズ」「拡大縮小なし」を選ぶこと。
印刷後に定規で黒枠の一辺を測り、config/field.yaml の robot_marker_size / corner_marker_size と一致しているか確認する。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2  # noqa: E402

from coco_link.common.config import FieldConfig  # noqa: E402


def marker_png(dictionary, marker_id: int, px: int = 600):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    return cv2.copyMakeBorder(img, px // 6, px // 6, px // 6, px // 6, cv2.BORDER_CONSTANT, value=255)


def write_pdf(path: Path, items: list[tuple[int, float, str]], dictionary) -> None:
    """items: (ID, 一辺 mm, ラベル). A4 に左上から並べる."""
    from PySide6.QtCore import QMarginsF, QRectF, Qt
    from PySide6.QtGui import (
        QFont,
        QGuiApplication,
        QImage,
        QPageLayout,
        QPageSize,
        QPainter,
        QPdfWriter,
        QPen,
    )

    _app = QGuiApplication.instance() or QGuiApplication([])
    pdf = QPdfWriter(str(path))
    pdf.setPageSize(QPageSize(QPageSize.PageSizeId.A4))
    pdf.setPageLayout(QPageLayout(QPageSize(QPageSize.PageSizeId.A4), QPageLayout.Orientation.Portrait,
                                  QMarginsF(10, 10, 10, 10), QPageLayout.Unit.Millimeter))
    pdf.setResolution(600)
    p = QPainter(pdf)
    mm = 600 / 25.4
    page_w, page_h = 190.0, 277.0
    x = y = 0.0
    row_h = 0.0
    for mid, size, label in items:
        quiet = size / 4          # 周囲の白余白
        cell = size + 2 * quiet
        if x + cell > page_w:
            x, y = 0.0, y + row_h + 8
            row_h = 0.0
        if y + cell + 6 > page_h:
            pdf.newPage()
            x = y = row_h = 0.0
        img = cv2.aruco.generateImageMarker(dictionary, mid, 600)
        qimg = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format.Format_Grayscale8)
        p.drawImage(QRectF((x + quiet) * mm, (y + quiet) * mm, size * mm, size * mm), qimg)
        p.setPen(QPen(Qt.GlobalColor.lightGray, 2, Qt.PenStyle.DashLine))
        p.drawRect(QRectF(x * mm, y * mm, cell * mm, cell * mm))      # 切り取り線
        p.setPen(Qt.GlobalColor.black)
        p.setFont(QFont("", 8))
        p.drawText(QRectF(x * mm, (y + cell) * mm, cell * mm, 6 * mm), Qt.AlignmentFlag.AlignCenter,
                   f"{label}  ID {mid}  ({size:.0f} mm)  ↑前")
        x += cell + 5
        row_h = max(row_h, cell + 6)
    p.end()


def main(argv=None) -> int:
    field = FieldConfig.load()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", type=int, nargs="*", default=list(range(1, 9)))
    ap.add_argument("--robot-size", type=float, default=field.robot_marker_size * 1000, help="[mm]")
    ap.add_argument("--corner-size", type=float, default=field.corner_marker_size * 1000, help="[mm]")
    ap.add_argument("--out", type=Path, default=Path("output/markers"))
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, field.aruco_dict))
    items = [(i, args.robot_size, "robot") for i in args.robots]
    names = ["origin(0,0)", "(w,0)", "(w,h)", "(0,h)"]
    items += [(i, args.corner_size, f"corner {n}") for i, n in zip(field.corner_marker_ids, names, strict=True)]
    for mid, _, _label in items:
        cv2.imwrite(str(args.out / f"aruco_{mid:02d}.png"), marker_png(dictionary, mid))
    pdf = args.out / "aruco_markers_A4.pdf"
    write_pdf(pdf, items, dictionary)
    print(f"{len(items)} markers -> {args.out}/ (PDF: {pdf})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
