"""床面キャリブレーション: 画像座標 [px] → フィールド座標 [m] の射影変換（ホモグラフィ）.

天井カメラで床を撮ると、床面上の点の画像座標とフィールド座標は 3×3 の射影変換 H で結ばれる:
    s [x y 1]^T = H [u v 1]^T
4 点以上の対応（四隅の ArUco マーカー、または画面上で 4 点クリック）から H を求める。

視差補正: ロボット上面のマーカーは床から高さ h にあるため、床面ホモグラフィで変換すると
カメラ直下点（天底）から外側へずれる。カメラ高さ H が分かれば
    p_true = c + (p - c) (H - h) / H        (c: 天底のフィールド座標)
で補正できる（相似三角形）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..common.config import FieldConfig, config_dir, load_yaml, save_yaml


@dataclass
class FieldCalibration:
    H: np.ndarray                      # 画像 → フィールド (3x3)
    image_size: tuple[int, int] = (0, 0)
    camera_height: float = 2.0         # [m] 床からカメラまで（視差補正用, 0 で補正なし）
    source: str = "manual"

    @classmethod
    def from_points(cls, image_pts, field_pts, image_size=(0, 0), camera_height: float = 2.0,
                    source: str = "manual") -> FieldCalibration:
        img = np.asarray(image_pts, dtype=np.float64).reshape(-1, 2)
        fld = np.asarray(field_pts, dtype=np.float64).reshape(-1, 2)
        if len(img) < 4:
            raise ValueError("4 点以上必要です")
        H, _ = cv2.findHomography(img, fld, method=0)
        if H is None:
            raise ValueError("ホモグラフィを計算できません（点が一直線上にある等）")
        return cls(H=H, image_size=tuple(image_size), camera_height=camera_height, source=source)

    @classmethod
    def from_corner_markers(cls, markers: dict[int, np.ndarray], field: FieldConfig, image_size=(0, 0),
                            camera_height: float = 2.0) -> FieldCalibration | None:
        """四隅マーカー (ID は field.corner_marker_ids の順に原点, (w,0), (w,h), (0,h)) の中心を使う."""
        ids = field.corner_marker_ids
        if not all(i in markers for i in ids):
            return None
        img = [markers[i].reshape(4, 2).mean(axis=0) for i in ids]
        fld = [(0.0, 0.0), (field.width, 0.0), (field.width, field.height), (0.0, field.height)]
        return cls.from_points(img, fld, image_size, camera_height, source="markers")

    def to_field(self, pts, height: float = 0.0) -> np.ndarray:
        """画像座標 (N,2) → フィールド座標 (N,2). height は床からの高さ（視差補正）."""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(p, self.H).reshape(-1, 2)
        if height > 0 and self.camera_height > height:
            c = self.nadir()
            out = c + (out - c) * (self.camera_height - height) / self.camera_height
        return out

    def to_image(self, pts) -> np.ndarray:
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, np.linalg.inv(self.H)).reshape(-1, 2)

    def nadir(self) -> np.ndarray:
        """カメラ直下点（画像中心を床へ投影した点で近似. 真下向きカメラを仮定）."""
        w, h = self.image_size
        center = np.array([[w / 2, h / 2]]) if w and h else np.array([[0.0, 0.0]])
        return cv2.perspectiveTransform(center.reshape(-1, 1, 2), self.H).reshape(2)

    # ------------------------------------------------------------ 保存
    @staticmethod
    def default_path() -> Path:
        return config_dir() / "vision_calibration.yaml"

    def save(self, path: str | Path | None = None) -> Path:
        path = Path(path or self.default_path())
        save_yaml(path, {"H": self.H.tolist(), "image_size": list(self.image_size),
                         "camera_height": self.camera_height, "source": self.source})
        return path

    @classmethod
    def load(cls, path: str | Path | None = None) -> FieldCalibration | None:
        data = load_yaml(path or cls.default_path())
        if "H" not in data:
            return None
        return cls(H=np.array(data["H"], dtype=np.float64), image_size=tuple(data.get("image_size", (0, 0))),
                   camera_height=float(data.get("camera_height", 2.0)), source=data.get("source", "file"))
