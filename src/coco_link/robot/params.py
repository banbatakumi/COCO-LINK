"""ロボットの物理パラメータ（シミュレータの真値・制御の公称モデル・同定結果の共通形式）."""

from __future__ import annotations

import random
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path

from ..common.config import config_dir, dataclass_from_dict, load_yaml, save_yaml


@dataclass
class RobotParams:
    wheel_radius: float = 0.021
    tread: float = 0.090
    body_radius: float = 0.060
    motor_gain_l: float = 25.0
    motor_gain_r: float = 25.0
    motor_tau_l: float = 0.060
    motor_tau_r: float = 0.060
    deadzone_l: float = 0.10
    deadzone_r: float = 0.10
    gyro_bias_z: float = 0.0
    batt_nominal_v: float = 4.8
    us_offset: float = 0.06
    us_fov_deg: float = 30.0
    us_max_range: float = 2.0

    # ---- 派生量
    @property
    def max_wheel_speed(self) -> float:
        """duty=1 での定常角速度 [rad/s]（左右の遅い方）."""
        return min(self.motor_gain_l * (1 - self.deadzone_l), self.motor_gain_r * (1 - self.deadzone_r))

    @property
    def max_speed(self) -> float:
        return self.max_wheel_speed * self.wheel_radius

    # ---- 入出力
    def to_dict(self) -> dict[str, float]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> RobotParams:
        return dataclass_from_dict(cls, d)

    @staticmethod
    def path_for(robot_id: int, root: Path | None = None) -> Path:
        return (root or config_dir() / "robots") / f"robot_{robot_id:02d}.yaml"

    @classmethod
    def load(cls, robot_id: int | None = None, root: Path | None = None) -> RobotParams:
        """default.yaml を読み、robot_XX.yaml があれば上書きする."""
        root = root or config_dir() / "robots"
        data = load_yaml(root / "default.yaml")
        if robot_id is not None:
            data.update(load_yaml(cls.path_for(robot_id, root)))
        return cls.from_dict(data)

    def save(self, robot_id: int, root: Path | None = None, extra: dict | None = None) -> Path:
        path = self.path_for(robot_id, root)
        data = self.to_dict()
        if extra:
            data["_meta"] = extra
        save_yaml(path, data)
        return path

    def randomized(self, rng: random.Random, spread: float = 0.10) -> RobotParams:
        """個体差を模擬した真値を作る（シミュレータ用）. 各値を ±spread の一様乱数で揺らす."""
        jitter = {"wheel_radius", "tread", "motor_gain_l", "motor_gain_r", "motor_tau_l", "motor_tau_r",
                  "deadzone_l", "deadzone_r"}
        out = {f.name: getattr(self, f.name) for f in fields(self)}
        for k in jitter:
            out[k] *= 1.0 + rng.uniform(-spread, spread)
        out["wheel_radius"] = self.wheel_radius * (1.0 + rng.uniform(-spread, spread) * 0.3)  # 半径は精度が高い
        out["gyro_bias_z"] = rng.uniform(-0.02, 0.02)
        return replace(self, **out)
