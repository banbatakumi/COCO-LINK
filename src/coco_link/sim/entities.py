"""シミュレーション世界の物体."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from ..common.geometry import Pose2D
from ..robot.firmware_model import FirmwareModel
from ..robot.params import RobotParams
from ..robot.plant import DiffDrivePlant


@dataclass
class CircleBody:
    """円で近似した物体（人・障害物・物資）."""

    x: float
    y: float
    r: float
    kind: str = "obstacle"        # "person" / "obstacle" / "cargo"
    uid: int = 0
    vx: float = 0.0               # 人のキーボード移動用
    vy: float = 0.0
    movable: bool = False         # ロボットに押されて動くか（物資）
    dragging: bool = False


@dataclass
class Battery:
    """NiMH ×4 の簡易モデル: 開放電圧は SOC に線形, 内部抵抗で電圧降下."""

    soc: float = 1.0               # 0..1
    v_full: float = 5.4
    v_empty: float = 4.4
    r_int_drop: float = 0.25       # duty 1.0 (両輪) あたりの電圧降下 [V]
    drain_per_s: float = 1.0 / 3600.0   # 待機時の放電率（1 時間で空）
    drain_motor_per_s: float = 1.0 / 1800.0

    def voltage(self, duty_l: float, duty_r: float) -> float:
        v_oc = self.v_empty + (self.v_full - self.v_empty) * self.soc
        return v_oc - self.r_int_drop * (abs(duty_l) + abs(duty_r)) / 2.0

    def step(self, duty_l: float, duty_r: float, dt: float) -> None:
        load = (abs(duty_l) + abs(duty_r)) / 2.0
        self.soc = max(0.0, self.soc - (self.drain_per_s + self.drain_motor_per_s * load) * dt)


@dataclass
class SimRobot:
    robot_id: int
    plant: DiffDrivePlant
    firmware: FirmwareModel
    battery: Battery = field(default_factory=Battery)
    duty: tuple[float, float] = (0.0, 0.0)
    dragging: bool = False
    gyro_noise: float = 0.003
    rng: random.Random = field(default_factory=random.Random)

    @property
    def pose(self) -> Pose2D:
        return self.plant.pose

    @pose.setter
    def pose(self, p: Pose2D) -> None:
        self.plant.pose = p

    @property
    def params(self) -> RobotParams:
        return self.plant.params

    @classmethod
    def create(cls, robot_id: int, pose: Pose2D, nominal: RobotParams, rng: random.Random,
               spread: float = 0.10) -> SimRobot:
        """個体差 (spread) を持つ仮想ロボットを生成. ファームは公称値で初期化される（実機と同じ状況）."""
        true_params = nominal.randomized(rng, spread) if spread > 0 else nominal
        plant = DiffDrivePlant(true_params, pose=pose,
                               enc_offset_l=rng.uniform(0, 2 * math.pi), enc_offset_r=rng.uniform(0, 2 * math.pi))
        fw = FirmwareModel(robot_id, {"wheel_radius": nominal.wheel_radius, "tread": nominal.tread})
        return cls(robot_id=robot_id, plant=plant, firmware=fw, rng=random.Random(rng.random()))

    def truth_dict(self) -> dict[str, float]:
        """sim_truth で送る真値."""
        return {k: round(v, 6) for k, v in self.params.to_dict().items()}
