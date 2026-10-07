"""ロボットの「本物の物理」モデル（シミュレータとテストで使う）.

車輪ごとに
    不感帯:   u_eff = sign(u) (|u| - u0)   (|u| > u0 のとき, それ以外 0)
    電圧:     u_eff ← u_eff * V_batt / V_nom
    一次遅れ: τ dω/dt + ω = K u_eff
差動2輪の運動学で機体速度を求め、円弧で姿勢を積分する。
このモデル構造は docs/system_identification.md の同定対象そのものである。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..common.geometry import Pose2D
from .kinematics import integrate_unicycle, wheel_to_body
from .params import RobotParams

ENC_COUNTS = 4096


def deadzone(u: float, u0: float) -> float:
    if abs(u) <= u0:
        return 0.0
    return math.copysign(abs(u) - u0, u)


@dataclass
class DiffDrivePlant:
    params: RobotParams
    pose: Pose2D = field(default_factory=Pose2D)
    omega_l: float = 0.0       # 真の車輪角速度 [rad/s]
    omega_r: float = 0.0
    theta_l: float = 0.0       # 真の車輪累積角 [rad]
    theta_r: float = 0.0
    vx: float = 0.0            # 機体速度
    wz: float = 0.0
    ax: float = 0.0            # 前後加速度（IMU 用）
    enc_offset_l: float = 0.0  # 磁石取り付け角（AS5600 の 0 点ずれ）
    enc_offset_r: float = 0.0

    def step(self, duty_l: float, duty_r: float, batt_v: float, dt: float, move_pose: bool = True) -> None:
        p = self.params
        k_batt = batt_v / p.batt_nominal_v
        vx_prev = self.vx
        for side, duty, K, tau, u0 in (("l", duty_l, p.motor_gain_l, p.motor_tau_l, p.deadzone_l),
                                       ("r", duty_r, p.motor_gain_r, p.motor_tau_r, p.deadzone_r)):
            u = deadzone(max(-1.0, min(1.0, duty)), u0) * k_batt
            w = getattr(self, f"omega_{side}")
            # 一次遅れを厳密離散化 (dt が τ に比べ大きくても安定)
            a = math.exp(-dt / tau)
            w = a * w + (1 - a) * K * u
            setattr(self, f"omega_{side}", w)
            setattr(self, f"theta_{side}", getattr(self, f"theta_{side}") + w * dt)
        self.vx, self.wz = wheel_to_body(self.omega_l, self.omega_r, p.wheel_radius, p.tread)
        self.ax = (self.vx - vx_prev) / dt if dt > 0 else 0.0
        if move_pose:
            self.pose = integrate_unicycle(self.pose, self.vx, self.wz, dt)

    def encoder_raw(self) -> tuple[int, int]:
        """AS5600 の 12bit 生値 (0..4095)."""
        def raw(th: float) -> int:
            return int(math.floor((th % (2 * math.pi)) / (2 * math.pi) * ENC_COUNTS)) % ENC_COUNTS
        return raw(self.theta_l + self.enc_offset_l), raw(self.theta_r + self.enc_offset_r)
