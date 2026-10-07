"""シミュレーションエンジン（Qt 非依存）.

固定ステップ (既定 200 Hz) で物理を進め、仮想ファームウェアを 100 Hz で動かす。
GUI スレッド・UDP 受信スレッドから同時にアクセスされるので、公開メソッドは `lock` で保護している。
"""

from __future__ import annotations

import math
import random
import threading
from dataclasses import dataclass, field

from ..common.geometry import Pose2D
from ..protocol import messages as m
from ..robot.firmware_model import CONTROL_HZ, SensorInputs
from ..robot.params import RobotParams
from . import sensors
from .entities import CircleBody, SimRobot

PHYSICS_HZ = 200.0
SENSOR_HZ = 50.0

DEFAULT_RADIUS = {"person": 0.15, "obstacle": 0.10, "cargo": 0.05}


@dataclass
class SimConfig:
    field_w: float = 3.0
    field_h: float = 2.0
    seed: int = 0
    param_spread: float = 0.10           # 個体差 ±10 %
    nominal: RobotParams = field(default_factory=RobotParams.load)
    person_speed: float = 0.4            # キーボード操作時の人の速さ [m/s]


class SimEngine:
    def __init__(self, config: SimConfig | None = None):
        self.config = config or SimConfig()
        self.lock = threading.RLock()
        self.rng = random.Random(self.config.seed)
        self.t = 0.0
        self.robots: dict[int, SimRobot] = {}
        self.bodies: list[CircleBody] = []
        self._next_uid = 1
        self._fw_acc = 0.0
        self._sensor_acc = 0.0
        self._sensor_cache: dict[int, tuple[float | None, float | None]] = {}

    # ------------------------------------------------------------ 配置
    @property
    def field_wh(self) -> tuple[float, float]:
        return self.config.field_w, self.config.field_h

    def add_robot(self, robot_id: int | None = None, pose: Pose2D | None = None) -> SimRobot:
        with self.lock:
            if robot_id is None:
                robot_id = next(i for i in range(1, 30) if i not in self.robots)
            if robot_id in self.robots:
                raise ValueError(f"robot {robot_id} already exists")
            if pose is None:
                pose = Pose2D(self.rng.uniform(0.3, self.config.field_w - 0.3),
                              self.rng.uniform(0.3, self.config.field_h - 0.3), self.rng.uniform(-math.pi, math.pi))
            # 個体差は ID ごとに決定的にする（同じ ID は毎回同じ真値）
            rng = random.Random(self.config.seed * 1000 + robot_id)
            robot = SimRobot.create(robot_id, pose, self.config.nominal, rng, self.config.param_spread)
            self.robots[robot_id] = robot
            return robot

    def remove_robot(self, robot_id: int) -> None:
        with self.lock:
            self.robots.pop(robot_id, None)

    def add_body(self, kind: str, x: float, y: float, r: float | None = None) -> CircleBody:
        with self.lock:
            body = CircleBody(x, y, r or DEFAULT_RADIUS[kind], kind=kind, uid=self._next_uid,
                              movable=(kind == "cargo"))
            self._next_uid += 1
            self.bodies.append(body)
            return body

    def remove_body(self, uid: int) -> None:
        with self.lock:
            self.bodies = [b for b in self.bodies if b.uid != uid]

    def clear(self) -> None:
        with self.lock:
            self.robots.clear()
            self.bodies.clear()

    def pick(self, x: float, y: float) -> SimRobot | CircleBody | None:
        """座標 (x, y) にある物体（GUI のドラッグ用）."""
        with self.lock:
            for r in self.robots.values():
                if math.hypot(r.pose.x - x, r.pose.y - y) <= r.params.body_radius:
                    return r
            for b in reversed(self.bodies):
                if math.hypot(b.x - x, b.y - y) <= max(b.r, 0.04):
                    return b
            return None

    def persons(self) -> list[CircleBody]:
        return [b for b in self.bodies if b.kind == "person"]

    # ------------------------------------------------------------ 通信（仮想ロボットノードから呼ばれる）
    def deliver(self, robot_id: int, msg: m.Message) -> list[m.Message]:
        with self.lock:
            robot = self.robots.get(robot_id)
            return robot.firmware.handle_message(msg) if robot else []

    # ------------------------------------------------------------ 時間発展
    def step(self, dt: float = 1.0 / PHYSICS_HZ) -> None:
        with self.lock:
            self.t += dt
            self._fw_acc += dt
            self._sensor_acc += dt
            update_sensors = self._sensor_acc >= 1.0 / SENSOR_HZ
            if update_sensors:
                self._sensor_acc -= 1.0 / SENSOR_HZ
            fw_dt = 1.0 / CONTROL_HZ
            run_fw = self._fw_acc >= fw_dt
            if run_fw:
                self._fw_acc -= fw_dt

            robots = list(self.robots.values())
            for r in robots:
                if update_sensors or r.robot_id not in self._sensor_cache:
                    self._sensor_cache[r.robot_id] = self._ultrasonic(r, robots)
                if run_fw:
                    r.duty = r.firmware.tick(fw_dt, self._sensor_inputs(r))
                batt_v = r.battery.voltage(*r.duty)
                r.plant.step(*r.duty, batt_v=batt_v, dt=dt, move_pose=not r.dragging)
                r.battery.step(*r.duty, dt)

            for b in self.bodies:
                if b.kind == "person" and not b.dragging:
                    b.x += b.vx * dt
                    b.y += b.vy * dt
            self._resolve_collisions(robots)

    def _sensor_inputs(self, r: SimRobot) -> SensorInputs:
        el, er = r.plant.encoder_raw()
        fw_p = r.firmware.params
        us_f, us_r = self._sensor_cache.get(r.robot_id, (None, None))
        return SensorInputs(
            # モータ/エンコーダの取り付け向きは実機同様ファームのパラメータで打ち消す前提。シミュは正向き。
            enc_raw_l=el, enc_raw_r=er, imu=sensors.imu(r),
            batt_adc_v=r.battery.voltage(*r.duty) / fw_p["batt_div_ratio"],
            us_front_m=us_f, us_rear_m=us_r, rssi=-45 - int(3 * r.rng.random()))

    def _ultrasonic(self, r: SimRobot, robots: list[SimRobot]) -> tuple[float | None, float | None]:
        p = r.params
        circles = list(sensors.obstacles_for(r, robots, self.bodies))
        args = (p.us_offset, p.us_fov_deg, p.us_max_range, circles, self.field_wh, r.rng)
        front = sensors.ultrasonic(r.pose, args[0], 0.0, *args[1:])
        rear = sensors.ultrasonic(r.pose, args[0], math.pi, *args[1:])
        return front, rear

    def _resolve_collisions(self, robots: list[SimRobot]) -> None:
        w, h = self.field_wh
        # ロボット同士: 重なりを半分ずつ押し戻す
        for i, a in enumerate(robots):
            for b in robots[i + 1:]:
                self._separate_robots(a, b)
        # ロボットと物体
        for r in robots:
            for body in self.bodies:
                dx, dy = r.pose.x - body.x, r.pose.y - body.y
                d = math.hypot(dx, dy)
                overlap = r.params.body_radius + body.r - d
                if overlap > 0 and d > 1e-9:
                    nx, ny = dx / d, dy / d
                    if body.movable and not body.dragging:      # 物資は押される
                        body.x -= nx * overlap
                        body.y -= ny * overlap
                    elif not r.dragging:
                        r.pose = Pose2D(r.pose.x + nx * overlap, r.pose.y + ny * overlap, r.pose.th)
        # 壁
        for r in robots:
            rad = r.params.body_radius
            x = min(max(r.pose.x, rad), w - rad)
            y = min(max(r.pose.y, rad), h - rad)
            if (x, y) != (r.pose.x, r.pose.y) and not r.dragging:
                r.pose = Pose2D(x, y, r.pose.th)
        for b in self.bodies:
            b.x = min(max(b.x, b.r), w - b.r)
            b.y = min(max(b.y, b.r), h - b.r)

    @staticmethod
    def _separate_robots(a: SimRobot, b: SimRobot) -> None:
        dx, dy = b.pose.x - a.pose.x, b.pose.y - a.pose.y
        d = math.hypot(dx, dy)
        overlap = a.params.body_radius + b.params.body_radius - d
        if overlap <= 0 or d < 1e-9:
            return
        nx, ny = dx / d, dy / d
        ka = 0.0 if a.dragging else (1.0 if b.dragging else 0.5)
        kb = 0.0 if b.dragging else (1.0 if a.dragging else 0.5)
        a.pose = Pose2D(a.pose.x - nx * overlap * ka, a.pose.y - ny * overlap * ka, a.pose.th)
        b.pose = Pose2D(b.pose.x + nx * overlap * kb, b.pose.y + ny * overlap * kb, b.pose.th)
