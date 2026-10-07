"""ESP32 ファームウェアの参照実装（Python 版）.

docs/firmware_spec.md の状態機械・車輪速度制御・メッセージ処理をそのまま実装したもの。
シミュレータの仮想ロボットはこのクラスを動かしている。ESP32 の C++ 実装はこれと同じ挙動にすること。

ハードウェアには依存しない。センサ値は `SensorInputs` として外から与え、
`tick()` がモータ duty を返す（実機ではこれを PWM に出す）。
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass, field

from ..protocol import messages as m
from .kinematics import body_to_wheel, limit_wheel_speeds

FW_VERSION = "ref-1.0"
CONTROL_HZ = 100.0          # ctrl_task 周期 (spec §2.3)
ENC_COUNTS = 4096           # AS5600 12bit
VEL_LPF_ALPHA = 0.3         # 角速度推定の一次ローパス (spec §5.1)
STOP_W_EPS = 0.05           # これ未満の目標角速度は停止扱い [rad/s] (spec §5.3)
LOW_BATT_HOLD_S = 2.0       # spec §4
LOW_BATT_HYST_V = 0.2

# 既定パラメータ (docs/protocol.md §7)
DEFAULT_PARAMS: dict[str, float] = {
    "robot_id": 1,
    "wheel_radius": 0.021,
    "tread": 0.090,
    "max_v": 0.40,
    "max_w": 4.0,
    "accel_limit": 1.0,
    "kp": 0.02,
    "ki": 0.4,
    "kff": 0.04,
    "tau_ff": 0.06,
    "u_deadzone": 0.10,
    "watchdog_ms": 300,
    "telemetry_hz": 50,
    "batt_nominal_v": 4.8,
    "batt_low_v": 4.2,
    "batt_div_ratio": 2.0,
    "motor_dir_l": 1,
    "motor_dir_r": 1,
    "enc_dir_l": 1,
    "enc_dir_r": 1,
}

# 値域チェック (spec §8)
PARAM_RANGES: dict[str, tuple[float, float]] = {
    "robot_id": (0, 29),
    "wheel_radius": (0.005, 0.2),
    "tread": (0.02, 0.5),
    "max_v": (0.0, 2.0),
    "max_w": (0.0, 20.0),
    "accel_limit": (0.05, 20.0),
    "kp": (0.0, 1.0),
    "ki": (0.0, 20.0),
    "kff": (0.0, 1.0),
    "tau_ff": (0.0, 0.5),
    "u_deadzone": (0.0, 0.6),
    "watchdog_ms": (50, 5000),
    "telemetry_hz": (1, 100),
    "batt_nominal_v": (2.0, 12.0),
    "batt_low_v": (2.0, 12.0),
    "batt_div_ratio": (1.0, 10.0),
    "motor_dir_l": (-1, 1),
    "motor_dir_r": (-1, 1),
    "enc_dir_l": (-1, 1),
    "enc_dir_r": (-1, 1),
}

MELODIES: dict[str, list[tuple[float, int]]] = {  # (周波数 Hz, ms)。0 Hz は休符 (spec §6.5)
    "startup": [(523.3, 100), (659.3, 100), (784.0, 150)],
    "ok": [(784.0, 80), (1046.5, 120)],
    "error": [(261.6, 200), (0.0, 50), (261.6, 200)],
    "chime": [(659.3, 150), (523.3, 300)],
}


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class SensorInputs:
    """ハードウェアから読んだ生値（シミュレータでは sensors.py が生成）."""

    enc_raw_l: int = 0                 # AS5600 RAW ANGLE 0..4095（モータ取り付け向きそのまま）
    enc_raw_r: int = 0
    imu: m.Imu = field(default_factory=m.Imu)
    batt_adc_v: float = 2.4            # 分圧後の ADC 電圧 [V]
    us_front_m: float | None = None
    us_rear_m: float | None = None
    rssi: int = -50
    faults: list[str] = field(default_factory=list)


class EncoderUnwrapper:
    """12bit 絶対角を累積角へ (spec §5.1)."""

    def __init__(self) -> None:
        self.prev: int | None = None
        self.count = 0

    def update(self, raw: int) -> int:
        if self.prev is not None:
            d = (raw - self.prev) % ENC_COUNTS
            if d >= ENC_COUNTS // 2:
                d -= ENC_COUNTS
            self.count += d
        self.prev = raw
        return self.count


class WheelController:
    """車輪速度 PI + フィードフォワード + 不感帯補償 + 電池電圧補償 (spec §5.3)."""

    def __init__(self) -> None:
        self.integral = 0.0
        self.ref_filt = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.ref_filt = 0.0

    def update(self, w_ref: float, w: float, dt: float, p: dict[str, float], batt_v: float,
               dw_ref: float = 0.0) -> float:
        """w_ref: 目標角速度, w: 推定角速度, dw_ref: 目標角加速度（加速度フィードフォワード用）."""
        # 目標値にも速度推定と同じローパスを通し、遅れを揃えてから偏差をとる。
        # 揃えないと、加速中に推定値の遅れぶんの偽の偏差が PI に入りオーバーシュートする (spec §5.3)
        self.ref_filt += VEL_LPF_ALPHA * (w_ref - self.ref_filt)
        if abs(w_ref) < STOP_W_EPS and abs(w) < 0.2:
            self.integral = 0.0
            self.ref_filt = 0.0
            return 0.0
        e = self.ref_filt - w
        # モデル τ dω/dt + ω = K u の逆系: u = (ω* + τ dω*/dt) / K  （2自由度制御のフィードフォワード）
        u_ff = p["kff"] * (w_ref + p["tau_ff"] * dw_ref)
        u_dz = math.copysign(p["u_deadzone"], w_ref) if abs(w_ref) >= STOP_W_EPS else 0.0
        u_unsat = u_ff + u_dz + p["kp"] * e + self.integral
        comp = p["batt_nominal_v"] / batt_v if batt_v > 1.0 else 1.0
        u = u_unsat * comp
        # アンチワインドアップ: 飽和していない、または積分が飽和を緩める方向のときだけ積分
        if abs(u) < 1.0 or (u > 0) != (e > 0):
            self.integral += p["ki"] * e * dt
        return clamp(u, -1.0, 1.0)


class FirmwareModel:
    """ESP32 ファームウェア 1 台分."""

    def __init__(self, robot_id: int, params: dict[str, float] | None = None, mac: str = "", fw: str = FW_VERSION):
        self.params = dict(DEFAULT_PARAMS)
        if params:
            self.params.update(params)
        self.params["robot_id"] = robot_id
        self.mac = mac or f"02:00:00:00:00:{robot_id:02x}"
        self.fw = fw
        self.state = m.STATE_BOOT
        self.faults: list[str] = []
        self.rx_count = 0
        self.t = 0.0                          # 起動からの時刻 [s]
        # 指令
        self._mode = "none"                   # "vel" / "wheel" / "none"
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._cmd_duty = (0.0, 0.0)
        self._last_cmd_t = -1e9
        # 車輪
        self._enc = (EncoderUnwrapper(), EncoderUnwrapper())
        self._theta = [0.0, 0.0]
        self._omega = [0.0, 0.0]
        self._w_target = [0.0, 0.0]           # 加速度制限後の目標
        self._ctrl = (WheelController(), WheelController())
        self.applied = (0.0, 0.0)
        # 電池
        self.batt_v = self.params["batt_nominal_v"]
        self._batt_filter: float | None = None
        self._low_since: float | None = None
        # LED / ブザー
        self.led_rgb = (0, 255, 0)
        self.led_pattern = "breath"
        self.led_period_ms = 2000
        self._tone_queue: list[tuple[float, float]] = []   # (周波数, 終了時刻)
        self.last_sensors = SensorInputs()
        self.loop_dt_us = int(1e6 / CONTROL_HZ)
        self.play_melody("startup")

    # ------------------------------------------------------------ 外部 API
    @property
    def robot_id(self) -> int:
        return int(self.params["robot_id"])

    def hello(self) -> m.Hello:
        return m.Hello(robot_id=self.robot_id, fw=self.fw, mac=self.mac,
                       caps=["led_rgb", "buzzer", "us_front", "us_rear", "imu", "enc"])

    def telemetry(self) -> m.Telemetry:
        s = self.last_sensors
        return m.Telemetry(
            robot_id=self.robot_id, state=self.state, faults=list(self.faults), batt_v=round(self.batt_v, 3),
            us_front_m=s.us_front_m, us_rear_m=s.us_rear_m, imu=s.imu,
            enc=m.Encoders(self._theta[0], self._theta[1], self._omega[0], self._omega[1]),
            applied=m.WheelDuty(*self.applied), rssi=s.rssi, loop_dt_us=self.loop_dt_us, rx_count=self.rx_count)

    def params_msg(self) -> m.Params:
        return m.Params(values=dict(self.params))

    def handle_message(self, msg: m.Message) -> list[m.Message]:
        """受信メッセージを処理し、返信メッセージのリストを返す (spec §4, §7)."""
        self.rx_count += 1
        replies: list[m.Message] = []
        st = self.state
        if isinstance(msg, m.Ping):
            replies.append(m.Pong(nonce=msg.nonce))
        elif isinstance(msg, m.GetParams):
            replies.append(self.params_msg())
        elif isinstance(msg, m.EStop):
            self._enter_estop()
        elif isinstance(msg, m.ClearEStop):
            if st == m.STATE_ESTOP:
                self.state = m.STATE_IDLE
                self._stop_motion()
        elif isinstance(msg, m.SetParam):
            if st in (m.STATE_IDLE, m.STATE_FAULT, m.STATE_RUNNING):
                self.set_param(msg.key, msg.value)
            replies.append(self.params_msg())
        elif isinstance(msg, m.SetLed):
            if st not in (m.STATE_LOW_BATT, m.STATE_FAULT):
                self.led_rgb = (int(clamp(msg.r, 0, 255)), int(clamp(msg.g, 0, 255)), int(clamp(msg.b, 0, 255)))
                self.led_pattern = msg.pattern if msg.pattern in m.LED_PATTERNS else "solid"
                self.led_period_ms = max(50, int(msg.period_ms))
        elif isinstance(msg, m.Buzzer):
            if st not in (m.STATE_LOW_BATT, m.STATE_FAULT):
                if msg.melody:
                    self.play_melody(msg.melody)
                else:
                    self.play_tone(msg.freq_hz, msg.dur_ms)
        elif isinstance(msg, m.CmdVel | m.CmdWheel | m.Stop):
            if st in (m.STATE_IDLE, m.STATE_RUNNING):
                if isinstance(msg, m.Stop):
                    self._mode = "vel"
                    self._cmd_vx = self._cmd_wz = 0.0
                    self.state = m.STATE_IDLE
                else:
                    if isinstance(msg, m.CmdVel):
                        self._mode = "vel"
                        self._cmd_vx, self._cmd_wz = msg.vx, msg.wz
                    else:
                        self._mode = "wheel"
                        self._cmd_duty = (clamp(msg.left, -1, 1), clamp(msg.right, -1, 1))
                    self._last_cmd_t = self.t
                    self.state = m.STATE_RUNNING
        return replies

    def set_param(self, key: str, value: float) -> bool:
        if key not in PARAM_RANGES:
            return False
        lo, hi = PARAM_RANGES[key]
        v = clamp(float(value), lo, hi)
        if key.startswith(("motor_dir", "enc_dir")):
            v = 1 if v >= 0 else -1
        if key == "robot_id":  # 再起動で反映 (spec §7)。参照実装ではそのまま保持のみ
            return True
        self.params[key] = v
        return True

    def tick(self, dt: float, sensors: SensorInputs) -> tuple[float, float]:
        """制御周期 (100 Hz) ごとに呼ぶ. 戻り値はモータ duty (左, 右)（motor_dir 適用後）."""
        p = self.params
        self.t += dt
        self.loop_dt_us = int(dt * 1e6)
        self.last_sensors = sensors
        self.faults = list(sensors.faults)
        # ---- 電池 (spec §6.3: 移動平均 ≒ 一次ローパスで代用)
        v = sensors.batt_adc_v * p["batt_div_ratio"]
        self._batt_filter = v if self._batt_filter is None else self._batt_filter + 0.1 * (v - self._batt_filter)
        self.batt_v = self._batt_filter
        # ---- エンコーダ (spec §5.1)
        for i, (raw, key) in enumerate(((sensors.enc_raw_l, "enc_dir_l"), (sensors.enc_raw_r, "enc_dir_r"))):
            cnt = self._enc[i].update(raw)
            th = p[key] * cnt * 2.0 * math.pi / ENC_COUNTS
            w_raw = (th - self._theta[i]) / dt if dt > 0 else 0.0
            self._omega[i] += VEL_LPF_ALPHA * (w_raw - self._omega[i])
            self._theta[i] = th
        # ---- 状態遷移
        self._update_state()
        # ---- 出力
        if self.state != m.STATE_RUNNING:
            if self.state != m.STATE_IDLE or self._mode == "wheel":
                self._w_target = [0.0, 0.0]
            out = self._velocity_control(0.0, 0.0, dt) if self.state == m.STATE_IDLE else (0.0, 0.0)
        elif self._mode == "wheel":
            out = self._cmd_duty
        else:
            out = self._velocity_control(self._cmd_vx, self._cmd_wz, dt)
        self.applied = (round(out[0], 4), round(out[1], 4))
        return out[0] * p["motor_dir_l"], out[1] * p["motor_dir_r"]

    # ------------------------------------------------------------ LED / ブザー
    def play_tone(self, freq_hz: float, dur_ms: float) -> None:
        self._tone_queue = [(freq_hz, self.t + dur_ms / 1000.0)] if freq_hz > 0 else []

    def play_melody(self, name: str) -> None:
        t = self.t
        self._tone_queue = []
        for f, ms in MELODIES.get(name, []):
            t += ms / 1000.0
            self._tone_queue.append((f, t))

    def buzzer_freq(self) -> float:
        """現在鳴っている周波数 (0 = 無音)."""
        while self._tone_queue and self._tone_queue[0][1] <= self.t:
            self._tone_queue.pop(0)
        return self._tone_queue[0][0] if self._tone_queue else 0.0

    def led_color(self) -> tuple[int, int, int]:
        """現在の LED 表示色（状態による強制表示を含む, spec §4 / §6.4）."""
        forced = {
            m.STATE_BOOT: ((255, 255, 255), "solid", 1000),
            m.STATE_ESTOP: ((255, 0, 0), "blink", 250),
            m.STATE_LOW_BATT: ((255, 120, 0), "blink", 1000),
            m.STATE_FAULT: ((160, 0, 255), "solid", 1000),
        }
        rgb, pattern, period = forced.get(self.state, (self.led_rgb, self.led_pattern, self.led_period_ms))
        phase = (self.t * 1000.0 / period) % 1.0
        if pattern == "blink":
            k = 1.0 if phase < 0.5 else 0.0
        elif pattern == "breath":
            k = 0.15 + 0.85 * (0.5 - 0.5 * math.cos(2 * math.pi * phase))
        elif pattern == "rainbow":
            r, g, b = colorsys.hsv_to_rgb(phase, 1.0, 1.0)
            return int(r * 255), int(g * 255), int(b * 255)
        else:
            k = 1.0
        return int(rgb[0] * k), int(rgb[1] * k), int(rgb[2] * k)

    # ------------------------------------------------------------ 内部
    def _enter_estop(self) -> None:
        self.state = m.STATE_ESTOP
        self._stop_motion()

    def _stop_motion(self) -> None:
        self._mode = "none"
        self._cmd_vx = self._cmd_wz = 0.0
        self._cmd_duty = (0.0, 0.0)
        self._w_target = [0.0, 0.0]
        for c in self._ctrl:
            c.reset()

    def _update_state(self) -> None:
        p = self.params
        if self.state == m.STATE_BOOT:
            self.state = m.STATE_FAULT if any(f.startswith(("enc", "motor")) for f in self.faults) else m.STATE_IDLE
            return
        # ウォッチドッグ (spec §4)
        if self.state == m.STATE_RUNNING and (self.t - self._last_cmd_t) * 1000.0 > p["watchdog_ms"]:
            self.state = m.STATE_IDLE
            self._mode = "vel"
            self._cmd_vx = self._cmd_wz = 0.0
        # 低電圧 (spec §4)
        if self.state in (m.STATE_IDLE, m.STATE_RUNNING):
            if self.batt_v < p["batt_low_v"]:
                if self._low_since is None:
                    self._low_since = self.t
                elif self.t - self._low_since >= LOW_BATT_HOLD_S:
                    self.state = m.STATE_LOW_BATT
                    self._stop_motion()
            else:
                self._low_since = None
        elif self.state == m.STATE_LOW_BATT and self.batt_v > p["batt_low_v"] + LOW_BATT_HYST_V:
            self.state = m.STATE_IDLE
            self._low_since = None

    def _velocity_control(self, vx: float, wz: float, dt: float) -> tuple[float, float]:
        p = self.params
        r, b = p["wheel_radius"], p["tread"]
        vx = clamp(vx, -p["max_v"], p["max_v"])
        wz = clamp(wz, -p["max_w"], p["max_w"])
        wl, wr = limit_wheel_speeds(*body_to_wheel(vx, wz, r, b), p["max_v"] / r)
        dmax = p["accel_limit"] / r * dt
        out = []
        for i, w_ref in enumerate((wl, wr)):
            step = clamp(w_ref - self._w_target[i], -dmax, dmax)
            self._w_target[i] += step
            out.append(self._ctrl[i].update(self._w_target[i], self._omega[i], dt, p, self.batt_v,
                                            dw_ref=step / dt if dt > 0 else 0.0))
        return out[0], out[1]
