"""同定の推定アルゴリズム（数値計算のみ. 通信・GUI に依存しない）.

モデル（robot/plant.py と同じ構造）:
    u_eff = sign(u) (|u| - u0) · V/V_nom
    τ dω/dt + ω = K u_eff
詳細は docs/system_identification.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares


def r_squared(y: np.ndarray, y_hat: np.ndarray) -> float:
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0


def effective_input(u: np.ndarray, u0: float, batt_v: np.ndarray, v_nom: float) -> np.ndarray:
    """不感帯と電池電圧を考慮した実効入力."""
    mag = np.maximum(np.abs(u) - u0, 0.0)
    return np.sign(u) * mag * (batt_v / v_nom)


# ---------------------------------------------------------------- ジャイロバイアス
def gyro_bias(gz: np.ndarray) -> tuple[float, float]:
    """静止中のジャイロ出力の平均（バイアス）と標準偏差（ノイズ）."""
    if gz.size == 0:
        raise ValueError("no gyro samples")
    return float(np.mean(gz)), float(np.std(gz))


# ---------------------------------------------------------------- 不感帯と静的ゲイン
@dataclass
class StaticFit:
    K: float          # [rad/s per duty]
    u0: float         # 不感帯 [duty]
    r2: float
    n: int


def static_gain_deadzone(u: np.ndarray, w: np.ndarray, batt_v: np.ndarray, v_nom: float,
                         w_min: float = 1.0) -> StaticFit:
    """ゆっくりした duty ランプ中の (u, ω) から |ω|·V_nom/V = K(|u| - u0) を直線回帰.

    ランプが十分遅ければ ω は準静的に定常値に追従するので、動特性を無視できる。
    回り始め付近 (|ω| < w_min) は静止摩擦の影響が大きいので除く。
    """
    w_n = np.abs(w) * v_nom / batt_v
    mask = (w_n > w_min) & (np.sign(u) == np.sign(w))
    if mask.sum() < 5:
        raise ValueError("ランプ中に車輪が回転しませんでした（max_duty を上げてください）")
    x, y = np.abs(u[mask]), w_n[mask]
    A = np.column_stack([x, np.ones_like(x)])
    (slope, icpt), *_ = np.linalg.lstsq(A, y, rcond=None)
    K = float(slope)
    u0 = float(-icpt / slope) if slope > 0 else float("nan")
    return StaticFit(K=K, u0=u0, r2=r_squared(y, A @ np.array([slope, icpt])), n=int(mask.sum()))


# ---------------------------------------------------------------- 一次遅れ (K, τ)
@dataclass
class FirstOrderFit:
    K: float
    tau: float
    r2: float
    arx_K: float          # 線形最小二乗 (ARX) による推定（初期値・比較用）
    arx_tau: float
    t: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    w_meas: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    w_model: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))


def _interval_mean_speed(K: float, tau: float, ueff: np.ndarray, dt: np.ndarray) -> np.ndarray:
    """一次遅れモデルを ZOH 入力でシミュレーションし、各サンプル区間の平均角速度を返す."""
    w = 0.0
    out = np.empty_like(ueff)
    for k in range(len(ueff)):
        a = math.exp(-dt[k] / tau)
        target = K * ueff[k]
        # 区間平均 = target + (w - target)·τ/Δt·(1 - a)  （指数関数の積分）
        out[k] = target + (w - target) * tau / dt[k] * (1.0 - a)
        w = target + (w - target) * a
    return out


def first_order(t: np.ndarray, theta: np.ndarray, u: np.ndarray, batt_v: np.ndarray, v_nom: float,
                u0: float) -> FirstOrderFit:
    """ステップ応答から K, τ を推定する.

    ファームの角速度推定値 (wl, wr) はローパスフィルタの遅れを含むので使わず、
    エンコーダ角の差分 Δθ/Δt（区間平均速度）を観測値とする。
    1. ARX モデル  ω̄[k+1] = a ω̄[k] + b u_eff[k]  を線形最小二乗 → τ = -Δt/ln a, K = b/(1-a)
    2. それを初期値に、区間平均速度の予測誤差を非線形最小二乗 (scipy.least_squares) で最小化
    """
    dt = np.diff(t)
    ok = dt > 1e-4
    w_meas = np.diff(theta)[ok] / dt[ok]
    ueff = effective_input(u[:-1], u0, batt_v[:-1], v_nom)[ok]
    dt = dt[ok]
    t_mid = (t[:-1][ok] + t[1:][ok]) / 2
    if len(w_meas) < 20:
        raise ValueError("サンプルが不足しています")
    # 1. ARX
    A = np.column_stack([w_meas[:-1], ueff[:-1]])
    (a, b), *_ = np.linalg.lstsq(A, w_meas[1:], rcond=None)
    dt_mean = float(np.mean(dt))
    if 0 < a < 1:
        arx_tau = -dt_mean / math.log(a)
        arx_K = b / (1 - a)
    else:
        arx_tau, arx_K = 0.05, float(np.max(np.abs(w_meas)) / max(np.max(np.abs(ueff)), 1e-3))
    # 2. 非線形最小二乗
    x0 = [max(arx_K, 1.0), min(max(arx_tau, 0.01), 0.5)]

    def resid(x):
        return _interval_mean_speed(x[0], x[1], ueff, dt) - w_meas

    sol = least_squares(resid, x0, bounds=([0.5, 0.003], [500.0, 2.0]))
    K, tau = float(sol.x[0]), float(sol.x[1])
    w_model = _interval_mean_speed(K, tau, ueff, dt)
    return FirstOrderFit(K=K, tau=tau, r2=r_squared(w_meas, w_model), arx_K=float(arx_K), arx_tau=float(arx_tau),
                         t=t_mid - t_mid[0], w_meas=w_meas, w_model=w_model)


# ---------------------------------------------------------------- 幾何パラメータ
def wheel_radius(distances: list[float], wheel_angles: list[float]) -> float:
    """直進: ビジョンで測った移動距離 d と車輪回転角 Δθ から r = Σd / Σ|Δθ|."""
    s = sum(abs(a) for a in wheel_angles)
    if s < 1e-3:
        raise ValueError("車輪がほとんど回転していません")
    return sum(distances) / s


def tread(r: float, diff_angles: list[float], yaw_changes: list[float]) -> float:
    """旋回: Δψ = r (Δθ_R - Δθ_L) / b より b = r Σ|Δθ_R - Δθ_L| / Σ|Δψ|."""
    s = sum(abs(y) for y in yaw_changes)
    if s < 1e-3:
        raise ValueError("ほとんど旋回していません")
    return r * sum(abs(d) for d in diff_angles) / s


def unwrap_sum(angles: np.ndarray) -> float:
    """角度列の累積変化量（±π の折り返しを展開）."""
    if len(angles) < 2:
        return 0.0
    return float(np.sum(np.angle(np.exp(1j * np.diff(angles)))))


# ---------------------------------------------------------------- 制御器設計
def imc_pi(K: float, tau: float, lam: float) -> tuple[float, float, float]:
    """IMC (内部モデル制御) 法による PI ゲイン.

    G(s) = K/(τs+1) に対し、閉ループを 1/(λs+1) にする PI 制御器は
        C(s) = (τs+1)/(Kλs)  →  kp = τ/(Kλ),  ki = 1/(Kλ)
    フィードフォワード kff = 1/K（定常時に必要な duty を直接与える）。
    """
    return tau / (K * lam), 1.0 / (K * lam), 1.0 / K


@dataclass
class StepMetrics:
    rise_time: float      # 10 → 90 % [s]
    overshoot: float      # [%]
    steady_error: float   # [%]


def step_metrics(t: np.ndarray, y: np.ndarray, target: float) -> StepMetrics:
    if len(t) < 5 or target == 0:
        return StepMetrics(float("nan"), float("nan"), float("nan"))
    y = y * np.sign(target)
    target = abs(target)
    t = t - t[0]
    try:
        t10 = t[np.argmax(y >= 0.1 * target)]
        t90 = t[np.argmax(y >= 0.9 * target)]
        rise = float(t90 - t10) if np.any(y >= 0.9 * target) else float("nan")
    except ValueError:
        rise = float("nan")
    final = float(np.mean(y[int(len(y) * 0.7):]))
    return StepMetrics(rise_time=rise, overshoot=max(0.0, (float(np.max(y)) - target) / target * 100.0),
                       steady_error=(final - target) / target * 100.0)
