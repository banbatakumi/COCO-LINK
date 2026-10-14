"""学習資料の図を、実際の coco_link のモデル・制御則を動かして生成する.

コード（モデル・制御則・同定）を変えたら、このスクリプトを再実行して図を作り直し、同じ PR でコミットすること。

準備（リポジトリ直下で）:
    pip install -e "pc[dev]"
    pip install -r docs/requirements.txt
実行:
    python docs/study/figures/gen_figures.py            # 全部
    python docs/study/figures/gen_figures.py formation  # 名前で絞る
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT = Path(__file__).resolve().parent
PC = OUT.parents[2] / "pc"
sys.path.insert(0, str(PC / "src"))
sys.path.insert(0, str(PC))

from coco_link.common.geometry import Pose2D  # noqa: E402
from coco_link.protocol import messages as m  # noqa: E402
from coco_link.robot.params import RobotParams  # noqa: E402
from coco_link.robot.plant import deadzone  # noqa: E402
from coco_link.sim.engine import PHYSICS_HZ, SimConfig, SimEngine  # noqa: E402

# 日本語フォント（OS ごとに使えるもののうち最初に見つかったもの）
_available = {f.name for f in matplotlib.font_manager.fontManager.ttflist}
plt.rcParams["font.family"] = next(
    (f for f in ("Noto Sans CJK JP", "Hiragino Sans", "Yu Gothic", "Meiryo", "IPAexGothic") if f in _available),
    "DejaVu Sans")
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3
plt.rcParams["figure.dpi"] = 110

C = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#17becf"]


def save(fig, name: str) -> None:
    fig.tight_layout()
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)
    print(f"  {name}.png")


# ---------------------------------------------------------------- 制御の基礎
def first_order() -> None:
    """一次遅れ系 K/(τs+1) のステップ応答と時定数."""
    t = np.linspace(0, 0.5, 500)
    K = 1.0
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for tau, c in ((0.03, C[2]), (0.06, C[0]), (0.12, C[3])):
        ax.plot(t, K * (1 - np.exp(-t / tau)), color=c, lw=2, label=f"τ = {tau * 1000:.0f} ms")
    tau = 0.06
    ax.axhline(0.632, color="gray", ls="--", lw=1)
    ax.axvline(tau, color=C[0], ls=":", lw=1.5)
    ax.annotate("63.2 %", (0.4, 0.632), xytext=(0.4, 0.5), arrowprops={"arrowstyle": "->"})
    ax.annotate("t = τ のとき\n最終値の 63 %", (tau, 0.632), xytext=(0.13, 0.25), arrowprops={"arrowstyle": "->"})
    ax.set_xlabel("時間 t [s]")
    ax.set_ylabel("出力 / (K × 入力)")
    ax.set_title("一次遅れ系のステップ応答（τ が小さいほど速い）")
    ax.legend(loc="lower right")
    save(fig, "first_order")


def deadzone_fig() -> None:
    """モータの静特性（不感帯と飽和）."""
    p = RobotParams()
    u = np.linspace(-1, 1, 401)
    w = np.array([p.motor_gain_l * deadzone(x, p.deadzone_l) for x in u])
    fig, ax = plt.subplots(figsize=(7, 3.8))
    ax.plot(u, w, color=C[0], lw=2.5, label="モデル: ω = K (|u| − u₀) sign(u)")
    ax.plot(u, p.motor_gain_l * u, color="gray", ls="--", lw=1, label="不感帯がない場合 ω = K u")
    ax.axvspan(-p.deadzone_l, p.deadzone_l, color=C[1], alpha=0.12, label=f"不感帯 |u| < u₀ = {p.deadzone_l}")
    ax.set_xlabel("duty u（PWM の割合, −1 〜 1）")
    ax.set_ylabel("定常の車輪角速度 ω [rad/s]")
    ax.set_title("モータの静特性: 小さい duty では回らない（不感帯）")
    ax.legend(loc="upper left", fontsize=9)
    save(fig, "deadzone")


def wheel_control() -> None:
    """車輪速度制御: フィードフォワードとフィードバックの役割の比較（ファーム参照実装 + 物理モデル）."""
    cases = [
        ("FF のみ（kp = ki = 0）", {"kp": 0.0, "ki": 0.0}),
        ("PI のみ（FF・不感帯補償なし）", {"kff": 0.0, "tau_ff": 0.0, "u_deadzone": 0.0}),
        ("FF + PI（実装）", {}),
    ]
    fig, ax = plt.subplots(figsize=(7.5, 4))
    target_plotted = False
    for (label, prm), c in zip(cases, (C[3], C[2], C[0]), strict=True):
        e = SimEngine(SimConfig(param_spread=0.0))
        r = e.add_robot(1, Pose2D(1.5, 1.0, 0.0))
        # 実機との差を模擬するため、真のゲインを公称より 15 % 小さくする（FF だけでは届かない状況）
        r.plant.params.motor_gain_l *= 0.85
        r.plant.params.motor_gain_r *= 0.85
        for k, v in prm.items():
            r.firmware.set_param(k, v)
        for _ in range(20):
            e.step()
        ts, ws, refs = [], [], []
        for i in range(int(1.2 * PHYSICS_HZ)):
            if i % 10 == 0:
                e.deliver(1, m.CmdVel(vx=0.15, wz=0.0))
            e.step()
            ts.append(i / PHYSICS_HZ)
            ws.append(r.plant.omega_l)
            refs.append(r.firmware._w_target[0])
        if not target_plotted:
            ax.plot(ts, refs, color="k", ls="--", lw=1.2, label="目標（加速度制限つき）")
            target_plotted = True
        ax.plot(ts, ws, color=c, lw=2, label=label)
    ax.set_xlabel("時間 [s]")
    ax.set_ylabel("左車輪の角速度 [rad/s]")
    ax.set_title("車輪速度制御: FF で素早く、PI で誤差をなくす")
    ax.legend(loc="lower right", fontsize=9)
    save(fig, "wheel_control")


# ---------------------------------------------------------------- 同定
def sysid() -> None:
    """シミュレータの仮想ロボットで同定を行い、当てはめ結果を描く."""
    from tests.test_sysid import run_offline

    e = SimEngine(SimConfig(seed=11, param_spread=0.15))
    e.add_robot(1, Pose2D(1.5, 1.0, 0.0))
    ctx, res = run_offline(e, 1, ["deadzone", "step"])
    truth = e.robots[1].params

    dz = res["deadzone"].plots[0]
    fig, ax = plt.subplots(figsize=(7, 3.8))
    name, xs, ys = dz.series[0]
    ax.plot(xs, ys, ".", ms=3, color=C[0], alpha=0.5, label="実測（左輪）")
    name, xs, ys = dz.series[1]
    ax.plot(xs, ys, color=C[1], lw=2, label="最小二乗法で当てはめた直線")
    u0 = ctx.params.deadzone_l
    ax.axvline(u0, color="gray", ls=":")
    ax.annotate(f"推定 u₀ = {u0:.3f}\n（真値 {truth.deadzone_l:.3f}）", (u0, 0), xytext=(u0 + 0.08, 4),
                arrowprops={"arrowstyle": "->"})
    ax.set_xlabel("|duty|")
    ax.set_ylabel("|ω|·V_nom/V [rad/s]")
    ax.set_title("試験2: duty をゆっくり増やす → 直線の切片から不感帯")
    ax.legend(loc="upper left", fontsize=9)
    save(fig, "sysid_deadzone")

    st = res["step"].plots[0]
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    _, t, wm = st.series[0]
    _, t2, wmod = st.series[1]
    ax.plot(t, wm, ".", ms=3, color=C[0], label="実測（エンコーダ角の差分 Δθ/Δt）")
    ax.plot(t2, wmod, color=C[1], lw=1.8, label="同定したモデル K/(τs+1) のシミュレーション")
    ax.set_xlabel("時間 [s]")
    ax.set_ylabel("左車輪 ω [rad/s]")
    ax.set_title(f"試験3: ステップ応答 → 推定 K = {ctx.params.motor_gain_l:.1f}（真値 {truth.motor_gain_l:.1f}）, "
                 f"τ = {ctx.params.motor_tau_l * 1000:.0f} ms（真値 {truth.motor_tau_l * 1000:.0f} ms）", fontsize=10)
    ax.legend(loc="lower left", fontsize=9)
    save(fig, "sysid_step")


# ---------------------------------------------------------------- 上位制御
def formation() -> None:
    from coco_link.modes.formation import FormationMode
    from coco_link.sim.scenario import load_scenario
    from tests.helpers import run_closed_loop

    e = SimEngine(SimConfig(seed=1, param_spread=0.10))
    load_scenario(e, PC / "scenarios" / "demo_formation.yaml")
    e.bodies.clear()
    paths: dict[int, list[tuple[float, float]]] = {rid: [] for rid in e.robots}
    starts = {rid: r.pose for rid, r in e.robots.items()}

    def rec(engine, t):
        if int(t * PHYSICS_HZ) % 10 == 0:
            for rid, r in engine.robots.items():
                paths[rid].append((r.pose.x, r.pose.y))

    mode = FormationMode(FormationMode.Params(shape="circle", spacing=0.3, center_x=1.5, center_y=1.0))
    run_closed_loop(e, mode, 20.0, on_step=rec)
    fig, ax = plt.subplots(figsize=(7.5, 5))
    for i, (rid, pts) in enumerate(sorted(paths.items())):
        xs, ys = zip(*pts, strict=True)
        c = C[i % len(C)]
        ax.plot(xs, ys, color=c, lw=1.8, label=f"ロボット #{rid}")
        s = starts[rid]
        ax.plot(s.x, s.y, "o", color=c, mfc="white", ms=8)
        ax.arrow(s.x, s.y, 0.12 * math.cos(s.th), 0.12 * math.sin(s.th), color=c, width=0.008)
        tgt = mode.viz.targets[rid]
        ax.plot(tgt.x, tgt.y, "*", color=c, ms=14, mec="k")
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 2)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("フォーメーション（円）: ○ 開始位置 → ★ 割り当てられたスロット（20 秒間の軌跡）", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    save(fig, "formation")


def follow() -> None:
    from coco_link.modes.follow_person import FollowPersonMode
    from coco_link.sim.scenario import load_scenario
    from tests.helpers import run_closed_loop

    e = SimEngine(SimConfig(seed=3, param_spread=0.10))
    load_scenario(e, PC / "scenarios" / "demo_follow.yaml")
    e.bodies = [b for b in e.bodies if b.kind == "person"]
    person = e.persons()[0]
    paths: dict[int | str, list[tuple[float, float]]] = {rid: [] for rid in e.robots}
    paths["person"] = []

    def walk(engine, t):   # 人が右へ歩き、上へ曲がり、左へ戻る
        person.vx, person.vy = ((0.15, 0.0) if t < 6 else (0.0, 0.12) if t < 10 else (-0.15, 0.0) if t < 15
                                else (0.0, 0.0))
        if int(t * PHYSICS_HZ) % 10 == 0:
            for rid, r in engine.robots.items():
                paths[rid].append((r.pose.x, r.pose.y))
            paths["person"].append((person.x, person.y))

    run_closed_loop(e, FollowPersonMode(), 20.0, on_step=walk)
    fig, ax = plt.subplots(figsize=(7.5, 5))
    xs, ys = zip(*paths.pop("person"), strict=True)
    ax.plot(xs, ys, color="g", lw=5, alpha=0.35, label="人（緑）の歩いた道")
    ax.plot(xs[-1], ys[-1], "o", color="g", ms=16, alpha=0.6)
    for i, (rid, pts) in enumerate(sorted(paths.items())):
        px, py = zip(*pts, strict=True)
        ax.plot(px, py, color=C[i], lw=1.6, label=f"ロボット #{rid}")
        ax.plot(px[-1], py[-1], "o", color=C[i], ms=9, mec="k")
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 2)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("人追従（隊列）: 全員が人の通った道をたどる", fontsize=10)
    ax.legend(loc="lower right", fontsize=8)
    save(fig, "follow")


def diff_drive() -> None:
    """差動2輪ロボットの記号の説明図（上から見た図, ロボットは上向き）."""
    fig, ax = plt.subplots(figsize=(6, 4.6))
    ax.set_aspect("equal")
    ax.axis("off")
    R, b = 1.0, 1.5
    ax.add_patch(plt.Circle((0, 0), R, fc="#e8eef8", ec="k", lw=1.5))
    for x, name, ha in ((-b / 2, "左車輪\n角速度 ω_L", "right"), (b / 2, "右車輪\n角速度 ω_R", "left")):
        ax.add_patch(plt.Rectangle((x - 0.09, -0.38), 0.18, 0.76, fc="#444"))
        ax.text(x + (-0.2 if ha == "right" else 0.2), 0.0, name, ha=ha, va="center", fontsize=10)
    ax.annotate("", xy=(0, 1.75), xytext=(0, 0), arrowprops={"arrowstyle": "-|>", "lw": 2.5, "color": C[1]})
    ax.text(0.08, 1.6, "v: 前進速度 [m/s]", color=C[1], fontsize=11)
    ax.annotate("", xy=(-0.55, 1.15), xytext=(0.55, 1.15),
                arrowprops={"arrowstyle": "-|>", "lw": 2, "color": C[2], "connectionstyle": "arc3,rad=0.5"})
    ax.text(-1.9, 1.3, "ω: 回転速度 [rad/s]\n（左回りが正）", color=C[2], fontsize=10)
    ax.annotate("", xy=(-b / 2, -0.75), xytext=(b / 2, -0.75), arrowprops={"arrowstyle": "<->"})
    ax.text(0, -0.95, "b: トレッド幅（左右の車輪の間隔）", ha="center", fontsize=10)
    ax.annotate("", xy=(b / 2 + 0.25, -0.38), xytext=(b / 2 + 0.25, 0.0), arrowprops={"arrowstyle": "<->"})
    ax.text(b / 2 + 0.32, -0.25, "r: 車輪半径", fontsize=9)
    ax.plot(0, 0, "k+", ms=10)
    ax.text(0.05, -0.2, "中心", fontsize=9)
    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-1.15, 1.95)
    ax.set_title("差動2輪ロボット（上から見た図）")
    save(fig, "diff_drive")


def parallax() -> None:
    """視差補正の説明図（横から見た図）."""
    H, h, x_true = 2.0, 0.08, 1.5
    x_seen = x_true * H / (H - h)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot([-0.2, 2.0], [0, 0], color="k", lw=2)
    ax.text(1.95, -0.12, "床", ha="right")
    ax.plot(0, H, "s", color="k", ms=12)
    ax.text(0.06, H + 0.02, "カメラ（高さ H = 2.0 m）", va="bottom")
    ax.plot([0, 0], [0, H], color="gray", ls=":")
    ax.text(0.03, 0.05, "カメラ真下 c", fontsize=9)
    ax.plot([0, x_seen], [H, 0], color=C[0], lw=1.2, ls="--")
    ax.add_patch(plt.Rectangle((x_true - 0.08, 0), 0.16, h, fc="#bbb", ec="k"))
    ax.plot(x_true, h, "o", color=C[1], ms=8)
    ax.text(x_true - 0.1, h + 0.12, "マーカー（高さ h = 0.08 m）\n本当の位置", color=C[1], ha="right", fontsize=9)
    ax.plot(x_seen, 0, "x", color=C[0], ms=12, mew=3)
    ax.text(x_seen + 0.03, 0.12, f"床面の変換では\nここに見える\n（{(x_seen - x_true) * 100:.0f} cm 外側）",
            color=C[0], fontsize=9)
    ax.set_xlim(-0.2, 2.1)
    ax.set_ylim(-0.2, 2.3)
    ax.set_xlabel("カメラ真下からの水平距離 [m]")
    ax.set_yticks([])
    ax.grid(False)
    ax.set_title("視差: 床より高いものは、床面ホモグラフィでは外側にずれて見える")
    save(fig, "parallax")


FIGURES = {"first_order": first_order, "deadzone": deadzone_fig, "wheel_control": wheel_control,
           "sysid": sysid, "formation": formation, "follow": follow, "diff_drive": diff_drive,
           "parallax": parallax}


if __name__ == "__main__":
    names = sys.argv[1:] or list(FIGURES)
    for n in names:
        FIGURES[n]()
