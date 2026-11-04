"""上位制御モードの基底クラス. 追加方法は docs/adding_a_mode.md.

モードは「World（世界のスナップショット）を受け取り、各ロボットへの RobotCommand を返す関数」である。
通信・スレッド・GUI のことは考えなくてよい（ControlCore が面倒を見る）。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar

from ..common.geometry import Pose2D, Vec2
from ..control.navigation import Navigator
from ..fleet.world_model import World


@dataclass
class RobotCommand:
    """1 台のロボットへの 1 周期分の指令."""

    vx: float = 0.0                              # [m/s]
    wz: float = 0.0                              # [rad/s]
    wheel: tuple[float, float] | None = None     # 指定時は cmd_wheel（開ループ duty）を送る
    led: tuple[int, int, int, str] | None = None  # (r, g, b, pattern) 変化したときだけ送られる
    buzzer: str | float | None = None            # メロディ名 or 周波数[Hz]（一度だけ鳴らす）
    safety: bool = True                          # 超音波による速度制限を適用するか


@dataclass
class Visualization:
    """操作GUIのフィールドビューに描く補助情報."""

    targets: dict[int, Pose2D] = field(default_factory=dict)           # ロボットごとの目標姿勢
    paths: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    points: list[tuple[float, float, str]] = field(default_factory=list)  # (x, y, ラベル)


def param(default: Any, label: str = "", **meta: Any):
    """GUI 表示情報付きのパラメータ宣言.

    例: spacing: float = param(0.25, "間隔 [m]", min=0.1, max=1.0, step=0.05)
        shape: str = param("circle", "隊形", choices=("line", "circle"))
    """
    return field(default=default, metadata={"label": label, **meta})


class Mode(ABC):
    """全モードの基底. 派生クラスに @register_mode を付けると GUI に出る."""

    name: ClassVar[str] = "base"          # GUI 表示名
    description: ClassVar[str] = ""

    @dataclass
    class Params:
        pass

    def __init__(self, params: Any = None):
        self.params = params if params is not None else self.Params()
        self.viz = Visualization()
        self.status = ""
        self.nav = Navigator()

    def on_start(self, world: World) -> None:  # noqa: B027  (任意実装)
        """開始時に 1 回呼ばれる."""

    @abstractmethod
    def step(self, world: World, dt: float) -> dict[int, RobotCommand]:
        """制御周期ごとに呼ばれる. 返さなかったロボットには何も送られない（=ウォッチドッグで停止）."""

    def navigation_targets(self, world: World) -> dict[int, Vec2]:
        """経路計画で向かうロボットごとの目標点（任意実装）.

        返したロボットは、`step()` の速度指令の代わりに「障害物・人・物資・他のロボットを
        避ける経路」を計画して追従する。目標が近いときや経路が見つからないときは
        `step()` の指令に任せるので、最終姿勢合わせや物資押しはモード側で実装できる。
        """
        return {}

    def run(self, world: World, dt: float) -> dict[int, RobotCommand]:
        """制御周期の入口. `step()` の結果に共通の経路計画ナビゲーションを適用する."""
        cmds = self.step(world, dt)
        cmds = self.nav.apply(world, cmds, self.navigation_targets(world))
        for key in [k for k in self.viz.paths if k.startswith("計画経路")]:
            del self.viz.paths[key]
        for rid, path in self.nav.paths().items():
            self.viz.paths[f"計画経路 #{rid}"] = [(p.x, p.y) for p in path]
        return cmds

    def on_stop(self, world: World) -> dict[int, RobotCommand]:
        """停止時に 1 回呼ばれる. 既定では全ロボットを停止."""
        return {rid: RobotCommand() for rid in world.usable_robots()}

    def on_field_click(self, x: float, y: float) -> None:  # noqa: B027
        """フィールドビューがクリックされたとき（目標位置の指定などに使う）."""

    def update_params(self, params: Any) -> None:
        """実行中のパラメータ変更."""
        self.params = params
