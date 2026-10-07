"""上位制御モードの基底クラス. 追加方法は docs/adding_a_mode.md.

モードは「World（世界のスナップショット）を受け取り、各ロボットへの RobotCommand を返す関数」である。
通信・スレッド・GUI のことは考えなくてよい（ControlCore が面倒を見る）。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar

from ..common.geometry import Pose2D
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

    def on_start(self, world: World) -> None:  # noqa: B027  (任意実装)
        """開始時に 1 回呼ばれる."""

    @abstractmethod
    def step(self, world: World, dt: float) -> dict[int, RobotCommand]:
        """制御周期ごとに呼ばれる. 返さなかったロボットには何も送られない（=ウォッチドッグで停止）."""

    def on_stop(self, world: World) -> dict[int, RobotCommand]:
        """停止時に 1 回呼ばれる. 既定では全ロボットを停止."""
        return {rid: RobotCommand() for rid in world.usable_robots()}

    def on_field_click(self, x: float, y: float) -> None:  # noqa: B027
        """フィールドビューがクリックされたとき（目標位置の指定などに使う）."""

    def update_params(self, params: Any) -> None:
        """実行中のパラメータ変更."""
        self.params = params
