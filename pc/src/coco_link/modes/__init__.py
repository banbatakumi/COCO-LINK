"""上位制御モード（プラグイン）. このディレクトリにファイルを置き @register_mode すると GUI に出る."""

from .base import Mode, RobotCommand, Visualization, param
from .registry import MODES, discover, register_mode

__all__ = ["MODES", "Mode", "RobotCommand", "Visualization", "discover", "param", "register_mode"]
