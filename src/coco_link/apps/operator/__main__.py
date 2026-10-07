"""coco-operator: 操作GUI（上位制御）.

シミュレータでも実機でも同じように起動する:
    coco-operator
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ...common.config import NetworkConfig
from ...fleet.control_core import ControlCore


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog="coco-operator", description="COCO-LINK 操作GUI")
    ap.add_argument("--control-hz", type=float, help="制御周期 [Hz]")
    ap.add_argument("--mode", help="起動直後に開始するモード名")
    ap.add_argument("--quit-after", type=float, help="指定秒数後に終了（テスト用）")
    ap.add_argument("--screenshot", type=Path, help="終了時にウィンドウを PNG 保存（テスト用）")
    return ap.parse_args(argv)


def build_window(core: ControlCore):
    """メインウィンドウを組み立てる（同定・ログのタブもここで追加）."""
    from .main_window import OperatorWindow
    return OperatorWindow(core)


def main(argv=None) -> int:
    args = parse_args(argv)
    net = NetworkConfig.load()
    if args.control_hz:
        net.control_hz = args.control_hz
    from ..common.app import create_app, run_for

    app = create_app("COCO-LINK Operator")
    core = ControlCore(net).start()
    win = build_window(core)
    win.show()
    if args.mode:
        core.start_mode(args.mode)
    shot = (lambda: win.grab().save(str(args.screenshot))) if args.screenshot else None
    code = run_for(app, args.quit_after, shot)
    core.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
