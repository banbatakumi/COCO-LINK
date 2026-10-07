"""coco-sim: シミュレータ.

例:
    coco-sim --scenario scenarios/demo_formation.yaml
    coco-sim --robots 4 --seed 1
    coco-sim --headless --scenario scenarios/demo_follow.yaml   # GUI なし
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from ...common.config import NetworkConfig
from ...sim.engine import SimConfig, SimEngine
from ...sim.runner import SimRunner
from ...sim.scenario import load_scenario


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog="coco-sim", description="COCO-LINK シミュレータ")
    ap.add_argument("--scenario", type=Path, help="初期配置 YAML (scenarios/*.yaml)")
    ap.add_argument("--robots", type=int, default=0, help="シナリオに加えてランダム配置するロボット台数")
    ap.add_argument("--seed", type=int, default=0, help="乱数シード（個体差の再現用）")
    ap.add_argument("--spread", type=float, default=0.10, help="個体差の大きさ（0 で全台公称値）")
    ap.add_argument("--speed", type=float, default=1.0, help="実時間に対する速度倍率")
    ap.add_argument("--operator-host", help="operator の IP（既定は config/network.yaml）")
    ap.add_argument("--headless", action="store_true", help="GUI なしで実行")
    ap.add_argument("--quit-after", type=float, help="指定秒数後に終了（テスト用）")
    ap.add_argument("--screenshot", type=Path, help="終了時にウィンドウを PNG 保存（テスト用）")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    net = NetworkConfig.load()
    if args.operator_host:
        net.operator_host = args.operator_host
    engine = SimEngine(SimConfig(seed=args.seed, param_spread=args.spread))
    if args.scenario:
        load_scenario(engine, args.scenario)
    for _ in range(args.robots):
        engine.add_robot()
    runner = SimRunner(engine, net, speed=args.speed).start()

    if args.headless:
        print(f"simulator running headless: {len(engine.robots)} robots -> {net.operator_host}")
        t0 = time.monotonic()
        try:
            while args.quit_after is None or time.monotonic() - t0 < args.quit_after:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        runner.stop()
        return 0

    from ..common.app import create_app, run_for
    from .main_window import SimulatorWindow

    app = create_app("COCO-LINK Simulator")
    win = SimulatorWindow(engine, runner, args.scenario)
    win.show()
    shot = (lambda: win.grab().save(str(args.screenshot))) if args.screenshot else None
    code = run_for(app, args.quit_after, shot)
    runner.stop()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
