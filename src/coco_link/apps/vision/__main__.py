"""coco-vision: ビジョンシステム.

例:
    coco-vision --camera 0
    coco-vision --video record.mp4
    coco-vision --synthetic          # カメラが無くても合成映像で動作確認
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ...common.config import FieldConfig, NetworkConfig
from ...vision.calibration import FieldCalibration
from ...vision.pipeline import VisionPipeline
from ...vision.publisher import VisionPublisher
from ...vision.worker import VisionWorker


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog="coco-vision", description="COCO-LINK ビジョン")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, help="カメラ番号")
    src.add_argument("--video", help="動画ファイル")
    src.add_argument("--synthetic", action="store_true", help="合成映像（既定）")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--operator-host", help="operator の IP（既定は config/network.yaml）")
    ap.add_argument("--no-send", action="store_true", help="operator へ送らない")
    ap.add_argument("--quit-after", type=float, help="指定秒数後に終了（テスト用）")
    ap.add_argument("--screenshot", type=Path, help="終了時にウィンドウを PNG 保存（テスト用）")
    return ap.parse_args(argv)


def make_source(args):
    from ...vision import sources
    if args.camera is not None:
        return sources.CameraSource(args.camera, args.width, args.height)
    if args.video:
        return sources.VideoSource(args.video)
    return sources.SyntheticSource()


def main(argv=None) -> int:
    args = parse_args(argv)
    net = NetworkConfig.load()
    if args.operator_host:
        net.operator_host = args.operator_host
    field = FieldConfig.load()
    calib = FieldCalibration.load()
    pipeline = VisionPipeline(field, calib, marker_height=0.0 if args.camera is None and not args.video else 0.08)
    worker = VisionWorker(make_source(args), pipeline, None if args.no_send else VisionPublisher(net)).start()

    from ..common.app import create_app, run_for
    from .main_window import VisionWindow

    app = create_app("COCO-LINK Vision")
    win = VisionWindow(worker)
    win.show()
    shot = (lambda: win.grab().save(str(args.screenshot))) if args.screenshot else None
    code = run_for(app, args.quit_after, shot)
    worker.stop()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
