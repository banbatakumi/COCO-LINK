"""MCAP ログをトピックごとの CSV に変換する（Excel / MATLAB / pandas での解析用）.

例:
    python tools/mcap_to_csv.py logs/coco_20261007_120000.mcap            # logs/coco_..._csv/ に出力
    python tools/mcap_to_csv.py logs/xxx.mcap --topics /robot/1/telemetry
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from coco_link.datalog import read_mcap  # noqa: E402


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        elif isinstance(v, list):
            if v and isinstance(v[0], dict):
                for i, item in enumerate(v):
                    out.update(flatten(item, f"{key}[{i}]."))
            else:
                out[key] = ";".join(map(str, v))
        else:
            out[key] = v
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--topics", nargs="*")
    args = ap.parse_args(argv)
    out = args.out or args.mcap.with_name(args.mcap.stem + "_csv")
    out.mkdir(parents=True, exist_ok=True)
    rows = defaultdict(list)
    t0 = None
    for topic, t, data in read_mcap(args.mcap, args.topics):
        t0 = t if t0 is None else t0
        rows[topic].append({"t": round(t - t0, 6), **flatten(data)})
    for topic, rs in rows.items():
        keys = list(dict.fromkeys(k for r in rs for k in r))
        path = out / (topic.strip("/").replace("/", "_") + ".csv")
        with path.open("w", newline="", encoding="utf-8-sig") as f:   # Excel で文字化けしないよう BOM 付き
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rs)
        print(f"{path}  ({len(rs)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
