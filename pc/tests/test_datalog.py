import subprocess
import sys
from pathlib import Path

from coco_link.datalog import McapLogger, read_mcap
from coco_link.datalog.mcap_logger import json_schema
from coco_link.protocol import messages as m


def test_json_schema_from_dataclass():
    s = json_schema(m.Telemetry)
    assert s["properties"]["imu"]["properties"]["gz"] == {"type": "number"}
    assert s["properties"]["us_front_m"]["anyOf"][1] == {"type": "null"}


def test_write_and_read(tmp_path):
    log = McapLogger()
    path = log.start(tmp_path / "t.mcap")
    log.write_message("/robot/1/telemetry", m.Telemetry(robot_id=1, batt_v=4.9), {"robot_t_ms": 10})
    log.write_message("/vision/world_state", m.WorldState(robots=[m.DetectedRobot(1, 0.5, 0.5, 0.1)]))
    log.write("/core/event", "coco_link.event", {"type": "object"}, {"event": "estop"})
    assert log.stop() == path
    msgs = list(read_mcap(path))
    assert [t for t, _, _ in msgs] == ["/robot/1/telemetry", "/vision/world_state", "/core/event"]
    assert msgs[0][2]["batt_v"] == 4.9 and msgs[0][2]["robot_t_ms"] == 10
    assert msgs[1][2]["robots"][0]["x"] == 0.5


def test_mcap_to_csv(tmp_path):
    log = McapLogger()
    path = log.start(tmp_path / "t.mcap")
    for i in range(3):
        log.write_message("/robot/2/telemetry", m.Telemetry(robot_id=2, batt_v=4.0 + i))
    log.stop()
    tool = Path(__file__).resolve().parents[1] / "tools" / "mcap_to_csv.py"
    subprocess.run([sys.executable, str(tool), str(path), "--out", str(tmp_path / "csv")], check=True)
    text = (tmp_path / "csv" / "robot_2_telemetry.csv").read_text(encoding="utf-8-sig")
    assert "imu.gz" in text.splitlines()[0]
    assert len(text.strip().splitlines()) == 4
