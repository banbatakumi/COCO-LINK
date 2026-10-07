import json
import time

import pytest

from coco_link.net import UdpEndpoint
from coco_link.protocol import (
    MAX_DATAGRAM,
    MESSAGE_TYPES,
    Buzzer,
    CmdVel,
    DetectedObject,
    DetectedRobot,
    FieldSize,
    Hello,
    Imu,
    ProtocolError,
    Sender,
    SetLed,
    Telemetry,
    WorldState,
    decode,
    encode,
)


@pytest.mark.parametrize("cls", list(MESSAGE_TYPES.values()))
def test_roundtrip_defaults(cls):
    msg = cls()
    env = decode(encode(msg, "op", 7, 1234))
    assert env.msg == msg
    assert (env.src, env.seq, env.t_ms) == ("op", 7, 1234)


def test_roundtrip_nested():
    t = Telemetry(robot_id=4, state="RUNNING", faults=["imu"], batt_v=4.81, us_front_m=0.42,
                  imu=Imu(az=9.8, gz=-0.25))
    env = decode(encode(t, "robot:4"))
    assert env.msg == t
    assert env.robot_id == 4


def test_wire_format_matches_spec():
    """docs/protocol.md §3 の例と同じ形になること."""
    raw = encode(CmdVel(vx=0.15, wz=-0.3), "op", 1024, 53122)
    assert json.loads(raw) == {"v": 1, "type": "cmd_vel", "src": "op", "seq": 1024, "t_ms": 53122,
                               "vx": 0.15, "wz": -0.3}


def test_optional_melody_is_omitted():
    assert "melody" not in json.loads(encode(Buzzer(freq_hz=440, dur_ms=100), "op"))
    assert decode(encode(Buzzer(melody="ok"), "op")).msg.melody == "ok"


def test_unknown_keys_are_ignored():
    raw = b'{"v":1,"type":"cmd_vel","src":"op","seq":0,"t_ms":0,"vx":0.1,"wz":0,"future":1}'
    assert decode(raw).msg == CmdVel(0.1, 0.0)


@pytest.mark.parametrize("raw", [b"not json", b"[]", b'{"v":2,"type":"stop"}', b'{"v":1,"type":"nope"}',
                                 b'{"v":1,"type":"cmd_vel","vx":"fast"}'])
def test_invalid_rejected(raw):
    with pytest.raises(ProtocolError):
        decode(raw)


def test_size_limits():
    """典型的な最大構成（ロボット30台・物体各10個）が 1400 byte に収まること."""
    ws = WorldState(frame=123456, stamp_ms=99999, latency_ms=33.3, field=FieldSize(3.0, 2.0),
                    robots=[DetectedRobot(i, 1.234567, 1.234567, -3.141592, 0.99) for i in range(12)],
                    persons=[DetectedObject(1.2345, 1.2345, 0.15)] * 4,
                    obstacles=[DetectedObject(1.2345, 1.2345, 0.15)] * 4,
                    cargo=[DetectedObject(1.2345, 1.2345, 0.15)] * 4)
    assert len(encode(ws, "vision")) <= MAX_DATAGRAM
    t = Telemetry(robot_id=29, state="LOW_BATT", faults=["imu", "enc_l", "enc_r", "us_front"],
                  batt_v=4.123456, us_front_m=1.234567, us_rear_m=1.234567,
                  imu=Imu(*[-12.345678] * 6), rssi=-70, loop_dt_us=10000, rx_count=4000000000)
    assert len(encode(t, "robot:29")) < 600


def test_sender_increments_seq():
    s = Sender("op")
    seqs = [decode(s.encode(SetLed())).seq for _ in range(3)]
    assert seqs == [0, 1, 2]


def test_udp_loopback():
    received = []
    rx = UdpEndpoint("op", "127.0.0.1", 0, on_message=lambda env, addr: received.append((env, addr))).start()
    tx = UdpEndpoint("robot:1", "127.0.0.1", 0)
    try:
        tx.send(Hello(robot_id=1, fw="test"), ("127.0.0.1", rx.port))
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.01)
        assert received
        env, addr = received[0]
        assert env.msg == Hello(robot_id=1, fw="test")
        assert addr[1] == tx.port
    finally:
        rx.close()
        tx.close()
