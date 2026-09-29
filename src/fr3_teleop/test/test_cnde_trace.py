import io
import struct
import pytest
from fr3_teleop.cnde_trace import Connection, decode_pcap, wire_types

FORMATS = {"actual_joint_pos": "<6d", "actual_TCP_pos": "<6d",
           "target_TCP_pos": "<6d", "robot_time": "<7i"}


def frame(count, kind, data):
    return struct.pack("<HBBH", 0x5a5a, count, kind, len(data)) + data + b"\xa5\xa5"


def configured():
    conn = Connection(FORMATS)
    request = frame(0, 1, struct.pack("<H", 8) + ",".join(FORMATS).encode())
    assert not conn.feed(False, 100, request[:4])
    assert conn.feed(False, 104, request[4:])[0]["kind"] == "configuration"
    assert not conn.feed(False, 100, request)  # TCP retransmission
    conn.feed(True, 200, frame(0, 6, b"\x00"))
    return conn


def sample(count, millis):
    return frame(count, 4, struct.pack("<18d7i", *range(18), 2026, 9, 28, 1, 2, 3, millis))


def test_decode_fragmentation_wrap_and_feedback():
    conn = configured()
    payload = sample(255, 0) + sample(0, 8)
    assert not conn.feed(True, 209, payload[:50])
    a, b = conn.feed(True, 259, payload[50:])
    assert a["valid"] and b["valid"]
    assert b["frame_delta"] == 1 and b["robot_time_changed"]
    assert b["values"]["actual_TCP_pos"] == list(range(6, 12))


def test_missing_handshake_fails_closed():
    conn = Connection(FORMATS)
    assert conn.feed(True, 0, sample(0, 0))[0]["kind"] == "invalid"
    assert conn.failed


def test_tcp_gap_fails_closed():
    conn = configured()
    assert conn.feed(True, 210, sample(0, 0))[0]["kind"] == "invalid"


def test_bad_layout_and_missing_target():
    conn = configured()
    assert conn.feed(True, 209, frame(0, 4, b"\x00"))[0]["kind"] == "invalid"
    conn = Connection(FORMATS)
    conn.frame(False, 0, 1, b"\x08\x00actual_joint_pos")
    conn.frame(True, 0, 6, b"\x00")
    event = conn.frame(True, 1, 4, struct.pack("<6d", *range(6)))
    assert not event["valid"] and "target_TCP_pos" in event["missing"]
    assert not event["actual_fields_valid"]


def test_actual_only_feedback_does_not_invent_target_or_claim_full_validity():
    conn = Connection(FORMATS)
    conn.frame(False, 0, 1, b"\x08\x00actual_joint_pos,actual_TCP_pos,robot_time")
    conn.frame(True, 0, 6, b"\x00")
    data = struct.pack("<12d7i", *range(12), 2026, 9, 28, 1, 2, 3, 0)
    first = conn.frame(True, 0, 4, data)
    repeated = conn.frame(True, 0, 4, data)
    assert first["actual_fields_valid"] and not first["valid"]
    assert first["actual_missing"] == []
    assert not first["target_available"]
    assert "target_TCP_pos" not in first["values"]
    assert first["robot_time_changed"] is None
    assert repeated["robot_time_changed"] is False  # values valid != fresh
    bad = struct.pack("<12d7i", float("nan"), *range(1,12), 2026,9,28,1,2,3,8)
    assert not conn.frame(True, 0, 4, bad)["actual_fields_valid"]


def test_ast_does_not_execute_sdk():
    source = 'raise RuntimeError("must never execute")\nCNDE_STATE_CONFIG = {A.B: ("joint", "field", "DOUBLE_6", "DOUBLE_6")}\n'
    assert wire_types(source) == {"joint": "<6d"}


def test_pcap_partial_record_rejected():
    header = struct.pack("<IHHIIII", 0xa1b2c3d4, 2, 4, 0, 0, 65535, 1)
    with pytest.raises(ValueError, match="partial"):
        list(decode_pcap(io.BytesIO(header+b"123"), FORMATS))


def test_complete_ethernet_pcap_roundtrip():
    def packet(inbound, seq, payload):
        host, robot = b"\xc0\xa8\x3a\x01", b"\xc0\xa8\x3a\x02"
        src, dst = (robot, host) if inbound else (host, robot)
        sport, dport = (20005, 50000) if inbound else (50000, 20005)
        tcp = struct.pack("!HHIIBBHHH", sport, dport, seq, 0, 0x50, 0x18, 0, 0, 0)+payload
        ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20+len(tcp), 0, 0, 64, 6, 0, src, dst)+tcp
        eth = bytes(12)+b"\x08\x00"+ip
        return struct.pack("<4I", 100, 500, len(eth), len(eth))+eth
    request = frame(0, 1, b"\x08\x00"+",".join(FORMATS).encode())
    header = struct.pack("<IHHIIII", 0xa1b2c3d4, 2, 4, 0, 0, 65535, 1)
    capture = header + packet(False, 100, request) + packet(True, 200, frame(0, 6, b"\x00"))
    capture += packet(True, 209, sample(1, 8))
    events = list(decode_pcap(io.BytesIO(capture), FORMATS))
    assert events[-1]["valid"]
    assert events[-1]["capture_realtime_ns"] == 100000500000
    assert events[-1]["connection"][0] == "192.168.58.2"
    key = ("192.168.58.2", "192.168.58.1", 20005, 50000)
    middle = header + packet(True, 500, b"tail of pre-capture frame")
    middle += packet(True, 600, sample(1,8))
    seeded = list(decode_pcap(io.BytesIO(middle), FORMATS, verified_configs={key:list(FORMATS)}))
    assert seeded[0]["kind"] == "capture_bootstrap"
    assert not seeded[0]["continuity_before_this_point"]
    assert seeded[-1]["actual_fields_valid"]
    # A gap AFTER the declared capture boundary must not be silently skipped.
    middle += packet(True, 99999, sample(2,16))
    seeded = list(decode_pcap(io.BytesIO(middle), FORMATS, verified_configs={key:list(FORMATS)}))
    assert seeded[-1]["kind"] == "invalid"
