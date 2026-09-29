"""Passive, fail-closed CNDE decoder. Reads pcap; never opens a robot socket.

Requires the configuration handshake from the SAME TCP connection. Unknown
fields, missing bytes or a TCP gap invalidate decoding; no guessed layout.
The SDK Python source is parsed as text/AST, never imported or executed.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import time


def wire_types(source):
    tree = ast.parse(source)
    config = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == "CNDE_STATE_CONFIG" for t in n.targets))
    formats = {}
    scalar = dict(UINT8="B", INT8="b", UINT16="H", INT16="h", UINT32="I",
                  INT32="i", UINT64="Q", INT64="q", FLOAT="f", DOUBLE="d")
    for value in config.values:
        name, _, _, wire = ast.literal_eval(value)
        names, types = name.split(","), wire.split(",")
        if len(names) != len(types):
            raise ValueError(f"ambiguous mapping {name}/{wire}")
        for name, spec in zip(names, types):
            base, _, count = spec.partition("_")
            formats[name] = "<" + str(int(count) if count else 1) + scalar[base]
    return formats


class Connection:
    def __init__(self, formats):
        self.formats = formats
        self.buffers = {False: bytearray(), True: bytearray()}
        self.next_seq = {}
        self.names = None
        self.ack = False
        self.failed = False
        self.previous_count = None
        self.previous_time = None

    def feed(self, robot_to_host, seq, payload):
        if self.failed or not payload:
            return []
        try:
            expected = self.next_seq.get(robot_to_host, seq)
            delta = ((seq - expected + 2**31) % 2**32) - 2**31
            if delta > 0:
                raise ValueError("TCP gap/out-of-order: capture is not complete")
            if delta < 0:
                payload = payload[min(-delta, len(payload)):]
            self.next_seq[robot_to_host] = (expected + len(payload)) % 2**32
            buf = self.buffers[robot_to_host]
            buf.extend(payload)
            events = []
            while len(buf) >= 6:
                head, count, kind, size = struct.unpack_from("<HBBH", buf)
                if head != 0x5A5A:
                    raise ValueError("CNDE frame header mismatch (capture started mid-stream?)")
                if len(buf) < size + 8:
                    break
                if struct.unpack_from("<H", buf, size + 6)[0] != 0xA5A5:
                    raise ValueError("CNDE tail mismatch")
                data = bytes(buf[6:6+size])
                del buf[:size+8]
                events.append(self.frame(robot_to_host, count, kind, data))
            return events
        except (ValueError, KeyError, UnicodeError, struct.error) as exc:
            self.failed = True
            return [dict(kind="invalid", valid=False, reason=str(exc))]

    def frame(self, inbound, count, kind, data):
        event = dict(kind="cnde_event", frame_count=count, frame_type=kind)
        if not inbound and kind == 1:
            self.names = data[2:].decode("utf-8").split(",")
            self.ack = False
            self.previous_count = self.previous_time = None
            for name in self.names:
                if name not in self.formats:
                    raise ValueError(f"unknown configured field {name}")
            event.update(kind="configuration", period_ms=struct.unpack_from("<H", data)[0], fields=self.names)
        elif inbound and kind == 6:
            if not data or data[0] != 0:
                raise ValueError("controller rejected CNDE configuration/start")
            if self.names is not None:
                self.ack = True
        elif inbound and kind == 4:
            if self.names is None or not self.ack:
                raise ValueError("no acknowledged configuration captured")
            values, offset = {}, 0
            for name in self.names:
                fmt = self.formats[name]
                values[name] = list(struct.unpack_from(fmt, data, offset))
                offset += struct.calcsize(fmt)
            if offset != len(data):
                raise ValueError("configured layout/payload length mismatch")
            delta = None if self.previous_count is None else (count-self.previous_count) % 256
            robot_time = values.get("robot_time")
            advancing = None if self.previous_time is None or robot_time is None else robot_time > self.previous_time
            self.previous_count, self.previous_time = count, robot_time
            required = {"actual_joint_pos", "actual_TCP_pos", "robot_time", "target_TCP_pos"}
            missing = sorted(required - values.keys())
            finite = all(math.isfinite(v) for array in values.values() for v in array)
            # Actual-feedback diagnosis is explicitly weaker than complete
            # target/actual diagnosis. Neither flag establishes freshness.
            actual_required = {"actual_joint_pos": 6, "actual_TCP_pos": 6, "robot_time": 7}
            actual_missing = sorted(actual_required.keys() - values.keys())
            actual_valid = not actual_missing and all(
                len(values[name]) == size and all(math.isfinite(v) for v in values[name])
                for name, size in actual_required.items())
            event.update(kind="feedback", decoded=True, valid=not missing and finite,
                         missing=missing, actual_fields_valid=actual_valid,
                         actual_missing=actual_missing,
                         target_available="target_TCP_pos" in values,
                         frame_delta=delta, robot_time_changed=advancing, values=values)
        return event


def tcp_packet(packet, linktype):
    if linktype == 1:
        protocol = struct.unpack_from("!H", packet, 12)[0]
        offset = 14
        if protocol == 0x8100:
            protocol = struct.unpack_from("!H", packet, 16)[0]
            offset = 18
    elif linktype == 113:
        protocol, offset = struct.unpack_from("!H", packet, 14)[0], 16
    elif linktype == 276:
        protocol, offset = struct.unpack_from("!H", packet)[0], 20
    else:
        raise ValueError(f"unsupported pcap linktype {linktype}")
    if protocol != 0x800:
        return None
    ip = packet[offset:]
    if len(ip) < 20 or ip[0] >> 4 != 4 or ip[9] != 6:
        return None
    if struct.unpack_from("!H", ip, 6)[0] & 0x3fff:
        raise ValueError("fragmented IPv4 unsupported")
    length = struct.unpack_from("!H", ip, 2)[0]
    if len(ip) < length:
        raise ValueError("truncated IPv4 capture")
    tcp = ip[(ip[0] & 15)*4:length]
    src, dst = socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20])
    sport, dport, seq = struct.unpack_from("!HHI", tcp)
    if 20005 not in (sport, dport):
        return None
    inbound = sport == 20005
    key = (src, dst, sport, dport) if inbound else (dst, src, dport, sport)
    return key, inbound, seq, tcp[13], tcp[(tcp[12] >> 4)*4:]


def decode_pcap(handle, formats, follow=False, verified_configs=None):
    """Optional configs must come from an acknowledged capture of the SAME
    TCP four-tuple. Used only when starting a second passive capture mid-session.
    Initial partial TCP/CNDE data may be skipped before the first whole frame;
    subsequent gaps remain fatal. No target or actual values are synthesized.
    """
    def read(size):
        data = b""
        while len(data) < size:
            chunk = handle.read(size-len(data))
            if not chunk:
                if not follow:
                    if data:
                        raise ValueError("partial pcap record")
                    return b""
                time.sleep(0.02)
            data += chunk
        return data
    header = read(24)
    magic = {b"\xd4\xc3\xb2\xa1": ("<",1000), b"\xa1\xb2\xc3\xd4": (">",1000),
             b"\x4d\x3c\xb2\xa1": ("<",1), b"\xa1\xb2\x3c\x4d": (">",1)}
    endian, multiplier = magic[header[:4]]
    linktype = struct.unpack_from(endian+"I", header, 20)[0]
    connections = {}
    bootstrap_pending = set()
    for key, names in (verified_configs or {}).items():
        connection = Connection(formats)
        for name in names:
            if name not in formats:
                raise ValueError(f"unknown bootstrap field {name}")
        connection.names, connection.ack = list(names), True
        connections[key] = connection
        bootstrap_pending.add(key)
    while True:
        record = read(16)
        if not record:
            if any(any(c.buffers.values()) for c in connections.values()):
                raise ValueError("capture ends in partial CNDE frame")
            return
        sec, fraction, captured, original = struct.unpack(endian+"4I", record)
        if captured > 16*1024*1024 or captured != original:
            raise ValueError("oversized/truncated pcap record")
        parsed = tcp_packet(read(captured), linktype)
        if parsed is None:
            continue
        key, inbound, seq, flags, payload = parsed
        if flags & 2 and not inbound:
            connections[key] = Connection(formats)
            bootstrap_pending.discard(key)
        if key in bootstrap_pending:
            # Never reinterpret partial payload bytes as a guessed layout.
            # Motion preflight must wait for a continuous post-bootstrap window.
            if not inbound or not payload.startswith(b"\x5a\x5a"):
                continue
            bootstrap_pending.remove(key)
            yield dict(kind="capture_bootstrap", connection=key,
                       capture_realtime_ns=sec*10**9+fraction*multiplier,
                       continuity_before_this_point=False)
        connection = connections.setdefault(key, Connection(formats))
        for event in connection.feed(inbound, (seq + bool(flags & 2)) % 2**32, payload):
            event.update(connection=key, capture_realtime_ns=sec*10**9+fraction*multiplier,
                         decoded_monotonic_ns=time.monotonic_ns())
            yield event


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap")
    parser.add_argument("--sdk-source", required=True)
    parser.add_argument("--follow", action="store_true")
    args = parser.parse_args()
    source = Path(args.sdk_source).read_bytes()
    print(json.dumps(dict(kind="schema", sha256=hashlib.sha256(source).hexdigest(),
                          monotonic_ns=time.monotonic_ns(), realtime_ns=time.time_ns())), flush=True)
    with open(args.pcap, "rb") as handle:
        for event in decode_pcap(handle, wire_types(source), args.follow):
            print(json.dumps(event), flush=True)


if __name__ == "__main__":
    main()
