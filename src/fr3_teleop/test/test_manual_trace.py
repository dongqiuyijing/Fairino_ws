import json
from fr3_teleop.manual_trace import ManualTrace, record


def test_flush_and_clock(tmp_path):
    trace = ManualTrace(tmp_path)
    for i in range(100):
        trace.record("velocity", value=i)
    trace.close()
    events = [json.loads(line) for line in trace.path.read_text().splitlines()]
    samples = [e for e in events if e["kind"] == "velocity"]
    assert [e["value"] for e in samples] == list(range(100))
    assert all(a["mono_ns"] <= b["mono_ns"] for a, b in zip(samples, samples[1:]))
    assert events[-1]["dropped"] == 0


def test_unconfigured_is_noop():
    record(object(), "ignored", value=1)
