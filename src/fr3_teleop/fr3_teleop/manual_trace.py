"""Opt-in diagnostics only; never publish commands or open robot connections."""
import json
import os
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Thread
from time import monotonic_ns, time_ns


class ManualTrace:
    def __init__(self, directory, capacity=8192):
        self.queue = Queue(maxsize=capacity)
        self.dropped = 0
        self.sequence = 0
        self.stopping = False
        self.path = Path(directory) / f"manager_{os.getpid()}_{monotonic_ns()}.jsonl"
        self.output = self.path.open("x", encoding="utf-8")
        self.worker = Thread(target=self._run, name="manual-trace", daemon=True)
        self.worker.start()

    def record(self, kind, **fields):
        self.sequence += 1
        event = dict(kind=kind, mono_ns=monotonic_ns(), seq=self.sequence, **fields)
        try:
            self.queue.put_nowait(event)
        except Full:
            self.dropped += 1

    def _write(self, event):
        self.output.write(json.dumps(event, ensure_ascii=False) + "\n")

    def _run(self):
        try:
            while not self.stopping or not self.queue.empty():
                try:
                    self._write(self.queue.get(timeout=0.02))
                except Empty:
                    pass
                self._write(dict(kind="health", mono_ns=monotonic_ns(),
                                 realtime_ns=time_ns(), dropped=self.dropped))
                self.output.flush()
        finally:
            self.output.close()

    def close(self):
        self.stopping = True
        self.worker.join()


def record(node, kind, **fields):
    trace = getattr(node, "_manual_trace", None)
    if trace is not None:
        trace.record(kind, **fields)


def from_environment():
    directory = os.environ.get("FR3_TRACE_DIR")
    return ManualTrace(directory) if directory else None
