"""
ecu_base.py — Base class for all ECU emulation modules.

THREADING MODEL:
  All CAN sends go through a shared send_queue (queue.Queue) provided by
  BusManager. A single writer thread drains the queue so the serial port
  is never accessed from more than one thread at a time.

Each module subclasses ECUModule and defines:
  ECU_ID    str   e.g. "01"
  ECU_NAME  str   e.g. "Motor"
  MESSAGES  list  of (arb_id, name, interval_ms, [default_data])
"""

import can
import threading
import time
import queue

from bap import build_bap_frames


class MessageState:
    __slots__ = ("arb_id", "name", "interval_ms", "data", "tx_count", "last_tx", "_frame_idx")

    def __init__(self, arb_id, name, interval_ms, data):
        self.arb_id      = arb_id
        self.name        = name
        self.interval_ms = interval_ms
        # data may be a flat byte list OR a list-of-frames for cyclic multi-frame messages
        self.data        = [list(f) for f in data] if data and isinstance(data[0], (list, tuple)) else list(data)
        self.tx_count    = 0
        self.last_tx     = 0.0
        self._frame_idx  = 0

    @property
    def is_multi(self):
        """True if data is a list-of-frames (cyclic send)."""
        return bool(self.data) and isinstance(self.data[0], list)

    def next_payload(self):
        """Return the next frame to send. Cycles through frames for multi-frame states."""
        if self.is_multi:
            frame = self.data[self._frame_idx]
            self._frame_idx = (self._frame_idx + 1) % len(self.data)
            return frame
        return self.data

    def current_display(self):
        """Flat byte list suitable for display (first frame if multi)."""
        if self.is_multi:
            return self.data[self._frame_idx] if self.data else []
        return self.data


class ECUModule:
    ECU_ID   = "00"
    ECU_NAME = "Unknown"
    MESSAGES = []   # [(arb_id, name, interval_ms, [data]), ...]
    RX_IDS   = ()

    def __init__(self, log_cb=None):
        self._log      = log_cb or (lambda m: None)
        self._bus      = None          # set by attach()
        self._tx_q     = None          # shared queue.Queue set by attach()
        self._bus_lock = threading.Lock()
        self._frame_cb = lambda *_args, **_kwargs: None
        self._stop_evt = threading.Event()
        self._thread   = None
        self._lock     = threading.Lock()
        self.enabled   = True

        self._states = [
            MessageState(a, n, i, d)
            for a, n, i, d in self.MESSAGES
        ]

    # ── Public API ────────────────────────────────────────────────────────────

    def attach(self, bus, tx_queue: queue.Queue, bus_lock=None, frame_cb=None):
        """Start periodic sending.  tx_queue is shared with all other modules."""
        self._bus  = bus
        self._tx_q = tx_queue
        self._bus_lock = bus_lock or threading.Lock()
        self._frame_cb = frame_cb or (lambda *_args, **_kwargs: None)
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name=f"ECU-{self.ECU_ID}"
        )
        self._thread.start()

    def detach(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._bus  = None
        self._tx_q = None

    def get_states(self):
        with self._lock:
            return list(self._states)

    def update_data(self, arb_id, new_data):
        with self._lock:
            for s in self._states:
                if s.arb_id == arb_id:
                    s.data = list(new_data)
                    break

    def wants_message(self, msg) -> bool:
        return bool(self.RX_IDS) and msg.arbitration_id in self.RX_IDS

    def on_message(self, msg):
        return None

    # ── Internal ─────────────────────────────────────────────────────────────

    def _run(self):
        now    = time.monotonic()
        next_tx = {s.arb_id: now + s.interval_ms / 1000.0 for s in self._states}

        while not self._stop_evt.is_set():
            now     = time.monotonic()
            soonest = 0.010

            for s in self._states:
                due = next_tx[s.arb_id]
                if now >= due:
                    self._enqueue(s.arb_id, s.next_payload())
                    with self._lock:
                        s.tx_count += 1
                        s.last_tx   = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0
                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            self._stop_evt.wait(timeout=max(0.001, soonest))

    # ── Helpers for subclasses ────────────────────────────────────────────────

    def _enqueue(self, arb_id: int, data: list):
        """Thread-safe: put a frame on the shared send queue.
        Uses put_nowait so a dead/slow writer never blocks the TX thread."""
        if self._tx_q is not None and self.enabled:
            try:
                self._tx_q.put_nowait((arb_id, data))
            except Exception:
                pass  # Queue full or writer dead — drop frame, never block

    def _tx(self, arb_id: int, data: list):
        """Alias used by subclasses."""
        self._enqueue(arb_id, data)

    def _tx_multi(self, arb_id: int, payload: list):
        """BAP multi-frame — enqueues all continuation frames."""
        n = len(payload)
        self._enqueue(arb_id, [0x80, n] + payload[:6])
        pos = 6
        while pos < n:
            last = pos + 7 >= n
            self._enqueue(arb_id, ([0xC1] if last else [0xC0]) + payload[pos:pos + 7])
            pos += 7

    def _mark_sent(self, arb_id: int, data: list):
        """Update display state for a manually enqueued frame.
        Does not overwrite multi-frame (cyclic) states."""
        with self._lock:
            for s in self._states:
                if s.arb_id == arb_id:
                    if not s.is_multi:
                        s.data = list(data[:8])
                    s.tx_count += 1
                    s.last_tx   = time.monotonic()
                    return

    def _send_frames_atomic(self, arb_id: int, frames: list[list[int]], ifg_s: float = 0.0004):
        if not self._bus or not frames or not self.enabled:
            return
        with self._bus_lock:
            for frame in frames:
                try:
                    self._bus.send(can.Message(
                        arbitration_id=arb_id,
                        data=bytes(frame),
                        is_extended_id=arb_id > 0x7FF
                    ))
                    self._frame_cb("tx", arb_id, list(frame))
                except Exception as e:
                    self._log(f"[TX_MF] bus.send error: {e}")
                time.sleep(ifg_s)

    def _tx_bap(self, arb_id: int, opcode: int, lsg_id: int, fct_id: int, payload=None,
                logical_channel: int = 0, atomic: bool = False, ifg_s: float = 0.0004,
                force_long: bool = False):
        frames = build_bap_frames(
            opcode, lsg_id, fct_id, payload or [],
            logical_channel=logical_channel, force_long=force_long
        )
        if atomic and len(frames) > 1:
            self._send_frames_atomic(arb_id, frames, ifg_s=ifg_s)
            return frames
        for frame in frames:
            self._enqueue(arb_id, frame)
        return frames
