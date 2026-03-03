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


class MessageState:
    """
    Holds current transmit state for one periodic message.

    Replay mode: if the constructor receives `data` as a list-of-lists
    (i.e. isinstance(data[0], (list, tuple))), the message cycles through
    those frames in order, wrapping around.  The `data` attribute always
    reflects the *last sent* frame for display purposes.
    """
    __slots__ = ("arb_id", "name", "interval_ms", "data",
                 "tx_count", "last_tx", "_replay", "_ridx")

    def __init__(self, arb_id, name, interval_ms, data):
        self.arb_id      = arb_id
        self.name        = name
        self.interval_ms = interval_ms
        self.tx_count    = 0
        self.last_tx     = 0.0

        # Detect replay sequence (list of lists / list of tuples)
        if data and isinstance(data[0], (list, tuple)):
            self._replay = [list(f) for f in data]
            self._ridx   = 0
            self.data    = list(self._replay[0])   # display: current frame
        else:
            self._replay = None
            self._ridx   = 0
            self.data    = list(data)

    def next_payload(self) -> list:
        """Return the next payload to send (advances replay index)."""
        if self._replay is None:
            return self.data
        frame = self._replay[self._ridx]
        self._ridx = (self._ridx + 1) % len(self._replay)
        self.data  = frame          # keep display in sync
        return frame


class ECUModule:
    ECU_ID   = "00"
    ECU_NAME = "Unknown"
    MESSAGES = []   # [(arb_id, name, interval_ms, [data]), ...]

    def __init__(self, log_cb=None):
        self._log      = log_cb or (lambda m: None)
        self._bus      = None          # set by attach()
        self._tx_q     = None          # shared queue.Queue set by attach()
        self._stop_evt = threading.Event()
        self._thread   = None
        self._lock     = threading.Lock()
        self.enabled   = True

        self._states = [
            MessageState(a, n, i, d)
            for a, n, i, d in self.MESSAGES
        ]

    # ── Public API ────────────────────────────────────────────────────────────

    def attach(self, bus, tx_queue: queue.Queue):
        """Start periodic sending.  tx_queue is shared with all other modules."""
        self._bus  = bus
        self._tx_q = tx_queue
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

    # ── Internal ─────────────────────────────────────────────────────────────

    def _run(self):
        now    = time.monotonic()
        # interval_ms=0 would flood (enqueue every loop); clamp to 10ms min
        next_tx = {
            s.arb_id: now + max(s.interval_ms, 10) / 1000.0
            for s in self._states
        }

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
                    interval = max(s.interval_ms, 10)  # avoid 0 → flood
                    next_tx[s.arb_id] = due + interval / 1000.0
                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            self._stop_evt.wait(timeout=max(0.001, soonest))

    # ── Helpers for subclasses ────────────────────────────────────────────────

    def _enqueue(self, arb_id: int, data: list):
        """Thread-safe: put a frame on the shared send queue."""
        if self._tx_q is not None and self.enabled:
            self._tx_q.put((arb_id, data))

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
        """Update display state for a manually enqueued frame."""
        with self._lock:
            for s in self._states:
                if s.arb_id == arb_id:
                    s.data      = list(data[:8])
                    s.tx_count += 1
                    s.last_tx   = time.monotonic()
                    return
