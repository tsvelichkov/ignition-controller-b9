"""
modules/16_stalk.py — Steering column stalk module (Lenkstockschalter)

Sends:
  0x52B  LSS_01          — Wiper/blinker stalk input state,      50ms  [replay, 8-frame cycle]
  0x366  Blinkmodi_02    — Turn signal output / BCM input,      1000ms  [dynamic]
  0x3D4  Blink_Anf_01    — Blinker request acknowledge,          50ms  [replay]

LSS_01 (0x52B) byte layout — verified from 0000027.TXT:
  byte 0 = CRC (unknown private key / scheme; constant per BZ value → use replay)
  byte 1 = 0x20 | BZ  (upper nibble always 0x2; lower nibble = 4-bit rolling counter)
  byte 2 = 0x81 (neutral/right stalk position in this log; 0x80 = left)
  byte 3 = 0xD0 (constant)
  byte 4 = 0x00 (idle/neutral; active stalk press may set other bits)

  NOTE: byte 0 is NOT a simple rolling counter as previously documented.
  It is deterministic per (BZ, body) and repeats every 8 frames. The counter
  is in byte 1 (upper nibble = 0x2, lower = BZ). Since no DBC E2E table exists
  for 0x52B (not in HCAN or ICAN KMatrix), frames are replayed verbatim from log.

0x366 Blinkmodi_02 byte layout:
  byte 2 bit 7  (0x80) = left blinker active
  byte 3 bit 0  (0x01) = right tip / single flash
  byte 3 bits   (0x15) = right full turn signal (5-tap)
  byte 4 bit 2  (0x04) = flash sync set
  byte 4 constant base = 0x20

Commands accepted via send_command():
  "blink_left"     — left turn signal (continuous, manual cancel)
  "blink_right"    — right turn signal (continuous, manual cancel)
  "blink_off"      — cancel all blinkers
  "blink_hazard"   — hazard lights (left + right simultaneous)
  "tip_left"       — 3-flash lane change left (auto-cancel after ~750ms)
  "tip_right"      — 3-flash lane change right (auto-cancel after ~750ms)

Source: 0000084.TXT blinker stalk log, left/right phase payload analysis.
        0000027.TXT verified LSS_01 byte layout and Blinkmodi_02 interval (1000ms).

Fixes applied (0000027.TXT audit):
  LSS_01: byte 0 is CRC not counter; byte 1 = 0x20|BZ not fixed 0x0F;
          byte 4 = 0x00 idle (not 0x0C); replaced counter logic with 8-frame replay.
  Blinkmodi_02: interval corrected in docstring (was 200ms; confirmed 1000ms in log).
"""
import sys, os, time, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule

# ── Blinkmodi_02 (0x366) payload constants ────────────────────────────────────
_BM_IDLE   = bytes([0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xF0])  # no blinker
_BM_LEFT   = bytes([0x00, 0x00, 0x80, 0x00, 0x24, 0x00, 0x00, 0xF0])  # left tip flash
_BM_LEFT5  = bytes([0x00, 0x00, 0x80, 0x0A, 0x24, 0x00, 0x00, 0xF0])  # left 5-tap / full
_BM_RIGHT1 = bytes([0x00, 0x00, 0x00, 0x01, 0x24, 0x00, 0x00, 0xF0])  # right tip flash
_BM_RIGHT  = bytes([0x00, 0x00, 0x00, 0x15, 0x24, 0x00, 0x00, 0xF0])  # right full
_BM_HAZARD = bytes([0x00, 0x00, 0x80, 0x15, 0x24, 0x00, 0x00, 0xF0])  # both sides

# ── LSS_01 (0x52B) byte 2 direction bit ──────────────────────────────────────
_LSS_B2_NEUTRAL = 0x81   # confirmed dominant in log (neutral / no stalk input)
_LSS_B2_LEFT    = 0x80   # left stalk press
_LSS_B2_RIGHT   = 0x81   # right blinker pressed (same as neutral in log; stalk HW bit)

# ── LSS_01 (0x52B) 8-frame replay sequence — from 0000027.TXT ────────────────
# All 1030 log frames collapse to 8 unique payloads, one per BZ nibble value.
# BZ cycles through odd values only (1,3,5,7,9,B,D,F → 8 steps) in this log,
# upper nibble of byte1 is always 0x2. Byte 0 is a CRC with an unknown private
# key — replayed verbatim. Byte 4 = 0x00 throughout (stalk idle).
# Sequence ordered by BZ ascending for clean rollover.
_SEQ_52B = [bytes.fromhex(h) for h in [
    "1f2181d000000000",   # BZ=1
    "de2381d000000000",   # BZ=3
    "cb2581d000000000",   # BZ=5
    "3d2781d000000000",   # BZ=7
    "052981d000000000",   # BZ=9
    "262b81d000000000",   # BZ=B
    "fb2d81d000000000",   # BZ=D
    "ae2f81d000000000",   # BZ=F
]]

# ── Blink_Anf_01 (0x3D4) replay sequence — 15 frames at 50ms ─────────────────
_SEQ_3D4 = [bytes.fromhex(h) for h in [
    "1a04820400000000", "2d0f820400000000", "f105820400000000",
    "f80e820400000000", "8a03820400000000", "8c01820400000000",
    "b907820400000000", "c702820400000000", "d806820400000000",
    "590a820400000000", "4209820400000000", "1708820400000000",
    "5d0b820400000000", "780c820400000000", "870d820400000000",
]]


class StalkECU(ECUModule):
    ECU_ID   = "16"
    ECU_NAME = "STALK"

    # Blinker state constants
    BLINK_OFF    = "off"
    BLINK_LEFT   = "left"
    BLINK_RIGHT  = "right"
    BLINK_HAZARD = "hazard"
    BLINK_TIP_L  = "tip_left"
    BLINK_TIP_R  = "tip_right"

    # Map state → (Blinkmodi_02 payload, LSS byte2)
    _STATE_MAP = {
        BLINK_OFF:    (_BM_IDLE,    _LSS_B2_NEUTRAL),
        BLINK_LEFT:   (_BM_LEFT5,   _LSS_B2_LEFT),
        BLINK_RIGHT:  (_BM_RIGHT,   _LSS_B2_RIGHT),
        BLINK_HAZARD: (_BM_HAZARD,  _LSS_B2_NEUTRAL),
        BLINK_TIP_L:  (_BM_LEFT,    _LSS_B2_LEFT),
        BLINK_TIP_R:  (_BM_RIGHT1,  _LSS_B2_RIGHT),
    }

    MESSAGES = [
        #(0x52B, "LSS_01",       50,  list(_SEQ_52B[0])),
        (0x366, "Blinkmodi_02", 1000, list(_BM_IDLE)),  # 1000ms confirmed in log
        #(0x3D4, "Blink_Anf_01", 50,  list(_SEQ_3D4[0])),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._blink_state = self.BLINK_OFF
        self._tip_timer   = None
        self._idx_52b     = 0   # index into _SEQ_52B replay
        self._idx_3d4     = 0
        self._cmd_lock    = threading.Lock()

    # ── Public command interface ──────────────────────────────────────────────

    def send_command(self, cmd: str):
        """Accept blinker commands from the nav_controller panel."""
        with self._cmd_lock:
            self._cancel_tip()
            if   cmd == "blink_left":    self._set_blink(self.BLINK_LEFT)
            elif cmd == "blink_right":   self._set_blink(self.BLINK_RIGHT)
            elif cmd == "blink_off":     self._set_blink(self.BLINK_OFF)
            elif cmd == "blink_hazard":  self._set_blink(self.BLINK_HAZARD)
            elif cmd == "tip_left":
                self._set_blink(self.BLINK_TIP_L)
                self._schedule_tip_off(flashes=3)
            elif cmd == "tip_right":
                self._set_blink(self.BLINK_TIP_R)
                self._schedule_tip_off(flashes=3)

    # ── Blinker state management ──────────────────────────────────────────────

    def _set_blink(self, state: str):
        self._blink_state = state
        bm_payload, lss_b2 = self._STATE_MAP.get(state, (_BM_IDLE, _LSS_B2_NEUTRAL))
        # Update 0x366 Blinkmodi_02
        self._write_state(0x366, bm_payload)
        # Patch the direction byte in the current LSS replay frame.
        # The replay provides a valid CRC'd frame; we swap byte 2 (direction)
        # while keeping CRC/BZ from the replay — this may invalidate CRC for
        # non-neutral positions, but 0x52B is currently commented out of MESSAGES.
        current_lss = bytearray(_SEQ_52B[self._idx_52b % len(_SEQ_52B)])
        current_lss[2] = lss_b2
        self._write_state(0x52B, bytes(current_lss))

    def _schedule_tip_off(self, flashes: int):
        # 200ms Blinkmodi cycle × flashes + 150ms margin
        delay = flashes * 0.200 + 0.150
        self._tip_timer = threading.Timer(delay, self._tip_off)
        self._tip_timer.daemon = True
        self._tip_timer.start()

    def _tip_off(self):
        with self._cmd_lock:
            if self._blink_state in (self.BLINK_TIP_L, self.BLINK_TIP_R):
                self._set_blink(self.BLINK_OFF)

    def _cancel_tip(self):
        if self._tip_timer and self._tip_timer.is_alive():
            self._tip_timer.cancel()
        self._tip_timer = None

    # ── State helpers ─────────────────────────────────────────────────────────

    def _write_state(self, arb_id: int, data: bytes):
        for s in self._states:
            if s.arb_id == arb_id:
                with self._lock:
                    s.data = data
                break

    def _read_state(self, arb_id: int):
        for s in self._states:
            if s.arb_id == arb_id:
                return s.data
        return None

    # ── ECU lifecycle ─────────────────────────────────────────────────────────

    def set_enabled(self, on: bool):
        self.enabled = on
        if not on:
            self._cancel_tip()
            with self._cmd_lock:
                self._set_blink(self.BLINK_OFF)

    # ── Frame generation ──────────────────────────────────────────────────────

    def _next_frame(self, arb_id: int):
        if arb_id == 0x52B:
            # Advance through the 8-frame replay sequence.
            # Byte 0 is an opaque CRC with unknown key — replayed verbatim.
            # Byte 1 = 0x20 | BZ (upper nibble constant 0x2, lower nibble = counter).
            frame = _SEQ_52B[self._idx_52b % len(_SEQ_52B)]
            self._idx_52b += 1
            return bytes(frame)
        if arb_id == 0x3D4:
            d = _SEQ_3D4[self._idx_3d4 % len(_SEQ_3D4)]
            self._idx_3d4 += 1
            return d
        return None  # 0x366 uses whatever is in s.data (set by _set_blink)

    def _run(self):
        now = time.monotonic()
        next_tx = {s.arb_id: now + s.interval_ms / 1000.0 for s in self._states}
        while not self._stop_evt.is_set():
            now = time.monotonic()
            soonest = 0.010
            for s in self._states:
                due = next_tx[s.arb_id]
                if now >= due:
                    frame = self._next_frame(s.arb_id)
                    if frame is not None:
                        with self._lock:
                            s.data = frame
                    self._enqueue(s.arb_id, s.data)
                    with self._lock:
                        s.tx_count += 1
                        s.last_tx = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0
                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait
            self._stop_evt.wait(timeout=max(0.001, soonest))
