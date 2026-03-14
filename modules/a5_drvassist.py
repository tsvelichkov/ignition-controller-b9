"""
modules/a5_drvassist.py — A5 Driver Assistance (Front Camera / VZE)

Sends:
  0x181  VZE_01  — Traffic sign display (speed limits etc),   100ms  [editable via UI]
  0x29C  VZE_02  — Traffic sign display 2 (signs 4-5),        100ms  [static]
  0x1F0  EA_02   — Front Assist / object detection / eCall,    50ms  [replay]
  0x30F  SWA_01  — Lane keep / lane change assist (SWA/LKA),  100ms  [replay]

VZE_01 (0x181) layout from DBC MLBevo_Gen2: BO_ 385 VZE_01: 8 Gateway.
Intel (@1+) bit layout: start_bit|length.
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule


# Cluster encoding: value = speed * 8/5 (16-bit LE in byte1, byte2). Overrides to match observed: 25→0x2a, 155→0xfa.
_SPEED_TO_VALUE = {25: 42, 155: 250}
_VALUE_TO_SPEED = {42: 25, 250: 155}


def _speed_to_value(speed):
    speed = max(0, min(255, int(speed)))
    return _SPEED_TO_VALUE.get(speed, round(speed * 8 / 5))


def _value_to_speed(raw):
    raw = max(0, min(0xFFFF, int(raw)))
    return _VALUE_TO_SPEED.get(raw, round(raw * 5 / 8))


def pack_vze_01(anzeigemodus=0, verkehrszeichen_1=90, byte_4=0, byte_5=0, byte_6=0, byte_7=0, **kwargs):
    """Pack VZE_01 (0x181): byte0=sign type, byte1+byte2=speed 16-bit LE, bytes 4–7 from args or 0."""
    data = [0] * 8
    data[0] = max(0, min(3, int(anzeigemodus)))
    val = _speed_to_value(verkehrszeichen_1)
    data[1] = val & 0xFF
    data[2] = (val >> 8) & 0xFF
    data[4] = int(byte_4) & 0xFF
    data[5] = int(byte_5) & 0xFF
    data[6] = int(byte_6) & 0xFF
    data[7] = int(byte_7) & 0xFF
    return data


def unpack_vze_01(data):
    """Unpack 0x181 to anzeigemodus, verkehrszeichen_1, and bytes 4–7 for UI."""
    if not data or len(data) < 3:
        return {}
    d = list(data)[:8]
    raw = d[1] + (d[2] << 8)
    return {
        "anzeigemodus": d[0] & 3,
        "verkehrszeichen_1": max(0, min(255, _value_to_speed(raw))),
        "byte_4": d[4] if len(d) > 4 else 0,
        "byte_5": d[5] if len(d) > 5 else 0,
        "byte_6": d[6] if len(d) > 6 else 0,
        "byte_7": d[7] if len(d) > 7 else 0,
    }


class VzeECU(ECUModule):
    ECU_ID   = "A5"
    ECU_NAME = "Driver Assistance"

    _SEQ_1F0 = [
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("6a01400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("ab06400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("7b0b400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("d100400000000000"),
        bytes.fromhex("0103400000000000"),
        bytes.fromhex("b305400000000000"),
        bytes.fromhex("ff08400000000000"),
        bytes.fromhex("3b0a400000000000"),
        bytes.fromhex("0e0d400000000000"),
        bytes.fromhex("240f400000000000"),
        bytes.fromhex("dd02400000000000"),
        bytes.fromhex("7e04400000000000"),
        bytes.fromhex("f207400000000000"),
        bytes.fromhex("6f09400000000000"),
        bytes.fromhex("b70c400000000000"),
        bytes.fromhex("bf0e400000000000"),
        bytes.fromhex("6a01400000000000"),
    ]

    _SEQ_30F = [
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
        bytes.fromhex("e404720000000100"),
        bytes.fromhex("0a08720000000100"),
        bytes.fromhex("b50c720000000100"),
        bytes.fromhex("5b00720000000100"),
    ]

    MESSAGES = [
        (0x181, "VZE_01",  200, pack_vze_01()),  # editable from UI; DBC cycle 200ms
        (0x29C, "VZE_02",  200, [0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00]),  # log: 200ms
        (0x1F0, "EA_02",    50, [0xB7, 0x0C, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]),
        (0x30F, "SWA_01",  100, [0xB5, 0x0C, 0x72, 0x00, 0x00, 0x00, 0x01, 0x00]),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._idx_1f0 = 0
        self._idx_30f = 0

    def set_enabled(self, on: bool):
        self.enabled = on

    def _next_frame(self, arb_id):
        if arb_id == 0x1F0:
            d = self._SEQ_1F0[self._idx_1f0 % len(self._SEQ_1F0)]
            self._idx_1f0 += 1
            return d
        if arb_id == 0x30F:
            d = self._SEQ_30F[self._idx_30f % len(self._SEQ_30F)]
            self._idx_30f += 1
            return d
        return None

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
                        s.last_tx   = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0

                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            self._stop_evt.wait(timeout=max(0.001, soonest))
