"""
modules/a5_drvassist.py — A5 Driver Assistance (Front Camera / VZE)

Sends:
  0x181  VZE_01  — Traffic sign display (speed limits etc),   200ms  [editable via UI]
  0x29C  VZE_02  — Traffic sign display 2 (signs 4-5),        200ms  [static]
  0x1F0  EA_02   — Front Assist / object detection / eCall,    50ms  [replay]
  0x30F  SWA_01  — Lane keep / lane change assist (SWA/LKA),  100ms  [replay]
  0x395  LWR_AFS_01 — AFS / headlight levelling (BO_ 917),     100ms  [DBC pack + rolling BZ]
  0x397  HC_01   — (placeholder),                              80ms  [replay]

VZE_01 (0x181) layout from DBC MLBevo_Gen2: BO_ 385 VZE_01: 8 Gateway.
LWR_AFS_01 (0x395): BO_ 917, ICAN V8.19.05F; Intel (@1+) — see pack_lwr_afs_01.
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


def _lwr_rd(data: bytes, start: int, length: int) -> int:
    v = 0
    for i in range(length):
        b = start + i
        if data[b // 8] & (1 << (b % 8)):
            v |= 1 << i
    return v


def _lwr_wr(buf: bytearray, start: int, length: int, val: int) -> None:
    val &= (1 << length) - 1
    for i in range(length):
        if (val >> i) & 1:
            b = start + i
            buf[b // 8] |= 1 << (b % 8)


# BO_ 917 LWR_AFS_01 — (name, start_bit, bit_length); Intel @1+ (MLBevo ICAN DBC).
_LWR_AFS_SPEC = (
    ("afs_abbiegelicht_li_anf", 0, 1),
    ("afs_abbiegelicht_re_anf", 1, 1),
    ("afs_abbiegelicht_dimm_anf", 2, 1),
    ("afs_verfuegbarkeit_glw", 3, 1),
    ("afs_status_adaptive_lv", 4, 1),
    ("afs_fernlicht_li_defekt", 5, 1),
    ("afs_fernlicht_re_defekt", 6, 1),
    ("afs_fernlicht_status", 7, 1),
    ("lwr_afs_bz", 8, 4),
    ("afs_lampe", 12, 1),
    ("lwr_lampe", 13, 1),
    ("afs_fehlertext", 14, 1),
    ("afs_led_blinkmode", 15, 1),
    ("lwr_touristenmodus_aktiv", 16, 1),
    ("afs_charisma_fahrpr", 17, 4),
    ("afs_charisma_status", 21, 2),
    ("lwr_sicherheitsposition_re", 23, 1),
    ("afs_allwetterlicht", 24, 1),
    ("lwr_touristenmodus_verbaut", 25, 1),
    ("afs_adaptive_lv_verbaut", 26, 1),
    ("lwr_reisemodus_texte", 27, 2),
    ("glw_sensor_blockiert", 29, 1),
    ("afs_charisma_umschaltung", 30, 2),
    ("lwr_pos_schrittmotor_raw", 32, 8),
    ("lwr_hoehenwert_vl_raw", 40, 8),
    ("lwr_hoehenwert_hl_raw", 48, 8),
    ("afs_ersatzlicht_links", 56, 1),
    ("afs_ersatzlicht_rechts", 57, 1),
    ("lwr_afs_grundeinstellung", 58, 1),
    ("lwr_afs_parametrisierung", 59, 1),
    ("lwr_initialisierungslauf", 60, 1),
    ("lwr_sicherheitsposition_li", 61, 1),
    ("lwr_modus", 62, 1),
)

# Decoded from log template 14 04 20 04 FE 65 65 0C (nominal AFS/LWR idle).
_LWR_AFS_DEFAULTS = {name: _lwr_rd(bytes.fromhex("14042004fe65650c"), s, ln) for name, s, ln in _LWR_AFS_SPEC}


def pack_lwr_afs_01(**kwargs) -> list:
    """
    Build LWR_AFS_01 (0x395) payload. Pass any field from _LWR_AFS_SPEC by name;
    ``lwr_afs_bz`` is the 4-bit rolling counter (0..15).
    Raw heights: DBC LWR_Hoehenwert_* = (1,-127) → physical ≈ raw - 127.
    LWR_Pos_Schrittmotor: DBC (0.4,0) → physical % ≈ raw * 0.4.
    """
    vals = dict(_LWR_AFS_DEFAULTS)
    vals.update(kwargs)
    buf = bytearray(8)
    for name, start, ln in _LWR_AFS_SPEC:
        _lwr_wr(buf, start, ln, int(vals[name]))
    return list(buf)


def unpack_lwr_afs_01(data) -> dict:
    """Decode 0x395 / BO_ 917 to signal dict (raw integer per DBC)."""
    if not data or len(data) < 8:
        return {}
    d = bytes(data[:8])
    return {name: _lwr_rd(d, s, ln) for name, s, ln in _LWR_AFS_SPEC}


# EA_02 (0x1F0): byte0=CRC, byte1=rolling counter 0–15, bytes 2–7=payload.
# CRC LUT from log replay; MQB uses per-message CRC-8 with unknown padding.
_EA_02_CRC = [0xD1, 0x6A, 0xDD, 0x01, 0x7E, 0xB3, 0xAB, 0xF2,
              0xFF, 0x6F, 0x3B, 0x7B, 0xB7, 0x0E, 0xBF, 0x24]
_EA_02_PAYLOAD = bytes.fromhex("400000000000")

# SWA_01 (0x30F): byte0=CRC, byte1=counter (0,4,8,12), bytes 2–7=payload.
_SWA_01_CRC = [0x5B, 0xE4, 0x0A, 0xB5]  # counter 0,4,8,12
_SWA_01_PAYLOAD = bytes.fromhex("720000000100")

# HC_01 (0x397): 3-frame base, byte 5 editable from UI.
_HC_01_BASE = [
    [0x00, 0x40, 0x00, 0x00, 0x50, 0xFA, 0x1F, 0x00],
    [0x00, 0x40, 0x10, 0x00, 0x50, 0xFA, 0x1E, 0x00],
    [0x00, 0x40, 0x00, 0x00, 0x50, 0xFA, 0x1F, 0x00],
]


class VzeECU(ECUModule):
    ECU_ID   = "A5"
    ECU_NAME = "Driver Assistance"

    MESSAGES = [
        (0x181, "VZE_01",  200, pack_vze_01()),  # editable from UI; DBC cycle 200ms
        (0x29C, "VZE_02",  200, [0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00]),  # log: 200ms
        (0x1F0, "EA_02",    50, [_EA_02_CRC[0], 0, *_EA_02_PAYLOAD]),
        (0x30F, "SWA_01",  100, [_SWA_01_CRC[0], 0, *_SWA_01_PAYLOAD]),
        (0x395, "LWR_AFS_01", 100, pack_lwr_afs_01(lwr_afs_bz=0)),
        (0x397, "HC_01",    80, [0x00, 0x40, 0x00, 0x00, 0x50, 0xFA, 0x1F, 0x00]),  # byte 4 editable
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._ctr_1f0 = 0
        self._ctr_30f = 0
        self._lwr_bz = 0
        self._idx_hc = 0
        self._hc_byte4 = 0x50

    def set_enabled(self, on: bool):
        self.enabled = on

    def set_hc_byte4(self, val: int):
        self._hc_byte4 = max(0, min(0xFF, int(val))) & 0xFF

    def get_hc_byte4(self) -> int:
        return self._hc_byte4

    def _next_frame(self, arb_id):
        if arb_id == 0x1F0:
            ctr = self._ctr_1f0 % 16
            self._ctr_1f0 += 1
            return bytes([_EA_02_CRC[ctr], ctr, *_EA_02_PAYLOAD])
        if arb_id == 0x30F:
            ctr_idx = self._ctr_30f % 4
            ctr_val = ctr_idx * 4  # 0, 4, 8, 12
            self._ctr_30f += 1
            return bytes([_SWA_01_CRC[ctr_idx], ctr_val, *_SWA_01_PAYLOAD])
        if arb_id == 0x395:
            bz = self._lwr_bz % 16
            self._lwr_bz += 1
            return bytes(pack_lwr_afs_01(lwr_afs_bz=bz))
        if arb_id == 0x397:
            base = _HC_01_BASE[self._idx_hc % len(_HC_01_BASE)]
            self._idx_hc += 1
            frame = list(base)
            frame[4] = self._hc_byte4
            return bytes(frame)
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
                            s.data = list(frame)
                    self._enqueue(s.arb_id, s.next_payload())
                    with self._lock:
                        s.tx_count += 1
                        s.last_tx   = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0

                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            self._stop_evt.wait(timeout=max(0.001, soonest))
