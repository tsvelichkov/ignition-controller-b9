"""
modules/02_autotrans.py — Automatic transmission / WBA_03 (0x394) emulation.

DBC: MLBevo HCAN K-Matrix, BO_ 916 WBA_03 (arbitration ID 0x394), cycle 160 ms,
     sender Gateway. VAL_ tables define WBA_Fahrstufe_02 (shifter) and
     WBA_eing_Gang_02 / GE_Sollgang (displayed / recommended gear).

Signals use Intel (little-endian) placement per DBC; nibbles align to:
  byte1: WBA_03_BZ (low) + WBA_Fahrstufe_02 (high)
  byte2 low: WBA_ZielFahrstufe
  byte3 low: WBA_eing_Gang_02
  byte5 low: GE_Sollgang

CRC (byte0): CRC-8 / AUTOSAR H2F (polynomial 0x2F, init 0xFF, xorout 0xFF) over
bytes 1–7 of the payload, then one extra byte appended for the calculation only:
the Data ID from DBC attribute GenMsgPDUConstants for BO_ 916, indexed by
WBA_03_BZ (low nibble of byte 1). This matches all 16 captured log templates.

Baseline 16-frame BZ sequence was taken from the same log as modules/03_esp.py.
"""
from __future__ import annotations

import time

from ecu_base import ECUModule

WBA_03_ARB_ID = 0x394

# WBA_Fahrstufe_02 (VAL_ 916)
FAHR_P = 1
FAHR_R = 2
FAHR_N = 3
FAHR_D = 4
FAHR_S = 5

# BA_ "GenMsgPDUConstants" BO_ 916 (MLBevo DBC) — Data ID byte for CRC, per BZ 0..15
_WBA_PDU_DATA_IDS = bytes(
    [0x47, 0x94, 0x92, 0x6A, 0x67, 0xB5, 0x0D, 0x38, 0xE3, 0x8A, 0x5D, 0xB4, 0x54, 0xAB, 0xAE, 0x27]
)


def _crc8_autosar_h2f(data: bytes) -> int:
    """CRC-8 / AUTOSAR (H2F), poly 0x2F, init 0xFF, xorout 0xFF."""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x2F) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


def wba_03_compute_crc(payload: list[int] | bytes) -> int:
    """
    WBA_03 CRC byte for an 8-byte payload. Uses BZ = payload[1] & 0x0F to pick
    the Data ID from the DBC GenMsgPDUConstants table.
    """
    p = list(payload) if not isinstance(payload, list) else payload
    if len(p) < 8:
        return 0
    bz = p[1] & 0x0F
    crc_in = bytes(p[1:8]) + bytes([_WBA_PDU_DATA_IDS[bz]])
    return _crc8_autosar_h2f(crc_in)


# WBA_03 templates: verbatim from 0000058-nav log (previously in 03_esp.py)
_WBA_TEMPLATES_16 = [
    [0x46, 0x16, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x6C, 0x17, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xA3, 0x18, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x2A, 0x19, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xE0, 0x1A, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x8A, 0x1B, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xEF, 0x1C, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xC9, 0x1D, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xF6, 0x1E, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x09, 0x1F, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xDE, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x60, 0x11, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x2E, 0x12, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0xC5, 0x13, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x12, 0x14, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
    [0x83, 0x15, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00],
]


def decode_wba_03(data: list[int] | bytes) -> dict | None:
    """Decode WBA_03 / 0x394 payload per MLBevo DBC (Intel bit numbering)."""
    b = list(data) if not isinstance(data, list) else data
    if len(b) < 8:
        return None
    return {
        "WBA_03_CRC": b[0],
        "WBA_03_BZ": b[1] & 0x0F,
        "WBA_Fahrstufe_02": (b[1] >> 4) & 0x0F,
        "WBA_ZielFahrstufe": b[2] & 0x0F,
        "WBA_GE_Warnung_02": (b[2] >> 4) & 0x0F,
        "WBA_eing_Gang_02": b[3] & 0x0F,
        "WBA_GE_Texte": (b[3] >> 4) & 0x07,
        "WBA_Segeln_aktiv": (b[3] >> 7) & 0x01,
        "WBA_Schaltschema": b[4] & 0x1F,
        "WBA_GE_Zusatzwarnungen": (b[4] >> 5) & 0x07,
        "GE_Sollgang": b[5] & 0x0F,
        "GE_Tipschaltempf_verfuegbar": (b[5] >> 4) & 0x01,
        "WBA_GE_Texte_02": (b[5] >> 5) & 0x07,
        "WBA_GE_Texte_03": b[6] & 0x0F,
        "WBA_Blinken": (b[6] >> 4) & 0x01,
        "GE_Wiederstart_Anz_Std": (b[6] >> 5) & 0x01,
        "GE_Stoppverbot_Anz_01": (b[6] >> 6) & 0x01,
        "GE_Stoppverbot_Anz_02": (b[6] >> 7) & 0x01,
        "GE_Stoppverbot_Anz_03": b[7] & 0x01,
        "GE_Stoppverbot_Anz_04": (b[7] >> 1) & 0x01,
        "GE_Stoppverbot_Anz_05": (b[7] >> 2) & 0x01,
        "GE_Stoppverbot_Anz_06": (b[7] >> 3) & 0x01,
        "GE_Stoppverbot_Anz_07": (b[7] >> 4) & 0x01,
        "GE_Stoppverbot_Anz_Std": (b[7] >> 5) & 0x01,
    }


class AutotransECU(ECUModule):
    ECU_ID = "02"
    ECU_NAME = "Autotrans"
    decode_wba_03 = staticmethod(decode_wba_03)
    compute_crc = staticmethod(wba_03_compute_crc)

    MESSAGES = [
        (WBA_03_ARB_ID, "WBA_03", 160, [list(f) for f in _WBA_TEMPLATES_16]),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._fahr = FAHR_P
        self._ziel = 0
        self._eing = 0
        self._soll = 0
        self._bz_i = 0

    def set_enabled(self, on: bool):
        """Match other powertrain-related modules — gated with KL15."""
        self.enabled = bool(on)

    def set_shifter_gear(self, fahrstufe: int, eing_gang: int, soll_gang: int | None = None):
        """
        fahrstufe: WBA_Fahrstufe_02 (1=P, 2=R, 3=N, 4=D, 5=S, … per DBC).
        eing_gang: WBA_eing_Gang_02 (0=no display, 1–7 = gear 1–7).
        soll_gang: GE_Sollgang (default: same as eing_gang).
        """
        self._fahr = max(0, min(15, int(fahrstufe)))
        self._eing = max(0, min(15, int(eing_gang)))
        self._soll = self._eing if soll_gang is None else max(0, min(15, int(soll_gang)))
        if self._fahr == FAHR_D:
            self._ziel = 2
        elif self._fahr == FAHR_S:
            self._ziel = 1
        else:
            self._ziel = 0

    def _pack_from_template(self, tmpl: list[int]) -> list[int]:
        buf = list(tmpl)
        bz = buf[1] & 0x0F
        buf[1] = ((self._fahr & 0x0F) << 4) | bz
        buf[2] = (buf[2] & 0xF0) | (self._ziel & 0x0F)
        buf[3] = (buf[3] & 0xF0) | (self._eing & 0x0F)
        buf[5] = (buf[5] & 0xF0) | (self._soll & 0x0F)
        buf[0] = wba_03_compute_crc(buf)
        return buf

    def _run(self):
        now = time.monotonic()
        next_tx = {s.arb_id: now + s.interval_ms / 1000.0 for s in self._states}

        while not self._stop_evt.is_set():
            now = time.monotonic()
            soonest = 0.010

            for s in self._states:
                due = next_tx[s.arb_id]
                if now >= due:
                    if s.is_multi:
                        idx = self._bz_i % len(s.data)
                        tmpl = s.data[idx]
                        self._bz_i = (self._bz_i + 1) % len(s.data)
                        out = self._pack_from_template(tmpl)
                        with self._lock:
                            s._frame_idx = idx
                    else:
                        out = self._pack_from_template(s.data)

                    self._enqueue(s.arb_id, out)
                    with self._lock:
                        s.tx_count += 1
                        s.last_tx = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0

                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            self._stop_evt.wait(timeout=max(0.001, soonest))
