"""
modules/09_bcm.py — BCM module 09 emulation

Sends:
  0x2A0  BCM_02          — BCM crash/sunroof/door status,        160ms  [CRC+BZ, BO_ 672]
  0x366  Blinkmodi_02    — Turn signal / alarm / DWA,           1000ms  [static]
  0x3C1  BCM_Taster_01   — Button states (PLA/ESP/HDC/AVH),       50ms  [replay]
  0x3D4  SMLS_01         — Stalk / MFA inputs (ICAN BO_ 980),    200ms  [CRC+BZ]
  0x3D5  Licht_Anf_01    — Lighting requests (dip/high/fog),     100ms  [CRC+BZ, BO_ 981]
  0x3D6  Licht_hinten_01 — Rear lights status,                    100ms  [static]
  0x5A0  RLS_01          — Rain/light sensor (BO_ 1440),          200ms  [static]
  0x5F0  Dimmung_01      — Interior KL58 dimming,                200ms  [static]
  0x64F  BCM1_04         — HUD eyebox / KL58s / light warnings, 1000ms  [static]
  0x658  Licht_vorne_01  — Front light status,                  1000ms  [CRC+BZ, BO_ 1624]
  0x65A  BCM_01          — Fluid sensors / KL15 status,         1000ms  [static]

Source: 0000046.TXT + 0000050.TXT car logs. Verified against 0000027.TXT.
  SMLS_01: body 82 84 00 01 00 00 verified from 0000027.TXT
    WH_Intervall=1, WH_Intervallstufen=9, LRH_on_off=2, LRH_aktiv=1
  GenMsgPDUConstants for BO_ 672 / 981 / 980 / 1624 from MLBevo HCAN KMatrix
    (ICAN BO_ 980 has no GenMsgPDUConstants attribute; HCAN table used).

Layout: class + replay sequences first; E2E PDU tables, frame builders, and MESSAGES at file end.

Fixes applied (0000027.TXT audit):
  BCM_02       body b3-b5: E0 EF 0F → E8 CF 06
  SMLS_01      body b2-b5: 82 04 00 00 → 82 84 00 01
  Licht_Anf_01 b1 upper nibble: 0xC0|bz → 0x00|bz; body b2-b4: 40 00 04 → 00 04 04
  Licht_vorne_01 b1 upper nibble: 0x30|bz → 0x00|bz; body b2: 00 → 06; removed 0x20/0x02
  Licht_hinten_01 static: 00 01 00… → 20 08 00…
  BCM1_04      static: 00 00 C0 A0… → 00 00 00 C6 02…
  BCM_01       static b3/b5: 38/2E → 10/20
  RLS_01       static: 00 1D 80 20 78… → 1E 7C 82 2E 78…
  Dimmung_01   static: 52 D2 52 AE 00 7E… → E5 00 5D E8 0E 7E…
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule


def _crc8_autosar_h2f(data: bytes) -> int:
    """CRC-8 / AUTOSAR H2F (poly 0x2F, init 0xFF, xorout 0xFF)."""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x2F) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


class BcmECU(ECUModule):
    ECU_ID   = "09"
    ECU_NAME = "BCM"
    # MESSAGES is set at bottom of this file (after E2E GenMsgPDUConstants + frame builders).

    # ── Replay sequences (no HCAN GenMsgPDUConstants / unclear E2E): 0x3C1 only ─

    _SEQ_3C1 = [
        bytes.fromhex("0000080ab6424400"),
        bytes.fromhex("0000080aba424400"),
        bytes.fromhex("0000080abe424400"),
        bytes.fromhex("0000080a82424400"),
        bytes.fromhex("0000080a86424400"),
        bytes.fromhex("0000080a8a424400"),
        bytes.fromhex("0000080a8e424400"),
        bytes.fromhex("0000080a92424400"),
        bytes.fromhex("0000080a96424400"),
        bytes.fromhex("0000080a9a424400"),
        bytes.fromhex("0000080a9e424400"),
        bytes.fromhex("0000080aa2424400"),
        bytes.fromhex("0000080aa6424400"),
        bytes.fromhex("0000080aaa424400"),
        bytes.fromhex("0000080aae424400"),
        bytes.fromhex("0000080ab2424400"),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._idx_2a0 = 0
        self._idx_3c1 = 0
        self._idx_3d4 = 0
        self._idx_3d5 = 0
        self._idx_658 = 0

    def set_enabled(self, on: bool):
        self.enabled = on

    def set_dimming(self, kl58d: int, display_pct: int) -> None:
        """
        kl58d:       DI_KL_58xd  0–253  — raw dimmer-dial / outside-light level (byte 0)
        display_pct: 0–100 %    — DI_KL_58xt (Dimmung_01 byte 2 bits 0-6)
                                  and BCM1_Stellgroesse_Kl_58s (BCM1_04 byte 3 bits 1-7)

        Log observations across 4 captures:
          DI_KL_58xs  (byte 1 bits 0-6)  is ALWAYS 0  → do not touch
          BCM1_Stellgroesse in Dimmung_01 (byte 5)    is ALWAYS 0x7E=126 → do not touch
          DI_KL_58xt  (byte 2 bits 0-6)  tracks display brightness (87/91/94 %)
          BCM1_Stellgroesse in BCM1_04   (byte 3 bits 1-7) tracks brightness (86/91/98/99)
        """
        kl58d = max(0, min(253, int(kl58d)))
        pct   = max(0, min(100, int(display_pct)))
        with self._lock:
            for s in self._states:
                if s.arb_id == 0x5F0:          # Dimmung_01
                    d = list(s.data)
                    d[0] = kl58d                               # DI_KL_58xd
                    d[2] = (d[2] & 0x80) | (pct & 0x7F)       # DI_KL_58xt (display %)
                    s.data = d
                elif s.arb_id == 0x64F:        # BCM1_04
                    d = list(s.data)
                    d[3] = (d[3] & 0x01) | ((pct & 0x7F) << 1)  # BCM1_Stellgroesse_Kl_58s
                    s.data = d

    def set_light_sensor(self, fw_lux: int) -> None:
        """
        fw_lux: LS_Helligkeit_FW in Lux (0–6126 per DBC, practical range 0–4000).

        Updates two messages atomically under the lock:
          RLS_01 (0x5A0)
            byte 0: LS_Helligkeit_IR (factor 400 Lux) — estimated from FW
            byte 1: LS_Helligkeit_FW low 8 bits  (factor 6 Lux, 10-bit value)
            byte 2 bits 0-1: LS_Helligkeit_FW high 2 bits (preserve LS_Verbau/defekt)
          Dimmung_01 (0x5F0)
            bytes 3-4: DI_Fotosensor — identical to LS_Helligkeit_FW raw per log analysis

        Log reference (raw_fw → IR):  21→0, 80→3, 371→50, 611→32.
        IR ≈ raw_fw / 7 is a reasonable approximation for the mid-range.
        """
        fw_lux = max(0, min(6126, int(fw_lux)))
        raw_fw = fw_lux // 6                           # 10-bit raw value, 0–1021
        raw_ir = min(253, max(0, raw_fw // 7))         # byte 0 estimate
        with self._lock:
            for s in self._states:
                if s.arb_id == 0x5A0:                  # RLS_01
                    d = list(s.data)
                    d[0] = raw_ir
                    d[1] = raw_fw & 0xFF
                    d[2] = (d[2] & 0xFC) | ((raw_fw >> 8) & 0x03)
                    s.data = d
                elif s.arb_id == 0x5F0:                # Dimmung_01 — DI_Fotosensor
                    d = list(s.data)
                    d[3] = raw_fw & 0xFF
                    d[4] = (raw_fw >> 8) & 0xFF
                    s.data = d

    def _next_frame(self, arb_id):
        if arb_id == 0x2A0:
            bz = self._idx_2a0 % 16
            self._idx_2a0 += 1
            return _build_bcm_02(bz)
        if arb_id == 0x3C1:
            d = self._SEQ_3C1[self._idx_3c1 % len(self._SEQ_3C1)]
            self._idx_3c1 += 1
            return list(d)
        if arb_id == 0x3D4:
            bz = self._idx_3d4 % 16
            self._idx_3d4 += 1
            return _build_smls_01(bz)
        if arb_id == 0x3D5:
            bz = self._idx_3d5 % 16
            self._idx_3d5 += 1
            return _build_licht_anf_01(bz)
        if arb_id == 0x658:
            bz = self._idx_658 % 16
            self._idx_658 += 1
            return _build_licht_vorne_01(bz)
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


# =============================================================================
# E2E CRC — GenMsgPDUConstants from MLBevo HCAN KMatrix (16 Data-ID bytes per
# message, indexed by BZ 0..15). Not part of the 8-byte CAN payload.
# All tables verified via CRC-8/AUTOSAR H2F against 0000027.TXT log frames.
# =============================================================================

# BO_ 980 SMLS_01 — ICAN has no attribute; table from HCAN_V8.18.01F
_SMLS_PDU_DATA_IDS = bytes(
    [0xC3, 0x79, 0xBF, 0xDB, 0xE9, 0x11, 0x46, 0x86, 0x69, 0xB6, 0x9B, 0x29, 0x15, 0x9C, 0x45, 0x0D]
)

# BO_ 672 BCM_02, BO_ 981 Licht_Anf_01, BO_ 1624 Licht_vorne_01
_BCM02_PDU_DATA_IDS = bytes(
    [0x6A, 0x65, 0x16, 0x26, 0xD9, 0x99, 0xD1, 0x84, 0xF3, 0x9C, 0xA4, 0xAA, 0x07, 0xEA, 0xC2, 0xCB]
)
_LICHT_ANF_PDU_DATA_IDS = bytes(
    [0xC5, 0x39, 0xC7, 0xF9, 0x92, 0xD8, 0x24, 0xCE, 0xF1, 0xB5, 0x7A, 0xC4, 0xBC, 0x60, 0xE3, 0xD1]
)
_LICHT_VORNE_PDU_DATA_IDS = bytes(
    [0x46, 0x0F, 0xE0, 0xCC, 0xBE, 0xB1, 0xD7, 0xF1, 0x1F, 0x31, 0xC1, 0x25, 0xB4, 0x3E, 0xC8, 0xA3]
)


def _e2e_crc(payload: list, pdu: bytes) -> int:
    """CRC-8/AUTOSAR over bytes[1..7] + Data-ID pdu[BZ], where BZ = payload[1] & 0x0F."""
    bz = payload[1] & 0x0F
    return _crc8_autosar_h2f(bytes(payload[1:8]) + bytes([pdu[bz]]))


def _build_smls_01(bz: int) -> list:
    """
    SMLS_01 (0x3D4, BO_ 980): rolling BZ in byte1 (upper nibble = 0); CRC in byte0.

    Body verified from 0000027.TXT (904/1060 frames dominant body):
      b2=0x80: WH_Intervall=0 (dominant idle; 0x82 = WH_Intervall=1 appears in 128 transient frames)
      b3=0x84: WH_Intervallstufen=9 (bits 23-26: b2[7]=1, b3[0..2]=100)
               LRH_on_off=2 at bits 30-31 = b3[6..7] = 0x84>>6 = 2 ✓
      b4=0x00
      b5=0x01: LRH_aktiv=1 at bit 40 = b5[0] ✓
    """
    bz &= 0x0F
    payload = [0x00, bz, 0x80, 0x84, 0x00, 0x01, 0x00, 0x00]
    payload[0] = _e2e_crc(payload, _SMLS_PDU_DATA_IDS)
    return payload


def _build_bcm_02(bz: int) -> list:
    """
    BCM_02 (0x2A0, BO_ 672): rolling BZ in byte1 (upper nibble = 0); CRC in byte0.

    Body verified from 0000027.TXT (1030/1032 frames):
      b2=0x80, b3=0xE8, b4=0xCF, b5=0x06
    """
    bz &= 0x0F
    payload = [0x00, bz, 0x80, 0xE8, 0xCF, 0x06, 0x00, 0x00]
    payload[0] = _e2e_crc(payload, _BCM02_PDU_DATA_IDS)
    return payload


def _build_licht_anf_01(bz: int) -> list:
    """
    Licht_Anf_01 (0x3D5, BO_ 981): rolling BZ in byte1 (upper nibble = 0); CRC in byte0.

    Body verified from 0000027.TXT (2062/2063 frames identical):
      b2=0x00, b3=0x04 (low-beam request bit), b4=0x04
    FIX: upper nibble was incorrectly 0xC; log confirms 0x0.
    """
    bz &= 0x0F
    payload = [0x00, bz, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00]
    payload[0] = _e2e_crc(payload, _LICHT_ANF_PDU_DATA_IDS)
    return payload


def _build_licht_vorne_01(bz: int) -> list:
    """
    Licht_vorne_01 (0x658, BO_ 1624): rolling BZ in byte1 (upper nibble = 0); CRC in byte0.

    Body verified from 0000027.TXT (206/207 frames):
      b2=0x06 (low-beam + DRL status bits), b3-b7=0x00
    FIX: upper nibble was incorrectly 0x3; b2 was 0x00, 0x20/0x02 were in wrong positions.
    """
    bz &= 0x0F
    payload = [0x00, bz, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00]
    payload[0] = _e2e_crc(payload, _LICHT_VORNE_PDU_DATA_IDS)
    return payload


# ── Static payloads — all verified from 0000027.TXT dominant frames ──────────

# RLS_01 (0x5A0, BO_ 1440): rain/light sensor.
# b0/b1 vary with ambient light level (noise); dominant pattern used as base.
# Physical: LS_Helligkeit nibbles in b0/b1 indicate daytime bright conditions.
_RLS_01_PAYLOAD = [0x1E, 0x7C, 0x82, 0x2E, 0x78, 0x00, 0x00, 0x00]

# Dimmung_01 (0x5F0, BO_ 1520): KL58 interior dimming level.
# b0/b3 vary slightly with dimmer position; dominant value used.
_DIMMUNG_01_PAYLOAD = [0xE5, 0x00, 0x5D, 0xE8, 0x0E, 0x7E, 0x00, 0x00]

# Licht_hinten_01 (0x3D6, BO_ 982): rear light status — single unique payload in log.
_LICHT_HINTEN_01_PAYLOAD = [0x20, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# BCM1_04 (0x64F, BO_ 1615): HUD eyebox / KL58 / light warnings.
# FIX: was [0x00, 0x00, 0xC0, 0xA0, 0x00, 0x00, 0x00, 0x00] — bytes 2-4 were shifted/wrong.
_BCM1_04_PAYLOAD = [0x00, 0x00, 0x00, 0xC6, 0x02, 0x00, 0x00, 0x00]

# BCM_01 (0x65A, BO_ 1626): fluid sensors / KL15 status.
# FIX: b3 0x38→0x10, b5 0x2E→0x20 (fuel level / brake fluid nibbles).
_BCM_01_PAYLOAD = [0x00, 0x00, 0x06, 0x10, 0x00, 0x20, 0x00, 0x00]


# ── Message table ─────────────────────────────────────────────────────────────
BcmECU.MESSAGES = [
    (0x2A0, "BCM_02",         160,  _build_bcm_02(0)),
    (0x366, "Blinkmodi_02",  1000,  [0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xF0]),
    (0x3C1, "BCM_Taster_01",   50,  [0x00, 0x00, 0x08, 0x0A, 0xB6, 0x42, 0x44, 0x00]),
    (0x3D4, "SMLS_01",        200,  _build_smls_01(0)),
    (0x3D5, "Licht_Anf_01",   100,  _build_licht_anf_01(0)),
    (0x3D6, "Licht_hinten_01",100,  list(_LICHT_HINTEN_01_PAYLOAD)),
    (0x5A0, "RLS_01",         200,  list(_RLS_01_PAYLOAD)),
    (0x5F0, "Dimmung_01",     200,  list(_DIMMUNG_01_PAYLOAD)),
    (0x64F, "BCM1_04",       1000,  list(_BCM1_04_PAYLOAD)),
    (0x658, "Licht_vorne_01",1000,  _build_licht_vorne_01(0)),
    (0x65A, "BCM_01",        1000,  list(_BCM_01_PAYLOAD)),
]
