"""
modules/19_gateway.py — Gateway ECU emulation

Always-on (ignition-independent):
  0x6B2         Diagnose_01    — live ticking clock, 1000ms
  0x37B         GNSS_05        — GNSS_UTC_Zeit + receiver flags (DBC BO_ 891), 1000ms; not cluster Uhrzeit_01 (0x6B6)
  0x663         NVEM_02        — 100ms
  0x16A95414    NVEM_06        — 100ms  [16-frame replay]
  0x3E5         TSG_FT_02      — 100ms, four doors (DBC FT/BFS/HBFS/HFS), CRC+BZ
  0x3CF         TSG_HBFS_01    — 100ms, hatch open bit (not on 0x3E5)
  0x583         ZV_02          — 200ms, BO_ 1411: ZV_*_offen bits (MIB/HUD door overview; DBC mirrors TSG)

  NM keepalives (29-bit, ~100ms in log):
    0x1B000010  NM_Gateway     — own node
    0x1B000014  NM_node_14     — Motor/ABS proxy
    0x1B00001B  NM_node_1B     — ESP proxy
    0x1B000053  NM_node_53     — Infotainment proxy  ← was missing
    0x1B000073  NM_node_73     — Kombi proxy         ← was missing


NOTE: 0x3C0 ignition is CORE — handled by BusManager, not here.

BCME BAP: extended IDs 0x17332300 (Get), 0x17332310 (S), 0x17332301 (D).

Reference: A4 MIB2.5 fullinit log (0000117). On 17332300 the cluster exchange uses **18 C2 / 18 C1**
(BAP short headers), not 1C 82/81 — those raw handoff bytes appear on **17333200** (HUD GET) and
are handled by module 5F. Do **not** replay the HUD nav fullinit bundle (17333211) on BCME; that was
wrong and floods the bus.

BCME (0x173323xx) on **0x17332310**:
  • **Handshake**: reply to MIB2.5 **18 C2** (Status BAP_Config) and **18 C1** (Status GetAll) on **0x17332300/01**.
  • After **both** Status replies have been sent once → handshake complete.
  • **Post-handshake**: **HeartbeatStatus** cycle on **17332310**, **~1 s** per step (same order/payload as capture):
      Fct 02 → 03 (long) → 04 → 0E → 0F → 11 → repeat.
  • **0x17332300** Get FCT **0x10** / **00 00 03** → short Status **48 D0 00 00 00 00** (anytime; does not gate HB).
"""
import threading
import time

from ecu_base import ECUModule
from bap import BAP_OP_HEARTBEAT_STATUS, BAP_OP_STATUS, build_bap_frames

# BCME BAP channel IDs (0x173323xx)
BCME_GET = 0x17332300  # MIB2.5 → Gateway (Get/SetGet)
BCME_D   = 0x17332301  # MIB2.5 → Gateway (long-frame continuation)
BCME_S   = 0x17332310  # Gateway → MIB2.5 (Status / HeartbeatStatus)

_BCME_LSG = 0x23

# Status 0x02 — answers MIB2.5 GET 18C2 (BAP_Config).
_BCME_RSP_18C2 = list(build_bap_frames(BAP_OP_STATUS, _BCME_LSG, 0x02, [0x03, 0x01, 0x23, 0x00, 0x04, 0x04])[0])

# Status 0x01 GetAll — answers GET 18C1 (reassembled payload 26 B).
_BCME_STATUS_GETALL_PAYLOAD = [
    0x03, 0x01, 0x23, 0x00, 0x04, 0x04, 0x08, 0x00, 0x38, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x44, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x02, 0x00, 0x00,
]
_BCME_RSP_GETALL_FRAMES = build_bap_frames(BAP_OP_STATUS, _BCME_LSG, 0x01, _BCME_STATUS_GETALL_PAYLOAD)

# Status 0x10 — short single CAN frame (not long transport): 48 D0 + 00 00 00 00.
_BCME_RSP_UNKNOWN_F10 = list(
    build_bap_frames(BAP_OP_STATUS, _BCME_LSG, 0x10, [0x00, 0x00, 0x00, 0x00])[0]
)

# After handshake: one logical HeartbeatStatus per second on 17332310 (capture ~630–640 ms tick).
_BCME_HB_PERIOD_S = 1.0
_BCME_HB_STEPS = [
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x02, [0x03, 0x01, 0x23, 0x00, 0x04, 0x04]),
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x03, [0x38, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00]),
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x04, [0x0A]),
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x0E, [0x44]),
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x0F, [0x00]),
    build_bap_frames(BAP_OP_HEARTBEAT_STATUS, _BCME_LSG, 0x11, [0x00, 0x00, 0xE8, 0x03, 0x02]),
]

_BASE = [0x00, 0x00, 0x00, 0x90, 0x89, 0xC0]  # verlern=0, km=0, 2025-01-01 12:00

# ── TSG_FT_02 (0x3E5, ICAN BO_ 997) — CRC-8 AUTOSAR H2F + GenMsgPDUConstants per BZ ──
# PDU Data-ID table recovered from hud-cmds/0000027.TXT (16 CRCs, body 20 2a 01 00 00 00 00).
_TSG_FT02_PDU_DATA_IDS = bytes(
    [0xC4, 0x6A, 0x69, 0x30, 0xCF, 0x61, 0x58, 0x51, 0x1B, 0x86, 0x99, 0xD3, 0xF6, 0x1D, 0x9A, 0x37]
)
# Idle body bytes 1–7 from same log (BZ nibble cycles 0–15); SP heating etc. preserved.
_TSG_FT02_BODY_BASE = bytes.fromhex("202a0100000000")

TSG_FT02_ARB_ID = 0x3E5
TSG_HBFS01_ARB_ID = 0x3CF  # BO_ 975 TSG_HBFS_01 — hatch / rear closure controller
# BO_ 1411 ZV_02 — GenMsgCycleTime 200ms in MLBevo ICAN DBC; door-open bits used by cluster/HUD.
ZV_02_ARB_ID = 0x583
# Idle payload from real car (0000150-doors.TXT) with ZV_FT/BT/HFS/HBFS/HD/HS_offen all clear.
_ZV_02_BODY_BASE = bytes.fromhex("0050800000084000")


def _crc8_autosar_h2f(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x2F) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


def _intel_wr(buf: bytearray, start: int, length: int, val: int) -> None:
    val &= (1 << length) - 1
    for i in range(length):
        b = start + i
        if (val >> i) & 1:
            buf[b // 8] |= 1 << (b % 8)
        else:
            buf[b // 8] &= ~(1 << (b % 8))


def _build_zv_02_frame(
    ft_open: bool,
    bt_open: bool,
    hfs_open: bool,
    hbfs_open: bool,
    hd_open: bool,
    hs_open: bool = False,
) -> list[int]:
    """ZV_02 Intel @1+: bits 24–29 = ZV_FT/BT/HFS/HBFS/HD/HS_offen (1 = open). HD = Heckdeckel (tailgate)."""
    buf = bytearray(_ZV_02_BODY_BASE)
    _intel_wr(buf, 24, 1, 1 if ft_open else 0)
    _intel_wr(buf, 25, 1, 1 if bt_open else 0)
    _intel_wr(buf, 26, 1, 1 if hfs_open else 0)
    _intel_wr(buf, 27, 1, 1 if hbfs_open else 0)
    _intel_wr(buf, 28, 1, 1 if hd_open else 0)
    _intel_wr(buf, 29, 1, 1 if hs_open else 0)
    return list(buf)


def _build_tsg_ft_02_frame(bz: int, ft: int, bfs: int, hbfs: int, hfs: int) -> list[int]:
    """
    Pack TSG_FT_02 per MLBevo ICAN DBC (Intel @1+). Door status VAL: 1=closed, 2=open.
    """
    def _cl(v: int) -> int:
        x = int(v)
        return x if x in (1, 2) else 1

    ft, bfs, hbfs, hfs = _cl(ft), _cl(bfs), _cl(hbfs), _cl(hfs)
    bz &= 0x0F
    buf = bytearray([0]) + bytearray(_TSG_FT02_BODY_BASE)
    _intel_wr(buf, 8, 4, bz)
    _intel_wr(buf, 12, 2, ft)
    _intel_wr(buf, 14, 1, 0)
    _intel_wr(buf, 15, 1, 0)
    _intel_wr(buf, 16, 1, 0)
    _intel_wr(buf, 17, 2, bfs)
    _intel_wr(buf, 19, 2, hbfs)
    _intel_wr(buf, 21, 2, hfs)
    _intel_wr(buf, 23, 1, 0)
    _intel_wr(buf, 24, 1, 1)
    _intel_wr(buf, 25, 1, 0)
    _intel_wr(buf, 26, 1, 0)
    _intel_wr(buf, 27, 2, 0)
    _intel_wr(buf, 29, 1, 0)
    _intel_wr(buf, 36, 1, 0)
    crc_in = bytes(buf[1:8]) + bytes([_TSG_FT02_PDU_DATA_IDS[bz]])
    buf[0] = _crc8_autosar_h2f(crc_in)
    return list(buf)

# GNSS_05 — ICAN BO_ 891 / 0x37B, 8 byte, Gateway. GNSS_UTC_Zeit = UTC seconds (Unix epoch);
# Uhrzeit_01 (0x6B6) is cluster display time — different message.


def _pack_gnss_05(
    utc_sec: int,
    empfaenger_ok: int = 1,
    gps_in_use: int = 1,
    glonass_in_use: int = 0,
    sat_empfangbar: int = 12,
    sat_sichtbar: int = 10,
    sat_genutzt: int = 8,
    nachrichtenpaket_id: int = 0,
) -> list:
    """Intel @1+ per MLBevo ICAN DBC (bit `start` = LSB of each field)."""
    buf = bytearray(8)
    u = max(1, min(0xFFFFFFFE, int(utc_sec) & 0xFFFFFFFF))

    def wr(start: int, length: int, val: int) -> None:
        val &= (1 << length) - 1
        for i in range(length):
            if (val >> i) & 1:
                b = start + i
                buf[b // 8] |= 1 << (b % 8)

    wr(0, 32, u)
    wr(32, 1, empfaenger_ok & 1)
    wr(33, 1, gps_in_use & 1)
    wr(34, 1, glonass_in_use & 1)
    wr(35, 5, max(1, min(30, sat_empfangbar)))
    wr(40, 5, max(1, min(30, sat_sichtbar)))
    wr(45, 5, max(1, min(30, sat_genutzt)))
    wr(50, 2, nachrichtenpaket_id & 3)
    return list(buf)


def _pack_gnss_05_from_utc_now() -> list:
    return _pack_gnss_05(int(time.time()))


class GatewayECU(ECUModule):
    ECU_ID   = "19"
    ECU_NAME = "Gateway"
    # BCME_S is our own TX id — do not RX it (avoids echo / noise). Handshake is on GET + D.
    RX_IDS   = (BCME_GET, BCME_D)
    MESSAGES = [
        # ── Standard CAN ──────────────────────────────────────────────────────
        (0x6B2,       "Diagnose_01",  1000, _BASE + [0x00, 0x00]),
        (0x37B,       "GNSS_05",      1000, _pack_gnss_05_from_utc_now()),

        # ── NVEM ──────────────────────────────────────────────────────────────
        # NVEM_02: odometer counter, b5/b6 increment slowly. Log 0000058: 100ms.
        (0x663,       "NVEM_02",       100, [0x60, 0x00, 0x20, 0x06, 0x00, 0xC6, 0x31, 0x00]),
        # NVEM_06: 16-frame CRC+counter replay. Log 0000058: 100ms.
        # Cluster validates the counter; single static frame causes rejection.
        (0x16A95414,  "NVEM_06",       100, [
            [0x37, 0x60, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x00, 0x61, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x18, 0x62, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0xE5, 0x63, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0xFF, 0x64, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x9C, 0x65, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x55, 0x66, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x62, 0x67, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0xB2, 0x68, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0xD5, 0x69, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x60, 0x6A, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x74, 0x6B, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x5C, 0x6C, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x58, 0x6D, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0x56, 0x6E, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
            [0xB0, 0x6F, 0x52, 0x48, 0x0A, 0x00, 0x53, 0x00],
        ]),
        # NVEM_07 (0x16A95415) removed — not present in any log capture

        # ── NM keepalives (29-bit, ~100ms) ────────────────────────────────────
        #(0x1B000010,  "NM_Gateway",    100, [0x10, 0x50, 0x44, 0x0B, 0x00, 0xFF, 0x01, 0x00]),
        #(0x1B000014,  "NM_node_14",    100, [0x14, 0x40, 0x44, 0x21, 0x00, 0x00, 0x00, 0x00]),
        #(0x1B00001B,  "NM_node_1B",    100, [0x1B, 0x40, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00]),
        #(0x1B000053,  "NM_node_53",    100, [0x53, 0x40, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00]),
        #(0x1B000073,  "NM_node_73",    100, [
        #    [0x73, 0x40, 0x04, 0x09, 0x00, 0x00, 0x02, 0x00],
        #    [0x73, 0x40, 0x04, 0x08, 0x00, 0x00, 0x02, 0x00],
        #    [0x73, 0x40, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00],
        #    [0x73, 0x40, 0x08, 0x08, 0x00, 0x00, 0x02, 0x00],
        #]),

        # ── TSG_FT_02 (0x3E5) — four door status + rolling BZ; CRC from 0000027.TXT
        (TSG_FT02_ARB_ID,  "TSG_FT_02",     100,  _build_tsg_ft_02_frame(0, 1, 1, 1, 1)),
        # ── TSG_HBFS_01 (0x3CF) — hatch open bit (HBFS_Tuer_geoeffnet); not in 0x3E5
        (TSG_HBFS01_ARB_ID, "TSG_HBFS_01",  100,  [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        # ── ZV_02 (0x583) — door/tailgate open flags for HUD/cluster (see DBC CM_ on ZV_*_offen)
        (ZV_02_ARB_ID,     "ZV_02",         200,  _build_zv_02_frame(False, False, False, False, False)),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._secs = 0
        self._tsg_ft02_bz = 0
        self._door_ft = 1
        self._door_bfs = 1
        self._door_hbfs = 1
        self._door_hfs = 1
        self._hbfs_hatch_open = False
        self._bcme_lock = threading.Lock()
        self._last_bcme_rx_at: dict[str, float] = {}
        self._bcme_replied_bap_config = False
        self._bcme_replied_getall = False
        self._bcme_hs_complete = False
        self._bcme_hb_idx = 0
        self._bcme_hb_next_due = float("inf")

    def set_door_driver_open(self, open_: bool) -> None:
        with self._lock:
            self._door_ft = 2 if open_ else 1

    def set_door_passenger_open(self, open_: bool) -> None:
        with self._lock:
            self._door_bfs = 2 if open_ else 1

    def set_door_rear_left_open(self, open_: bool) -> None:
        # LHD: rear left = driver-side rear = HFS (Hinten Fahrerseite), not HBFS.
        with self._lock:
            self._door_hfs = 2 if open_ else 1

    def set_door_rear_right_open(self, open_: bool) -> None:
        # LHD: rear right = passenger-side rear = HBFS (Hinten Beifahrerseite).
        with self._lock:
            self._door_hbfs = 2 if open_ else 1

    def set_hatch_open(self, open_: bool) -> None:
        """TSG_HBFS_01 (0x3CF) bit 0 — HBFS_Tuer_geoeffnet (hatch/liftgate open)."""
        with self._lock:
            self._hbfs_hatch_open = bool(open_)

    def set_enabled(self, on: bool):
        """Respect UI toggle and ignition: when False, stop sending messages."""
        was = self.enabled
        self.enabled = bool(on)
        with self._bcme_lock:
            if not self.enabled:
                self._last_bcme_rx_at.clear()
                self._bcme_reset_handshake_locked()
            elif on and not was:
                self._last_bcme_rx_at.clear()
                self._bcme_reset_handshake_locked()

    def _bcme_reset_handshake_locked(self):
        self._bcme_replied_bap_config = False
        self._bcme_replied_getall = False
        self._bcme_hs_complete = False
        self._bcme_hb_idx = 0
        self._bcme_hb_next_due = float("inf")

    def _bcme_maybe_start_hb_after_handshake_locked(self, now: float):
        """Call with _bcme_lock held. First time both Status replies sent → arm 1 Hz HB on S."""
        if not self._bcme_replied_bap_config or not self._bcme_replied_getall or self._bcme_hs_complete:
            return
        self._bcme_hs_complete = True
        self._bcme_hb_idx = 0
        self._bcme_hb_next_due = now + _BCME_HB_PERIOD_S
        self._log("[BCME] handshake complete (BAP_Config + GetAll) → HeartbeatStatus 1 s cadence")

    def _bcme_debounced(self, key: str, now: float) -> bool:
        last = self._last_bcme_rx_at.get(key, 0.0)
        if now - last < 0.050:
            return False
        self._last_bcme_rx_at[key] = now
        return True

    @staticmethod
    def _bcme_mib25_short_key(data: list[int]) -> str | None:
        """18 C1 / 18 C2 (BAP_Config / GetAll intro); allow zero padding to DLC 8."""
        if len(data) < 2 or data[0] != 0x18 or data[1] not in (0xC1, 0xC2):
            return None
        if len(data) > 2 and any(b != 0 for b in data[2:]):
            return None
        return "18c2" if data[1] == 0xC2 else "18c1"

    @staticmethod
    def _is_bcme_get_f10_000003(data: list[int]) -> bool:
        """Get LSG 23 FCT 0x10, payload 00 00 03 — short or long-transport first frame."""
        # Single frame: 18 D0 00 00 03 [00 pad…]
        if len(data) >= 5 and data[0] == 0x18 and data[1] == 0xD0 and data[2:5] == [0x00, 0x00, 0x03]:
            return len(data) == 5 or all(b == 0 for b in data[5:])
        # Long: 80 03 18 D0 00 00 03 (7 B) or same + 00 pad; or 00 00 00 03 tail (8 B capture)
        if len(data) >= 7 and data[0] == 0x80 and data[1] == 0x03 and data[2] == 0x18 and data[3] == 0xD0:
            if data[4:7] == [0x00, 0x00, 0x03]:
                return len(data) == 7 or all(b == 0 for b in data[7:])
            if len(data) >= 8 and data[4:8] == [0x00, 0x00, 0x00, 0x03]:
                return True
        return False

    def on_message(self, msg):
        if not self.enabled:
            return
        aid = msg.arbitration_id
        data = list(msg.data)
        now = time.monotonic()

        if aid not in (BCME_GET, BCME_D):
            return

        if aid == BCME_GET and self._is_bcme_get_f10_000003(data):
            with self._bcme_lock:
                if not self._bcme_debounced("get_f10", now):
                    return
            self._log("[BCME] GET FCT 0x10 (unknown) -> Status 48 D0 00 00 00 00")
            self._tx(BCME_S, list(_BCME_RSP_UNKNOWN_F10))
            return

        short_key = self._bcme_mib25_short_key(data)
        if short_key is None:
            return
        with self._bcme_lock:
            if not self._bcme_debounced(short_key, now):
                return
        if short_key == "18c2":
            ch = "D" if aid == BCME_D else "GET"
            self._log(f"[BCME] {ch} 18C2 -> Status 0x02 BAP_Config")
            self._tx(BCME_S, list(_BCME_RSP_18C2))
            with self._bcme_lock:
                self._bcme_replied_bap_config = True
                self._bcme_maybe_start_hb_after_handshake_locked(time.monotonic())
            return

        self._log(f"[BCME] {'D' if aid == BCME_D else 'GET'} 18C1 -> Status 0x01 GetAll (long)")
        self._send_frames_atomic(BCME_S, [list(f) for f in _BCME_RSP_GETALL_FRAMES])
        with self._bcme_lock:
            self._bcme_replied_getall = True
            self._bcme_maybe_start_hb_after_handshake_locked(time.monotonic())

    def _run(self):
        now = time.monotonic()
        next_tx = {s.arb_id: now + s.interval_ms / 1000.0 for s in self._states}

        while not self._stop_evt.is_set():
            now = time.monotonic()
            soonest = 0.010

            hb_step: list[list[int]] | None = None
            with self._bcme_lock:
                if self.enabled and self._bcme_hs_complete and now >= self._bcme_hb_next_due:
                    hb_step = [list(f) for f in _BCME_HB_STEPS[self._bcme_hb_idx]]
                    self._bcme_hb_idx = (self._bcme_hb_idx + 1) % len(_BCME_HB_STEPS)
                    self._bcme_hb_next_due = now + _BCME_HB_PERIOD_S
            if hb_step is not None and self.enabled:
                self._send_frames_atomic(BCME_S, hb_step)

            for s in self._states:
                due = next_tx[s.arb_id]
                if now >= due:
                    if s.arb_id == 0x6B2:
                        b6 = 0x80 if (self._secs & 1) else 0x00
                        b7 = (self._secs >> 1) & 0x1F
                        with self._lock:
                            s.data = _BASE + [b6, b7]
                        self._secs = (self._secs + 1) % 60
                    elif s.arb_id == 0x37B:
                        with self._lock:
                            s.data = _pack_gnss_05_from_utc_now()
                    elif s.arb_id == TSG_FT02_ARB_ID:
                        bz = self._tsg_ft02_bz
                        self._tsg_ft02_bz = (self._tsg_ft02_bz + 1) & 0x0F
                        with self._lock:
                            fr = _build_tsg_ft_02_frame(
                                bz, self._door_ft, self._door_bfs, self._door_hbfs, self._door_hfs
                            )
                            s.data = fr
                    elif s.arb_id == TSG_HBFS01_ARB_ID:
                        with self._lock:
                            h = 1 if self._hbfs_hatch_open else 0
                            s.data = [h, 0, 0, 0, 0, 0, 0, 0]
                    elif s.arb_id == ZV_02_ARB_ID:
                        with self._lock:
                            s.data = _build_zv_02_frame(
                                self._door_ft == 2,
                                self._door_bfs == 2,
                                self._door_hfs == 2,
                                self._door_hbfs == 2,
                                self._hbfs_hatch_open,
                            )

                    self._enqueue(s.arb_id, s.next_payload())
                    with self._lock:
                        s.tx_count += 1
                        s.last_tx   = now
                    next_tx[s.arb_id] = due + s.interval_ms / 1000.0

                wait = next_tx[s.arb_id] - now
                if wait < soonest:
                    soonest = wait

            with self._bcme_lock:
                if self.enabled and self._bcme_hs_complete and self._bcme_hb_next_due < float("inf"):
                    w = self._bcme_hb_next_due - now
                    if w < soonest:
                        soonest = w

            self._stop_evt.wait(timeout=max(0.001, soonest))
