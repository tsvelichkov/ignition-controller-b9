"""
modules/44_eps.py — EPS / Steering Rack (Module 44) emulation

Sends:
  0x086  LWI_01    — Steering wheel angle + angular velocity, ~50ms
                     DBC: LWI_Lenkradwinkel (13-bit, 0.1 deg/bit), VZ sign bit,
                          LWI_Lenkradw_Geschw (9-bit, 5 deg/s per bit)
  0x0B5  EPS_01    — EPS torque/status (not in DBC, identified from log),
                     b2=0xB4 fixed (operational status), b3=steering data,
                     b4-b5=0xA1 0x02 fixed, ~10ms

Source: 0000046.TXT car log (straight-line driving, ~1.6° steering offset).
Both messages use replay sequences extracted from the log to preserve authentic
CRC and rolling counter progression.
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule


class EPSECU(ECUModule):
    ECU_ID   = "44"
    ECU_NAME = "EPS/Steering"

    # Replay sequences from 0000046.TXT — straight-line driving (~1.6° offset)
    _LWI_SEQ = [
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("470e0f0000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("470e0f0000550000"),
        bytes.fromhex("30080f0000550000"),
        bytes.fromhex("a9020f0000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("61000f0000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("30080f0000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("61000f0000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("30080f0000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("8f0c0f0000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("a9020f0000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("470e0f0000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("61000f0000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("470e0f0000550000"),
        bytes.fromhex("30080f0000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("61000f0000550000"),
        bytes.fromhex("f80a0f0000550000"),
        bytes.fromhex("fb04100000550000"),
        bytes.fromhex("620e100000550000"),
        bytes.fromhex("1508100000550000"),
        bytes.fromhex("8c02100000550000"),
        bytes.fromhex("aa0c100000550000"),
        bytes.fromhex("3306100000550000"),
        bytes.fromhex("4400100000550000"),
        bytes.fromhex("dd0a100000550000"),
        bytes.fromhex("de040f0000550000"),
        bytes.fromhex("620e100000550000"),
    ]

    _B5_SEQ = [
        bytes.fromhex("5a1bb466a1020000"),
        bytes.fromhex("d31cb487a1020000"),
        bytes.fromhex("b71db487a1020000"),
        bytes.fromhex("1b1eb487a1020000"),
        bytes.fromhex("7f1fb487a1020000"),
        bytes.fromhex("3d10b487a1020000"),
        bytes.fromhex("5911b487a1020000"),
        bytes.fromhex("4412b463a1020000"),
        bytes.fromhex("2013b463a1020000"),
        bytes.fromhex("3314b463a1020000"),
        bytes.fromhex("5715b463a1020000"),
        bytes.fromhex("fb16b463a1020000"),
        bytes.fromhex("9f17b463a1020000"),
        bytes.fromhex("0218b449a1020000"),
        bytes.fromhex("6619b449a1020000"),
        bytes.fromhex("ca1ab449a1020000"),
        bytes.fromhex("ae1bb449a1020000"),
        bytes.fromhex("bd1cb449a1020000"),
        bytes.fromhex("d91db449a1020000"),
        bytes.fromhex("8d1eb45ca1020000"),
        bytes.fromhex("e91fb45ca1020000"),
        bytes.fromhex("ab10b45ca1020000"),
        bytes.fromhex("cf11b45ca1020000"),
        bytes.fromhex("6312b45ca1020000"),
        bytes.fromhex("0713b45ca1020000"),
        bytes.fromhex("c714b44ca1020000"),
        bytes.fromhex("a315b44ca1020000"),
        bytes.fromhex("0f16b44ca1020000"),
        bytes.fromhex("6b17b44ca1020000"),
        bytes.fromhex("2918b44ca1020000"),
        bytes.fromhex("4d19b44ca1020000"),
        bytes.fromhex("a21ab49aa1020000"),
        bytes.fromhex("c61bb49aa1020000"),
        bytes.fromhex("d51cb49aa1020000"),
        bytes.fromhex("b11db49aa1020000"),
        bytes.fromhex("1d1eb49aa1020000"),
        bytes.fromhex("791fb49aa1020000"),
        bytes.fromhex("1210b494a1020000"),
        bytes.fromhex("7611b494a1020000"),
        bytes.fromhex("da12b494a1020000"),
        bytes.fromhex("be13b494a1020000"),
        bytes.fromhex("ad14b494a1020000"),
        bytes.fromhex("c915b494a1020000"),
        bytes.fromhex("d416b470a1020000"),
        bytes.fromhex("b017b470a1020000"),
        bytes.fromhex("f218b470a1020000"),
        bytes.fromhex("9619b470a1020000"),
        bytes.fromhex("3a1ab470a1020000"),
        bytes.fromhex("5e1bb470a1020000"),
        bytes.fromhex("6c1cb452a1020000"),
        bytes.fromhex("081db452a1020000"),
        bytes.fromhex("a41eb452a1020000"),
        bytes.fromhex("c01fb452a1020000"),
        bytes.fromhex("8210b452a1020000"),
        bytes.fromhex("e611b452a1020000"),
        bytes.fromhex("6f12b466a1020000"),
        bytes.fromhex("0b13b466a1020000"),
        bytes.fromhex("1814b466a1020000"),
        bytes.fromhex("7c15b466a1020000"),
        bytes.fromhex("d016b466a1020000"),
        bytes.fromhex("b417b466a1020000"),
        bytes.fromhex("d518b44fa1020000"),
        bytes.fromhex("b119b44fa1020000"),
        bytes.fromhex("1d1ab44fa1020000"),
        bytes.fromhex("791bb44fa1020000"),
        bytes.fromhex("6a1cb44fa1020000"),
        bytes.fromhex("0e1db44fa1020000"),
        bytes.fromhex("921eb4a7a1020000"),
        bytes.fromhex("f61fb4a7a1020000"),
        bytes.fromhex("b410b4a7a1020000"),
        bytes.fromhex("d011b4a7a1020000"),
        bytes.fromhex("7c12b4a7a1020000"),
        bytes.fromhex("1813b4a7a1020000"),
        bytes.fromhex("7414b4a3a1020000"),
        bytes.fromhex("1015b4a3a1020000"),
        bytes.fromhex("bc16b4a3a1020000"),
        bytes.fromhex("d817b4a3a1020000"),
        bytes.fromhex("9a18b4a3a1020000"),
        bytes.fromhex("fe19b4a3a1020000"),
        bytes.fromhex("b51ab44da1020000"),
        bytes.fromhex("d11bb44da1020000"),
        bytes.fromhex("c21cb44da1020000"),
        bytes.fromhex("a61db44da1020000"),
        bytes.fromhex("0a1eb44da1020000"),
        bytes.fromhex("6e1fb44da1020000"),
        bytes.fromhex("2410b461a1020000"),
        bytes.fromhex("4011b461a1020000"),
        bytes.fromhex("ec12b461a1020000"),
        bytes.fromhex("8813b461a1020000"),
        bytes.fromhex("9b14b461a1020000"),
        bytes.fromhex("ff15b461a1020000"),
        bytes.fromhex("8016b471a1020000"),
        bytes.fromhex("e417b471a1020000"),
        bytes.fromhex("a618b471a1020000"),
        bytes.fromhex("c219b471a1020000"),
        bytes.fromhex("6e1ab471a1020000"),
        bytes.fromhex("0a1bb471a1020000"),
        bytes.fromhex("431cb441a1020000"),
        bytes.fromhex("271db441a1020000"),
        bytes.fromhex("8b1eb441a1020000"),
        bytes.fromhex("ef1fb441a1020000"),
        bytes.fromhex("ad10b441a1020000"),
        bytes.fromhex("c911b441a1020000"),
        bytes.fromhex("4a12b452a1020000"),
        bytes.fromhex("2e13b452a1020000"),
        bytes.fromhex("3d14b452a1020000"),
        bytes.fromhex("5915b452a1020000"),
        bytes.fromhex("f516b452a1020000"),
        bytes.fromhex("9117b452a1020000"),
        bytes.fromhex("ac18b456a1020000"),
        bytes.fromhex("c819b456a1020000"),
        bytes.fromhex("641ab456a1020000"),
        bytes.fromhex("001bb456a1020000"),
        bytes.fromhex("131cb456a1020000"),
        bytes.fromhex("771db456a1020000"),
        bytes.fromhex("891eb44aa1020000"),
        bytes.fromhex("ed1fb44aa1020000"),
        bytes.fromhex("af10b44aa1020000"),
        bytes.fromhex("cb11b44aa1020000"),
        bytes.fromhex("6712b44aa1020000"),
        bytes.fromhex("0313b44aa1020000"),
        bytes.fromhex("e014b473a1020000"),
        bytes.fromhex("8415b473a1020000"),
        bytes.fromhex("2816b473a1020000"),
        bytes.fromhex("4c17b473a1020000"),
        bytes.fromhex("0e18b473a1020000"),
        bytes.fromhex("6a19b473a1020000"),
        bytes.fromhex("131ab47ea1020000"),
        bytes.fromhex("771bb47ea1020000"),
        bytes.fromhex("641cb47ea1020000"),
        bytes.fromhex("001db47ea1020000"),
        bytes.fromhex("ac1eb47ea1020000"),
        bytes.fromhex("c81fb47ea1020000"),
        bytes.fromhex("2c10b44da1020000"),
        bytes.fromhex("4811b44da1020000"),
        bytes.fromhex("e412b44da1020000"),
        bytes.fromhex("8013b44da1020000"),
        bytes.fromhex("9314b44da1020000"),
        bytes.fromhex("f715b44da1020000"),
        bytes.fromhex("de16b457a1020000"),
        bytes.fromhex("ba17b457a1020000"),
        bytes.fromhex("f818b457a1020000"),
        bytes.fromhex("9c19b457a1020000"),
        bytes.fromhex("301ab457a1020000"),
        bytes.fromhex("541bb457a1020000"),
        bytes.fromhex("921cb45aa1020000"),
        bytes.fromhex("f61db45aa1020000"),
        bytes.fromhex("5a1eb45aa1020000"),
        bytes.fromhex("3e1fb45aa1020000"),
        bytes.fromhex("7c10b45aa1020000"),
    ]

    MESSAGES = [
        # (arb_id, name, interval_ms, initial_data)
        (0x086, "LWI_01",  100, [0x44, 0x00, 0x10, 0x00, 0x00, 0x55, 0x00, 0x00]),  # log: 100ms
        (0x0B5, "EPS_01",  10,  [0x5A, 0x1B, 0xB4, 0x66, 0xA1, 0x02, 0x00, 0x00]),  # log: 10ms OK
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._lwi_idx = 0
        self._b5_idx  = 0

    def set_enabled(self, on: bool):
        self.enabled = on

    def _next_frame(self, arb_id):
        if arb_id == 0x086:
            d = self._LWI_SEQ[self._lwi_idx % len(self._LWI_SEQ)]
            self._lwi_idx += 1
            return d
        if arb_id == 0x0B5:
            d = self._B5_SEQ[self._b5_idx % len(self._B5_SEQ)]
            self._b5_idx += 1
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
