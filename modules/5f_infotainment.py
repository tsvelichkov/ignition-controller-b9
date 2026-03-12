"""
modules/5f_infotainment.py — Infotainment / HUD BAP emulation.

This module now uses a small explicit state machine derived from the
reference non-nav and nav logs instead of the old 23-step replay.
"""

import queue
import time

from ecu_base import ECUModule
from bap import BAP_LSG_HUD_NAV, BAP_OP_HEARTBEAT_STATUS, BAP_OP_STATUS, build_bap_header, decode_bap_header
from hud_bap_hints import HUD_NAV_FUNCTION_NAMES

HUD_S = 0x17333210
HUD_D = 0x17333211
HUD_ASG = 0x17333202
HUD_RX = 0x17330410
HUD_BOOT = 0x17330400
HUD_GET = 0x17333200

HUD_LSG = BAP_LSG_HUD_NAV
MFL_HB = 0x331

REQ_OP_GET = 1
REQ_OP_SETGET = 2

HB_INTERVAL_S = 10
DEFAULT_COMPASS_INFO = [0x02, 0x35, 0x01]
DEFAULT_CURRENT_POSITION_INFO_OFFROAD = [0x07, 0x4F, 0x66, 0x66, 0x72, 0x6F, 0x61, 0x64]
DEFAULT_UNKNOWN35_PAYLOAD = [0x00, 0x00, 0x30, 0x00, 0x00, 0x00]
DEFAULT_FEATURE_ENABLE_MASK = [
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF,
    0xFF, 0xF0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
]
STANDBY_FUNCTION_LIST_INIT = [0x78, 0x01, 0xDC, 0x00, 0x06, 0x00, 0x00, 0x00]
STANDBY_FUNCTION_LIST_READY = [0x78, 0x01, 0xFF, 0x86, 0x67, 0x5F, 0x67, 0x48]
DISTANCE_INACTIVE = [0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]
FUNC_SYNC_IDLE = [0x00] * 8
FUNC_SYNC_ACTIVE = [0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x40, 0x00]
FUNC_SYNC_TRIGGER = [0x90, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x61, 0x00]
FUNC_SYNC_TRIGGER_C0 = [0xC0, 0x00, 0x01, 0x3D, 0x01]
FUNC_SYNC_TRIGGER_D0 = [0xD0, 0x01, 0x00, 0x40, 0x00]
MIB_FUNCTION_LIST = [0x38, 0x07, 0xE8, 0x00, 0x06, 0x00]
A4_MIB3_READY_FUNCTION_LIST = [0x78, 0x01, 0xFF, 0x86, 0x67, 0x5F, 0x67, 0x48]
FULLINIT_HANDOFF_BUNDLE = [
    [0x80, 0x2E, 0x4C, 0x81, 0x03, 0x00, 0x32, 0x00],
    [0xC0, 0x04, 0x08, 0x08, 0x00, 0x78, 0x01, 0xDC],
    [0xC1, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0A, 0x03],
    [0xC2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00],
    [0xC3, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0xC4, 0x00, 0xFF, 0x00, 0x08, 0x00, 0x00, 0x00],
    [0xC5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
]
FULLINIT_HANDOFF_OFFSETS = [0.000, 0.010, 0.032, 0.048, 0.066, 0.080, 0.095]

ARROW_PRESETS = {
    "offroad": {"name": "Offroad", "main": 0x09, "dir": 0x00, "dist_m": 0, "bearing": 0x5F, "offroad": True},
    "right": {"name": "Turn Right", "main": 0x0D, "dir": 0x40, "dist_m": 500, "bearing": 0x40, "offroad": False},
    "left": {"name": "Turn Left", "main": 0x0D, "dir": 0xC0, "dist_m": 500, "bearing": 0xC0, "offroad": False},
    "straight": {"name": "Straight", "main": 0x0B, "dir": 0x00, "dist_m": 500, "bearing": 0x00, "offroad": False},
}


def bap(op, lsg, fn):
    return tuple(build_bap_header(op, lsg, fn))


def bap_hb(fn, lsg=HUD_LSG):
    return list(bap(BAP_OP_HEARTBEAT_STATUS, lsg, fn))


def bap_st(fn, lsg=HUD_LSG):
    return list(bap(BAP_OP_STATUS, lsg, fn))


class InfotainmentECU(ECUModule):
    ECU_ID = "5F"
    ECU_NAME = "Infotainment"
    RX_IDS = (HUD_GET, HUD_ASG, HUD_RX, HUD_BOOT)
    MESSAGES = [
        (HUD_S, "HUD Primary", 0, [0x3C, 0x82, 0x00]),
        (HUD_D, "HUD Secondary", 0, [0x3C, 0x93, 0x00]),
        (MFL_HB, "MFL Heartbeat", 0, [0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]),
    ]

    PHASE_STANDBY_INIT = "STANDBY_INIT"
    PHASE_STANDBY_READY = "STANDBY_READY"
    PHASE_PRE_ACTIVE = "PRE_ACTIVE"
    PHASE_WAIT_ROUTE_START = "WAIT_ROUTE_START"
    PHASE_WAIT_ACTIVE_CONFIG = "WAIT_ACTIVE_CONFIG"
    PHASE_NAV_ACTIVE = "NAV_ACTIVE"

    @staticmethod
    def _encode_text_payload(text):
        clean = (text or "").strip() or "Offroad"
        data = clean.encode("ascii", "replace")[:31]
        return [len(data)] + list(data)

    def __init__(self, log_cb=None, config=None):
        super().__init__(log_cb)
        self._cfg = config or {}
        self._verbose = bool(self._cfg.get("verbose_bap"))
        self._hud_mode = self._cfg.get("hud_mode", "full")
        self._demo_route = self._hud_mode == "minimal"
        self._rx_pending = queue.Queue()
        self._tick = 0
        self._mfl_ctr = 0

        self._phase = self.PHASE_STANDBY_INIT
        self._hud_inited = False
        self._hud_bap_ok = False
        self._status_t = 0.0
        self._standby_step = 0
        self._ready_since = 0.0

        self._asg_last = {}
        self._asg_81_done = False
        self._asg_82_done = False
        self._boot_1102_seen = False
        self._boot_1101_seen = False
        self._a4_nav_prime_sent = False
        self._a4_nav_prime_cleared = False
        self._hud_get_82_seen = 0
        self._hud_get_81_seen = False
        self._active_visual_keepalive_next_at = 0.0

        self._init_cfg_seen = False
        self._init_cfg_lead = 0x12
        self._fn36_seen = False
        self._poll14_seen = False
        self._last_poll14_at = 0.0
        self._route_burst_sent = False
        self._active_cfg_seen = False
        self._nav_conf = False
        self._activation_script = []
        self._last_fullinit_bundle_at = 0.0
        self._last_raw_handoff_at = {}
        self._hud_status_0f_ready = False
        self._hud_status_10_ready = False
        self._hud_status_14_ready = False
        self._hud_status_0f_seen = False
        self._hud_status_10_seen = False
        self._hud_status_14_seen = False
        self._a4_preready_prime_done = False
        self._a4_seed_bundle_sent = False
        self._a4_postseed_followup_sent = False
        self._a4_nav_kick_sent = False
        self._a4_display_cfg_prime_sent = False
        self._pre_active_bundle_sent = False
        self._pre_active_running_sent = False
        self._pre_active_running_due_at = 0.0
        self._pre_active_fn36_next_at = 0.0
        self._ready_ack_sent = False
        self._a4_preready_poll10_promoted = False
        self._a4_preready_poll11_promoted = False
        self._reset_nav_state()

    def set_enabled(self, on: bool):
        was_enabled = self.enabled
        self.enabled = on
        if was_enabled and not on:
            self._log("Ignition OFF — resetting HUD state")
            self._reset_session()

    def get_nav_settings(self):
        arrow = self._current_arrow()
        return {
            "enabled": bool(self._cfg.get("hud_nav_enabled", True)),
            "distance_enabled": bool(self._cfg.get("hud_distance_enabled", True)),
            "distance_m": max(0, int(self._cfg.get("hud_distance_m", arrow.get("dist_m", 0)))),
            "distance_graph": int(self._cfg.get("hud_distance_graph", 0x64)) & 0xFF,
            "street_name": self._cfg.get("hud_street_name", "Offroad"),
            "arrow_main": arrow.get("main", 0x00),
            "arrow_dir": arrow.get("dir", 0x00),
            "arrow_bearing": arrow.get("bearing", 0x00),
        }

    def configure_nav(self, *, enabled=None, distance_enabled=None, distance_m=None, street_name=None,
                      arrow_main=None, arrow_dir=None, arrow_bearing=None,
                      distance_graph=None):
        if enabled is not None:
            self._cfg["hud_nav_enabled"] = bool(enabled)
        if distance_enabled is not None:
            self._cfg["hud_distance_enabled"] = bool(distance_enabled)
        if distance_m is not None:
            self._cfg["hud_distance_m"] = max(0, int(distance_m))
        if distance_graph is not None:
            self._cfg["hud_distance_graph"] = int(distance_graph) & 0xFF
        if street_name is not None:
            self._cfg["hud_street_name"] = (street_name or "").strip() or "Offroad"
        if arrow_main is not None:
            self._cfg["hud_arrow_main"] = int(arrow_main) & 0xFF
        if arrow_dir is not None:
            self._cfg["hud_arrow_dir"] = int(arrow_dir) & 0xFF
        if arrow_bearing is not None:
            self._cfg["hud_arrow_bearing"] = int(arrow_bearing) & 0xFF
        self._sync_nav_state()
        if self._phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE):
            self._send_hud_route_payloads()

    def _reset_session(self):
        self._phase = self.PHASE_STANDBY_INIT
        self._hud_inited = False
        self._hud_bap_ok = False
        self._status_t = 0.0
        self._standby_step = 0
        self._ready_since = 0.0
        self._demo_route = False
        self._asg_last.clear()
        self._asg_81_done = False
        self._asg_82_done = False
        self._boot_1102_seen = False
        self._boot_1101_seen = False
        self._a4_nav_prime_sent = False
        self._a4_nav_prime_cleared = False
        self._hud_get_82_seen = 0
        self._hud_get_81_seen = False
        self._active_visual_keepalive_next_at = 0.0
        self._init_cfg_seen = False
        self._init_cfg_lead = 0x12
        self._fn36_seen = False
        self._poll14_seen = False
        self._last_poll14_at = 0.0
        self._route_burst_sent = False
        self._active_cfg_seen = False
        self._nav_conf = False
        self._activation_script = []
        self._last_fullinit_bundle_at = 0.0
        self._last_raw_handoff_at = {}
        self._hud_status_0f_ready = False
        self._hud_status_10_ready = False
        self._hud_status_14_ready = False
        self._hud_status_0f_seen = False
        self._hud_status_10_seen = False
        self._hud_status_14_seen = False
        self._a4_preready_prime_done = False
        self._a4_seed_bundle_sent = False
        self._a4_postseed_followup_sent = False
        self._a4_nav_kick_sent = False
        self._a4_display_cfg_prime_sent = False
        self._pre_active_bundle_sent = False
        self._pre_active_running_sent = False
        self._pre_active_running_due_at = 0.0
        self._pre_active_fn36_next_at = 0.0
        self._ready_ack_sent = False
        self._a4_preready_poll10_promoted = False
        self._a4_preready_poll11_promoted = False
        self._reset_nav_state()
        while True:
            try:
                self._rx_pending.get_nowait()
            except queue.Empty:
                break

    def _set_phase(self, phase):
        if phase != self._phase:
            self._log(f"[PHASE] {self._phase} -> {phase}")
            self._phase = phase
            if phase == self.PHASE_STANDBY_READY:
                self._ready_since = time.monotonic()
                self._standby_step = 0
                self._a4_preready_prime_done = False
                self._a4_seed_bundle_sent = False
                self._a4_postseed_followup_sent = False
                self._a4_nav_kick_sent = False
                self._a4_display_cfg_prime_sent = False
                self._pre_active_bundle_sent = False
                self._pre_active_running_sent = False
                self._pre_active_running_due_at = 0.0
                self._ready_ack_sent = False
                self._a4_preready_poll10_promoted = False
                self._a4_preready_poll11_promoted = False
                self._pre_active_fn36_next_at = 0.0
                self._active_visual_keepalive_next_at = 0.0
            elif phase == self.PHASE_PRE_ACTIVE:
                self._pre_active_fn36_next_at = time.monotonic()
                self._active_visual_keepalive_next_at = 0.0
            elif phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE):
                self._active_visual_keepalive_next_at = time.monotonic() + 1.000
        self._sync_nav_state()

    def _vlog(self, msg):
        if self._verbose:
            self._log(msg)

    def _current_arrow(self):
        preset = dict(ARROW_PRESETS.get(self._cfg.get("hud_arrow", "straight"), ARROW_PRESETS["straight"]))
        if not preset.get("offroad"):
            preset["dist_m"] = max(0, int(self._cfg.get("hud_distance_m", preset["dist_m"])))
        main_override = self._cfg.get("hud_arrow_main")
        dir_override = self._cfg.get("hud_arrow_dir")
        bearing_override = self._cfg.get("hud_arrow_bearing")
        custom_arrow = any(value is not None for value in (main_override, dir_override, bearing_override))
        if main_override is not None:
            preset["main"] = int(main_override) & 0xFF
        if dir_override is not None:
            preset["dir"] = int(dir_override) & 0xFF
        if bearing_override is not None:
            preset["bearing"] = int(bearing_override) & 0xFF
        if custom_arrow:
            preset["offroad"] = preset.get("main", 0x00) == 0x09
            preset["name"] = f"0x{preset['main']:02X}/0x{preset['dir']:02X}"
        return preset

    def _reset_nav_state(self):
        self._nav_state = {
            "fsg_state": 0x00,
            "route_guidance_active": False,
            "session_state": [0x00, 0x01],
            "compass_info": [0x00, 0x00, 0x00],
            "current_position_info": [0x00],
            "next_maneuver_visible": False,
            "next_maneuver_distance_m": None,
            "destination_distance_m": None,
            "info_states": 0x00,
            "function_sync": list(FUNC_SYNC_IDLE),
        }
        self._sync_nav_state()

    def _sync_nav_state(self):
        if not hasattr(self, "_nav_state"):
            return
        state = self._nav_state
        if self._phase == self.PHASE_PRE_ACTIVE:
            state["fsg_state"] = 0x00 if self._pre_active_running_sent else 0x03
            state["route_guidance_active"] = False
            state["session_state"] = [0x00, 0x00]
            state["compass_info"] = list(DEFAULT_COMPASS_INFO)
            state["current_position_info"] = self._encode_text_payload(self._cfg.get("hud_street_name", "Offroad"))
            state["next_maneuver_visible"] = False
            state["next_maneuver_distance_m"] = None
            state["destination_distance_m"] = None
            state["info_states"] = 0xFF
            state["function_sync"] = [0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]
            return
        if self._phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE):
            arrow = self._current_arrow()
            nav_enabled = bool(self._cfg.get("hud_nav_enabled", True))
            distance_enabled = bool(self._cfg.get("hud_distance_enabled", True))
            state["fsg_state"] = 0x00
            state["route_guidance_active"] = nav_enabled
            state["session_state"] = [0x00, 0x00]
            state["compass_info"] = list(DEFAULT_COMPASS_INFO)
            state["current_position_info"] = self._encode_text_payload(self._cfg.get("hud_street_name", "Offroad"))
            state["next_maneuver_visible"] = nav_enabled and not arrow.get("offroad", False)
            state["next_maneuver_distance_m"] = max(int(arrow.get("dist_m", 0)), 0) if state["next_maneuver_visible"] and distance_enabled else None
            state["destination_distance_m"] = max(int(arrow.get("dist_m", 0)), 0) if state["next_maneuver_visible"] and distance_enabled else None
            state["info_states"] = 0xFF
            state["function_sync"] = list(FUNC_SYNC_ACTIVE)
            return
        state["fsg_state"] = 0x00
        state["route_guidance_active"] = False
        state["session_state"] = [0x00, 0x01]
        state["compass_info"] = [0x00, 0x00, 0x00]
        state["current_position_info"] = self._encode_text_payload(self._cfg.get("hud_street_name", "Offroad"))
        state["next_maneuver_visible"] = False
        state["next_maneuver_distance_m"] = None
        state["destination_distance_m"] = None
        state["info_states"] = 0x00
        state["function_sync"] = list(FUNC_SYNC_IDLE)

    def _destination_distance_payload(self):
        dist_m = self._nav_state["destination_distance_m"]
        if dist_m is None:
            return list(DISTANCE_INACTIVE)
        dist_dm = max(int(dist_m), 0) * 10
        return [
            dist_dm & 0xFF,
            (dist_dm >> 8) & 0xFF,
            (dist_dm >> 16) & 0xFF,
            (dist_dm >> 24) & 0xFF,
            0x00, 0x01,
        ]

    def _run(self):
        self._log(f"[INIT] 5F HUD state machine started mode={self._hud_mode}")
        if self.enabled and not self._hud_inited:
            self._hud_init()
        last = time.monotonic()
        while not self._stop_evt.is_set():
            now = time.monotonic()
            self._drain_rx_pending()
            self._service_activation_script(now)
            if now - last >= 0.100:
                last = now
                try:
                    self._tick_100ms()
                except Exception as e:
                    self._log(f"[ERROR] _tick_100ms exception: {e!r}")
                    import traceback
                    self._log(traceback.format_exc())
            self._stop_evt.wait(timeout=0.005)

    def _tick_100ms(self):
        t = self._tick
        self._tick += 1
        now = time.monotonic()

        if t % 4 == 1:
            self._mfl_ctr = (self._mfl_ctr + 1) & 0x0F
            d = [0x20 | self._mfl_ctr, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]
            self._tx(MFL_HB, d)
            self._mark_sent(MFL_HB, d)

        if not self.enabled:
            return

        if not self._hud_inited:
            self._hud_init()

        self._advance_startup_transition(now)
        if self._phase == self.PHASE_PRE_ACTIVE and not self._pre_active_running_sent and now >= self._pre_active_running_due_at:
            self._pre_active_running_sent = True
        if self._phase == self.PHASE_PRE_ACTIVE and self._pre_active_running_sent and self._hud_status_0f_ready:
            self._log("[HUD] PRE_ACTIVE running + HUD 0x0F=00 -> NAV_ACTIVE")
            self._active_cfg_seen = True
            self._nav_conf = True
            self._set_phase(self.PHASE_NAV_ACTIVE)
        if self._phase == self.PHASE_PRE_ACTIVE and now >= self._pre_active_fn36_next_at:
            self._tx(HUD_S, bap_hb(0x36) + [0x00, 0x00])
            self._pre_active_fn36_next_at = now + 1.000
        if self._phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE) and now >= self._active_visual_keepalive_next_at:
            self._send_active_visual_keepalive()
            self._active_visual_keepalive_next_at = now + 1.000
        self._sync_nav_state()

        if self._phase == self.PHASE_STANDBY_READY and self._demo_route and self._hud_ready_for_route():
            self._set_phase(self.PHASE_WAIT_ROUTE_START)

        if self._phase == self.PHASE_WAIT_ROUTE_START and not self._route_burst_sent:
            self._start_route_demo()

        if now - self._status_t >= 5.0:
            self._status_t = now
            self._log(
                f"[STATUS] phase={self._phase}"
                f" hud_bap={'OK' if self._hud_bap_ok else 'wait'}"
                f" boot={'11-01' if self._boot_1101_seen else '11-02' if self._boot_1102_seen else 'wait'}"
                f" handoff={self._handoff_status_text()}"
            )

    def _advance_startup_transition(self, now):
        return


    def _drain_rx_pending(self):
        while True:
            try:
                arb_id, data = self._rx_pending.get_nowait()
            except queue.Empty:
                break
            self._tx(arb_id, data)

    def on_message(self, msg):
        can_id = msg.arbitration_id
        data = list(msg.data)
        if not data:
            return

        if can_id == HUD_RX:
            self._handle_hud_rx(data)
            return
        if can_id == HUD_BOOT:
            self._handle_hud_boot(data)
            return
        if can_id == HUD_GET:
            self._handle_request(can_id, data)
            return
        if can_id == HUD_ASG:
            self._handle_asg(data)
            return

    def _handle_hud_rx(self, data):
        raw = bytes(data).hex().upper()
        if not self._hud_bap_ok:
            self._hud_bap_ok = True
            self._log(f"[BAP] First frame from HUD_RX {raw}")
        decoded = self._decode_request(data)
        if not decoded:
            self._vlog(f"[HUD_RX] raw={raw}")
            return
        opcode, lsg, fn, payload = decoded
        self._vlog(f"[HUD_RX] op={opcode} lsg=0x{lsg:02X} fn=0x{fn:02X} payload={bytes(payload).hex().upper()}")
        known_hud_rx = {0x01, 0x02, 0x03, 0x04, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x17, 0x21, 0x25, 0x26, 0x32, 0x35, 0x39}
        if fn not in known_hud_rx:
            self._vlog(f"[HUD_RX] ignore unsupported fn=0x{fn:02X} lsg=0x{lsg:02X}")
            return
        if opcode == BAP_OP_STATUS:
            self._handle_hud_status(fn, payload)
        if fn == 0x10 and payload[:2] == [0x1E, 0x01]:
            self._a4_preready_poll10_promoted = True
        if fn == 0x11 and payload[:3] == [0x34, 0x19, 0x01]:
            self._a4_preready_poll11_promoted = True
        if (self._phase == self.PHASE_STANDBY_READY and not self._init_cfg_seen and
                fn == 0x11 and payload[:3] == [0xFC, 0x1F, 0x01] and not self._a4_nav_kick_sent):
            self._a4_nav_kick_sent = True
            self._log("[A4] cold fn=0x11 seen -> arm nav-ready promotion window")
            self._schedule_activation_frame(time.monotonic() + 0.050, HUD_S, [0x3C, 0xA7, 0x04])
        self._respond_to_hud_poll(opcode, fn, payload)
        if fn == 0x14:
            self._poll14_seen = True
            self._last_poll14_at = time.monotonic()
            if self._phase == self.PHASE_STANDBY_READY and not self._init_cfg_seen:
                if not self._a4_nav_kick_sent:
                    self._a4_nav_kick_sent = True
                    self._log("[A4] first 31 14 in standby-ready -> arm nav-ready promotion window")
                if not self._a4_display_cfg_prime_sent:
                    self._a4_display_cfg_prime_sent = True
                    self._log("[A4] first 31 14 in standby-ready -> send HUD_D fn=0x18 nav-ready")
                    self._send_display_config(0x12)
                if not self._a4_preready_prime_done:
                    base = time.monotonic()
                    self._a4_preready_prime_done = True
                    self._log("[A4] queue nav prime window from 31 14")
                    self._schedule_activation_frame(base + 0.652, HUD_S, [0x3C, 0xAB, 0x02, 0x00, 0x00, 0x00])
                    self._schedule_activation_frame(base + 0.682, HUD_S, [0x4C, 0xA6, 0x05])
                    self._schedule_activation_frame(base + 0.791, HUD_S, [0x4C, 0x8F, 0x00])
                if not self._pre_active_bundle_sent:
                    self._log("[A4] queue pre-active nav bundle from first 31 14 in standby-ready")
                    self._queue_pre_active_nav_bundle()
            self._maybe_arm_route_demo()
        if (self._phase == self.PHASE_STANDBY_READY and fn == 0x03 and self._a4_seed_bundle_sent and
                not self._a4_postseed_followup_sent and
                not self._a4_preready_poll10_promoted and
                not self._a4_preready_poll11_promoted):
            self._a4_postseed_followup_sent = True
            self._queue_a4_postseed_followup_script()

    def _handle_hud_status(self, fn, payload):
        if fn == 0x01:
            return
        if fn == 0x0F and payload:
            was_ready = self._hud_status_0f_ready
            self._hud_status_0f_seen = True
            op_state = payload[0]
            self._hud_status_0f_ready = op_state == 0x00 and self._handoff_complete()
            if self._hud_status_0f_ready and not was_ready:
                self._log(f"[HUD] 41 0F {op_state:02X} -> nav-ready")
                self._send_ready_ack()
                if self._phase == self.PHASE_PRE_ACTIVE:
                    self._log("[HUD] display-active acknowledged -> NAV_ACTIVE")
                    self._active_cfg_seen = True
                    self._nav_conf = True
                    self._set_phase(self.PHASE_NAV_ACTIVE)
            return
        if fn == 0x10 and payload:
            self._hud_status_10_seen = True
            self._hud_status_10_ready = payload[-1] == 0x50
            return
        if fn == 0x14 and payload:
            self._hud_status_14_seen = True
            self._hud_status_14_ready = payload[-1] == 0x50
            return

    def _hud_ready_for_route(self):
        return self._hud_status_0f_ready and self._hud_status_10_ready and self._hud_status_14_ready

    def _handle_hud_boot(self, data):
        if len(data) != 2 or data[0] != 0x11:
            self._vlog(f"[HUD_BOOT] raw={' '.join(f'{b:02X}' for b in data)}")
            return
        state = data[1]
        self._vlog(f"[HUD_BOOT] state=0x{state:02X}")
        if state == 0x02:
            self._boot_1102_seen = True
            self._log("[HUD_BOOT] 11 02 -> bootstrap request")
            self._dtx(HUD_S, [0x4C, 0x8F, 0x00])
        elif state == 0x01:
            self._boot_1101_seen = True
            self._log("[HUD_BOOT] 11 01 -> bootstrap ready")

    def _handle_request(self, can_id, data):
        if len(data) == 2 and data[0] == 0x1C and data[1] in (0x81, 0x82):
            self._handle_raw_handoff(can_id, data[1])
            return
        decoded = self._decode_request(data)
        if not decoded:
            self._vlog(f"[REQ] {hex(can_id)} undecoded {' '.join(f'{b:02X}' for b in data)}")
            return
        opcode, lsg, fn, payload = decoded
        self._vlog(f"[REQ] {hex(can_id)} op={opcode} lsg=0x{lsg:02X} fn=0x{fn:02X} payload={bytes(payload).hex().upper()}")
        if lsg != HUD_LSG:
            self._vlog(f"[REQ] ignore non-nav lsg=0x{lsg:02X}")
            return

        if fn == 0x18:
            lead = payload[0] if payload else 0x12
            if lead in (0x21, 0x22):
                self._log(f"[HUD] Active display config requested 0x{lead:02X}")
                self._send_display_config(lead)
                self._active_cfg_seen = True
                self._nav_conf = True
                self._set_phase(self.PHASE_NAV_ACTIVE)
                return
            self._log(f"[HUD] Init display config requested 0x{lead:02X}")
            self._init_cfg_seen = True
            self._init_cfg_lead = lead
            self._send_display_config(lead)
            self._clear_a4_nav_prime()
            self._maybe_arm_route_demo()
            return

        if fn == 0x36:
            raw_value = payload[0] if payload else 0x00
            value = raw_value
            self._log(f"[HUD] fn=0x36 handshake value=0x{raw_value:02X} -> reply=0x{value:02X}")
            self._fn36_seen = True
            self._dtx(HUD_S, bap_st(0x36) + [value])
            if (self._phase == self.PHASE_STANDBY_READY and self._init_cfg_lead == 0x12 and
                    not self._a4_seed_bundle_sent and
                    not self._a4_preready_poll10_promoted and
                    not self._a4_preready_poll11_promoted):
                self._a4_seed_bundle_sent = True
                self._log("[HUD] send A4 seed bundle after fn=0x36")
                self._queue_software_hud_seed_bundle()
            if value == 0x01:
                self._clear_a4_nav_prime()
            self._maybe_arm_route_demo()
            return

        if self._reply_known_hud_nav_function(fn, payload, f"REQ {hex(can_id)}", opcode=opcode):
            return

        self._vlog(f"[REQ] Unhandled fn=0x{fn:02X}")

    def _handle_asg(self, data):
        if len(data) >= 2 and data[0] == 0x1C and data[1] in (0x81, 0x82):
            self._handle_handoff_marker("HUD_ASG", data[1])
            return

        self._handle_request(HUD_ASG, data)

    def _handle_raw_handoff(self, can_id, marker):
        if can_id == HUD_GET:
            self._handle_handoff_marker("HUD_GET", marker)
            return
        self._vlog(f"[RAW] {hex(can_id)} 1C {marker:02X}")

    def _handoff_complete(self):
        return self._asg_81_done

    def _handoff_status_text(self):
        if self._handoff_complete():
            return "yes"
        pending = []
        if not self._asg_81_done:
            pending.append("asg")
        return "+".join(pending) if pending else "wait"

    def _should_handle_handoff(self, source, marker):
        key = (source, marker)
        now = time.monotonic()
        last = self._last_raw_handoff_at.get(key, 0.0)
        if now - last < 0.050:
            self._vlog(f"[HANDOFF] debounce {source} 1C {marker:02X}")
            return False
        self._last_raw_handoff_at[key] = now
        return True

    def _maybe_finish_handoff(self):
        if not self._handoff_complete():
            return
        if self._phase == self.PHASE_STANDBY_INIT:
            self._log("[HANDOFF] ASG complete -> STANDBY_READY")
            self._set_phase(self.PHASE_STANDBY_READY)

    def _handle_handoff_marker(self, source, marker):
        if not self._hud_inited:
            self._hud_init()
        if not self._should_handle_handoff(source, marker):
            return
        now = time.monotonic()

        if source == "HUD_ASG":
            if marker == 0x82:
                if self._asg_82_done:
                    self._vlog("[HANDOFF] ignore duplicate HUD_ASG 1C82")
                    return
                self._asg_82_done = True
                self._log("[HANDOFF] HUD_ASG 1C82 -> 0C82")
                self._schedule_activation_frame(now + 0.245, HUD_S,
                                                [0x0C, 0x82, 0x03, 0x00, 0x32, 0x00, 0x04, 0x08])
                return
            if marker == 0x81:
                if self._asg_81_done:
                    self._vlog("[HANDOFF] ignore duplicate HUD_ASG 1C81")
                    return
                self._asg_81_done = True
                self._log("[HANDOFF] HUD_ASG 1C81 -> fullinit bundle")
                self._send_fullinit_handoff_bundle("HUD_ASG 1C81")
                self._maybe_finish_handoff()
                return
            return

        if source == "HUD_GET":
            if marker == 0x82:
                if self._hud_get_82_seen:
                    self._vlog("[HANDOFF] ignore duplicate HUD_GET 1C82")
                    return
                self._hud_get_82_seen += 1
                self._log(f"[HANDOFF] HUD_GET 1C82 -> 4C82 ({self._hud_get_82_seen})")
                self._dtx(HUD_S, [0x4C, 0x82, 0x03, 0x00, 0x32, 0x00, 0x04, 0x08])
                return
            if marker == 0x81:
                if self._hud_get_81_seen:
                    self._vlog("[HANDOFF] ignore duplicate HUD_GET 1C81")
                    return
                self._hud_get_81_seen = True
                self._log("[HANDOFF] HUD_GET 1C81 complete")
                self._maybe_finish_handoff()
                return

    def _send_ready_ack(self):
        if self._ready_ack_sent or not self.enabled:
            return
        self._ready_ack_sent = True
        frame = [0x4C, 0x8F, 0x00]
        self._send_frames_atomic(HUD_S, [frame], ifg_s=0.0)
        self._mark_sent(HUD_S, frame)

    def _hud_fn_name(self, fn):
        return HUD_NAV_FUNCTION_NAMES.get(fn, f"0x{fn:02X}")

    def _function_list_payload(self):
        if self._phase == self.PHASE_NAV_ACTIVE:
            return list(A4_MIB3_READY_FUNCTION_LIST)
        return list(STANDBY_FUNCTION_LIST_INIT)

    def _route_guidance_status_payload(self):
        return [0x01 if self._nav_state["route_guidance_active"] else 0x00]

    def _fsg_operation_state_payload(self):
        return [self._nav_state["fsg_state"]]

    def _compass_info_payload(self):
        return list(self._nav_state["compass_info"])

    def _current_position_info_payload(self):
        return list(self._nav_state["current_position_info"])

    def _function_sync_payload(self):
        return list(self._nav_state["function_sync"])

    def _unknown35_payload(self):
        return list(DEFAULT_UNKNOWN35_PAYLOAD)

    def _feature_enable_payload(self):
        return list(DEFAULT_FEATURE_ENABLE_MASK)

    def _maneuver_state_payload(self):
        state = 0x04 if self._phase == self.PHASE_NAV_ACTIVE else 0x00
        return [0x04, state, 0x00, 0x00, 0x00, 0x00]

    def _send_maneuver_descriptor(self):
        arrow = self._current_arrow()
        h17 = bap_st(0x17)
        descriptor = [arrow["main"], arrow["dir"], 0x00, 0x00]
        self._tx_mf_atomic(HUD_S, [
            [0xA0, 0x0C] + h17 + descriptor,
            [0xC0, 0x00, 0x01, 0x3D, 0x01],
            [0xE0] + [0x00] * 7,
            [0xE1, 0x00],
        ])

    def _reply_known_hud_nav_function(self, fn, payload, source, opcode=None):
        fn_name = self._hud_fn_name(fn)
        if fn == 0x02:
            if source == "HUD_RX" and opcode == BAP_OP_STATUS:
                self._vlog(f"[BAP] ignore passive {source} {fn_name} status")
                return False
            self._dtx(HUD_S, [0x4C, 0x82, 0x03, 0x00, 0x32, 0x00, 0x04, 0x08])
            return True
        if fn == 0x03:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x03, self._function_list_payload(), atomic=True)
            return True
        if fn == 0x04:
            self._dtx(HUD_S, [0x4C, 0x84, HB_INTERVAL_S])
            return True
        if fn == 0x0D:
            if payload:
                self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, fn, list(payload), atomic=True)
                return True
            self._vlog(f"[BAP] skip empty {source} {fn_name}")
            return False
        if fn == 0x0F:
            reply_opcode = BAP_OP_HEARTBEAT_STATUS if opcode == BAP_OP_HEARTBEAT_STATUS else BAP_OP_STATUS
            self._tx_bap(HUD_S, reply_opcode, HUD_LSG, 0x0F, self._fsg_operation_state_payload(), atomic=True)
            return True
        if fn == 0x10:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x10, self._compass_info_payload(), atomic=True)
            return True
        if fn == 0x11:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x11, self._route_guidance_status_payload())
            return True
        if fn == 0x12:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x12, self._distance_payload(), atomic=True)
            return True
        if fn == 0x13:
            self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x13, self._current_position_info_payload(), atomic=True)
            return True
        if fn == 0x14:
            self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x14, list(self._nav_state["session_state"]), atomic=True)
            return True
        if fn == 0x15:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x15, self._destination_distance_payload(), atomic=True)
            return True
        if fn == 0x17:
            self._send_maneuver_descriptor()
            return True
        if fn == 0x21:
            arrow = self._current_arrow()
            bearing = arrow.get("bearing", 0x00)
            self._tx(HUD_D, [0x90, 0x05] + bap_hb(0x21) + [bearing, 0x02, 0x02, 0x03])
            self._tx(HUD_D, [0xD0, 0x00])
            return True
        if fn == 0x25:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x25, self._function_sync_payload(), atomic=True)
            return True
        if fn == 0x26:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x26, [self._nav_state["info_states"]])
            return True
        if fn == 0x32:
            self._tx_bap(HUD_D, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x32, self._feature_enable_payload(), atomic=True, force_long=True)
            return True
        if fn == 0x35:
            self._tx_bap(HUD_S, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x35, self._unknown35_payload(), atomic=True)
            return True
        if fn == 0x39:
            reply_opcode = BAP_OP_HEARTBEAT_STATUS if opcode == BAP_OP_HEARTBEAT_STATUS else BAP_OP_STATUS
            self._tx_bap(HUD_S, reply_opcode, HUD_LSG, 0x39, self._maneuver_state_payload(), atomic=True)
            return True
        self._vlog(f"[BAP] no response mapping for {source} {fn_name}")
        return False

    def _asg_should_handle(self, key_data):
        key = tuple(key_data[:2])
        now = time.monotonic()
        last = self._asg_last.get(key, 0.0)
        if now - last < 0.05:
            self._vlog(f"[HUD_ASG] debounce {' '.join(f'{b:02X}' for b in key_data)}")
            return False
        self._asg_last[key] = now
        return True

    def _decode_request(self, data):
        if len(data) >= 4 and data[0] == 0x80:
            header = (data[2] << 8) | data[3]
            opcode, lsg, fn = decode_bap_header(header)
            return (opcode, lsg, fn, list(data[4:]))
        if len(data) >= 2:
            header = (data[0] << 8) | data[1]
            opcode, lsg, fn = decode_bap_header(header)
            return (opcode, lsg, fn, list(data[2:]))
        return None

    def _dtx(self, arb_id, data):
        self._rx_pending.put((arb_id, data))

    def _send_fullinit_handoff_bundle(self, source):
        now = time.monotonic()
        if now - self._last_fullinit_bundle_at < 0.250:
            self._vlog(f"[HANDOFF] skip duplicate fullinit bundle from {source}")
            return
        self._last_fullinit_bundle_at = now
        self._vlog(f"[HANDOFF] schedule paced fullinit bundle from {source}")
        for offset_s, frame in zip(FULLINIT_HANDOFF_OFFSETS, FULLINIT_HANDOFF_BUNDLE):
            self._schedule_activation_frame(now + offset_s, HUD_D, frame)

    def _tx_mf_atomic(self, arb_id, frames, ifg_s=0.0004):
        self._send_frames_atomic(arb_id, frames, ifg_s=ifg_s)

    def _hud_init(self):
        if self._hud_inited:
            return
        self._hud_inited = True
        self._log("[INIT] HUD standby handshake armed")

    def _respond_to_active_poll(self, fn):
        return False

    def _respond_to_hud_poll(self, opcode, fn, payload):
        if self._phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE):
            if self._respond_to_active_poll(fn):
                return
        self._reply_known_hud_nav_function(fn, payload, "HUD_RX", opcode=opcode)

    def _maybe_arm_route_demo(self):
        if not (self._init_cfg_seen and self._fn36_seen):
            return
        if self._phase == self.PHASE_STANDBY_INIT:
            self._set_phase(self.PHASE_STANDBY_READY)

    def _send_display_config(self, lead):
        third = 0x01 if lead in (0x12, 0x16, 0x17) else 0x00
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x18, [lead, 0x00, third, 0x00, 0x00], atomic=True, force_long=True)

    def _send_maneuver_state_confirmation(self, value):
        self._fn36_seen = True
        self._tx(HUD_S, [0x4C, 0xB6, value & 0xFF])

    def _send_software_hud_seed_bundle(self):
        # Mirror the first visible bundle from the software HUD reference trace.
        self._tx(HUD_S, [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00])
        self._tx_mf_atomic(HUD_S, [
            [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x01, 0x2C, 0x01],
            [0xC0, 0x01, 0x00, 0x00],
        ])
        self._tx(HUD_S, [0x4C, 0x90, 0x00, 0x68, 0x01])
        self._tx_mf_atomic(HUD_D, [
            [0x80, 0x08, 0x4C, 0x93, 0x07, 0x4F, 0x66, 0x66],
            [0xC0, 0x72, 0x6F, 0x61, 0x64],
        ])
        self._tx(HUD_S, [0x4C, 0xAF, 0x00, 0x00, 0x00])

    def _queue_software_hud_seed_bundle(self):
        # Keep the bundle behind the queued fn=0x36 response.
        self._dtx(HUD_S, [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00])
        self._dtx(HUD_S, [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x01, 0x2C, 0x01])
        self._dtx(HUD_S, [0xC0, 0x01, 0x00, 0x00])
        self._dtx(HUD_S, [0x4C, 0x90, 0x00, 0x68, 0x01])
        self._dtx(HUD_D, [0x80, 0x08, 0x4C, 0x93, 0x07, 0x4F, 0x66, 0x66])
        self._dtx(HUD_D, [0xC0, 0x72, 0x6F, 0x61, 0x64])
        self._dtx(HUD_S, [0x4C, 0xAF, 0x00, 0x00, 0x00])

    def _queue_pre_active_nav_bundle(self):
        base = time.monotonic()
        if self._activation_script:
            base = max(base, self._activation_script[-1]["due_at"] + 0.010)
        self._set_phase(self.PHASE_PRE_ACTIVE)
        self._pre_active_bundle_sent = True
        self._pre_active_running_sent = False
        self._pre_active_running_due_at = base + 5.000

        # Mirror the car traces: announce data readiness while keeping FSG in 0x03,
        # then promote to normal-operation 0x00 after the HUD has seen the staging bundle.
        self._schedule_activation_frame(base + 0.010, HUD_S, [0x4C, 0x90, 0x0F, 0xFF, 0xFF])
        self._schedule_activation_frame(base + 0.020, HUD_D, [0x4C, 0x93, 0x00])
        self._schedule_activation_frame(base + 0.030, HUD_S, [0x4C, 0x91, 0x00])
        self._schedule_activation_frame(base + 0.040, HUD_D, [0x4C, 0x94, 0x00, 0x00])
        self._schedule_activation_frame(base + 0.050, HUD_S, [0x4C, 0x95, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])
        self._schedule_activation_frame(base + 0.060, HUD_D, [0x4C, 0x94, 0x00, 0x00])
        self._schedule_activation_frame(base + 0.070, HUD_S, [0x4C, 0x8F, 0x03])
        self._schedule_activation_frame(base + 0.090, HUD_S, [0x4C, 0x95, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])
        self._schedule_activation_frame(base + 0.120, HUD_S, [0x4C, 0xA6, 0xFF])
        self._schedule_activation_atomic(base + 0.150, HUD_S, [
            [0x80, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x40, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 0.240, HUD_S, [0x4C, 0x8F, 0x03])
        self._schedule_activation_frame(base + 0.270, HUD_S, [0x4C, 0xA6, 0xFF])
        self._schedule_activation_atomic(base + 0.300, HUD_S, [
            [0x80, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x40, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 0.390, HUD_D, [0x4C, 0x93, 0x00])
        self._schedule_activation_frame(base + 0.420, HUD_S, [0x4C, 0x8F, 0x03])
        self._schedule_activation_atomic(base + 0.450, HUD_S, [
            [0x80, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x00, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 0.900, HUD_S, [0x4C, 0x90, 0x02, 0x35, 0x01])
        self._schedule_activation_frame(base + 0.930, HUD_S, [0x4C, 0x8F, 0x03])
        self._schedule_activation_frame(base + 0.960, HUD_S, [0x4C, 0xA6, 0xFF])
        self._schedule_activation_atomic(base + 0.990, HUD_S, [
            [0x80, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x40, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 1.140, HUD_S, [0x4C, 0x91, 0x00])
        self._schedule_activation_atomic(base + 1.170, HUD_S, [
            [0x80, 0x08, 0x4C, 0xA5, 0x00, 0x00, 0x00, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_atomic(self._pre_active_running_due_at, HUD_S, [
            [0x4C, 0x8F, 0x00],
        ])

    def _queue_a4_postseed_followup_script(self):
        base = time.monotonic()
        if self._activation_script:
            base = max(base, self._activation_script[-1]["due_at"] + 0.010)
        self._schedule_activation_frame(base + 0.210, HUD_S, [0x4C, 0xAD, 0x00, 0x01, 0xF4, 0x01, 0x00, 0x01])
        self._schedule_activation_frame(base + 0.360, HUD_S, [0x4C, 0xAD, 0x00, 0x01, 0xEE, 0x02, 0x00, 0x01])
        self._schedule_activation_frame(base + 0.590, HUD_S, [0x4C, 0xAB, 0x02, 0x02, 0xFF, 0x00])
        self._schedule_activation_frame(base + 0.620, HUD_S, [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00])
        self._schedule_activation_atomic(base + 0.650, HUD_S, [
            [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x11, 0xEE, 0x02],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._schedule_activation_frame(base + 0.710, HUD_S, [0x4C, 0xB9, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 0.770, HUD_D, [
            [0x80, 0x02, 0x4C, 0x94, 0x00, 0x00],
        ])
        self._schedule_activation_atomic(base + 0.780, HUD_S, [
            [0x90, 0x08, 0x4C, 0x92, 0x00, 0x00, 0x00, 0x00],
            [0xD0, 0xFF, 0xFF, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 0.840, HUD_S, [0x4C, 0xAD, 0x00, 0x11, 0xE8, 0x03, 0x00, 0x01])
        self._schedule_activation_frame(base + 0.870, HUD_S, [0x4C, 0xAB, 0x02, 0x02, 0xFF, 0x1D])
        self._schedule_activation_atomic(base + 0.900, HUD_S, [
            [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x01],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._schedule_activation_frame(base + 0.960, HUD_S, [0x4C, 0xB7, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 1.190, HUD_D, [
            [0x90, 0x05, 0x3C, 0xA1, 0x80, 0x00, 0x00, 0xFF],
            [0xD0, 0xFF],
        ])
        self._schedule_activation_frame(base + 1.200, HUD_S, [0x4C, 0xB5, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00])

    def _send_software_hud_route_bundle(self):
        self._tx(HUD_S, [0x4C, 0xAB, 0x02, 0x02, 0xFF, 0x00])
        self._tx(HUD_S, [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00])
        self._tx_mf_atomic(HUD_S, [
            [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x11, 0xEE, 0x02],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._tx(HUD_S, [0x4C, 0xB9, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._tx(HUD_S, [0x4C, 0xB5, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00])
        self._tx(HUD_S, [0x4C, 0xB7, 0x00, 0x00, 0x00, 0x00])
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x14, [0x00, 0x00], atomic=True, force_long=True)

    def _send_software_hud_details_bundle(self):
        # Static nav detail block copied from the software HUD reference.
        self._tx(HUD_S, [0x4C, 0x95, 0x88, 0x13, 0x00, 0x00, 0x00, 0x01])
        self._tx_mf_atomic(HUD_D, [
            [0x80, 0x5B, 0x4C, 0xAE, 0x47, 0x0E, 0xE8, 0x02],
            [0x90, 0x10, 0x4C, 0xB2, 0x00, 0x00, 0x00, 0x00],
            [0xC0, 0xBE, 0x65, 0xAE, 0x00, 0x01, 0xFF, 0xFF],
            [0xD0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00],
            [0xC1, 0x20, 0x53, 0x63, 0x68, 0x6C, 0x6F, 0xC3],
            [0xD1, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0xC2, 0x9F, 0x6C, 0xC3, 0xA4, 0x6E, 0x64, 0x65],
            [0xC3, 0x20, 0x44, 0x20, 0x49, 0x6E, 0x67, 0x6F],
            [0xC4, 0x6C, 0x73, 0x74, 0x61, 0x64, 0x74, 0x20],
            [0xC5, 0x38, 0x35, 0x30, 0x34, 0x39, 0x0D, 0x53],
            [0xC6, 0x63, 0x68, 0x6C, 0x6F, 0xC3, 0x9F, 0x6C],
            [0xC7, 0xC3, 0xA4, 0x6E, 0x64, 0x65, 0x0A, 0x49],
            [0xC8, 0x6E, 0x67, 0x6F, 0x6C, 0x73, 0x74, 0x61],
            [0xC9, 0x64, 0x74, 0x07, 0x42, 0x61, 0x76, 0x61],
            [0xCA, 0x72, 0x69, 0x61, 0x05, 0x38, 0x35, 0x30],
            [0xCB, 0x34, 0x39, 0x07, 0x47, 0x65, 0x72, 0x6D],
            [0xCC, 0x61, 0x6E, 0x79],
        ])
        self._tx_mf_atomic(HUD_S, [
            [0x80, 0x07, 0x4C, 0x96, 0x10, 0x24, 0x12, 0x0F],
            [0xC0, 0x02, 0x19, 0x3E],
        ])
        self._tx_mf_atomic(HUD_S, [
            [0x80, 0x0B, 0x4C, 0xBC, 0x88, 0x13, 0x00, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01],
        ])

    def _schedule_activation_frame(self, due_at, arb_id, frame):
        self._activation_script.append({
            "due_at": due_at,
            "arb_id": arb_id,
            "frames": [list(frame)],
        })

    def _schedule_activation_atomic(self, due_at, arb_id, frames):
        self._activation_script.append({
            "due_at": due_at,
            "arb_id": arb_id,
            "frames": [list(frame) for frame in frames],
        })

    def _service_activation_script(self, now):
        if not self._activation_script or not self.enabled:
            return
        if self._activation_script[0]["due_at"] > now:
            return
        item = self._activation_script.pop(0)
        frames = item["frames"]
        if len(frames) == 1:
            self._tx(item["arb_id"], frames[0])
        else:
            self._tx_mf_atomic(item["arb_id"], frames)

    def _queue_a4_mib3_preready_script(self):
        base = time.monotonic()
        if self._activation_script:
            base = max(base, self._activation_script[-1]["due_at"] + 0.700)
        self._schedule_activation_atomic(base + 0.800, HUD_S, [
            [0x80, 0x07, 0x3C, 0xAC, 0x00, 0x00, 0x01, 0x01],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._schedule_activation_frame(base + 1.800, HUD_S, [0x3C, 0xAD, 0x00, 0x11, 0xE8, 0x03, 0x00, 0x01])
        self._schedule_activation_frame(base + 2.800, HUD_S, [0x3C, 0xAF, 0x00, 0x00, 0x00])
        self._schedule_activation_frame(base + 3.800, HUD_S, [0x3C, 0xB1, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 4.800, HUD_D, [
            [0x80, 0x10, 0x3C, 0xB2, 0x00, 0x00, 0x00, 0x00],
            [0xC0, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xF0, 0x00],
            [0xC1, 0x00, 0x00, 0x00, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 5.800, HUD_S, [0x3C, 0xB5, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00])
        self._schedule_activation_frame(base + 6.800, HUD_S, [0x3C, 0xB6, 0x01])
        self._schedule_activation_frame(base + 7.800, HUD_S, [0x3C, 0xB7, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_frame(base + 8.800, HUD_S, [0x3C, 0xB9, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 9.800, HUD_S, [
            [0x80, 0x0B, 0x3C, 0xBC, 0x00, 0x00, 0x00, 0x00],
            [0xC0, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00],
        ])

    def _queue_reference_activation_script(self):
        base = time.monotonic()
        self._activation_script = []

        # Replay the first visible navigation activation bundle using
        # the same ordering and rough spacing as the software HUD trace.
        self._schedule_activation_frame(base + 0.030, HUD_S, [0x4C, 0xA6, 0x05])
        self._schedule_activation_frame(base + 0.140, HUD_S, [0x4C, 0x8F, 0x00])

        self._schedule_activation_atomic(base + 0.430, HUD_S, [
            [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00],
        ])
        self._schedule_activation_atomic(base + 0.460, HUD_S, [
            [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x01, 0x2C, 0x01],
            [0xC0, 0x01, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 0.640, HUD_S, [0x4C, 0x90, 0x00, 0x68, 0x01])
        self._schedule_activation_atomic(base + 0.641, HUD_D, [
            [0x80, 0x08, 0x4C, 0x93, 0x07, 0x4F, 0x66, 0x66],
            [0xC0, 0x72, 0x6F, 0x61, 0x64],
        ])
        self._schedule_activation_frame(base + 0.670, HUD_S, [0x4C, 0xAF, 0x00, 0x00, 0x00])

        self._schedule_activation_frame(base + 1.080, HUD_S, [0x4C, 0xAD, 0x00, 0x01, 0xF4, 0x01, 0x00, 0x01])
        self._schedule_activation_frame(base + 1.230, HUD_S, [0x4C, 0xAD, 0x00, 0x01, 0xEE, 0x02, 0x00, 0x01])
        self._schedule_activation_frame(base + 1.460, HUD_S, [0x4C, 0xAB, 0x02, 0x02, 0xFF, 0x00])
        self._schedule_activation_atomic(base + 1.490, HUD_S, [
            [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x00],
        ])
        self._schedule_activation_atomic(base + 1.520, HUD_S, [
            [0x80, 0x08, 0x4C, 0xAD, 0x00, 0x11, 0xEE, 0x02],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._schedule_activation_frame(base + 1.580, HUD_S, [0x4C, 0xB9, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 1.640, HUD_D, [
            [0x80, 0x02, 0x4C, 0x94, 0x00, 0x00],
        ])
        self._schedule_activation_atomic(base + 1.650, HUD_S, [
            [0x90, 0x08, 0x4C, 0x92, 0x00, 0x00, 0x00, 0x00],
            [0xD0, 0xFF, 0xFF, 0x00, 0x00],
        ])
        self._schedule_activation_frame(base + 1.710, HUD_S, [0x4C, 0xAD, 0x00, 0x11, 0xE8, 0x03, 0x00, 0x01])
        self._schedule_activation_frame(base + 1.740, HUD_S, [0x4C, 0xAB, 0x02, 0x02, 0xFF, 0x1D])
        self._schedule_activation_atomic(base + 1.770, HUD_S, [
            [0x80, 0x07, 0x4C, 0xAC, 0x00, 0x00, 0x01, 0x01],
            [0xC0, 0x01, 0x02, 0x00],
        ])
        self._schedule_activation_frame(base + 1.830, HUD_S, [0x4C, 0xB7, 0x00, 0x00, 0x00, 0x00])
        self._schedule_activation_atomic(base + 2.060, HUD_D, [
            [0x90, 0x05, 0x3C, 0xA1, 0x80, 0x00, 0x00, 0xFF],
            [0xD0, 0xFF],
        ])
        self._schedule_activation_frame(base + 2.070, HUD_S, [0x4C, 0xB5, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00])

    def _prime_nav_display_state(self):
        # Mirror the reference promotion into a visible nav-ready display state.
        self._send_display_config(0x12)
        self._clear_a4_nav_prime()
        self._send_maneuver_state_confirmation(0x01)
        self._tx(HUD_S, [0x4C, 0xA7, 0x04])

    def _promote_nav_display_state(self):
        self._send_display_config(0x21)
        self._active_cfg_seen = True
        self._nav_conf = True
        self._send_maneuver_state_confirmation(0x01)
        self._tx(HUD_S, [0x4C, 0xB1, 0x00, 0x00, 0x00])
        self._set_phase(self.PHASE_NAV_ACTIVE)

    def _prime_a4_nav_activation(self):
        if self._a4_nav_prime_sent:
            return
        self._a4_nav_prime_sent = True
        self._log("[INIT] A4 nav prime -> 4CA605")
        self._tx(HUD_S, [0x4C, 0xA6, 0x05])

    def _clear_a4_nav_prime(self):
        if self._a4_nav_prime_cleared:
            return
        self._a4_nav_prime_cleared = True
        self._tx(HUD_S, [0x4C, 0xA6, 0x00])

    def _start_route_demo(self):
        self._route_burst_sent = True
        self._set_phase(self.PHASE_WAIT_ACTIVE_CONFIG)
        self._log("[NAV] Starting paced route demo trigger")
        self._queue_reference_activation_script()

    def start_nav_demo(self):
        if not self.enabled:
            self._log("[NAV] Manual trigger ignored: ignition off")
            return False
        if self._phase in (self.PHASE_WAIT_ACTIVE_CONFIG, self.PHASE_NAV_ACTIVE):
            self._log("[NAV] Manual trigger ignored: navigation already active")
            return False
        self._demo_route = True
        self._log("[NAV] Manual trigger requested")
        if self._phase == self.PHASE_STANDBY_READY and self._hud_ready_for_route():
            self._start_route_demo()
        else:
            self._log("[NAV] Manual trigger armed; waiting for HUD ready state")
        return True

    def _distance_payload(self):
        dist_m = self._nav_state["next_maneuver_distance_m"]
        if not self._nav_state["next_maneuver_visible"] or dist_m is None:
            return [0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00]
        dist_dm = int(dist_m) * 10
        graph = int(self._cfg.get("hud_distance_graph", 0x64)) & 0xFF
        return [
            dist_dm & 0xFF,
            (dist_dm >> 8) & 0xFF,
            (dist_dm >> 16) & 0xFF,
            (dist_dm >> 24) & 0xFF,
            0x00, 0x01, graph, 0x01,
        ]

    def _send_active_visual_keepalive(self):
        self._sync_nav_state()
        arrow = self._current_arrow()
        bearing = arrow.get("bearing", 0x00)
        self._vlog(f"[VIS] bearing=0x{bearing:02X}")
        self._tx(HUD_S, bap_hb(0x11) + self._route_guidance_status_payload())
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x13, self._current_position_info_payload(), atomic=True, force_long=True)
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x39, self._maneuver_state_payload(), atomic=True)
        self._tx(HUD_S, bap_hb(0x35) + self._unknown35_payload())
        if self._nav_state["next_maneuver_visible"] and not arrow.get("offroad", False):
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x12, self._distance_payload(), atomic=True)
            self._send_maneuver_descriptor()
        self._tx(HUD_D, bap_hb(0x1D) + [0x00, 0x00, 0xFF])
        self._tx(HUD_D, [0x90, 0x05] + bap_hb(0x21) + [bearing, 0x02, 0x02, 0x03])
        self._tx(HUD_D, [0xD0, 0x00])
        self._tx_bap(HUD_D, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x32, self._feature_enable_payload(), atomic=True, force_long=True)

    def _send_hud_route_payloads(self):
        self._sync_nav_state()
        arrow = self._current_arrow()
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x12, self._distance_payload(), atomic=True)
        self._send_maneuver_descriptor()
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x25, self._function_sync_payload(), atomic=True)
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x11, self._route_guidance_status_payload())
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x15, self._destination_distance_payload(), atomic=True)
        self._send_active_visual_keepalive()
        self._log(f"[ARROW] -> HUD {arrow['name']} dist={arrow['dist_m']}m")

