"""
modules/5f_infotainment.py — Infotainment / HUD BAP emulation.

This module now uses a small explicit state machine derived from the
reference non-nav and nav logs instead of the old 23-step replay.
"""

import copy
import queue
import time

from ecu_base import ECUModule
from bap import BAP_LSG_HUD_NAV, BAP_OP_HEARTBEAT_STATUS, BAP_OP_STATUS, build_bap_header, decode_bap_header, function_label

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
NAV_DATA_REFRESH_S = 4.0
DEFAULT_COMPASS_INFO = [0x02, 0x35, 0x01]
DEFAULT_CURRENT_POSITION_INFO_OFFROAD = [0x07, 0x4F, 0x66, 0x66, 0x72, 0x6F, 0x61, 0x64]
# FSG_Setup (0x35): VoiceGuidance, POI_Types, FunctionSupport, Dummy2-4 (BAP-FC-NAV-SD p.243)
DEFAULT_FSG_SETUP_PAYLOAD = [0x00, 0x00, 0x30, 0x00, 0x00, 0x00]
DEFAULT_FEATURE_ENABLE_MASK = [
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
]
STANDBY_FUNCTION_LIST_INIT = [0x78, 0x01, 0xDC, 0x00, 0x06, 0x00, 0x00, 0x00]
STANDBY_FUNCTION_LIST_READY = [0x78, 0x01, 0xFF, 0x86, 0x67, 0x5F, 0x67, 0x48]
DISTANCE_INACTIVE = [0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]
DISTANCE_TO_DESTINATION_ACTIVE = [0x7C, 0x0B, 0x00, 0x00, 0x01, 0x01]
FUNC_SYNC_IDLE = [0xff] * 8
FUNC_SYNC_ACTIVE = [0x00] * 8
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
    "offroad": {"name": "Offroad", "main": 0x09, "dir": 0x00, "dist_m": 0, "offroad": True},
    "right": {"name": "Turn Right", "main": 0x0D, "dir": 0x40, "dist_m": 500, "offroad": False},
    "left": {"name": "Turn Left", "main": 0x0D, "dir": 0xC0, "dist_m": 500, "offroad": False},
    "straight": {"name": "Straight", "main": 0x0B, "dir": 0x00, "dist_m": 500, "offroad": False},
}

# Central source of truth for live HUD navigation content.
NAV_STATE_TEMPLATE = {
    "0F_FSGOperationState": 0x00,
    "10_CompassInfo": list(DEFAULT_COMPASS_INFO),
    "11_RouteGuidanceStatus_active": False,
    "12_DistanceToNextManeuver_visible": False,
    "12_DistanceToNextManeuver_m": None,
    "13_CurrentPositionInfo": list(DEFAULT_CURRENT_POSITION_INFO_OFFROAD),
    # TurnToInfo session: [turn_len, session] (BAP-FC-NAV-SD p.41). Minimal payload for nav session state.
    # turn_len=0 (no street), session: 0x00=inactive, 0x01=active
    "14_TurnToInfo_session": [0x00, 0x01],
    "15_DistanceToDestination_m": None,
    "15_DistanceToDestination": list(DISTANCE_TO_DESTINATION_ACTIVE),
    # ManeuverDescriptor (0x17): up to 3 maneuvers, each MainElement+Direction+Z_Level+Sidestreets (BAP-FC-NAV-SD p.53)
    # Maneuver_1: [main, dir, z_level, sidestreets_len]; M2,M3 empty (0x00) for now
    "17_ManeuverDescriptor": [
        0x0B, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00],
    "25_FunctionSynchronisation": list(FUNC_SYNC_IDLE),
    # InfoStates (0x26): 0x00=no error, 0x01=no nav medium, 0x02=db corrupted, 0x03=no GPS,
    # 0x04=db update ongoing, 0x05=init MOST Map, 0x06=mobile nav active, 0x07=no db, 0xFF=unknown (BAP-FC-NAV-SD p.158)
    "26_InfoStates": 0x00,
    # MIB3 periodic extras (hardcoded from log)
    "27_ActiveRgType": [0x04],
    "31_Exitview": [0x00, 0x00, 0x00],
    "14_TurnToInfo_street": [0x07, 0x4F, 0x66, 0x66, 0x72, 0x6F, 0x61, 0x64, 0x00],  # "Offroad"
    "16_TimeToDestination": [0x10, 0x17, 0x13, 0x09, 0x03, 0x1A, 0x3E],
    "2D_MapScale": [0x00, 0x11, 0xE8, 0x03, 0x01, 0x01],
    "2F_Altitude": [0x00, 0x00, 0x00],
    # ManeuverState: [State, Dummy1, Dummy2, Dummy3] (BAP-FC-NAV-SD p.250)
    # State: 0x00=init/unknown, 0x01=Follow, 0x02=Prepare, 0x03=Distance, 0x04=CallForAction
    "37_ManeuverState": [0x04, 0x00, 0x00, 0x00],
    "3C_DistanceToDestinationExtended": [0x86, 0x0B, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01],
    "2E_DestinationInfo": [
        0x2F, 0x61, 0xE6, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF,
        0x07, 0x4F, 0x66, 0x66, 0x72, 0x6F, 0x61, 0x64, 0x00,
    ],
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

    PHASE_BOOT = "BOOT"
    PHASE_READY = "READY"

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

        self._phase = self.PHASE_BOOT
        self._hud_inited = False
        self._hud_bap_ok = False
        self._status_t = 0.0
        self._asg_last = {}
        self._asg_81_done = False
        self._asg_82_done = False
        self._boot_1102_seen = False
        self._boot_1101_seen = False
        self._hud_fsg_setup_seen = False
        self._hud_get_82_seen = 0
        self._hud_get_81_seen = False
        self._active_visual_keepalive_next_at = 0.0
        self._route_guidance_status_next_at = 0.0
        self._activation_script = []
        self._last_fullinit_bundle_at = 0.0
        self._last_raw_handoff_at = {}
        self._reset_nav_state()

    def set_enabled(self, on: bool):
        was_enabled = self.enabled
        self.enabled = on
        if was_enabled and not on:
            self._log("Ignition OFF — resetting HUD state")
            self._reset_session()

    def get_nav_settings(self):
        arrow = self._configured_nav_arrow()
        return {
            "enabled": bool(self._cfg.get("hud_nav_enabled", False)),
            "distance_enabled": bool(self._cfg.get("hud_distance_enabled", True)),
            "distance_m": max(0, int(self._cfg.get("hud_distance_m", arrow.get("dist_m", 0)))),
            "distance_graph": int(self._cfg.get("hud_distance_graph", 0x64)) & 0xFF,
            "street_name": self._cfg.get("hud_street_name", "Offroad"),
            "arrow_main": arrow.get("main", 0x00),
            "arrow_dir": arrow.get("dir", 0x00),
        }

    def configure_nav(self, *, enabled=None, distance_enabled=None, distance_m=None, street_name=None,
                      arrow_main=None, arrow_dir=None,
                      distance_graph=None):
        was_enabled = bool(self._cfg.get("hud_nav_enabled", False))
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
        self._sync_nav_state()
        nav_enabled = bool(self._cfg.get("hud_nav_enabled", False))
        if self._phase == self.PHASE_READY:
            self._send_hud_route_payloads()
        elif nav_enabled and not was_enabled and self._phase == self.PHASE_READY and self._hud_ready_for_route():
            self._log("[NAV] Navigation enabled -> start nav trigger")
            self._demo_route = True
            self._active_visual_keepalive_next_at = time.monotonic() + NAV_DATA_REFRESH_S

    def _reset_session(self):
        self._phase = self.PHASE_BOOT
        self._hud_inited = False
        self._hud_bap_ok = False
        self._status_t = 0.0
        self._demo_route = False
        self._asg_last.clear()
        self._asg_81_done = False
        self._asg_82_done = False
        self._boot_1102_seen = False
        self._boot_1101_seen = False
        self._hud_fsg_setup_seen = False
        self._hud_get_82_seen = 0
        self._hud_get_81_seen = False
        self._active_visual_keepalive_next_at = 0.0
        self._route_guidance_status_next_at = 0.0
        self._activation_script = []
        self._last_fullinit_bundle_at = 0.0
        self._last_raw_handoff_at = {}
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
            if phase == self.PHASE_READY:
                self._active_visual_keepalive_next_at = time.monotonic() + NAV_DATA_REFRESH_S
        self._sync_nav_state()

    def _vlog(self, msg):
        if self._verbose:
            self._log(msg)

    def _configured_nav_arrow(self):
        preset = dict(ARROW_PRESETS.get(self._cfg.get("hud_arrow", "straight"), ARROW_PRESETS["straight"]))
        if not preset.get("offroad"):
            preset["dist_m"] = max(0, int(self._cfg.get("hud_distance_m", preset["dist_m"])))
        main_override = self._cfg.get("hud_arrow_main")
        dir_override = self._cfg.get("hud_arrow_dir")
        custom_arrow = any(value is not None for value in (main_override, dir_override))
        if main_override is not None:
            preset["main"] = int(main_override) & 0xFF
        if dir_override is not None:
            preset["dir"] = int(dir_override) & 0xFF
        if custom_arrow:
            preset["offroad"] = preset.get("main", 0x00) == 0x09
            preset["name"] = f"0x{preset['main']:02X}/0x{preset['dir']:02X}"
        return preset

    def _reset_nav_state(self):
        self._nav_state = copy.deepcopy(NAV_STATE_TEMPLATE)
        self._sync_nav_state()

    def _sync_nav_state(self):
        if not hasattr(self, "_nav_state"):
            return
        state = self._nav_state
        arrow = self._configured_nav_arrow()
        nav_enabled = bool(self._cfg.get("hud_nav_enabled", False))
        distance_enabled = bool(self._cfg.get("hud_distance_enabled", True))
        if self._phase == self.PHASE_READY and nav_enabled:
            state["0F_FSGOperationState"] = 0x00
            state["10_CompassInfo"] = list(DEFAULT_COMPASS_INFO)
            state["11_RouteGuidanceStatus_active"] = True
            state["12_DistanceToNextManeuver_visible"] = not arrow.get("offroad", False)
            state["12_DistanceToNextManeuver_m"] = max(int(arrow.get("dist_m", 0)), 0) if state["12_DistanceToNextManeuver_visible"] and distance_enabled else None
            state["13_CurrentPositionInfo"] = self._encode_text_payload(self._cfg.get("hud_street_name", "Offroad"))
            state["14_TurnToInfo_session"] = [0x00, 0x00]
            state["15_DistanceToDestination_m"] = max(int(arrow.get("dist_m", 0)), 0) if state["12_DistanceToNextManeuver_visible"] and distance_enabled else None
            state["17_ManeuverDescriptor"] = [arrow["main"], arrow["dir"], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            state["_21_val"] = 0x5F if arrow.get("offroad") else arrow.get("dir", 0x00)
            state["25_FunctionSynchronisation"] = list(FUNC_SYNC_ACTIVE)
            state["26_InfoStates"] = 0xFF
            return
        state["0F_FSGOperationState"] = 0x00
        state["10_CompassInfo"] = [0x00, 0x00, 0x00]
        state["11_RouteGuidanceStatus_active"] = False
        state["12_DistanceToNextManeuver_visible"] = False
        state["12_DistanceToNextManeuver_m"] = None
        state["13_CurrentPositionInfo"] = self._encode_text_payload(self._cfg.get("hud_street_name", "Offroad"))
        state["14_TurnToInfo_session"] = [0x00, 0x01]
        state["15_DistanceToDestination_m"] = None
        state["17_ManeuverDescriptor"] = [arrow["main"], arrow["dir"], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        state["_21_val"] = 0x5F if arrow.get("offroad") else arrow.get("dir", 0x00)
        state["25_FunctionSynchronisation"] = list(FUNC_SYNC_IDLE)
        state["26_InfoStates"] = 0x00

    def _destination_distance_payload(self):
        dist_m = self._nav_state["15_DistanceToDestination_m"]
        if dist_m is None:
            return list(DISTANCE_INACTIVE)
        return list(self._nav_state["15_DistanceToDestination"])

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
        self._sync_nav_state()
        if now >= self._route_guidance_status_next_at:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x11, self._route_guidance_status_payload())
            self._route_guidance_status_next_at = now + NAV_DATA_REFRESH_S
        if self._phase == self.PHASE_READY and self._demo_route and now >= self._active_visual_keepalive_next_at:
            self._send_active_visual_keepalive()
            self._active_visual_keepalive_next_at = now + NAV_DATA_REFRESH_S

        if now - self._status_t >= 5.0:
            self._status_t = now
            self._log(
                f"[STATUS] phase={self._phase}"
                f" hud_bap={'OK' if self._hud_bap_ok else 'wait'}"
                f" boot={'fsg-setup' if self._hud_fsg_setup_seen else '11-01' if self._boot_1101_seen else '11-02' if self._boot_1102_seen else 'wait'}"
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
        known_hud_rx = {0x01, 0x02, 0x03, 0x04, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x21, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x2F, 0x31, 0x32, 0x35, 0x37, 0x39, 0x3C}
        if fn not in known_hud_rx:
            self._vlog(f"[HUD_RX] ignore unsupported fn=0x{fn:02X} lsg=0x{lsg:02X}")
            return
        if fn == 0x0E and payload[:2] == [0x74, 0x19]:
            self._mark_hud_fsg_setup_ready()
        if opcode == BAP_OP_STATUS:
            self._handle_hud_status(fn, payload)
        self._respond_to_hud_poll(opcode, fn, payload)

    def _mark_hud_fsg_setup_ready(self):
        if self._hud_fsg_setup_seen:
            return
        self._hud_fsg_setup_seen = True
        self._log("[HUD] 0x0E FSG-Setup 74 19 -> nav-ready gate open")
        self._maybe_finish_handoff()

    def _handle_hud_status(self, fn, payload):
        pass

    def _hud_ready_for_route(self):
        return self._handoff_complete() and self._hud_fsg_setup_seen

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
            self._log(f"[HUD] Display config requested 0x{lead:02X}")
            self._send_display_config(lead)
            return

        if fn == 0x36:
            raw_value = payload[0] if payload else 0x00
            value = raw_value
            self._log(f"[HUD] fn=0x36 handshake value=0x{raw_value:02X} -> reply=0x{value:02X}")
            self._dtx(HUD_S, bap_st(0x36) + [value])
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
        if not self._hud_fsg_setup_seen:
            return
        if self._phase == self.PHASE_BOOT:
            self._log("[BOOT] handoff complete + FSG-Setup -> READY")
            self._set_phase(self.PHASE_READY)

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

    def _hud_fn_name(self, fn):
        return function_label(fn)

    def _function_list_payload(self):
        return list(A4_MIB3_READY_FUNCTION_LIST)

    def _route_guidance_status_payload(self):
        return [0x01 if self._nav_state["11_RouteGuidanceStatus_active"] else 0x00]

    def _fsg_operation_state_payload(self):
        return [self._nav_state["0F_FSGOperationState"]]

    def _compass_info_payload(self):
        return list(self._nav_state["10_CompassInfo"])

    def _current_position_info_payload(self):
        return list(self._nav_state["13_CurrentPositionInfo"])

    def _function_sync_payload(self):
        return list(self._nav_state["25_FunctionSynchronisation"])

    def _fsg_setup_payload(self):
        return list(DEFAULT_FSG_SETUP_PAYLOAD)

    def _feature_enable_payload(self):
        return list(DEFAULT_FEATURE_ENABLE_MASK)

    def _maneuver_state_payload(self):
        state = 0x04 if self._phase == self.PHASE_READY else 0x00
        return [0x04, state, 0x00, 0x00, 0x00, 0x00]

    def _send_maneuver_descriptor(self):
        payload = list(self._nav_state["17_ManeuverDescriptor"])
        h17 = bap_st(0x17)
        # BAP multi-frame: first frame 8 bytes max; 0x80/0x0C = first frame, 12-byte total; payload split across frames
        frames = [[0x80, 0x0C] + h17 + payload[:4]]
        frames.append([0xC0] + payload[4:11])
        frames.append([0xC1] + payload[11:12])
        self._tx_mf_atomic(HUD_S, frames)

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
            self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x14, list(self._nav_state["14_TurnToInfo_session"]), atomic=True)
            return True
        if fn == 0x15:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x15, self._destination_distance_payload(), atomic=True)
            return True
        if fn == 0x17:
            self._send_maneuver_descriptor()
            return True
        if fn == 0x21:
            val = self._nav_state.get("_21_val", 0x00)
            self._tx(HUD_D, [0x90, 0x05] + bap_hb(0x21) + [val, 0x02, 0x02, 0x03])
            self._tx(HUD_D, [0xD0, 0x00])
            return True
        if fn == 0x25:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x25, self._function_sync_payload(), atomic=True)
            return True
        if fn == 0x26:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x26, [self._nav_state["26_InfoStates"]])
            return True
        if fn == 0x32:
            self._tx_bap(HUD_D, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x32, self._feature_enable_payload(), atomic=True, force_long=True)
            return True
        if fn == 0x35:
            self._tx_bap(HUD_S, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x35, self._fsg_setup_payload(), atomic=True)
            return True
        if fn == 0x39:
            reply_opcode = BAP_OP_HEARTBEAT_STATUS if opcode == BAP_OP_HEARTBEAT_STATUS else BAP_OP_STATUS
            self._tx_bap(HUD_S, reply_opcode, HUD_LSG, 0x39, self._maneuver_state_payload(), atomic=True)
            return True
        if fn == 0x27:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x27, list(self._nav_state["27_ActiveRgType"]))
            return True
        if fn == 0x31:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x31, list(self._nav_state["31_Exitview"]))
            return True
        if fn == 0x16:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x16, list(self._nav_state["16_TimeToDestination"]))
            return True
        if fn == 0x2D:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x2D, list(self._nav_state["2D_MapScale"]))
            return True
        if fn == 0x2E:
            self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x2E, list(self._nav_state["2E_DestinationInfo"]), atomic=True)
            return True
        if fn == 0x2F:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x2F, list(self._nav_state["2F_Altitude"]))
            return True
        if fn == 0x37:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x37, list(self._nav_state["37_ManeuverState"]))
            return True
        if fn == 0x3C:
            self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x3C, list(self._nav_state["3C_DistanceToDestinationExtended"]), atomic=True)
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
        if self._phase == self.PHASE_READY:
            if self._respond_to_active_poll(fn):
                return
        self._reply_known_hud_nav_function(fn, payload, "HUD_RX", opcode=opcode)

    def _send_display_config(self, lead):
        third = 0x01 if lead in (0x12, 0x16, 0x17) else 0x00
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x18, [lead, 0x00, third, 0x00, 0x00], atomic=True, force_long=True)

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

    def _start_route_demo(self):
        self._demo_route = True
        self._log("[NAV] Starting nav demo")
        self._send_hud_route_payloads()
        self._active_visual_keepalive_next_at = time.monotonic() + NAV_DATA_REFRESH_S

    def start_nav_demo(self):
        if not self.enabled:
            self._log("[NAV] Manual trigger ignored: ignition off")
            return False
        if self._phase != self.PHASE_READY:
            self._log("[NAV] Manual trigger armed; waiting for HUD ready (0x0E 74 19)")
            self._demo_route = True
            return True
        self._start_route_demo()
        return True

    def _distance_payload(self):
        dist_m = self._nav_state["12_DistanceToNextManeuver_m"]
        if not self._nav_state["12_DistanceToNextManeuver_visible"] or dist_m is None:
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
        val = self._nav_state.get("_21_val", 0x00)
        self._tx(HUD_S, bap_hb(0x11) + self._route_guidance_status_payload())
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x39, self._maneuver_state_payload(), atomic=True)
        self._tx(HUD_S, bap_hb(0x35) + self._fsg_setup_payload())
        self._tx(HUD_D, bap_hb(0x1D) + [0x00, 0x00, 0xFF])
        self._tx(HUD_D, [0x90, 0x05] + bap_hb(0x21) + [val, 0x02, 0x02, 0x03])
        self._tx(HUD_D, [0xD0, 0x00])
        self._tx_bap(HUD_D, BAP_OP_HEARTBEAT_STATUS, HUD_LSG, 0x32, self._feature_enable_payload(), atomic=True, force_long=True)
        # MIB3 periodic extras
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x27, list(self._nav_state["27_ActiveRgType"]))
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x31, list(self._nav_state["31_Exitview"]))
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x14, list(self._nav_state["14_TurnToInfo_street"]))
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x16, list(self._nav_state["16_TimeToDestination"]))
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x2E, list(self._nav_state["2E_DestinationInfo"]), atomic=True)
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x2D, list(self._nav_state["2D_MapScale"]))
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x2F, list(self._nav_state["2F_Altitude"]))
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x37, list(self._nav_state["37_ManeuverState"]))
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x3C, list(self._nav_state["3C_DistanceToDestinationExtended"]), atomic=True)

    def _send_hud_route_payloads(self):
        self._sync_nav_state()
        arrow = self._configured_nav_arrow()
        self._tx_bap(HUD_D, BAP_OP_STATUS, HUD_LSG, 0x13, self._current_position_info_payload(), atomic=True, force_long=True)
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x12, self._distance_payload(), atomic=True)
        self._send_maneuver_descriptor()
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x25, self._function_sync_payload(), atomic=True)
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x11, self._route_guidance_status_payload())
        self._tx_bap(HUD_S, BAP_OP_STATUS, HUD_LSG, 0x15, self._destination_distance_payload(), atomic=True)
        self._send_active_visual_keepalive()
        self._log(f"[ARROW] -> HUD {arrow['name']} dist={arrow['dist_m']}m")

