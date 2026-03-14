#!/usr/bin/env python
"""
nav_controller.py — Audi MLB CAN Navigation Controller

THREADING MODEL (fixes SerialTimeoutException):
  All CAN writes go through a single shared queue.Queue.
  One dedicated writer thread drains it with an inter-frame gap.
  No module ever calls bus.send() from its own thread.

Usage:  python nav_controller.py
Needs:  pip install python-can
        modules/ folder alongside this script
"""

import argparse
import can, threading, time, importlib.util, os, sys, queue

# ── Serial throughput (csscan_serial) ─────────────────────────────────────────
# At 115200 baud a CAN frame takes ~2–3ms → ~300–400 fps max. Queue backs up.
# Use baudrate=921600 (8×) for ~2400+ fps. IFG prevents adapter buffer overrun.
IFG_SECS = 0.0005   # 0.5ms min gap between sends (2000 fps max at higher baud)
SERIAL_BAUDRATE = 115200  # 115200 if adapter unstable; 921600 if supported

# ── CAN CONNECTION CONFIG ─────────────────────────────────────────────────────
# Set CAN_INTERFACE to one of the options below. Config is detected at startup.
#
# Option 1: Auto-detect (csscan_serial) — CS-Scan serial adapter
CAN_INTERFACE = "csscan_serial"
CAN_AVAILABLE_CONFIGS = can.detect_available_configs(CAN_INTERFACE)

# Inject baudrate for csscan_serial (921600 = 8× throughput; use 115200 if unstable)
for c in CAN_AVAILABLE_CONFIGS:
    if c.get("interface") == "csscan_serial" and "baudrate" not in c:
        c["baudrate"] = SERIAL_BAUDRATE
#
# Option 2: SocketCAN (Linux) — native kernel CAN, vcan0 for virtual
#   CAN_INTERFACE = "socketcan"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "socketcan", "channel": "vcan0"}]
#   # or real device: channel="can0"
#
# Option 3: PCAN (Windows/Linux) — PEAK PCAN-USB, PCAN-PCI, etc.
#   CAN_INTERFACE = "pcan"
#   CAN_AVAILABLE_CONFIGS = can.detect_available_configs("pcan")
#   # or manual: [{"interface": "pcan", "channel": "PCAN_USBBUS1"}]
#
# Option 4: Serial (generic) — CAN over serial, e.g. /dev/ttyUSB0 or COM3
#   CAN_INTERFACE = "serial"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "serial", "channel": "COM3", "bitrate": 500000}]
#
# Option 5: SLCAN / Lawicel CAN-USB — 115200 serial, 500 kbps CAN (see README Lawicel section)
#   CAN_INTERFACE = "slcan"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "slcan", "channel": "COM5", "bitrate": 500000, "tty_baudrate": 115200}]
#
# Option 5b: SLCAN auto-detect (any SLCAN adapter)
#   CAN_INTERFACE = "slcan"
#   CAN_AVAILABLE_CONFIGS = can.detect_available_configs("slcan")
#
# Option 5c: Custom Lawicel driver (use if slcan gives no traffic with SLGreen/Arduino)
#   CAN_INTERFACE = "lawicel"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "lawicel", "channel": "COM10", "bitrate": 500000, "tty_baudrate": 115200}]
#
# Option 6: Virtual — no hardware, for testing
#   CAN_INTERFACE = "virtual"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "virtual", "channel": None}]
#
# Option 7: gs_usb — candleLight, Geschwister Schneider USB-CAN
#   CAN_INTERFACE = "gs_usb"
#   CAN_AVAILABLE_CONFIGS = can.detect_available_configs("gs_usb")

import tkinter as tk
from tkinter import filedialog, font as tkfont, ttk
from datetime import datetime

from bap import HudBapSession, load_hud_bap_messages

try:
    from tksheet import Sheet
    TKSHEET_AVAILABLE = True
except ImportError:
    TKSHEET_AVAILABLE = False

# ── IGN CRC LUTs (0x3C0 Klemmen_Status_01) ───────────────────────────────────
# b2=0x23: Kl_S=1, Kl_15=1, Kl_Infotainment=1  (ignition ON)
IGN_CRC = [0x9B,0x2E,0xDE,0x6B,0x11,0xA4,0x54,0xE1,
           0xA0,0x15,0xE5,0x50,0x2A,0x9F,0x6F,0xDA]
# b2=0x00: all terminals off — CRC00[i] = CRC23[i] ^ 0xFD
# (verified vs log BZ=12,13,14 in mib2_5+mib3)
IGN_CRC_OFF = [0x66,0xD3,0x23,0x96,0xEC,0x59,0xA9,0x1C,
               0x5D,0xE8,0x18,0xAD,0xD7,0x62,0x92,0x27]
# b2=0x21: KL15 off / KL_S still on — transition frame — CRC21[i] = CRC23[i] ^ 0x1C
# (verified vs mib2_5 ctr=0x0B, 0000058 ctr=0x00/0x01/0x0F)
IGN_CRC_KL15 = [0x87,0x32,0xC2,0x77,0x0D,0xB8,0x48,0xFD,
                0xBC,0x09,0xF9,0x4C,0x36,0x83,0x73,0xC6]

# ── MFL button definitions (0x5BF MFL_Tasten_Kon_01) ─────────────────────────
MFL = {
    "MENU":    {"code":0x01, "event":0x01, "label":"MENU",    "icon":"≡"},
    "LEFT":    {"code":0x03, "event":0x01, "label":"LEFT",    "icon":"◀"},
    "RIGHT":   {"code":0x02, "event":0x01, "label":"RIGHT",   "icon":"▶"},
    "OK":      {"code":0x07, "event":0x01, "label":"OK",      "icon":"✓"},
    "RETURN":  {"code":0x08, "event":0x01, "label":"BACK",    "icon":"↩"},
    "VIEW":    {"code":0x23, "event":0x01, "label":"VIEW",    "icon":"⊞"},
    "WHEEL_U": {"code":0x06, "event":0x01, "label":"WHEEL ▲", "icon":"▲"},
    "WHEEL_D": {"code":0x06, "event":0x0F, "label":"WHEEL ▼", "icon":"▼"},
}
MFL_CLEAR = [0x00, 0x00, 0x00, 0x21]

MFL_LAYOUT = [
    ("MENU",    0, 1),
    ("WHEEL_U", 0, 2),
    ("LEFT",    1, 0),
    ("OK",      1, 1),
    ("RIGHT",   1, 2),
    ("VIEW",    2, 0),
    ("RETURN",  2, 1),
    ("WHEEL_D", 2, 2),
]

# ── Palette ───────────────────────────────────────────────────────────────────
C = {
    "bg":     "#0e1014", "panel":  "#15171c", "border": "#252830",
    "accent": "#d97706", "on":     "#22c55e", "off":    "#ef4444",
    "text":   "#e2e8f0", "sub":    "#64748b",
    "log_bg": "#0a0b0d", "log_fg": "#4ade80",
    "btn":    "#1c1f27", "btn_hov":"#252933", "btn_act":"#d97706",
    "ehdr":   "#191c23", "mrow":   "#111318", "mrow2":  "#13151c",
    "flash":  "#16a34a",
}

MODULES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modules")
# 0x3C0 byte 2: 0x23 = KL15/Infotainment on, 0x00/0x21 = off
KL15_ON_BYTE = 0x23

DEFAULT_APP_CONFIG = {
    "start_ignition": False,
    "send_ignition_updates": True,
    "verbose_bap": False,
    "hud_mode": "full",
    "hud_nav_enabled": False,
    "hud_distance_enabled": True,
    "hud_arrow": "straight",
    "hud_arrow_main": None,
    "hud_arrow_dir": None,
    "hud_distance_m": 500,
    "hud_distance_graph": 0x64,
    "hud_distance_unit": "m",
    "hud_street_name": "Offroad",
    "hud_lane_guidance_enabled": False,
    "hud_lane_num_lanes": 3,
    "hud_lane_recommended": 2,
    "auto_open_hud": False,
}

HUD_SOURCE_IDS = {
    0x17330400,  # HUD_BOOT
    0x17330410,  # HUD_RX
    0x17333200,  # HUD_GET
    0x17333202,  # HUD_ASG
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Audi MLB CAN Navigation Controller")
    parser.add_argument("--ignition", choices=("on", "off"), default="off",
                        help="Set the initial ignition state.")
    parser.add_argument("--no-send-ignition-updates", action="store_true",
                        help="Do not send periodic 0x3C0; monitor ignition from bus instead.")
    parser.add_argument("--verbose-bap", action="store_true",
                        help="Enable verbose 5F/HUD BAP logging.")
    parser.add_argument("--hud-mode", choices=("full", "minimal"), default="full",
                        help="Select full emulation or minimal straight-arrow HUD mode.")
    parser.add_argument("--auto-open-hud", action="store_true",
                        help="Open the HUD BAP monitor window on startup.")
    return parser.parse_args(argv)


def make_app_config(args):
    cfg = dict(DEFAULT_APP_CONFIG)
    cfg.update({
        "start_ignition": args.ignition == "on",
        "send_ignition_updates": not getattr(args, "no_send_ignition_updates", False),
        "verbose_bap": bool(args.verbose_bap),
        "hud_mode": args.hud_mode,
        "auto_open_hud": bool(args.auto_open_hud),
    })
    return cfg

# ─────────────────────────────────────────────────────────────────────────────
# BUS MANAGER
# ─────────────────────────────────────────────────────────────────────────────

class BusManager:
    def __init__(self, log_cb, status_cb, frame_cb=None, ign_from_bus_cb=None):
        self._log    = log_cb
        self._status = status_cb
        self._frame  = frame_cb or (lambda *_args, **_kwargs: None)
        self._ign_from_bus_cb = ign_from_bus_cb  # called when 0x3C0 RX updates ignition (from another device)
        self._bus    = None
        self._stop   = threading.Event()
        self.send_ignition_updates = True  # when False, do not send periodic 0x3C0 (another device may send it)

        # Shared send queues.
        # _prio_q: urgent frames (MFL button presses) — checked first by writer
        # _tx_q:   normal periodic ECU frames
        self._prio_q = queue.Queue()
        self._tx_q   = queue.Queue()  # unbounded
        self._bus_lock = threading.Lock()  # guards bus.send() — shared with ECUs for atomic MF sequences

        self._main_thread   = None   # connection thread
        self._writer_thread = None   # serial writer
        self._reader_thread = None   # shared RX dispatcher
        self._tick_thread   = None   # ignition heartbeat — runs from app start

        self._ecus     = []
        self._mfl_q    = []
        self._mfl_lk   = threading.Lock()

        self.ignition  = False
        self.connected = False
        self._ig_ctr   = 0
        self._ign_on = False
        self._tick     = 0

    # ── ECU management ────────────────────────────────────────────────────────

    def register_ecu(self, ecu):
        self._ecus.append(ecu)
        # Apply current ignition state before attaching so it never sends
        # frames in the wrong state — modules start enabled=False by default
        if hasattr(ecu, 'set_enabled'):
            ecu.set_enabled(self.ignition)
        else:
            ecu.enabled = self.ignition
        if self.connected and self._bus:
            ecu.attach(self._bus, self._tx_q, self._bus_lock, self._emit_frame)

    def get_ecus(self):
        return list(self._ecus)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        self._stop.clear()
        # Tick loop starts immediately — 0x3C0 sends from launch, bus or not
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True, name="IGNTick")
        self._tick_thread.start()
        self._main_thread = threading.Thread(target=self._run, daemon=True, name="BusMgr")
        self._main_thread.start()

    def stop(self):
        self._stop.set()
        for e in self._ecus:
            e.detach()
        if self._main_thread:
            self._main_thread.join(timeout=3)
        if self._writer_thread:
            self._writer_thread.join(timeout=2)
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        if self._tick_thread:
            self._tick_thread.join(timeout=2)

    def set_send_ignition_updates(self, on: bool):
        """When False, stop sending periodic 0x3C0; ignition state can still be updated from bus RX."""
        self.send_ignition_updates = bool(on)

    def set_ignition(self, on: bool):
        self.ignition = on
        self._ign_on = on
        if not on and self.connected:
            # Drain the queue so stale ECU frames don't delay the shutdown burst.
            try:
                while True:
                    self._tx_q.get_nowait()
            except queue.Empty:
                pass
            # Replicate exact log shutdown burst: one 0x21 frame immediately
            # followed by one 0x00 frame (~1ms apart as seen in mib2_5 log).
            ctr_21 = (self._ig_ctr + 1) & 0x0F
            ctr_00 = (self._ig_ctr + 2) & 0x0F
            self._tx_q.put((0x3C0, [IGN_CRC_KL15[ctr_21], ctr_21, 0x21, 0x00]))
            self._tx_q.put((0x3C0, [IGN_CRC_OFF[ctr_00],  ctr_00, 0x00, 0x00]))
            self._ig_ctr = ctr_00   # tick loop continues counting from here
        for e in self._ecus:
            if hasattr(e, 'set_enabled'):
                e.set_enabled(on)
            else:
                e.enabled = on

    def queue_mfl(self, key: str):
        with self._mfl_lk:
            self._mfl_q.append(key)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run(self):
        cfgs = CAN_AVAILABLE_CONFIGS
        if not cfgs:
            self._status("error", "No adapter found")
            return

        cfg = cfgs[0]
        try:
            bus_kw = {"interface": cfg["interface"]}
            if "channel" in cfg:
                bus_kw["channel"] = cfg["channel"]
            if "bitrate" in cfg:
                bus_kw["bitrate"] = cfg["bitrate"]
            # Higher serial baud = more fps; csscan_serial supports baudrate kw
            if cfg.get("interface") == "csscan_serial":
                bus_kw["baudrate"] = cfg.get("baudrate", SERIAL_BAUDRATE)
            # Lawicel CAN-USB / SLCAN: serial port baud (e.g. 115200), bitrate = CAN speed (e.g. 500000)
            if cfg.get("interface") == "slcan" and "tty_baudrate" in cfg:
                bus_kw["tty_baudrate"] = cfg["tty_baudrate"]
            if cfg.get("interface") == "lawicel":
                from lawicel_canusb import LawicelBusAdapter
                self._bus = LawicelBusAdapter(
                    channel=cfg["channel"],
                    bitrate=cfg.get("bitrate", 500000),
                    tty_baudrate=cfg.get("tty_baudrate", 115200),
                    serial_timeout=cfg.get("serial_timeout", 0.05),
                )
            else:
                self._bus = can.Bus(**bus_kw)
            self.connected = True
            self._status("ok", cfg["channel"])
            self._log(f"Connected: {cfg['interface']} / {cfg['channel']}")
        except Exception as e:
            self._status("error", str(e))
            self._log(f"Connection error: {e}")
            return

        # Start serial writer thread
        self._writer_thread = threading.Thread(
            target=self._writer_loop, daemon=True, name="CANWriter"
        )
        self._writer_thread.start()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="CANReader"
        )
        self._reader_thread.start()

        # Attach ECU modules — apply ignition state first, then start threads
        for e in self._ecus:
            if hasattr(e, 'set_enabled'):
                e.set_enabled(self.ignition)
            else:
                e.enabled = self.ignition
            e.attach(self._bus, self._tx_q, self._bus_lock, self._emit_frame)

        try:
            self._stop.wait()  # block until stop() — tick loop runs independently
        finally:
            for e in self._ecus:
                e.detach()
            self._bus.shutdown()
            self.connected = False
            self._status("off", "")

    def _emit_frame(self, direction: str, arb_id: int, data):
        try:
            self._frame(direction, arb_id, list(data))
        except Exception:
            pass

    def _dispatch_message(self, msg):
        for ecu in self._ecus:
            handler = getattr(ecu, "on_message", None)
            if not callable(handler):
                continue
            try:
                wants = getattr(ecu, "wants_message", None)
                if callable(wants) and not wants(msg):
                    continue
                handler(msg)
            except Exception as e:
                self._log(f"RX err [{getattr(ecu, 'ECU_ID', '?')}] {e}")

    def _reader_loop(self):
        while not self._stop.is_set():
            if not self._bus:
                time.sleep(0.05)
                continue
            try:
                msg = self._bus.recv(timeout=0.05)
            except Exception as e:
                self._log(f"RX bus err: {e}")
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            self._emit_frame("rx", msg.arbitration_id, list(msg.data))
            # Monitor 0x3C0 from bus so ignition state reflects actual bus (e.g. another device sending it)
            if msg.arbitration_id == 0x3C0 and len(msg.data) >= 3:
                on = (msg.data[2] == KL15_ON_BYTE)
                if on != self.ignition:
                    self.ignition = on
                    self._ign_on = on
                    for e in self._ecus:
                        if hasattr(e, "set_enabled"):
                            e.set_enabled(on)
                        else:
                            e.enabled = on
                    if self._ign_from_bus_cb:
                        try:
                            self._ign_from_bus_cb(on)
                        except Exception:
                            pass
            self._dispatch_message(msg)

    def _writer_loop(self):
        """
        Single thread that drains the TX queues and writes to the serial port.
        _prio_q is checked first so MFL button presses are never delayed by
        a backlog of periodic ECU frames.
        Enforces IFG_SECS between consecutive sends to prevent adapter overrun.
        """
        last_send = 0.0
        while not self._stop.is_set():
            # Priority queue first (non-blocking)
            try:
                arb_id, data = self._prio_q.get_nowait()
            except queue.Empty:
                # Fall back to normal queue (blocking with timeout)
                try:
                    arb_id, data = self._tx_q.get(timeout=0.05)
                except queue.Empty:
                    continue

            if self._bus:
                # Enforce inter-frame gap so serial buffer doesn't overrun
                now = time.monotonic()
                elapsed = now - last_send
                if elapsed < IFG_SECS:
                    time.sleep(IFG_SECS - elapsed)
                try:
                    with self._bus_lock:
                        self._bus.send(can.Message(
                            arbitration_id=arb_id,
                            data=data,
                            is_extended_id=arb_id > 0x7FF
                        ))
                    self._emit_frame("tx", arb_id, data)
                except can.CanError as e:
                    self._log(f"TX err {hex(arb_id)}: {e}")
                last_send = time.monotonic()



    def _tick_loop(self):
        """Ignition heartbeat + MFL dispatch — 100ms tick."""
        last = time.monotonic()
        while not self._stop.is_set():
            with self._mfl_lk:
                pending = list(self._mfl_q)
                self._mfl_q.clear()
            for key in pending:
                threading.Thread(
                    target=self._do_mfl, args=(key,), daemon=True
                ).start()

            now = time.monotonic()
            if now - last >= 0.100:
                last = now
                # 0x3C0 Klemmen_Status_01 — only when we are responsible for sending it
                if self.connected and self.send_ignition_updates:
                    self._ig_ctr = (self._ig_ctr + 1) & 0x0F
                    if self._ign_on:
                        crc, kl15 = IGN_CRC[self._ig_ctr], 0x23
                    else:
                        crc, kl15 = IGN_CRC_OFF[self._ig_ctr], 0x00
                    self._tx_q.put((0x3C0, [crc, self._ig_ctr, kl15, 0x00]))
                self._tick += 1

            time.sleep(0.005)

    def _do_mfl(self, key: str):
        btn = MFL.get(key)
        if not btn:
            return
        if key == "OFF":
            self._prio_q.put((0x5BF, MFL_CLEAR))
            self._log(f"MFL {btn['label']:8s} → {' '.join(f'{b:02X}' for b in MFL_CLEAR)}")
            return
        press = [btn["code"], 0x00, btn["event"], 0x21]
        self._prio_q.put((0x5BF, press))
        self._log(f"MFL {btn['label']:8s} → {' '.join(f'{b:02X}' for b in press)}")
        time.sleep(0.120)   # hold time before clear
        self._prio_q.put((0x5BF, MFL_CLEAR))


# ─────────────────────────────────────────────────────────────────────────────
# MODULE LOADER
# ─────────────────────────────────────────────────────────────────────────────

def load_modules(log_cb=None, config=None):
    ecus = []
    if not os.path.isdir(MODULES_DIR):
        return ecus
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from ecu_base import ECUModule
    for fname in sorted(os.listdir(MODULES_DIR)):
        if not fname.endswith(".py") or fname.startswith("_"):
            continue
        path = os.path.join(MODULES_DIR, fname)
        spec = importlib.util.spec_from_file_location(fname[:-3], path)
        mod  = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
            sys.modules[spec.name] = mod  # so VZE tab and others can resolve pack_vze_01 etc.
        except Exception as e:
            print(f"[WARN] {fname}: {e}")
            continue
        for attr in dir(mod):
            cls = getattr(mod, attr)
            try:
                if (isinstance(cls, type) and
                        issubclass(cls, ECUModule) and cls is not ECUModule):
                    try:
                        inst = cls(log_cb=log_cb, config=config)
                    except TypeError:
                        try:
                            inst = cls(log_cb=log_cb)
                        except TypeError:
                            inst = cls()
                    ecus.append(inst)
                    break
            except Exception:
                continue
    return ecus


# ─────────────────────────────────────────────────────────────────────────────
# WIDGETS
# ─────────────────────────────────────────────────────────────────────────────

class MFLBtn(tk.Frame):
    def __init__(self, parent, key, cmd=None):
        b = MFL[key]
        super().__init__(parent, bg=C["btn"], cursor="hand2",
                         highlightbackground=C["border"], highlightthickness=1)
        self._cmd = cmd; self._af = None; self._pressing = False
        fi = tkfont.Font(family="Segoe UI", size=14)
        fl = tkfont.Font(family="Segoe UI", size=7)
        self._il = tk.Label(self, text=b["icon"], font=fi, bg=C["btn"], fg=C["accent"])
        self._il.pack(padx=10, pady=(7, 1))
        self._nl = tk.Label(self, text=b["label"], font=fl, bg=C["btn"], fg=C["sub"])
        self._nl.pack(pady=(0, 7))

        # Forward child events to the Frame so all clicks are handled here,
        # and Enter/Leave crossing into a child label doesn't reset the colour.
        for child in (self._il, self._nl):
            child.bindtags((str(self),) + child.bindtags())

        # Bind only on the Frame — children now forward to us via bindtags above.
        self.bind("<Enter>",           lambda e: self._bg(C["btn_hov"]))
        self.bind("<Leave>",           self._on_leave)
        self.bind("<ButtonPress-1>",   lambda e: self._on_press())
        self.bind("<ButtonRelease-1>", lambda e: self._on_release())

    def _bg(self, c):
        self.configure(bg=c); self._il.configure(bg=c); self._nl.configure(bg=c)

    def _on_leave(self, e):
        # Ignore Leave if pointer moved onto a child — only reset on true exit
        if not self._pressing:
            self._bg(C["btn"])

    def _on_press(self):
        self._pressing = True
        self._bg(C["btn_act"])

    def _on_release(self):
        self._pressing = False
        self._bg(C["btn_hov"])
        if self._cmd:
            self._cmd()

    def flash(self):
        if self._af: self.after_cancel(self._af)
        self._bg(C["btn_act"])
        self._af = self.after(160, lambda: self._bg(C["btn"]))


def _hex(data):
    """Format up to 8 bytes as hex. Handles flat list or list-of-frames (shows current frame)."""
    if data and isinstance(data[0], list):
        data = data[0]   # multi-frame state: show first/current frame
    return " ".join(f"{b:02X}" for b in data[:8])


class ECUCard(tk.Frame):
    def __init__(self, parent, ecu):
        super().__init__(parent, bg=C["panel"],
                         highlightbackground=C["border"], highlightthickness=1)
        self._ecu = ecu; self._rows = {}
        fh = tkfont.Font(family="Consolas", size=9, weight="bold")
        fi = tkfont.Font(family="Consolas", size=8)
        fd = tkfont.Font(family="Consolas", size=8)
        fc = tkfont.Font(family="Consolas", size=7)

        hdr = tk.Frame(self, bg=C["ehdr"]); hdr.pack(fill="x")
        tk.Label(hdr, text=f" {ecu.ECU_ID}  {ecu.ECU_NAME}",
                 font=fh, bg=C["ehdr"], fg=C["text"], anchor="w"
                 ).pack(side="left", padx=8, pady=5)
        self._sdot = tk.Label(hdr, text="●", font=fi, bg=C["ehdr"], fg=C["sub"])
        self._sdot.pack(side="right", padx=8)
        self._enabled = True
        self._tog = tk.Label(hdr, text="ON ", font=fi, bg=C["ehdr"], fg=C["on"],
                             cursor="hand2")
        self._tog.pack(side="right", padx=(0, 4))
        self._tog.bind("<Button-1>", self._toggle)

        ch = tk.Frame(self, bg=C["panel"]); ch.pack(fill="x", padx=2, pady=(2, 0))
        for txt, w, side in [("MSG-ID",7,"left"),("NAME",17,"left"),
                              ("LAST DATA",22,"left"),("TX#",5,"right")]:
            tk.Label(ch, text=txt, font=fc, bg=C["panel"], fg=C["sub"],
                     width=w, anchor="w" if side=="left" else "e"
                     ).pack(side=side, padx=2)

        for i, s in enumerate(ecu.get_states()):
            bg = C["mrow"] if i % 2 == 0 else C["mrow2"]
            row = tk.Frame(self, bg=bg); row.pack(fill="x")
            tk.Label(row, text=f"0x{s.arb_id:X}", font=fi, bg=bg, fg=C["accent"],
                     width=7, anchor="w").pack(side="left", padx=(6,2), pady=2)
            tk.Label(row, text=s.name[:17], font=fi, bg=bg, fg=C["sub"],
                     width=17, anchor="w").pack(side="left")
            dl = tk.Label(row, text=_hex(s.current_display()), font=fd, bg=bg, fg=C["text"],
                          width=22, anchor="w")
            dl.pack(side="left", padx=3)
            cl = tk.Label(row, text="0", font=fc, bg=bg, fg=C["sub"], width=5, anchor="e")
            cl.pack(side="right", padx=6)
            self._rows[s.arb_id] = (dl, cl, bg)

    def refresh(self):
        active = False
        for s in self._ecu.get_states():
            if s.arb_id not in self._rows: continue
            dl, cl, bg = self._rows[s.arb_id]
            cl.configure(text=str(s.tx_count))
            dl.configure(text=_hex(s.current_display()))
            if s.tx_count and (time.monotonic() - s.last_tx) < 0.25:
                dl.configure(fg=C["on"]); active = True
            else:
                dl.configure(fg=C["text"])
        self._sdot.configure(fg=C["flash"] if active else C["sub"])

    def _toggle(self, _event=None):
        self._enabled = not self._enabled
        if self._enabled:
            self._tog.configure(text="ON ", fg=C["on"])
            self._ecu.set_enabled(True)
        else:
            self._tog.configure(text="OFF", fg=C["off"])
            self._ecu.set_enabled(False)


def _panel(parent, title):
    o = tk.Frame(parent, bg=C["panel"],
                 highlightbackground=C["border"], highlightthickness=1)
    tk.Label(o, text=title, font=tkfont.Font(family="Segoe UI", size=7),
             bg=C["panel"], fg=C["sub"]).pack(anchor="w", padx=10, pady=(7, 2))
    return o


def _message_to_row(message) -> list:
    return [
        message.direction.upper(),
        message.timestamp,
        message.can_id_label,
        message.opcode_text,
        message.lsg_text,
        message.fct_text,
        message.data_text,
        message.text,
    ]


class BapTableWindow(tk.Toplevel):
    COLS = ("direction", "timestamp", "canid", "opcode", "lsgid", "fctid", "data", "text")
    HEADERS = ["Direction", "Timestamp", "CAN ID", "Opcode", "LSG ID", "Function", "Data", "Text"]
    WIDTHS = [70, 110, 120, 140, 150, 220, 320, 360]

    def __init__(self, parent, title):
        super().__init__(parent)
        self.title(title)
        self.configure(bg=C["bg"])
        self.geometry("1320x620")
        self.minsize(960, 360)
        self._messages = []
        self._use_sheet = TKSHEET_AVAILABLE

        top = tk.Frame(self, bg=C["panel"])
        top.pack(fill="x", padx=8, pady=(8, 4))
        tk.Button(
            top,
            text="Copy Selected",
            font=tkfont.Font(family="Segoe UI", size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self.copy_selected_rows,
        ).pack(side="right", padx=10, pady=6)
        self._status = tk.Label(
            top,
            text="",
            font=tkfont.Font(family="Consolas", size=8),
            bg=C["panel"],
            fg=C["sub"],
            anchor="w",
        )
        self._status.pack(fill="x", padx=10, pady=8)

        body = tk.Frame(self, bg=C["bg"])
        body.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)

        if self._use_sheet:
            self._sheet = Sheet(
                body,
                headers=self.HEADERS,
                data=[],
                show_row_index=True,
                height=520,
                theme="dark",
                table_wrap="w",
                max_column_width=400,
            )
            self._sheet.enable_bindings(
                "single_select", "drag_select", "select_all",
                "copy", "arrowkeys", "row_select",
            )
            self._sheet.disable_bindings("edit_cell", "edit_header", "edit_index")
            self._sheet.grid(row=0, column=0, sticky="nsew")
            self.bind("<Control-c>", lambda _e: self.copy_selected_rows())
            self.bind("<Control-C>", lambda _e: self.copy_selected_rows())
            self._tree = None
        else:
            style = ttk.Style()
            style.configure("HudBap.Treeview", rowheight=72)
            self._tree = ttk.Treeview(body, columns=self.COLS, show="headings", style="HudBap.Treeview")
            self._tree.grid(row=0, column=0, sticky="nsew")
            self._tree.tag_configure("hud_source", background="#4a2f0b", foreground=C["text"])
            self._tree.bind("<Control-c>", lambda _e: self.copy_selected_rows())
            self._tree.bind("<Control-C>", lambda _e: self.copy_selected_rows())
            self.bind("<Control-c>", lambda _e: self.copy_selected_rows())
            self.bind("<Control-C>", lambda _e: self.copy_selected_rows())
            ysb = ttk.Scrollbar(body, orient="vertical", command=self._tree.yview)
            xsb = ttk.Scrollbar(body, orient="horizontal", command=self._tree.xview)
            ysb.grid(row=0, column=1, sticky="ns")
            xsb.grid(row=1, column=0, sticky="ew")
            self._tree.configure(yscrollcommand=ysb.set, xscrollcommand=xsb.set)
            for col, width in zip(self.COLS, self.WIDTHS):
                self._tree.heading(col, text=dict(zip(self.COLS, self.HEADERS))[col])
                self._tree.column(col, width=width, stretch=True)
            self._sheet = None

    def set_status(self, text: str):
        self._status.configure(text=text)

    def clear(self):
        self._messages = []
        if self._use_sheet:
            self._sheet.set_sheet_data([], reset_col_positions=False)
        else:
            for item in self._tree.get_children():
                self._tree.delete(item)

    def load_messages(self, messages):
        self.clear()
        self._messages = list(messages)
        if self._use_sheet:
            self._refresh_sheet()
        else:
            for message in messages:
                self._tree.insert(
                    "",
                    "end",
                    values=_message_to_row(message),
                    tags=("hud_source",) if message.can_id in HUD_SOURCE_IDS else (),
                )
        self.set_status(f"{len(messages)} decoded messages")

    def _refresh_sheet(self):
        if not self._use_sheet:
            return
        self._sheet.set_sheet_data([_message_to_row(m) for m in self._messages])
        self._sheet.set_all_cell_sizes_to_text()
        for r, m in enumerate(self._messages):
            if m.can_id in HUD_SOURCE_IDS:
                for c in range(len(self.HEADERS)):
                    self._sheet[(r, c)].bg = "#4a2f0b"
        self._sheet.refresh()

    def append_message(self, message):
        self._messages.append(message)
        row = _message_to_row(message)
        if self._use_sheet:
            self._refresh_sheet()
        else:
            self._tree.insert(
                "",
                "end",
                values=row,
                tags=("hud_source",) if message.can_id in HUD_SOURCE_IDS else (),
            )
        if not self._use_sheet:
            self._tree.yview_moveto(1.0)

    def append_messages(self, messages):
        if not messages:
            return
        self._messages.extend(messages)
        if self._use_sheet:
            self._refresh_sheet()
        else:
            for message in messages:
                self._tree.insert(
                    "",
                    "end",
                    values=_message_to_row(message),
                    tags=("hud_source",) if message.can_id in HUD_SOURCE_IDS else (),
                )
            self._tree.yview_moveto(1.0)

    def copy_selected_rows(self):
        if self._use_sheet:
            boxes = self._sheet.get_all_selection_boxes()
            if not boxes:
                self.set_status("No rows selected")
                return
            lines = []
            for (r1, c1, r2, c2) in boxes:
                data = self._sheet[(r1, c1):(r2, c2)].data
                for row in data:
                    lines.append("\t".join(str(v) for v in row))
            if not lines:
                self.set_status("No rows selected")
                return
            self.clipboard_clear()
            self.clipboard_append("\n".join(lines))
        else:
            items = self._tree.selection()
            if not items:
                self.set_status("No rows selected")
                return
            lines = []
            for item in items:
                values = self._tree.item(item, "values")
                lines.append("\t".join(str(v) for v in values))
            self.clipboard_clear()
            self.clipboard_append("\n".join(lines))
        noun = "row" if len(lines) == 1 else "rows"
        self.set_status(f"Copied {len(lines)} {noun} to clipboard")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN APP
# ─────────────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    REFRESH = 200
    HUD_FRAME_REFRESH = 20

    def __init__(self, config=None):
        super().__init__()
        self._cfg = dict(DEFAULT_APP_CONFIG)
        if config:
            self._cfg.update(config)
        self.title("CAN Nav Controller"); self.configure(bg=C["bg"])
        self.geometry("1560x920")
        self.minsize(1320, 820); self.resizable(True, True)
        self.protocol("WM_DELETE_WINDOW", self._close)
        self._ign = False; self._cards = []
        self._hud_logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "hud_bap")
        self._hud_session = HudBapSession(self._hud_logs_dir)
        self._hud_live_win = None
        self._hud_frame_q = queue.Queue()
        self._setup_bus(); self._build(); self._start_refresh(); self._start_hud_frame_drain()
        self.after(50, lambda: self._log(
            f"Startup config: ignition={'on' if self._cfg.get('start_ignition') else 'off'}"
            f" send_ignition_updates={'on' if self._cfg.get('send_ignition_updates') else 'off'}"
            f" hud_mode={self._cfg.get('hud_mode')}"
            f" verbose_bap={'on' if self._cfg.get('verbose_bap') else 'off'}"
        ))
        if self._cfg.get("auto_open_hud"):
            self.after(200, self._open_hud_bap_window)
        if self._cfg.get("start_ignition"):
            self.after(300, self._apply_initial_ignition)

    def _setup_bus(self):
        self._mgr = BusManager(
            log_cb=    lambda m: self.after(0, self._log, m),
            status_cb= lambda s, d: self.after(0, self._setstatus, s, d),
            frame_cb=  lambda d, a, data: self._hud_frame_q.put((d, a, list(data))),
            ign_from_bus_cb=lambda on: self.after(0, self._on_ignition_from_bus, on),
        )
        for ecu in load_modules(log_cb=lambda m: self.after(0, self._log, m), config=self._cfg):
            self._mgr.register_ecu(ecu)
        self._mgr.start()
        self._mgr.set_send_ignition_updates(bool(self._cfg.get("send_ignition_updates", True)))

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build(self):
        self.columnconfigure(0, minsize=360, weight=0)
        self.columnconfigure(2, weight=1)
        self.rowconfigure(2, weight=1)

        bar = tk.Frame(self, bg=C["panel"]); bar.grid(row=0,column=0,columnspan=3,sticky="ew")
        bar.columnconfigure(1, weight=1)
        tk.Label(bar, text="  ◈  CAN NAV CONTROLLER",
                 font=tkfont.Font(family="Segoe UI",size=11,weight="bold"),
                 bg=C["panel"], fg=C["accent"]).grid(row=0,column=0,sticky="w",pady=8)
        self._dot = tk.Label(bar, text="●",
                              font=tkfont.Font(family="Segoe UI",size=9),
                              bg=C["panel"], fg=C["off"])
        self._dot.grid(row=0, column=1, sticky="e", padx=4)
        self._slbl = tk.Label(bar, text="DISCONNECTED",
                               font=tkfont.Font(family="Consolas",size=8),
                               bg=C["panel"], fg=C["sub"])
        self._slbl.grid(row=0, column=2, sticky="e", padx=12)
        self._qlbl = tk.Label(bar, text="Q:0",
                               font=tkfont.Font(family="Consolas",size=8),
                               bg=C["panel"], fg=C["sub"])
        self._qlbl.grid(row=0, column=3, sticky="e", padx=12)

        tk.Frame(self, bg=C["border"], height=1).grid(row=1,column=0,columnspan=3,sticky="ew")

        left = tk.Frame(self, bg=C["bg"])
        left.grid(row=2,column=0,sticky="nsew",padx=(10,5),pady=10)
        # Ignition + MFL/stalk stay narrow; notebook (col 2) gets remaining width
        left.columnconfigure(0, weight=0, minsize=130)
        left.columnconfigure(1, weight=0, minsize=130)
        left.columnconfigure(2, weight=1)
        left.rowconfigure(0, weight=0)
        left.rowconfigure(1, weight=0)
        left.rowconfigure(2, weight=1, minsize=320)
        left.rowconfigure(3, weight=0)
        ign = self._build_ign(left)
        ign.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=(0, 3), pady=(0, 3))
        mfl = self._build_mfl(left)
        mfl.grid(row=1, column=0, sticky="nsew", padx=(0, 3), pady=3)
        stalk = self._build_stalk(left)
        stalk.grid(row=1, column=1, sticky="nsew", padx=3, pady=3)
        # Tabs in place of former nav panel (col 2): NAV HUD = nav controls + HUD BAP, VZE = traffic signs
        self._left_notebook = ttk.Notebook(left)
        self._left_notebook.grid(row=0, column=2, rowspan=3, sticky="nsew", padx=(3, 0), pady=(0, 6))
        tab_nav = tk.Frame(self._left_notebook, bg=C["bg"])
        tab_vze = tk.Frame(self._left_notebook, bg=C["bg"])
        self._left_notebook.add(tab_nav, text="NAV HUD")
        self._left_notebook.add(tab_vze, text="VZE")
        tab_nav.columnconfigure(0, weight=1)
        tab_nav.rowconfigure(1, weight=1)
        self._build_nav_controls(tab_nav).grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 2))
        self._build_hud_bap(tab_nav).grid(row=1, column=0, sticky="nsew", padx=4, pady=2)
        self._build_vze_tab(tab_vze).pack(fill="both", expand=True)
        self._build_log(left).grid(row=3, column=0, columnspan=3, sticky="nsew")

        tk.Frame(self, bg=C["border"], width=1).grid(row=2,column=1,sticky="ns")

        right = tk.Frame(self, bg=C["bg"])
        right.grid(row=2,column=2,sticky="nsew",padx=(5,10),pady=10)
        right.columnconfigure(0,weight=1); right.rowconfigure(1,weight=1)
        tk.Label(right, text="ATTACHED ECUs",
                 font=tkfont.Font(family="Segoe UI",size=8),
                 bg=C["bg"], fg=C["sub"]).grid(row=0,column=0,sticky="w",pady=(0,4))

        cvs = tk.Canvas(right, bg=C["bg"], highlightthickness=0)
        cvs.grid(row=1,column=0,sticky="nsew")
        sb = tk.Scrollbar(right, orient="vertical", command=cvs.yview,
                           bg=C["panel"], troughcolor=C["bg"], width=8)
        sb.grid(row=1,column=1,sticky="ns")
        cvs.configure(yscrollcommand=sb.set)
        self._ecu_frame = tk.Frame(cvs, bg=C["bg"])
        win = cvs.create_window((0,0), window=self._ecu_frame, anchor="nw")
        cvs.bind("<Configure>", lambda e: cvs.itemconfig(win, width=e.width))
        self._ecu_frame.bind("<Configure>", lambda e: cvs.configure(scrollregion=cvs.bbox("all")))
        cvs.bind_all("<MouseWheel>", lambda e: cvs.yview_scroll(int(-e.delta/120), "units"))

        self._build_ecu_cards()

    def _build_ign(self, parent):
        pnl = _panel(parent, "IGNITION")
        self._ign_btn = tk.Button(pnl, text="OFF",
            font=tkfont.Font(family="Segoe UI",size=14,weight="bold"),
            width=9, bg=C["btn"], fg=C["off"], activebackground=C["btn_hov"],
            activeforeground=C["text"], relief="flat", bd=0, cursor="hand2",
            command=self._toggle_ign)
        self._ign_btn.pack(padx=20, pady=(8,4))
        self._ign_hint = tk.Label(pnl, text="Click to enable ignition",
            font=tkfont.Font(family="Segoe UI",size=8), bg=C["panel"], fg=C["sub"])
        self._ign_hint.pack(pady=(0,2))
        self._send_ign_var = tk.BooleanVar(value=bool(self._cfg.get("send_ignition_updates", True)))
        def _on_send_ign_toggle():
            on = bool(self._send_ign_var.get())
            self._cfg["send_ignition_updates"] = on
            if hasattr(self, "_mgr") and self._mgr:
                self._mgr.set_send_ignition_updates(on)
        tk.Checkbutton(
            pnl, text="Send ignition updates (0x3C0)",
            variable=self._send_ign_var,
            command=_on_send_ign_toggle,
            bg=C["panel"], fg=C["text"], activebackground=C["panel"], activeforeground=C["text"],
            selectcolor=C["panel"], highlightthickness=0,
            font=tkfont.Font(family="Segoe UI", size=8),
        ).pack(pady=(0, 6))
        self._inds = {}
        for name, tag in [("0x3C0  Ignition","[CORE]"),("0x6B2  Diagnose","[19]"),
                           ("0x331  MFL hb","[5F]"),("NM     Keepalive","[5F]")]:
            r = tk.Frame(pnl, bg=C["panel"]); r.pack(fill="x", padx=8, pady=1)
            d = tk.Label(r, text="●", font=tkfont.Font(family="Consolas",size=8),
                         bg=C["panel"], fg=C["border"])
            d.pack(side="left")
            tk.Label(r, text=f" {name}", font=tkfont.Font(family="Consolas",size=8),
                     bg=C["panel"], fg=C["sub"]).pack(side="left")
            tk.Label(r, text=tag, font=tkfont.Font(family="Consolas",size=7),
                     bg=C["panel"], fg=C["border"]).pack(side="right")
            self._inds[name] = d
        tk.Frame(pnl, bg=C["panel"], height=6).pack()
        return pnl

    def _build_controls(self, parent):
        row = tk.Frame(parent, bg=C["bg"])
        row.pack(fill="x", pady=(0,6))
        row.columnconfigure(0, weight=1, uniform="ctrl")
        row.columnconfigure(1, weight=1, uniform="ctrl")
        row.columnconfigure(2, weight=1, uniform="ctrl")
        self._build_mfl(row).grid(row=0, column=0, sticky="nsew", padx=(0,3))
        self._build_stalk(row).grid(row=0, column=1, sticky="nsew", padx=3)
        self._build_nav_controls(row).grid(row=0, column=2, sticky="nsew", padx=(3,0))

    def _build_mfl(self, parent):
        pnl = _panel(parent, "MFL  (0x5BF)")
        g = tk.Frame(pnl, bg=C["panel"]); g.pack(fill="x", padx=12, pady=(8,12))
        for c in range(3): g.columnconfigure(c, weight=1, minsize=78)
        self._btns = {}
        for key, row, col in MFL_LAYOUT:
            b = MFLBtn(g, key, cmd=lambda k=key: self._mfl(k))
            b.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
            self._btns[key] = b
        return pnl

    def _build_stalk(self, parent):
        pnl = _panel(parent, "STALK / BLINKERS  (16)")
        BTNS = [
            ("blink_left",  "blink_left",  "<",  "LEFT"),
            ("blink_off",   "blink_off",   "X",  "OFF"),
            ("blink_right", "blink_right", ">",  "RIGHT"),
        ]
        g = tk.Frame(pnl, bg=C["panel"]); g.pack(padx=10, pady=(8,4), fill="x")
        for c in range(3): g.columnconfigure(c, weight=1, minsize=64)
        self._stalk_btns = {}
        fi = tkfont.Font(family="Segoe UI", size=11)
        fl = tkfont.Font(family="Segoe UI", size=6)
        for col, (key, cmd, icon, lbl) in enumerate(BTNS):
            frm = tk.Frame(g, bg=C["btn"], cursor="hand2",
                           highlightbackground=C["border"], highlightthickness=1)
            frm.grid(row=0, column=col, padx=2, pady=2, sticky="nsew")
            il = tk.Label(frm, text=icon, font=fi, bg=C["btn"], fg=C["accent"])
            il.pack(padx=6, pady=(5,1))
            nl = tk.Label(frm, text=lbl, font=fl, bg=C["btn"], fg=C["sub"])
            nl.pack(pady=(0,5))
            self._stalk_btns[key] = (frm, il, nl)
            for w in (frm, il, nl):
                w.bind("<ButtonPress-1>",   lambda e,b=(frm,il,nl):      self._stalk_press(b))
                w.bind("<ButtonRelease-1>", lambda e,b=(frm,il,nl),c=cmd: self._stalk_release(b,c))
                w.bind("<Enter>",  lambda e,b=(frm,il,nl): self._stalk_bg(b, C["btn_hov"]))
                w.bind("<Leave>",  lambda e,b=(frm,il,nl): self._stalk_bg(b, C["btn"]))
        self._stalk_enabled = False
        self._update_stalk_buttons()
        return pnl

    def _nav_settings(self):
        settings = {
            "enabled": bool(self._cfg.get("hud_nav_enabled", False)),
            "distance_enabled": bool(self._cfg.get("hud_distance_enabled", True)),
            "distance_m": max(0, int(self._cfg.get("hud_distance_m", 500))),
            "distance_graph": int(self._cfg.get("hud_distance_graph", 0x64)) & 0xFF,
            "distance_unit": self._cfg.get("hud_distance_unit", "m"),
            "street_name": self._cfg.get("hud_street_name", "Offroad"),
            "arrow_main": self._cfg.get("hud_arrow_main"),
            "arrow_dir": self._cfg.get("hud_arrow_dir"),
            "lane_guidance_enabled": bool(self._cfg.get("hud_lane_guidance_enabled", False)),
            "lane_num_lanes": max(2, min(8, int(self._cfg.get("hud_lane_num_lanes", 3)))),
            "lane_recommended": max(0, min(7, int(self._cfg.get("hud_lane_recommended", 2)))),
        }
        ecu = self._find_infotainment_ecu()
        if ecu is not None and hasattr(ecu, "get_nav_settings"):
            settings.update(ecu.get_nav_settings())
        return settings

    def _current_lanes_from_vars(self):
        """Build list of lane dicts from current _nav_lane_vars (for passing to _rebuild_lane_rows)."""
        lanes = []
        for v in self._nav_lane_vars:
            try:
                direction = int(v["direction"].get().strip() or "0", 16) & 0xFF
                lane_type = int(v["lane_type"].get().strip() or "1", 16) & 0xFF
                mark_l = int(v["mark_l"].get().strip() or "0") & 0x0F
                mark_r = int(v["mark_r"].get().strip() or "0") & 0x0F
                lane_desc = int(v["lane_desc"].get().strip() or "0", 16) & 0x0F
                guidance = 0x02 if v["preferred"].get() else 0x00
            except ValueError:
                direction, lane_type, mark_l, mark_r, lane_desc, guidance = 0, 1, 0, 0, 0, 0
            lanes.append({
                "direction": direction, "sidestreets_len": 0, "lane_type": lane_type,
                "marking_left": mark_l, "marking_right": mark_r,
                "lane_description": lane_desc, "guidance": guidance,
            })
        return lanes

    def _rebuild_lane_rows(self, initial_lanes=None):
        """Rebuild lane rows in _nav_lane_rows_frame from _nav_lane_num_var; optionally seed from initial_lanes."""
        for w in self._nav_lane_rows_frame.winfo_children():
            w.destroy()
        self._nav_lane_vars.clear()
        try:
            n = max(2, min(8, int(self._nav_lane_num_var.get().strip() or "3")))
        except ValueError:
            n = 3
        self._nav_lane_num_var.set(str(n))
        small = tkfont.Font(family="Segoe UI", size=7)
        entry_font = tkfont.Font(family="Consolas", size=8)
        # Header
        tk.Label(self._nav_lane_rows_frame, text="Preferred", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=0, padx=(0, 4), pady=(0, 2))
        tk.Label(self._nav_lane_rows_frame, text="Dir", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=1, padx=(0, 4))
        tk.Label(self._nav_lane_rows_frame, text="Type", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=2, padx=(0, 4))
        tk.Label(self._nav_lane_rows_frame, text="ML", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=3, padx=(0, 4))
        tk.Label(self._nav_lane_rows_frame, text="MR", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=4, padx=(0, 4))
        tk.Label(self._nav_lane_rows_frame, text="Desc", bg=C["panel"], fg=C["sub"], font=small).grid(row=0, column=5, padx=(0, 4))
        preferred_set = False
        if initial_lanes:
            for L in initial_lanes:
                if int(L.get("guidance", 0)) == 0x02:
                    preferred_set = True
                    break
        for i in range(n):
            if initial_lanes and i < len(initial_lanes):
                L = initial_lanes[i]
                direction = f"{int(L.get('direction', 0)) & 0xFF:02X}"
                lane_type = f"{int(L.get('lane_type', 1)) & 0xFF:02X}"
                mark_l = str(int(L.get("marking_left", 0)) & 0x0F)
                mark_r = str(int(L.get("marking_right", 0)) & 0x0F)
                lane_desc = f"{int(L.get('lane_description', 0)) & 0x0F:X}"
                preferred = int(L.get("guidance", 0)) == 0x02
                if preferred:
                    preferred_set = True
            else:
                direction = "00"
                lane_type = "01"
                mark_l = "0"
                mark_r = "0"
                lane_desc = "0"
                preferred = not preferred_set and (i == n - 1 or i == n // 2)
                if preferred:
                    preferred_set = True
            var_pref = tk.BooleanVar(value=preferred)
            var_dir = tk.StringVar(value=direction)
            var_type = tk.StringVar(value=lane_type)
            var_ml = tk.StringVar(value=mark_l)
            var_mr = tk.StringVar(value=mark_r)
            var_desc = tk.StringVar(value=lane_desc)
            row = i + 1
            tk.Checkbutton(
                self._nav_lane_rows_frame, variable=var_pref,
                bg=C["panel"], fg=C["text"], activebackground=C["panel"], activeforeground=C["text"],
                selectcolor=C["panel"], highlightthickness=0, font=small,
            ).grid(row=row, column=0, padx=(0, 4), pady=1)
            def _entry(parent, var, w=2):
                return tk.Entry(parent, textvariable=var, width=w, font=entry_font, bg=C["bg"], fg=C["text"], insertbackground=C["text"], relief="flat")
            _entry(self._nav_lane_rows_frame, var_dir, 2).grid(row=row, column=1, padx=(0, 4), pady=1)
            _entry(self._nav_lane_rows_frame, var_type, 2).grid(row=row, column=2, padx=(0, 4), pady=1)
            _entry(self._nav_lane_rows_frame, var_ml, 1).grid(row=row, column=3, padx=(0, 4), pady=1)
            _entry(self._nav_lane_rows_frame, var_mr, 1).grid(row=row, column=4, padx=(0, 4), pady=1)
            _entry(self._nav_lane_rows_frame, var_desc, 1).grid(row=row, column=5, padx=(0, 4), pady=1)
            self._nav_lane_vars.append({
                "direction": var_dir, "lane_type": var_type, "mark_l": var_ml, "mark_r": var_mr,
                "lane_desc": var_desc, "preferred": var_pref,
            })
        if not preferred_set and self._nav_lane_vars:
            self._nav_lane_vars[0]["preferred"].set(True)

    def _build_nav_controls(self, parent):
        pnl = _panel(parent, "NAV HUD  (5F)")
        settings = self._nav_settings()
        body = tk.Frame(pnl, bg=C["panel"])
        body.pack(fill="x", padx=10, pady=(8, 10))

        self._nav_enabled_var = tk.BooleanVar(value=bool(settings["enabled"]))
        self._nav_distance_enabled_var = tk.BooleanVar(value=bool(settings["distance_enabled"]))
        self._nav_lane_guidance_var = tk.BooleanVar(value=bool(settings["lane_guidance_enabled"]))
        self._nav_distance_var = tk.StringVar(value=str(settings["distance_m"]))
        self._nav_distance_graph_var = tk.IntVar(value=min(100, max(0, int(settings['distance_graph']) & 0xFF)))
        self._nav_distance_unit_var = tk.StringVar(value=str(settings.get("distance_unit", "m")))
        self._nav_street_var = tk.StringVar(value=str(settings["street_name"]))
        self._nav_arrow_main_var = tk.StringVar(value=f"{int(settings.get('arrow_main') or 0) & 0xFF:02X}")
        self._nav_arrow_dir_var = tk.StringVar(value=f"{int(settings.get('arrow_dir') or 0) & 0xFF:02X}")
        self._nav_lane_num_var = tk.StringVar(value=str(settings["lane_num_lanes"]))
        self._nav_lane_vars = []  # list of {direction, lane_type, mark_l, mark_r, lane_desc, preferred} per lane
        tk.Checkbutton(
            body,
            text="Navigation enabled",
            variable=self._nav_enabled_var,
            bg=C["panel"],
            fg=C["text"],
            activebackground=C["panel"],
            activeforeground=C["text"],
            selectcolor=C["panel"],
            highlightthickness=0,
            font=tkfont.Font(family="Segoe UI", size=8),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 2))
        tk.Checkbutton(
            body,
            text="Distance visible",
            variable=self._nav_distance_enabled_var,
            bg=C["panel"],
            fg=C["text"],
            activebackground=C["panel"],
            activeforeground=C["text"],
            selectcolor=C["panel"],
            highlightthickness=0,
            font=tkfont.Font(family="Segoe UI", size=8),
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 2))
        tk.Checkbutton(
            body,
            text="Lane guidance",
            variable=self._nav_lane_guidance_var,
            bg=C["panel"],
            fg=C["text"],
            activebackground=C["panel"],
            activeforeground=C["text"],
            selectcolor=C["panel"],
            highlightthickness=0,
            font=tkfont.Font(family="Segoe UI", size=8),
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(0, 2))
        lane_top = tk.Frame(body, bg=C["panel"])
        lane_top.grid(row=3, column=0, columnspan=5, sticky="w", pady=(0, 4))
        tk.Label(lane_top, text="Lanes:", font=tkfont.Font(family="Segoe UI", size=7),
                 bg=C["panel"], fg=C["sub"]).pack(side="left", padx=(0, 4))
        lane_num_label = tk.Label(lane_top, textvariable=self._nav_lane_num_var, width=2,
                 font=tkfont.Font(family="Consolas", size=9), bg=C["panel"], fg=C["text"])
        lane_num_label.pack(side="left", padx=(0, 4))
        def _lane_minus():
            try:
                n = max(2, min(8, int(self._nav_lane_num_var.get().strip() or "3")))
            except ValueError:
                n = 3
            if n <= 2:
                return
            self._nav_lane_num_var.set(str(n - 1))
            initial = self._current_lanes_from_vars()[: n - 1]
            self._rebuild_lane_rows(initial_lanes=initial)
        def _lane_plus():
            try:
                n = max(2, min(8, int(self._nav_lane_num_var.get().strip() or "3")))
            except ValueError:
                n = 3
            if n >= 8:
                return
            self._nav_lane_num_var.set(str(n + 1))
            initial = self._current_lanes_from_vars()
            initial.append({"direction": 0, "sidestreets_len": 0, "lane_type": 1, "marking_left": 0, "marking_right": 0, "lane_description": 0, "guidance": 0})
            self._rebuild_lane_rows(initial_lanes=initial)
        btn_style = dict(font=tkfont.Font(family="Segoe UI", size=9), width=2, relief="flat", bd=0, cursor="hand2",
                        bg=C["btn"], fg=C["text"], activebackground=C["btn_hov"], activeforeground=C["text"])
        tk.Button(lane_top, text="−", command=_lane_minus, **btn_style).pack(side="left", padx=(0, 2))
        tk.Button(lane_top, text="+", command=_lane_plus, **btn_style).pack(side="left", padx=(0, 8))
        self._nav_lane_rows_frame = tk.Frame(body, bg=C["panel"])
        self._nav_lane_rows_frame.grid(row=4, column=0, columnspan=5, sticky="ew", pady=(0, 6))
        body.columnconfigure(0, weight=1)
        initial_lanes = settings.get("lanes")
        if not initial_lanes:
            n = max(2, min(8, int(settings.get("lane_num_lanes", 3))))
            rec = max(0, min(n - 1, int(settings.get("lane_recommended", 0))))
            initial_lanes = [
                {"direction": 0, "sidestreets_len": 0, "lane_type": 1, "marking_left": 0, "marking_right": 0, "lane_description": 0, "guidance": 0x02 if i == rec else 0}
                for i in range(n)
            ]
        self._rebuild_lane_rows(initial_lanes=initial_lanes)

        labels = [
            ("Arrow main", self._nav_arrow_main_var),
            ("Arrow dir", self._nav_arrow_dir_var),
            ("Distance", self._nav_distance_var),
            ("Unit", self._nav_distance_unit_var),
        ]
        for col, (label, var) in enumerate(labels):
            tk.Label(body, text=label, font=tkfont.Font(family="Segoe UI", size=7),
                     bg=C["panel"], fg=C["sub"]).grid(row=5, column=col, sticky="w", padx=(0, 4))
            tk.Entry(
                body,
                textvariable=var,
                width=6 if label == "Unit" else 8,
                font=tkfont.Font(family="Consolas", size=8),
                bg=C["bg"],
                fg=C["text"],
                insertbackground=C["text"],
                relief="flat",
            ).grid(row=6, column=col, sticky="ew", padx=(0, 6), pady=(0, 6))
            body.columnconfigure(col, weight=1)
        tk.Label(body, text="Graph %", font=tkfont.Font(family="Segoe UI", size=7),
                 bg=C["panel"], fg=C["sub"]).grid(row=5, column=4, sticky="w", padx=(0, 4))
        graph_scale = tk.Scale(
            body,
            from_=0,
            to=100,
            variable=self._nav_distance_graph_var,
            orient="horizontal",
            length=120,
            showvalue=True,
            bg=C["panel"],
            fg=C["text"],
            troughcolor=C["bg"],
            activebackground=C["panel"],
            highlightthickness=0,
            font=tkfont.Font(family="Segoe UI", size=7),
        )
        graph_scale.grid(row=6, column=4, sticky="ew", padx=(0, 6), pady=(0, 6))
        body.columnconfigure(4, weight=1)

        tk.Label(body, text="Street name", font=tkfont.Font(family="Segoe UI", size=7),
                 bg=C["panel"], fg=C["sub"]).grid(row=7, column=0, columnspan=5, sticky="w")
        tk.Entry(
            body,
            textvariable=self._nav_street_var,
            font=tkfont.Font(family="Consolas", size=8),
            bg=C["bg"],
            fg=C["text"],
            insertbackground=C["text"],
            relief="flat",
        ).grid(row=8, column=0, columnspan=5, sticky="ew", pady=(0, 8))

        tk.Button(
            body,
            text="Apply Nav Settings",
            font=tkfont.Font(family="Segoe UI", size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self._apply_nav_settings,
        ).grid(row=9, column=0, columnspan=5, sticky="ew")
        return pnl

    def _stalk_bg(self, trio, colour):
        frm, il, nl = trio
        frm.configure(bg=colour); il.configure(bg=colour)
        if nl: nl.configure(bg=colour)

    def _stalk_press(self, trio):
        if self._stalk_enabled: self._stalk_bg(trio, C["btn_act"])

    def _stalk_release(self, trio, cmd):
        if not self._stalk_enabled: return
        self._stalk_bg(trio, C["btn_hov"])
        ecu = self._find_stalk_ecu()
        if ecu:
            ecu.send_command(cmd)
            self._log("Stalk -> " + cmd)
        else:
            self._log("Stalk: module 16 not attached")

    def _find_stalk_ecu(self):
        for e in self._mgr.get_ecus():
            if getattr(e, "ECU_ID", None) == "16": return e
        return None

    def _find_infotainment_ecu(self):
        for e in self._mgr.get_ecus():
            if getattr(e, "ECU_ID", None) == "5F":
                return e
        return None

    def _find_a5_ecu(self):
        for e in self._mgr.get_ecus():
            if getattr(e, "ECU_ID", None) == "A5":
                return e
        return None

    def _update_stalk_buttons(self):
        ecu = self._find_stalk_ecu()
        self._stalk_enabled = self._ign and (ecu is not None)
        for key, trio in self._stalk_btns.items():
            frm, il, nl = trio
            if self._stalk_enabled:
                frm.configure(cursor="hand2", highlightbackground=C["border"])
                il.configure(fg=C["accent"] if key != "blink_off" else C["sub"])
                if nl: nl.configure(fg=C["sub"])
            else:
                frm.configure(cursor="", highlightbackground=C["bg"])
                il.configure(fg=C["border"])
                if nl: nl.configure(fg=C["border"])
                self._stalk_bg(trio, C["btn"])

    def _build_log(self, parent):
        pnl = _panel(parent, "LOG")
        hdr = tk.Frame(pnl, bg=C["panel"]); hdr.pack(fill="x", padx=6, pady=(2,0))
        tk.Button(
            hdr,
            text="Copy",
            font=tkfont.Font(family="Segoe UI",size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self._copy_log,
        ).pack(side="right", pady=4)
        self._log_w = tk.Text(pnl, bg=C["log_bg"], fg=C["log_fg"],
            font=tkfont.Font(family="Consolas",size=8),
            relief="flat", bd=0, state="disabled", wrap="none", height=20)
        self._log_w.pack(fill="both", expand=True, padx=2, pady=2)
        return pnl

    def _build_hud_bap(self, parent):
        pnl = _panel(parent, "HUD BAP")
        row = tk.Frame(pnl, bg=C["panel"]); row.pack(fill="x", padx=8, pady=(4, 4))
        tk.Button(
            row,
            text="Open Live Window",
            font=tkfont.Font(family="Segoe UI", size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self._open_hud_bap_window,
        ).pack(side="left", padx=(0, 6), pady=4)
        tk.Button(
            row,
            text="Start Nav",
            font=tkfont.Font(family="Segoe UI", size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self._start_nav_demo,
        ).pack(side="left", padx=(0, 6), pady=4)
        tk.Button(
            row,
            text="Load Log",
            font=tkfont.Font(family="Segoe UI", size=8),
            bg=C["btn"],
            fg=C["text"],
            activebackground=C["btn_hov"],
            activeforeground=C["text"],
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self._load_hud_bap_log,
        ).pack(side="left", pady=4)
        self._hud_log_lbl = tk.Label(
            pnl,
            text=f"Session log: {os.path.basename(str(self._hud_session.path))}",
            font=tkfont.Font(family="Consolas", size=7),
            bg=C["panel"],
            fg=C["sub"],
            justify="left",
            anchor="w",
            wraplength=260,
        )
        self._hud_log_lbl.pack(fill="x", padx=10, pady=(0, 8))
        return pnl

    def _build_vze_tab(self, parent):
        """VZE tab: A5 loaded check + sign type + first sign (decimal) for testing."""
        pnl = _panel(parent, "VZE  (0x181)")
        small = tkfont.Font(family="Segoe UI", size=8)
        entry_font = tkfont.Font(family="Consolas", size=9)
        row = tk.Frame(pnl, bg=C["panel"])
        row.pack(fill="x", padx=8, pady=(4, 4))
        self._vze_a5_status_lbl = tk.Label(
            row, text="Module A5: checking…",
            font=small, bg=C["panel"], fg=C["sub"])
        self._vze_a5_status_lbl.pack(side="left")
        self.after(100, self._update_vze_a5_status)

        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "pack_vze_01", None):
            tk.Label(pnl, text="VZE pack/unpack not available (a5_drvassist).",
                     font=small, bg=C["panel"], fg=C["sub"]).pack(pady=8)
            return pnl

        body = tk.Frame(pnl, bg=C["panel"])
        body.pack(fill="x", padx=8, pady=(4, 8))
        body.columnconfigure(1, weight=1)

        # Sign type: 0=EU_RDW, 1=USA, 2=Canada, 3=China (DBC value table)
        tk.Label(body, text="Sign type", font=small, bg=C["panel"], fg=C["sub"]).grid(row=0, column=0, sticky="w", padx=(0, 8))
        self._vze_anzeigemodus_var = tk.StringVar(value="0")
        sign_type_combo = ttk.Combobox(
            body, textvariable=self._vze_anzeigemodus_var,
            values=["EU_RDW (0)", "USA (1)", "Canada (2)", "China (3)"],
            width=14, state="readonly", font=entry_font
        )
        sign_type_combo.grid(row=0, column=1, sticky="w", pady=2)
        sign_type_combo.current(0)
        self._vze_sign_type_combo = sign_type_combo

        tk.Label(body, text="First sign (km/h, decimal)", font=small, bg=C["panel"], fg=C["sub"]).grid(row=1, column=0, sticky="w", padx=(0, 8))
        self._vze_vz1_var = tk.StringVar(value="90")
        vze_entry = tk.Entry(body, textvariable=self._vze_vz1_var, width=6, font=entry_font, bg=C["bg"], fg=C["text"], insertbackground=C["text"])
        vze_entry.grid(row=1, column=1, sticky="w", pady=2)
        vze_entry.bind("<FocusOut>", lambda e: self._apply_vze_settings())
        vze_entry.bind("<Return>", lambda e: self._apply_vze_settings())
        sign_type_combo.bind("<<ComboboxSelected>>", lambda e: self._apply_vze_settings())

        # Bytes 4, 5, 6, 7: one checkbox per bit (0=LSB .. 7=MSB). Build vars first so First sign row can reference them.
        self._vze_byte_vars = []  # [ [b4_0..b4_7], [b5_0..b5_7], [b6_0..b6_7], [b7_0..b7_7] ]
        bit_frame = tk.Frame(pnl, bg=C["panel"])
        for byteno in range(4):
            row_vars = []
            for bit in range(8):
                var = tk.BooleanVar(value=False)
                var.trace_add("write", lambda *_: self._apply_vze_settings())
                row_vars.append(var)
                cb = tk.Checkbutton(
                    bit_frame, variable=var, text=str(bit),
                    bg=C["panel"], fg=C["text"], activebackground=C["panel"], activeforeground=C["text"],
                    selectcolor=C["panel"], highlightthickness=0, font=small,
                )
                cb.grid(row=byteno, column=bit + 1, padx=1, pady=1)
            self._vze_byte_vars.append(row_vars)
        for row_lbl, row_no in [("Byte 4", 0), ("Byte 5", 1), ("Byte 6", 2), ("Byte 7", 3)]:
            tk.Label(bit_frame, text=row_lbl, font=small, bg=C["panel"], fg=C["sub"], width=6, anchor="w").grid(row=row_no, column=0, sticky="w", padx=(0, 4))
        # First sign related (under speed): B4.3=blink, B4.6=weather plain, B4.7=weather rain, B5.0=warning clock
        first_sign_frame = tk.Frame(pnl, bg=C["panel"])
        first_sign_frame.pack(fill="x", padx=8, pady=(2, 4))
        tk.Label(first_sign_frame, text="First sign:", font=small, bg=C["panel"], fg=C["sub"]).pack(side="left", padx=(0, 8))
        for lbl, var in [
            ("Sign warning (blink)", self._vze_byte_vars[0][3]),
            ("Weather plain", self._vze_byte_vars[0][6]),
            ("Weather rain", self._vze_byte_vars[0][7]),
            ("Warning clock", self._vze_byte_vars[1][0]),
        ]:
            cb = tk.Checkbutton(
                first_sign_frame, variable=var, text=lbl,
                bg=C["panel"], fg=C["text"], activebackground=C["panel"], activeforeground=C["text"],
                selectcolor=C["panel"], highlightthickness=0, font=small,
            )
            cb.pack(side="left", padx=(0, 12))
        bit_frame.pack(fill="x", padx=8, pady=(2, 4))

        btn_row = tk.Frame(pnl, bg=C["panel"])
        btn_row.pack(pady=(4, 8))
        tk.Button(
            btn_row, text="Apply VZE (0x181)",
            font=tkfont.Font(family="Segoe UI", size=9), bg=C["btn"], fg=C["text"],
            activebackground=C["btn_hov"], activeforeground=C["text"], relief="flat", bd=0, cursor="hand2",
            command=self._apply_vze_settings,
        ).pack(side="left", padx=(0, 6))
        tk.Button(
            btn_row, text="Load from A5",
            font=tkfont.Font(family="Segoe UI", size=8), bg=C["btn"], fg=C["text"],
            activebackground=C["btn_hov"], activeforeground=C["text"], relief="flat", bd=0, cursor="hand2",
            command=self._load_vze_from_a5,
        ).pack(side="left")
        tk.Frame(pnl, bg=C["panel"], height=4).pack()
        self._vze_bytes_lbl = tk.Label(
            pnl,
            text="0x181 payload: -- -- -- -- -- -- -- --",
            font=tkfont.Font(family="Consolas", size=9),
            bg=C["panel"],
            fg=C["sub"],
            justify="left",
        )
        self._vze_bytes_lbl.pack(fill="x", padx=8, pady=(0, 8))
        self.after(50, self._update_vze_bytes_display)
        return pnl

    def _update_vze_a5_status(self):
        ecu = self._find_a5_ecu()
        if hasattr(self, "_vze_a5_status_lbl") and self._vze_a5_status_lbl.winfo_exists():
            if ecu is not None:
                self._vze_a5_status_lbl.configure(text="Module A5: loaded", fg=C["on"])
            else:
                self._vze_a5_status_lbl.configure(text="Module A5: not loaded", fg=C["off"])

    def _update_vze_bytes_display(self):
        """Show the current form values packed as 0x181 payload (hex bytes) at the bottom."""
        if not hasattr(self, "_vze_bytes_lbl") or not self._vze_bytes_lbl.winfo_exists():
            return
        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "pack_vze_01", None):
            return
        try:
            idx = self._vze_sign_type_combo.current() if hasattr(self, "_vze_sign_type_combo") and self._vze_sign_type_combo.winfo_exists() else 0
            anzeigemodus = max(0, min(3, idx))
            vz1 = max(0, min(255, int(self._vze_vz1_var.get().strip() or "0")))
            def bits_to_byte(vars_row):
                return sum(2**i for i in range(8) if vars_row[i].get()) if vars_row else 0
            bv = self._vze_byte_vars if getattr(self, "_vze_byte_vars", None) and len(getattr(self, "_vze_byte_vars", [])) >= 4 else []
            b4 = bits_to_byte(bv[0]) if len(bv) > 0 else 0
            b5 = bits_to_byte(bv[1]) if len(bv) > 1 else 0
            b6 = bits_to_byte(bv[2]) if len(bv) > 2 else 0
            b7 = bits_to_byte(bv[3]) if len(bv) > 3 else 0
        except (ValueError, AttributeError):
            self._vze_bytes_lbl.configure(text="0x181 payload: (invalid values)")
            return
        payload = a5_mod.pack_vze_01(anzeigemodus=anzeigemodus, verkehrszeichen_1=vz1, byte_4=b4, byte_5=b5, byte_6=b6, byte_7=b7)
        hex_str = " ".join(f"{b:02X}" for b in payload[:8])
        self._vze_bytes_lbl.configure(text=f"0x181 payload: {hex_str}")

    def _load_vze_from_a5(self):
        ecu = self._find_a5_ecu()
        if ecu is None:
            self._log("Load VZE: module A5 not loaded")
            return
        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "unpack_vze_01", None):
            self._log("Load VZE: unpack_vze_01 not available")
            return
        for s in ecu.get_states():
            if s.arb_id == 0x181:
                params = a5_mod.unpack_vze_01(s.data)
                break
        else:
            self._log("Load VZE: no 0x181 state in A5")
            return
        if not hasattr(self, "_vze_sign_type_combo"):
            return
        self._vze_loading = True
        try:
            self._vze_sign_type_combo.current(max(0, min(3, params.get("anzeigemodus", 0))))
            self._vze_vz1_var.set(str(max(0, min(255, params.get("verkehrszeichen_1", 0)))))
            # Bits we don't read on load (First sign group): B4.3, B4.6, B4.7, B5.0 — leave checkboxes unchanged
            skip_bits = {(0, 3), (0, 6), (0, 7), (1, 0)}
            for byteno, key in enumerate(("byte_4", "byte_5", "byte_6", "byte_7")):
                b = params.get(key, 0) & 0xFF
                if getattr(self, "_vze_byte_vars", None) and len(self._vze_byte_vars) > byteno:
                    for i in range(8):
                        if (byteno, i) in skip_bits:
                            continue
                        self._vze_byte_vars[byteno][i].set(bool((b >> i) & 1))
        finally:
            self._vze_loading = False
        self._update_vze_bytes_display()
        self._apply_vze_settings()
        self._log("Load VZE: form filled from current 0x181")

    def _apply_vze_settings(self):
        if getattr(self, "_vze_loading", False):
            return
        ecu = self._find_a5_ecu()
        if ecu is None:
            return
        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "pack_vze_01", None):
            self._log("Apply VZE: pack_vze_01 not available")
            return
        try:
            idx = self._vze_sign_type_combo.current() if hasattr(self, "_vze_sign_type_combo") and self._vze_sign_type_combo.winfo_exists() else 0
            anzeigemodus = max(0, min(3, idx))
            vz1 = max(0, min(255, int(self._vze_vz1_var.get().strip() or "0")))
            def bits_to_byte(vars_row):
                return sum(2**i for i in range(8) if vars_row[i].get()) if vars_row else 0
            bv = self._vze_byte_vars if getattr(self, "_vze_byte_vars", None) and len(self._vze_byte_vars) >= 4 else []
            b4 = bits_to_byte(bv[0]) if len(bv) > 0 else 0
            b5 = bits_to_byte(bv[1]) if len(bv) > 1 else 0
            b6 = bits_to_byte(bv[2]) if len(bv) > 2 else 0
            b7 = bits_to_byte(bv[3]) if len(bv) > 3 else 0
        except ValueError as e:
            self._log(f"Apply VZE: invalid number — {e}")
            return
        payload = a5_mod.pack_vze_01(anzeigemodus=anzeigemodus, verkehrszeichen_1=vz1, byte_4=b4, byte_5=b5, byte_6=b6, byte_7=b7)
        ecu.update_data(0x181, payload)
        self._update_vze_bytes_display()
        self._log(f"Apply VZE: 0x181 type={anzeigemodus} speed={vz1} km/h")

    def _build_ecu_cards(self):
        for ecu in self._mgr.get_ecus():
            card = ECUCard(self._ecu_frame, ecu)
            card.pack(fill="x", pady=(0,8)); self._cards.append(card)
        if not self._cards:
            tk.Label(self._ecu_frame, text="No ECU modules found in modules/",
                     font=tkfont.Font(family="Segoe UI",size=9),
                     bg=C["bg"], fg=C["sub"]).pack(pady=20)

    # ── Actions ───────────────────────────────────────────────────────────────

    def _on_ignition_from_bus(self, on: bool):
        """Update UI from 0x3C0 received on bus (another device sending ignition). Manager already updated ECUs."""
        if self._ign == on:
            return
        self._ign = on
        if on:
            self._ign_btn.configure(text="ON ", fg=C["on"])
            self._ign_hint.configure(text="KL15 active (from bus)")
            for d in self._inds.values(): d.configure(fg=C["accent"])
            self._log("Ignition ON (from bus 0x3C0)")
        else:
            self._ign_btn.configure(text="OFF", fg=C["off"])
            self._ign_hint.configure(text="Click to enable ignition")
            for d in self._inds.values(): d.configure(fg=C["border"])
            self._log("Ignition OFF (from bus 0x3C0)")
        self._update_stalk_buttons()

    def _toggle_ign(self):
        self._ign = not self._ign; self._mgr.set_ignition(self._ign)
        if self._ign:
            self._ign_btn.configure(text="ON ", fg=C["on"])
            self._ign_hint.configure(text="KL15 active — click to disable")
            for d in self._inds.values(): d.configure(fg=C["accent"])
            self._log("Ignition ON")
        else:
            self._ign_btn.configure(text="OFF", fg=C["off"])
            self._ign_hint.configure(text="Click to enable ignition")
            for d in self._inds.values(): d.configure(fg=C["border"])
            self._log("Ignition OFF")
        self._update_stalk_buttons()

    def _apply_initial_ignition(self):
        if not self._ign:
            self._toggle_ign()
            self._log("Startup config: ignition=on")

    def _start_nav_demo(self):
        ecu = self._find_infotainment_ecu()
        if ecu is None:
            self._log("Start Nav: infotainment module 5F not attached")
            return
        if hasattr(ecu, "start_nav_demo") and ecu.start_nav_demo():
            self._log("Start Nav: manual route trigger sent")
        else:
            self._log("Start Nav: trigger ignored")

    def _parse_hex_byte(self, raw, label):
        text = str(raw).strip()
        if text.lower().startswith("0x"):
            text = text[2:]
        value = int(text, 16)
        if not 0 <= value <= 0xFF:
            raise ValueError(f"{label} must be between 00 and FF")
        return value

    def _apply_nav_settings(self):
        ecu = self._find_infotainment_ecu()
        if ecu is None:
            self._log("Apply Nav: infotainment module 5F not attached")
            return
        try:
            distance_m = max(0, int(self._nav_distance_var.get().strip() or "0"))
            distance_graph = max(0, min(0x64, int(self._nav_distance_graph_var.get())))
            distance_unit = (self._nav_distance_unit_var.get() or "m").strip().lower() or "m"
            if distance_unit not in ("m", "km"):
                distance_unit = "m"
            arrow_main = self._parse_hex_byte(self._nav_arrow_main_var.get(), "Arrow main")
            arrow_dir = self._parse_hex_byte(self._nav_arrow_dir_var.get(), "Arrow dir")
        except ValueError as exc:
            self._log(f"Apply Nav: {exc}")
            return

        street_name = (self._nav_street_var.get() or "").strip() or "Offroad"
        enabled = bool(self._nav_enabled_var.get())
        distance_enabled = bool(self._nav_distance_enabled_var.get())
        lane_guidance_enabled = bool(self._nav_lane_guidance_var.get())

        lanes = []
        preferred_index = None
        for i, v in enumerate(self._nav_lane_vars):
            try:
                direction = int(v["direction"].get().strip() or "0", 16) & 0xFF
                lane_type = int(v["lane_type"].get().strip() or "1", 16) & 0xFF
                mark_l = int(v["mark_l"].get().strip() or "0") & 0x0F
                mark_r = int(v["mark_r"].get().strip() or "0") & 0x0F
                lane_desc = int(v["lane_desc"].get().strip() or "0", 16) & 0x0F
                preferred = bool(v["preferred"].get())
            except ValueError:
                direction, lane_type, mark_l, mark_r, lane_desc = 0, 1, 0, 0, 0
                preferred = i == 0
            guidance = 0x02 if preferred else 0x00
            if preferred:
                preferred_index = i
            lanes.append({
                "direction": direction, "sidestreets_len": 0, "lane_type": lane_type,
                "marking_left": mark_l, "marking_right": mark_r,
                "lane_description": lane_desc, "guidance": guidance,
            })
        if preferred_index is None and lanes:
            lanes[0]["guidance"] = 0x02

        self._cfg["hud_nav_enabled"] = enabled
        self._cfg["hud_distance_enabled"] = distance_enabled
        self._cfg["hud_distance_m"] = distance_m
        self._cfg["hud_distance_graph"] = distance_graph
        self._cfg["hud_distance_unit"] = distance_unit
        self._cfg["hud_street_name"] = street_name
        self._cfg["hud_arrow_main"] = arrow_main
        self._cfg["hud_arrow_dir"] = arrow_dir
        self._cfg["hud_lane_guidance_enabled"] = lane_guidance_enabled
        self._cfg["hud_lanes"] = lanes
        self._cfg["hud_lane_num_lanes"] = len(lanes)
        self._cfg["hud_lane_recommended"] = preferred_index if preferred_index is not None else 0
        if hasattr(ecu, "configure_nav"):
            ecu.configure_nav(
                enabled=enabled,
                distance_enabled=distance_enabled,
                distance_m=distance_m,
                distance_graph=distance_graph,
                distance_unit=distance_unit,
                street_name=street_name,
                arrow_main=arrow_main,
                arrow_dir=arrow_dir,
                lane_guidance_enabled=lane_guidance_enabled,
                lanes=lanes,
            )
            self._log(
                f"Apply Nav: enabled={'yes' if enabled else 'no'}"
                f" distance_visible={'yes' if distance_enabled else 'no'}"
                f" lane_guidance={'yes' if lane_guidance_enabled else 'no'}"
                f" main=0x{arrow_main:02X}"
                f" dir=0x{arrow_dir:02X}"
                f" distance={distance_m}m"
                f" graph=0x{distance_graph:02X}"
                f" street={street_name}"
            )
        else:
            self._log("Apply Nav: infotainment module does not support live nav updates")

    def _mfl(self, key: str):
        self._btns[key].flash()                # always give visual feedback
        if not self._mgr.connected:
            self._log(f"MFL {MFL[key]['label']} (not connected — no TX)")
            return
        self._mgr.queue_mfl(key)

    def _open_hud_bap_window(self):
        if self._hud_live_win is not None and self._hud_live_win.winfo_exists():
            self._hud_live_win.focus_force()
            return
        self._hud_live_win = BapTableWindow(self, "HUD Bap")
        self._hud_live_win.load_messages(self._hud_session.messages)
        self._hud_live_win.set_status(f"Live session log: {self._hud_session.path}")

    def _load_hud_bap_log(self):
        path = filedialog.askopenfilename(
            title="Load HUD BAP Log",
            initialdir=self._hud_logs_dir,
            filetypes=[("Log files", "*.log"), ("All files", "*.*")],
        )
        if not path:
            return
        win = BapTableWindow(self, f"HUD Bap Log - {os.path.basename(path)}")
        win.set_status(f"Loading {path} ...")

        def _worker():
            try:
                messages = load_hud_bap_messages(path)
                error = None
            except Exception as exc:
                messages = []
                error = str(exc)
            self.after(0, self._finish_load_hud_bap_log, win, path, messages, error)

        threading.Thread(target=_worker, daemon=True, name="HudBapLogLoad").start()

    def _finish_load_hud_bap_log(self, win, path, messages, error):
        if error:
            win.set_status(f"Failed to load {path}: {error}")
            return
        win.load_messages(messages)
        win.set_status(f"{len(messages)} decoded messages from {path}")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _on_bus_frame(self, direction: str, arb_id: int, data: list):
        message = self._hud_session.handle_frame(direction, arb_id, data)
        if message is None:
            return
        if self._hud_live_win is not None and self._hud_live_win.winfo_exists():
            self._hud_live_win.append_message(message)

    def _drain_hud_frames(self, batch_limit=250):
        messages = []
        drained = 0
        while drained < batch_limit:
            try:
                direction, arb_id, data = self._hud_frame_q.get_nowait()
            except queue.Empty:
                break
            message = self._hud_session.handle_frame(direction, arb_id, data)
            if message is not None:
                messages.append(message)
            drained += 1
        if self._hud_live_win is not None and self._hud_live_win.winfo_exists():
            if messages:
                self._hud_live_win.append_messages(messages)
            self._hud_live_win.set_status(
                f"Live session log: {self._hud_session.path}"
                f"  pending={self._hud_frame_q.qsize()}"
            )

    def _copy_log(self):
        try:
            text = self._log_w.get("1.0", "end-1c")
            self.clipboard_clear()
            self.clipboard_append(text)
            self._log("Log copied to clipboard")
        except Exception as exc:
            self._log(f"Copy log failed: {exc}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        _, bottom = self._log_w.yview()
        stick_to_bottom = bottom >= 0.999
        self._log_w.configure(state="normal")
        self._log_w.insert("end", f"[{ts}] {msg}\n")
        if int(self._log_w.index("end-1c").split(".")[0]) > 600:
            self._log_w.delete("1.0", "80.0")
        if stick_to_bottom:
            self._log_w.see("end")
        self._log_w.configure(state="disabled")

    def _setstatus(self, state: str, detail: str):
        col = {"ok":C["on"],"error":C["off"],"off":C["sub"]}.get(state, C["sub"])
        lbl = {"ok":f"CONNECTED  {detail}","error":f"ERROR  {detail}",
               "off":"DISCONNECTED"}.get(state, state)
        self._dot.configure(fg=col); self._slbl.configure(text=lbl, fg=col)

    def _start_refresh(self):
        def _r():
            for card in self._cards:
                try: card.refresh()
                except Exception: pass
            try:
                q  = self._mgr._tx_q.qsize()
                pq = self._mgr._prio_q.qsize()
                txt = f"Q:{q}" if pq == 0 else f"Q:{q}  P:{pq}"
                self._qlbl.configure(text=txt,
                                     fg=C["off"] if q > 50 else C["sub"])
            except Exception:
                pass
            self.after(self.REFRESH, _r)
        self.after(self.REFRESH, _r)

    def _start_hud_frame_drain(self):
        def _r():
            self._drain_hud_frames(batch_limit=1000)
            self.after(self.HUD_FRAME_REFRESH, _r)
        self.after(self.HUD_FRAME_REFRESH, _r)

    def _close(self):
        self._mgr.stop()
        self._hud_session.close()
        self.destroy()


if __name__ == "__main__":
    args = parse_args()
    App(config=make_app_config(args)).mainloop()
