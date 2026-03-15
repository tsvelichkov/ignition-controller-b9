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

try:
    from serial.serialutil import SerialException
except ImportError:
    SerialException = type("SerialException", (Exception,), {})

# ── Serial throughput (csscan_serial) ─────────────────────────────────────────
# At 115200 baud a CAN frame takes ~2–3ms → ~300–400 fps max. Queue backs up.
# Use baudrate=921600 (8×) for ~4000+ fps. IFG prevents adapter buffer overrun.
IFG_SECS = 0.00025  # 0.25ms min gap between sends (~4000 fps max at higher baud)
SERIAL_BAUDRATE = 921600  # 8× throughput; use 115200 if adapter unstable

# ── CAN CONNECTION CONFIG ─────────────────────────────────────────────────────
# Set CAN_INTERFACE to one of the options below. Config is detected at startup.
#
# Multi-interface auto-detect: try csscan_serial, slcan, lawicel (COM ports), virtual
# Set CSS_SCAN_CHANNEL to force a specific port (e.g. "COM8"); None = auto-detect
CSS_SCAN_CHANNEL = "COM8"

def _detect_can_configs():
    configs = []
    seen = set()
    # 1. CSS Electronics (csscan_serial)
    css_configs = can.detect_available_configs("csscan_serial")
    if CSS_SCAN_CHANNEL and not any(str(c.get("channel")) == str(CSS_SCAN_CHANNEL) for c in css_configs):
        css_configs.insert(0, {"interface": "csscan_serial", "channel": CSS_SCAN_CHANNEL, "baudrate": SERIAL_BAUDRATE})
    for c in css_configs:
        if c.get("interface") == "csscan_serial" and "baudrate" not in c:
            c["baudrate"] = SERIAL_BAUDRATE
        key = (c.get("interface"), str(c.get("channel", "")))
        if key not in seen:
            seen.add(key)
            configs.append(c)
    # 2. SLCAN (python-can built-in)
    for c in can.detect_available_configs("slcan"):
        c.setdefault("bitrate", 500000)
        c.setdefault("tty_baudrate", 115200)
        key = ("slcan", str(c.get("channel", "")))
        if key not in seen:
            seen.add(key)
            configs.append(c)
    # 3. Lawicel/SLCAN on serial ports (COM3+, /dev/ttyUSB*, /dev/ttyACM*)
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            ch = port.device
            if ch not in [str(c.get("channel")) for c in configs]:
                cfg = {"interface": "lawicel", "channel": ch, "bitrate": 500000, "tty_baudrate": 115200}
                configs.append(cfg)
    except Exception:
        pass
    # 4. Virtual (always available for testing)
    configs.append({"interface": "virtual", "channel": None})
    return configs

CAN_AVAILABLE_CONFIGS = _detect_can_configs()
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

from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QFrame, QLabel, QPushButton, QCheckBox, QLineEdit, QPlainTextEdit, QSlider,
    QTabWidget, QScrollArea, QTableWidget, QTableWidgetItem, QHeaderView,
    QFileDialog, QComboBox, QSizePolicy, QSplitter, QGroupBox, QSpinBox,
    QAbstractItemView,
)
from PyQt6.QtCore import Qt, QTimer, QSize, pyqtSignal, QPoint
from PyQt6.QtGui import QFont, QColor, QPalette, QIcon, QAction, QTextCursor, QCursor

from bap import HudBapSession, load_hud_bap_messages, HUD_TRACE_IDS

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

# ── Palette (dark theme) ─────────────────────────────────────────────────────
C = {
    "bg":     "#1e293b", "panel":  "#334155", "border": "#475569",
    "accent": "#f59e0b", "on":     "#22c55e", "off":    "#ef4444",
    "text":   "#f1f5f9", "sub":    "#94a3b8",
    "log_bg": "#0f172a", "log_fg": "#94a3b8",
    "btn":    "#475569", "btn_hov":"#64748b", "btn_act":"#f59e0b",
    "ehdr":   "#334155", "mrow":   "#334155", "mrow2":  "#475569",
    "flash":  "#22c55e", "hud_source": "#fef3c7",
}
INPUT_MIN_H = 28  # min height for QLineEdit, QSpinBox, QComboBox in tabs
ICONS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")

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

HUD_SENDER_IDS = {0x17333210, 0x17333211}  # HUD_S, HUD_D — frames sent from 5F to HUD


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
    def __init__(self, log_cb, status_cb, frame_cb=None, ign_from_bus_cb=None, configs=None):
        self._log    = log_cb
        self._status = status_cb
        self._frame  = frame_cb or (lambda *_args, **_kwargs: None)
        self._ign_from_bus_cb = ign_from_bus_cb  # called when 0x3C0 RX updates ignition (from another device)
        self._bus    = None
        self._stop   = threading.Event()
        self._configs = list(configs) if configs else list(CAN_AVAILABLE_CONFIGS)
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

    def set_configs(self, configs):
        """Set CAN configs for next connection attempt (used by Reconnect)."""
        self._configs = list(configs)

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
        cfgs = self._configs
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
        Batches up to 8 frames per iteration to reduce loop overhead when backlogged.
        """
        last_send = 0.0
        batch = []
        batch_size = 8
        while not self._stop.is_set():
            # Priority queue first (non-blocking)
            try:
                arb_id, data = self._prio_q.get_nowait()
                batch.append((arb_id, data))
            except queue.Empty:
                # Drain normal queue into batch (non-blocking up to batch_size)
                for _ in range(batch_size - len(batch)):
                    try:
                        batch.append(self._tx_q.get_nowait())
                    except queue.Empty:
                        break
                if not batch:
                    try:
                        arb_id, data = self._tx_q.get(timeout=0.05)
                        batch.append((arb_id, data))
                    except queue.Empty:
                        continue

            if self._bus and batch:
                for arb_id, data in batch:
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
                    except (can.CanError, SerialException) as e:
                        self._log(f"TX err {hex(arb_id)}: {e}")
                    last_send = time.monotonic()
                batch.clear()



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
# WIDGETS (PyQt6)
# ─────────────────────────────────────────────────────────────────────────────

def _load_icon(name: str) -> QIcon:
    path = os.path.join(ICONS_DIR, f"{name}.svg")
    if os.path.exists(path):
        return QIcon(path)
    return QIcon()


class MFLBtn(QPushButton):
    def __init__(self, parent, key, cmd=None):
        b = MFL[key]
        super().__init__(parent)
        self._cmd = cmd
        self._key = key
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFlat(True)
        self.setCheckable(False)
        self.setFixedHeight(56)
        self.setStyleSheet(f"""
            QPushButton {{
                background: {C["btn"]};
                border: 1px solid {C["border"]};
                border-radius: 6px;
                color: {C["accent"]};
                font-size: 18px;
            }}
            QPushButton:hover {{ background: {C["btn_hov"]}; }}
            QPushButton:pressed {{ background: {C["btn_act"]}; }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)
        icon_lbl = QLabel(b["icon"])
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setStyleSheet(f"color: {C['accent']}; font-size: 16px; background: transparent; border: none;")
        layout.addWidget(icon_lbl)
        lbl = QLabel(b["label"])
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"color: {C['sub']}; font-size: 10px; background: transparent; border: none;")
        layout.addWidget(lbl)
        self._icon_lbl = icon_lbl
        self._lbl = lbl
        self.clicked.connect(self._on_click)

    def _on_click(self):
        if self._cmd:
            self._cmd()

    def flash(self):
        self.setStyleSheet(f"QPushButton {{ background: {C['btn_act']}; border: 1px solid {C['border']}; border-radius: 6px; }}")
        QTimer.singleShot(160, lambda: self.setStyleSheet(f"""
            QPushButton {{ background: {C["btn"]}; border: 1px solid {C["border"]}; border-radius: 6px; }}
            QPushButton:hover {{ background: {C["btn_hov"]}; }}
        """))


def _hex(data):
    """Format up to 8 bytes as hex. Handles flat list or list-of-frames (shows current frame)."""
    if data and isinstance(data[0], list):
        data = data[0]   # multi-frame state: show first/current frame
    return " ".join(f"{b:02X}" for b in data[:8])


class ECUCard(QFrame):
    def __init__(self, parent, ecu):
        super().__init__(parent)
        self._ecu = ecu
        self._rows = {}
        self._enabled = True
        self.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        self.setStyleSheet(f"ECUCard {{ background: {C['panel']}; border: 1px solid {C['border']}; border-radius: 6px; }}")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)

        hdr = QWidget()
        hdr_layout = QHBoxLayout(hdr)
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        title = QLabel(f" {ecu.ECU_ID}  {ecu.ECU_NAME}")
        title.setStyleSheet(f"font-weight: bold; color: {C['text']};")
        hdr_layout.addWidget(title)
        hdr_layout.addStretch()
        self._sdot = QLabel("●")
        self._sdot.setStyleSheet(f"color: {C['sub']};")
        hdr_layout.addWidget(self._sdot)
        self._tog = QPushButton("ON ")
        self._tog.setFlat(True)
        self._tog.setStyleSheet(f"color: {C['on']}; border: none; background: transparent; min-width: 0;")
        self._tog.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tog.clicked.connect(self._toggle)
        hdr_layout.addWidget(self._tog)
        layout.addWidget(hdr)

        grid = QGridLayout()
        grid.addWidget(QLabel("MSG-ID"), 0, 0)
        grid.addWidget(QLabel("NAME"), 0, 1)
        grid.addWidget(QLabel("LAST DATA"), 0, 2)
        grid.addWidget(QLabel("TX#"), 0, 3)
        for col, lbl in enumerate(["MSG-ID", "NAME", "LAST DATA", "TX#"]):
            w = grid.itemAtPosition(0, col).widget()
            w.setStyleSheet(f"color: {C['sub']}; font-size: 11px;")
        for i, s in enumerate(ecu.get_states()):
            row = i + 1
            grid.addWidget(QLabel(f"0x{s.arb_id:X}"), row, 0)
            grid.addWidget(QLabel(s.name[:17]), row, 1)
            dl = QLabel(_hex(s.current_display()))
            cl = QLabel("0")
            grid.addWidget(dl, row, 2)
            grid.addWidget(cl, row, 3)
            self._rows[s.arb_id] = (dl, cl)
        layout.addLayout(grid)

    def refresh(self):
        active = False
        for s in self._ecu.get_states():
            if s.arb_id not in self._rows:
                continue
            dl, cl = self._rows[s.arb_id]
            cl.setText(str(s.tx_count))
            dl.setText(_hex(s.current_display()))
            if s.tx_count and (time.monotonic() - s.last_tx) < 0.25:
                dl.setStyleSheet(f"color: {C['on']};")
                active = True
            else:
                dl.setStyleSheet(f"color: {C['text']};")
        self._sdot.setStyleSheet(f"color: {C['flash'] if active else C['sub']};")

    def _toggle(self):
        self._enabled = not self._enabled
        if self._enabled:
            self._tog.setText("ON ")
            self._tog.setStyleSheet(f"color: {C['on']};")
            self._ecu.set_enabled(True)
        else:
            self._tog.setText("OFF")
            self._tog.setStyleSheet(f"color: {C['off']};")
            self._ecu.set_enabled(False)


def _panel(parent, title: str) -> QGroupBox:
    g = QGroupBox(title)
    g.setStyleSheet(f"""
        QGroupBox {{
            font-size: 11px; color: {C['sub']}; background-color: {C['bg']}; border: 1px solid {C['border']};
            border-radius: 6px; margin-top: 8px; padding-top: 8px;
        }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; background-color: {C['bg']}; }}
        QGroupBox QLabel {{ color: {C['text']}; }}
    """)
    g.setAutoFillBackground(True)
    return g


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


class _CellDetailPopup(QFrame):
    """Floating panel showing full cell content on hover."""

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Tool | Qt.WindowType.FramelessWindowHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, False)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(4)
        self._header = QLabel()
        self._header.setStyleSheet(f"color: {C['accent']}; font-weight: bold; font-size: 11px;")
        layout.addWidget(self._header)
        self._content = QLabel()
        self._content.setWordWrap(True)
        self._content.setStyleSheet(f"color: {C['text']}; font-family: Consolas; font-size: 12px; max-width: 480px;")
        self._content.setMaximumWidth(480)
        layout.addWidget(self._content)
        self.setStyleSheet(f"""
            QFrame {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 6px;
            }}
        """)
        self.hide()

    def show_at(self, col_name: str, text: str, global_pos: QPoint):
        if not text:
            self.hide()
            return
        self._header.setText(col_name)
        self._content.setText(text)
        self.adjustSize()
        # Position below cursor, offset so it doesn't cover the cell
        x, y = global_pos.x() + 12, global_pos.y() + 16
        screen = QApplication.primaryScreen().availableGeometry()
        if x + self.width() > screen.right():
            x = global_pos.x() - self.width() - 8
        if y + self.height() > screen.bottom():
            y = global_pos.y() - self.height() - 8
        if x < screen.left():
            x = screen.left()
        if y < screen.top():
            y = screen.top()
        self.move(x, y)
        self.show()


class BapTableWindow(QWidget):
    HEADERS = ["Direction", "Timestamp", "CAN ID", "Opcode", "LSG ID", "Function", "Data", "Text"]
    WIDTHS = [70, 110, 120, 140, 150, 220, 320, 360]

    def __init__(self, parent, title: str):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle(title)
        self.setMinimumSize(960, 360)
        self.resize(1320, 620)
        self._messages = []
        self.setStyleSheet(f"background: {C['bg']};")

        layout = QVBoxLayout(self)
        top = QWidget()
        top.setStyleSheet(f"background: {C['bg']};")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8, 8, 8, 4)
        copy_btn = QPushButton("Copy Selected")
        copy_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 6px 12px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        copy_btn.clicked.connect(self.copy_selected_rows)
        top_layout.addStretch()
        top_layout.addWidget(copy_btn)
        self._status = QLabel("")
        self._status.setStyleSheet(f"color: {C['sub']}; font-family: Consolas;")
        top_layout.insertWidget(0, self._status, 1)
        layout.addWidget(top)

        self._table = QTableWidget()
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setTextElideMode(Qt.TextElideMode.ElideNone)
        self._table.setWordWrap(True)
        self._table.setColumnCount(len(self.HEADERS))
        self._table.setHorizontalHeaderLabels(self.HEADERS)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for i, w in enumerate(self.WIDTHS):
            self._table.setColumnWidth(i, w)
        self._table.setAlternatingRowColors(True)
        pal = self._table.palette()
        pal.setColor(QPalette.ColorRole.Base, QColor(C['bg']))
        pal.setColor(QPalette.ColorRole.AlternateBase, QColor(C['bg']))
        self._table.setPalette(pal)
        self._table.setStyleSheet(f"""
            QTableWidget {{ background: {C['bg']}; gridline-color: {C['border']}; }}
            QTableWidget::item {{ padding: 4px; color: {C['text']}; }}
            QHeaderView::section {{ background: {C['bg']}; color: {C['text']}; padding: 6px; border: none; }}
        """)
        self._auto_scroll = True
        vsb = self._table.verticalScrollBar()
        vsb.valueChanged.connect(self._on_scroll_changed)
        layout.addWidget(self._table)

        self._cell_popup = _CellDetailPopup(self)
        self._table.cellClicked.connect(self._on_cell_clicked)
        copy_action = QAction(self)
        copy_action.setShortcut("Ctrl+C")
        copy_action.triggered.connect(self.copy_selected_rows)
        self.addAction(copy_action)

    def _on_scroll_changed(self, value: int):
        vsb = self._table.verticalScrollBar()
        self._auto_scroll = (value >= vsb.maximum() - 2)

    def _on_cell_clicked(self, row: int, col: int):
        item = self._table.item(row, col)
        text = item.text() if item else ""
        if not text:
            self._cell_popup.hide()
            return
        col_name = self.HEADERS[col] if col < len(self.HEADERS) else ""
        self._cell_popup.show_at(col_name, text, QCursor.pos())

    def set_status(self, text: str):
        self._status.setText(text)

    def clear(self):
        self._messages = []
        self._table.setRowCount(0)

    def load_messages(self, messages):
        self.clear()
        self._auto_scroll = True
        msgs = list(messages)
        self._messages = msgs
        if not msgs:
            self.set_status("No messages")
            return
        self._table.setRowCount(len(msgs))
        for row, message in enumerate(msgs):
            row_data = _message_to_row(message)
            for col, val in enumerate(row_data):
                s = str(val)
                item = QTableWidgetItem(s)
                if message.can_id in HUD_SENDER_IDS:
                    item.setBackground(QColor("#7f1d1d"))
                    item.setForeground(QColor("#fecaca"))
                self._table.setItem(row, col, item)
        if self._auto_scroll:
            self._table.scrollToBottom()
        self.set_status(f"{len(msgs)} decoded messages")

    def _append_row(self, message):
        row_data = _message_to_row(message)
        row = self._table.rowCount()
        self._table.insertRow(row)
        for col, val in enumerate(row_data):
            s = str(val)
            item = QTableWidgetItem(s)
            if message.can_id in HUD_SENDER_IDS:
                item.setBackground(QColor("#7f1d1d"))
                item.setForeground(QColor("#fecaca"))
            self._table.setItem(row, col, item)
        if self._auto_scroll:
            self._table.scrollToBottom()

    def append_message(self, message):
        self._messages.append(message)
        self._append_row(message)

    def append_messages(self, messages):
        if not messages:
            return
        self._messages.extend(messages)
        for message in messages:
            self._append_row(message)
        if self._auto_scroll:
            self._table.scrollToBottom()

    def copy_selected_rows(self):
        items = self._table.selectedItems()
        if not items:
            self.set_status("No rows selected")
            return
        rows = sorted(set(item.row() for item in items))
        lines = []
        for r in rows:
            row_vals = []
            for c in range(self._table.columnCount()):
                it = self._table.item(r, c)
                row_vals.append(it.text() if it else "")
            lines.append("\t".join(row_vals))
        QApplication.clipboard().setText("\n".join(lines))
        noun = "row" if len(lines) == 1 else "rows"
        self.set_status(f"Copied {len(lines)} {noun} to clipboard")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN APP
# ─────────────────────────────────────────────────────────────────────────────

class App(QMainWindow):
    REFRESH = 250
    HUD_FRAME_REFRESH = 25
    status_updated = pyqtSignal(str, str)  # thread-safe: (state, detail)

    def __init__(self, config=None):
        super().__init__()
        self._cfg = dict(DEFAULT_APP_CONFIG)
        if config:
            self._cfg.update(config)
        self.setWindowTitle("CAN Nav Controller")
        self.setMinimumSize(1320, 950)
        self.resize(1560, 1080)
        self.setStyleSheet(f"background: {C['bg']};")
        self._ign = False
        self._cards = []
        self._hud_logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "hud_bap")
        self._hud_session = HudBapSession(self._hud_logs_dir)
        self._hud_live_win = None
        self._hud_frame_q = queue.Queue()
        self.status_updated.connect(self._setstatus)
        self._setup_bus()   # Create manager (needed by _build for stalk/ECU lookups)
        self._build()
        self._mgr.start()   # Start after UI exists so status callbacks can update labels
        self._start_refresh()
        self._start_hud_frame_drain()
        QTimer.singleShot(50, lambda: self._log(
            f"Startup config: ignition={'on' if self._cfg.get('start_ignition') else 'off'}"
            f" send_ignition_updates={'on' if self._cfg.get('send_ignition_updates') else 'off'}"
            f" hud_mode={self._cfg.get('hud_mode')}"
            f" verbose_bap={'on' if self._cfg.get('verbose_bap') else 'off'}"
        ))
        if self._cfg.get("auto_open_hud"):
            QTimer.singleShot(200, self._open_hud_bap_window)
        if self._cfg.get("start_ignition"):
            QTimer.singleShot(300, self._apply_initial_ignition)

    def _schedule(self, fn, *args):
        QTimer.singleShot(0, lambda: fn(*args))

    def _setup_bus(self):
        self._mgr = BusManager(
            log_cb=    lambda m: self._schedule(self._log, m),
            status_cb= lambda s, d: self.status_updated.emit(s, d),  # Qt signal = thread-safe
            frame_cb=  lambda d, a, data: self._hud_frame_q.put((d, a, list(data))) if a in HUD_TRACE_IDS else None,
            ign_from_bus_cb=lambda on: self._schedule(self._on_ignition_from_bus, on),
            configs=CAN_AVAILABLE_CONFIGS,
        )
        for ecu in load_modules(log_cb=lambda m: self._schedule(self._log, m), config=self._cfg):
            self._mgr.register_ecu(ecu)
        self._mgr.set_send_ignition_updates(bool(self._cfg.get("send_ignition_updates", True)))

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        main = QVBoxLayout(central)
        main.setSpacing(0)
        main.setContentsMargins(0, 0, 0, 0)

        bar = QFrame()
        bar.setStyleSheet(f"background: {C['panel']}; border-bottom: 1px solid {C['border']};")
        bar.setFixedHeight(44)
        bar_layout = QHBoxLayout(bar)
        bar_layout.setContentsMargins(12, 8, 12, 8)
        title = QLabel("  CAN NAV CONTROLLER")
        title.setStyleSheet(f"font-weight: bold; font-size: 13px; color: {C['accent']};")
        bar_layout.addWidget(title)
        bar_layout.addStretch()
        self._can_combo = QComboBox()
        self._can_combo.setMinimumWidth(180)
        self._can_combo.setStyleSheet(f"""
            QComboBox {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 4px 8px; }}
        """)
        for cfg in CAN_AVAILABLE_CONFIGS:
            iface = cfg.get("interface", "?")
            ch = cfg.get("channel")
            ch_str = ch if ch else "(no hardware)"
            self._can_combo.addItem(f"{iface} / {ch_str}", cfg)
        reconnect_btn = QPushButton("Reconnect")
        reconnect_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 4px 10px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        reconnect_btn.clicked.connect(self._reconnect_can)
        bar_layout.addWidget(QLabel("Interface:"))
        bar_layout.addWidget(self._can_combo)
        bar_layout.addWidget(reconnect_btn)
        self._dot = QLabel("●")
        self._dot.setStyleSheet(f"color: {C['off']};")
        bar_layout.addWidget(self._dot)
        self._slbl = QLabel("DISCONNECTED")
        self._slbl.setStyleSheet(f"color: {C['sub']}; font-family: Consolas; font-size: 11px;")
        bar_layout.addWidget(self._slbl)
        self._qlbl = QLabel("Q:0")
        self._qlbl.setStyleSheet(f"color: {C['sub']}; font-family: Consolas; font-size: 11px;")
        bar_layout.addWidget(self._qlbl)

        content = QWidget()
        content_layout = QHBoxLayout(content)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)

        left = QWidget()
        left.setMinimumWidth(360)
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(6)
        ign = self._build_ign(left)
        left_layout.addWidget(ign)
        ctrl_row = QWidget()
        ctrl_layout = QHBoxLayout(ctrl_row)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.setSpacing(6)
        mfl = self._build_mfl(left)
        stalk = self._build_stalk(left)
        ctrl_layout.addWidget(mfl)
        ctrl_layout.addWidget(stalk)
        left_layout.addWidget(ctrl_row)
        self._left_notebook = QTabWidget()
        self._left_notebook.setMinimumHeight(420)
        # NAV HUD tab — scrollable
        tab_nav = QWidget()
        tab_nav_layout = QVBoxLayout(tab_nav)
        tab_nav_layout.setContentsMargins(4, 4, 4, 4)
        nav_scroll = QScrollArea()
        nav_scroll.setWidgetResizable(True)
        nav_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        nav_scroll.setStyleSheet(f"QScrollArea {{ background: {C['bg']}; border: none; }}")
        nav_content = QWidget()
        nav_content.setStyleSheet(f"background: {C['bg']};")
        nav_content_layout = QVBoxLayout(nav_content)
        nav_content_layout.setContentsMargins(0, 0, 0, 0)
        nav_scroll_part, nav_apply_btn = self._build_nav_controls(tab_nav)
        nav_hud_part, nav_open_live_btn, nav_load_log_btn = self._build_hud_bap(tab_nav)
        nav_content_layout.addWidget(nav_scroll_part)
        nav_content_layout.addWidget(nav_hud_part)
        nav_content_layout.addStretch()
        nav_scroll.setWidget(nav_content)
        tab_nav_layout.addWidget(nav_scroll, 1)
        nav_footer = QWidget()
        nav_footer.setStyleSheet(f"background: {C['bg']};")
        nav_footer_layout = QHBoxLayout(nav_footer)
        nav_footer_layout.setContentsMargins(0, 8, 0, 0)
        nav_footer_layout.addWidget(nav_apply_btn)
        nav_footer_layout.addWidget(nav_open_live_btn)
        nav_footer_layout.addWidget(nav_load_log_btn)
        nav_footer_layout.addStretch()
        tab_nav_layout.addWidget(nav_footer)
        self._left_notebook.addTab(tab_nav, "NAV HUD")
        # VZE tab — scrollable
        tab_vze = QWidget()
        tab_vze_layout = QVBoxLayout(tab_vze)
        tab_vze_layout.setContentsMargins(4, 4, 4, 4)
        vze_scroll = QScrollArea()
        vze_scroll.setWidgetResizable(True)
        vze_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        vze_scroll.setStyleSheet(f"QScrollArea {{ background: {C['bg']}; border: none; }}")
        vze_content = QWidget()
        vze_content.setStyleSheet(f"background: {C['bg']};")
        vze_content_layout = QVBoxLayout(vze_content)
        vze_content_layout.setContentsMargins(0, 0, 0, 0)
        scroll_part, footer_part = self._build_vze_tab(tab_vze)
        vze_content_layout.addWidget(scroll_part)
        vze_scroll.setWidget(vze_content)
        tab_vze_layout.addWidget(vze_scroll, 1)
        tab_vze_layout.addWidget(footer_part)
        self._left_notebook.addTab(tab_vze, "VZE")
        left_layout.addWidget(self._left_notebook, 1)
        content_layout.addWidget(left)

        splitter = QFrame()
        splitter.setFrameShape(QFrame.Shape.VLine)
        splitter.setStyleSheet(f"background: {C['border']}; max-width: 1px;")
        content_layout.addWidget(splitter)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        ecu_lbl = QLabel("ATTACHED ECUs")
        ecu_lbl.setStyleSheet(f"color: {C['sub']}; font-size: 11px;")
        right_layout.addWidget(ecu_lbl)
        self._ecu_scroll = QScrollArea()
        self._ecu_scroll.setWidgetResizable(True)
        self._ecu_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._ecu_scroll.setStyleSheet(f"background: {C['bg']}; border: none;")
        self._ecu_frame = QWidget()
        self._ecu_frame.setStyleSheet(f"background: {C['bg']};")
        self._ecu_frame_layout = QVBoxLayout(self._ecu_frame)
        self._ecu_frame_layout.setContentsMargins(0, 0, 0, 0)
        self._ecu_frame_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self._ecu_scroll.setWidget(self._ecu_frame)
        right_layout.addWidget(self._ecu_scroll, 1)
        right_layout.addWidget(self._build_log(right))
        content_layout.addWidget(right, 1)

        main.addWidget(bar)
        main.addWidget(content, 1)

        self._build_ecu_cards()

    def _build_ign(self, parent):
        pnl = _panel(parent, "IGNITION")
        layout = QVBoxLayout(pnl)
        layout.setContentsMargins(10, 12, 10, 10)
        self._ign_btn = QPushButton("OFF")
        self._ign_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['off']}; font-weight: bold; font-size: 14px;
                border: 1px solid {C['border']}; border-radius: 6px; padding: 10px 24px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        self._ign_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._ign_btn.clicked.connect(self._toggle_ign)
        layout.addWidget(self._ign_btn)
        self._ign_hint = QLabel("Click to enable ignition")
        self._ign_hint.setStyleSheet(f"color: {C['sub']}; font-size: 11px;")
        layout.addWidget(self._ign_hint)
        self._send_ign_cb = QCheckBox("Send ignition updates (0x3C0)")
        self._send_ign_cb.setChecked(bool(self._cfg.get("send_ignition_updates", True)))
        self._send_ign_cb.setStyleSheet(f"color: {C['text']};")
        self._send_ign_cb.stateChanged.connect(lambda s: self._on_send_ign_toggle())
        layout.addWidget(self._send_ign_cb)
        self._inds = {}
        return pnl

    def _on_send_ign_toggle(self):
        on = self._send_ign_cb.isChecked()
        self._cfg["send_ignition_updates"] = on
        if hasattr(self, "_mgr") and self._mgr:
            self._mgr.set_send_ignition_updates(on)

    def _reconnect_can(self):
        idx = self._can_combo.currentIndex()
        cfg = self._can_combo.itemData(idx)
        if not cfg:
            return
        self._log("Reconnecting with selected interface...")
        self._mgr.set_configs([cfg])
        self._mgr.stop()
        self._mgr.start()

    def _build_mfl(self, parent):
        pnl = _panel(parent, "MFL  (0x5BF)")
        layout = QVBoxLayout(pnl)
        g = QWidget()
        g_layout = QGridLayout(g)
        g_layout.setSpacing(6)
        self._btns = {}
        for key, row, col in MFL_LAYOUT:
            b = MFLBtn(g, key, cmd=lambda k=key: self._mfl(k))
            g_layout.addWidget(b, row, col)
            self._btns[key] = b
        layout.addWidget(g)
        return pnl

    def _build_stalk(self, parent):
        pnl = _panel(parent, "STALK / BLINKERS  (16)")
        layout = QVBoxLayout(pnl)
        BTNS = [
            ("blink_left",  "blink_left",  "<",  "LEFT"),
            ("blink_off",   "blink_off",   "X",  "OFF"),
            ("blink_right", "blink_right", ">",  "RIGHT"),
        ]
        g = QWidget()
        g_layout = QHBoxLayout(g)
        g_layout.setSpacing(4)
        self._stalk_btns = {}
        for key, cmd, icon, lbl in BTNS:
            btn = QPushButton(f"{icon}\n{lbl}")
            btn.setStyleSheet(f"""
                QPushButton {{ background: {C['btn']}; color: {C['accent']}; border: 1px solid {C['border']};
                    border-radius: 6px; padding: 8px; font-size: 11px; }}
                QPushButton:hover {{ background: {C['btn_hov']}; }}
                QPushButton:pressed {{ background: {C['btn_act']}; }}
            """)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(48)
            btn.clicked.connect(lambda checked, c=cmd: self._stalk_click(c))
            g_layout.addWidget(btn)
            self._stalk_btns[key] = (btn, cmd)
        layout.addWidget(g)
        self._stalk_enabled = False
        self._update_stalk_buttons()
        return pnl

    def _stalk_click(self, cmd):
        if not self._stalk_enabled:
            return
        ecu = self._find_stalk_ecu()
        if ecu:
            ecu.send_command(cmd)
            self._log("Stalk -> " + cmd)
        else:
            self._log("Stalk: module 16 not attached")

    def _nav_settings(self):
        settings = {
            "enabled": bool(self._cfg.get("hud_nav_enabled", False)),
            "distance_valid": bool(self._cfg.get("hud_distance_valid", True)),
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
            "exitview_variant": self._cfg.get("exitview_variant", "EU"),
            "exitview_id": max(0, min(65535, int(self._cfg.get("exitview_id", 0)))),
            "maneuver_state": self._cfg.get("maneuver_state", "CallForAction"),
        }
        ecu = self._find_infotainment_ecu()
        if ecu is not None and hasattr(ecu, "get_nav_settings"):
            settings.update(ecu.get_nav_settings())
        return settings

    def _parse_sidestreets(self, s):
        """Parse hex bytes from string like '40 80' or '40' -> [0x40, 0x80]. Max 17 bytes."""
        out = []
        for part in (s or "").split():
            try:
                out.append(int(part.strip(), 16) & 0xFF)
            except ValueError:
                pass
        return out[:17]

    def _current_lanes_from_vars(self):
        """Build list of lane dicts from current _nav_lane_vars (for passing to _rebuild_lane_rows)."""
        lanes = []
        for v in self._nav_lane_vars:
            try:
                direction = int((v["direction"].text() or "0").strip(), 16) & 0xFF
                lane_type = int((v["lane_type"].text() or "1").strip(), 16) & 0xFF
                mark_l = int((v["mark_l"].text() or "0").strip()) & 0x0F
                mark_r = int((v["mark_r"].text() or "0").strip()) & 0x0F
                lane_desc = int((v["lane_desc"].text() or "0").strip(), 16) & 0x0F
                guidance = 0x02 if v["preferred"].isChecked() else 0x00
                sidestreets = self._parse_sidestreets(v.get("sidestr") and v["sidestr"].text())
            except ValueError:
                direction, lane_type, mark_l, mark_r, lane_desc, guidance = 0, 1, 0, 0, 0, 0
                sidestreets = []
            lanes.append({
                "direction": direction, "sidestreets": sidestreets, "lane_type": lane_type,
                "marking_left": mark_l, "marking_right": mark_r,
                "lane_description": lane_desc, "guidance": guidance,
            })
        return lanes

    def _rebuild_lane_rows(self, initial_lanes=None):
        """Rebuild lane rows in _nav_lane_rows_frame from _nav_lane_num_var; optionally seed from initial_lanes."""
        while self._nav_lane_rows_layout.count():
            item = self._nav_lane_rows_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._nav_lane_vars.clear()
        try:
            n = max(2, min(8, self._nav_lane_spin.value()))
        except (ValueError, AttributeError):
            n = 3
        if hasattr(self, "_nav_lane_spin"):
            self._nav_lane_spin.setValue(n)
        entry_style = f"background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px;"
        header_style = f"color: {C['sub']}; font-size: 10px;"
        header_row = QWidget()
        header_layout = QHBoxLayout(header_row)
        header_layout.setContentsMargins(0, 0, 0, 4)
        rec_lbl = QLabel("Rec")
        rec_lbl.setStyleSheet(header_style)
        rec_lbl.setFixedWidth(24)
        header_layout.addWidget(rec_lbl)
        for lbl, w in [("Dir", 32), ("Sidestr", 56), ("Type", 32), ("L", 24), ("R", 24), ("Desc", 24)]:
            h = QLabel(lbl)
            h.setStyleSheet(header_style)
            h.setFixedWidth(w)
            if lbl in ("L", "R"):
                h.setToolTip("0=no line, 1=solid, 2=dashed (LaneMarking per BAP)")
            header_layout.addWidget(h)
        header_layout.addStretch()
        self._nav_lane_rows_layout.addWidget(header_row)
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
                ss = L.get("sidestreets", [])
                sidestr = " ".join(f"{b:02X}" for b in ss) if ss else ""
                preferred = int(L.get("guidance", 0)) == 0x02
                if preferred:
                    preferred_set = True
            else:
                direction, lane_type, mark_l, mark_r, lane_desc, sidestr = "00", "01", "0", "0", "0", ""
                preferred = not preferred_set and (i == n - 1 or i == n // 2)
                if preferred:
                    preferred_set = True
            row_w = QWidget()
            row_layout = QHBoxLayout(row_w)
            row_layout.setContentsMargins(0, 2, 0, 2)
            pref_cb = QCheckBox()
            pref_cb.setChecked(preferred)
            pref_cb.setStyleSheet(f"color: {C['text']};")
            row_layout.addWidget(pref_cb)
            le_dir = QLineEdit(direction)
            le_sidestr = QLineEdit(sidestr)
            le_type = QLineEdit(lane_type)
            le_ml = QLineEdit(mark_l)
            le_mr = QLineEdit(mark_r)
            le_ml.setPlaceholderText("0-2")
            le_mr.setPlaceholderText("0-2")
            le_desc = QLineEdit(lane_desc)
            le_sidestr.setPlaceholderText("40 80")
            for le, w in [(le_dir, 32), (le_sidestr, 56), (le_type, 32), (le_ml, 24), (le_mr, 24), (le_desc, 24)]:
                le.setMaxLength(64 if le is le_sidestr else 4)
                le.setFixedWidth(w)
                le.setStyleSheet(entry_style)
                row_layout.addWidget(le)
            row_layout.addStretch()
            self._nav_lane_rows_layout.addWidget(row_w)
            self._nav_lane_vars.append({
                "direction": le_dir, "sidestr": le_sidestr, "lane_type": le_type, "mark_l": le_ml, "mark_r": le_mr,
                "lane_desc": le_desc, "preferred": pref_cb,
            })
        if not preferred_set and self._nav_lane_vars:
            self._nav_lane_vars[0]["preferred"].setChecked(True)

    def _build_nav_controls(self, parent):
        pnl = QWidget(parent)
        pnl.setStyleSheet(f"background: {C['bg']};")
        settings = self._nav_settings()
        layout = QVBoxLayout(pnl)
        layout.setContentsMargins(10, 12, 10, 10)

        self._nav_enabled_cb = QCheckBox("Navigation enabled")
        self._nav_enabled_cb.setChecked(bool(settings["enabled"]))
        self._nav_enabled_cb.setStyleSheet(f"color: {C['text']};")
        layout.addWidget(self._nav_enabled_cb)

        dist_grp = _panel(pnl, "Distance")
        dist_layout = QVBoxLayout(dist_grp)
        self._nav_distance_valid_cb = QCheckBox("Distance valid")
        self._nav_distance_valid_cb.setChecked(bool(settings.get("distance_valid", True)))
        self._nav_distance_valid_cb.setStyleSheet(f"color: {C['text']};")
        dist_layout.addWidget(self._nav_distance_valid_cb)
        dist_row = QWidget()
        dist_row_layout = QHBoxLayout(dist_row)
        dist_row_layout.setContentsMargins(0, 4, 0, 4)
        unit = str(settings.get("distance_unit", "m")).lower()
        dist_m = int(settings.get("distance_m", 0))
        dist_display = dist_m // 1000 if unit == "km" else dist_m
        self._nav_distance_var = QLineEdit(str(dist_display))
        self._nav_distance_unit_combo = QComboBox()
        self._nav_distance_unit_combo.addItems(["m", "km"])
        idx = self._nav_distance_unit_combo.findText("km" if unit == "km" else "m")
        if idx >= 0:
            self._nav_distance_unit_combo.setCurrentIndex(idx)
        _input_style = f"background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px;"
        self._nav_distance_var.setMinimumHeight(INPUT_MIN_H)
        self._nav_distance_var.setStyleSheet(_input_style)
        self._nav_distance_unit_combo.setMinimumHeight(INPUT_MIN_H)
        self._nav_distance_unit_combo.setStyleSheet(f"QComboBox {{ background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px; }}")
        dist_row_layout.addWidget(QLabel("Value:"))
        dist_row_layout.addWidget(self._nav_distance_var)
        dist_row_layout.addWidget(QLabel("Unit:"))
        dist_row_layout.addWidget(self._nav_distance_unit_combo)
        dist_layout.addWidget(dist_row)
        self._nav_distance_enabled_cb = QCheckBox("Bargraph")
        self._nav_distance_enabled_cb.setChecked(bool(settings.get("distance_enabled", True)))
        self._nav_distance_enabled_cb.setStyleSheet(f"color: {C['text']};")
        dist_layout.addWidget(self._nav_distance_enabled_cb)
        graph_row = QWidget()
        graph_row_layout = QHBoxLayout(graph_row)
        graph_row_layout.setContentsMargins(0, 2, 0, 2)
        graph_row_layout.addWidget(QLabel("Graph %:"))
        self._nav_distance_graph_var = QSlider(Qt.Orientation.Horizontal)
        self._nav_distance_graph_var.setRange(0, 100)
        self._nav_distance_graph_var.setValue(min(100, max(0, int(settings.get('distance_graph', 0x64)) & 0xFF)))
        graph_row_layout.addWidget(self._nav_distance_graph_var)
        dist_layout.addWidget(graph_row)

        maneuver_grp = _panel(pnl, "Next maneuver")
        maneuver_grp_layout = QVBoxLayout(maneuver_grp)
        maneuver_row = QWidget()
        maneuver_row_layout = QHBoxLayout(maneuver_row)
        maneuver_row_layout.setContentsMargins(0, 4, 0, 4)
        self._nav_arrow_main_var = QLineEdit(f"{int(settings.get('arrow_main') or 0) & 0xFF:02X}")
        self._nav_arrow_dir_var = QLineEdit(f"{int(settings.get('arrow_dir') or 0) & 0xFF:02X}")
        for le in (self._nav_arrow_main_var, self._nav_arrow_dir_var):
            le.setMinimumHeight(INPUT_MIN_H)
            le.setStyleSheet(_input_style)
        maneuver_row_layout.addWidget(QLabel("Arrow main:"))
        maneuver_row_layout.addWidget(self._nav_arrow_main_var)
        def _arrow_main_minus():
            try:
                v = int((self._nav_arrow_main_var.text() or "0").strip(), 16) & 0xFF
            except ValueError:
                v = 0
            self._nav_arrow_main_var.setText(f"{(v - 1) & 0xFF:02X}")
            self._apply_nav_settings()
        def _arrow_main_plus():
            try:
                v = int((self._nav_arrow_main_var.text() or "0").strip(), 16) & 0xFF
            except ValueError:
                v = 0
            self._nav_arrow_main_var.setText(f"{(v + 1) & 0xFF:02X}")
            self._apply_nav_settings()
        arrow_main_minus_btn = QPushButton("−")
        arrow_main_minus_btn.setFixedWidth(28)
        arrow_main_minus_btn.clicked.connect(_arrow_main_minus)
        arrow_main_plus_btn = QPushButton("+")
        arrow_main_plus_btn.setFixedWidth(28)
        arrow_main_plus_btn.clicked.connect(_arrow_main_plus)
        maneuver_row_layout.addWidget(arrow_main_minus_btn)
        maneuver_row_layout.addWidget(arrow_main_plus_btn)
        maneuver_row_layout.addWidget(QLabel("Arrow dir:"))
        maneuver_row_layout.addWidget(self._nav_arrow_dir_var)
        maneuver_row_layout.addStretch()
        maneuver_grp_layout.addWidget(maneuver_row)
        maneuver_grp_layout.addWidget(QLabel("Street name:"))
        self._nav_street_var = QLineEdit(str(settings["street_name"]))
        self._nav_street_var.setMinimumHeight(INPUT_MIN_H)
        self._nav_street_var.setStyleSheet(_input_style)
        maneuver_grp_layout.addWidget(self._nav_street_var)

        dist_maneuver_row = QWidget()
        dist_maneuver_layout = QHBoxLayout(dist_maneuver_row)
        dist_maneuver_layout.setContentsMargins(0, 0, 0, 0)
        dist_maneuver_layout.addWidget(dist_grp)
        dist_maneuver_layout.addWidget(maneuver_grp)
        layout.addWidget(dist_maneuver_row)

        lane_grp = _panel(pnl, "Lane guidance")
        lane_grp_layout = QVBoxLayout(lane_grp)
        self._nav_lane_guidance_cb = QCheckBox("Enabled")
        self._nav_lane_guidance_cb.setChecked(bool(settings.get("lane_guidance_enabled", False)))
        self._nav_lane_guidance_cb.setStyleSheet(f"color: {C['text']};")
        lane_grp_layout.addWidget(self._nav_lane_guidance_cb)
        lane_top = QWidget()
        lane_top_layout = QHBoxLayout(lane_top)
        lane_top_layout.setContentsMargins(0, 4, 0, 4)
        lane_top_layout.addWidget(QLabel("Lanes:"))
        self._nav_lane_spin = QSpinBox()
        self._nav_lane_spin.setMinimumHeight(INPUT_MIN_H)
        self._nav_lane_spin.setStyleSheet(f"background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px;")
        self._nav_lane_spin.setRange(2, 8)
        self._nav_lane_spin.setValue(max(2, min(8, int(settings.get("lane_num_lanes", 3)))))
        self._nav_lane_spin.setFixedWidth(50)
        lane_top_layout.addWidget(self._nav_lane_spin)
        minus_btn = QPushButton("−")
        minus_btn.setFixedWidth(28)
        minus_btn.clicked.connect(lambda: self._nav_lane_spin.setValue(max(2, self._nav_lane_spin.value() - 1)) or self._rebuild_lane_rows(initial_lanes=self._current_lanes_from_vars()[:self._nav_lane_spin.value() - 1]))
        plus_btn = QPushButton("+")
        plus_btn.setFixedWidth(28)
        plus_btn.clicked.connect(lambda: self._nav_lane_spin.setValue(min(8, self._nav_lane_spin.value() + 1)) or self._rebuild_lane_rows(initial_lanes=self._current_lanes_from_vars() + [{"direction": 0, "sidestreets": [], "lane_type": 1, "marking_left": 0, "marking_right": 0, "lane_description": 0, "guidance": 0}]))
        lane_top_layout.addWidget(minus_btn)
        lane_top_layout.addWidget(plus_btn)
        lane_top_layout.addStretch()
        lane_grp_layout.addWidget(lane_top)

        self._nav_lane_rows_frame = QWidget()
        self._nav_lane_rows_layout = QVBoxLayout(self._nav_lane_rows_frame)
        self._nav_lane_rows_layout.setContentsMargins(0, 0, 0, 0)
        lane_grp_layout.addWidget(self._nav_lane_rows_frame)

        self._nav_lane_vars = []
        initial_lanes = settings.get("lanes")
        if not initial_lanes:
            n = max(2, min(8, int(settings.get("lane_num_lanes", 3))))
            rec = max(0, min(n - 1, int(settings.get("lane_recommended", 0))))
            initial_lanes = [
                {"direction": 0, "sidestreets_len": 0, "lane_type": 1, "marking_left": 0, "marking_right": 0, "lane_description": 0, "guidance": 0x02 if i == rec else 0}
                for i in range(n)
            ]
        self._rebuild_lane_rows(initial_lanes=initial_lanes)

        def _lane_minus():
            n = self._nav_lane_spin.value()
            if n <= 2:
                return
            self._nav_lane_spin.setValue(n - 1)
            self._rebuild_lane_rows(initial_lanes=self._current_lanes_from_vars()[: n - 1])
        def _lane_plus():
            n = self._nav_lane_spin.value()
            if n >= 8:
                return
            self._nav_lane_spin.setValue(n + 1)
            initial = self._current_lanes_from_vars()
            initial.append({"direction": 0, "sidestreets_len": 0, "lane_type": 1, "marking_left": 0, "marking_right": 0, "lane_description": 0, "guidance": 0})
            self._rebuild_lane_rows(initial_lanes=initial)
        minus_btn.clicked.disconnect()
        plus_btn.clicked.disconnect()
        minus_btn.clicked.connect(_lane_minus)
        plus_btn.clicked.connect(_lane_plus)

        maneuver_state_grp = _panel(pnl, "ManeuverState (0x37)")
        maneuver_state_layout = QVBoxLayout(maneuver_state_grp)
        maneuver_state_layout.addWidget(QLabel("State:"))
        self._nav_maneuver_state_combo = QComboBox()
        self._nav_maneuver_state_combo.addItems([
            "init/unknown", "Follow", "Prepare", "Distance", "CallForAction",
        ])
        self._nav_maneuver_state_combo.setStyleSheet(f"""
            QComboBox {{ background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px; }}
        """)
        maneuver_state_layout.addWidget(self._nav_maneuver_state_combo)

        exitview_grp = _panel(pnl, "Exit view")
        exitview_layout = QVBoxLayout(exitview_grp)
        exitview_layout.setContentsMargins(0, 4, 0, 4)
        variant_row = QWidget()
        variant_row_layout = QHBoxLayout(variant_row)
        variant_row_layout.setContentsMargins(0, 0, 0, 0)
        variant_row_layout.addWidget(QLabel("Variant:"))
        self._nav_exitview_variant_combo = QComboBox()
        self._nav_exitview_variant_combo.addItems(["EU", "NAR", "ROW", "ASIA", "Unknown"])
        self._nav_exitview_variant_combo.setStyleSheet(f"""
            QComboBox {{ background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px; }}
        """)
        variant_row_layout.addWidget(self._nav_exitview_variant_combo)
        variant_row_layout.addStretch()
        exitview_layout.addWidget(variant_row)
        id_row = QWidget()
        id_row_layout = QHBoxLayout(id_row)
        id_row_layout.setContentsMargins(0, 4, 0, 0)
        exitview_id_lbl = QLabel("ID (0=off):")
        exitview_id_lbl.setToolTip("Exit number shown on HUD (e.g. 42 = Exit 42). 0 disables exit view.")
        id_row_layout.addWidget(exitview_id_lbl)
        self._nav_exitview_id_spin = QSpinBox()
        self._nav_exitview_id_spin.setRange(0, 65535)
        self._nav_exitview_id_spin.setValue(max(0, min(65535, int(settings.get("exitview_id", 0)))))
        self._nav_exitview_id_spin.setStyleSheet(f"background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px;")
        self._nav_exitview_id_spin.setFixedWidth(80)
        id_row_layout.addWidget(self._nav_exitview_id_spin)
        exitview_minus_btn = QPushButton("−")
        exitview_minus_btn.setFixedWidth(28)
        exitview_minus_btn.clicked.connect(lambda: (
            self._nav_exitview_id_spin.setValue(65535 if self._nav_exitview_id_spin.value() <= 0 else self._nav_exitview_id_spin.value() - 1),
            self._apply_nav_settings()
        ))
        exitview_plus_btn = QPushButton("+")
        exitview_plus_btn.setFixedWidth(28)
        exitview_plus_btn.clicked.connect(lambda: (
            self._nav_exitview_id_spin.setValue(0 if self._nav_exitview_id_spin.value() >= 65535 else self._nav_exitview_id_spin.value() + 1),
            self._apply_nav_settings()
        ))
        id_row_layout.addWidget(exitview_minus_btn)
        id_row_layout.addWidget(exitview_plus_btn)
        id_row_layout.addStretch()
        exitview_layout.addWidget(id_row)
        idx = self._nav_exitview_variant_combo.findText(settings.get("exitview_variant", "EU"))
        if idx >= 0:
            self._nav_exitview_variant_combo.setCurrentIndex(idx)

        idx = self._nav_maneuver_state_combo.findText(settings.get("maneuver_state", "CallForAction"))
        if idx >= 0:
            self._nav_maneuver_state_combo.setCurrentIndex(idx)

        maneuver_exit_col = QWidget()
        maneuver_exit_col_layout = QVBoxLayout(maneuver_exit_col)
        maneuver_exit_col_layout.setContentsMargins(0, 0, 0, 0)
        maneuver_exit_col_layout.addWidget(maneuver_state_grp)
        maneuver_exit_col_layout.addWidget(exitview_grp)
        lane_maneuver_exit_row = QWidget()
        lane_maneuver_exit_layout = QHBoxLayout(lane_maneuver_exit_row)
        lane_maneuver_exit_layout.setContentsMargins(0, 0, 0, 0)
        lane_maneuver_exit_layout.addWidget(lane_grp)
        lane_maneuver_exit_layout.addWidget(maneuver_exit_col)
        layout.addWidget(lane_maneuver_exit_row)

        apply_btn = QPushButton("Apply Nav Settings")
        apply_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 8px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        apply_btn.clicked.connect(self._apply_nav_settings)
        return pnl, apply_btn

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
        for key, (btn, cmd) in self._stalk_btns.items():
            btn.setEnabled(self._stalk_enabled)
            if self._stalk_enabled:
                btn.setStyleSheet(f"""
                    QPushButton {{ background: {C['btn']}; color: {C['accent'] if key != 'blink_off' else C['sub']};
                        border: 1px solid {C['border']}; border-radius: 6px; padding: 8px; }}
                    QPushButton:hover {{ background: {C['btn_hov']}; }}
                    QPushButton:pressed {{ background: {C['btn_act']}; }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{ background: {C['btn']}; color: {C['border']}; border: 1px solid {C['border']};
                        border-radius: 6px; padding: 8px; }}
                    QPushButton:disabled {{ color: {C['border']}; }}
                """)

    def _build_log(self, parent):
        pnl = _panel(parent, "LOG")
        layout = QVBoxLayout(pnl)
        hdr = QWidget()
        hdr_layout = QHBoxLayout(hdr)
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        copy_btn = QPushButton("Copy")
        copy_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 4px 10px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        copy_btn.clicked.connect(self._copy_log)
        hdr_layout.addStretch()
        hdr_layout.addWidget(copy_btn)
        layout.addWidget(hdr)
        self._log_w = QPlainTextEdit()
        self._log_w.setReadOnly(True)
        self._log_w.setStyleSheet(f"""
            QPlainTextEdit {{ background: {C['log_bg']}; color: {C['log_fg']};
                font-family: Consolas; font-size: 11px; border: 1px solid {C['border']}; border-radius: 4px; }}
        """)
        self._log_w.setMinimumHeight(120)
        layout.addWidget(self._log_w)
        return pnl

    def _build_hud_bap(self, parent):
        pnl = _panel(parent, "HUD BAP")
        layout = QVBoxLayout(pnl)
        self._hud_log_lbl = QLabel(f"Session log: {os.path.basename(str(self._hud_session.path))}")
        self._hud_log_lbl.setStyleSheet(f"color: {C['sub']}; font-size: 11px;")
        self._hud_log_lbl.setWordWrap(True)
        layout.addWidget(self._hud_log_lbl)
        open_live_btn = QPushButton("Open Live Window")
        open_live_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 6px 12px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        open_live_btn.clicked.connect(self._open_hud_bap_window)
        load_log_btn = QPushButton("Load Log")
        load_log_btn.setStyleSheet(f"""
            QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']};
                border-radius: 4px; padding: 6px 12px; }}
            QPushButton:hover {{ background: {C['btn_hov']}; }}
        """)
        load_log_btn.clicked.connect(self._load_hud_bap_log)
        return pnl, open_live_btn, load_log_btn

    def _build_vze_tab(self, parent):
        """VZE tab: A5 loaded check + sign type + First sign groupbox (speed + bits)."""
        pnl = QWidget(parent)
        layout = QVBoxLayout(pnl)
        layout.setContentsMargins(10, 12, 10, 10)
        self._vze_a5_status_lbl = QLabel("Module A5: checking…")
        self._vze_a5_status_lbl.setStyleSheet(f"color: {C['sub']};")
        layout.addWidget(self._vze_a5_status_lbl)
        QTimer.singleShot(100, self._update_vze_a5_status)

        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "pack_vze_01", None):
            layout.addWidget(QLabel("VZE pack/unpack not available (a5_drvassist)."))
            return pnl, QWidget()

        body = QGridLayout()
        body.addWidget(QLabel("Sign type"), 0, 0)
        self._vze_sign_type_combo = QComboBox()
        self._vze_sign_type_combo.setMinimumHeight(INPUT_MIN_H)
        self._vze_sign_type_combo.setStyleSheet(f"""
            QComboBox {{ background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px; }}
            QComboBox::drop-down {{ border: none; }}
        """)
        self._vze_sign_type_combo.addItems(["EU_RDW (0)", "USA (1)", "Canada (2)", "China (3)"])
        self._vze_sign_type_combo.currentIndexChanged.connect(self._apply_vze_settings)
        body.addWidget(self._vze_sign_type_combo, 0, 1)
        layout.addLayout(body)

        first_sign_gb = _panel(parent, "First sign")
        first_layout = QVBoxLayout(first_sign_gb)
        first_layout.setContentsMargins(10, 10, 10, 10)
        speed_row = QWidget()
        speed_row_layout = QHBoxLayout(speed_row)
        speed_row_layout.setContentsMargins(0, 0, 0, 0)
        speed_row_layout.addWidget(QLabel("Speed (km/h):"))
        self._vze_vz1_var = QLineEdit("90")
        self._vze_vz1_var.setMinimumHeight(INPUT_MIN_H)
        self._vze_vz1_var.setStyleSheet(f"background: {C['panel']}; color: {C['text']}; border: 1px solid {C['border']}; padding: 4px 6px; min-height: {INPUT_MIN_H}px;")
        self._vze_vz1_var.setFixedWidth(60)
        self._vze_vz1_var.editingFinished.connect(self._apply_vze_settings)
        speed_row_layout.addWidget(self._vze_vz1_var)
        speed_row_layout.addSpacing(16)
        self._vze_bit_blink = QCheckBox("Blink")
        self._vze_bit_weather = QCheckBox("Weather")
        self._vze_bit_rain = QCheckBox("Rain")
        self._vze_bit_warning = QCheckBox("Warning clock")
        for cb in (self._vze_bit_blink, self._vze_bit_weather, self._vze_bit_rain, self._vze_bit_warning):
            cb.setStyleSheet(f"color: {C['text']};")
            cb.stateChanged.connect(self._apply_vze_settings)
            speed_row_layout.addWidget(cb)
        speed_row_layout.addStretch()
        first_layout.addWidget(speed_row)
        layout.addWidget(first_sign_gb)

        other_bits_gb = _panel(parent, "Other bits (Bytes 4–7)")
        bit_layout = QGridLayout(other_bits_gb)
        bit_layout.setContentsMargins(10, 10, 10, 10)
        self._vze_byte_vars = []
        for byteno in range(4):
            row_vars = []
            bit_layout.addWidget(QLabel(["Byte 4", "Byte 5", "Byte 6", "Byte 7"][byteno]), byteno, 0)
            for bit in range(8):
                cb = QCheckBox(str(bit))
                cb.setStyleSheet(f"color: {C['text']};")
                cb.stateChanged.connect(self._apply_vze_settings)
                bit_layout.addWidget(cb, byteno, bit + 1)
                row_vars.append(cb)
            self._vze_byte_vars.append(row_vars)
        layout.addWidget(other_bits_gb)

        footer = QWidget()
        footer.setStyleSheet(f"background: {C['bg']};")
        footer_layout = QVBoxLayout(footer)
        footer_layout.setContentsMargins(0, 8, 0, 0)
        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        apply_btn = QPushButton("Apply VZE (0x181)")
        apply_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']}; border-radius: 4px; padding: 6px 12px; }}")
        apply_btn.clicked.connect(self._apply_vze_settings)
        load_btn = QPushButton("Load from A5")
        load_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['text']}; border: 1px solid {C['border']}; border-radius: 4px; padding: 6px 12px; }}")
        load_btn.clicked.connect(self._load_vze_from_a5)
        btn_layout.addWidget(apply_btn)
        btn_layout.addWidget(load_btn)
        footer_layout.addWidget(btn_row)
        self._vze_bytes_lbl = QLabel("0x181 payload: -- -- -- -- -- -- -- --")
        self._vze_bytes_lbl.setStyleSheet(f"color: {C['sub']}; font-family: Consolas; font-size: 11px;")
        footer_layout.addWidget(self._vze_bytes_lbl)
        QTimer.singleShot(50, self._update_vze_bytes_display)
        return pnl, footer

    def _update_vze_a5_status(self):
        ecu = self._find_a5_ecu()
        if hasattr(self, "_vze_a5_status_lbl") and self._vze_a5_status_lbl:
            if ecu is not None:
                self._vze_a5_status_lbl.setText("Module A5: loaded")
                self._vze_a5_status_lbl.setStyleSheet(f"color: {C['on']};")
            else:
                self._vze_a5_status_lbl.setText("Module A5: not loaded")
                self._vze_a5_status_lbl.setStyleSheet(f"color: {C['off']};")

    def _vze_bits_to_bytes(self):
        """Build bytes 4–7 from First sign checkboxes (B4.3,4.6,4.7, B5.0) + other bits grid."""
        def row_to_byte(vars_row):
            return sum(2**i for i in range(8) if vars_row[i].isChecked()) & 0xFF if vars_row else 0
        bv = getattr(self, "_vze_byte_vars", [])
        g4 = row_to_byte(bv[0]) if len(bv) > 0 else 0
        g5 = row_to_byte(bv[1]) if len(bv) > 1 else 0
        g6 = row_to_byte(bv[2]) if len(bv) > 2 else 0
        g7 = row_to_byte(bv[3]) if len(bv) > 3 else 0
        if hasattr(self, "_vze_bit_blink") and self._vze_bit_blink:
            g4 = (g4 & ~0xC8) | (int(self._vze_bit_blink.isChecked()) << 3) | (int(self._vze_bit_weather.isChecked()) << 6) | (int(self._vze_bit_rain.isChecked()) << 7)
            g5 = (g5 & ~0x01) | int(self._vze_bit_warning.isChecked())
        return g4, g5, g6, g7

    def _update_vze_bytes_display(self):
        """Show the current form values packed as 0x181 payload (hex bytes) at the bottom."""
        if not hasattr(self, "_vze_bytes_lbl") or not self._vze_bytes_lbl:
            return
        a5_mod = sys.modules.get("a5_drvassist")
        if not a5_mod or not getattr(a5_mod, "pack_vze_01", None):
            return
        try:
            idx = self._vze_sign_type_combo.currentIndex() if hasattr(self, "_vze_sign_type_combo") else 0
            anzeigemodus = max(0, min(3, idx))
            vz1 = max(0, min(255, int((self._vze_vz1_var.text() or "0").strip())))
            b4, b5, b6, b7 = self._vze_bits_to_bytes()
        except (ValueError, AttributeError):
            self._vze_bytes_lbl.setText("0x181 payload: (invalid values)")
            return
        payload = a5_mod.pack_vze_01(anzeigemodus=anzeigemodus, verkehrszeichen_1=vz1, byte_4=b4, byte_5=b5, byte_6=b6, byte_7=b7)
        hex_str = " ".join(f"{b:02X}" for b in payload[:8])
        self._vze_bytes_lbl.setText(f"0x181 payload: {hex_str}")

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
            self._vze_sign_type_combo.setCurrentIndex(max(0, min(3, params.get("anzeigemodus", 0))))
            self._vze_vz1_var.setText(str(max(0, min(255, params.get("verkehrszeichen_1", 0)))))
            b4 = params.get("byte_4", 0) & 0xFF
            b5 = params.get("byte_5", 0) & 0xFF
            self._vze_bit_blink.setChecked(bool((b4 >> 3) & 1))
            self._vze_bit_weather.setChecked(bool((b4 >> 6) & 1))
            self._vze_bit_rain.setChecked(bool((b4 >> 7) & 1))
            self._vze_bit_warning.setChecked(bool((b5 >> 0) & 1))
            for byteno, key in enumerate(("byte_4", "byte_5", "byte_6", "byte_7")):
                b = params.get(key, 0) & 0xFF
                if getattr(self, "_vze_byte_vars", None) and len(self._vze_byte_vars) > byteno:
                    for i in range(8):
                        self._vze_byte_vars[byteno][i].setChecked(bool((b >> i) & 1))
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
            idx = self._vze_sign_type_combo.currentIndex() if hasattr(self, "_vze_sign_type_combo") else 0
            anzeigemodus = max(0, min(3, idx))
            vz1 = max(0, min(255, int((self._vze_vz1_var.text() or "0").strip())))
            b4, b5, b6, b7 = self._vze_bits_to_bytes()
        except (ValueError, AttributeError) as e:
            self._log(f"Apply VZE: invalid number — {e}")
            return
        payload = a5_mod.pack_vze_01(anzeigemodus=anzeigemodus, verkehrszeichen_1=vz1, byte_4=b4, byte_5=b5, byte_6=b6, byte_7=b7)
        ecu.update_data(0x181, payload)
        self._update_vze_bytes_display()
        self._log(f"Apply VZE: 0x181 type={anzeigemodus} speed={vz1} km/h")

    def _build_ecu_cards(self):
        for ecu in self._mgr.get_ecus():
            card = ECUCard(self._ecu_frame, ecu)
            self._ecu_frame_layout.addWidget(card)
            self._cards.append(card)
        if not self._cards:
            lbl = QLabel("No ECU modules found in modules/")
            lbl.setStyleSheet(f"color: {C['sub']};")
            self._ecu_frame_layout.addWidget(lbl)

    # ── Actions ───────────────────────────────────────────────────────────────

    def _on_ignition_from_bus(self, on: bool):
        """Update UI from 0x3C0 received on bus (another device sending ignition). Manager already updated ECUs."""
        if self._ign == on:
            return
        self._ign = on
        if on:
            self._ign_btn.setText("ON")
            self._ign_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['on']}; font-weight: bold; }}")
            self._ign_hint.setText("KL15 active (from bus)")
            for d in self._inds.values():
                d.setStyleSheet(f"color: {C['accent']};")
            self._log("Ignition ON (from bus 0x3C0)")
        else:
            self._ign_btn.setText("OFF")
            self._ign_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['off']}; font-weight: bold; }}")
            self._ign_hint.setText("Click to enable ignition")
            for d in self._inds.values():
                d.setStyleSheet(f"color: {C['border']};")
            self._log("Ignition OFF (from bus 0x3C0)")
        self._update_stalk_buttons()

    def _toggle_ign(self):
        self._ign = not self._ign; self._mgr.set_ignition(self._ign)
        if self._ign:
            self._ign_btn.setText("ON")
            self._ign_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['on']}; font-weight: bold; }}")
            self._ign_hint.setText("KL15 active — click to disable")
            for d in self._inds.values():
                d.setStyleSheet(f"color: {C['accent']};")
            self._log("Ignition ON")
        else:
            self._ign_btn.setText("OFF")
            self._ign_btn.setStyleSheet(f"QPushButton {{ background: {C['btn']}; color: {C['off']}; font-weight: bold; }}")
            self._ign_hint.setText("Click to enable ignition")
            for d in self._inds.values():
                d.setStyleSheet(f"color: {C['border']};")
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
            distance_val = max(0, int((self._nav_distance_var.text() or "0").strip()))
            distance_graph = max(0, min(0x64, self._nav_distance_graph_var.value()))
            distance_unit = self._nav_distance_unit_combo.currentText().lower()
            distance_m = distance_val * 1000 if distance_unit == "km" else distance_val
            arrow_main = self._parse_hex_byte(self._nav_arrow_main_var.text(), "Arrow main")
            arrow_dir = self._parse_hex_byte(self._nav_arrow_dir_var.text(), "Arrow dir")
        except ValueError as exc:
            self._log(f"Apply Nav: {exc}")
            return

        street_name = (self._nav_street_var.text() or "").strip() or "Offroad"
        enabled = self._nav_enabled_cb.isChecked()
        distance_valid = self._nav_distance_valid_cb.isChecked()
        distance_enabled = self._nav_distance_enabled_cb.isChecked()
        lane_guidance_enabled = self._nav_lane_guidance_cb.isChecked()

        lanes = []
        preferred_index = None
        for i, v in enumerate(self._nav_lane_vars):
            try:
                direction = int((v["direction"].text() or "0").strip(), 16) & 0xFF
                lane_type = int((v["lane_type"].text() or "1").strip(), 16) & 0xFF
                mark_l = int((v["mark_l"].text() or "0").strip()) & 0x0F
                mark_r = int((v["mark_r"].text() or "0").strip()) & 0x0F
                lane_desc = int((v["lane_desc"].text() or "0").strip(), 16) & 0x0F
                preferred = v["preferred"].isChecked()
                sidestreets = self._parse_sidestreets(v.get("sidestr") and v["sidestr"].text())
            except ValueError:
                direction, lane_type, mark_l, mark_r, lane_desc = 0, 1, 0, 0, 0
                sidestreets = []
                preferred = i == 0
            guidance = 0x02 if preferred else 0x00
            if preferred:
                preferred_index = i
            lanes.append({
                "direction": direction, "sidestreets": sidestreets, "lane_type": lane_type,
                "marking_left": mark_l, "marking_right": mark_r,
                "lane_description": lane_desc, "guidance": guidance,
            })
        if preferred_index is None and lanes:
            lanes[0]["guidance"] = 0x02

        self._cfg["hud_nav_enabled"] = enabled
        self._cfg["hud_distance_valid"] = distance_valid
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

        exitview_variant = self._nav_exitview_variant_combo.currentText()
        exitview_id = max(0, min(65535, self._nav_exitview_id_spin.value()))
        maneuver_state = self._nav_maneuver_state_combo.currentText()
        self._cfg["exitview_variant"] = exitview_variant
        self._cfg["exitview_id"] = exitview_id
        self._cfg["maneuver_state"] = maneuver_state

        if hasattr(ecu, "configure_nav"):
            ecu.configure_nav(
                enabled=enabled,
                distance_valid=distance_valid,
                distance_enabled=distance_enabled,
                distance_m=distance_m,
                distance_graph=distance_graph,
                distance_unit=distance_unit,
                street_name=street_name,
                arrow_main=arrow_main,
                arrow_dir=arrow_dir,
                lane_guidance_enabled=lane_guidance_enabled,
                lanes=lanes,
                exitview_variant=exitview_variant,
                exitview_id=exitview_id,
                maneuver_state=maneuver_state,
            )
            self._log(
                f"Apply Nav: enabled={'yes' if enabled else 'no'}"
                f" distance_valid={'yes' if distance_valid else 'no'}"
                f" distance_bargraph={'on' if distance_enabled else 'off'}"
                f" lane_guidance={'yes' if lane_guidance_enabled else 'no'}"
                f" main=0x{arrow_main:02X}"
                f" dir=0x{arrow_dir:02X}"
                f" distance={distance_m}m"
                f" graph=0x{distance_graph:02X}"
                f" street={street_name}"
                f" exitview={exitview_variant} id={exitview_id}"
                f" maneuver_state={maneuver_state}"
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
        if self._hud_live_win is not None and self._hud_live_win.isVisible():
            self._hud_live_win.raise_()
            self._hud_live_win.activateWindow()
            return
        self._hud_live_win = BapTableWindow(self, "HUD Bap")
        self._hud_live_win.destroyed.connect(lambda: setattr(self, "_hud_live_win", None))
        self._hud_live_win.load_messages(self._hud_session.messages)
        self._hud_live_win.set_status(f"Live session log: {self._hud_session.path}")
        self._hud_live_win.show()

    def _load_hud_bap_log(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load HUD BAP Log",
            self._hud_logs_dir,
            "Log files (*.log);;All files (*.*)",
        )
        if not path:
            return
        win = BapTableWindow(self, f"HUD Bap Log - {os.path.basename(path)}")
        win.set_status(f"Loading {path} ...")
        win.show()
        win.raise_()
        win.activateWindow()
        QApplication.processEvents()

        try:
            messages = load_hud_bap_messages(path)
            win.load_messages(messages)
            if messages:
                win.set_status(f"{len(messages)} decoded messages from {path}")
            else:
                win.set_status(f"No BAP messages found in {path} (check Busmaster format)")
        except Exception as exc:
            win.set_status(f"Failed to load {path}: {exc}")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _on_bus_frame(self, direction: str, arb_id: int, data: list):
        message = self._hud_session.handle_frame(direction, arb_id, data)
        if message is None:
            return
        if self._hud_live_win is not None and self._hud_live_win.isVisible():
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
        if self._hud_live_win is not None and self._hud_live_win.isVisible():
            if messages:
                self._hud_live_win.append_messages(messages)
            self._hud_live_win.set_status(
                f"Live session log: {self._hud_session.path}"
                f"  pending={self._hud_frame_q.qsize()}"
            )

    def _copy_log(self):
        try:
            text = self._log_w.toPlainText()
            QApplication.clipboard().setText(text)
            self._log("Log copied to clipboard")
        except Exception as exc:
            self._log(f"Copy log failed: {exc}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        sb = self._log_w.verticalScrollBar()
        stick_to_bottom = sb.value() >= sb.maximum() - 4
        self._log_w.setReadOnly(False)
        self._log_w.appendPlainText(f"[{ts}] {msg}")
        lines = self._log_w.document().lineCount()
        if lines > 600:
            cursor = QTextCursor(self._log_w.document())
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.movePosition(QTextCursor.MoveOperation.Down, QTextCursor.MoveMode.KeepAnchor, 80)
            cursor.removeSelectedText()
        if stick_to_bottom:
            sb.setValue(sb.maximum())
        self._log_w.setReadOnly(True)

    def _setstatus(self, state: str, detail: str):
        if not hasattr(self, "_slbl") or not self._slbl:
            return
        col = {"ok":C["on"],"error":C["off"],"off":C["sub"]}.get(state, C["sub"])
        lbl = {"ok":f"CONNECTED  {detail}","error":f"ERROR  {detail}",
               "off":"DISCONNECTED"}.get(state, state)
        self._dot.setStyleSheet(f"color: {col};")
        self._slbl.setText(lbl)
        self._slbl.setStyleSheet(f"color: {col};")

    def _start_refresh(self):
        def _r():
            for card in self._cards:
                try: card.refresh()
                except Exception: pass
            try:
                q  = self._mgr._tx_q.qsize()
                pq = self._mgr._prio_q.qsize()
                txt = f"Q:{q}" if pq == 0 else f"Q:{q}  P:{pq}"
                self._qlbl.setText(txt)
                self._qlbl.setStyleSheet(f"color: {C['off'] if q > 50 else C['sub']};")
            except Exception:
                pass
            QTimer.singleShot(self.REFRESH, _r)
        QTimer.singleShot(self.REFRESH, _r)

    def _start_hud_frame_drain(self):
        def _r():
            self._drain_hud_frames(batch_limit=1000)
            QTimer.singleShot(self.HUD_FRAME_REFRESH, _r)
        QTimer.singleShot(self.HUD_FRAME_REFRESH, _r)

    def _close(self):
        self._mgr.stop()
        self._hud_session.close()

    def closeEvent(self, event):
        self._close()
        event.accept()


if __name__ == "__main__":
    args = parse_args()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = App(config=make_app_config(args))
    window.show()
    sys.exit(app.exec())
