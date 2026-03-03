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

import can, threading, time, importlib.util, os, sys, queue

# ── Serial throughput (csscan_serial) ─────────────────────────────────────────
# At 115200 baud a CAN frame takes ~2–3ms → ~300–400 fps max. Queue backs up.
# Use baudrate=921600 (8×) for ~2400+ fps. IFG prevents adapter buffer overrun.
IFG_SECS = 0.0005   # 0.5ms min gap between sends (2000 fps max at higher baud)
SERIAL_BAUDRATE = 921600  # 115200 if adapter unstable; 921600 if supported

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
# Option 5: SLCAN — SLCAN protocol over serial (many USB-CAN adapters)
#   CAN_INTERFACE = "slcan"
#   CAN_AVAILABLE_CONFIGS = can.detect_available_configs("slcan")
#
# Option 6: Virtual — no hardware, for testing
#   CAN_INTERFACE = "virtual"
#   CAN_AVAILABLE_CONFIGS = [{"interface": "virtual", "channel": None}]
#
# Option 7: gs_usb — candleLight, Geschwister Schneider USB-CAN
#   CAN_INTERFACE = "gs_usb"
#   CAN_AVAILABLE_CONFIGS = can.detect_available_configs("gs_usb")

import tkinter as tk
from tkinter import font as tkfont
from datetime import datetime

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
    # Thumbwheel: Tastencode=0x06 (Up__Down_ThumbWheel), Eventcode encodes direction
    # Eventcode 1 = W_1_Tick_up,  Eventcode 15 (0x0F) = W_1_Tick_dn  (DBC VAL_)
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

# ─────────────────────────────────────────────────────────────────────────────
# BUS MANAGER
# ─────────────────────────────────────────────────────────────────────────────

class BusManager:
    def __init__(self, log_cb, status_cb):
        self._log    = log_cb
        self._status = status_cb
        self._bus    = None
        self._stop   = threading.Event()

        # Shared send queues.
        # _prio_q: urgent frames (MFL button presses) — checked first by writer
        # _tx_q:   normal periodic ECU frames
        self._prio_q = queue.Queue()
        self._tx_q   = queue.Queue()  # unbounded

        self._main_thread   = None   # connection thread
        self._writer_thread = None   # serial writer
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
            ecu.attach(self._bus, self._tx_q)

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
        if self._tick_thread:
            self._tick_thread.join(timeout=2)

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

        # Attach ECU modules — apply ignition state first, then start threads
        for e in self._ecus:
            if hasattr(e, 'set_enabled'):
                e.set_enabled(self.ignition)
            else:
                e.enabled = self.ignition
            e.attach(self._bus, self._tx_q)

        try:
            self._stop.wait()  # block until stop() — tick loop runs independently
        finally:
            for e in self._ecus:
                e.detach()
            self._bus.shutdown()
            self.connected = False
            self._status("off", "")

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
                    self._bus.send(can.Message(
                        arbitration_id=arb_id,
                        data=data,
                        is_extended_id=arb_id > 0x7FF
                    ))
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
                # 0x3C0 Klemmen_Status_01
                if self.connected:
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
        press = [btn["code"], 0x00, btn["event"], 0x21]
        self._prio_q.put((0x5BF, press))
        self._log(f"MFL {btn['label']:8s} → {' '.join(f'{b:02X}' for b in press)}")
        time.sleep(0.120)   # hold time before clear
        self._prio_q.put((0x5BF, MFL_CLEAR))


# ─────────────────────────────────────────────────────────────────────────────
# MODULE LOADER
# ─────────────────────────────────────────────────────────────────────────────

def load_modules(log_cb=None):
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
        except Exception as e:
            print(f"[WARN] {fname}: {e}")
            continue
        for attr in dir(mod):
            cls = getattr(mod, attr)
            try:
                if (isinstance(cls, type) and
                        issubclass(cls, ECUModule) and cls is not ECUModule):
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


def _hex(data): return " ".join(f"{b:02X}" for b in data[:8])


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
            dl = tk.Label(row, text=_hex(s.data), font=fd, bg=bg, fg=C["text"],
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
            dl.configure(text=_hex(s.data))
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


# ─────────────────────────────────────────────────────────────────────────────
# MAIN APP
# ─────────────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    REFRESH = 200

    def __init__(self):
        super().__init__()
        self.title("CAN Nav Controller"); self.configure(bg=C["bg"])
        self.minsize(920, 580); self.resizable(True, True)
        self.protocol("WM_DELETE_WINDOW", self._close)
        self._ign = False; self._cards = []
        self._setup_bus(); self._build(); self._start_refresh()

    def _setup_bus(self):
        self._mgr = BusManager(
            log_cb=    lambda m: self.after(0, self._log, m),
            status_cb= lambda s, d: self.after(0, self._setstatus, s, d),
        )
        for ecu in load_modules(log_cb=lambda m: self.after(0, self._log, m)):
            self._mgr.register_ecu(ecu)
        self._mgr.start()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build(self):
        self.columnconfigure(0, minsize=285, weight=0)
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

        # Left panel as scrollable canvas — no clipping
        left_outer = tk.Frame(self, bg=C["bg"])
        left_outer.grid(row=2,column=0,sticky="nsew",padx=(10,5),pady=10)
        left_outer.columnconfigure(0,weight=1); left_outer.rowconfigure(0,weight=1)
        left_cvs = tk.Canvas(left_outer, bg=C["bg"], highlightthickness=0, width=300)
        left_cvs.grid(row=0,column=0,sticky="nsew")
        left_sb = tk.Scrollbar(left_outer, orient="vertical", command=left_cvs.yview,
                               bg=C["panel"], troughcolor=C["bg"], width=8)
        left_sb.grid(row=0,column=1,sticky="ns")
        left_cvs.configure(yscrollcommand=left_sb.set)
        left = tk.Frame(left_cvs, bg=C["bg"])
        left_win = left_cvs.create_window((0,0), window=left, anchor="nw")
        left_cvs.bind("<Configure>", lambda e: left_cvs.itemconfig(left_win, width=e.width))
        left.bind("<Configure>", lambda e: left_cvs.configure(scrollregion=left_cvs.bbox("all")))
        left.columnconfigure(0, weight=1)
        left_cvs.bind("<Shift-MouseWheel>",
                      lambda e: left_cvs.yview_scroll(int(-e.delta/120), "units"))
        self._build_ign(left); self._build_mfl(left); self._build_stalk(left); self._build_log(left)

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
        pnl = _panel(parent, "IGNITION"); pnl.pack(fill="x", pady=(0,6))
        self._ign_btn = tk.Button(pnl, text="OFF",
            font=tkfont.Font(family="Segoe UI",size=14,weight="bold"),
            width=9, bg=C["btn"], fg=C["off"], activebackground=C["btn_hov"],
            activeforeground=C["text"], relief="flat", bd=0, cursor="hand2",
            command=self._toggle_ign)
        self._ign_btn.pack(padx=20, pady=(8,4))
        self._ign_hint = tk.Label(pnl, text="Click to enable ignition",
            font=tkfont.Font(family="Segoe UI",size=8), bg=C["panel"], fg=C["sub"])
        self._ign_hint.pack(pady=(0,6))
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

    def _build_mfl(self, parent):
        pnl = _panel(parent, "MFL STEERING WHEEL  (0x5BF)")
        pnl.pack(fill="x", pady=(0,6))
        g = tk.Frame(pnl, bg=C["panel"]); g.pack(padx=12, pady=(4,12))
        for c in range(3): g.columnconfigure(c, weight=1, minsize=74)
        self._btns = {}
        for key, row, col in MFL_LAYOUT:
            b = MFLBtn(g, key, cmd=lambda k=key: self._mfl(k))
            b.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
            self._btns[key] = b

    def _build_stalk(self, parent):
        pnl = _panel(parent, "STALK / BLINKERS  (16)")
        pnl.pack(fill="x", pady=(0,6))
        BTNS = [
            ("tip_left",    "tip_left",    "<<", "TIP<"),
            ("blink_left",  "blink_left",  "<",  "LEFT"),
            ("blink_hazard","blink_hazard","!!", "HAZ"),
            ("blink_right", "blink_right", ">",  "RIGHT"),
            ("tip_right",   "tip_right",   ">>", "TIP>"),
        ]
        g = tk.Frame(pnl, bg=C["panel"]); g.pack(padx=10, pady=(4,4), fill="x")
        for c in range(5): g.columnconfigure(c, weight=1, minsize=46)
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
        off_row = tk.Frame(pnl, bg=C["panel"]); off_row.pack(fill="x", padx=10, pady=(0,8))
        off_f = tk.Frame(off_row, bg=C["btn"], cursor="hand2",
                         highlightbackground=C["border"], highlightthickness=1)
        off_f.pack(fill="x")
        off_l = tk.Label(off_f, text="X  OFF",
                         font=tkfont.Font(family="Segoe UI", size=9),
                         bg=C["btn"], fg=C["sub"])
        off_l.pack(pady=4)
        self._stalk_btns["blink_off"] = (off_f, off_l, None)
        for w in (off_f, off_l):
            w.bind("<ButtonPress-1>",   lambda e,b=(off_f,off_l,None): self._stalk_press(b))
            w.bind("<ButtonRelease-1>", lambda e,b=(off_f,off_l,None): self._stalk_release(b,"blink_off"))
            w.bind("<Enter>",  lambda e,b=(off_f,off_l,None): self._stalk_bg(b, C["btn_hov"]))
            w.bind("<Leave>",  lambda e,b=(off_f,off_l,None): self._stalk_bg(b, C["btn"]))
        self._stalk_enabled = False
        self._update_stalk_buttons()

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
        pnl = _panel(parent, "LOG"); pnl.pack(fill="both", expand=True)
        self._log_w = tk.Text(pnl, bg=C["log_bg"], fg=C["log_fg"],
            font=tkfont.Font(family="Consolas",size=8),
            relief="flat", bd=0, state="disabled", wrap="none", height=8)
        self._log_w.pack(fill="both", expand=True, padx=2, pady=2)

    def _build_ecu_cards(self):
        for ecu in self._mgr.get_ecus():
            card = ECUCard(self._ecu_frame, ecu)
            card.pack(fill="x", pady=(0,8)); self._cards.append(card)
        if not self._cards:
            tk.Label(self._ecu_frame, text="No ECU modules found in modules/",
                     font=tkfont.Font(family="Segoe UI",size=9),
                     bg=C["bg"], fg=C["sub"]).pack(pady=20)

    # ── Actions ───────────────────────────────────────────────────────────────

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

    def _mfl(self, key: str):
        self._btns[key].flash()                # always give visual feedback
        if not self._mgr.connected:
            self._log(f"MFL {MFL[key]['label']} (not connected — no TX)")
            return
        self._mgr.queue_mfl(key)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_w.configure(state="normal")
        self._log_w.insert("end", f"[{ts}] {msg}\n")
        self._log_w.see("end")
        if int(self._log_w.index("end-1c").split(".")[0]) > 600:
            self._log_w.delete("1.0", "80.0")
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

    def _close(self): self._mgr.stop(); self.destroy()


if __name__ == "__main__":
    App().mainloop()
