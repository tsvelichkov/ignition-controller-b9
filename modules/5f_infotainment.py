"""
modules/5f_infotainment.py — Infotainment / MIB3 ECU emulation
HUD + Cluster BAP nav protocol ported from hud_nav.js v23.
"""
import time, threading
from ecu_base import ECUModule
import can
import queue

HUD_S=0x17333210; HUD_D=0x17333211; HUD_ASG=0x17333202; HUD_RX=0x17330410
CLU_S=0x17333110; CLU_D=0x17333111; CLU_RX=0x17330110

# NM keepalives removed — now owned by 19_gateway.py which sends all 5 nodes

HUD_FL=[0x78,0x01,0xDC,0x00,0x06,0x00]
CLU_FL=[0x78,0x01,0x98,0x0A,0x80,0x60,0x42,0x00]

ARROWS=[
    {"name":"NE diag",  "code":0x83,"dist_lo":0x08,"dist_hi":0x07,"type":0x01},
    {"name":"Right",    "code":0x5F,"dist_lo":0xE8,"dist_hi":0x03,"type":0x01},
    {"name":"Straight", "code":0x43,"dist_lo":0xE8,"dist_hi":0x03,"type":0x01},
]

class InfotainmentECU(ECUModule):
    ECU_ID="5F"; ECU_NAME="Infotainment"
    MESSAGES=[
        (HUD_S,   "HUD Primary",   0, [0x3C,0x82,0x00]),
        (HUD_D,   "HUD Secondary", 0, [0x3C,0x9D,0x00]),
        (CLU_S,   "CLU Primary",   0, [0x3C,0x42,0x00]),
        (CLU_D,   "CLU Secondary", 0, [0x3C,0x60,0x00]),
        (0x331,   "MFL Heartbeat", 0, [0x20,0x00,0x00,0x00,0x01,0x00,0x00,0x00]),
    ]

    def __init__(self, log_cb=None):
        super().__init__(log_cb)
        self._rx_thread=None
        self._tick=0; self._mfl_ctr=0
        self._hud_inited=False; self._hud_activ=False
        self._nav_conf=False; self._clu_conf=False
        self._clu_cycle=0; self._clu_step=0
        self.arrow_idx=0

    def attach(self, bus, tx_queue):
        self._bus=bus; self._tx_q=tx_queue; self._stop_evt.clear()
        self._thread=threading.Thread(target=self._run,daemon=True,name="ECU-5F-tx")
        self._rx_thread=threading.Thread(target=self._rx_run,daemon=True,name="ECU-5F-rx")
        self._thread.start(); self._rx_thread.start()

    def detach(self):
        self._stop_evt.set()
        for t in (self._thread, self._rx_thread):
            if t: t.join(timeout=2)
        self._bus=None; self._tx_q=None

    # ── TX tick loop ──────────────────────────────────────────────────────────
    def set_enabled(self, on: bool):
        """Called by BusManager on ignition toggle."""
        if self.enabled and not on:
            # Ignition just turned OFF — send explicit deactivation to cluster
            self._log("Ignition OFF — deactivating cluster/HUD nav")
            self._tx(CLU_S, [0x3C, 0x69, 0x00])          # nav enable off
            self._tx(CLU_S, [0x3C, 0x5E, 0x00])          # nav type off
            self._tx(HUD_S, [0x3C, 0x91, 0x00])          # HUD deactivate
            self._tx(HUD_S, [0x3C, 0x95, 0x00,0x00,0x00,0x00,0xFF,0x00])
            # Reset confirmed state so re-activation works on next ignition ON
            self._nav_conf   = False
            self._clu_conf   = False
            self._hud_inited = False
            self._hud_activ  = False
            self._clu_step   = 0
            self._clu_cycle  = 0
        self.enabled = on

    def _run(self):
        last=time.monotonic()
        while not self._stop_evt.is_set():
            now=time.monotonic()
            if now-last>=0.100:
                last=now
                if self.enabled:
                    self._tick_100ms()
                else:
                    # Ignition off — keep MFL heartbeat alive so bus
                    # nodes stay awake and see the clean KL15=off state
                    # (NM keepalives handled by 19_gateway)
                    self._tick += 1
                    t = self._tick
                    if t % 4 == 1:
                        self._mfl_ctr = (self._mfl_ctr + 1) & 0x0F
                        self._tx(0x331, [0x20|self._mfl_ctr,0x00,0x00,0x00,0x01,0x00,0x00,0x00])
            self._stop_evt.wait(timeout=0.005)

    def _tick_100ms(self):
        t=self._tick; self._tick+=1

        # MFL_01 heartbeat every 320ms
        if t%4==1:
            self._mfl_ctr=(self._mfl_ctr+1)&0x0F
            d=[0x20|self._mfl_ctr,0x00,0x00,0x00,0x01,0x00,0x00,0x00]
            self._tx(0x331,d); self._mark_sent(0x331,d)

        # NM keepalives handled by 19_gateway — removed from here

        # HUD heartbeat every 1s
        if t%10==0:
            if not self._hud_inited: self._hud_init()
            self._tx(HUD_S,[0x3C,0xA7,0x04])
            self._mark_sent(HUD_S,[0x3C,0xA7,0x04])

        # Activate HUD nav at t=15s
        if t==150 and not self._hud_activ:
            self._hud_activate()

        # Keep pushing nav active until confirmed
        if self._hud_activ and not self._nav_conf and t%10==5:
            self._tx(HUD_S,[0x4C,0x91,0x01])
            self._tx(HUD_S,[0x4C,0x95,0xF6,0x0E,0x00,0x00,0x01,0x01])

        # Cluster cycle — one step per second at offset tick 3
        if t%10==3 and t>0:
            self._clu_tick()

        # Refresh HUD nav text every 10s
        if self._nav_conf and t%100==0:
            self._tx_multi(HUD_D,[0x3C,0x93,0x07,0x6F,0x66,0x66,0x72,0x6F,0x61,0x64])

        # Refresh cluster arrow every 10s
        if self._clu_conf and t%100==5:
            a=ARROWS[self.arrow_idx]
            self._tx(CLU_D,[0x80,0x05,0x3C,0x60,a["code"],a["dist_lo"],a["dist_hi"],a["type"]])
            self._tx(CLU_D,[0xC0,0x00])

    # ── Cluster 12s cycle ─────────────────────────────────────────────────────
    def _clu_tick(self):
        steps=[
            lambda: self._tx(CLU_S,[0x3C,0x4F,0x00,0x00]),
            lambda: self._tx(CLU_S,[0x3C,0x53,0x00]),
            lambda: self._tx(CLU_S,[0x3C,0x54,0x00,0x00]),
            lambda: self._tx(CLU_S,[0x80,0x02,0x3C,0x5C,0x00,0x00]),
            lambda: self._tx(CLU_S,[0x3C,0x5E,0x09 if self._clu_cycle==0 else 0x00]),
            lambda: (self._tx(CLU_S,[0x3C,0x69,0x06]),
                     self._mark_sent(CLU_S,[0x3C,0x69,0x06])),
            lambda: (self._tx(CLU_S,[0x80,0x08,0x3C,0x6A,0x00,0x00,0x00,0x00]),
                     self._tx(CLU_S,[0xC0,0x00,0x00,0x00,0x00])),
            lambda: self._tx(CLU_S,[0x3C,0x71,0x00,0x00,0x00,0x00]),
            lambda: (self._tx(CLU_S,[0x3C,0x42,0x03,0x00,0x31,0x00,0x04,0x08]),
                     self._mark_sent(CLU_S,[0x3C,0x42,0x03,0x00,0x31,0x00,0x04,0x08])),
            lambda: (self._tx(CLU_S,[0x80,0x08,0x3C,0x43]+CLU_FL[:4]),
                     self._tx(CLU_S,[0xC0]+CLU_FL[4:])),
            lambda: self._tx(CLU_S,[0x3C,0x44,0x0A]),
        ]
        if self._clu_step<len(steps):
            steps[self._clu_step](); self._clu_step+=1
        else:
            self._clu_step=0; self._clu_cycle+=1

    # ── RX loop ───────────────────────────────────────────────────────────────
    def _rx_run(self):
        while not self._stop_evt.is_set():
            if not self._bus:
                time.sleep(0.1); continue
            try:
                msg=self._bus.recv(timeout=0.1)
                if msg: self._handle_rx(msg)
            except Exception: pass

    def _handle_rx(self, msg):
        a=msg.arbitration_id; d=list(msg.data)
        if not d: return

        if a==HUD_ASG:
            if d[0]==0x1C and d[1]==0x82:
                if not self._hud_inited: self._hud_init()
                self._tx(HUD_S,[0x4C,0x82,0x03,0x00,0x32,0x00,0x04,0x08])
                if self._nav_conf: self._tx(HUD_S,[0x4C,0x8F,0x01])
            elif d[0]==0x1C and d[1]==0x81:
                if not self._hud_inited: self._hud_init()
                self._hud_all_props(0x4C)
            return

        if a==HUD_RX and len(d)>=3 and d[0]==0x41 and d[1]==0x0F and d[2]==0x02:
            if not self._nav_conf:
                self._nav_conf=True; self._log("HUD NAV CONFIRMED")
                self._send_arrows()
            return

        if a==CLU_RX and len(d)>=2 and d[0]==0x30:
            self._handle_clu_get(d[1]); return

        if a==CLU_RX and len(d)>=3 and d[0]==0x40 and d[1]==0x4F and d[2]==0x02:
            if not self._clu_conf:
                self._clu_conf=True; self._log("CLUSTER NAV CONFIRMED")
                self._send_arrows()
            return

    def _handle_clu_get(self, fct):
        if   fct==0x43: self._tx(CLU_S,[0x80,0x08,0x40,0x43]+CLU_FL[:4]); self._tx(CLU_S,[0xC0]+CLU_FL[4:])
        elif fct==0x44: self._tx(CLU_S,[0x40,0x44,0x0A])
        elif fct==0x42: self._tx(CLU_S,[0x40,0x42,0x03,0x00,0x31,0x00,0x04,0x08])
        elif fct==0x4F: self._tx(CLU_S,[0x40,0x4F,0x00,0x00])
        elif fct==0x5E: self._tx(CLU_S,[0x40,0x5E,0x00])
        elif fct==0x69: self._tx(CLU_S,[0x40,0x69,0x06])
        elif fct==0x6A: self._tx(CLU_S,[0x80,0x08,0x40,0x6A,0x00,0x00,0x00,0x00]); self._tx(CLU_S,[0xC0,0x00,0x00,0x00,0x00])
        elif fct==0x70: self._tx(CLU_S,[0x40,0x70,0x00,0x00,0x00,0x00])

    # ── HUD helpers ───────────────────────────────────────────────────────────
    def _hud_init(self):
        if self._hud_inited: return
        self._hud_inited=True; self._log("HUD init")
        self._hud_all_props(0x3C)
        self._tx(HUD_S,[0x4C,0x82,0x03,0x00,0x32,0x00,0x04,0x08])

    def _hud_activate(self):
        self._hud_activ=True; self._log("HUD activate")
        self._tx_multi(HUD_S,[0x4C,0xA5,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00])
        self._tx(HUD_S,[0x4C,0x91,0x01])
        self._tx_multi(HUD_S,[0x4C,0xA5,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        self._tx(HUD_S,[0x4C,0x95,0xF6,0x0E,0x00,0x00,0x01,0x01])

    def _hud_all_props(self, op):
        f=HUD_FL
        self._tx(HUD_S,[op,0x82,0x03,0x00,0x32,0x00,0x04,0x08])
        self._tx_multi(HUD_S,[op,0x83]+f+[0x00,0x00])
        self._tx(HUD_S,[op,0x84,0x0A])
        self._tx(HUD_S,[op,0x8F,0x00])
        self._tx(HUD_S,[op,0x90,0x01,0x4E,0x01])
        self._tx(HUD_S,[op,0x91,0x01 if self._hud_activ else 0x00])
        v=[0xF6,0x0E,0x00,0x00,0x01,0x01] if self._hud_activ else [0x00,0x00,0x00,0x00,0xFF,0x00]
        self._tx(HUD_S,[op,0x95]+v)
        self._tx_multi(HUD_S,[op,0xA5]+[0x00]*8)
        self._tx(HUD_S,[op,0xA6,0x00])

    def _send_arrows(self):
        a=ARROWS[self.arrow_idx]; self._log(f"Arrow: {a['name']} (0x{a['code']:02X})")
        self._tx(CLU_D,[0x3C,0x50,0x25,0x00,0x00,0x00,0x00,0x00])
        self._tx(CLU_D,[0x80,0x05,0x3C,0x60,a["code"],a["dist_lo"],a["dist_hi"],a["type"]])
        self._tx(CLU_D,[0xC0,0x00])
        self._mark_sent(CLU_D,[0x3C,0x60,a["code"],a["dist_lo"],a["dist_hi"],a["type"]])
        self._tx(HUD_D,[0x80,0x03,0x3C,0x9D,0x00,0x00,0xFF])
        self._tx(HUD_D,[0xC0,0x00])
        self._tx(HUD_D,[0x90,0x05,0x3C,0xA1,a["code"],0x02,0x02,0x03])
        self._tx(HUD_D,[0xD0,0x00])
        self._mark_sent(HUD_D,[0x3C,0xA1,a["code"],0x02,0x02,0x03])

    def next_arrow(self): self.arrow_idx=(self.arrow_idx+1)%len(ARROWS); self._maybe_send()
    def prev_arrow(self): self.arrow_idx=(self.arrow_idx-1)%len(ARROWS); self._maybe_send()
    def _maybe_send(self):
        if self._nav_conf or self._clu_conf: self._send_arrows()
