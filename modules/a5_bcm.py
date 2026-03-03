"""
modules/a5_bcm.py — VZE (Traffic Sign Recognition / Audi Presence) module A5

Sends:
  0x181  VZE_01 — Traffic sign display (speed limits etc), 100ms
  0x29C  VZE_02 — Traffic sign display 2 (signs 4-5),     100ms

Both are static — no active navigation or camera input on bench.
VZE_01 payload 0x00 0x90 ... signals "no speed limit sign visible" with
sensor present and active. VZE_02 signals no secondary signs.

Source: 0000046.TXT car log.
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule


class VzeECU(ECUModule):
    ECU_ID   = "A5"
    ECU_NAME = "VZE"

    MESSAGES = [
        (0x181, "VZE_01", 100, [0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]),
        (0x29C, "VZE_02", 100, [0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00]),
    ]

    def set_enabled(self, on: bool):
        self.enabled = on
