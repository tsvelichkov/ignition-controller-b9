"""
modules/46_bcm2.py — BCM2 module 46 emulation

Sends:
  0x644  — BCM2 / fuel level (HUD display),  ~500ms
  0x643  Einheiten_01 — KBI units (date, pressure, temp, etc.), 1000ms

Fuel level encoding (0x644) — resistance-based, two sensors:
  Higher resistance = lower fuel. 50–60 Ω = full tank.

  Sensor 1 (byte0 + byte1 bits 0–3):
    raw = byte0 + (byte1 & 0x0F) * 256   (0–4095)
    byte1 low nibble: 0x1→256, 0x2→512, etc.

  Sensor 2 (byte1 bits 4–7 + byte2):
    raw = (byte1 >> 4) * 16 + byte2   (e.g. 0x1x 0x01 = 17)
    byte2: 0x01→16Ω, 0x02→32Ω, 0x10→255Ω

  DBC: no raw 0x644 definition; Kombi_02/03 have KBI_Inhalt_Tank, KBI_Tankfuellstand_Prozent.
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule

class Bcm2ECU(ECUModule):
    ECU_ID   = "46"
    ECU_NAME = "BCM2"

    MESSAGES = [
        (0x644, "BCM2_01",  500, [0x3B, 0x30, 0x04, 0x00, 0x00, 0x00, 0xAA, 0xDD]),  # 100% full
        (0x643, "Einheiten_01", 1000, [0x00, 0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]),
    ]

    def set_fuel_percent(self, percent: int):
        """Set fuel level for 0x644. percent 0–100. display = (byte0+byte1)/107*100. 3b 30 04 = full."""
        percent = max(0, min(100, percent))
        raw_sum = int(percent * 107 / 100)
        b0, b1 = raw_sum // 2, raw_sum - raw_sum // 2
        self.set_fuel_raw(b0, b1)

    def set_fuel_raw(self, b0: int, b1: int, b2: int = 0x04):
        """Set 0x644 fuel bytes directly. b0, b1, b2 = 0–255."""
        b0 = max(0, min(255, b0))
        b1 = max(0, min(255, b1))
        b2 = max(0, min(255, b2))
        data = [b0, b1, b2, 0x00, 0x00, 0x00, 0xAA, 0xDD]
        self.update_data(0x644, data)

    @staticmethod
    def fuel_sensor1_raw(b0: int, b1: int) -> int:
        """Decode sensor 1 raw: byte0 + (byte1 & 0x0F) * 256."""
        return b0 + (b1 & 0x0F) * 256

    @staticmethod
    def fuel_sensor2_raw(b1: int, b2: int) -> int:
        """Decode sensor 2 raw: (byte1 >> 4) * 16 + byte2. 0x1x 0x01 = 17."""
        return (b1 >> 4) * 16 + b2

