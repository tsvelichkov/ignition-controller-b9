"""
modules/42_door_driver.py — Driver door (Fahrertür) module 42 emulation

BCM-E BAP only (29-bit, 0x17332A10 — node 0x2A, channel 0x10):
  fn=0x82  STATUS  — node descriptor  (node=0x2A, 3 FGs, ver=01)
  fn=0x78  SET MF  — keepalive/heartbeat config
  fn=0x84  STATUS  — BAP protocol version 0x0A
  fn=0x8F  STATUS  — FG status
  fn=0x90  SET MF  — subscribe / cycle config
  fn=0x91  STATUS  — FG 0x11 value
  fn=0x92  STATUS  — FG 0x12 value
  fn=0x93  STATUS  — FG 0x13 value

  BAP cycle: 8 entries × ~4ms stagger, full period ~57ms.
  Multi-frame entries send header + continuation back-to-back.

NOTE: TSG_FT_02 (0x3E5) is handled by 19_gateway.py as a 16-frame replay.
      Do NOT add 0x3E5 here — dual-send causes counter to stay at 0.

Source: 0000046.TXT + 0000050.TXT car logs.
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecu_base import ECUModule


_BAP_ID = 0x17332A10   # 29-bit, node 0x2A (42), channel 0x10


class DriverDoorECU(ECUModule):
    ECU_ID   = "42"
    ECU_NAME = "Door Driver"

    MESSAGES = []   # no standard CAN messages — TSG_FT_02 is in 19_gateway.py

    _BAP_SEQ = [
        # fn=0x82  STATUS  node descriptor: node=0x2A, 3 FGs, proto ver=0x01
        ([0x3A, 0x82, 0x03, 0x00, 0x2A, 0x00, 0x03, 0x01],),

        # fn=0x78  SET MF  keepalive/heartbeat config  (header + cont)
        ([0x80, 0x08, 0x3A, 0x83, 0x78, 0x01, 0xF0, 0x00],
         [0xC0, 0x00, 0x00, 0x00, 0x00]),

        # fn=0x84  STATUS  BAP protocol version 0x0A
        ([0x3A, 0x84, 0x0A],),

        # fn=0x8F  STATUS  FG status
        ([0x3A, 0x8F, 0x00, 0x00],),

        # fn=0x90  SET MF  subscribe / cycle config  (header + cont)
        ([0x80, 0x05, 0x3A, 0x90, 0x00, 0x00, 0x00, 0x00],
         [0xC0, 0x00]),

        # fn=0x91  STATUS
        ([0x3A, 0x91, 0x00, 0x00, 0x00],),

        # fn=0x92  STATUS
        ([0x3A, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00],),

        # fn=0x93  STATUS
        ([0x3A, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],),
    ]
    _BAP_STEP_MS  = 4.0
    _BAP_CYCLE_MS = 57.0

    def _run(self):
        bap_step      = 0
        bap_cycle_due = time.monotonic()
        bap_step_due  = bap_cycle_due

        while not self._stop_evt.is_set():
            now = time.monotonic()

            if now >= bap_step_due:
                for frame in self._BAP_SEQ[bap_step]:
                    self._enqueue(_BAP_ID, frame)

                bap_step += 1
                if bap_step >= len(self._BAP_SEQ):
                    bap_step      = 0
                    bap_cycle_due += self._BAP_CYCLE_MS / 1000.0
                    bap_step_due   = bap_cycle_due
                else:
                    bap_step_due  += self._BAP_STEP_MS / 1000.0

            self._stop_evt.wait(timeout=max(0.001, bap_step_due - time.monotonic()))
