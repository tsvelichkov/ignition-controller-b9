#!/usr/bin/env python3
"""
Send SLCAN init commands (C, S, O) to an Arduino + MCP2515 or other SLCAN adapter.
Run this once before opening SavvyCAN (or any app that doesn't send init): the
adapter stays in "open" state so you don't have to type S and O manually.

  python slcan_init.py COM10 50000
  python slcan_init.py /dev/ttyUSB0 500000

Bitrate codes: S0=10k S1=20k S2=50k S3=100k S5=250k S6=500k S8=1M
"""

import sys
import serial

CR = b"\r"
SLCAN_BITRATE = {
    10000: "S0",
    20000: "S1",
    50000: "S2",
    100000: "S3",
    250000: "S5",
    500000: "S6",
    800000: "S7",
    1000000: "S8",
}


def main():
    if len(sys.argv) < 2:
        print("Usage: slcan_init.py <PORT> [BITRATE] [BAUD]")
        print("  PORT    e.g. COM10 or /dev/ttyUSB0")
        print("  BITRATE CAN bitrate (default 500000 → S6)")
        print("  BAUD    serial baud (default 115200)")
        print()
        print("Example (50 kbps, like S2):  python slcan_init.py COM10 50000")
        print("Example (500 kbps):         python slcan_init.py COM10 500000")
        sys.exit(1)
    port = sys.argv[1]
    bitrate = int(sys.argv[2]) if len(sys.argv) > 2 else 500000
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    cmd = SLCAN_BITRATE.get(bitrate)
    if not cmd:
        print(f"Unknown bitrate {bitrate}. Use one of: {list(SLCAN_BITRATE.keys())}")
        sys.exit(1)

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"Open {port} failed: {e}")
        sys.exit(1)

    def send(s: str) -> bytes:
        ser.write(s.encode("ascii") + CR)
        ser.flush()
        return ser.read(64)

    # Close channel (ignore BELL if already closed), set bitrate, open
    send("C")
    r = send(cmd)
    if b"\x07" in r:
        print(f"Warning: adapter returned error for {cmd}")
    r = send("O")
    if b"\x07" in r:
        print("Warning: adapter returned error for O")
        ser.close()
        sys.exit(1)
    ser.close()
    print(f"Init done: {cmd} (bitrate {bitrate}), O (open). You can now connect SavvyCAN to {port}.")


if __name__ == "__main__":
    main()
