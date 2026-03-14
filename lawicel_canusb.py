"""
Lawicel CAN-USB / SLCAN protocol driver (standalone).

Serial: 115200 baud (default). CAN bitrate set via S0–S8 (e.g. S6 = 500 kbps).
Protocol: ASCII commands + CR; standard frame 't' + 3 hex ID + 1 digit DLC + hex data;
          extended 'T' + 8 hex ID + 1 digit DLC + hex data.

Ref: https://github.com/fville/CANAnalyzer/blob/master/lawicel_canusb.py
     www.lawicel.com / www.canusb.com

Use with nav_controller: prefer python-can's "slcan" interface (same protocol).
This module is for scripts or when you need the raw protocol without python-can.
"""

import serial
import threading
import time
from collections import deque

CR = 13
BELL = 7

# Serial and CAN defaults for this project
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_CAN_BITRATE = 500000
SERIAL_TIMEOUT = 0.001  # 1 ms


class LawicelError(Exception):
    def __init__(self, msg):
        self.msg = msg
        super().__init__("Lawicel: " + msg)


# CAN bitrate to SLCAN command (S0–S8)
BITRATE_TO_S = {
    10000: "S0",
    20000: "S1",
    50000: "S2",
    100000: "S3",
    250000: "S5",
    500000: "S6",
    800000: "S7",
    1000000: "S8",
}


class LawicelCanUsb:
    """Lawicel CAN-USB over serial (SLCAN protocol)."""

    def __init__(self, device: str, serial_baud: int = DEFAULT_SERIAL_BAUD, timeout: float = SERIAL_TIMEOUT):
        self.ser = serial.Serial(device, serial_baud, timeout=timeout)
        self._rxfifo = deque()

    def close(self):
        try:
            self.close_channel()
        except (LawicelError, OSError):
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _send_command(self, cmd: str | None) -> list:
        R = []
        if cmd:
            self.ser.write(cmd.encode("ascii") + bytes([CR]))
        while True:
            d = self.ser.read(size=1)
            if not d:
                raise LawicelError("serial read timeout")
            ch = d.decode("ascii", errors="replace") if isinstance(d, bytes) else chr(d)
            if ch in ("t", "T"):
                frame_str = self._parse_incoming_frame(ch)
                self._rxfifo.appendleft(frame_str)
                return R
            code = d[0] if isinstance(d, bytes) else (d if isinstance(d, int) else ord(d))
            if code == CR:
                break
            if code == BELL and len(R) == 0:
                raise LawicelError("adapter returned BELL (error)")
            R.append(code)
        return R

    def _parse_incoming_frame(self, firstchar: str) -> str:
        n_id = 8 if firstchar == "T" else 3
        r_packet = firstchar
        s = self.ser.read(size=n_id + 1)
        if len(s) < n_id + 1:
            raise LawicelError("incomplete frame (id+dlc)")
        r_packet += s.decode("ascii", errors="replace") if isinstance(s, bytes) else "".join(chr(x) for x in s)
        n_data = int(r_packet[-1])  # DLC 0..8
        s2 = self.ser.read(size=n_data * 2)
        if len(s2) < n_data * 2:
            raise LawicelError("incomplete frame (data)")
        r_packet += s2.decode("ascii", errors="replace") if isinstance(s2, bytes) else s2
        self.ser.read(size=1)  # trailing CR
        return r_packet

    def set_bitrate(self, bitrate: int) -> None:
        cmd = BITRATE_TO_S.get(bitrate)
        if not cmd:
            raise LawicelError(f"unsupported bitrate {bitrate}; use one of {list(BITRATE_TO_S.keys())}")
        self._send_command(cmd)

    def open_channel(self) -> None:
        self._send_command("O")

    def close_channel(self) -> None:
        self._send_command("C")

    def poll(self) -> None:
        self._send_command(None)

    def transmit_std(self, packet: str) -> None:
        """Send standard frame. packet = 3 hex ID + 1 digit DLC + hex data (e.g. '18180102030405060708')."""
        self._send_command("t" + packet)

    def transmit_ext(self, packet: str) -> None:
        """Send extended frame. packet = 8 hex ID + 1 digit DLC + hex data."""
        self._send_command("T" + packet)

    def get_rxfifo_len(self) -> int:
        return len(self._rxfifo)

    def get_rx_frame(self) -> tuple | None:
        """Return (is_extended, arb_id, data_bytes) or None if fifo empty."""
        if not self._rxfifo:
            return None
        s = self._rxfifo.pop()
        if s[0] == "t":
            arb_id = int(s[1:4], 16)
            n = int(s[4], 16)
            data = bytes(int(s[5 + i * 2 : 5 + i * 2 + 2], 16) for i in range(n))
            return (False, arb_id, data)
        elif s[0] == "T":
            arb_id = int(s[1:9], 16)
            n = int(s[9], 16)
            data = bytes(int(s[10 + i * 2 : 10 + i * 2 + 2], 16) for i in range(n))
            return (True, arb_id, data)
        return None

    def get_version(self) -> bytes:
        return bytes(self._send_command("V"))


def open_lawicel(
    device: str,
    can_bitrate: int = DEFAULT_CAN_BITRATE,
    serial_baud: int = DEFAULT_SERIAL_BAUD,
    serial_timeout: float | None = None,
) -> LawicelCanUsb:
    """Open Lawicel adapter: close channel, set CAN bitrate, open channel."""
    if serial_timeout is None:
        serial_timeout = SERIAL_TIMEOUT
    c = LawicelCanUsb(device, serial_baud=serial_baud, timeout=serial_timeout)
    try:
        c.close_channel()
    except LawicelError as e:
        if "BELL" not in str(e):
            raise
    c.set_bitrate(can_bitrate)
    c.open_channel()
    return c


class LawicelBusAdapter:
    """
    Drop-in replacement for can.Bus using the custom Lawicel driver.
    Use when python-can's slcan gives no traffic (e.g. SLGreen, arduino-canbus-monitor).
    Implements send(msg), recv(timeout), shutdown() so BusManager can use it as self._bus.
    """

    def __init__(
        self,
        channel: str,
        bitrate: int = DEFAULT_CAN_BITRATE,
        tty_baudrate: int = DEFAULT_SERIAL_BAUD,
        serial_timeout: float = 0.05,
    ):
        # Longer serial_timeout so poll() can read a full frame when adapter pushes data
        self._lawicel = open_lawicel(
            channel,
            can_bitrate=bitrate,
            serial_baud=tty_baudrate,
            serial_timeout=serial_timeout,
        )
        self._lock = threading.Lock()

    def send(self, msg) -> None:
        """Send a can.Message (same API as can.Bus.send)."""
        try:
            import can
        except ImportError:
            can = None
        arb_id = msg.arbitration_id
        data = msg.data if hasattr(msg.data, "__iter__") else list(msg.data)
        dlc = len(data)
        hex_data = "".join(f"{b:02X}" for b in data[:8])
        with self._lock:
            try:
                if getattr(msg, "is_extended_id", arb_id > 0x7FF):
                    self._lawicel.transmit_ext(f"{arb_id:08X}{dlc}{hex_data}")
                else:
                    self._lawicel.transmit_std(f"{arb_id:03X}{dlc}{hex_data}")
            except LawicelError as e:
                if can is not None:
                    raise can.CanError(str(e)) from e
                raise

    def recv(self, timeout: float | None = None):
        """Return a can.Message or None (same API as can.Bus.recv)."""
        try:
            import can
        except ImportError:
            can = None
        timeout = 0.05 if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self._lawicel.poll()
            except LawicelError as e:
                if "timeout" not in str(e).lower():
                    if can is not None:
                        raise can.CanError(str(e)) from e
                    raise
            frame = self._lawicel.get_rx_frame()
            if frame:
                is_ext, arb_id, data = frame
                if can is not None:
                    return can.Message(
                        arbitration_id=arb_id,
                        data=bytearray(data),
                        is_extended_id=is_ext,
                    )
                return (arb_id, data, is_ext)
            time.sleep(0.001)
        return None

    def shutdown(self) -> None:
        try:
            self._lawicel.close()
        except Exception:
            pass


if __name__ == "__main__":
    import sys
    dev = "COM5" if sys.platform == "win32" else "/dev/ttyUSB0"
    with open_lawicel(dev, can_bitrate=500000, serial_baud=115200) as c:
        print("Version:", c.get_version())
        c.transmit_std("18180102030405060708")
        time.sleep(0.01)
        c.poll()
        print("RX fifo:", c.get_rxfifo_len())
