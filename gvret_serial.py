"""
GVRET serial (ESP32RET / SavvyCAN protocol) for CAN over USB-UART.

Ref: SavvyCAN connections/gvretserial.cpp (Collin Kidder).
Compatible with LilyGO T-CAN485 and similar boards running ESP32RET firmware.

API matches LawicelBusAdapter: send(msg), recv(timeout), shutdown().

Throughput: each classic CAN frame uses ~11+ bytes on the wire (F1, cmd, ts, id, meta, data),
so GVRET is inherently more serial-heavy than compact binary adapters (e.g. csscan_serial).
Default UART is 1_000_000 (ESP32RET / common CP210x-CH340). Use tty_baudrate=2_000_000 only when
firmware and the USB bridge are both configured for it.
"""

from __future__ import annotations

import struct
import threading
import time
from collections import deque

try:
    import serial
except ImportError as e:  # pragma: no cover
    raise ImportError("gvret_serial requires pyserial (pip install pyserial)") from e

import can


def _encode_gvret_baud(speed: int, enabled: bool = True, listen_only: bool = False) -> int:
    """32-bit CAN bus word: speed in lower bits + SavvyCAN/GVRET flag bits."""
    v = int(speed) & 0x0FFFFFFF
    v |= 0x80000000
    if enabled:
        v |= 0x40000000
    if listen_only:
        v |= 0x20000000
    return v


class _GVRETRxParser:
    """
    Incremental parser for inbound 0xF1 … frames.
    BUILD_CAN_FRAME (cmd 0): 4B ts + 4B id + 1B (bus<<4|dlc) + dlc data bytes (SavvyCAN gvretserial.cpp).
    """

    IDLE = 0
    GET_CMD = 1
    BUILD_CAN = 2
    SKIP_FIXED = 3

    __slots__ = ("_state", "_skip_left", "_bc_step", "_ts", "_arb", "_dlc", "_data", "_data_i")

    def __init__(self) -> None:
        self._state = self.IDLE
        self._skip_left = 0
        self._bc_step = 0
        self._ts = 0
        self._arb = 0
        self._dlc = 0
        self._data = bytearray()
        self._data_i = 0

    def feed(self, b: int) -> list[can.Message]:
        out: list[can.Message] = []
        c = b & 0xFF

        if self._state == self.SKIP_FIXED:
            self._skip_left -= 1
            if self._skip_left <= 0:
                self._state = self.IDLE
            return out

        if self._state == self.IDLE:
            if c == 0xF1:
                self._state = self.GET_CMD
            return out

        if self._state == self.GET_CMD:
            if c == 0:
                self._state = self.BUILD_CAN
                self._bc_step = 0
                self._ts = 0
                self._arb = 0
                self._dlc = 0
                self._data = bytearray()
                self._data_i = 0
            elif c == 1:
                self._state = self.SKIP_FIXED
                self._skip_left = 4
            elif c in (5, 9):
                self._state = self.IDLE
            elif c == 6:
                self._state = self.SKIP_FIXED
                self._skip_left = 10
            elif c == 7:
                self._state = self.SKIP_FIXED
                self._skip_left = 6
            elif c == 12:
                self._state = self.SKIP_FIXED
                self._skip_left = 1
            elif c == 2:
                self._state = self.SKIP_FIXED
                self._skip_left = 2
            elif c == 13:
                self._state = self.SKIP_FIXED
                self._skip_left = 15
            else:
                self._state = self.IDLE
            return out

        if self._state == self.BUILD_CAN:
            if self._bc_step <= 3:
                self._ts |= c << (8 * self._bc_step)
            elif self._bc_step <= 7:
                self._arb |= c << (8 * (self._bc_step - 4))
            elif self._bc_step == 8:
                self._dlc = min(c & 0x0F, 8)
                self._data = bytearray(self._dlc)
                self._data_i = 0
            elif self._dlc > 0 and self._data_i < self._dlc:
                self._data[self._data_i] = c
                self._data_i += 1
                if self._data_i >= self._dlc:
                    ext = bool(self._arb & (1 << 31))
                    aid = self._arb & 0x7FFFFFFF
                    out.append(
                        can.Message(
                            arbitration_id=aid,
                            data=bytes(self._data),
                            is_extended_id=ext,
                        )
                    )
                    self._state = self.IDLE
                    self._bc_step = 0
                    return out

            self._bc_step += 1
            if self._state == self.BUILD_CAN and self._bc_step == 9 and self._dlc == 0:
                ext = bool(self._arb & (1 << 31))
                aid = self._arb & 0x7FFFFFFF
                out.append(
                    can.Message(
                        arbitration_id=aid,
                        data=b"",
                        is_extended_id=ext,
                    )
                )
                self._state = self.IDLE
                self._bc_step = 0

        return out

    def consume_chunk(self, data: bytes | memoryview) -> list[can.Message]:
        """Parse a UART read in one pass; one RX lock extend per chunk."""
        if not data:
            return []
        mv = data if isinstance(data, memoryview) else memoryview(data)
        out: list[can.Message] = []
        feed = self.feed
        for k in range(len(mv)):
            got = feed(mv[k])
            if got:
                out.extend(got)
        return out


class GVRETBusAdapter:
    """
    ESP32RET / GVRET over serial. Drop-in like LawicelBusAdapter for nav_controller BusManager.
    """

    # Blocking read with a short timeout: on Windows `in_waiting` is often wrong until data is read.
    _READ_TIMEOUT_S = 0.02
    _READ_CHUNK_MAX = 65536

    def __init__(
        self,
        channel: str,
        bitrate: int = 500_000,
        tty_baudrate: int = 1_000_000,
        serial_timeout: float = 0.0,
        esp_boot_delay: float = 0.35,
        gvret_bus: int = 0,
    ):
        self._gvret_bus = gvret_bus & 3
        # timeout=0: dedicated RX thread polls in_waiting; avoids 10–50ms sleeps per empty read.
        self._ser = serial.Serial(
            channel,
            baudrate=tty_baudrate,
            timeout=serial_timeout,
            write_timeout=2.0,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            self._ser.setDTR(False)
            self._ser.setRTS(False)
        except Exception:
            pass

        self._rx_q: deque[can.Message] = deque(maxlen=50_000)
        self._rx_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._parser = _GVRETRxParser()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True, name="GVRET-RX")
        self._reader.start()

        time.sleep(esp_boot_delay)
        self._handshake_and_configure(bitrate)

    def _handshake_and_configure(self, bitrate: int) -> None:
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        c0 = _encode_gvret_baud(bitrate, enabled=True, listen_only=False)
        c1 = 0  # second CAN — unused on single-transceiver boards
        pkt = bytearray([0xF1, 5])
        pkt += struct.pack("<I", c0 & 0xFFFFFFFF)
        pkt += struct.pack("<I", c1 & 0xFFFFFFFF)
        pkt.append(0)
        extra = bytes(
            [
                0xF1,
                0x0C,
                0xF1,
                0x06,
                0xF1,
                0x07,
                0xF1,
                0x01,
                0xF1,
                0x09,
            ]
        )
        with self._tx_lock:
            self._ser.write(bytes([0xE7, 0xE7]))
            self._ser.flush()
        time.sleep(0.05)
        with self._tx_lock:
            self._ser.write(pkt)
            self._ser.write(extra)
            self._ser.flush()

    def _reader_loop(self) -> None:
        ser = self._ser
        consume = self._parser.consume_chunk
        cmax = self._READ_CHUNK_MAX
        to = self._READ_TIMEOUT_S
        try:
            ser.timeout = to
        except Exception:
            pass
        while not self._stop.is_set():
            try:
                chunk = ser.read(cmax)
            except Exception:
                time.sleep(0.02)
                continue
            if not chunk:
                continue
            msgs = consume(chunk)
            if msgs:
                with self._rx_lock:
                    self._rx_q.extend(msgs)

    def send(self, msg: can.Message) -> None:
        arb = int(msg.arbitration_id) & 0x7FFFFFFF
        if getattr(msg, "is_extended_id", False) or arb > 0x7FF:
            arb |= 1 << 31
        data = bytes(msg.data)[:8]
        dlc = len(data)
        buf = bytearray([0xF1, 0x00])
        buf += struct.pack("<I", arb & 0xFFFFFFFF)
        buf.append(self._gvret_bus & 3)
        buf.append(dlc)
        buf.extend(data)
        buf.append(0)
        with self._tx_lock:
            self._ser.write(buf)

    def recv(self, timeout: float | None = None) -> can.Message | None:
        timeout = 0.05 if timeout is None else timeout
        deadline = time.monotonic() + timeout
        spin = 0
        while time.monotonic() < deadline:
            with self._rx_lock:
                if self._rx_q:
                    return self._rx_q.popleft()
            spin += 1
            if spin < 8:
                continue
            time.sleep(0.0004)
        return None

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        self._reader.join(timeout=1.0)
