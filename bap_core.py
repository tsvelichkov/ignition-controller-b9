from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable
import re
import time


BAP_OP_GET = 1
BAP_OP_SETGET = 2
BAP_OP_HEARTBEAT_STATUS = 3
BAP_OP_STATUS = 4
BAP_OP_STATUS_ACK = 5
BAP_OP_ACK = 6
BAP_OP_ERROR = 7
BAP_OP_RESET = 0

BAP_OPCODE_NAMES = {
    BAP_OP_GET: "Get",
    BAP_OP_SETGET: "SetGet",
    BAP_OP_HEARTBEAT_STATUS: "HeartbeatStatus",
    BAP_OP_STATUS: "Status",
    BAP_OP_STATUS_ACK: "StatusAck",
    BAP_OP_ACK: "Ack",
    BAP_OP_ERROR: "Error",
    BAP_OP_RESET: "Reset",
}

BUSMASTER_LINE_RE = re.compile(
    r"^(?P<timestamp>\d{2}:\d{2}:\d{2}:\d{3})\s+"
    r"(?P<direction>[TtRr][xX])\s+"
    r"(?P<channel>\d+)\s+"
    r"(?P<canid>0x[0-9A-Fa-f]+)\s+"
    r"(?P<frame_type>\S+)\s+"
    r"(?P<dlc>\d+)\s+"
    r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)\s*$"
)


def build_bap_header(opcode: int, lsg_id: int, fct_id: int) -> list[int]:
    return [((opcode & 0x07) << 4) | ((lsg_id >> 2) & 0x0F), ((lsg_id & 0x03) << 6) | (fct_id & 0x3F)]


def decode_bap_header(header: int) -> tuple[int, int, int]:
    return (header >> 12) & 0x07, (header >> 6) & 0x3F, header & 0x3F


def build_bap_frames(
    opcode: int,
    lsg_id: int,
    fct_id: int,
    payload=None,
    logical_channel: int = 0,
    force_long: bool = False,
) -> list[list[int]]:
    payload = list(payload or [])
    logical_channel &= 0x03
    header = build_bap_header(opcode, lsg_id, fct_id)
    if len(payload) <= 6 and not force_long:
        return [header + payload]

    total_len = len(payload) & 0x0FFF
    start_word = 0x8000 | (logical_channel << 12) | total_len
    frames = [[(start_word >> 8) & 0xFF, start_word & 0xFF] + header + payload[:4]]
    payload = payload[4:]
    continuation = 0
    continuation_base = 0xC0 | (logical_channel << 4)
    while payload:
        frames.append([continuation_base | (continuation & 0x0F)] + payload[:7])
        payload = payload[7:]
        continuation += 1
    return frames


def opcode_label(opcode: int) -> str:
    name = BAP_OPCODE_NAMES.get(opcode, "Unknown")
    return f"{opcode} {name}"


def _resolve_name(value: int, generic_map: dict[int, str], protocol_map: dict[int, str] | None = None) -> str | None:
    if protocol_map:
        name = protocol_map.get(value)
        if name:
            return name
    return generic_map.get(value)


def lsg_label(lsg_id: int, generic_lsg_names: dict[int, str], protocol_lsg_names: dict[int, str] | None = None) -> str:
    name = _resolve_name(lsg_id, generic_lsg_names, protocol_lsg_names)
    return f"0x{lsg_id:02X} {name}" if name else f"0x{lsg_id:02X}"


def function_label(
    fct_id: int,
    generic_function_names: dict[int, str],
    protocol_function_names: dict[int, str] | None = None,
) -> str:
    name = _resolve_name(fct_id, generic_function_names, protocol_function_names)
    return f"0x{fct_id:02X} {name}" if name else f"0x{fct_id:02X}"


def hex_data(data: list[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


@dataclass(slots=True, frozen=True)
class BapProtocol:
    name: str
    lsg_names: dict[int, str]
    function_names: dict[int, str]
    describe_message: Callable[[int, int, int, int, list[int]], str]
    generic_lsg_names: dict[int, str]
    generic_function_names: dict[int, str]

    def lsg_label(self, lsg_id: int) -> str:
        return lsg_label(lsg_id, self.generic_lsg_names, self.lsg_names)

    def function_label(self, fct_id: int) -> str:
        return function_label(fct_id, self.generic_function_names, self.function_names)


@dataclass(slots=True)
class BapMessage:
    direction: str
    timestamp: str
    can_id: int
    logical_channel: int
    opcode: int
    lsg_id: int
    fct_id: int
    payload: tuple[int, ...]
    text: str
    protocol: BapProtocol

    @property
    def can_id_label(self) -> str:
        return f"0x{self.can_id:X}"

    @property
    def opcode_text(self) -> str:
        return opcode_label(self.opcode)

    @property
    def lsg_text(self) -> str:
        return self.protocol.lsg_label(self.lsg_id)

    @property
    def fct_text(self) -> str:
        return self.protocol.function_label(self.fct_id)

    @property
    def data_text(self) -> str:
        return hex_data(list(self.payload))


@dataclass(slots=True)
class _PendingBap:
    direction: str
    timestamp: str
    can_id: int
    logical_channel: int
    header: int
    payload_len: int
    payload: list[int]


class BapReassembler:
    def __init__(self, protocol: BapProtocol):
        self._protocol = protocol
        self._pending: dict[tuple[str, int, int], _PendingBap] = {}

    def feed(self, direction: str, timestamp: str, can_id: int, data: list[int]) -> BapMessage | None:
        if len(data) < 2:
            return None

        first_word = (data[0] << 8) | data[1]
        if first_word & 0xC000 == 0x8000 and len(data) >= 4:
            logical_channel = (first_word >> 12) & 0x03
            key = (direction, can_id, logical_channel)
            pending = _PendingBap(
                direction=direction,
                timestamp=timestamp,
                can_id=can_id,
                logical_channel=logical_channel,
                header=(data[2] << 8) | data[3],
                payload_len=first_word & 0x0FFF,
                payload=list(data[4:]),
            )
            self._pending[key] = pending
            return self._finalize_if_complete(key)

        if data[0] & 0xC0 == 0xC0:
            logical_channel = (data[0] >> 4) & 0x03
            key = (direction, can_id, logical_channel)
            pending = self._pending.get(key)
            if pending is None:
                return None
            pending.payload.extend(data[1:])
            return self._finalize_if_complete(key)

        header = first_word
        opcode, lsg_id, fct_id = decode_bap_header(header)
        payload = list(data[2:])
        return BapMessage(
            direction=direction.lower(),
            timestamp=timestamp,
            can_id=can_id,
            logical_channel=0,
            opcode=opcode,
            lsg_id=lsg_id,
            fct_id=fct_id,
            payload=tuple(payload),
            text=self._protocol.describe_message(can_id, opcode, lsg_id, fct_id, payload),
            protocol=self._protocol,
        )

    def _finalize_if_complete(self, key: tuple[str, int, int]) -> BapMessage | None:
        pending = self._pending.get(key)
        if pending is None or len(pending.payload) < pending.payload_len:
            return None
        payload = pending.payload[:pending.payload_len]
        opcode, lsg_id, fct_id = decode_bap_header(pending.header)
        message = BapMessage(
            direction=pending.direction.lower(),
            timestamp=pending.timestamp,
            can_id=pending.can_id,
            logical_channel=pending.logical_channel,
            opcode=opcode,
            lsg_id=lsg_id,
            fct_id=fct_id,
            payload=tuple(payload),
            text=self._protocol.describe_message(pending.can_id, opcode, lsg_id, fct_id, payload),
            protocol=self._protocol,
        )
        del self._pending[key]
        return message


def format_elapsed_timestamp(elapsed_seconds: float) -> str:
    total_ms = max(0, int(round(elapsed_seconds * 1000)))
    hours, rem = divmod(total_ms, 3600000)
    minutes, rem = divmod(rem, 60000)
    seconds, millis = divmod(rem, 1000)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}:{millis:03d}"


def busmaster_header(session_label: str) -> str:
    started = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return "\n".join(
        [
            "***BUSMASTER Ver 3.2.0***",
            "***PROTOCOL CAN***",
            "***NOTE: PLEASE DO NOT EDIT THIS DOCUMENT***",
            "***[START LOGGING SESSION]***",
            f"***START DATE AND TIME {started}***",
            "***HEX***",
            "***SYSTEM MODE***",
            "***START CHANNEL BAUD RATE***",
            f"***CHANNEL 1 - {session_label}***",
            "***END CHANNEL BAUD RATE***",
            "***START DATABASE FILES***",
            "***END OF DATABASE FILES***",
            "***<Time><Tx/Rx><Channel><CAN ID><Type><DLC><DataBytes>***",
            "",
        ]
    )


def busmaster_footer() -> str:
    return "***[STOP LOGGING SESSION]***\n"


def format_busmaster_line(timestamp: str, direction: str, can_id: int, data: list[int], channel: int = 1) -> str:
    data_hex = hex_data(data)
    suffix = f" {data_hex}" if data_hex else ""
    return f"{timestamp} {direction.title()} {channel} 0x{can_id:X} x {len(data)}{suffix} "


def parse_busmaster_line(line: str):
    match = BUSMASTER_LINE_RE.match(line.strip())
    if not match:
        return None
    data = [int(token, 16) for token in match.group("data").split()] if match.group("data").strip() else []
    return {
        "timestamp": match.group("timestamp"),
        "direction": match.group("direction").lower(),
        "can_id": int(match.group("canid"), 16),
        "channel": int(match.group("channel")),
        "data": data,
    }


class BapLogSession:
    def __init__(self, root_directory: str, prefix: str, session_label: str, trace_ids: set[int], reassembler: BapReassembler):
        self._root = Path(root_directory)
        self._root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self._root / f"{prefix}_{stamp}.log"
        self._start_monotonic = time.monotonic()
        self._trace_ids = set(trace_ids)
        self._reassembler = reassembler
        self.messages: list[BapMessage] = []
        self._fh = self.path.open("w", encoding="ascii", buffering=1)
        self._fh.write(busmaster_header(session_label))

    def handle_frame(self, direction: str, can_id: int, data: list[int]) -> BapMessage | None:
        if can_id not in self._trace_ids:
            return None
        timestamp = format_elapsed_timestamp(time.monotonic() - self._start_monotonic)
        self._fh.write(format_busmaster_line(timestamp, direction, can_id, list(data)) + "\n")
        message = self._reassembler.feed(direction.lower(), timestamp, can_id, list(data))
        if message is not None:
            self.messages.append(message)
        return message

    def close(self):
        if getattr(self, "_fh", None):
            self._fh.write(busmaster_footer())
            self._fh.close()
            self._fh = None
