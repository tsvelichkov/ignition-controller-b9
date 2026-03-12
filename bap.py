from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import time


BAP_OP_GET = 1
BAP_OP_SETGET = 2
BAP_OP_HEARTBEAT_STATUS = 3
BAP_OP_STATUS = 4
BAP_OP_STATUS_ACK = 5
BAP_OP_ACK = 6
BAP_OP_ERROR = 7

BAP_OPCODE_NAMES = {
    BAP_OP_GET: "Get",
    BAP_OP_SETGET: "SetGet",
    BAP_OP_HEARTBEAT_STATUS: "HeartbeatStatus",
    BAP_OP_STATUS: "Status",
    BAP_OP_STATUS_ACK: "StatusAck",
    BAP_OP_ACK: "Ack",
    BAP_OP_ERROR: "Error",
}

BAP_LSG_CLUSTER_NAV = 0x31
BAP_LSG_HUD_NAV = 0x32

HUD_S = 0x17333210
HUD_D = 0x17333211
HUD_ASG = 0x17333202
HUD_RX = 0x17330410
HUD_GET = 0x17333200
HUD_TRACE_IDS = {HUD_S, HUD_D, HUD_ASG, HUD_RX, HUD_GET}

LSG_NAMES = {
    0x04: "HUD",
    0x07: "TSK_07",
    BAP_LSG_CLUSTER_NAV: "Navigation_Cluster",
    BAP_LSG_HUD_NAV: "Navigation_SD",
}

FUNCTION_NAMES = {
    0x02: "BAP_Config",
    0x03: "FunctionList",
    0x04: "HeartbeatInterval",
    0x0D: "Unknown0D",
    0x0E: "FSGSetup",
    0x0F: "FSGOperationState",
    0x10: "CompassInfo",
    0x11: "RouteGuidanceStatus",
    0x12: "DistanceToNextManeuver",
    0x13: "CurrentPositionInfo",
    0x14: "NavigationSessionState",
    0x15: "DistanceToDestination",
    0x16: "Unknown16",
    0x17: "ManeuverDescriptor",
    0x18: "RouteGuidanceDisplayConfig",
    0x1C: "NavigationDisplayState",
    0x1D: "ManeuverSupplement",
    0x1E: "Unknown1E",
    0x20: "ClusterArrow",
    0x21: "ArrowBearing",
    0x25: "FunctionSynchronisation",
    0x26: "InfoStates",
    0x27: "Unknown27",
    0x29: "Unknown29",
    0x2B: "Unknown2B",
    0x2C: "Unknown2C",
    0x2D: "Unknown2D",
    0x2F: "Unknown2F",
    0x31: "Unknown31",
    0x32: "FeatureEnable",
    0x35: "Unknown35",
    0x36: "Handshake36",
    0x37: "Unknown37",
    0x39: "ManeuverState",
    0x3C: "Unknown3C",
}

MANEUVER_NAMES = {
    0x03: "Arrived",
    0x06: "OffRoad",
    0x09: "OffroadCompass",
    0x0B: "FollowStreet",
    0x0D: "Turn",
}

BUSMASTER_LINE_RE = re.compile(
    r"^(?P<timestamp>\d{2}:\d{2}:\d{2}:\d{3})\s+"
    r"(?P<direction>Tx|Rx)\s+"
    r"(?P<channel>\d+)\s+"
    r"(?P<canid>0x[0-9A-Fa-f]+)\s+"
    r"(?P<frame_type>\S+)\s+"
    r"(?P<dlc>\d+)\s*"
    r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)$"
)


def build_bap_header(opcode: int, lsg_id: int, fct_id: int) -> list[int]:
    return [((opcode & 0x07) << 4) | ((lsg_id >> 2) & 0x0F), ((lsg_id & 0x03) << 6) | (fct_id & 0x3F)]


def decode_bap_header(header: int) -> tuple[int, int, int]:
    return (header >> 12) & 0x07, (header >> 6) & 0x3F, header & 0x3F


def build_bap_frames(opcode: int, lsg_id: int, fct_id: int, payload=None, logical_channel: int = 0,
                     force_long: bool = False) -> list[list[int]]:
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


def lsg_label(lsg_id: int) -> str:
    name = LSG_NAMES.get(lsg_id)
    return f"0x{lsg_id:02X} {name}" if name else f"0x{lsg_id:02X}"


def function_label(fct_id: int) -> str:
    name = FUNCTION_NAMES.get(fct_id)
    return f"0x{fct_id:02X} {name}" if name else f"0x{fct_id:02X}"


def _hex(data: list[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def _payload_u32_le(data: list[int]) -> int:
    value = 0
    for index, byte in enumerate(data[:4]):
        value |= (byte & 0xFF) << (index * 8)
    return value


def _asciiish_text_core(data: list[int]) -> str:
    if not data:
        return ""
    chars = []
    printable = 0
    for byte in data:
        if byte in (0x09, 0x0A, 0x0D):
            chars.append(" ")
            printable += 1
        elif 0x20 <= byte <= 0x7E:
            chars.append(chr(byte))
            printable += 1
        elif byte == 0x00:
            continue
        else:
            chars.append(".")
    if not chars:
        return ""
    text = "".join(chars).strip()
    if not text:
        return ""
    if printable < max(4, len(chars) // 2):
        return ""
    return " ".join(text.split())


def _asciiish_text(data: list[int]) -> str:
    if len(data) >= 2 and data[0] == len(data) - 1:
        prefixed = _asciiish_text_core(data[1:])
        if prefixed:
            return prefixed
    return _asciiish_text_core(data)


# Navigation_SD FunctionList bit positions (bit N = available when set)
NAV_SD_FUNCTIONLIST_BITS = {
    1: "GetAll",
    2: "BAP_Config",
    3: "FunctionList",
    4: "HeartBeat",
    15: "FSG_OperationState",
    16: "CompassInfo",
    17: "RG_Status",
    18: "DistanceToNextManeuver",
    19: "CurrentPositionInfo",
    20: "TurnToInfo",
    21: "DistanceToDestination",
    22: "TimeToDestination",
    23: "ManeuverDescriptor",
    24: "LaneGuidance",
    25: "TMCinfo",
    26: "MagnetFieldZone",
    27: "Calibration",
    28: "ASG_Capabilities",
    29: "LastDest_List",
    30: "FavoriteDest_List",
    31: "PreferredDest_List",
    32: "NavBook",
    33: "Address_List",
    34: "RG_ActDeact",
    35: "RepeatLastNavAnnouncement",
    36: "VoiceGuidance",
    37: "FunctionSynchronisation",
    38: "InfoStates",
    39: "ActiveRgType",
    40: "TrafficBlock_Indication",
    41: "GetNextListPos",
    42: "NbSpeller",
    43: "MapColorAndType",
    44: "MapViewAndOrientation",
    45: "MapScale",
    46: "DestinationInfo",
    47: "Altitude",
    48: "OnlineNavigationState",
    49: "Exitview",
    50: "SemidynamicRouteGuidance",
    51: "POI_Search",
    52: "POI_List",
    53: "FSG_Setup",
    54: "Map_Presentation",
    55: "ManeuverState",
    56: "ETC_Status",
    57: "MapContentSettings",
    58: "VideoStreams",
    59: "RequestSync_Video",
    60: "DistanceToDestinationExtended",
    61: "LaneGuidance2",
    62: "Picture",
}


def _decode_nav_sd_functionlist(payload: list[int]) -> str:
    """Decode Navigation_SD FunctionList bitmask; return available functions (single line for Treeview)."""
    if len(payload) < 8:
        return _hex(payload)
    available = []
    for bit, name in sorted(NAV_SD_FUNCTIONLIST_BITS.items()):
        byte_idx = bit // 8
        bit_idx = bit % 8
        if (payload[byte_idx] >> bit_idx) & 1:
            available.append(name)
    if not available:
        return "FunctionList=(none)"
    return "FunctionList=\n" + "\n".join(available)


def describe_bap_message(can_id: int, opcode: int, lsg_id: int, fct_id: int, payload: list[int]) -> str:
    ascii_text = _asciiish_text(payload)
    if fct_id == 0x03 and payload:
        if lsg_id == BAP_LSG_HUD_NAV:
            return _decode_nav_sd_functionlist(payload)
        return f"FunctionList={_hex(payload)}"
    if fct_id == 0x04 and payload:
        return f"heartbeat interval={payload[0]}s"
    if fct_id == 0x0F and payload:
        states = {
            0x00: "normalOperation",
            0x01: "offStandby",
            0x02: "reserved",
            0x03: "initializing",
            0x0E: "functionInactiv",
        }
        return f"FSG state={states.get(payload[0], hex(payload[0]))}"
    if fct_id == 0x11 and payload:
        return "route guidance active" if payload[0] else "route guidance inactive"
    if fct_id == 0x12 and len(payload) >= 8:
        if payload[4:8] == [0xFF, 0xFF, 0x00, 0x00]:
            return "next maneuver distance unavailable"
        dist_dm = _payload_u32_le(payload)
        return f"next maneuver distance={dist_dm / 10:.1f}m"
    if fct_id == 0x13 and ascii_text:
        return f"street={ascii_text}"
    if fct_id == 0x15 and len(payload) >= 6:
        if payload[:6] == [0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]:
            return "destination distance inactive"
        return f"destination distance raw={_hex(payload)}"
    if fct_id == 0x17 and len(payload) >= 2:
        maneuver = MANEUVER_NAMES.get(payload[0], f"0x{payload[0]:02X}")
        direction = payload[1]
        return f"maneuver={maneuver} dir=0x{direction:02X}"
    if fct_id == 0x18 and len(payload) >= 4:
        config = payload[:4]
        if config == [0x12, 0x00, 0x01, 0x00]:
            return "display config=nav-ready"
        if config == [0x16, 0x00, 0x01, 0x00]:
            return "display config=init"
        if config == [0x22, 0x00, 0x00, 0x00]:
            return "display config=nav-start"
        return f"display config={_hex(config)}"
    if fct_id == 0x21 and payload:
        return f"bearing=0x{payload[0]:02X}"
    if fct_id == 0x25 and payload:
        return f"function sync={_hex(payload)}"
    if fct_id == 0x26 and payload:
        return f"info state=0x{payload[0]:02X}"
    if fct_id == 0x32 and payload:
        return f"feature mask={_hex(payload)}"
    if can_id == HUD_GET:
        return f"HUD GET payload={_hex(payload)}"
    if can_id == HUD_ASG:
        return f"HUD announce payload={_hex(payload)}"
    if ascii_text:
        return ascii_text
    if payload:
        return _hex(payload)
    return ""


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

    @property
    def can_id_label(self) -> str:
        return f"0x{self.can_id:X}"

    @property
    def opcode_text(self) -> str:
        return opcode_label(self.opcode)

    @property
    def lsg_text(self) -> str:
        return lsg_label(self.lsg_id)

    @property
    def fct_text(self) -> str:
        return function_label(self.fct_id)

    @property
    def data_text(self) -> str:
        return _hex(list(self.payload))


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
    def __init__(self):
        self._pending: dict[tuple[str, int, int], _PendingBap] = {}

    def feed(self, direction: str, timestamp: str, can_id: int, data: list[int]) -> BapMessage | None:
        if len(data) < 2:
            return None
        if can_id in (HUD_ASG, HUD_GET) and len(data) == 2 and data[0] == 0x1C and data[1] in (0x81, 0x82):
            marker = data[1]
            channel = "HUD announce" if can_id == HUD_ASG else "HUD handoff"
            phase = "start" if marker == 0x82 else "complete"
            return BapMessage(
                direction=direction.lower(),
                timestamp=timestamp,
                can_id=can_id,
                logical_channel=0,
                opcode=1,
                lsg_id=BAP_LSG_HUD_NAV,
                fct_id=0x02,
                payload=tuple(),
                text=f"{channel} marker=1C{marker:02X} ({phase})",
            )

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
            text=describe_bap_message(can_id, opcode, lsg_id, fct_id, payload),
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
            text=describe_bap_message(pending.can_id, opcode, lsg_id, fct_id, payload),
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
    data_hex = _hex(data)
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


class HudBapSession:
    def __init__(self, root_directory: str):
        self._root = Path(root_directory)
        self._root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self._root / f"hud_bap_{stamp}.log"
        self._start_monotonic = time.monotonic()
        self._reassembler = BapReassembler()
        self.messages: list[BapMessage] = []
        self._fh = self.path.open("w", encoding="ascii", buffering=1)
        self._fh.write(busmaster_header("Cursor HUD BAP Session"))

    def handle_frame(self, direction: str, can_id: int, data: list[int]) -> BapMessage | None:
        if can_id not in HUD_TRACE_IDS:
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


def load_hud_bap_messages(log_path: str) -> list[BapMessage]:
    reassembler = BapReassembler()
    messages: list[BapMessage] = []
    with open(log_path, "r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            parsed = parse_busmaster_line(line)
            if not parsed or parsed["can_id"] not in HUD_TRACE_IDS:
                continue
            message = reassembler.feed(parsed["direction"], parsed["timestamp"], parsed["can_id"], parsed["data"])
            if message is not None:
                messages.append(message)
    return messages
