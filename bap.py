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
    0x0E: "FSG_Setup",
    0x0F: "FSG_OperationState",
    0x10: "CompassInfo",
    0x11: "RG_Status",
    0x12: "DistanceToNextManeuver",
    0x13: "CurrentPositionInfo",
    0x14: "TurnToInfo",
    0x15: "DistanceToDestination",
    0x16: "TimeToDestination",
    0x17: "ManeuverDescriptor",
    0x18: "LaneGuidance",
    0x1C: "ASG_Capabilities",
    0x1D: "ManeuverSupplement",
    0x1E: "FavoriteDest_List",
    0x20: "NavBook",
    0x21: "Address_List",
    0x25: "FunctionSynchronisation",
    0x26: "InfoStates",
    0x27: "ActiveRgType",
    0x29: "GetNextListPos",
    0x2B: "MapColorAndType",
    0x2C: "MapViewAndOrientation",
    0x2D: "MapScale",
    0x2E: "DestinationInfo",
    0x2F: "Altitude",
    0x31: "Exitview",
    0x32: "FeatureEnable",
    0x35: "FSG_Setup",
    0x36: "Map_Presentation",
    0x37: "ManeuverState",
    0x39: "MapContentSettings",
    0x3C: "DistanceToDestinationExtended",
}

MANEUVER_NAMES = {
    0x00: "NoSymbol",
    0x01: "NoInfo",
    0x02: "DirectionToDestination",
    0x03: "Arrived",
    0x04: "NearDestination",
    0x05: "ArrivedDestinationOffmap",
    0x06: "OffRoad",
    0x07: "OffMap",
    0x08: "NoRoute",
    0x09: "CalcRoute",
    0x0A: "RecalcRoute",
    0x0B: "FollowStreet",
    0x0C: "ChangeLane",
    0x0D: "Turn",
    0x0E: "TurnOnMainroad",
    0x0F: "ExitRight",
    0x10: "ExitLeft",
    0x19: "Uturn",
    0x1C: "PrepareTurn",
    0x1D: "PrepareRoundabout",
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


def _payload_s32_le(data: list[int]) -> int:
    v = _payload_u32_le(data)
    return v if v < 0x80000000 else v - 0x100000000


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
    if fct_id == 0x02 and len(payload) >= 6:
        # BAP_Config: BAP_Version_major, minor, LSG_Class, LSG_Sub_Class, LSG_Version_major, minor (BAP-FC-NAV-SD p.17)
        bap_maj, bap_min = payload[0], payload[1]
        lsg_class, lsg_sub = payload[2], payload[3]
        lsg_maj, lsg_min = payload[4], payload[5]
        return f"BAP_Config BAP={bap_maj}.{bap_min} LSG_Class={lsg_class} LSG_def={lsg_maj}.{lsg_min}"
    if fct_id == 0x03 and payload:
        if lsg_id == BAP_LSG_HUD_NAV:
            return _decode_nav_sd_functionlist(payload)
        return f"FunctionList={_hex(payload)}"
    if fct_id == 0x04 and payload:
        # HeartbeatInterval: HeartBeatTime in 100ms units (BAP-FC-NAV-SD p.25)
        val = payload[0]
        ms = val * 100
        return f"HeartbeatInterval {ms}ms" if ms < 1000 else f"HeartbeatInterval {ms / 1000:.1f}s"
    if fct_id == 0x0F and payload:
        states = {
            0x00: "normalOperation",
            0x01: "offStandby",
            0x02: "reserved",
            0x03: "initializing",
            0x0E: "functionInactiv",
        }
        return f"FSG state={states.get(payload[0], hex(payload[0]))}"
    if fct_id == 0x10 and len(payload) >= 3:
        # CompassInfo: Direction_Symbolic, Direction_Angle(Word), (BAP-FC-NAV-SD p.29)
        sym = payload[0]
        angle = payload[1] | (payload[2] << 8)
        dir_names = {0x00: "N", 0x01: "NNO", 0x02: "NO", 0x03: "ONO", 0x04: "O", 0x05: "OSO", 0x06: "SO",
                     0x07: "ssO", 0x08: "S", 0x09: "ssW", 0x0A: "SW", 0x0B: "WSW", 0x0C: "W", 0x0D: "WNW",
                     0x0E: "NW", 0x0F: "NNW", 0xFF: "n/a"}
        sym_str = dir_names.get(sym, f"0x{sym:02X}")
        angle_str = f"{angle}°" if angle != 0xFFFF else "n/a"
        return f"CompassInfo dir={sym_str} angle={angle_str}"
    if fct_id == 0x11 and payload:
        # RG_Status: RG_Status enum (BAP-FC-NAV-SD p.32)
        states = {0x00: "inactive", 0x01: "active", 0x02: "suspended", 0xFF: "n/a"}
        return f"RG_Status {states.get(payload[0], f'0x{payload[0]:02X}')}"
    if fct_id == 0x12 and len(payload) >= 8:
        # DistanceToNextManeuver: Distance(Long), Unit, BargraphOnOff, Bargraph, ValidityInformation (BAP-FC-NAV-SD p.34)
        dist_raw = _payload_u32_le(payload)
        unit = payload[4]
        bargraph_on = payload[5]
        bargraph_pct = payload[6]
        validity = payload[7] if len(payload) > 7 else 0
        dist_valid = bool(validity & 1)
        if not dist_valid or unit == 0xFF:
            bargraph_str = "bargraph=off" if bargraph_on == 0 else (f"bargraph={bargraph_pct}%" if bargraph_on == 1 else "bargraph=n/a")
            return f"DistanceToNextManeuver unavailable {bargraph_str}"
        unit_names = {0x00: "m", 0x01: "km", 0x02: "yd", 0x03: "ft", 0x04: "mi", 0x05: "1/4mi", 0xFF: "n/a"}
        unit_str = unit_names.get(unit, f"0x{unit:02X}")
        return f"DistanceToNextManeuver dist={dist_raw / 10:.1f}{unit_str} bargraph={bargraph_pct}%"
    if fct_id == 0x13 and len(payload) >= 1:
        # CurrentPositionInfo: PositionInfo string (len+UTF8) (BAP-FC-NAV-SD p.39)
        n = payload[0]
        if n == 0:
            return "CurrentPositionInfo position=\"\""
        if n > 0 and 1 + n <= len(payload):
            pos_str = bytes(payload[1:1 + n]).decode("utf-8", errors="replace")
            return f"CurrentPositionInfo position=\"{pos_str}\""
        return f"CurrentPositionInfo {_hex(payload)}"
    if fct_id == 0x14 and len(payload) >= 1:
        # TurnToInfo: TurnToInfo string (len+UTF8), SignPost string (len+UTF8) (BAP-FC-NAV-SD p.41)
        turn_len = payload[0]
        turn_bytes = payload[1:1 + turn_len] if turn_len > 0 and 1 + turn_len <= len(payload) else []
        turn_str = bytes(turn_bytes).decode("utf-8", errors="replace") if turn_bytes else ""
        pos = 1 + turn_len
        sign_str = ""
        if pos < len(payload):
            sign_len = payload[pos]
            sign_bytes = payload[pos + 1:pos + 1 + sign_len] if sign_len > 0 and pos + 1 + sign_len <= len(payload) else []
            sign_str = bytes(sign_bytes).decode("utf-8", errors="replace") if sign_bytes else ""
        if not turn_str and not sign_str:
            return "TurnToInfo (no turn info)"
        if sign_str:
            return f"TurnToInfo turn_to=\"{turn_str}\" signpost=\"{sign_str}\""
        return f"TurnToInfo turn_to=\"{turn_str}\""
    if fct_id == 0x15 and len(payload) >= 6:
        # DistanceToDestination: Distance(Long), Unit, DistType|Validity (BAP-FC-NAV-SD p.43)
        dist_raw = _payload_u32_le(payload)
        unit = payload[4]
        byte5 = payload[5]
        dist_type = (byte5 >> 4) & 0x0F  # 0=to Destination, 1=to Stopover
        dist_valid = bool(byte5 & 1)
        if not dist_valid or unit == 0xFF:
            type_str = "stopover" if dist_type == 1 else "destination"
            return f"DistanceToDestination inactive (type={type_str})"
        unit_names = {0x00: "m", 0x01: "km", 0x02: "yd", 0x03: "ft", 0x04: "mi", 0x05: "1/4mi", 0xFF: "n/a"}
        unit_str = unit_names.get(unit, f"0x{unit:02X}")
        type_str = "stopover" if dist_type == 1 else "destination"
        return f"DistanceToDestination dist={dist_raw / 10:.1f}{unit_str} type={type_str}"
    if fct_id == 0x16 and len(payload) >= 7:
        # TimeToDestination: TimeInfoType|NavTimeFormat, Min, Hour, Day, Month, Year, Validity (BAP-FC-NAV-SD p.47)
        info_type = (payload[0] >> 4) & 0x0F
        time_format = payload[0] & 0x0F
        minute, hour = payload[1], payload[2]
        day, month, year = payload[3], payload[4], payload[5]
        validity = payload[6]
        min_valid = bool(validity & 2)
        hour_valid = bool(validity & 4)
        type_names = {0x0: "driving_time", 0x1: "arrival_time", 0x2: "arrival_other_tz", 0xF: "n/a"}
        type_str = type_names.get(info_type, f"0x{info_type:X}")
        if info_type == 0xF or not (min_valid or hour_valid):
            return f"TimeToDestination inactive type={type_str}"
        if info_type == 0x0:
            return f"TimeToDestination driving_time={hour}h{minute:02d}m"
        return f"TimeToDestination arrival={2000+year}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}"
    if fct_id == 0x17 and len(payload) >= 4:
        # ManeuverDescriptor: up to 3 maneuvers, each MainElement+Direction+Z_Level+Sidestreets (BAP-FC-NAV-SD p.53)
        dir_names = {0x00: "straight", 0x40: "left", 0x80: "back", 0xC0: "right"}
        parts = []
        pos = 0
        for i in range(3):
            if pos + 4 > len(payload):
                break
            main, direction, z_level = payload[pos], payload[pos + 1], payload[pos + 2]
            sidelen = payload[pos + 3]
            pos += 4 + sidelen
            if main == 0x00 and direction == 0 and z_level == 0 and sidelen == 0:
                continue
            m_str = MANEUVER_NAMES.get(main, f"0x{main:02X}")
            d_str = dir_names.get(direction, f"0x{direction:02X}")
            parts.append(f"{m_str}({d_str})")
        if not parts:
            return "ManeuverDescriptor inactive (no maneuver)"
        return f"ManeuverDescriptor {' '.join(parts)}"
    if fct_id == 0x18 and len(payload) >= 5:
        # LaneGuidance StatusArray: ASG_ID|TAID, LaneGuidanceOnOff, Mode|RecordAddress, Start, Elements (BAP-FC-NAV-SD p.66)
        asg_id = (payload[0] >> 4) & 0x0F
        taid = payload[0] & 0x0F
        on_off = payload[1]
        rec_addr = payload[2] & 0x0F
        start, elements = payload[3], payload[4]
        asg_names = {0x0: "default", 0x1: "cluster", 0x2: "HUD", 0x3: "rear"}
        on_off_str = "on" if on_off == 0x01 else "off"
        arr_info = f" elements={elements}" if elements else ""
        return f"LaneGuidance ASG={asg_names.get(asg_id, asg_id)} TAID={taid} OnOff={on_off_str}{arr_info}"
    if fct_id == 0x1D and payload:
        # ManeuverSupplement: HUD-specific, e.g. roundabout exit / supplement value (Word) + validity
        if len(payload) >= 3:
            val = payload[0] | (payload[1] << 8)
            validity = payload[2]
            if validity == 0xFF:
                return "ManeuverSupplement (no supplement / n/a)"
            return f"ManeuverSupplement exit={val} validity=0x{validity:02X}"
        return f"ManeuverSupplement {_hex(payload)}"
    if fct_id == 0x21 and len(payload) >= 5:
        # Address_List: ASG_ID|TAID, OtherListType, OtherList_Reference(Word), TotalNumListElements(Word) (BAP-FC-NAV-SD p.126)
        asg_taid = payload[0]
        asg_id = (asg_taid >> 4) & 0x0F
        taid = asg_taid & 0x0F
        other_type = payload[1]
        other_ref = payload[2] | (payload[3] << 8)
        total = payload[4] | (payload[5] << 8) if len(payload) >= 6 else (0xFFFF if payload[4] == 0xFF else payload[4])
        asg_names = {0x0: "default", 0x1: "cluster", 0x2: "HUD", 0x3: "rear", 0x8: "reserved", 0x9: "cluster_all", 0xA: "HUD_all", 0xB: "rear_all"}
        type_names = {0x00: "LastDest", 0x01: "FavoriteDest", 0x02: "NavBook", 0x03: "PhoneBook", 0x04: "HomeAddr", 0x05: "BusinessAddr", 0x0F: "completeList"}
        asg_str = asg_names.get(asg_id, f"0x{asg_id:X}")
        type_str = type_names.get(other_type, f"0x{other_type:02X}")
        total_str = "unknown" if total == 0xFFFF else str(total)
        return f"Address_List ASG={asg_str} TAID={taid} type={type_str} ref={other_ref} total={total_str}"
    if fct_id == 0x21 and payload:
        return f"Address_List {_hex(payload)}"
    if fct_id == 0x25 and len(payload) >= 8:
        # FunctionSynchronisation: FctList_1(6), FctList_2(1), FctList_3(1) - same encoding as FunctionList (BAP-FC-NAV-SD p.153)
        # Bit=1 means "to be synchronised" (don't evaluate yet)
        pending = []
        for bit, name in sorted(NAV_SD_FUNCTIONLIST_BITS.items()):
            if bit >= 64:
                break
            byte_idx = bit // 8
            bit_idx = bit % 8
            if byte_idx < len(payload) and (payload[byte_idx] >> bit_idx) & 1:
                pending.append(name)
        if not pending:
            return "FunctionSynchronisation (all evaluate immediately)"
        return f"FunctionSynchronisation pending=[{', '.join(pending)}]"
    if fct_id == 0x26 and payload:
        # InfoStates: States enum (BAP-FC-NAV-SD p.158)
        states = {
            0x00: "no error/no information",
            0x01: "no nav data medium",
            0x02: "nav database corrupted",
            0x03: "no GPS signal",
            0x04: "nav database update ongoing",
            0x05: "Initializing MOST Map",
            0x06: "Navigation in mobile device active",
            0x07: "No nav database available",
            0xFF: "unknown",
        }
        return f"InfoStates {states.get(payload[0], f'0x{payload[0]:02X}')}"
    if fct_id == 0x27 and payload:
        # ActiveRgType: RGType enum (BAP-FC-NAV-SD p.160)
        rg_types = {
            0x00: "RGI (ManeuverDescriptor)",
            0x01: "MOST-KDK",
            0x02: "compass (CompassInfo)",
            0x03: "MOST-Map",
            0x04: "LVDS-Map",
            0x05: "LVDS-KDK",
            0xFF: "ASG default",
        }
        return f"ActiveRgType {rg_types.get(payload[0], f'0x{payload[0]:02X}')}"
    if fct_id == 0x37 and len(payload) >= 4:
        # ManeuverState: State, Dummy1, Dummy2, Dummy3 (BAP-FC-NAV-SD p.250)
        states = {
            0x00: "init/unknown",
            0x01: "Follow",
            0x02: "Prepare",
            0x03: "Distance",
            0x04: "CallForAction",
        }
        state = states.get(payload[0], f"0x{payload[0]:02X}")
        return f"ManeuverState={state}"
    if fct_id == 0x36 and payload:
        # Map_Presentation: ASG_HMI_State bitfield (BAP-FC-NAV-SD p.248)
        b = payload[0]
        map_view = "large" if (b & 1) else "small"
        left_menu = "open" if (b & 2) else "closed"
        right_menu = "open" if (b & 4) else "closed"
        return f"Map_Presentation map={map_view} left={left_menu} right={right_menu}"
    if fct_id == 0x35 and len(payload) >= 6:
        # FSG_Setup: VoiceGuidance, Supported_POI_Types, FunctionSupport, Dummy2-4 (BAP-FC-NAV-SD p.243)
        voice = payload[0]
        poi = payload[1]
        func = payload[2]
        voice_list = [n for i, n in enumerate(["full", "reduced", "traffic"]) if (voice >> i) & 1]
        voice_str = ",".join(voice_list) if voice_list else "none"
        poi_str = "fuel+parking" if (poi & 1) else "none"
        func_list = [n for i, n in enumerate(["video_nav", "home_addr", "business_addr", "mobile_rg", "ext_maneuver", "maneuver_list"]) if (func >> i) & 1]
        func_str = ",".join(func_list) if func_list else "none"
        return f"FSG_Setup voice=[{voice_str}] POI={poi_str} func=[{func_str}]"
    if fct_id == 0x39 and len(payload) >= 6:
        # MapContentSettings: AvailableContents, ContentStatus, Extension1-4 (BAP-FC-NAV-SD p.256)
        avail = payload[0]
        status = payload[1]
        contents = ["GoogleInfo", "OnlineInfo", "POIInfo", "TrafficInfo"]
        avail_list = [c for i, c in enumerate(contents) if (avail >> i) & 1]
        status_list = [c for i, c in enumerate(contents) if (status >> i) & 1]
        avail_str = ",".join(avail_list) if avail_list else "none"
        status_str = ",".join(status_list) if status_list else "all off"
        return f"MapContentSettings available=[{avail_str}] on=[{status_str}]"
    if fct_id == 0x3C and len(payload) >= 10:
        # DistanceToDestinationExtended: TotalDistance (4+1), percentage, ERangeWarning+Ext1, Ext2-4, Validity (BAP-FC-NAV-SD p.275)
        dist_raw = _payload_u32_le(payload)
        unit = payload[4]
        pct = payload[5]
        erange = payload[6] & 0x0F
        validity = payload[10] if len(payload) > 10 else 0
        unit_names = {0x00: "m", 0x01: "km", 0x02: "yd", 0x03: "ft", 0x04: "mi", 0x05: "1/4mi", 0xFF: "n/a"}
        unit_str = unit_names.get(unit, f"0x{unit:02X}")
        dist_valid = bool(validity & 1)
        pct_valid = bool(validity & 2)
        dist_str = f"{dist_raw / 10:.1f}{unit_str}" if dist_valid and unit != 0xFF else "invalid"
        pct_str = f"{pct}%" if pct_valid and pct != 0xFF else "invalid"
        erange_names = {0x0: "no warning", 0x1: "low range", 0x2: "very low", 0x3: "endangered", 0x4: "very endangered",
                        0x5: "charge planned", 0x6: "charge required", 0x9: "RangeExtender", 0xA: "RE imminent", 0xB: "RE in use"}
        erange_str = erange_names.get(erange, f"0x{erange:X}")
        return f"DistanceToDestinationExtended dist={dist_str} pct={pct_str} range={erange_str}"
    if fct_id == 0x2B and len(payload) >= 4:
        # MapColorAndType: Colour, ActiveMapType, MainMapSetup, SupportedMapTypes (BAP-FC-NAV-SD p.176)
        colour = payload[0]
        map_type = payload[1]
        main_setup = payload[2]
        supported = payload[3]
        colour_names = {0x00: "day", 0x01: "night", 0x02: "auto", 0xFF: "n/a"}
        type_names = {0x00: "destination", 0x01: "position2D", 0x02: "position3D", 0x03: "overview", 0x04: "range", 0xFF: "n/a"}
        setup_names = {0x00: "init", 0x01: "main_in_ASG", 0x02: "main_in_FSG", 0xFF: "n/a"}
        supp_list = [n for i, n in enumerate(["MainMapSetup", "destination", "position2D", "position3D", "overview", "range"]) if (supported >> i) & 1]
        supp_str = ",".join(supp_list) if supp_list else "none"
        return f"MapColorAndType colour={colour_names.get(colour, hex(colour))} type={type_names.get(map_type, hex(map_type))} supported=[{supp_str}]"
    if fct_id == 0x2C and len(payload) >= 7:
        # MapViewAndOrientation: MapView, SupplementaryMapView, SupportedMapViews, SupportedSupplementaryMapViews, MapVisibility, MapOrientation, Modification (BAP-FC-NAV-SD p.180)
        map_view = payload[0]
        supp_view = payload[1]
        supp_map = payload[2]
        supp_supp = payload[3]
        visibility = payload[4]
        orientation = payload[5]
        view_names = {0x00: "standard", 0x01: "google_earth", 0x02: "traffic", 0xFF: "n/a"}
        supp_names = {0x00: "none", 0x01: "intersection_zoom", 0x02: "compass", 0x03: "3+1Box", 0xFF: "n/a"}
        orient_names = {0x00: "automatic", 0x01: "north", 0x02: "direction_of_travel", 0xFF: "n/a"}
        vis_list = [n for i, n in enumerate(["LVDS", "supplementary", "standard"]) if (visibility >> i) & 1]
        vis_str = ",".join(vis_list) if vis_list else "none"
        return f"MapViewAndOrientation view={view_names.get(map_view, hex(map_view))} supp={supp_names.get(supp_view, hex(supp_view))} orient={orient_names.get(orientation, hex(orientation))} visible=[{vis_str}]"
    if fct_id == 0x2D and len(payload) >= 6:
        # MapScale: Reserve1, AutoZoom|AutoZoomState, Scale(Word), Unit, SupportedAutoZoom (BAP-FC-NAV-SD p.186)
        autozoom_nibbles = payload[1]
        autozoom = (autozoom_nibbles >> 4) & 0x0F
        autozoom_active = bool(autozoom_nibbles & 1)
        scale_raw = payload[2] | (payload[3] << 8)
        unit = payload[4]
        unit_names = {0x00: "m", 0x01: "km", 0x02: "yd", 0x03: "ft", 0x04: "mi", 0x05: "1/4mi", 0x06: "km(0.1)", 0x07: "mi(0.1)", 0xFF: "n/a"}
        unit_str = unit_names.get(unit, f"0x{unit:02X}")
        if scale_raw == 0:
            scale_str = "invalid"
        elif scale_raw == 0xFFFE:
            scale_str = "max"
        elif scale_raw == 0xFFFF:
            scale_str = "init"
        else:
            scale_str = f"{scale_raw * 0.1:.1f}{unit_str}"
        az_names = {0x0: "off", 0x1: "on", 0x2: "on@intersection", 0xF: "n/a"}
        az_str = az_names.get(autozoom, f"0x{autozoom:X}")
        active_str = " active" if autozoom_active else ""
        return f"MapScale scale={scale_str} AutoZoom={az_str}{active_str}"
    if fct_id == 0x2E and len(payload) >= 12:
        # DestinationInfo: Position(8), TotalNumOfStopovers, Stopover_SN, POI_Type, strings (BAP-FC-NAV-SD p.193)
        lat_raw = _payload_s32_le(payload)
        lon_raw = _payload_s32_le(payload[4:8])
        lat_deg = lat_raw / 1e6 if lat_raw != 0 else 0
        lon_deg = lon_raw / 1e6 if lon_raw != 0 else 0
        total_stop, stop_sn, poi_type = payload[8], payload[9], payload[10]
        pos = 11

        def _read_bap_string() -> str:
            nonlocal pos
            if pos >= len(payload):
                return ""
            n = payload[pos]
            pos += 1
            if pos + n > len(payload):
                return ""
            s = bytes(payload[pos:pos + n]).decode("utf-8", errors="replace")
            pos += n
            return s

        poi_desc = _read_bap_string()
        street = _read_bap_string()
        town = _read_bap_string()
        state = _read_bap_string()
        postal = _read_bap_string()
        country = _read_bap_string()
        stop_str = "final" if stop_sn in (0, 0xFF) else f"stopover {stop_sn}"
        addr = ", ".join(x for x in [street, town, state, postal, country] if x)
        if lat_deg or lon_deg:
            coord = f" {lat_deg:.4f}°N {lon_deg:.4f}°E" if lon_deg >= 0 else f" {lat_deg:.4f}°N {lon_deg:.4f}°W"
        else:
            coord = ""
        return f"DestinationInfo {addr}{coord} ({stop_str})"
    if fct_id == 0x2F and len(payload) >= 3:
        # Altitude: Altitude(Word signed), Unit (BAP-FC-NAV-SD p.204)
        alt_raw = payload[0] | (payload[1] << 8)
        alt_signed = alt_raw if alt_raw < 0x8000 else alt_raw - 0x10000
        unit = payload[2]
        unit_names = {0x00: "m", 0x01: "ft", 0xFF: "n/a"}
        unit_str = unit_names.get(unit, f"0x{unit:02X}")
        if alt_raw == 0x7FFF:
            return "Altitude init/unknown"
        return f"Altitude {alt_signed}{unit_str}"
    if fct_id == 0x31 and len(payload) >= 3:
        # Exitview: Variant, Exitview_ID(Word) (BAP-FC-NAV-SD p.210)
        variant = payload[0]
        exit_id = payload[1] | (payload[2] << 8)
        variant_names = {0x00: "EU", 0x01: "NAR", 0x02: "ROW", 0x03: "ASIA", 0xFF: "unknown"}
        variant_str = variant_names.get(variant, f"0x{variant:02X}")
        if exit_id == 0:
            return f"Exitview variant={variant_str} no exit view"
        return f"Exitview variant={variant_str} id={exit_id}"
    if fct_id == 0x32 and payload:
        # FeatureEnable: 16-byte bitmask (HUD nav feature enable, project-specific)
        if len(payload) < 16:
            return f"FeatureEnable {_hex(payload)}"
        total = sum(bin(b).count("1") for b in payload[:16])
        if total == 0:
            return "FeatureEnable (all disabled)"
        active_bytes = [i for i in range(16) if payload[i]]
        if len(active_bytes) <= 3:
            parts = [f"byte{i}=0x{payload[i]:02X}" for i in active_bytes]
            return f"FeatureEnable {total} bits [{', '.join(parts)}]"
        return f"FeatureEnable {total} bits enabled"
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
