from __future__ import annotations

BAP_LSG_CLUSTER_NAV = 0x31
BAP_LSG_HUD_NAV = 0x32

HUD_S = 0x17333210
HUD_D = 0x17333211
HUD_ASG = 0x17333202
HUD_RX = 0x17330410
HUD_GET = 0x17333200
HUD_TRACE_IDS = {HUD_S, HUD_D, HUD_ASG, HUD_RX, HUD_GET}

GENERIC_LSG_NAMES = {
    0x04: "HUD",
    0x07: "TSK_07",
}

NAV_LSG_NAMES = {
    BAP_LSG_CLUSTER_NAV: "Navigation_Cluster",
    BAP_LSG_HUD_NAV: "Navigation_SD",
}

GENERIC_FUNCTION_NAMES = {
    0x01: "GetAll",
    0x02: "BAP_Config",
    0x03: "FunctionList",
    0x04: "HeartbeatInterval",
}

NAV_FUNCTION_NAMES = {
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

LSG_NAMES = {**GENERIC_LSG_NAMES, **NAV_LSG_NAMES}
FUNCTION_NAMES = {**GENERIC_FUNCTION_NAMES, **NAV_FUNCTION_NAMES}

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
