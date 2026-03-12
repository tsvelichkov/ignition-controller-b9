"""
Local HUD-nav BAP hints derived from decoded traces and prior experiments.

This is intentionally small and pragmatic: it gives the infotainment module a
single place to look up function names and the expected "reply to this" shape
instead of scattering those assumptions across state-machine code.
"""

HUD_NAV_FUNCTION_NAMES = {
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
    0x17: "ManeuverDescriptor",
    0x18: "LaneGuidance",
    0x25: "FunctionSynchronisation",
    0x26: "InfoStates",
    0x36: "Map_Presentation",
}

HUD_NAV_HINTS = {
    0x02: "Reply immediately with nav BAP_Config status.",
    0x03: "Reply with current FunctionList for the current nav phase.",
    0x04: "Reply immediately with heartbeat interval 0x0A.",
    0x0D: "If HUD provides payload, mirror it back as nav status for now.",
    0x0E: "Observed as HUD-side setup traffic; currently recognized but intentionally ignored.",
    0x0F: "Reply with infotainment FSG operation state.",
    0x10: "Reply with CompassInfo; mirror HUD payload when available.",
    0x11: "Reply with route guidance active/inactive state.",
    0x12: "Reply with distance payload based on current arrow preset.",
    0x13: "Reply with empty CurrentPositionInfo until a real source exists.",
    0x14: "Reply with TurnToInfo (street name / session state) payload.",
    0x17: "Reply with maneuver descriptor based on current arrow preset.",
    0x18: "LaneGuidance: choose init vs active display config payload.",
    0x25: "Reply with function sync payload matching current nav phase.",
    0x26: "Reply with InfoStates status.",
    0x36: "Map_Presentation: special handshake, echo value and optionally seed nav bundle.",
}
