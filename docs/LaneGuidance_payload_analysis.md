# LaneGuidance (0x18) Payload Analysis

## Extracted Padded Payload (hud_bap_20260315_123840.log, ~00:14:539)

**BAP frames (0x17333211):**
```
80 2D 4C 98 21 01 00 00
C0 08 00 00 01 10 00 00
C1 00 01 00 00 00 00 01
C2 00 02 00 00 19 00 00
C3 00 00 19 00 00 00 00
C4 19 00 00 00 00 19 00
C5 00 00 00 19 00 00
```

**Reconstructed payload (45 bytes):**
```
21 01 00 00 08  00 00 01 10 00 00  00 01 00 00 00 00  01 00 02 00 00 19  00 00 00 00 19 00  00 00 00 19 00 00
```

**Header:** ASG=0x21 (HUD), OnOff=0x01, Mode|RA=0x00, Start=0x00, **Elements=8**

**Lane records (40 bytes, 8 lanes × 5 bytes each when len=0):**

| Lane | dir | len | type | mark_lr | desc_guid | Notes |
|------|-----|-----|------|---------|-----------|-------|
| L0   | 0x00 | 0 | 0x01 | 0x10 (L=1 R=0) | 0x00 | normal, solid left |
| L1   | 0x00 | 0 | 0x01 | 0x00 | 0x00 | normal |
| L2   | 0x00 | 0 | 0x01 | 0x00 | 0x02 | normal, **rec** |
| L3   | 0x00 | 0 | 0x19 | 0x00 | 0x00 | **hide** (padding) |
| L4   | 0x00 | 0 | 0x19 | 0x00 | 0x00 | **hide** (padding) |
| L5   | 0x00 | 0 | 0x19 | 0x00 | 0x00 | **hide** (padding) |
| L6   | 0x00 | 0 | 0x19 | 0x00 | 0x00 | **hide** (padding) |
| L7   | 0x00 | 0 | 0x19 | 0x00 | 0x00 | **hide** (padding) |

## Format Verification (BAP-FC-NAV-SD p.69–72)

**RecordAddress 0 (full):** `LaneDirection, LaneSidestreets, LaneType, LaneMarking_left|right, LaneDescription|GuidanceInfo`

Per lane when `sidestreets_len=0`:
- `dir` (1) + `len` (1) + `type` (1) + `mark_lr` (1) + `desc_guid` (1) = **5 bytes**

Per lane when `sidestreets_len=N`:
- `dir` (1) + `len` (1) + `[N sidestreet bytes]` + `type` (1) + `mark_lr` (1) + `desc_guid` (1) = **6+N bytes**

## Why "5th lane blocked"?

- **0x02** = lane blocked (BAP spec)
- **0x19** = hide lane (BAP spec)

The padded lanes (L3–L7) use **type=0x19 (hide)**. Some HUDs may render "hide" as a greyed/blocked lane, so the 5th lane (L4) can appear blocked even though we send 0x19, not 0x02.

## Current Payload (no padding, 3 lanes)

```
21 01 00 00 03  00 00 19 00 00  00 00 01 00 00  00 00 01 00 02
```

- L0: type=**0x19** (hide) — if you expect normal, set Type=01 in UI
- L1: type=0x01 (normal)
- L2: type=0x01 (normal), rec

**Format is correct.** Ensure lane 0 has Type=01 in the UI if you want a normal lane.
