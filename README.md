# CAN Nav Controller

Audi MLB CAN bus emulator for testing navigation and infotainment systems. Emulates ignition, gateway, infotainment, and other ECUs to simulate a vehicle CAN network for development and bench testing.

## What It Does

- **Ignition simulation** — Sends `0x3C0` Klemmen_Status frames (KL15 on/off) to power up or shut down modules on the infotainment canbus
- **MFL steering wheel** — Emulates Multi-Function Lever buttons (MENU, OK, LEFT, RIGHT, etc.) on `0x5BF`
- **ECU emulation** — Modular ECU modules send periodic CAN frames (gateway, infotainment, motor, ESP, etc.)
- **BAP navigation** — Infotainment module drives HUD and cluster with navigation arrows and route data (WIP)

## Requirements

- **Python 3.8+**
- **python-can** — CAN bus abstraction
- **python-can-csscan-serial** — CSS Electronics USB adapter (default interface)
- **tkinter** — GUI (included with most Python installs)
- **CAN hardware** — Adapter to connect to the vehicle or MIB unit (see [CAN Connection](#can-connection))

## Setup

### 1. Clone the repository

```bash
git clone <repo-url>
cd ignition-controller-b9
```

### 2. Create a virtual environment (recommended)

```bash
python -m venv venv
venv\Scripts\activate          # Windows
# source venv/bin/activate     # Linux/macOS
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the application

```bash
python nav_controller.py
```

## Project Structure

```
ignition-controller-b9/
├── nav_controller.py    # Main app — GUI, bus manager, ignition, MFL
├── ecu_base.py          # Base class for ECU modules
├── modules/             # ECU emulation modules (auto-loaded)
│   ├── 01_motor.py      # Motor ECU
│   ├── 03_esp.py        # ESP
│   ├── 09_bcm.py        # Body Control Module
│   ├── 13_acc.py        # ACC
│   ├── 15_airbag.py     # Airbag
│   ├── 19_gateway.py    # Gateway — NM keepalives, NVEM, Diagnose
│   ├── 44_eps.py        # Electric Power Steering
│   ├── 5f_infotainment.py  # Infotainment — HUD, cluster, BAP nav
│   └── a5_drvassist.py  # A5 Driver Assistance
├── requirements.txt
└── README.md
```

## CAN Connection

The default interface is **csscan_serial** (CSS Electronics USB adapter). To use a different adapter, edit the config block at the top of `nav_controller.py`:

### CSS Electronics (csscan_serial) — default

The [python-can-csscan-serial](https://canlogger.csselectronics.com/tools-docs/csscan_serial/python_csscan/index.html) package adds support for the CSS Electronics USB interface. It is installed via `pip install python-can-csscan-serial` (included in `requirements.txt`).

**Queue / throughput**: At 115200 baud the serial link limits to ~300–400 frames/sec; the ECU emulators can produce more, so the queue backs up. The app uses `SERIAL_BAUDRATE = 921600` by default (edit near top of `nav_controller.py`). If the adapter does not support 921600, set it to `115200` and consider reducing ECU modules or message rates.

The adapter appears as a USB virtual-serial-port:
- **Windows**: Device Manager → Ports (COM & LPT) → "USB Serial Device" (e.g. COM19)
- **Linux**: `/dev/ttyACM*` or `/dev/ttyUSB*` — check with `journalctl -f` when plugging in

| Interface       | Use case                          |
|-----------------|-----------------------------------|
| `csscan_serial` | CSS Electronics USB (default)     |
| `socketcan`     | Linux kernel CAN (vcan0, can0)   |
| `pcan`          | PEAK PCAN-USB, PCAN-PCI           |
| `serial`        | Generic CAN over serial (COM3)    |
| `slcan`         | SLCAN protocol (many USB adapters)|
| `virtual`       | No hardware — testing only        |
| `gs_usb`        | candleLight, Geschwister Schneider|

Example for SocketCAN on Linux:

```python
CAN_INTERFACE = "socketcan"
CAN_AVAILABLE_CONFIGS = [{"interface": "socketcan", "channel": "vcan0"}]
```

Example for virtual (no hardware):

```python
CAN_INTERFACE = "virtual"
CAN_AVAILABLE_CONFIGS = [{"interface": "virtual", "channel": None}]
```

## Usage

1. **Connect** your CAN adapter to the MIB unit or vehicle CAN bus.
2. **Launch** `nav_controller.py` — the app auto-detects the adapter and connects.
3. **Turn ignition ON** — Click the ignition button to send KL15. ECUs start sending frames.
4. **Use MFL buttons** — Simulate steering wheel buttons (MENU, OK, arrows, etc.).
5. **Monitor ECUs** — Each attached ECU shows its messages and TX counts in the right panel.

## ECU Modules

ECU modules in `modules/` are loaded automatically. Each module subclasses `ECUModule` and defines:

- `ECU_ID` — Short identifier (e.g. `"19"` for gateway)
- `ECU_NAME` — Display name
- `MESSAGES` — List of `(arb_id, name, interval_ms, data)` for periodic frames

To add a new ECU, create a `.py` file in `modules/` that subclasses `ECUModule`.

## Lane guidance (HUD)

The NAV HUD panel can send **LaneGuidance** (BAP function `0x18`) so the HUD or cluster shows lane recommendations. You set the **number of lanes** (2–8); the UI then shows one row per lane. Each row configures that lane and has a **Preferred** checkbox (exactly one lane should be preferred = “best recommendation”). Values follow the BAP Navigation_SD function catalogue (e.g. *476121999-BAP-FC-NAV-SD-P30DF48-v2-80-F*), §14.5 LaneGuidance.

| Input | Type | Supported values |
|-------|------|------------------|
| **Dir** (LaneDirection) | Byte (hex) | `00` = straight (default), `40` = left, `C0` = right; `00`–`FE` per ManeuverDescriptor Direction; `FF` = invalid. |
| **Type** (LaneType) | Byte (hex) | `00` unspecific, `01` normal, `02` blocked, `03` forbidden, `04` restricted, `05` breakdown, `06` dynamically controlled, `07` car pools, `08` bus, `09` bicycle, `0A` taxi, `0B` truck, `0C` bus & bicycle, `0D` bus & taxi, `0E` taxi & bicycle, `0F` bus/taxi/bicycle, `10` green strip, `11` motorway, `12` tramway (road), `13` separated tramway, `14` exit lane, `15` enter lane, `16` toll, `17` further lanes left (leftmost only), `18` further lanes right (rightmost only), `19` hide lane, `1A` ETC/non-ETC mixed; `1B`–`FF` reserved. |
| **ML** (LaneMarking_left) | Nibble | `0` = no line, `1` = solid, `2` = dashed; `3`–`F` reserved. |
| **MR** (LaneMarking_right) | Nibble | Same as ML. |
| **Desc** (LaneDescription) | Nibble (hex) | `0` lane available, `1` merging left, `2` merging right, `3` ending later, `4` forbidden, `5` expanding left, `6` expanding right, `7` available later, `8` multiple additional lane begin, `9` multiple continuation with lane ends, `A` lane begins in the middle, `B`–`E` reserved, `F` not supported. |
| **Preferred** (GuidanceInfo) | Checkbox | Checked = best recommendation (`0x2`); unchecked = not recommended (`0x0`). Only one lane should be preferred. |

- **LaneDirection** encoding matches ManeuverDescriptor Direction (e.g. turn on mainroad). `0xFF` = invalid.
- **LaneSidestreets** is not exposed in the UI; the emulator sends length `0` (no sidestreet data).
- Record format: RecordAddress `0` = full (Dir, SidestreetsLen, Type, MarkL\|MarkR, Desc\|Guidance); RecordAddress `2` = reduced (Dir, Desc\|Guidance).

## Related Repos

- [`MIGINC/BAP_RE`](https://github.com/MIGINC/BAP_RE) — reverse-engineering notes and traces for the BAP protocol. Useful as a protocol reference for:
  - BAP glossary and roles like `FSG`, `ASG`, and `LSG`
  - startup flow around `BAP_Config (0x02)` and `FunctionList (0x03)`
  - `FSGOperationState (0x0F)` meanings such as `0x00 = Normal Operation`, `0x01 = Off/Standby`, `0x03 = Initializing`, `0x0E = Function Inactive`

## License

See repository for license information.
