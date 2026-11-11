# TronV2ardrupilot

Agentic Holybro S500 V2: ArduCopter 4.7.2 on a Pixhawk 6C, RPLidar C1 obstacle avoidance on the flight controller, a Raspberry Pi 5 companion on TELEM3, and QGroundControl plus MAVProxy on an Ubuntu laptop.

The full setup and test procedure is the build guide (`docs/S500-build-guide.pdf` is a snapshot).

## Layout

```
TronV2ardrupilot/
├── agent/       tron: production companion code (objective → goto → search → track), tests, configs
├── gcs/         ground scripts: goto.py (arm, take off, go), target.py (go, already flying)
├── pi/          Pi scripts: nudge.py (body-relative move)
├── params/      flight controller parameters; newest dated file is current
│   └── archive/ older snapshots
├── amc/         ArduPilot Methodic Configurator project (S500joey)
├── docs/        build guide snapshot
├── ardupilot/   submodule, pinned to Copter-4.7.2
└── firmware/    the .apj you flashed (ignored by Git)
```

## Ports

| Port | Listener |
|---|---|
| 14550 | QGroundControl |
| 14551 | gcs/goto.py, gcs/target.py |
| 14552 | pi/nudge.py when tested in SITL |
| 14553 | tron on the laptop |
| 14560 | tron on the Pi |
| 14561 | pi/nudge.py on the Pi |

## Quick start

Laptop radio router:

```bash
apenv
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553
```

Scripts (drone flying in GUIDED for target.py and nudge.py; goto.py takes off itself once in GUIDED):

```bash
python3 gcs/goto.py   udpin:0.0.0.0:14551 LAT LON ALT
python3 gcs/target.py udpin:0.0.0.0:14551 LAT LON ALT
python3 pi/nudge.py   udpin:0.0.0.0:14552 FORWARD RIGHT DOWN
```

tron:

```bash
cd agent && pip install -e '.[test]' && pytest
tron status
tron goto LAT LON ALT
tron mission objective.json
```

See `agent/README.md` for tron and the Pi setup.

## Parameters

Save a new snapshot after any change, named `YYYY-MM-DD_NN_what-changed.params`, and move the previous one into `params/archive/`. Current: `params/2026-11-10_03.params`.
