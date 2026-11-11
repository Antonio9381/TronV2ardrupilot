# S500 Agentic Drone Build Guide

Nov 6, 2026 · @Antonio

## Contents

1. Overview
2. Progress
3. Phase 0: Parts and pre-checks
4. Phase 1: Simulation on Ubuntu
5. Phase 2: Firmware choice (done)
6. Phase 3: Flashing (done)
7. Phase 4: Wiring
8. Phase 5: Base setup in QGroundControl
9. Phase 6: Lidar and obstacle avoidance
10. Phase 7: Raspberry Pi 5 on TELEM3
11. Phase 8: Ground station (QGroundControl + MAVProxy + your scripts)
12. Phase 9: Flight tests
13. Testing every script
    1. Test 1: Install tron and run its unit tests (laptop)
    2. Test 2: Start one SITL session for all script tests
    3. Test 3: goto.py in SITL
    4. Test 4: target.py in SITL
    5. Test 5: nudge.py in SITL
    6. Test 5b: wasd.py in SITL (keyboard flying)
    7. Test 6: tron in SITL
    8. Test 7: Old scripts and tron side by side in SITL
    9. Test 8: tron on the Pi (install, unit tests)
    10. Test 9: Ground checks on the real drone (props off)
    11. Test 10: Field flights, old script then tron
14. Everyday flight session
15. Troubleshooting
16. Sources

## Overview

The Holybro S500 V2 runs ArduCopter 4.7.2 on its Pixhawk 6C. An RPLidar C1 plugs straight into the Pixhawk, so obstacle avoidance always runs on the flight controller. Targets come from the ground station over the SiK radio, and from the Raspberry Pi 5 over a UART. **QGroundControl** (stable release) on Ubuntu is the ground station for setup, monitoring and flying.

**Who does what**

- **Pixhawk 6C + ArduCopter:** always the pilot. Runs GUIDED mode, BendyRuler path planning from the lidar, and every failsafe.
- **Raspberry Pi 5:** mission manager (no camera fitted yet; one is planned for colour tracking). Sends GPS targets when far away, body-relative moves when close.
- **Ground station (Ubuntu laptop):** QGroundControl shows the drone and handles setup; MAVProxy shares the radio link between QGC and your agent scripts.
- **RC transmitter (FS-iA6B receiver, PPM):** final manual override at all times.

**Tools**

| Job | Tool |
| --- | --- |
| Simulation, building, script development | ArduPilot SITL + MAVProxy |
| Drone setup, calibration, parameters, motor test | QGroundControl |
| Starting tune values from prop size and battery | ArduPilot Methodic Configurator (Phase 5.7) |
| Sharing the radio between QGC and scripts | MAVProxy router (Phase 8) |
| Optional, on Windows: lidar radar view, tuning helper, log analysis | Mission Planner |

The old ground scripts connect to `udpin:0.0.0.0:14551` and tron to `udpin:0.0.0.0:14553` (full port map under Testing every script). In simulation and in the field, MAVProxy feeds those ports, so the same commands run unchanged in both.

Use the **stable** QGroundControl release, not Daily: the Daily build's firmware page had no ArduPilot option.

**Order of work.** Simulation first, so every command and parameter is proven before it touches the aircraft. Each phase ends with a check that must pass before the next one.

**Safety rules for every bench phase**

- Propellers **off** for everything until Phase 9. That includes "just a quick motor test".
- Flight battery disconnected while wiring. Power the Pixhawk from USB for configuration.
- Never power the Pi 5 from a Pixhawk port.
- One change at a time, then test. After each phase that passes, save the parameters (QGC: Parameters → Tools → Save to file) into your repo's `params/` folder and commit.

## Progress

| Phase | Status | Notes |
| --- | --- | --- |
| 0 Parts and pre-checks | Done | PX4 parameters backed up, motor mapping recorded |
| 1 Simulation | Done | Avoidance in GUIDED works with GUID\_OPTIONS = 64 |
| 2 Firmware choice | Done | Copter/stable/Pixhawk6C/arducopter.apj |
| 3 Flashing | Done | Flashed with the uploader |
| 4 Wiring | Done | SiK TELEM1, lidar TELEM2, Pi TELEM3, FS-iA6B on RC IN (PPM) |
| 5 Base setup in QGroundControl | Done | DShot300; internal compass off (COMPASS\_USE2 = 0) |
| 6 Lidar and avoidance | Done | Avoidance parameters set; confirm in flight (Phase 9 items 3 and 5) |
| 7 Raspberry Pi 5 | Done | nudge.py reads the mode over TELEM3 and refuses outside GUIDED |
| 8 Ground station | In progress | Router and QGC working; target.py not yet flown |
| 9 Flight tests | In progress | Loiter hover flown and log checked; items 2 onward to do |
| Script tests | To do | Tests 1–10 below |

## Phase 0: Parts and pre-checks

| Part | Notes |
| --- | --- |
| Holybro S500 V2 frame, motors, ESCs | Four ESCs on DShot300, 10-inch props, 4S battery |
| Holybro Pixhawk 6C (not Mini) | The Mini has no TELEM3, which this plan uses for the Pi |
| GPS/compass module | On GPS1 |
| Holybro SiK radio pair (915 MHz) | Air unit on TELEM1, ground unit on the laptop's USB |
| Slamtec RPLidar C1 | On TELEM2; keep its USB adapter board for bench tests |
| FS-iA6B receiver (PPM) + transmitter | On RC IN; manual override |
| Raspberry Pi 5 + microSD (32 GB+) | On TELEM3 |
| 5V 5A BEC (4S input) | Dedicated supply for the Pi 5 |
| JST-GH 6-pin cables, Dupont jumpers | For TELEM2 → lidar and TELEM3 → Pi |

**Pre-checks**

- [x] Read the Pixhawk 6C serial number. If its middle block is **001**, TELEM3 pins 4 and 5 are I2C; fine here, because the Pi uses only TX, RX and GND, but never connect anything to those two pins.
- [x] Back up the PX4 parameters before flashing.
- [x] Record the PX4 motor mapping: ESCs on AUX 1–4 with DShot600, AUX1 = Motor 3, AUX2 = Motor 2, AUX3 = Motor 4, AUX4 = Motor 1.
- [x] Note the prop size and cell count: 10-inch, 4S for the stock kit.
- [x] Test the lidar alone on its USB adapter board with Slamtec's RoboStudio, so you know the sensor works before wiring it in.

## Phase 1: Simulation on Ubuntu

ArduPilot's simulator (SITL) fakes an RPLidar surrounded by virtual posts, so avoidance and your scripts can be tested with no hardware. Everything here runs on your laptop in Ubuntu, not on the Pi.

**1.1 One-time shell setup.** Two shortcuts used throughout this guide: `$REPO` for your repo and `apenv` to activate the ArduPilot Python environment.

```bash
echo 'export REPO=$HOME/Documents/2026/usyd/yr2sem2/joey-tron/tronV2adrupilot' >> ~/.bashrc
echo "alias apenv='source ~/venv-ardupilot/bin/activate'" >> ~/.bashrc
. ~/.bashrc
```

Don't make the ArduPilot venv activate automatically in every terminal. It carries setuptools 84, and colcon (your ROS 2 Jazzy builds) needs a version below 80. Activate it with `apenv` only where you're doing ArduPilot work.

**1.2 Repo layout (done).** Your repo `TronV2ardrupilot` holds everything for this build, with ArduPilot as a submodule pinned to Copter-4.7.2:

```
tronV2adrupilot/
├── agent/       tron: production companion code, tests, configs
├── gcs/         ground scripts: goto.py, target.py, wasd.py
├── pi/          Pi scripts: nudge.py
├── params/      newest dated .params file is current
│   └── archive/ older snapshots
├── amc/         Methodic Configurator project
├── docs/        build guide snapshot
├── ardupilot/   submodule, Copter-4.7.2
├── firmware/    the .apj you flashed (ignored by Git)
└── logs/        flight logs (ignored by Git)
```

Check the submodule is on the right version. Your prompt shows `APMrover2-stable` because several tags can point at one commit; what matters is that Copter-4.7.2 is among them:

```bash
git -C $REPO/ardupilot tag --points-at HEAD | grep Copter-4.7.2
```

If that prints nothing, pin it and rebuild:

```bash
cd $REPO/ardupilot
git checkout Copter-4.7.2
git submodule update --init --recursive
cd $REPO && git add ardupilot && git commit -m "Pin ArduPilot to Copter-4.7.2"
```

**1.3 Prerequisites and build (done).** For reference, or if you set up another machine:

```bash
cd $REPO/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh
apenv
./waf configure --board sitl
./waf copter
```

Run the install script without `-y` and answer **N** to "Make ArduPilot venv default" and to "Install ArduPilot STM32 toolchain" (you flash the official `.apj`, so you don't need the compiler). If it stalls on wxPython from extras.wxpython.org, press Ctrl+C: wxPython 4.2.1 is already installed system-wide, and the venv can see it.

**1.4 Start SITL with the simulated lidar and posts.** In a new terminal:

```bash
apenv
cd $REPO/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -A --serial5=sim:rplidara2 -l 51.8752066,14.6487840,54.15,0 --console --map --out=udp:127.0.0.1:14551
```

The first time ever, add `-w` to wipe the simulated parameters to defaults. Never use `-w` after that, or your settings are lost.

The simulator puts its lidar on SERIAL5. On the real drone the lidar is on SERIAL2 (Phase 4), so never load a SITL parameter file onto the drone.

**1.5 MAVProxy commands.** Type these at the `STABILIZE>` prompt in that same terminal, **one line at a time**, waiting for each reply. Pasting several lines at once glues them together.

```
script /tmp/post-locations.scr
param set SERIAL5_PROTOCOL 11
param set PRX1_TYPE 5
param set AVOID_ENABLE 7
param set AVOID_MARGIN 2
param set OA_TYPE 1
param fetch
```

Wait for `Received ... parameters`, then:

```
param set OA_BR_LOOKAHEAD 5
param set OA_MARGIN_MAX 2
param set OA_DB_OUTPUT 3
param set GUID_OPTIONS 64
reboot
```

The first line draws the posts on the map as blue circles. `GUID_OPTIONS` = 64 is what makes GUIDED mode use BendyRuler; without it GUIDED flies straight through obstacles ([Guided mode options](https://ardupilot.org/copter/docs/ac2_guidedmode.html)). Changing `OA_TYPE` or `GUID_OPTIONS` needs the reboot. `OA_DB_OUTPUT` = 3 shows detected obstacles on the map as red `[OA_DB]` labels.

**1.6 Manual test.** After the reboot reconnects and the console shows GPS and EKF ready:

```
mode guided
arm throttle
takeoff 10
```

Right-click the map at a point straight past a post, about 20–30 m away, and choose **Fly To**. It should bend around the post, keeping about 2 m away. To make the dodge obvious, `param set OA_MARGIN_MAX 4`, then set it back to 2.

**1.7 The same test from Python.** In the repo at `gcs/goto.py`. It arms, takes off and flies to a target, but only once you've put the drone in GUIDED yourself, and it stops the moment you switch out of GUIDED. Targets are capped at 100 m away and 30 m high. It works in SITL (Test 3) and in the field (Test 10, Flight 3).

With SITL from 1.4 running and the drone disarmed, type mode guided at the SITL prompt, then in a second terminal:

```bash
apenv
cd $REPO/gcs
python3 goto.py udpin:0.0.0.0:14551 <lat> <lon> 10
```

Pick a target behind a post; the map shows the latitude and longitude under the mouse.

**Phase 1 passes when:** the drone bends around posts in both 1.6 and 1.7, and `goto.py` prints "arrived". Commit `goto.py`.

## Phase 2: Firmware choice (done)

Use **ArduCopter 4.7.2, stable, board folder `Pixhawk6C`**, the same version as your SITL build ([Copter-4.7.2 announcement](https://discuss.ardupilot.org/t/copter-4-7-2-released/145800)).

| Folder in `Copter/stable/` | Use it? | Why |
| --- | --- | --- |
| `Pixhawk6C` | **Yes** | Standard multirotor firmware; supports DShot600 on the AUX outputs |
| `Pixhawk6C-bdshot` | No | Adds bidirectional DShot (motor RPM feedback), which the stock S500 ESCs don't need |
| `Pixhawk6C-heli`, `Pixhawk6C-bdshot-heli` | No | Traditional single-rotor helicopters |

Inside the folder, check `git-version.txt` says Copter 4.7.2, then download `arducopter.apj` (not `arducopter_with_bl.hex`) into `$REPO/firmware/`. The `.apj` is JSON, so you can read its header without flashing:

```bash
cd $REPO/firmware
python3 -c "import json;d=json.load(open('arducopter.apj'));print(d['board_id'], d.get('git_identity'))"
```

`board_id` must be **56**, the Pixhawk 6C's ID in [ArduPilot's board definition](https://github.com/ArduPilot/ardupilot/blob/1641aa42ec37884caf204195c5d97eb97663e334/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6C/hwdef.dat). If the file won't parse, the download is incomplete.

## Phase 3: Flashing (done)

The board was flashed with ArduPilot's uploader from Ubuntu, and QGroundControl now identifies it as `Pixhawk6C [Holybro]` running ArduCopter. The methods are kept here for future updates.

**Before any flash:** flight battery disconnected, props off, USB cable straight into the laptop, parameters saved.

### Option A: Terminal uploader (recommended, what you used)

Close QGroundControl first; it holds the USB port. Start with the Pixhawk unplugged:

```bash
apenv
cd $REPO/ardupilot
python3 Tools/scripts/uploader.py --port /dev/ttyACM0 $REPO/firmware/arducopter.apj
```

Plug the board in when it says it's waiting for the bootloader. It reports board ID 56, then erases, programs and verifies.

### Option B: QGroundControl stable

1. Unplug the Pixhawk and open Vehicle Setup → **Firmware**.
2. Plug it in. In the dialog choose **ArduPilot**, tick **Advanced settings**, pick **Custom firmware file...**, and select `arducopter.apj`.
3. Wait for erase, program and verify before unplugging.

### Confirm and reset

1. Open QGroundControl. It should connect as an ArduPilot vehicle, and the vehicle messages should show `ArduCopter V4.7.2` and `Pixhawk6C`.
2. Reset to defaults: **Parameters → Tools → Reset all to firmware's defaults**, then **Parameters → Tools → Reboot Vehicle**.
3. Save the parameters as `$REPO/params/s500_ardu_00_fresh.params` and commit.

If a flash fails partway, the bootloader survives. Unplug, wait 10 seconds and repeat.

## Phase 4: Wiring

Every TELEM port on the 6C is a 6-pin JST-GH with 3.3 V logic, and so are the Pi 5's GPIO and the C1, so no level shifters are needed ([Holybro 6C port pinouts](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports)). **Wire with the flight battery disconnected and the Pixhawk unpowered.**

&#91;embedded content: S500 wiring · 6 devices, 3 power paths\]

Only the lidar draws power from the Pixhawk. The Pi runs from its own 5 A supply and shares just ground with the flight controller.

### Port plan

| 6C port | ArduPilot serial | Device | Protocol | Baud |
| --- | --- | --- | --- | --- |
| TELEM1 | SERIAL1 | SiK radio (air unit) | `SERIAL1_PROTOCOL` = 2 (MAVLink2) | `SERIAL1_BAUD` = 57 |
| TELEM2 | SERIAL2 | RPLidar C1 | `SERIAL2_PROTOCOL` = 11 (Lidar360) | `SERIAL2_BAUD` = 460 (460800) |
| TELEM3 | SERIAL5 | Raspberry Pi 5 | `SERIAL5_PROTOCOL` = 2 (MAVLink2) | `SERIAL5_BAUD` = 921 |
| GPS1 | SERIAL3 | GPS/compass | default | default |
| RC IN | none | FS-iA6B (PPM) receiver | detected automatically |  |

The SERIAL numbers don't follow the TELEM numbers: **TELEM3 is SERIAL5** on this board ([ArduPilot Pixhawk 6C page](https://ardupilot.org/copter/docs/common-holybro-pixhawk6C.html)). Getting this wrong is the most common reason a device "doesn't work".

### TELEM pinout (all three ports)

| Pin | Signal | Use here |
| --- | --- | --- |
| 1 (red) | +5 V out | Lidar power only, never the Pi |
| 2 | TX (out) | → device RX |
| 3 | RX (in) | ← device TX |
| 4 | CTS | Not connected |
| 5 | RTS | Not connected |
| 6 | GND | Always connected |

### SiK radio → TELEM1

Plug the Holybro radio's 6-pin cable straight in. TELEM1 has its own 1.5 A power limit, which covers the radio.

### RPLidar C1 → TELEM2

| TELEM2 pin | Signal | C1 signal |
| --- | --- | --- |
| 1 | +5 V | VCC |
| 2 | TX | RX |
| 3 | RX | TX |
| 6 | GND | GND |

- The C1 has a 5-pin XH2.54 socket with VCC, TX, RX and GND ([C1 specifications](https://waveshare.com/wiki/RPLIDAR_C1)). Wire by the signal names printed on the board or in Slamtec's datasheet, not by pin position; third-party diagrams disagree on the order.
- The C1 draws about 230 mA. The 6C's ports other than TELEM1 share a combined 1.5 A limit ([PX4 6C page](https://docs.px4.io/v1.13/zh/flight_controller/pixhawk6c)), so TELEM2 power is within spec. If you see lidar dropouts, move its VCC and GND to a separate 5 V BEC and keep its ground tied to the Pixhawk's.

### Raspberry Pi 5 → TELEM3

| TELEM3 pin | Signal | Pi 5 header pin |
| --- | --- | --- |
| 1 | +5 V | **Leave unconnected** |
| 2 | TX | Pin 10 (GPIO15, RXD) |
| 3 | RX | Pin 8 (GPIO14, TXD) |
| 6 | GND | Pin 6 (GND) |

Pins 4 and 5 stay unconnected; on early 6C units they are I2C. Power the Pi from a dedicated 5 V 5 A BEC into its USB-C port; ArduPilot advises [against powering a companion computer from a TELEM port](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html). The ground wire to Pi pin 6 must still be connected, or the link won't work.

### GPS → GPS1, receiver → RC IN

The GPS stays on GPS1 as it was under PX4. Plug the FS-iA6B's PPM output into **RC IN** with the RC cable that came with the 6C; ArduPilot detects PPM automatically.

### Mounting the lidar

- Flat, with its cable exit pointing to the **rear** ([ArduPilot RPLidar page](https://ardupilot.org/copter/docs/common-rplidar-a2.html)).
- On the S500 the props, arms and GPS mast sit near the scan plane. Mount the C1 on a short mast above the props, with nothing else in its scan plane. Anything it sees all the time is treated as an obstacle (Phase 6 checks this).

### Before powering on

- [x] Every TX goes to an RX, every RX to a TX.
- [x] No 5 V wire between the Pixhawk and the Pi.
- [x] Ground connected Pixhawk ↔ Pi and Pixhawk ↔ lidar.
- [x] Nothing on TELEM3 pins 4 and 5.
- [x] Every BEC output checked with a multimeter before connecting it.
- [x] Props off.

## Phase 5: Base setup in QGroundControl

All in QGroundControl on Ubuntu, Pixhawk on USB. Props off. Setup pages are under **Vehicle Setup** (Vehicle Configuration in newer versions); page names can differ slightly between QGC versions. Parameters are set in **Parameters**: search the name, click it, type the value, **Save**. Reboot with **Parameters → Tools → Reboot Vehicle** when a step says so.

**5.1 Frame.** **Frame** page → Quad, X. That sets:

```
FRAME_CLASS = 1
FRAME_TYPE  = 1
```

**5.2 Serial ports** (Phase 4 port plan). Reboot afterwards.

```
SERIAL1_PROTOCOL = 2
SERIAL1_BAUD     = 57
SERIAL2_PROTOCOL = 11
SERIAL2_BAUD     = 460
BRD_SER2_RTSCTS  = 0
SERIAL5_PROTOCOL = 2
SERIAL5_BAUD     = 921
```

If `BRD_SER5_RTSCTS` appears, set it to 0 too.

**5.3 Calibration** on the **Sensors** and **Radio** pages:

1. **Accelerometer:** follow the six orientations, then **Level Horizon** with the drone flat.
2. **Compass:** rotate the drone through every axis until it finishes, then reboot.
3. **Radio:** transmitter on, receiver bound. Calibrate, moving every stick and switch to its limits.

**5.4 Flight modes.** **Flight Modes** page, on a 3-position switch:

| Position | Mode | Purpose |
| --- | --- | --- |
| 1 | Stabilize | Full manual |
| 2 | Loiter | GPS hold, your normal takeover mode |
| 3 | RTL | Come home |

Flipping this switch during autonomous flight immediately replaces GUIDED. That is your manual override.

**5.5 Battery monitor** (6C power port 1; values from the [ArduPilot 6C page](https://ardupilot.org/copter/docs/common-holybro-pixhawk6C.html)), on the **Power** page or in Parameters:

```
BATT_MONITOR    = 4
BATT_VOLT_PIN   = 8
BATT_CURR_PIN   = 4
BATT_VOLT_MULT  = 18.182
BATT_AMP_PERVLT = 36.364
```

Reboot, then compare QGC's battery voltage with a multimeter on the battery. Adjust `BATT_VOLT_MULT` if they differ by more than 0.1 V.

**5.6 Motor outputs (DShot300 on AUX 1–4).** The ESCs stay wired exactly as under PX4, and DShot needs no ESC calibration. On the 6C, MAIN 1–8 are `SERVO1`–`SERVO8` and AUX 1–8 are `SERVO9`–`SERVO16`. Functions 33–36 are Motor 1–4, and PX4 and ArduPilot number quad X motors the same way (1 front-right, 2 rear-left, 3 front-left, 4 rear-right), so these values copy your PX4 mapping:

```
MOT_PWM_TYPE     = 5
SERVO1_FUNCTION  = 0
SERVO2_FUNCTION  = 0
SERVO3_FUNCTION  = 0
SERVO4_FUNCTION  = 0
SERVO9_FUNCTION  = 35
SERVO10_FUNCTION = 34
SERVO11_FUNCTION = 36
SERVO12_FUNCTION = 33
```

`MOT_PWM_TYPE` = 5 is DShot300. DShot600 made these ESCs pulse every 2 seconds, so don't go back to 6. Reboot, connect the flight battery (props still off), then open the **Motors** page:

1. Test each motor at low throttle. ArduPilot's letters A, B, C, D go clockwise from the front-right motor.
2. Check each letter spins the motor in that position, in the direction ArduPilot's quad X diagram shows: front-right and rear-left counter-clockwise, front-left and rear-right clockwise.
3. Wrong motor spins: swap the `SERVO9`–`SERVO12` values. Wrong direction: swap any two of that motor's three wires, or reverse it in the ESC configurator.
4. Repeat until all four are right. Disconnect the battery.

**5.7 Starting tune.** QGC has no equivalent of Mission Planner's Initial Tune Parameters helper. Use [ArduPilot Methodic Configurator](https://github.com/ArduPilot/MethodicConfigurator), which runs on Linux and works out the starting filter and controller values from your props and battery. Install it in its own environment inside your repo (ignored by Git, so only your parameter files get committed), and close QGC first so it can reach the USB port. Later runs need only the activate line and the last line:

```bash
cd $REPO
grep -qx '.venv-amc/' .gitignore || echo '.venv-amc/' >> .gitignore
python3 -m venv $REPO/.venv-amc
source $REPO/.venv-amc/bin/activate
pip install --upgrade pip
pip install ardupilot_methodic_configurator
ardupilot_methodic_configurator
```

Enter 10-inch props and a 4S battery when it asks for your components, and apply its initial values. Alternatively, boot Windows once and use Mission Planner's Initial Tune Parameters, then save the parameters to your repo.

**5.8 Failsafes and fence.** On the **Failsafes** page or in Parameters. Starting values for a 4S pack:

```
FS_THR_ENABLE   = 1
BATT_LOW_VOLT   = 14.0
BATT_CRT_VOLT   = 13.2
BATT_FS_LOW_ACT = 2
FS_GCS_ENABLE   = 1
FENCE_ENABLE    = 1
FENCE_TYPE      = 7
FENCE_RADIUS    = 100
FENCE_ALT_MAX   = 50
```

That is RTL on RC loss, low battery or ground-station loss, inside a 100 m radius, 50 m ceiling geofence. Copter 4.7 renamed some position parameters to metres; if a name doesn't appear, search for its `_M` variant.

**Phase 5 passes when:** no setup page is marked red, the motor test is right for all four motors, and the only pre-arm warning left is about proximity (fixed in Phase 6). Save `$REPO/params/s500_ardu_05_base.params` and commit.

## Phase 6: Lidar and obstacle avoidance

The goal: a lidar picture that matches reality in every direction, then avoidance on with the values already proven in SITL. Props off. The serial port (TELEM2 = SERIAL2) was set in Phase 5.

**6.1 Enable the lidar** ([ArduPilot RPLidar page](https://ardupilot.org/copter/docs/common-rplidar-a2.html)):

```
PRX1_TYPE   = 5
PRX1_ORIENT = 0
```

Reboot. You should feel a slight vibration as the C1 spins up.

**6.2 Check it's alive.** QGC has no radar view, so use **Analyze Tools → MAVLink Inspector → `DISTANCE_SENSOR`**. ArduPilot reports the lidar as distance readings by direction; the `orientation` field says which: 0 front, 2 right, 4 back, 6 left (odd numbers are the diagonals). If there are no `DISTANCE_SENSOR` messages, or you get "PreArm: check proximity sensor": check TX/RX aren't swapped, `SERIAL2_PROTOCOL` = 11, `SERIAL2_BAUD` = 460, and the C1 has 5 V at VCC. Optional: Mission Planner on Windows shows the same data as a radar picture (Ctrl+F → Proximity).

**6.3 Check the directions.** Hold a box or board about 1.5 m from the drone and watch which \`DISTANCE\_SENSOR\` orientation drops to about 1.5 m:

- [x] In front → shows in front.
- [x] On the right → shows on the right.
- [x] Behind → shows behind.
- [x] On the left → shows on the left.

Everything rotated by a fixed angle means the C1's cable isn't pointing to the rear: set `PRX1_YAW_CORR` or remount. Left and right swapped means `PRX1_ORIENT` is wrong.

**6.4 Mask the airframe.** With nothing around, any permanent red lines are parts of the S500. For each one:

```
PRX1_IGN_ANG1 = <angle of the obstruction, degrees>
PRX1_IGN_WID1 = <width to ignore, degrees>
```

Use `IGN_ANG2`/`IGN_WID2` and so on for more. Keep them narrow: the drone is blind inside them. Remounting clear of the frame is better than masking.

**6.5 Turn on avoidance** (same values as SITL):

```
AVOID_ENABLE = 7
AVOID_MARGIN = 2
AVOID_BEHAVE = 1
OA_TYPE      = 1
```

Refresh the parameters (**Parameters → Tools → Refresh**); the `OA_BR_` parameters appear only after `OA_TYPE` is set. Then:

```
OA_BR_TYPE      = 1
OA_BR_LOOKAHEAD = 5
OA_MARGIN_MAX   = 2
OA_DB_ALT_MIN   = 2
GUID_OPTIONS    = 64
```

**Reboot.** BendyRuler won't run until the Pixhawk restarts.

| Parameter | What it does |
| --- | --- |
| `AVOID_ENABLE` = 7, `AVOID_MARGIN` = 2, `AVOID_BEHAVE` = 1 | Simple avoidance: stop 2 m from obstacles. Works **only in Loiter and AltHold** |
| `OA_TYPE` = 1 | BendyRuler path planning in GUIDED, AUTO and RTL ([BendyRuler page](https://ardupilot.org/copter/docs/common-oa-bendyruler.html)) |
| `GUID_OPTIONS` = 64 | Makes GUIDED use path planning, so BendyRuler applies to your scripts' position targets |
| `OA_BR_LOOKAHEAD` = 5 | Look 5 m ahead when planning |
| `OA_MARGIN_MAX` = 2 | Keep 2 m from obstacles while planning |
| `OA_DB_ALT_MIN` = 2 | Ignore obstacles near home below 2 m, so the ground isn't one during takeoff |

In GUIDED, BendyRuler is the only avoidance, and only for **position targets**. Velocity commands (WASD-style) get none.

**Phase 6 passes when:** the radar view matches reality in all four directions, the frame is clear or masked, pre-arm checks pass, and the parameters above are set. Save `$REPO/params/s500_ardu_06_lidar.param` and commit.

## Phase 7: Raspberry Pi 5 on TELEM3

The goal: the Pi reads the Pixhawk's heartbeat over TELEM3 and sends a body-relative move you've already proven in SITL. Based on ArduPilot's [Raspberry Pi via MAVLink guide](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html), with the Pi 5 differences added.

**7.1 Install the OS.** Raspberry Pi Imager → **Raspberry Pi OS Lite (64-bit)**. In Imager's settings set the hostname (`s500-pi`), your username, bench Wi-Fi, and enable SSH. Boot and SSH in.

**7.2 Enable the GPIO UART.** On the Pi 5, GPIO 14/15 is `/dev/ttyAMA0` and must be switched on; `/dev/serial0` points at the small debug connector instead.

```bash
sudo raspi-config
```

Interface Options → Serial Port → login shell over serial: **No** → serial port hardware: **Yes**. Then:

```bash
echo "dtparam=uart0=on" | sudo tee -a /boot/firmware/config.txt
sudo usermod -a -G dialout $USER
sudo reboot
```

After the reboot, `ls -l /dev/ttyAMA0` should show the port.

**7.3 Software and your repo.** The Pi doesn't need ArduPilot's source, and a plain clone skips the submodule. The repo is public, so the Pi clones it over HTTPS with no keys, and only ever pulls.

```bash
sudo apt update && sudo apt install -y python3-venv git
python3 -m venv ~/drone-venv
source ~/drone-venv/bin/activate
pip install pymavlink MAVProxy
git clone https://github.com/Antonio9381/TronV2ardrupilot.git ~/TronV2ardrupilot
```

**7.4 First contact** (Pixhawk on USB power, Pi on its own supply, TELEM3 wired as in Phase 4):

```bash
source ~/drone-venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600
```

MAVProxy should print the flight mode and load parameters. Stuck at "Waiting for heartbeat": TX/RX swapped, ground missing, `SERIAL5_PROTOCOL`/`SERIAL5_BAUD` not 2/921, or `dtparam=uart0=on` missing.

**7.5 The relative-move script.** In the repo at `pi/nudge.py`. Usage: `nudge.py CONN FORWARD RIGHT YAW UP`, in metres forward (negative = back), metres right (negative = left), degrees of yaw to the right (negative = turn left) and metres up (negative = down). It refuses unless the drone is armed and in GUIDED, caps moves at 5 m and 180°, keeps the current heading unless you ask for yaw, and won't go below 2 m. It reads the lidar first: a move toward an obstacle is shortened to stop 2.5 m short of it, or refused if the drone is already closer. That stops the target landing inside an obstacle, which made BendyRuler wander. With `GUID_OPTIONS` = 64 the moves that are sent are still planned around obstacles.

**Test in SITL first, on the laptop.** Add `--out=udp:127.0.0.1:14552` to the Phase 1.4 command, take off with `mode guided`, `arm throttle`, `takeoff 10`, then:

```bash
apenv
python3 $REPO/pi/nudge.py udpin:0.0.0.0:14552 0 2 0 0
```

The drone moves 2 m to its right. Repeat towards a post: it should route around it rather than hit it.

**On the real Pi** (props off, disarmed): `python3 ~/TronV2ardrupilot/pi/nudge.py /dev/ttyAMA0 0 2 0` 0 should print the mode and refuse. That proves the link and the safety check; the real move happens in Phase 9.

**Phase 7 passes when:** MAVProxy on the Pi sees the Pixhawk, `nudge.py` works in SITL, and on the real Pi it reads the mode and refuses outside GUIDED.

## Phase 8: Ground station (QGroundControl + MAVProxy + your scripts)

Only one program can open the SiK radio's serial port. MAVProxy opens it and shares the link: QGroundControl on UDP 14550, the old scripts on 14551, tron on 14553. This is the same arrangement SITL already uses, so the scripts don't change.

**8.1 Install QGroundControl stable.** Download the stable AppImage from the QGroundControl website (not Daily), then:

```bash
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

In QGC's **Application Settings → General → AutoConnect**, untick **SiK Radio**. Otherwise QGC grabs the radio's port before MAVProxy can. Leave **UDP** ticked.

**8.2 Start the router.** Plug in the ground SiK radio first, then:

```bash
apenv
ls /dev/ttyUSB*
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553
```

Use the device name `ls` showed. When MAVProxy shows the flight mode, open QGroundControl; it connects to UDP 14550 automatically. Keep the MAVProxy terminal open for the whole flight: closing it cuts QGC and the scripts off.

**8.3 The field target script.** In the repo at `gcs/target.py`. Unlike `goto.py`, it never arms or takes off: it sends one target only when the drone is already flying in GUIDED.

Test it in SITL first: take off manually in MAVProxy (`mode guided`, `arm throttle`, `takeoff 10`), then, in another terminal:

```bash
apenv
python3 $REPO/gcs/target.py udpin:0.0.0.0:14551 <lat> <lon> 10
```

In the field the command is identical, with the router from 8.2 running.

**8.4 Where the agent fits.** Once Phase 9 passes, the LLM planner goes in front of these scripts: sentence → objective dictionary → validated in plain code (geofence, altitude, known targets) → `target.py`-style GPS targets from the ground station, and `nudge.py`-style relative moves from the Pi's mission manager. QGroundControl stays on screen the whole time as your view and override.

**Phase 8 passes when:** MAVProxy holds the radio link, QGC shows the drone through it, and `target.py` works against SITL and refuses on the real drone while disarmed.

## Phase 9: Flight tests

Each flight adds exactly one new thing; move on only when the previous one passes. Fly in an open field, within CASA's rules and your club's field rules, with a second person watching.

**Every flight:** one hand near the mode switch. Anything unexpected → **Loiter** (or Stabilize if GPS is the problem), land, read the log before flying again.

1. **Manual hover.** Props on for the first time. Take off in Stabilize, hover at 2–3 m, then try Loiter. *Passes:* no oscillation, holds position in Loiter, all four motors balanced in the log.
2. **Failsafes.** At 10 m in Loiter, switch to RTL: it comes home and lands. Then, with a spotter, turn the transmitter off briefly: RC failsafe RTL. Switch it back on and retake control with the mode switch. *Passes:* both RTLs behave, and you can always retake control.
3. **Lidar in Loiter.** Fly slowly towards a large soft obstacle (foam board, inflatable, a wall with plenty of room). *Passes:* stops about 2 m away however you push the stick.
4. **Go to location from QGC.** Take off in Loiter to 10 m. In QGC, click the flight mode in the top toolbar and choose **Guided**, then click the map 30–50 m away with nothing in between → **Go to location** and confirm the slider. *Passes:* flies there and holds.
5. **Go to location past one obstacle.** Lower the waypoint speed first (`WP``_SPD, about 2 m/s; yours is 3`). Put the obstacle on the line. *Passes:* bends around it, keeping about 2 m away.
6. **Target from your script.** Same as 5, but with `target.py` through the MAVProxy router (Phase 8). *Passes:* "arrived", with avoidance.
7. **Pi relative moves.** Hover in GUIDED at 10 m, SSH into the Pi from close range, run `nudge.py /dev/ttyAMA0 0 2 0` 0. *Passes:* moves about 2 m right; repeated towards the obstacle, it routes around it.
8. **Ground station link loss in GUIDED.** Unplug the ground radio during a target flight. *Passes:* RTL after the GCS failsafe timeout.
9. **Only now: the agent.** Start the mission manager and planner with simple objectives (one target, then target plus search) before adding camera tracking.

After each passing flight, download the log (QGC: **Analyze Tools → Log Download**) into `$REPO/logs/` with that day's parameter file. For plots, open the `.bin` log in UAV Log Viewer (plot.ardupilot.org) or MAVExplorer.

Once the basics are solid, run AutoTune at a safe height for a better-tuned S500, especially before adding a camera payload.

## Testing every script

Run the tests in this order: unit tests, then SITL, then the Pi on the bench, then the real drone on the ground, then flights. Don't move on until the current test passes. Each test lists every terminal from a fresh start, so you can jump straight to one.

| Test | Script | Where | Props | Passes when |
| --- | --- | --- | --- | --- |
| 1 | tron unit tests | Laptop | n/a | 76 passed |
| 2 | SITL session | Laptop | n/a | Console shows GUIDED-ready, 3 outputs |
| 3 | gcs/goto.py | Laptop + SITL | n/a | "arrived", bends round posts |
| 4 | gcs/target.py | Laptop + SITL | n/a | Refuses on the ground, "arrived" in the air |
| 5 | pi/nudge.py | Laptop + SITL | n/a | Moves 2 m right |
| 5b | gcs/wasd.py | Laptop + SITL | n/a | Keys move it, release stops it, obstacle blocks |
| 6 | tron | Laptop + SITL | n/a | Every row of the Test 6 table matches |
| 7 | Old + tron together | Laptop + SITL | n/a | Both work in one session |
| 8 | tron on the Pi | Pi | n/a | 76 passed |
| 9 | All scripts, ground | Real drone | Off | Every script reads the drone and refuses |
| 10 | All scripts, flying | Real drone | On | Each old/new pair does the same move |

Terminal names used below: **L1, L2, L3** are laptop terminals, **P1, P2** are SSH sessions into the Pi. `apenv` is your alias that activates the ArduPilot venv (Phase 1.1); `$REPO` is your repo on the laptop.

**Port map.** Every script listens on its own UDP port, so they never fight over one connection.

| Port | Who listens | Used by |
| --- | --- | --- |
| 14550 | QGroundControl | You, watching |
| 14551 | goto.py, target.py, wasd.py | Old ground scripts |
| 14552 | nudge.py (SITL only) | Old Pi script, tested on the laptop |
| 14553 | tron | New code on the laptop |
| 14560 | tron | New code on the Pi |
| 14561 | nudge.py | Old Pi script on the Pi |

### Test 1: Install tron and run its unit tests (laptop)

This needs no drone and no simulator. The tests fly a simulated drone and a fake autopilot that talks real MAVLink.

1. **L1**, get the latest code and install tron into the ArduPilot venv. You only need the install once per laptop: `-e` means every later `git pull` takes effect without reinstalling.

```bash
cd $REPO
git pull
apenv
cd agent
pip install -e '.[test]'
which tron
```

`which tron` should print a path inside `~/venv-ardupilot`. If `cd $REPO` lands in your home folder, `$REPO` isn't set: redo Phase 1.1 and open a new terminal.

2. **Run the tests**, with `apenv` active and always as `python -m pytest`. Plain `pytest` runs `/usr/bin/pytest` with Ubuntu's own Python, where OpenCV can't load.

```bash
python -m pytest
```

**Passes when:** `76 passed` (about 45 seconds). `74 passed, 1 skipped` means OpenCV couldn't load; that's fine while the drone has no camera.

3. **Check the command line:**

```bash
tron --version
echo '{"lat": 51.8754761, "long": 14.6490749, "alt": 10, "colour": "red"}' > /tmp/obj.json
tron check-objective /tmp/obj.json
echo '{"lat": 51.87, "alt": 10}' > /tmp/bad.json
tron check-objective /tmp/bad.json; echo "exit $?"
```

The first prints the parsed objective. The second logs `objective error: missing long` and `exit 2`.

### Test 2: Start one SITL session for all script tests

Tests 3 to 7 all use this one simulator. It is the Phase 1.4 command with two extra outputs, so every script gets its own port.

1. **L1**, start SITL (no `-w`, so your Phase 1 settings stay):

```bash
apenv
cd $REPO/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -A --serial5=sim:rplidara2 -l 51.8752066,14.6487840,54.15,0 --console --map --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553
```

2. In the same terminal, at the `STABILIZE>` prompt, one line at a time:

```
script /tmp/post-locations.scr
param show GUID_OPTIONS
param show OA_TYPE
param set SIM_BATT_VOLTAGE 16.8
```

`GUID_OPTIONS` must say 64 and `OA_TYPE` 1. If not, redo Phase 1.5. `SIM_BATT_VOLTAGE` makes the simulated battery a full 4S. SITL's default is 12.6 V, and tron refuses to fly below 14.2 V.

3. Wait until the console shows **GPS: OK** and **EKF** is no longer red (about 30 seconds). The map shows the drone at the test field with the blue posts.

**Take off by hand** (used by Tests 4, 5 and 7), at the `STABILIZE>` prompt, one line at a time:

```
mode guided
arm throttle
takeoff 10
```

**Reset between tests:** type `mode rtl` and wait for it to land and disarm. Quit SITL with Ctrl+C in L1 when you're done.

**Picking a target:** move the mouse over the map; the bottom of the map window shows the latitude and longitude under it. Points used below, from the SITL home (51.8752066, 14.6487840):

| Name | Lat | Lon | Where |
| --- | --- | --- | --- |
| A | 51.8754761 | 14.6490749 | 30 m north, 20 m east |
| B | 51.8752066 | 14.6491477 | 25 m east |
| FAR | 51.8758354 | 14.6487840 | 70 m north |
| OUT | 51.8770032 | 14.6487840 | 200 m north, outside tron's 80 m limit |

### Test 3: goto.py in SITL

`goto.py` arms, takes off and flies to a point by itself, but only after you put the drone in GUIDED. These SITL checks must pass before its field test in Test 10.

1. SITL running from Test 2, drone **landed and disarmed**, mode **STABILIZE** (type `mode stabilize` if not).
2. **Refusal checks, L2**, one line at a time:

```bash
apenv
cd $REPO/gcs
python3 goto.py udpin:0.0.0.0:14551 51.8754761 14.6490749 10
python3 goto.py udpin:0.0.0.0:14551 51.8770032 14.6487840 10
python3 goto.py udpin:0.0.0.0:14551 51.8754761 14.6490749 60
```

Expect, in order: `not in GUIDED, refusing ...`, then `target is 200 m away, limit 100 m, refusing`, then `alt must be 2-30 m`. The drone never arms.

3. **The flight.** In L1 type `mode guided` (the drone stays disarmed on the ground). Then in **L2**:

```bash
python3 goto.py udpin:0.0.0.0:14551 51.8754761 14.6490749 10
```

It prints `mode GUIDED, armed False, target 36 m away`, `armed`, `taking off to 10 m`, then counts down `... m to go, alt 10.0 m` and ends with `arrived`. On the map it climbs, then flies to point A round any post.

4. **Already flying.** While it hovers at A, send it to B:

```bash
python3 goto.py udpin:0.0.0.0:14551 51.8752066 14.6491477 10
```

It prints `armed True`, skips the takeoff, and flies straight to B.

5. **Override.** Send it to FAR, and about 5 seconds in type `rc 3 1500` then `mode loiter` in L1:

```bash
python3 goto.py udpin:0.0.0.0:14551 51.8758354 14.6487840 10
```

It must print `stopped: pilot took over (mode LOITER, armed True)` and exit; the drone holds in Loiter.

6. **Passes when:** all three refusals in step 2, `arrived` in steps 3 and 4, and the stop in step 5.
7. Reset: `mode rtl` in L1.

It prints `no heartbeat from the drone` after 30 s if it can't hear SITL: check the L1 command has `--out=udp:127.0.0.1:14551`. If arming is refused, the reason is on the `FC:` lines above it.

### Test 4: target.py in SITL

`target.py` never arms or takes off; it only sends a target when the drone is already flying in GUIDED. This test checks both halves.

1. SITL running from Test 2, drone **landed and disarmed**.
2. **Refusal check, L2:**

```bash
apenv
cd $REPO/gcs
python3 target.py udpin:0.0.0.0:14551 51.8754761 14.6490749 10
```

It must print `refusing: mode STABILIZE, armed False` (or whatever mode it's in) and exit. The drone doesn't move.

3. Take off by hand in L1: `mode guided`, `arm throttle`, `takeoff 10`. Wait until the console shows about 10 m.
4. **L2**, same command again:

```bash
python3 target.py udpin:0.0.0.0:14551 51.8754761 14.6490749 10
```

Still hovering at A, two more checks in **L2**:

```bash
python3 target.py udpin:0.0.0.0:14551 51.8770032 14.6487840 10
python3 target.py udpin:0.0.0.0:14551 51.8758354 14.6487840 10
```

The first must print `target is 172 m away, limit 100 m, refusing` (measured from where the drone is). The second flies toward FAR: about 5 seconds in, type `rc 3 1500` then `mode loiter` in L1. It must print `stopped: pilot took over (mode LOITER, armed True)`. Type  in L1 afterwards.

5. **Passes when:** step 2 refused, the 172 m target was refused, the Loiter switch stopped it, and step 4 counts down `m to go, alt 10.0 m` and ends with `arrived` at point A.
6. Reset: `mode rtl` in L1.

### Test 5: nudge.py in SITL

`nudge.py` is the Pi's relative-move script. On the laptop it listens on 14552 instead of the Pi's serial port, and otherwise runs exactly as on the Pi.

1. SITL running from Test 2, drone **landed and disarmed**.
2. **Refusal check, L2:**

```bash
apenv
python3 $REPO/pi/nudge.py udpin:0.0.0.0:14552 0 2 0 0
```

It prints `mode STABILIZE armed False` (or the current mode) then `not flying in GUIDED, refusing to send`.

3. **Size check, L2:**

```bash
python3 $REPO/pi/nudge.py udpin:0.0.0.0:14552 6 0 0 0
```

It prints `move larger than 5 m, refusing` straight away.

4. Take off by hand in L1: `mode guided`, `arm throttle`, `takeoff 10`.
5. **L2**, the real move:

```bash
python3 $REPO/pi/nudge.py udpin:0.0.0.0:14552 0 2 0 0
```

It prints `mode GUIDED armed True` then `sent`. On the map the drone moves about 2 m to its right (east, if its nose points north).

6. Try `3 0 0` `0 (3 m forward), 0 0 0 2 (2 m up), 0 0 90 0 (turn 90° right) and 0 0 -90 0 (turn back). Then turn to face a post on the map and send 5 0 0 0: it prints obstacle at ... m, move shortened to ... m (or refuses if already within 2.5 m) and stops short without wandering`.
7. **Passes when:** steps 2 and 3 refused, and each move in 5 and 6 went the right direction by about the right distance.
8. Reset: `mode rtl` in L1.

### Test 5b: wasd.py in SITL (keyboard flying)

`gcs/wasd.py` flies the drone from the keyboard, slowly, in a small window. It sends body-frame velocity 10 times a second while the drone is armed in GUIDED; the moment you let go of every key, it sends zero and the drone stops. It sends nothing in any other mode.

| Key | Does |
| --- | --- |
| W / S | forward / back at 0.5 m/s |
| A / D | left / right at 0.5 m/s |
| Q / E | turn left / right at 15°/s |
| R / F | up / down at 0.3 m/s |
| Space | stop |
| Esc or close the window | stop and quit |

Keys combine (W+D goes diagonally, W+E curves). It reads the lidar: a horizontal move toward an obstacle closer than 3 m is blocked and the window says so; without lidar data horizontal moves stay off. F won't take it below 2 m. Velocity commands get no BendyRuler path planning, so this lidar stop is the only avoidance.

1. Install the window library once: `apenv` then `pip install pygame`.
2. SITL running from Test 2. Take off by hand in L1: `mode guided`, `arm throttle`, `takeoff 10`.
3. **L2:**

```bash
apenv
python3 $REPO/gcs/wasd.py udpin:0.0.0.0:14551
```

A small window opens. Click it so it has the keyboard.

4. Hold **W** for 3 seconds: the map shows it creeping forward about 1.5 m. Let go: it stops within about 0.3 m. Try A, D, S, then Q and E (it turns on the spot), then R and F.
5. Turn to face a post and hold W: it stops about 3 m short and the window says `blocked: obstacle ... m that way`.
6. In L1 type `rc 3 1500` then `mode loiter`. The window shows `not sending: mode LOITER`, and keys do nothing. `mode guided` again to carry on.
7. Press **Esc**. It sends one last stop and closes.

**Passes when:** every key moves the right way, releasing stops it, the obstacle blocks W, and nothing is sent outside GUIDED.

### Test 6: tron in SITL

This runs every tron command and trips every safety stop on purpose. tron listens on 14553. It prints one result line, and its exit code is 0 for done, 1 for stopped part way, 2 for refused before sending anything.

1. SITL running from Test 2, drone **landed and disarmed**, `SIM_BATT_VOLTAGE` 16.8.
2. **L3**, make a SITL config with a smaller search square so the search test finishes in about 2 minutes:

```bash
apenv
cd $REPO/agent
printf '[link]\nconn = "udpin:0.0.0.0:14553"\n\n[mission]\nsearch_radius_m = 10\n' > config.sitl.toml
alias t='tron -c config.sitl.toml'
```

3. **Status:**

```bash
t status
```

Prints JSON: `mode`, `armed: false`, `sats` 10 or more, `battery_v` 16.8, and `home` near 51.8752, 14.6488. If it says `no ArduPilot heartbeat`, L1 is missing `--out=udp:127.0.0.1:14553`.

4. **Refuses on the ground:**

```bash
t goto 51.8754761 14.6490749 10; echo "exit $?"
```

Expect `rejected mode is STABILIZE, need GUIDED` (or whatever mode it's in) and `exit 2`. Nothing is sent.

5. **Take off** (works only in SITL; it checks for the simulator first):

```bash
t sitl-takeoff 10
```

The drone arms in GUIDED and climbs; the command returns when it reaches about 10 m.

6. **Goto and nudge:**

```bash
t goto 51.8754761 14.6490749 10
t nudge 0 3
t nudge 2 0 0 2
t nudge 0 0 90
t nudge 0 0 -90
t status
```

Each prints `arrived`. The second nudge goes 2 m forward and 2 m up, the next two turn 90° right on the spot and back, so `status` shows `rel_alt` about 12.

7. **Limits** (each must print `rejected ...`, exit 2, and the drone must not move):

```bash
t goto 51.8770032 14.6487840 10
t goto 51.8754761 14.6490749 48
t nudge 6 0
```

Expect `target 200 m from home, limit 80 m`, `alt 48.0 m outside 3-40 m` and `nudge larger than 5.0 m`.

8. **RC override.** Start a long goto in L3:

```bash
t goto 51.8758354 14.6487840 10
```

While it flies, in L1 type `rc 3 1500` then `mode loiter`. L3 must print `override pilot took over (LOITER, armed=True)` at once, and the drone holds in Loiter. Back in L1: `mode guided`.

9. **Ctrl+C.** Run the same goto again, and press Ctrl+C in L3 halfway. It logs `interrupted, holding position` and the drone stops where it is.
10. **Low battery.** Start a goto back over home:

```bash
t goto 51.8752066 14.6487840 10
```

While it flies, in L1: `param set SIM_BATT_VOLTAGE 14.0`. L3 prints `low_battery battery 14.00 V` and the drone holds. Restore it: `param set SIM_BATT_VOLTAGE 16.8`.

If SITL's own battery failsafe switches to RTL first, L3 prints `override` instead. That's also a pass: either way tron stopped.

11. **Mission, no colour** (fly there and hold):

```bash
echo '{"lat": 51.8752066, "long": 14.6491477, "alt": 10}' > /tmp/obj_b.json
t mission /tmp/obj_b.json
```

Expect `mission ok: phase=transit found=False arrived`.

12. **Mission with a colour.** SITL has no camera, so it flies the whole search square, finds nothing and holds:

```bash
echo '{"lat": 51.8754761, "long": 14.6490749, "alt": 10, "colour": "red", "target": "tarp"}' > /tmp/obj_red.json
t mission /tmp/obj_red.json
```

On the map it flies to A, then an expanding square of 9 points, 5 m apart, pausing 2 s at each. Expect `mission failed: phase=search found=False arrived search finished without a sighting`. That proves the search pattern; finding a target needs the real camera (Test 8).

13. **Finish:**

```bash
t rtl
```

The drone returns and lands.

**Passes when:** every step printed what it says above. The full log of each run is in `$REPO/agent/logs/`, one file per command, newest last (`ls -t logs | head`).

### Test 7: Old scripts and tron side by side in SITL

This is the rehearsal for Test 10: the same moves from both, in one session, taking turns. **Only one script commands at a time**: wait for one to finish before starting the other. `status` is safe to run any time.

1. SITL running from Test 2, landed and disarmed. L2 in `$REPO/gcs` with `apenv`; L3 in `$REPO/agent` with `apenv` and the `t` alias from Test 6.
2. Take off by hand in L1: `mode guided`, `arm throttle`, `takeoff 10`.
3. Run these pairs, one line at a time, waiting for each to finish:

| Step | Old (L2) | New (L3) | Both should |
| --- | --- | --- | --- |
| a | `python3 target.py udpin:0.0.0.0:14551 51.8754761 14.6490749 10` |  | fly to A, `arrived` |
| b |  | `t goto 51.8752066 14.6491477 10` | fly to B, `arrived` |
| c | `python3 target.py udpin:0.0.0.0:14551 51.8752066 14.6487840 10` |  | fly over home |
| d |  | `t goto 51.8754761 14.6490749 10` | fly to A |
| e | `python3 $REPO/pi/nudge.py udpin:0.0.0.0:14552 0 2 0` 0 |  | move 2 m right |
| f |  | `t nudge 0 2` | move 2 m right again |
| g |  | `t status` | read the drone while nothing else commands |

4. **Passes when:** every row did what the last column says, and tron's `status` agrees with the console in L1.
5. Reset: `t rtl`.

### Test 8: tron on the Pi (install, unit tests)

On the bench with home Wi-Fi; no drone needed. tron gets its own venv on the Pi that can see the OpenCV and NumPy installed from apt (the unit tests need them, and a camera will later). Your old `~/drone-venv` stays as it is.

1. **P1**, get the code:

```bash
ssh wrylands@joeyberry.local
cd ~/TronV2ardrupilot
git pull
ls agent
```

2. **P1**, Install OpenCV, NumPy and tron, once:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy
python3 -m venv --system-site-packages ~/tron-venv
source ~/tron-venv/bin/activate
pip install -e ~/TronV2ardrupilot/agent pytest
```

Don't `pip install opencv` into this venv: the apt one must win, or the Pi camera library breaks.

3. **P1**, unit tests:

```bash
cd ~/TronV2ardrupilot/agent
python -m pytest
```

**Passes when:** `76 passed` (about a minute on the Pi).

**For now Test 8 passes at step 3.** The drone has no camera, and `config.pi.toml` has `[camera] source = "none"`. Steps 4 to 6 are for later, once a downward camera is fitted: first `sudo apt install -y python3-picamera2`, then set `source = "picamera2"` in `config.pi.toml`.

4. **P1**, (Later, with a camera) Check the camera is seen:

```bash
rpicam-hello --list-cameras
```

It lists the camera module. Note which one: Camera Module 2 has a 62.2° horizontal view (the default in `config.pi.toml`), Camera Module 3 about 66°. If it differs, add `hfov_deg = 66` under `[camera]` in `config.pi.toml`. A USB webcam instead: set `source = "opencv"`.

5. **P1**, (Later, with a camera) Colour bench check. Point the camera at the floor, then hold something red in view:

```bash
cd ~/TronV2ardrupilot/agent
python3 - <<'EOF'
import time
from tron.detect import ColourDetector, Locator, PiCamera
loc = Locator(PiCamera(), ColourDetector("red"), 62.2)
for _ in range(40):
    print(loc.locate(2.0))
    time.sleep(0.5)
loc.close()
EOF
```

It prints `(forward_m, right_m)` as if the camera were 2 m up, or `None` when it sees nothing red. Move the object toward the top of the picture: forward grows. Move it right: right grows.

6. (Later) **Passes when:** tests pass, `None` with nothing red in view, and the numbers follow the object the right way. If forward or right is reversed, the camera is mounted rotated; tell me how it sits and I'll add a rotation setting.

Push any config change from the Pi only if you set up credentials there; otherwise make the same edit on the laptop and `git pull` on the Pi.

### Test 9: Ground checks on the real drone (props off)

Every script connects to the real drone, reads it correctly, and refuses to move it. Do this at home the day before, and again at the field before the first takeoff.

**Setup:** props off, flight battery in, drone outside or near a window for GPS, disarmed. Pi and laptop on the same Wi-Fi (or the phone hotspot at the field).

1. **L1**, the radio router with the extra port for tron:

```bash
apenv
ls /dev/ttyUSB*
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553
```

Then open QGroundControl; it connects on 14550.

2. **L2**, old ground script:

```bash
apenv
cd $REPO/gcs
python3 target.py udpin:0.0.0.0:14551 0 0 10
```

Must print `refusing: mode LOITER, armed False` (or your current mode). The 0 0 target is never sent.

3. **L3**, tron on the laptop (the default config already listens on 14553):

```bash
apenv
cd $REPO/agent
tron status
```

Compare with QGC: same mode, `sats` matches, `battery_v` within 0.1 V, `home` where the drone is. Then:

```bash
tron goto <lat from status> <lon from status> 10; echo "exit $?"
tron nudge 2 0; echo "exit $?"
tron sitl-takeoff 5; echo "exit $?"
```

The first two print `rejected mode is LOITER, need GUIDED`, exit 2. The third logs `SIM_SPEEDUP not found: this is not SITL, refusing to arm`, exit 2. Also read the `limits:` log line: it must end `path planning True`, and show `radius 80 m, alt 3-40 m` from your fence. If it says `path planning False`, set `GUID_OPTIONS` = 64 and `OA_TYPE` = 1 and reboot.

**Old goto.py, L2**, using the `lat` and `lon` that `tron status` just printed:

```bash
python3 goto.py udpin:0.0.0.0:14551 <lat> <lon> 5
```

Must print `mode LOITER, armed False, target 0 m away`, then `not in GUIDED, refusing ...`. With CH6 low it can never arm the drone.

4. **P1**, on the Pi, MAVProxy owns the Pixhawk's serial port and shares it on two UDP ports. `--source-system 1` stops this MAVProxy pretending to be a ground station, so it can't hide a real loss of your radio link from the GCS failsafe.

```bash
ssh wrylands@joeyberry.local
source ~/drone-venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --source-system 1 --source-component 192 --out udp:127.0.0.1:14560 --out udp:127.0.0.1:14561
```

Wait until it shows the flight mode. Leave it running.

5. **P2**, a second SSH session, old Pi script through the split:

```bash
ssh wrylands@joeyberry.local
source ~/tron-venv/bin/activate
python3 ~/TronV2ardrupilot/pi/nudge.py udpin:127.0.0.1:14561 0 2 0 0
```

Must print the mode, then `not flying in GUIDED, refusing to send`.

6. **P2**, tron on the Pi:

```bash
cd ~/TronV2ardrupilot/agent
tron -c config.pi.toml status
tron -c config.pi.toml nudge 0 2; echo "exit $?"
```

`status` matches QGC; the nudge prints `rejected ...`, exit 2.

7. **Passes when:** all five scripts read the drone, and every move was refused. Leave L1, QGC and P1 running if you're flying next.

### Test 10: Field flights, old script then tron

Each move is flown twice: first with the old script you already trust, then with tron. Run Everyday flight session steps 1–4 and Test 9 first, and keep L1, QGC and P1 running. A spotter watches; your hand stays on the mode switches. **CH6 low** (back to Loiter) stops any script instantly.

**Throttle stick at mid before you flip CH6 low in the air.** In Loiter the throttle stick sets climb rate, so a stick at the bottom makes it descend the moment you take over.

Every old script stops itself when you leave GUIDED, but the last target it sent stays in the flight controller until you do. `nudge.py` returns as soon as it has sent; wait until the drone has stopped (QGC speed near 0) before the next move.

**Flight 1: goto and nudge**

1. Arm, take off in Loiter to about 10 m, and check it holds still hands-off for 10 seconds.
2. Flip **CH6 high**: GUIDED.
3. **L3**, read where it is and work out a point 10 m north:

```bash
tron status
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 10, 0))"
```

Put the `lat` and `lon` from `status` into the second line. It prints the target's lat and lon. Call the start S and the target N.

4. **L2, old:** `python3 target.py udpin:0.0.0.0:14551 <N lat> <N lon> 10`. Passes: flies 10 m north, `arrived`.
5. **L3, new:** `tron goto <S lat> <S lon> 10`. Passes: flies back, `arrived`, exit 0.
6. **P2, old:** `python3 ~/TronV2ardrupilot/pi/nudge.py udpin:127.0.0.1:14561 0 2 0 0`. Passes: about 2 m right.
7. **P2, new:** `tron -c config.pi.toml nudge 0 -2`. Passes: about 2 m left, `arrived`.
8. **Refusal in the air, L3:** work out a point 200 m north (`o(<lat>, <lon>, 200, 0)`) and `tron goto` it. Passes: `rejected target 200 m from home, limit 80 m`, and the drone doesn't move.
9. **Override, L3:** make a point 30 m north and `tron goto` it. After it starts moving, flip **CH6 low**. Passes: it holds in Loiter at once and L3 prints `override pilot took over (LOITER, armed=True)`.
10. Land in Loiter, disarm, battery out. Download the log.

**Flight 2: past an obstacle.** Same as Phase 9 item 5, but run the move as step 4 with `target.py`, then step 5 with `tron goto`, the obstacle on the line both times. Passes: both bend round it at about 2 m.

**Flight 2b: nudge.py in detail** (Pi). Hover at 10 m in GUIDED with at least 10 m clear all round. In **P2** (P1's MAVProxy split running), one line at a time, waiting for the drone to stop each time:

```bash
source ~/tron-venv/bin/activate
cd ~/TronV2ardrupilot/pi
python3 nudge.py udpin:127.0.0.1:14561 3 0 0 0
python3 nudge.py udpin:127.0.0.1:14561 -3 0 0 0
python3 nudge.py udpin:127.0.0.1:14561 0 3 0 0
python3 nudge.py udpin:127.0.0.1:14561 0 -3 0 0
python3 nudge.py udpin:127.0.0.1:14561 0 0 0 2
python3 nudge.py udpin:127.0.0.1:14561 0 0 0 -2
python3 nudge.py udpin:127.0.0.1:14561 0 0 90 0
python3 nudge.py udpin:127.0.0.1:14561 0 0 -90 0
python3 nudge.py udpin:127.0.0.1:14561 6 0 0 0
```

| Command | Should |
| --- | --- |
| `3 0 0 0` | go 3 m the way the nose points |
| `-3 0 0 0` | come back 3 m |
| `0 3 0 0` | go 3 m to its right, nose still pointing the same way |
| `0 -3 0 0` | come back |
| `0 0 0 2` | climb 2 m (QGC altitude about 12 m) |
| `0 0 0 -2` | descend 2 m, back to about 10 m |
| `0 0 90 0` | turn 90° right on the spot |
| `0 0 -90 0` | turn back |
| `6 0 0 0` | print `move larger than 5 m, refusing`, no movement |

Then flip **CH6 low** and run `python3 nudge.py udpin:127.0.0.1:14561 0 2 0 0`: it must print `mode LOITER armed True` and `not flying in GUIDED, refusing to send`. Flip CH6 high again. Last, face the Flight 2 obstacle from about 6 m and send `5 0 0` 0: it must print obstacle at ... m, move shortened to ... m (or refuse if already close), stop about 2.5 m short and hold still, with no wandering. **Passes when:** every row matches and nothing moved on the two refusals.

**Flight 2c: wasd.py** (laptop). Hover at 10 m in GUIDED, at least 10 m clear all round. **L2:** `apenv` then `python3 $REPO/gcs/wasd.py udpin:0.0.0.0:14551`, and click the window.

1. Tap W for 1 second: it creeps forward and stops when you let go.
2. Same for S, A, D, Q, E, R and F, one at a time, each for 1–2 seconds.
3. Face the Flight 2 obstacle from about 6 m and hold W: it must stop about 3 m short, with `blocked` in the window.
4. Flip CH6 low (throttle at mid): the window shows `not sending: mode LOITER` and the keys do nothing. CH6 high again, then Esc.

**Passes when:** every key does what the Test 5b table says, the drone stops every time you let go, and the obstacle blocks W.

**Flight 3: goto.py with automatic takeoff.** Fly this only after Flights 1 and 2 pass. `goto.py` arms and takes off by itself, so the area must be clear for 20 m round both the takeoff spot and the target, with the spotter ready.

1. Drone disarmed on the ground, CH5 on Loiter, **CH6 low**, throttle stick at mid. L1, QGC and L3 running.
2. **L3**, work out a target 15 m north of the drone:

```bash
tron status
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 15, 0))"
```

Call the drone's position S and the printed point T.

3. **L2, ground refusal:** `python3 goto.py udpin:0.0.0.0:14551 <T lat> <T lon> 5`. With CH6 low it must print `not in GUIDED, refusing ...` and never arm.
4. Flip **CH6 high** on the ground. QGC shows Guided; the drone stays disarmed. Everyone stands back.
5. **L2, the flight:** run the same command again. It prints `armed`, `taking off to 5 m`, climbs to 5 m, flies 15 m north, counts down and prints `arrived`, then hovers. If anything looks wrong, at any moment: CH6 low (Loiter). If it's still on the ground after arming, CH6 low and it disarms by itself within seconds.
6. **L2, already flying:** `python3 goto.py udpin:0.0.0.0:14551 <S lat> <S lon> 5`. It prints `armed True`, skips the takeoff and flies back to S.
7. **Override:** make a point 30 m north (`o(<lat>, <lon>, 30, 0)`), send `goto.py` there, and flip CH6 low halfway. It must print `stopped: pilot took over (mode LOITER, armed True)` while the drone holds in Loiter.
8. Land in Loiter, disarm, battery out.

**Passes when:** it refused with CH6 low, took off and arrived on its own, flew back without a second takeoff, and stopped on the switch.

**Flight 4: tron mission to a point** (no camera needed). Without a camera, a mission flies to the objective's point and holds. If the objective has a colour, tron logs `no camera configured: ignoring colour ...` and does the same.

1. Hover at 10 m in GUIDED.
2. **L3**, write an objective 15 m north and check it:

```bash
tron status
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 15, 0))"
echo '{"lat": <printed lat>, "long": <printed lon>, "alt": 10, "colour": "red", "target": "tarp"}' > /tmp/pt.json
tron check-objective /tmp/pt.json
tron mission /tmp/pt.json
```

It logs the colour warning, flies 15 m north and prints `mission ok: phase=transit found=False arrived`, then holds.

3. **P2**, the same from the Pi, back to the start. Use the start position from step 2's `tron status`:

```bash
cd ~/TronV2ardrupilot/agent
echo '{"lat": <start lat>, "long": <start lon>, "alt": 10}' > ~/back.json
tron -c config.pi.toml mission ~/back.json
```

4. **Passes when:** both missions print `mission ok` and end hovering at the point. CH6 low, land.

**Flight 5, later, once a downward camera is fitted: the colour mission** (tron on the Pi with its camera; there is no old equivalent).

1. Lay a red tarp (at least 1 m across) about 10 m north of the takeoff spot.
2. Before takeoff, **P2**, write the objective at the tarp. Use the drone's position from `status` as the start:

```bash
cd ~/TronV2ardrupilot/agent
tron -c config.pi.toml status
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 10, 0))"
echo '{"lat": <tarp lat>, "long": <tarp lon>, "alt": 8, "colour": "red", "target": "tarp"}' > ~/tarp.json
tron check-objective ~/tarp.json
```

3. Take off in Loiter to 8 m, CH6 high.
4. **P2:** `tron -c config.pi.toml mission ~/tarp.json`.
5. Watch: it flies to the point, searches if the tarp isn't under it, then follows the tarp for 30 s and holds.
6. **Passes when:** it ends with `mission ok: phase=track found=True`. Then CH6 low, land.

If a step fails, land and send me the matching log from `$REPO/agent/logs/` or `~/TronV2ardrupilot/agent/logs/`, plus the `.bin` flight log.

## Everyday flight session

Once Tests 1–10 have passed, this is the whole routine for a normal flying day or a demo, in order. Print it or keep it open on your phone.

### 1. The day before, at home

- [ ] Charge the flight batteries (16.8 V), transmitter, laptop and phone. The phone is the field hotspot.
- [ ] Laptop: `cd $REPO && git pull`
- [ ] Pi, on home Wi-Fi: `ssh wrylands@joeyberry.local`, then `cd ~/TronV2ardrupilot && git pull`
- [ ] Pi knows your phone hotspot (once only, hotspot on): `sudo nmcli dev wifi connect "<hotspot name>" password "<hotspot password>"`
- [ ] Weather: light wind, no rain. Field: open, allowed for drones, and you'll keep CASA's recreational rules (below 120 m, in sight, 30 m from other people, daylight).

### 2. At the field: drone and transmitter

- [ ] Props on the right motors (front-right and rear-left counter-clockwise, front-left and rear-right clockwise), tight.
- [ ] Battery strapped down, lidar mast clear, GPS arrow pointing forward, nothing loose near the props.
- [ ] Transmitter switches: CH5 Loiter, CH6 low, CH7 disarmed, CH8 low, throttle down.
- [ ] Transmitter on first, then the flight battery. Leave the drone still and level for 30 seconds.
- [ ] Phone hotspot on; laptop joined to it.

### 3. Laptop: start everything

**L1, radio router** (ground SiK radio plugged in first; use the device `ls` shows):

```bash
apenv
ls /dev/ttyUSB*
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553
```

**QGroundControl:** start the AppImage; it connects on 14550.

**L3, tron:**

```bash
apenv
cd $REPO/agent
tron status
```

**P1, Pi router** (only if the demo uses the Pi):

```bash
ssh wrylands@joeyberry.local
source ~/drone-venv/bin/activate
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --source-system 1 --source-component 192 --out udp:127.0.0.1:14560 --out udp:127.0.0.1:14561
```

**P2, Pi tron:**

```bash
ssh wrylands@joeyberry.local
source ~/tron-venv/bin/activate
cd ~/TronV2ardrupilot/agent
tron -c config.pi.toml status
```

Leave L1 and P1 open all session: closing either cuts the link.

### 4. Pre-flight checks

- [ ] QGC top bar says Ready. If not, read the pre-arm message; don't override it.
- [ ] GPS 10+ satellites, HDOP under 1.0, the home H where the drone is.
- [ ] Battery 16.4–16.8 V in QGC, and `tron status` agrees.
- [ ] The drone's arrow on the QGC map points where its nose points.
- [ ] Lidar: MAVLink Inspector → DISTANCE\_SENSOR readings look sensible.
- [ ] Flipping CH6 shows Guided in QGC and back to Loiter. Leave it low.
- [ ] In L3, `tron nudge 0 0` prints `rejected mode is LOITER, need GUIDED`. If it says `path planning off` instead, set `GUID_OPTIONS` = 64 and `OA_TYPE` = 1 and reboot.

### 5. Fly the demo

1. Spotter in place, everyone else at least 30 m back and behind you.
2. Arm with CH7, take off in Loiter to about 10 m, and let it hold hands-off for 10 seconds.
3. Flip **CH6 high** (GUIDED).
4. In L3, get the drone's position and make target points. Example for 15 m north (N) and 15 m east (E):

```bash
tron status
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 15, 0))"
python3 -c "from tron.geo import offset_latlon as o; print(*o(<lat>, <lon>, 0, 15))"
```

5. Run any of these, one at a time, waiting for each result line:

| Demo | Command | What people see |
| --- | --- | --- |
| Go there | `tron goto <N lat> <N lon> 10` | Flies 15 m north and holds |
| Square dance | `tron goto <E lat> <E lon> 10`, then back to start | Flies a corner and returns |
| Small moves | `tron nudge 3 0`, `tron nudge 0 3`, `tron nudge 0 0 0 2, tron nudge 0 0 90` | 3 m forward, 3 m right, 2 m up, a 90° turn |
| Fly it like a game | L2: python3 $REPO/gcs/wasd.py udpin:0.0.0.0:14551, then WASD, Q/E, R/F | Slow keyboard flying; stops the moment you let go |
| Dodge | `tron goto` to a point behind an obstacle | Bends round it, about 2 m clear |
| Mission | `tron mission /tmp/pt.json` (objective file as in Test 10, Flight 4) | Flies to the objective and holds |
| The Pi does it | P2: `tron -c config.pi.toml nudge 0 -3` | Moves 3 m left, commanded from the drone itself |
| Come home | `tron rtl` | Climbs to 15 m, returns, lands, disarms |

6. **Auto take-off demo** (optional, from the ground): land, CH6 low, then follow Test 10 Flight 3 with `goto.py`.
7. Finish with `tron rtl`, or CH6 low (throttle at mid) and land in Loiter. It disarms by itself after landing; otherwise CH7.
8. Land by about 8 minutes or 15.0 V, whichever comes first. tron refuses below 14.2 V, and the drone RTLs by itself at 14.0 V.

**If anything looks wrong:**

| Situation | Do |
| --- | --- |
| Drone wandering or a command looks wrong | Throttle stick to mid, then CH6 low: instantly back to Loiter |
| Unsure where it is, or drifting away | CH5 to RTL |
| About to hit someone or something | CH8 emergency stop: motors cut, it falls. Last resort only |
| Want to stop a tron command | Ctrl+C in its terminal: it holds position |
| Want to stop goto.py, target.py or nudge.py | CH6 low; Ctrl+C alone leaves the drone flying to its last target |

### 6. Packing up

- [ ] Flight battery out before stopping anything on the laptop.
- [ ] Ctrl+C in P1 and L1, close QGC.
- [ ] QGC Analyze Tools → Log Download: newest log into `$REPO/logs/`.
- [ ] Changed a parameter? Save `params/YYYY-MM-DD_NN_what-changed.params` and move the old one into `params/archive/`.
- [ ] `cd $REPO && git add params agent gcs pi && git commit -m "Flight YYYY-MM-DD" && git push`
- [ ] One line in your notes: what worked, what to change.

## Troubleshooting

Everything that has come up so far, and the fix.

| Symptom | Cause | Fix |
| --- | --- | --- |
| `git push` rejected, "fetch first" | GitHub repo was created with its own commit (README) | `git pull origin main --allow-unrelated-histories`, resolve any conflict, then `git push -u origin main` |
| Install script stalls on wxPython | extras.wxpython.org not responding | Ctrl+C; wxPython 4.2.1 is already installed system-wide |
| `No such file or directory: 'mavproxy.py'` | ArduPilot venv not active | `apenv` before `sim_vehicle.py` |
| `apenv: command not found` | Alias not loaded in that terminal | `. ~/.bashrc` |
| `[ble: detached]`, garbled prompt after a build | Prompt add-on confused by long output | Close the terminal and open a new one |
| Prompt shows `detached at APMrover2-stable` | Another tag on the same commit | Check `git -C $REPO/ardupilot tag --points-at HEAD` lists Copter-4.7.2 |
| `can't convert ... to float`, `Usage: takeoff ...` | Several MAVProxy lines pasted at once | Type one line at a time |
| `Unable to find parameter 'OA_BR_LOOKAHEAD'` | Set before `param fetch` finished | Wait for `Received ... parameters`, then set it |
| `PreArm: BendyRuler OA requires reboot` | `OA_TYPE` changed | `reboot` (SITL) or reboot the Pixhawk |
| Drone flies straight through obstacles in GUIDED | `GUID_OPTIONS` not 64 | `GUID_OPTIONS` = 64, reboot |
| Settings gone after restarting SITL | Started with `-w` | Never use `-w` after the first run |
| QGC Firmware page has no ArduPilot option | QGC Daily build | Use QGC stable, or the uploader (Phase 3) |
| `Permission denied` on `/dev/ttyACM0` or `/dev/ttyUSB0` | dialout group not active yet | Log out and back in |
| Uploader can't see the board | QGC or ModemManager holds the port | Close QGC; `sudo apt remove -y modemmanager` |
| "PreArm: check proximity sensor" | No lidar data | TX/RX, `SERIAL2_PROTOCOL` = 11, `SERIAL2_BAUD` = 460, 5 V at the C1 |
| Pi stuck at "Waiting for heartbeat" | Link not set up | TX/RX, ground, `SERIAL5_PROTOCOL`/`SERIAL5_BAUD` = 2/921, `dtparam=uart0=on` |
| MAVProxy can't open the radio in the field | QGC auto-connected to it first | Untick SiK Radio in QGC's AutoConnect; start MAVProxy before QGC |
| QGC shows no vehicle in the field | Router not running, or UDP autoconnect off | Start the 8.2 router; tick UDP in QGC's AutoConnect |

**Script and tron messages**

| Message | Cause | Fix |
| --- | --- | --- |
| `no heartbeat from the drone` / `no ArduPilot heartbeat` | Nothing on that UDP port | Check the MAVProxy (or SITL) command has the `--out` for that port |
| `not in GUIDED, refusing` / `need GUIDED` | Working as designed | Flip CH6 high (SITL: `mode guided`) |
| `rejected path planning off` | `GUID_OPTIONS` not 64 or `OA_TYPE` not 1 | Set both, reboot |
| `rejected battery low (12.60 V)` in SITL | SITL simulates 3S by default | `param set SIM_BATT_VOLTAGE 16.8` |
| `rejected GPS not good enough` | Under 10 satellites or no 3D fix | Wait in the open; check the Pi isn't next to the GPS |
| `rejected too low` | Below 2 m | Climb in Loiter first |
| `arming refused` from goto.py | A pre-arm check failed | Read the `FC:` lines above it, or QGC's messages |
| `stopped: pilot took over` / `override` | Mode left GUIDED | Working as designed |
| Pi camera import error after pip | pip's OpenCV shadows apt's | `pip uninstall opencv-python opencv-python-headless` in `~/tron-venv` |
| `pytest` can't find tron, or numpy.core.multiarray failed to import | Ran the system pytest, outside the venv | `apenv` (laptop) or `source ~/tron-venv/bin/activate` (Pi) |
| cd $REPO goes to your home folder | REPO not set in that terminal | Redo Phase 1.1, then . \~/.bashrc or open a new terminal; echo $REPO must print the repo path |
| no lidar data, can't check the way is clear | No DISTANCE\_SENSOR messages (lidar off, or SITL started without --serial5=sim:rplidara2) | Fix the lidar (Phase 6). Pure yaw and up/down nudges still work |
| obstacle ... m that way, refusing / move shortened | Lidar sees something within 2.5 m of where the move would end | Working as designed; move another way or nudge back first |
| wasd.py: ModuleNotFoundError: pygame | pygame not installed in this venv | apenv, then pip install pygame |
| wasd.py keys do nothing | Window not focused, or the drone isn't armed in GUIDED | Click the window; read its first line for the mode |

## Sources

ArduPilot documentation:

- [Holybro Pixhawk 6C](https://ardupilot.org/copter/docs/common-holybro-pixhawk6C.html): UART mapping, battery values, firmware folder
- [Guided mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html): `GUID_OPTIONS` bit 6 enables path planning in GUIDED
- [Object avoidance with BendyRuler](https://ardupilot.org/copter/docs/common-oa-bendyruler.html)
- [Simple object avoidance](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html): Loiter and AltHold only
- [RPLidar A1/A2/C1/S1 setup](https://ardupilot.org/copter/docs/common-rplidar-a2.html)
- [Communicating with Raspberry Pi via MAVLink](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)
- [Setting up the build environment](https://ardupilot.org/dev/docs/building-setup-linux.html) and [SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [Simulated peripherals](https://ardupilot.org/dev/docs/adding_simulated_devices.html): simulated 360° lidar and posts
- [ArduPilot Methodic Configurator](https://github.com/ArduPilot/MethodicConfigurator): starting tune values on Linux
- [Copter stable firmware](https://firmware.ardupilot.org/Copter/stable/)
- Optional Mission Planner extras: [loading firmware](https://ardupilot.org/copter/docs/common-loading-firmware-onto-pixhawk.html), [advanced tools](https://ardupilot.org/planner/docs/common-mp-tools.html) (proximity view)

Other sources:

- [Copter-4.7.2 release announcement](https://discuss.ardupilot.org/t/copter-4-7-2-released/145800) (ArduPilot forum)
- [Pixhawk6C board definition](https://github.com/ArduPilot/ardupilot/blob/1641aa42ec37884caf204195c5d97eb97663e334/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6C/hwdef.dat): board ID 56
- [Pixhawk 6C port pinouts](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports) (Holybro)
- [Pixhawk 6C power limits](https://docs.px4.io/v1.13/zh/flight_controller/pixhawk6c) (PX4 docs)
- [RPLIDAR C1 specifications](https://waveshare.com/wiki/RPLIDAR_C1) (Waveshare)
