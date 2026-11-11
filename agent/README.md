# tron

Companion-side control for the Tron S500. It takes an objective (`lat`, `long`, `alt`, `colour`, `target`), flies to it in GUIDED, searches with an expanding square, then tracks the colour with small body-frame steps. The Pixhawk keeps doing the avoidance (BendyRuler via `GUID_OPTIONS` 64).

## Safety behaviour

- It never arms or takes off on real hardware. `sitl-takeoff` refuses unless `SIM_SPEEDUP` exists.
- Every command checks that the vehicle is in GUIDED and armed, with a fresh heartbeat and position, home set, ≥10 sats, the battery above `min_battery_v`, and path planning on.
- Targets are limited to the FC fence minus a margin, and to `alt_min_m`–`FENCE_ALT_MAX - 5`. One leg can be at most 150 m, one nudge at most 5 m and 180°.
- Nudges read the lidar first: a move toward an obstacle is shortened to stop 2.5 m short of it, or refused if it's already closer. If the flight controller reports the lidar unhealthy, only pure yaw/up/down nudges are allowed. An open field with nothing in range is fine: ArduPilot sends no readings then, and that counts as clear.
- Nudges hold the current heading unless you ask for yaw, so forward stays forward.
- If you change mode on the RC, it stops instantly and sends nothing more.
- No progress for 20 s or a timeout → hold. Low battery → hold (RTL in a mission).
- Ctrl+C → hold.
- It does not send heartbeats by default, so it cannot mask a real GCS failsafe.

## Install

```bash
cd $REPO/agent
python3 -m venv .venv && source .venv/bin/activate
pip install -e '.[test]'
pytest
```

On the Pi: `sudo apt install -y python3-opencv python3-numpy`, then `python3 -m venv --system-site-packages ~/tron-venv` and `pip install -e . pytest`.

The drone has no camera yet, so `[camera] source = "none"` everywhere. A mission then ignores `colour` and just flies to the point and holds. When a downward camera is fitted, `sudo apt install python3-picamera2` and set `source = "picamera2"` (or `"opencv"` for a USB webcam); the camera must point straight down with the top of the image toward the nose.

## Try it on SITL first

```bash
sim_vehicle.py -v ArduCopter --console --map --out udp:127.0.0.1:14553
param set GUID_OPTIONS 64
param set OA_TYPE 1
param set SIM_BATT_VOLTAGE 16.8
reboot
tron sitl-takeoff 10 && tron goto -33.88773 151.187 10
```

Use your SITL home coordinates. This exact flow was tested against ArduCopter 4.7.2 SITL.

## Commands

```bash
tron status
tron goto -33.8881 151.1871 10
tron nudge 2 0          # FORWARD RIGHT [YAW] [UP]: metres, degrees; negative = back, left, turn left, down
tron nudge 0 0 90       # turn 90° right on the spot
tron nudge 0 0 0 2      # climb 2 m
tron hold | rtl | land
tron check-objective objective.json
tron mission objective.json
tron sitl-takeoff 10    # SITL only
```

`objective.json`:

```json
{"lat": -33.8881, "long": 151.1871, "alt": 10, "colour": "red", "target": "tarp"}
```

Exit codes: 0 = done, 1 = aborted, 2 = rejected. Logs go to `logs/`.

## Running next to the old scripts

Only one program can open a serial port, so add one more UDP output for `tron`.

Laptop (SiK radio), with QGC on 14550, the old scripts on 14551 and `tron` on 14553:

```bash
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 \
  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553
tron status
```

On the Pi (TELEM3), MAVProxy owns the UART and splits it:

```bash
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --source-system 1 --source-component 192 \
  --out udp:127.0.0.1:14560 --out udp:127.0.0.1:14561
tron -c config.pi.toml status
python3 ../pi/nudge.py udpin:127.0.0.1:14561 0 2 0 0
```

`--source-system 1` stops the Pi's MAVProxy posing as a ground station, so it can't hide a real radio-link loss from the GCS failsafe.

**Only one program commands the drone at a time.** Use `status` on both freely. Send movement commands from one script, wait for it to finish, then try the other.

## Field test order

1. `tron status` on the ground: check mode, sats, home and battery against QGC.
2. Take off in Loiter to 5 m, switch to GUIDED.
3. Old `target.py`, then `tron goto` to the same point 10 m away.
4. Old `nudge.py 2 0 0 0`, then `tron nudge 2 0`.
5. `tron goto` toward a point beyond the fence. It must print `rejected`.
6. Run `tron goto` to 40 m away and flick to Loiter mid-flight. It must print `override`.
7. `tron mission` to a point (no camera yet, so no colour search).

Abort at any point by switching to Loiter on the RC.
