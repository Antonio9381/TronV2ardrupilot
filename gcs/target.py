import math
import sys
import time

from pymavlink import mavutil

mav = mavutil.mavlink
MAX_DIST = 100

if len(sys.argv) != 5:
    sys.exit("usage: target.py CONN LAT LON ALT")
conn = sys.argv[1]
lat, lon, alt = (float(v) for v in sys.argv[2:5])
if not 2 <= alt <= 40:
    sys.exit("alt must be 2-40 m")

m = mavutil.mavlink_connection(conn, source_system=255, source_component=190)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
    if hb is None:
        sys.exit("no heartbeat from the drone")
    if hb.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break

mode = mavutil.mode_string_v10(hb)
armed = bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
if mode != "GUIDED" or not armed:
    sys.exit(f"refusing: mode {mode}, armed {armed}")


def distance(p):
    dn = (lat - p.lat / 1e7) * 111320
    de = (lon - p.lon / 1e7) * 111320 * math.cos(math.radians(lat))
    return math.hypot(dn, de)


def recv(types, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=types, blocking=True, timeout=0.5)
        if msg and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == m.target_component:
            return msg
    sys.exit("lost telemetry, stopping (drone keeps its last target; take over with the mode switch)")


m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, 0, 0, 0, 0, 0)
start = distance(recv("GLOBAL_POSITION_INT", 10))
if start > MAX_DIST:
    sys.exit(f"target is {start:.0f} m away, limit {MAX_DIST} m, refusing")

m.mav.set_position_target_global_int_send(
    0, m.target_system, m.target_component,
    mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b110111111000, int(lat * 1e7), int(lon * 1e7), alt,
    0, 0, 0, 0, 0, 0, 0, 0)

deadline = time.time() + 30 + 3 * start
while time.time() < deadline:
    msg = recv(["HEARTBEAT", "GLOBAL_POSITION_INT"])
    if msg.get_type() == "HEARTBEAT":
        mode = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
        if mode != "GUIDED" or not armed:
            sys.exit(f"stopped: pilot took over (mode {mode}, armed {armed})")
        continue
    d = distance(msg)
    print(f"{d:6.1f} m to go, alt {msg.relative_alt / 1000:5.1f} m")
    if d < 2:
        print("arrived")
        break
else:
    sys.exit("timed out; take over with the mode switch")
