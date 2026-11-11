import math
import sys
import time

from pymavlink import mavutil

mav = mavutil.mavlink
MAX_ALT = 30
MAX_DIST = 100


def usage():
    sys.exit("usage: goto.py CONN LAT LON ALT")


def recv(m, types, timeout):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=types, blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_srcSystem() != m.target_system or msg.get_srcComponent() != m.target_component:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("FC:", msg.text)
            continue
        return msg
    return None


def heartbeat_state(hb):
    return mavutil.mode_string_v10(hb), bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)


def command(m, cmd, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
    end = time.time() + 5
    while time.time() < end:
        ack = recv(m, ["COMMAND_ACK", "STATUSTEXT"], end - time.time())
        if ack is not None and ack.command == cmd:
            return ack.result
    return None


def distance(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * 111320
    de = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def watch(m, until, timeout):
    end = time.time() + timeout
    while time.time() < end:
        msg = recv(m, ["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"], 3)
        if msg is None:
            sys.exit("lost telemetry, stopping (drone keeps its last target; take over with the mode switch)")
        if msg.get_type() == "HEARTBEAT":
            mode, armed = heartbeat_state(msg)
            if mode != "GUIDED" or not armed:
                sys.exit(f"stopped: pilot took over (mode {mode}, armed {armed})")
        elif until(msg):
            return msg
    sys.exit("timed out; take over with the mode switch")


if len(sys.argv) != 5:
    usage()
conn = sys.argv[1]
lat, lon, alt = (float(v) for v in sys.argv[2:5])
if not 2 <= alt <= MAX_ALT:
    sys.exit(f"alt must be 2-{MAX_ALT} m")

m = mavutil.mavlink_connection(conn, source_system=255, source_component=190)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
    if hb is None:
        sys.exit("no heartbeat from the drone")
    if hb.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break

m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, 0, 0, 0, 0, 0)
pos = recv(m, ["GLOBAL_POSITION_INT", "STATUSTEXT"], 10)
if pos is None:
    sys.exit("no position from the drone")
d = distance(pos.lat / 1e7, pos.lon / 1e7, lat, lon)
if d > MAX_DIST:
    sys.exit(f"target is {d:.0f} m away, limit {MAX_DIST} m, refusing")

mode, armed = heartbeat_state(hb)
print(f"mode {mode}, armed {armed}, target {d:.0f} m away")
if mode != "GUIDED":
    sys.exit("not in GUIDED, refusing (flip the GUIDED switch, or 'mode guided' in SITL)")

if not armed:
    result = command(m, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if result != mav.MAV_RESULT_ACCEPTED:
        sys.exit(f"arming refused (result {result}); read the FC messages above")
    print("armed")
    result = command(m, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    if result != mav.MAV_RESULT_ACCEPTED:
        sys.exit(f"takeoff refused (result {result}); it will disarm by itself in a few seconds")
    print(f"taking off to {alt:.0f} m")
    watch(m, lambda p: p.get_type() == "GLOBAL_POSITION_INT" and p.relative_alt / 1000 >= alt * 0.9, 60)
elif pos.relative_alt / 1000 < 1:
    sys.exit("armed but on the ground; disarm and run again")

m.mav.set_position_target_global_int_send(
    0, m.target_system, m.target_component,
    mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b110111111000, int(lat * 1e7), int(lon * 1e7), alt,
    0, 0, 0, 0, 0, 0, 0, 0)


def arrived(p):
    if p.get_type() != "GLOBAL_POSITION_INT":
        return False
    d = distance(p.lat / 1e7, p.lon / 1e7, lat, lon)
    print(f"{d:6.1f} m to go, alt {p.relative_alt / 1000:5.1f} m")
    return d < 2


watch(m, arrived, 30 + 3 * d)
print("arrived")
