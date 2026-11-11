import math
import sys
import time

from pymavlink import mavutil

mav = mavutil.mavlink
MAX_MOVE = 5
MAX_YAW = 180
MIN_ALT = 2
CLEARANCE = 2.5
HALF_WIDTH = 1.0
POSITION_YAW_MASK = 0b100111111000

USAGE = """usage: nudge.py CONN FORWARD RIGHT YAW UP
  FORWARD  metres, negative = back
  RIGHT    metres, negative = left
  YAW      degrees, negative = turn left
  UP       metres, negative = down"""


def free_distance(sectors, bearing):
    free = math.inf
    for k, d in sectors.items():
        diff = math.radians((k * 45 - bearing + 180) % 360 - 180)
        if abs(diff) < math.pi / 2 and abs(d * math.sin(diff)) < HALF_WIDTH:
            free = min(free, d * math.cos(diff))
    return free


if len(sys.argv) != 6:
    sys.exit(USAGE)
conn = sys.argv[1]
try:
    fwd, right, yaw, up = (float(v) for v in sys.argv[2:6])
except ValueError:
    sys.exit(USAGE)
if max(abs(fwd), abs(right), abs(up)) > MAX_MOVE:
    sys.exit(f"move larger than {MAX_MOVE} m, refusing")
if abs(yaw) > MAX_YAW:
    sys.exit(f"yaw larger than {MAX_YAW} degrees, refusing")

m = mavutil.mavlink_connection(conn, baud=921600, source_system=1, source_component=191)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    if hb is None:
        sys.exit("no heartbeat from the drone")
    if hb.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break

mode = mavutil.mode_string_v10(hb)
armed = bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
print("mode", mode, "armed", armed)
if mode != "GUIDED" or not armed:
    sys.exit("not flying in GUIDED, refusing to send")

for msg_id in (mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mav.MAVLINK_MSG_ID_DISTANCE_SENSOR, mav.MAVLINK_MSG_ID_SYS_STATUS):
    m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, 200000, 0, 0, 0, 0, 0)

rel_alt = None
lidar_healthy = None
sectors = {}
start = time.time()
while time.time() - start < 3:
    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "DISTANCE_SENSOR", "SYS_STATUS"], blocking=True, timeout=0.5)
    if msg is None or msg.get_srcSystem() != m.target_system or msg.get_srcComponent() != m.target_component:
        continue
    t = msg.get_type()
    if t == "GLOBAL_POSITION_INT":
        rel_alt = msg.relative_alt / 1000
    elif t == "SYS_STATUS":
        bit = mav.MAV_SYS_STATUS_SENSOR_PROXIMITY
        lidar_healthy = bool(msg.onboard_control_sensors_present & bit
                             and msg.onboard_control_sensors_enabled & bit
                             and msg.onboard_control_sensors_health & bit)
    elif msg.orientation <= 7:
        valid = 0 < msg.current_distance < msg.max_distance
        sectors[msg.orientation] = (msg.current_distance if valid else msg.max_distance) / 100
    if rel_alt is not None and lidar_healthy is not None and time.time() - start > 1.5:
        break

if rel_alt is None:
    sys.exit("no position from the drone, refusing")
if rel_alt + up < MIN_ALT:
    sys.exit(f"would end {rel_alt + up:.1f} m up, below {MIN_ALT} m, refusing")

horiz = math.hypot(fwd, right)
if horiz > 0:
    if not lidar_healthy:
        sys.exit("lidar not healthy, can't check the way is clear, refusing")
    free = free_distance(sectors, math.degrees(math.atan2(right, fwd)))
    allowed = free - CLEARANCE
    if allowed < 0.3:
        sys.exit(f"obstacle {free:.1f} m that way, refusing")
    if horiz > allowed:
        fwd, right = fwd * allowed / horiz, right * allowed / horiz
        print(f"obstacle at {free:.1f} m, move shortened to {allowed:.1f} m")

m.mav.set_position_target_local_ned_send(
    0, m.target_system, m.target_component,
    mav.MAV_FRAME_BODY_OFFSET_NED, POSITION_YAW_MASK,
    fwd, right, -up, 0, 0, 0, 0, 0, 0, math.radians(yaw), 0)
print(f"sent: forward {fwd:.1f} m, right {right:.1f} m, yaw {yaw:.0f} deg, up {up:.1f} m")
