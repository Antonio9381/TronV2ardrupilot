import sys
from pymavlink import mavutil

conn = sys.argv[1]
fwd, right, down = (float(v) for v in sys.argv[2:5])
if max(abs(fwd), abs(right), abs(down)) > 5:
    sys.exit("move larger than 5 m, refusing")

m = mavutil.mavlink_connection(conn, baud=921600, source_system=1, source_component=191)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True)
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break

mode = mavutil.mode_string_v10(hb)
print("mode", mode)
if mode != "GUIDED":
    sys.exit("not in GUIDED, refusing to send")

m.mav.set_position_target_local_ned_send(
    0, m.target_system, m.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    0b110111111000,
    fwd, right, down, 0, 0, 0, 0, 0, 0, 0, 0)
print("sent")
