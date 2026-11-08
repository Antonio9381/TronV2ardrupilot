import sys, math
from pymavlink import mavutil

conn = sys.argv[1]
lat, lon, alt = (float(v) for v in sys.argv[2:5])

m = mavutil.mavlink_connection(conn)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True)
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break

mode = mavutil.mode_string_v10(hb)
armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
if mode != "GUIDED" or not armed:
    sys.exit(f"refusing: mode {mode}, armed {armed}")

m.mav.set_position_target_global_int_send(
    0, m.target_system, m.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b110111111000, int(lat * 1e7), int(lon * 1e7), alt,
    0, 0, 0, 0, 0, 0, 0, 0)

while True:
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    dn = (lat - p.lat / 1e7) * 111320
    de = (lon - p.lon / 1e7) * 111320 * math.cos(math.radians(lat))
    d = math.hypot(dn, de)
    print(f"{d:6.1f} m to go, alt {p.relative_alt / 1000:5.1f} m")
    if d < 2:
        break
print("arrived")
