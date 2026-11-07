import sys, time, math
from pymavlink import mavutil

lat, lon, alt = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

m = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
m.wait_heartbeat()
m.set_mode("GUIDED")

while not m.motors_armed():
    m.arducopter_arm()
    time.sleep(2)
    m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)

m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

while True:
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if p.relative_alt / 1000 > alt * 0.9:
        break

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
    print(f"{d:6.1f} m to go")
    if d < 2:
        break
print("arrived")
