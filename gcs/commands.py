"""Every command the LLM (or you) can give the drone, on one page.

Each command does the same four things:
  1. check the safety rules
  2. send one MAVLink message
  3. watch telemetry until it has happened
  4. return done / blocked / failed, with a reason

Try one by hand:  python3 commands.py takeoff 5
"""
import sys, time
from pymavlink import mavutil

# ---- limits (the safety rules, all in one place) ----
MAX_ALT_M = 30          # never climb above this
MIN_BATTERY_PCT = 15    # no take-off below this

# ---- connection ----
# Simulator and field both use this address: MAVProxy feeds it in both cases.
ADDRESS = "udpin:0.0.0.0:14551"
drone = None


def connect(address=ADDRESS):
    """Open the link and wait until the flight controller answers."""
    global drone
    drone = mavutil.mavlink_connection(address)
    heartbeat()
    return done("connected")


# ---- reading telemetry ----
def read(kind, timeout=3):
    """Return the newest message of one kind, e.g. 'GLOBAL_POSITION_INT'."""
    while drone.recv_match(blocking=False):      # throw away anything old
        pass
    return drone.recv_match(type=kind, blocking=True, timeout=timeout)


def heartbeat():
    """The flight controller's heartbeat (ignores QGroundControl's and MAVProxy's)."""
    while True:
        hb = drone.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        if hb is None:
            raise TimeoutError("no heartbeat from the drone")
        if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            drone.target_system = hb.get_srcSystem()
            drone.target_component = hb.get_srcComponent()
            return hb


def mode():
    return mavutil.mode_string_v10(heartbeat())


def armed():
    return bool(heartbeat().base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def altitude():
    """Height above the take-off point, in metres."""
    return read("GLOBAL_POSITION_INT").relative_alt / 1000


# ---- results: every command returns one of these ----
def done(reason):    return {"status": "done", "reason": reason}
def blocked(reason): return {"status": "blocked", "reason": reason}   # a safety rule said no
def failed(reason):  return {"status": "failed", "reason": reason}    # it was tried and didn't work


def wait_until(condition, timeout):
    """Keep checking until condition() is true. False if time runs out."""
    end = time.time() + timeout
    while time.time() < end:
        if condition():
            return True
    return False


# ---- the commands ----
def get_status():
    """Everything the LLM needs to know before it plans."""
    gps = read("GPS_RAW_INT")
    battery = read("SYS_STATUS")
    return {
        "status": "done",
        "mode": mode(),
        "armed": armed(),
        "altitude_m": round(altitude(), 1),
        "battery_pct": battery.battery_remaining,
        "battery_volts": round(battery.voltage_battery / 1000, 1),
        "gps_fix": gps.fix_type >= 3,
        "satellites": gps.satellites_visible,
    }


def takeoff(alt_m):
    """Arm and climb straight up to alt_m metres."""
    now = get_status()
    if alt_m > MAX_ALT_M:                     return blocked(f"{alt_m} m is above the {MAX_ALT_M} m limit")
    if alt_m < 1:                             return blocked("take-off height must be at least 1 m")
    if now["armed"]:                          return blocked("already armed or flying")
    if not now["gps_fix"]:                    return blocked("no GPS fix yet")
    if now["battery_pct"] < MIN_BATTERY_PCT:  return blocked(f"battery {now['battery_pct']}% is below {MIN_BATTERY_PCT}%")

    drone.set_mode("GUIDED")                  # GUIDED = "accept commands from code"
    if not wait_until(lambda: mode() == "GUIDED", 5):
        return failed("could not switch to GUIDED")

    drone.arducopter_arm()
    if not wait_until(armed, 10):
        return failed("the drone refused to arm (check the pre-arm messages in QGroundControl)")

    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)
    if not wait_until(lambda: altitude() > alt_m - 0.5, 60):
        return failed(f"only reached {altitude():.1f} m")
    return done(f"at {altitude():.1f} m")


def hold():
    """Stop and hover in place."""
    if mode() != "GUIDED":
        return blocked(f"drone is in {mode()}, the pilot has control")
    # A velocity target of zero. 3527 means "use only the velocity fields".
    drone.mav.set_position_target_local_ned_send(
        0, drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 3527,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    return done(f"holding at {altitude():.1f} m")


def land():
    """Land where it is, then the motors switch off by themselves."""
    if not armed():
        return blocked("already on the ground")
    drone.set_mode("LAND")
    if not wait_until(lambda: not armed(), 120):
        return failed(f"still armed at {altitude():.1f} m")
    return done("landed and disarmed")


# The list the LLM will be given later.
COMMANDS = {"get_status": get_status, "takeoff": takeoff, "hold": hold, "land": land}

if __name__ == "__main__":
    name, numbers = sys.argv[1], [float(x) for x in sys.argv[2:]]
    connect()
    print(COMMANDS[name](*numbers))
