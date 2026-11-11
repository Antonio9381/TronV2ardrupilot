import logging
import math
import threading
import time

from pymavlink import mavutil

from .state import VehicleState

log = logging.getLogger("tron.vehicle")

mavlink = mavutil.mavlink

COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
    "RTL": 6, "CIRCLE": 7, "LAND": 9, "POSHOLD": 16, "BRAKE": 17, "SMART_RTL": 21,
}

POSITION_ONLY_MASK = 0b110111111000
POSITION_YAW_MASK = 0b100111111000

STREAMS = {
    mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5.0,
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2.0,
    mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
    mavlink.MAVLINK_MSG_ID_HOME_POSITION: 0.5,
    mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 5.0,
}


class VehicleError(RuntimeError):
    pass


class MavlinkVehicle:
    def __init__(self, conn, baud=57600, source_system=255, source_component=190,
                 send_heartbeat=False, clock=time.monotonic):
        self._conn_str = conn
        self._baud = baud
        self._source_system = source_system
        self._source_component = source_component
        self._send_heartbeat = send_heartbeat
        self._clock = clock
        self._m = None
        self._state = VehicleState()
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._acks = {}
        self._params = {}
        self._running = False
        self._threads = []
        self.target_system = None
        self.target_component = None

    def connect(self, timeout=30.0):
        self._m = mavutil.mavlink_connection(
            self._conn_str, baud=self._baud,
            source_system=self._source_system, source_component=self._source_component,
        )
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            msg = self._m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.autopilot != mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue
            if msg.get_srcComponent() != mavlink.MAV_COMP_ID_AUTOPILOT1:
                continue
            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()
            self._m.target_system = self.target_system
            self._m.target_component = self.target_component
            self._handle(msg)
            break
        else:
            raise VehicleError(f"no ArduPilot heartbeat on {self._conn_str} within {timeout:.0f} s")
        self._running = True
        self._start(self._reader)
        if self._send_heartbeat:
            self._start(self._heartbeat)
        for msg_id, hz in STREAMS.items():
            self.command_long(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz, wait=False)
        self.command_long(mavlink.MAV_CMD_REQUEST_MESSAGE, mavlink.MAVLINK_MSG_ID_HOME_POSITION, wait=False)
        log.info("connected to system %s on %s", self.target_system, self._conn_str)
        return self

    def close(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        if self._m is not None:
            self._m.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def state(self):
        with self._lock:
            return self._state

    def now(self):
        return self._clock()

    def _start(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    def _heartbeat(self):
        mav_type = (mavlink.MAV_TYPE_ONBOARD_CONTROLLER
                    if self._source_component == mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
                    else mavlink.MAV_TYPE_GCS)
        while self._running:
            with self._send_lock:
                self._m.mav.heartbeat_send(mav_type, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1.0)

    def _reader(self):
        while self._running:
            try:
                msg = self._m.recv_match(blocking=True, timeout=0.5)
            except Exception:
                log.exception("receive failed")
                time.sleep(0.5)
                continue
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            if msg.get_srcSystem() != self.target_system:
                continue
            if msg.get_srcComponent() != self.target_component:
                continue
            self._handle(msg)

    def _handle(self, msg):
        t = msg.get_type()
        now = self._clock()
        with self._cond:
            s = self._state
            if t == "HEARTBEAT":
                armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._state = s.evolve(mode=mavutil.mode_string_v10(msg), armed=armed, last_heartbeat=now)
            elif t == "GLOBAL_POSITION_INT":
                hdg = None if msg.hdg == 65535 else msg.hdg / 100.0
                self._state = s.evolve(
                    lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                    rel_alt=msg.relative_alt / 1000.0, amsl=msg.alt / 1000.0,
                    heading=hdg, ground_speed=math.hypot(msg.vx, msg.vy) / 100.0,
                    last_position=now,
                )
            elif t == "HOME_POSITION":
                self._state = s.evolve(home_lat=msg.latitude / 1e7, home_lon=msg.longitude / 1e7,
                                       home_amsl=msg.altitude / 1000.0)
            elif t == "GPS_RAW_INT":
                self._state = s.evolve(gps_fix=msg.fix_type, sats=msg.satellites_visible, last_gps=now)
            elif t == "SYS_STATUS":
                v = None if msg.voltage_battery in (0, 65535) else msg.voltage_battery / 1000.0
                pct = None if msg.battery_remaining < 0 else msg.battery_remaining
                self._state = s.evolve(battery_v=v, battery_pct=pct, last_battery=now)
            elif t == "DISTANCE_SENSOR":
                if msg.orientation > 7:
                    return
                if 0 < msg.current_distance < msg.max_distance:
                    d = msg.current_distance / 100.0
                else:
                    d = msg.max_distance / 100.0
                sectors = list(s.sectors)
                sectors[msg.orientation] = d
                self._state = s.evolve(sectors=tuple(sectors), last_proximity=now)
            elif t == "COMMAND_ACK":
                self._acks[msg.command] = msg.result
            elif t == "PARAM_VALUE":
                self._params[msg.param_id] = msg.param_value
            elif t == "STATUSTEXT":
                log.info("FC: %s", msg.text)
                return
            else:
                return
            self._cond.notify_all()

    def _wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        with self._cond:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)
            return True

    def command_long(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=True, timeout=3.0):
        with self._cond:
            self._acks.pop(command, None)
        with self._send_lock:
            self._m.mav.command_long_send(self.target_system, self.target_component,
                                          command, 0, p1, p2, p3, p4, p5, p6, p7)
        if not wait:
            return None
        if not self._wait(lambda: command in self._acks, timeout):
            raise VehicleError(f"no ACK for command {command}")
        with self._lock:
            return self._acks[command]

    def get_param(self, name, timeout=3.0):
        with self._cond:
            self._params.pop(name, None)
        for _ in range(3):
            with self._send_lock:
                self._m.mav.param_request_read_send(self.target_system, self.target_component,
                                                    name.encode(), -1)
            if self._wait(lambda: name in self._params, timeout / 3):
                with self._lock:
                    return self._params[name]
        raise VehicleError(f"parameter {name} not received")

    def set_mode(self, name, timeout=3.0):
        mapping = self._m.mode_mapping() or COPTER_MODES
        if name not in mapping:
            raise VehicleError(f"unknown mode {name}")
        result = self.command_long(mavlink.MAV_CMD_DO_SET_MODE,
                                   mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[name])
        if result != mavlink.MAV_RESULT_ACCEPTED:
            raise VehicleError(f"mode {name} rejected (result {result})")
        if not self._wait(lambda: self._state.mode == name, timeout):
            raise VehicleError(f"mode did not change to {name}")

    def arm(self, timeout=5.0):
        result = self.command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if result != mavlink.MAV_RESULT_ACCEPTED:
            raise VehicleError(f"arming rejected (result {result})")
        if not self._wait(lambda: self._state.armed, timeout):
            raise VehicleError("vehicle did not arm")

    def takeoff(self, alt):
        result = self.command_long(mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
        if result != mavlink.MAV_RESULT_ACCEPTED:
            raise VehicleError(f"takeoff rejected (result {result})")

    def goto_global(self, lat, lon, rel_alt, yaw_deg=None):
        mask = POSITION_ONLY_MASK if yaw_deg is None else POSITION_YAW_MASK
        yaw = 0.0 if yaw_deg is None else math.radians(yaw_deg)
        with self._send_lock:
            self._m.mav.set_position_target_global_int_send(
                0, self.target_system, self.target_component,
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
                int(round(lat * 1e7)), int(round(lon * 1e7)), float(rel_alt),
                0, 0, 0, 0, 0, 0, yaw, 0)

    def hold(self):
        s = self.state()
        if s.has_position and s.mode == "GUIDED":
            self.goto_global(s.lat, s.lon, s.rel_alt, s.heading)

    def rtl(self):
        self.set_mode("RTL")

    def land(self):
        self.set_mode("LAND")
