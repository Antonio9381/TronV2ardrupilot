import socket
import threading
import time

import pytest
from pymavlink import mavutil

from tron.vehicle import MavlinkVehicle, VehicleError

mavlink = mavutil.mavlink


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeAutopilot:
    def __init__(self, port):
        self.m = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
        self.custom_mode = 0
        self.armed = False
        self.params = {"FENCE_RADIUS": 100.0, "GUID_OPTIONS": 64.0, "SIM_SPEEDUP": 1.0}
        self.targets = []
        self.commands = []
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        last = 0
        while self.running:
            now = time.monotonic()
            if now - last > 0.2:
                last = now
                base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if self.armed:
                    base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                self.m.mav.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                          base, self.custom_mode, mavlink.MAV_STATE_ACTIVE)
                self.m.mav.global_position_int_send(0, -338880000, 1511870000, 40000, 10000, 100, 0, 0, 9000)
                self.m.mav.gps_raw_int_send(0, 3, -338880000, 1511870000, 40000, 80, 100, 0, 0, 17)
                self.m.mav.sys_status_send(0, 0, 0, 0, 15800, 1000, 87, 0, 0, 0, 0, 0, 0)
                for k in range(8):
                    self.m.mav.distance_sensor_send(0, 20, 1200, 350 if k == 2 else 1200, 0, 0, k, 0)
                self.m.mav.distance_sensor_send(0, 20, 1200, 90, 0, 0, 25, 0)
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "COMMAND_LONG":
                self.commands.append(msg.command)
                if msg.command == mavlink.MAV_CMD_DO_SET_MODE:
                    self.custom_mode = int(msg.param2)
                if msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.armed = msg.param1 == 1
                if msg.command == mavlink.MAV_CMD_REQUEST_MESSAGE:
                    self.m.mav.home_position_send(-338880000, 1511870000, 30000, 0, 0, 0, [1, 0, 0, 0], 0, 0, 0)
                result = (mavlink.MAV_RESULT_DENIED if msg.command == mavlink.MAV_CMD_NAV_TAKEOFF and not self.armed
                          else mavlink.MAV_RESULT_ACCEPTED)
                self.m.mav.command_ack_send(msg.command, result)
            elif t == "PARAM_REQUEST_READ":
                if msg.param_id in self.params:
                    self.m.mav.param_value_send(msg.param_id.encode(), self.params[msg.param_id],
                                                mavlink.MAV_PARAM_TYPE_REAL32, len(self.params), 0)
            elif t == "SET_POSITION_TARGET_GLOBAL_INT":
                self.targets.append(msg)

    def close(self):
        self.running = False
        self.thread.join(2)
        self.m.close()


@pytest.fixture
def link():
    port = free_port()
    v = MavlinkVehicle(f"udpin:127.0.0.1:{port}")
    ap = FakeAutopilot(port)
    v.connect(timeout=5)
    yield v, ap
    v.close()
    ap.close()


def wait(pred, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_telemetry_parsed(link):
    v, ap = link
    assert wait(lambda: v.state().has_telemetry)
    assert v.state().sats == 17
    assert wait(lambda: v.state().sectors[7] is not None)
    assert v.state().sectors[2] == pytest.approx(3.5)
    assert v.state().sectors[0] == pytest.approx(12.0)
    s = v.state()
    assert s.lat == pytest.approx(-33.888)
    assert s.rel_alt == pytest.approx(10.0)
    assert s.heading == pytest.approx(90.0)
    assert s.battery_v == pytest.approx(15.8)
    assert s.mode == "STABILIZE"
    assert mavlink.MAV_CMD_SET_MESSAGE_INTERVAL in ap.commands


def test_mode_arm_and_params(link):
    v, ap = link
    assert v.get_param("FENCE_RADIUS") == 100.0
    with pytest.raises(VehicleError):
        v.get_param("NOPE", timeout=0.6)
    v.set_mode("GUIDED")
    assert v.state().mode == "GUIDED"
    v.arm()
    assert v.state().armed


def test_takeoff_rejection_raises(link):
    v, _ = link
    with pytest.raises(VehicleError, match="takeoff rejected"):
        v.takeoff(5)


def test_goto_message(link):
    v, ap = link
    v.goto_global(-33.8885, 151.1875, 12.5)
    assert wait(lambda: ap.targets)
    t = ap.targets[0]
    assert (t.lat_int, t.lon_int) == (-338885000, 1511875000)
    assert t.alt == pytest.approx(12.5)
    assert t.coordinate_frame == mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert t.type_mask == 0b110111111000
    v.goto_global(-33.8885, 151.1875, 12.5, 90)
    assert wait(lambda: len(ap.targets) > 1)
    assert ap.targets[1].type_mask == 0b100111111000
    assert ap.targets[1].yaw == pytest.approx(1.5708, abs=1e-3)


def test_hold_only_in_guided(link):
    v, ap = link
    assert wait(lambda: v.state().has_position)
    v.hold()
    time.sleep(0.3)
    assert ap.targets == []
    v.set_mode("GUIDED")
    v.hold()
    assert wait(lambda: ap.targets)


def test_no_heartbeat_times_out():
    v = MavlinkVehicle(f"udpin:127.0.0.1:{free_port()}")
    with pytest.raises(VehicleError, match="no ArduPilot heartbeat"):
        v.connect(timeout=1.5)
    v.close()
