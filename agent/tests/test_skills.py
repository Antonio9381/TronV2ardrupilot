import pytest

from tron.geo import offset_latlon
from tron.skills import Status

from .sim import HOME


def test_goto_arrives(pilot, vehicle):
    lat, lon = offset_latlon(*HOME, 30, 20)
    out = pilot.goto(lat, lon, 15)
    assert out.status is Status.ARRIVED
    assert vehicle.distance_to(lat, lon) <= 2
    assert vehicle.state().rel_alt == pytest.approx(15, abs=1)


def test_goto_outside_fence_sends_nothing(pilot, vehicle):
    lat, lon = offset_latlon(*HOME, 200, 0)
    out = pilot.goto(lat, lon, 10)
    assert out.status is Status.REJECTED
    assert vehicle.commands == []


def test_goto_refuses_outside_guided(pilot, vehicle):
    vehicle.set_state(mode="LOITER")
    out = pilot.goto(*offset_latlon(*HOME, 10, 0), 10)
    assert out.status is Status.REJECTED
    assert vehicle.commands == []


def test_pilot_override_stops_immediately(pilot, vehicle):
    vehicle.at(vehicle.t + 3, lambda v: v.set_state(mode="LOITER"))
    out = pilot.goto(*offset_latlon(*HOME, 60, 0), 10)
    assert out.status is Status.OVERRIDE
    assert ("hold",) not in vehicle.commands
    assert vehicle.commands[-1][0] == "goto"


def test_stuck_holds(pilot, vehicle):
    vehicle.blocked_after_m = 8
    out = pilot.goto(*offset_latlon(*HOME, 40, 0), 10)
    assert out.status is Status.STUCK
    assert vehicle.commands[-1] == ("hold",)


def test_link_lost(pilot, vehicle):
    vehicle.link_dead_at = vehicle.t + 2
    out = pilot.goto(*offset_latlon(*HOME, 60, 0), 10)
    assert out.status is Status.LINK_LOST


def test_low_battery_holds(pilot, vehicle):
    vehicle.drain_v_per_s = 0.5
    out = pilot.goto(*offset_latlon(*HOME, 60, 0), 10)
    assert out.status is Status.LOW_BATTERY
    assert vehicle.commands[-1] == ("hold",)


def test_timeout(pilot, vehicle):
    vehicle.speed = 0.3
    out = pilot.goto(*offset_latlon(*HOME, 60, 0), 10, timeout_s=10)
    assert out.status is Status.TIMEOUT


@pytest.mark.parametrize("heading,fwd,right,north,east", [
    (0, 4, 0, 4, 0), (90, 4, 0, 0, 4), (90, 0, 3, -3, 0), (225, 2, 0, -1.414, -1.414),
])
def test_nudge_uses_heading(pilot, vehicle, heading, fwd, right, north, east):
    vehicle.set_state(heading=heading)
    out = pilot.nudge(fwd, right)
    assert out.status is Status.ARRIVED
    _, lat, lon, alt, yaw = vehicle.commands[0]
    assert yaw == pytest.approx(heading)
    exp = offset_latlon(*HOME, north, east)
    assert lat == pytest.approx(exp[0], abs=1e-6)
    assert lon == pytest.approx(exp[1], abs=1e-6)
    assert alt == pytest.approx(10)


def test_nudge_cap(pilot, vehicle):
    assert pilot.nudge(6, 0).status is Status.REJECTED
    assert vehicle.commands == []


def test_nudge_yaw_and_up(pilot, vehicle):
    out = pilot.nudge(0, 0, 90, 2)
    assert out.status is Status.ARRIVED
    s = vehicle.state()
    assert s.heading == pytest.approx(90, abs=5)
    assert s.rel_alt == pytest.approx(12, abs=1)
    assert vehicle.distance_to(*HOME) < 0.5


def test_nudge_yaw_left_wraps(pilot, vehicle):
    vehicle.set_state(heading=10)
    assert pilot.nudge(0, 0, -30).status is Status.ARRIVED
    assert vehicle.commands[0][4] == pytest.approx(340)


def test_nudge_keeps_heading(pilot, vehicle):
    vehicle.set_state(heading=30)
    pilot.nudge(0, 3)
    assert vehicle.commands[0][4] == pytest.approx(30)


def test_nudge_shortened_by_obstacle(pilot, vehicle):
    vehicle.set_state(sectors=(4.0,) + (12.0,) * 7)
    assert pilot.nudge(3, 0).status is Status.ARRIVED
    _, lat, lon, _, _ = vehicle.commands[0]
    assert vehicle.distance_to(lat, lon) == pytest.approx(0, abs=0.6)
    assert lat == pytest.approx(offset_latlon(*HOME, 1.5, 0)[0], abs=2e-6)


def test_nudge_refused_next_to_obstacle(pilot, vehicle):
    vehicle.set_state(sectors=(2.6,) + (12.0,) * 7)
    out = pilot.nudge(2, 0)
    assert out.status is Status.REJECTED and "obstacle" in out.message
    assert vehicle.commands == []


def test_obstacle_beside_does_not_block(pilot, vehicle):
    vehicle.set_state(sectors=(12.0, 12.0, 1.5) + (12.0,) * 5)
    pilot.nudge(3, 0)
    _, lat, _, _, _ = vehicle.commands[0]
    assert lat == pytest.approx(offset_latlon(*HOME, 3, 0)[0], abs=2e-6)


def test_obstacle_behind_only_blocks_backwards(pilot, vehicle):
    vehicle.set_state(sectors=(12.0,) * 4 + (2.0,) + (12.0,) * 3)
    assert pilot.nudge(-2, 0).status is Status.REJECTED
    assert pilot.nudge(2, 0).status is Status.ARRIVED


def test_nudge_refused_when_lidar_unhealthy(pilot, vehicle):
    vehicle.set_state(lidar_healthy=False)
    out = pilot.nudge(1, 0)
    assert out.status is Status.REJECTED and "lidar" in out.message
    assert pilot.nudge(0, 0, 45).status is Status.ARRIVED


def test_open_field_no_readings_is_clear(pilot, vehicle):
    vehicle.lidar_dead = True
    vehicle.set_state(sectors=(2.0,) * 8, last_proximity=0.0)
    assert pilot.nudge(3, 0).status is Status.ARRIVED
    _, lat, _, _, _ = vehicle.commands[0]
    assert lat == pytest.approx(offset_latlon(*HOME, 3, 0)[0], abs=2e-6)


def test_yaw_cap(pilot, vehicle):
    assert pilot.nudge(0, 0, 200).status is Status.REJECTED
