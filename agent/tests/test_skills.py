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
    _, lat, lon, alt = vehicle.commands[0]
    exp = offset_latlon(*HOME, north, east)
    assert lat == pytest.approx(exp[0], abs=1e-6)
    assert lon == pytest.approx(exp[1], abs=1e-6)
    assert alt == pytest.approx(10)


def test_nudge_cap(pilot, vehicle):
    assert pilot.nudge(6, 0).status is Status.REJECTED
    assert vehicle.commands == []
