from tron.geo import offset_latlon
from tron.mission import Mission, Phase
from tron.objective import Objective
from tron.skills import Status

from .sim import HOME, FakeLocator


def objective(n=30, e=0, colour="red"):
    lat, lon = offset_latlon(*HOME, n, e)
    return Objective(lat, lon, 10.0, colour)


def test_transit_only(pilot, vehicle, cfg):
    r = Mission(pilot, FakeLocator(vehicle, HOME), cfg.mission).run(objective(colour=None))
    assert r.ok and r.phase is Phase.TRANSIT


def test_search_finds_and_tracks(pilot, vehicle, cfg):
    cfg.mission.track_s = 10
    tgt = offset_latlon(*HOME, 40, 12)
    r = Mission(pilot, FakeLocator(vehicle, tgt, 6), cfg.mission).run(objective())
    assert r.ok and r.found
    assert r.phase is Phase.TRACK
    assert vehicle.distance_to(*tgt) < 1.5
    assert vehicle.commands[-1] == ("hold",)


def test_search_gives_up(pilot, vehicle, cfg):
    r = Mission(pilot, FakeLocator(vehicle, offset_latlon(*HOME, 0, -60), 3), cfg.mission).run(objective())
    assert not r.ok and not r.found
    assert r.phase is Phase.SEARCH


def test_lost_target_researches_then_ends(pilot, vehicle, cfg):
    cfg.mission.track_s = 120
    loc = FakeLocator(vehicle, offset_latlon(*HOME, 32, 0), 6)
    vehicle.at(vehicle.t + 25, lambda v: setattr(loc, "hidden", True))
    r = Mission(pilot, loc, cfg.mission).run(objective())
    assert r.found
    assert r.outcome.status in (Status.ARRIVED,)
    assert "lost" in r.outcome.message or "without" in r.outcome.message


def test_override_mid_mission_leaves_pilot_alone(pilot, vehicle, cfg):
    cfg.mission.end_action = "rtl"
    vehicle.at(vehicle.t + 4, lambda v: v.set_state(mode="LOITER"))
    r = Mission(pilot, FakeLocator(vehicle, HOME), cfg.mission).run(objective())
    assert not r.ok and r.outcome.status is Status.OVERRIDE
    assert ("mode", "RTL") not in vehicle.commands


def test_rejected_transit_does_not_rtl(pilot, vehicle, cfg):
    cfg.mission.end_action = "rtl"
    vehicle.set_state(mode="LOITER")
    r = Mission(pilot, FakeLocator(vehicle, HOME), cfg.mission).run(objective())
    assert r.outcome.status is Status.REJECTED
    assert vehicle.commands == []


def test_low_battery_mission_rtl(pilot, vehicle, cfg):
    vehicle.drain_v_per_s = 0.3
    r = Mission(pilot, FakeLocator(vehicle, HOME), cfg.mission).run(objective(60))
    assert r.outcome.status is Status.LOW_BATTERY
    assert ("mode", "RTL") in vehicle.commands
