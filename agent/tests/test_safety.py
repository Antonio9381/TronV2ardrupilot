import pytest

from tron.geo import offset_latlon
from tron.safety import Guard, SafetyError

from .sim import HOME


def test_vehicle_fence_tightens_limits(cfg):
    g = Guard(cfg.limits)
    g.apply_vehicle_params({"FENCE_ENABLE": 1, "FENCE_RADIUS": 60, "FENCE_ALT_MAX": 30,
                            "GUID_OPTIONS": 64, "OA_TYPE": 1, "WP_SPD": 1.34})
    assert g.fence_radius_m == 50
    assert g.alt_max_m == 25
    assert g.cruise_speed_ms == pytest.approx(1.34)
    assert g.path_planning


def test_requires_path_planning(cfg, vehicle):
    g = Guard(cfg.limits)
    g.apply_vehicle_params({"GUID_OPTIONS": 0, "OA_TYPE": 1})
    with pytest.raises(SafetyError, match="GUID_OPTIONS"):
        g.check_ready(vehicle.state(), vehicle.now())


@pytest.mark.parametrize("change,msg", [
    ({"mode": "LOITER"}, "need GUIDED"),
    ({"armed": False}, "not armed"),
    ({"sats": 6}, "GPS"),
    ({"battery_v": 13.9}, "battery"),
    ({"rel_alt": 0.4}, "too low"),
    ({"home_lat": None}, "home"),
    ({"last_heartbeat": 0.0}, "heartbeat"),
    ({"last_gps": 0.0}, "GPS report"),
])
def test_check_ready(guard, vehicle, change, msg):
    vehicle.set_state(**change)
    with pytest.raises(SafetyError, match=msg):
        guard.check_ready(vehicle.state(), vehicle.now())


def test_validate_target(guard, vehicle):
    s = vehicle.state()
    ok = offset_latlon(*HOME, 40, 0)
    guard.validate_target(s, *ok, 10)
    far = offset_latlon(*HOME, 95, 0)
    with pytest.raises(SafetyError, match="from home"):
        guard.validate_target(s, *far, 10)
    with pytest.raises(SafetyError, match="alt"):
        guard.validate_target(s, *ok, 46)
    with pytest.raises(SafetyError, match="alt"):
        guard.validate_target(s, *ok, 1)


def test_validate_nudge(guard):
    guard.validate_nudge(5, -5, -180, 5)
    with pytest.raises(SafetyError):
        guard.validate_nudge(5.1, 0, 0, 0)
    with pytest.raises(SafetyError):
        guard.validate_nudge(0, 0, 181, 0)
    with pytest.raises(SafetyError):
        guard.validate_nudge(float("inf"), 0, 0, 0)
