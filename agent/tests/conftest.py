import pytest

from tron.config import Config
from tron.safety import Guard
from tron.skills import Pilot

from .sim import FakeVehicle

PARAMS = {"FENCE_ENABLE": 1, "FENCE_RADIUS": 100, "FENCE_ALT_MAX": 50, "WP_SPD": 3.0,
          "GUID_OPTIONS": 64, "OA_TYPE": 1}


@pytest.fixture
def cfg():
    return Config()


@pytest.fixture
def vehicle():
    return FakeVehicle()


@pytest.fixture
def guard(cfg):
    g = Guard(cfg.limits)
    g.apply_vehicle_params(PARAMS)
    return g


@pytest.fixture
def pilot(vehicle, guard):
    return Pilot(vehicle, guard, sleep=vehicle.sleep)
