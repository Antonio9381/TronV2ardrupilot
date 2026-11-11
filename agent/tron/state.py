from dataclasses import dataclass, replace
from typing import Optional


@dataclass(frozen=True)
class VehicleState:
    mode: str = "UNKNOWN"
    armed: bool = False
    lat: Optional[float] = None
    lon: Optional[float] = None
    rel_alt: Optional[float] = None
    amsl: Optional[float] = None
    heading: Optional[float] = None
    ground_speed: float = 0.0
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    home_amsl: Optional[float] = None
    gps_fix: int = 0
    sats: int = 0
    battery_v: Optional[float] = None
    battery_pct: Optional[int] = None
    last_heartbeat: float = 0.0
    last_position: float = 0.0
    last_gps: float = 0.0
    last_battery: float = 0.0
    sectors: tuple = (None,) * 8
    last_proximity: float = 0.0

    @property
    def has_position(self):
        return self.lat is not None and self.lon is not None and self.rel_alt is not None

    @property
    def has_home(self):
        return self.home_lat is not None and self.home_lon is not None

    @property
    def has_telemetry(self):
        return self.has_position and self.has_home and self.last_gps > 0 and self.last_battery > 0

    def evolve(self, **changes):
        return replace(self, **changes)
