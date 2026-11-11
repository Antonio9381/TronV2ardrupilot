import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path


@dataclass
class LinkConfig:
    conn: str = "udpin:0.0.0.0:14553"
    baud: int = 57600
    source_system: int = 255
    source_component: int = 190
    send_heartbeat: bool = False
    connect_timeout_s: float = 30.0


@dataclass
class Limits:
    fence_radius_m: float = 80.0
    fence_margin_m: float = 10.0
    alt_min_m: float = 3.0
    alt_max_m: float = 40.0
    alt_margin_m: float = 5.0
    max_leg_m: float = 150.0
    max_nudge_m: float = 5.0
    min_battery_v: float = 14.2
    min_sats: int = 10
    arrive_tol_m: float = 2.0
    arrive_alt_tol_m: float = 1.0
    stuck_window_s: float = 20.0
    stuck_progress_m: float = 1.0
    link_timeout_s: float = 3.0
    position_timeout_s: float = 2.0
    require_path_planning: bool = True
    require_lidar: bool = True
    obstacle_clearance_m: float = 2.5
    body_half_width_m: float = 1.0
    max_yaw_deg: float = 180.0
    yaw_tol_deg: float = 5.0
    proximity_timeout_s: float = 2.0


@dataclass
class MissionConfig:
    search_step_m: float = 5.0
    search_radius_m: float = 15.0
    search_dwell_s: float = 2.0
    track_s: float = 30.0
    track_deadband_m: float = 0.5
    track_max_step_m: float = 1.5
    lost_s: float = 5.0
    max_researches: int = 2
    end_action: str = "hold"


@dataclass
class CameraConfig:
    source: str = "none"
    index: int = 0
    hfov_deg: float = 62.2
    min_area_frac: float = 0.002


@dataclass
class LogConfig:
    directory: str = "logs"
    level: str = "INFO"


@dataclass
class Config:
    link: LinkConfig = field(default_factory=LinkConfig)
    limits: Limits = field(default_factory=Limits)
    mission: MissionConfig = field(default_factory=MissionConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    log: LogConfig = field(default_factory=LogConfig)


def _apply(section, values, name):
    known = {f.name: f for f in fields(section)}
    for key, value in values.items():
        if key not in known:
            raise ValueError(f"unknown setting [{name}].{key}")
        expected = type(getattr(section, key))
        if expected is float and isinstance(value, int):
            value = float(value)
        if not isinstance(value, expected):
            raise ValueError(f"[{name}].{key} must be {expected.__name__}")
        setattr(section, key, value)


def load_config(path=None):
    cfg = Config()
    if path is None:
        return cfg
    data = tomllib.loads(Path(path).read_text())
    for name, values in data.items():
        if not hasattr(cfg, name):
            raise ValueError(f"unknown section [{name}]")
        _apply(getattr(cfg, name), values, name)
    if cfg.mission.end_action not in ("hold", "rtl", "land"):
        raise ValueError("[mission].end_action must be hold, rtl or land")
    return cfg
