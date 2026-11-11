import logging
import math

from .geo import distance_m, free_distance

log = logging.getLogger("tron.safety")

PATH_PLANNING_BIT = 64


class SafetyError(Exception):
    pass


class Guard:
    def __init__(self, limits):
        self.limits = limits
        self.fence_radius_m = limits.fence_radius_m
        self.alt_max_m = limits.alt_max_m
        self.cruise_speed_ms = 2.0
        self.path_planning = False

    def apply_vehicle_params(self, params):
        lim = self.limits
        if params.get("FENCE_ENABLE", 0) >= 1:
            if params.get("FENCE_RADIUS", 0) > 0:
                self.fence_radius_m = min(lim.fence_radius_m, params["FENCE_RADIUS"] - lim.fence_margin_m)
            if params.get("FENCE_ALT_MAX", 0) > 0:
                self.alt_max_m = min(lim.alt_max_m, params["FENCE_ALT_MAX"] - lim.alt_margin_m)
        if params.get("WP_SPD", 0) > 0:
            self.cruise_speed_ms = params["WP_SPD"]
        guid = int(params.get("GUID_OPTIONS", 0))
        self.path_planning = bool(guid & PATH_PLANNING_BIT) and params.get("OA_TYPE", 0) >= 1
        if self.fence_radius_m <= 0 or self.alt_max_m <= lim.alt_min_m:
            raise SafetyError("vehicle fence leaves no usable flight area")
        log.info("limits: radius %.0f m, alt %.0f-%.0f m, speed %.1f m/s, path planning %s",
                 self.fence_radius_m, lim.alt_min_m, self.alt_max_m, self.cruise_speed_ms, self.path_planning)

    def link_ok(self, state, now):
        return now - state.last_heartbeat <= self.limits.link_timeout_s

    def check_ready(self, state, now):
        lim = self.limits
        if lim.require_path_planning and not self.path_planning:
            raise SafetyError("path planning off: set GUID_OPTIONS bit 64 and OA_TYPE 1")
        if not self.link_ok(state, now):
            raise SafetyError("no recent heartbeat")
        if state.mode != "GUIDED":
            raise SafetyError(f"mode is {state.mode}, need GUIDED")
        if not state.armed:
            raise SafetyError("vehicle not armed")
        if not state.has_position or now - state.last_position > lim.position_timeout_s:
            raise SafetyError("no fresh position")
        if not state.has_home:
            raise SafetyError("home position unknown")
        if state.rel_alt < lim.alt_min_m - lim.arrive_alt_tol_m:
            raise SafetyError(f"too low ({state.rel_alt:.1f} m), take off first")
        if now - state.last_gps > lim.link_timeout_s:
            raise SafetyError("no recent GPS report")
        if state.sats < lim.min_sats or state.gps_fix < 3:
            raise SafetyError(f"GPS not good enough ({state.sats} sats, fix {state.gps_fix})")
        if state.battery_v is not None and state.battery_v < lim.min_battery_v:
            raise SafetyError(f"battery low ({state.battery_v:.2f} V)")

    def validate_target(self, state, lat, lon, alt):
        lim = self.limits
        for name, v in (("lat", lat), ("lon", lon), ("alt", alt)):
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise SafetyError(f"{name} is not a finite number")
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise SafetyError("lat/lon out of range")
        if not lim.alt_min_m <= alt <= self.alt_max_m:
            raise SafetyError(f"alt {alt:.1f} m outside {lim.alt_min_m:.0f}-{self.alt_max_m:.0f} m")
        from_home = distance_m(state.home_lat, state.home_lon, lat, lon)
        if from_home > self.fence_radius_m:
            raise SafetyError(f"target {from_home:.0f} m from home, limit {self.fence_radius_m:.0f} m")
        leg = distance_m(state.lat, state.lon, lat, lon)
        if leg > lim.max_leg_m:
            raise SafetyError(f"leg {leg:.0f} m exceeds {lim.max_leg_m:.0f} m")

    def validate_nudge(self, forward, right, yaw, up):
        for v in (forward, right, yaw, up):
            if not math.isfinite(v):
                raise SafetyError("nudge is not finite")
        if max(abs(forward), abs(right), abs(up)) > self.limits.max_nudge_m:
            raise SafetyError(f"nudge larger than {self.limits.max_nudge_m:.1f} m")
        if abs(yaw) > self.limits.max_yaw_deg:
            raise SafetyError(f"yaw larger than {self.limits.max_yaw_deg:.0f} deg")

    def clamp_to_obstacles(self, state, now, forward, right):
        lim = self.limits
        horiz = math.hypot(forward, right)
        if horiz == 0:
            return forward, right, None
        if not state.lidar_healthy:
            if lim.require_lidar:
                raise SafetyError("lidar not healthy, can't check the way is clear")
            return forward, right, None
        fresh = now - state.last_proximity <= lim.proximity_timeout_s
        sectors = state.sectors if fresh else (None,) * 8
        bearing = math.degrees(math.atan2(right, forward))
        free = free_distance(sectors, bearing, lim.body_half_width_m)
        allowed = free - lim.obstacle_clearance_m
        if allowed < 0.3:
            raise SafetyError(f"obstacle {free:.1f} m that way, refusing")
        if horiz <= allowed:
            return forward, right, None
        k = allowed / horiz
        return forward * k, right * k, f"obstacle at {free:.1f} m, move shortened to {allowed:.1f} m"
