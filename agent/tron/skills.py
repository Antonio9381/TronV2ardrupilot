import enum
import logging
import math
import time
from dataclasses import dataclass

from .geo import angle_diff, body_to_ne, distance_m, offset_latlon
from .safety import SafetyError

log = logging.getLogger("tron.skills")


class Status(enum.Enum):
    ARRIVED = "arrived"
    SENT = "sent"
    OVERRIDE = "override"
    STUCK = "stuck"
    TIMEOUT = "timeout"
    LINK_LOST = "link_lost"
    REJECTED = "rejected"
    LOW_BATTERY = "low_battery"


@dataclass(frozen=True)
class Outcome:
    status: Status
    message: str = ""
    remaining_m: float = float("nan")

    @property
    def ok(self):
        return self.status in (Status.ARRIVED, Status.SENT)


class Pilot:
    def __init__(self, vehicle, guard, sleep=time.sleep, poll_s=0.2, resend_s=2.0):
        self.vehicle = vehicle
        self.guard = guard
        self.limits = guard.limits
        self.sleep = sleep
        self.poll_s = poll_s
        self.resend_s = resend_s

    def _abort_reason(self, s, now):
        if not self.guard.link_ok(s, now):
            return Outcome(Status.LINK_LOST, "telemetry lost")
        if s.mode != "GUIDED" or not s.armed:
            return Outcome(Status.OVERRIDE, f"pilot took over ({s.mode}, armed={s.armed})")
        if s.battery_v is not None and s.battery_v < self.limits.min_battery_v:
            return Outcome(Status.LOW_BATTERY, f"battery {s.battery_v:.2f} V")
        return None

    def _prepare(self, lat, lon, alt):
        s, now = self.vehicle.state(), self.vehicle.now()
        try:
            self.guard.check_ready(s, now)
            self.guard.validate_target(s, lat, lon, alt)
        except SafetyError as e:
            return None, Outcome(Status.REJECTED, str(e))
        return s, None

    def _remaining(self, s, lat, lon, alt):
        horiz = distance_m(s.lat, s.lon, lat, lon)
        return horiz, abs(s.rel_alt - alt)

    def send(self, lat, lon, alt, yaw_deg=None):
        s, rejected = self._prepare(lat, lon, alt)
        if rejected:
            return rejected
        self.vehicle.goto_global(lat, lon, alt, yaw_deg)
        return Outcome(Status.SENT, remaining_m=self._remaining(s, lat, lon, alt)[0])

    def goto(self, lat, lon, alt, timeout_s=None, yaw_deg=None):
        s, rejected = self._prepare(lat, lon, alt)
        if rejected:
            log.warning("goto rejected: %s", rejected.message)
            return rejected
        horiz, vert = self._remaining(s, lat, lon, alt)
        if timeout_s is None:
            timeout_s = 30.0 + 2.0 * math.hypot(horiz, vert) / max(self.guard.cruise_speed_ms, 0.5)
        log.info("goto %.7f %.7f %.1f m (%.0f m away, timeout %.0f s)", lat, lon, alt, horiz, timeout_s)
        tol = min(self.limits.arrive_tol_m, max(0.5, 0.25 * horiz))
        start = last_sent = self.vehicle.now()
        self.vehicle.goto_global(lat, lon, alt, yaw_deg)
        best = math.hypot(horiz, vert)
        best_at = start
        while True:
            self.sleep(self.poll_s)
            s, now = self.vehicle.state(), self.vehicle.now()
            abort = self._abort_reason(s, now)
            if abort:
                if abort.status is Status.LOW_BATTERY:
                    self.vehicle.hold()
                log.warning("goto aborted: %s", abort.message)
                return abort
            horiz, vert = self._remaining(s, lat, lon, alt)
            yaw_ok = yaw_deg is None or (s.heading is not None and
                                         abs(angle_diff(s.heading, yaw_deg)) <= self.limits.yaw_tol_deg)
            if horiz <= tol and vert <= self.limits.arrive_alt_tol_m and yaw_ok:
                log.info("arrived (%.1f m)", horiz)
                return Outcome(Status.ARRIVED, remaining_m=horiz)
            dist = math.hypot(horiz, vert)
            if dist < best - self.limits.stuck_progress_m:
                best, best_at = dist, now
            if now - best_at > self.limits.stuck_window_s:
                self.vehicle.hold()
                log.warning("no progress for %.0f s, holding %.1f m short", now - best_at, horiz)
                return Outcome(Status.STUCK, "no progress", horiz)
            if now - start > timeout_s:
                self.vehicle.hold()
                log.warning("timed out %.1f m short", horiz)
                return Outcome(Status.TIMEOUT, "timed out", horiz)
            if now - last_sent >= self.resend_s:
                self.vehicle.goto_global(lat, lon, alt, yaw_deg)
                last_sent = now

    def body_target(self, forward, right, yaw=0.0, up=0.0):
        s, now = self.vehicle.state(), self.vehicle.now()
        try:
            self.guard.validate_nudge(forward, right, yaw, up)
            if not s.has_position or s.heading is None:
                raise SafetyError("no position or heading")
            forward, right, note = self.guard.clamp_to_obstacles(s, now, forward, right)
        except SafetyError as e:
            return None, Outcome(Status.REJECTED, str(e))
        if note:
            log.warning(note)
        north, east = body_to_ne(forward, right, s.heading)
        lat, lon = offset_latlon(s.lat, s.lon, north, east)
        return (lat, lon, s.rel_alt + up, (s.heading + yaw) % 360), None

    def nudge(self, forward, right, yaw=0.0, up=0.0, timeout_s=None):
        target, rejected = self.body_target(forward, right, yaw, up)
        if rejected:
            return rejected
        lat, lon, alt, heading = target
        return self.goto(lat, lon, alt, timeout_s=timeout_s, yaw_deg=heading)

    def step(self, forward, right):
        target, rejected = self.body_target(forward, right)
        if rejected:
            return rejected
        return self.send(*target)

    def check(self):
        s, now = self.vehicle.state(), self.vehicle.now()
        return self._abort_reason(s, now)

    def hold(self):
        self.vehicle.hold()
        return Outcome(Status.ARRIVED, "holding")
