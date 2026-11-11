import enum
import logging
import math
from dataclasses import dataclass

from .geo import expanding_square, offset_latlon
from .skills import Outcome, Status

log = logging.getLogger("tron.mission")


class Phase(enum.Enum):
    TRANSIT = "transit"
    SEARCH = "search"
    TRACK = "track"
    DONE = "done"


@dataclass(frozen=True)
class MissionResult:
    ok: bool
    phase: Phase
    found: bool
    outcome: Outcome


class Mission:
    def __init__(self, pilot, locator, cfg):
        self.pilot = pilot
        self.vehicle = pilot.vehicle
        self.locator = locator
        self.cfg = cfg
        self.phase = Phase.TRANSIT
        self.found = False

    def _set(self, phase):
        if phase is not self.phase:
            log.info("phase %s -> %s", self.phase.value, phase.value)
            self.phase = phase

    def _finish(self, outcome):
        hands_off = outcome.status in (Status.OVERRIDE, Status.LINK_LOST) or (
            outcome.status is Status.REJECTED and self.phase is Phase.TRANSIT)
        if not hands_off:
            action = "rtl" if outcome.status is Status.LOW_BATTERY else self.cfg.end_action
            log.info("end action: %s", action)
            try:
                getattr(self.vehicle, action)()
            except Exception:
                log.exception("end action %s failed, holding", action)
                self.vehicle.hold()
        ok = outcome.ok and (self.found or self.phase is Phase.TRANSIT)
        result = MissionResult(ok, self.phase, self.found, outcome)
        self._set(Phase.DONE)
        return result

    def _look(self, duration_s):
        end = self.vehicle.now() + duration_s
        while True:
            abort = self.pilot.check()
            if abort:
                return None, abort
            hit = self.locator.locate(self.vehicle.state().rel_alt)
            if hit is not None:
                return hit, None
            if self.vehicle.now() >= end:
                return None, None
            self.pilot.sleep(self.pilot.poll_s)

    def _search(self, lat, lon, alt):
        self._set(Phase.SEARCH)
        for north, east in expanding_square(self.cfg.search_step_m, self.cfg.search_radius_m):
            p_lat, p_lon = offset_latlon(lat, lon, north, east)
            out = self.pilot.goto(p_lat, p_lon, alt)
            if out.status in (Status.OVERRIDE, Status.LINK_LOST, Status.LOW_BATTERY):
                return None, out
            if out.status is Status.REJECTED:
                log.info("skipping search point %+.0f N %+.0f E: %s", north, east, out.message)
                continue
            hit, abort = self._look(self.cfg.search_dwell_s)
            if abort:
                return None, abort
            if hit is not None:
                log.info("target seen %.1f m fwd %.1f m right", *hit)
                self.found = True
                return hit, None
        return None, Outcome(Status.ARRIVED, "search finished without a sighting")

    def _track(self):
        self._set(Phase.TRACK)
        cfg = self.cfg
        start = last_seen = self.vehicle.now()
        researches = 0
        while self.vehicle.now() - start < cfg.track_s:
            abort = self.pilot.check()
            if abort:
                return abort
            s = self.vehicle.state()
            hit = self.locator.locate(s.rel_alt)
            now = self.vehicle.now()
            if hit is None:
                if now - last_seen > cfg.lost_s:
                    researches += 1
                    if researches > cfg.max_researches:
                        return Outcome(Status.ARRIVED, "target lost")
                    log.info("target lost, search %d/%d", researches, cfg.max_researches)
                    self.vehicle.hold()
                    hit, out = self._search(s.lat, s.lon, s.rel_alt)
                    if hit is None:
                        return out
                    self._set(Phase.TRACK)
                    last_seen = self.vehicle.now()
                else:
                    self.pilot.sleep(self.pilot.poll_s)
                continue
            last_seen = now
            fwd, right = hit
            dist = math.hypot(fwd, right)
            if dist > cfg.track_deadband_m:
                scale = min(1.0, cfg.track_max_step_m / dist)
                out = self.pilot.step(fwd * scale, right * scale)
                if out.status is Status.REJECTED:
                    log.warning("track step rejected: %s", out.message)
                    self.vehicle.hold()
            self.pilot.sleep(self.pilot.poll_s)
        return Outcome(Status.ARRIVED, "tracking finished")

    def run(self, objective):
        log.info("mission %s", objective)
        out = self.pilot.goto(objective.lat, objective.lon, objective.alt)
        if out.status is not Status.ARRIVED or objective.colour is None:
            return self._finish(out)
        hit, out = self._search(objective.lat, objective.lon, objective.alt)
        if hit is None:
            return self._finish(out)
        return self._finish(self._track())
