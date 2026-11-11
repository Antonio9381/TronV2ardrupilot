import math

from tron.geo import distance_m, ne_offset_m, offset_latlon
from tron.state import VehicleState

HOME = (-33.888, 151.187)


class FakeVehicle:
    def __init__(self, speed=3.0, climb=1.5, alt=10.0, heading=0.0, battery_v=16.0):
        self.t = 100.0
        self.speed = speed
        self.climb = climb
        self.target = None
        self.blocked_after_m = None
        self.events = []
        self.commands = []
        self.link_dead_at = None
        self.drain_v_per_s = 0.0
        self._s = VehicleState(
            mode="GUIDED", armed=True, lat=HOME[0], lon=HOME[1], rel_alt=alt, amsl=alt + 30,
            heading=heading, home_lat=HOME[0], home_lon=HOME[1], home_amsl=30.0,
            gps_fix=3, sats=18, battery_v=battery_v, battery_pct=90,
            last_heartbeat=self.t, last_position=self.t,
            last_gps=self.t, last_battery=self.t,
        )
        self.travelled = 0.0

    def at(self, t, fn):
        self.events.append((t, fn))

    def now(self):
        return self.t

    def state(self):
        return self._s

    def sleep(self, dt):
        end = self.t + dt
        while self.t < end - 1e-9:
            step = min(0.1, end - self.t)
            self._physics(step)
            self.t += step
            for ev in [e for e in self.events if e[0] <= self.t]:
                self.events.remove(ev)
                ev[1](self)
        alive = self.link_dead_at is None or self.t < self.link_dead_at
        if alive:
            self._s = self._s.evolve(last_heartbeat=self.t, last_position=self.t, last_gps=self.t, last_battery=self.t)

    def _physics(self, dt):
        self._s = self._s.evolve(battery_v=self._s.battery_v - self.drain_v_per_s * dt)
        s = self._s
        if self.target is None or s.mode != "GUIDED":
            return
        lat, lon, alt = self.target
        n, e = ne_offset_m(s.lat, s.lon, lat, lon)
        d = math.hypot(n, e)
        move = min(d, self.speed * dt)
        if self.blocked_after_m is not None:
            move = max(0.0, min(move, self.blocked_after_m - self.travelled))
        self.travelled += move
        if d > 1e-6:
            nlat, nlon = offset_latlon(s.lat, s.lon, n / d * move, e / d * move)
        else:
            nlat, nlon = s.lat, s.lon
        dz = alt - s.rel_alt
        nalt = s.rel_alt + max(-self.climb * dt, min(self.climb * dt, dz))
        self._s = s.evolve(lat=nlat, lon=nlon, rel_alt=nalt, ground_speed=move / dt)

    def goto_global(self, lat, lon, alt):
        self.commands.append(("goto", lat, lon, alt))
        self.target = (lat, lon, alt)

    def hold(self):
        self.commands.append(("hold",))
        s = self._s
        if s.mode == "GUIDED":
            self.target = (s.lat, s.lon, s.rel_alt)

    def set_mode(self, mode):
        self.commands.append(("mode", mode))
        self._s = self._s.evolve(mode=mode)

    def rtl(self):
        self.set_mode("RTL")

    def land(self):
        self.set_mode("LAND")

    def set_state(self, **kw):
        self._s = self._s.evolve(**kw)

    def distance_to(self, lat, lon):
        return distance_m(self._s.lat, self._s.lon, lat, lon)


class FakeLocator:
    def __init__(self, vehicle, target_latlon, visible_radius_m=6.0):
        self.vehicle = vehicle
        self.target = target_latlon
        self.radius = visible_radius_m
        self.hidden = False
        self.calls = 0

    def locate(self, alt_m):
        self.calls += 1
        if self.hidden:
            return None
        s = self.vehicle.state()
        n, e = ne_offset_m(s.lat, s.lon, *self.target)
        if math.hypot(n, e) > self.radius:
            return None
        h = math.radians(s.heading)
        fwd = n * math.cos(h) + e * math.sin(h)
        right = -n * math.sin(h) + e * math.cos(h)
        return fwd, right

    def close(self):
        pass
