import argparse
import json
import logging
import sys
import time
from pathlib import Path

from . import __version__
from .config import load_config
from .detect import build_locator
from .mission import Mission
from .objective import ObjectiveError, load_objective
from .safety import Guard, SafetyError
from .skills import Pilot, Status
from .vehicle import MavlinkVehicle, VehicleError

log = logging.getLogger("tron")

GUARD_PARAMS = ("FENCE_ENABLE", "FENCE_RADIUS", "FENCE_ALT_MAX", "WP_SPD", "GUID_OPTIONS", "OA_TYPE")


def setup_logging(cfg, command):
    root = logging.getLogger()
    root.setLevel(cfg.log.level)
    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    console = logging.StreamHandler()
    console.setFormatter(fmt)
    root.addHandler(console)
    try:
        directory = Path(cfg.log.directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{time.strftime('%Y%m%d-%H%M%S')}-{command}.log"
        handler = logging.FileHandler(path)
        handler.setFormatter(fmt)
        root.addHandler(handler)
    except OSError as e:
        log.warning("file logging disabled: %s", e)


def open_vehicle(cfg, args):
    link = cfg.link
    v = MavlinkVehicle(args.conn or link.conn, baud=link.baud, source_system=link.source_system,
                       source_component=link.source_component, send_heartbeat=link.send_heartbeat)
    v.connect(timeout=link.connect_timeout_s)
    return v


def make_guard(cfg, vehicle):
    guard = Guard(cfg.limits)
    params = {}
    for name in GUARD_PARAMS:
        try:
            params[name] = vehicle.get_param(name)
        except VehicleError:
            log.warning("could not read %s", name)
    guard.apply_vehicle_params(params)
    return guard


def wait_for_state(vehicle, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if vehicle.state().has_telemetry:
            return
        time.sleep(0.2)
    log.warning("telemetry incomplete after %.0f s", timeout)


def print_status(vehicle):
    s = vehicle.state()
    out = {
        "mode": s.mode, "armed": s.armed, "lat": s.lat, "lon": s.lon, "rel_alt": s.rel_alt,
        "heading": s.heading, "ground_speed": round(s.ground_speed, 2), "gps_fix": s.gps_fix,
        "sats": s.sats, "battery_v": s.battery_v, "battery_pct": s.battery_pct,
        "home": [s.home_lat, s.home_lon],
    }
    print(json.dumps(out, indent=2))


def outcome_code(outcome):
    if outcome.ok:
        return 0
    return 2 if outcome.status is Status.REJECTED else 1


def cmd_sitl_takeoff(vehicle, guard, args):
    try:
        vehicle.get_param("SIM_SPEEDUP")
    except VehicleError:
        log.error("SIM_SPEEDUP not found: this is not SITL, refusing to arm")
        return 2
    alt = min(args.alt, guard.alt_max_m)
    vehicle.set_mode("GUIDED")
    vehicle.arm()
    vehicle.takeoff(alt)
    end = time.monotonic() + 60
    while time.monotonic() < end:
        s = vehicle.state()
        if s.rel_alt is not None and s.rel_alt >= alt - 0.5:
            log.info("reached %.1f m", s.rel_alt)
            return 0
        time.sleep(0.5)
    log.error("takeoff did not reach %.1f m", alt)
    return 1


def run(args, cfg):
    if args.command == "check-objective":
        print(load_objective(args.file))
        return 0
    if args.command == "mission":
        objective = load_objective(args.file)
    with open_vehicle(cfg, args) as vehicle:
        wait_for_state(vehicle)
        if args.command == "status":
            print_status(vehicle)
            return 0
        if args.command == "rtl":
            vehicle.rtl()
            return 0
        if args.command == "land":
            vehicle.land()
            return 0
        guard = make_guard(cfg, vehicle)
        pilot = Pilot(vehicle, guard)
        if args.command == "sitl-takeoff":
            return cmd_sitl_takeoff(vehicle, guard, args)
        try:
            if args.command == "hold":
                pilot.hold()
                return 0
            if args.command == "goto":
                out = pilot.goto(args.lat, args.lon, args.alt)
            elif args.command == "nudge":
                out = pilot.nudge(args.forward, args.right, args.down)
            else:
                locator = build_locator(cfg.camera, objective.colour)
                try:
                    result = Mission(pilot, locator, cfg.mission).run(objective)
                finally:
                    locator.close()
                print(f"mission {'ok' if result.ok else 'failed'}: phase={result.phase.value} "
                      f"found={result.found} {result.outcome.status.value} {result.outcome.message}")
                return 0 if result.ok else 1
            print(f"{out.status.value} {out.message}".strip())
            return outcome_code(out)
        except KeyboardInterrupt:
            log.warning("interrupted, holding position")
            vehicle.hold()
            return 130


def build_parser():
    p = argparse.ArgumentParser(prog="tron", description="Tron companion control")
    p.add_argument("--version", action="version", version=__version__)
    p.add_argument("-c", "--config", help="TOML config file")
    p.add_argument("--conn", help="override [link].conn, e.g. udpin:0.0.0.0:14553 or /dev/ttyAMA0")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("hold")
    sub.add_parser("rtl")
    sub.add_parser("land")
    g = sub.add_parser("goto")
    g.add_argument("lat", type=float)
    g.add_argument("lon", type=float)
    g.add_argument("alt", type=float)
    n = sub.add_parser("nudge")
    n.add_argument("forward", type=float)
    n.add_argument("right", type=float)
    n.add_argument("down", type=float, nargs="?", default=0.0)
    m = sub.add_parser("mission")
    m.add_argument("file")
    c = sub.add_parser("check-objective")
    c.add_argument("file")
    t = sub.add_parser("sitl-takeoff")
    t.add_argument("alt", type=float, nargs="?", default=10.0)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(args.config)
    except (OSError, ValueError) as e:
        print(f"config error: {e}", file=sys.stderr)
        return 2
    setup_logging(cfg, args.command)
    try:
        return run(args, cfg)
    except ObjectiveError as e:
        log.error("objective error: %s", e)
        return 2
    except (VehicleError, SafetyError) as e:
        log.error("%s", e)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
