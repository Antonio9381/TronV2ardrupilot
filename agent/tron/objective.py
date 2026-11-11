import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .detect import COLOURS

LON_KEYS = ("long", "lon", "lng", "longitude")
LAT_KEYS = ("lat", "latitude")
ALT_KEYS = ("alt", "altitude")
COLOUR_KEYS = ("colour", "color")


class ObjectiveError(ValueError):
    pass


@dataclass(frozen=True)
class Objective:
    lat: float
    lon: float
    alt: float
    colour: Optional[str] = None
    target: Optional[str] = None


def _pick(data, keys, name, required=True):
    found = [k for k in keys if k in data]
    if len(found) > 1:
        raise ObjectiveError(f"{name} given more than once ({', '.join(found)})")
    if not found:
        if required:
            raise ObjectiveError(f"missing {name}")
        return None
    return data[found[0]]


def _number(value, name):
    if isinstance(value, bool):
        raise ObjectiveError(f"{name} must be a number")
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ObjectiveError(f"{name} must be a number") from None
    if not math.isfinite(v):
        raise ObjectiveError(f"{name} must be finite")
    return v


def parse_objective(data):
    if not isinstance(data, dict):
        raise ObjectiveError("objective must be a JSON object")
    known = set(LAT_KEYS + LON_KEYS + ALT_KEYS + COLOUR_KEYS + ("target",))
    unknown = sorted(set(data) - known)
    if unknown:
        raise ObjectiveError(f"unknown keys: {', '.join(unknown)}")
    lat = _number(_pick(data, LAT_KEYS, "lat"), "lat")
    lon = _number(_pick(data, LON_KEYS, "long"), "long")
    alt = _number(_pick(data, ALT_KEYS, "alt"), "alt")
    if not -90 <= lat <= 90:
        raise ObjectiveError("lat out of range")
    if not -180 <= lon <= 180:
        raise ObjectiveError("long out of range")
    colour = _pick(data, COLOUR_KEYS, "colour", required=False)
    if colour is not None:
        if not isinstance(colour, str):
            raise ObjectiveError("colour must be text")
        colour = colour.strip().lower() or None
        if colour is not None and colour not in COLOURS:
            raise ObjectiveError(f"colour must be one of {', '.join(sorted(COLOURS))}")
    target = data.get("target")
    if target is not None and not isinstance(target, str):
        raise ObjectiveError("target must be text")
    return Objective(lat, lon, alt, colour, (target or "").strip() or None)


def load_objective(path):
    try:
        data = json.loads(Path(path).read_text())
    except json.JSONDecodeError as e:
        raise ObjectiveError(f"invalid JSON: {e}") from None
    return parse_objective(data)
