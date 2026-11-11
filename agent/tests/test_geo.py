import math

import pytest

from tron.geo import body_to_ne, distance_m, expanding_square, ne_offset_m, offset_latlon


def test_offset_roundtrip():
    lat, lon = offset_latlon(-33.888, 151.187, 30.0, -40.0)
    n, e = ne_offset_m(-33.888, 151.187, lat, lon)
    assert n == pytest.approx(30.0, abs=0.01)
    assert e == pytest.approx(-40.0, abs=0.01)
    assert distance_m(-33.888, 151.187, lat, lon) == pytest.approx(50.0, abs=0.05)


@pytest.mark.parametrize("heading,fwd,right,north,east", [
    (0, 1, 0, 1, 0), (90, 1, 0, 0, 1), (90, 0, 1, -1, 0), (180, 1, 0, -1, 0), (270, 0, 1, 1, 0),
])
def test_body_to_ne(heading, fwd, right, north, east):
    n, e = body_to_ne(fwd, right, heading)
    assert n == pytest.approx(north, abs=1e-9)
    assert e == pytest.approx(east, abs=1e-9)


def test_expanding_square_stays_inside():
    pts = expanding_square(5, 15)
    assert pts[0] == (0.0, 0.0)
    assert len(pts) == len(set(pts))
    assert all(max(abs(n), abs(e)) <= 15 for n, e in pts)
    assert len(pts) > 8
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert min(steps) == pytest.approx(5)
