import math

EARTH_RADIUS_M = 6378137.0


def distance_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, a)))


def ne_offset_m(lat0, lon0, lat, lon):
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return north, east


def offset_latlon(lat, lon, north_m, east_m):
    dlat = math.degrees(north_m / EARTH_RADIUS_M)
    dlon = math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def body_to_ne(forward_m, right_m, heading_deg):
    h = math.radians(heading_deg)
    north = forward_m * math.cos(h) - right_m * math.sin(h)
    east = forward_m * math.sin(h) + right_m * math.cos(h)
    return north, east


def free_distance(sectors, bearing_deg, half_width_m):
    free = math.inf
    for k, d in enumerate(sectors):
        if d is None:
            continue
        diff = math.radians((k * 45 - bearing_deg + 180) % 360 - 180)
        if abs(diff) >= math.pi / 2:
            continue
        if abs(d * math.sin(diff)) < half_width_m:
            free = min(free, d * math.cos(diff))
    return free


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180


def expanding_square(step_m, radius_m):
    points = [(0.0, 0.0)]
    n = e = 0.0
    leg = 1
    direction = 0
    moves = ((1, 0), (0, 1), (-1, 0), (0, -1))
    while True:
        for _ in range(2):
            dn, de = moves[direction % 4]
            n += dn * leg * step_m
            e += de * leg * step_m
            if max(abs(n), abs(e)) > radius_m:
                return points
            points.append((n, e))
            direction += 1
        leg += 1
