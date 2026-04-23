import math
from data_types import GPSAxis, LatLonDeg
from constants import EARTH_RADIUS
from copy import copy


def get_point_ahead(
    start: LatLonDeg, distance_m: float, bearing_deg: float
) -> LatLonDeg:
    """
    Return the point reached by travelling distance_m metres from start
    along the given bearing (degrees clockwise from true North).

    Uses the spherical-Earth forward azimuth formula (haversine family).
    Negative distance_m is valid and travels in the opposite direction
    (e.g. distance=-1000, bearing=0 moves 1000 m South).

    Coordinate conventions
    ----------------------
    bearing=0°   → North  (increasing latitude)
    bearing=90°  → East   (increasing longitude)
    bearing=180° → South
    bearing=270° → West

    Numerical safety
    ----------------
    After many UKF predict steps, floating-point accumulation can push the
    asin() argument infinitesimally outside [-1, 1], causing a ValueError.
    The argument is clamped to [-1, 1] before the call — the error introduced
    is on the order of 1e-10 degrees (~0.01 mm) and is negligible.
    """
    # 1. Convert to radians
    lat1 = math.radians(start.lat)
    lng1 = math.radians(start.lng)
    bearing = math.radians(bearing_deg)

    # 2. Angular distance (radians)
    delta = distance_m / EARTH_RADIUS

    # 3. Calculate the new latitude
    asin_arg = math.sin(lat1) * math.cos(delta) + math.cos(lat1) * math.sin(
        delta
    ) * math.cos(bearing)
    lat2 = math.asin(asin_arg)

    # 4. New longitude
    y = math.sin(bearing) * math.sin(delta) * math.cos(lat1)
    x = math.cos(delta) - math.sin(lat1) * math.sin(lat2)
    lng2 = lng1 + math.atan2(y, x)

    # 5. Back to degrees; normalise longitude to [-180, 180]
    res_lat = math.degrees(lat2)
    res_lng = math.degrees(math.fmod((lng2 + 3 * math.pi), (2 * math.pi)) - math.pi)

    return LatLonDeg(lat=res_lat, lng=res_lng)


def get_distance_meters_per_axis(
    from_pos: LatLonDeg, to_pos: LatLonDeg, axis: GPSAxis
) -> float:
    """Calculate the distance in metres between from_pos and to_pos along the specified axis."""
    from_pos = copy(from_pos)
    if axis == GPSAxis.LONGITUDE:
        from_pos.lat = 0
    elif axis == GPSAxis.LATITUDE:
        from_pos.lng = 0
    distance = getDistanceMeters(from_pos, to_pos)
    # In case of negative longitude/latitude, the distance along that axis should be negative
    if axis == GPSAxis.LONGITUDE and from_pos.lng < 0:
        distance *= -1
    elif axis == GPSAxis.LATITUDE and from_pos.lat < 0:
        distance *= -1
    return distance


def getDistanceMeters(from_pos: LatLonDeg, to_pos: LatLonDeg) -> float:
    """Haversine formula for great-circle distance between two lat/lng points."""

    delta_lat = math.radians(to_pos.lat - from_pos.lat)
    delta_lng = math.radians(to_pos.lng - from_pos.lng)

    a = math.sin(delta_lat / 2) ** 2 + math.cos(math.radians(from_pos.lat)) * math.cos(
        math.radians(to_pos.lat)
    ) * (math.sin(delta_lng / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance = EARTH_RADIUS * c
    return distance
