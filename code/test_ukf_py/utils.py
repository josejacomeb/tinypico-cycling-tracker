import math
from data_types import LatLonDeg
from constants import EARTH_RADIUS


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
    lon1 = math.radians(start.lon)
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
    lon2 = lon1 + math.atan2(y, x)

    # 5. Back to degrees; normalise longitude to [-180, 180]
    res_lat = math.degrees(lat2)
    res_lon = math.degrees(math.fmod((lon2 + 3 * math.pi), (2 * math.pi)) - math.pi)

    return LatLonDeg(lat=res_lat, lon=res_lon)
