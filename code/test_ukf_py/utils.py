import math
from data_types import LatLonDeg
from constants import EARTH_RADIUS

def get_point_ahead(
    start: LatLonDeg, distance_m: float, bearing_deg: float
) -> LatLonDeg:
    """
    Calculates a new LatLonDeg point given a start point, distance (m), and bearing (deg).
    """
    # 1. Convert everything to Radians
    lat1 = math.radians(start.lat)
    lon1 = math.radians(start.lon)
    bearing = math.radians(bearing_deg)

    # 2. Calculate angular distance (distance / radius)
    delta = distance_m / EARTH_RADIUS

    # 3. Calculate the new Latitude
    lat2 = math.asin(
        math.sin(lat1) * math.cos(delta)
        + math.cos(lat1) * math.sin(delta) * math.cos(bearing)
    )

    # 4. Calculate the new Longitude
    y = math.sin(bearing) * math.sin(delta) * math.cos(lat1)
    x = math.cos(delta) - math.sin(lat1) * math.sin(lat2)
    lon2 = lon1 + math.atan2(y, x)

    # 5. Convert back to Degrees and return
    # We use (lon + 180) % 360 - 180 to keep longitude within [-180, 180]
    res_lat = math.degrees(lat2)
    res_lon = math.degrees(lon2)

    # Normalize longitude to -180 to +180 range
    res_lon = (res_lon + 180) % 360 - 180

    return LatLonDeg(lat=res_lat, lon=res_lon)
