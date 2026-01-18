from dataclasses import dataclass
from enum import Enum


# Define the Axis Enum to match the C++ logic
class GPSAxis(Enum):
    LATITUDE = 1
    LONGITUDE = 2


@dataclass
class LatLonDeg:
    lat: float = 0.0  # Latitude in degrees
    lon: float = 0.0  # Longitude in degrees
