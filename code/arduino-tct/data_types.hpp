#pragma once

struct LatLonDeg {
  double lat;  // Latitude  in degrees, range [-90, +90]
  double lon;  // Longitude in degrees, range [-180, +180]

  LatLonDeg()
    : lat(0.0), lon(0.0) {}

  LatLonDeg(double latitude_deg, double longitude_deg)
    : lat(latitude_deg), lon(longitude_deg) {}
};