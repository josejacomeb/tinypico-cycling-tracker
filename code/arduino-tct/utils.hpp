#include <math.h>

#include "constants.hpp"
#include "data_types.hpp"

struct LatLonDeg getPointAhead(
  const LatLonDeg& start,
  double distance_m,
  double bearing_deg) {

  double lat1 = start.lat * M_PI / 180.0;
  double lon1 = start.lon * M_PI / 180.0;
  double bearing = bearing_deg * M_PI / 180.0;
  double delta = distance_m / EARTH_RADIUS;

  double lat2 = asin(
    sin(lat1) * cos(delta) + cos(lat1) * sin(delta) * cos(bearing));

  double y = sin(bearing) * sin(delta) * cos(lat1);
  double x = cos(delta) - sin(lat1) * sin(lat2);

  double lon2 = lon1 + atan2(y, x);

  LatLonDeg result;
  result.lat = lat2 * 180.0 / M_PI;
  result.lon = lon2 * 180.0 / M_PI;

  return result;
}
