#include "workout.hpp"

void Workout::start() {
  start_workout_ms = millis();
}

void Workout::end() {
  elapsed_workout_ms = millis() - start_workout_ms;
}

void Workout::reset() {
  start_workout_ms = 0;
  elapsed_workout_ms = 0;
}

double Workout::haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dlat = (lat2 - lat1) * DEG_TO_RAD;
  double dlon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * sin(dlon / 2) * sin(dlon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

void Workout::pushWP(double lat, double lon, double alt) {
  WP wp = { lat, lon, alt, 0.0, true };
  if (!win[win_end].valid) {
    win[win_end] = wp;
    win_end = (win_end + 1) % MAXW;
    return;
  }
  int prev = (win_end - 1 + MAXW) % MAXW;
  double d = haversine(win[prev].lat, win[prev].lon, lat, lon);
  wp.accumDist = win[prev].accumDist + d;
  win[win_end] = wp;
  win_end = (win_end + 1) % MAXW;

  while (true) {
    int s = win_start;
    int e = (win_end - 1 + MAXW) % MAXW;
    if (!win[s].valid || !win[e].valid) break;
    double wd = win[e].accumDist - win[s].accumDist;
    if (wd > GPS_GRADE_WINDOW) {
      win[s].valid = false;
      win_start = (win_start + 1) % MAXW;
    } else break;
  }
}

bool Workout::gpsGrade(double& grade) {
  int s = win_start;
  int e = (win_end - 1 + MAXW) % MAXW;
  if (!win[s].valid || !win[e].valid) return false;
  if (s == e) return false;

  double hd = win[e].accumDist - win[s].accumDist;
  if (hd < 3.0) return false;

  double ed = win[e].alt - win[s].alt;
  grade = (ed / hd) * 100.0;
  return true;
}

char* Workout::paceFromSpeed(double speed_m_s) {
  if (speed_m_s < 0.5) {
    snprintf(buf, 6, "--:--");
    return buf;
  }
  double secpkm = 1000.0 / speed_m_s;
  int mm = int(secpkm / 60);
  int ss = int(secpkm - mm * 60);
  if (ss == 60) {
    mm++;
    ss = 0;
  }
  sprintf(buf, "%02d:%02d", mm, ss);
  return buf;
}

double& Workout::get_slope(float& pitch) {
  double ggrade = 0;
  float imuSlopePercent = tan(pitch * DEG_TO_RAD) * 100.0;
  if (gpsGrade(ggrade)) {
    slope = ALPHA_IMU * imuSlopePercent + (1.0 - ALPHA_IMU) * ggrade;
  }
  return slope;
}