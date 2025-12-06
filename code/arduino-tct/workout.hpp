#pragma once
#include <Arduino.h>

// ---- Window buffer for GPS grade ----
struct WP {
  double lat, lon, alt, accumDist;
  bool valid;
};

class Workout {
private:
  // Variables for Distance
  // ---- Slope blending ----
  float ALPHA_IMU = 0.6;
  float GPS_GRADE_WINDOW = 25.0;

  // ---- Position tracking ----
  struct Pos {
    double lat;
    double lon;
    double alt;
    bool valid;
  };
  static const int MAXW = 60;
  WP win[MAXW];
  int win_start = 0, win_end = 0;

  // ---- Timing ----
  unsigned long lastDisplay = 0;
  unsigned long lastLog = 0;
  double slope;
  bool gpsGrade(double& grade);
public:
  void start();
  void end();
  void reset();
  //unsigned long& elapsed_workout_ms();
  Pos lastPos = { 0, 0, 0, false };
  double totalDist_m = 0;
  char buf[10];
  unsigned long elapsed_workout_ms, start_workout_ms, total_ms;
  double& get_slope(float& pitch);
  char* paceFromSpeed(double speed_m_s);
  void pushWP(double lat, double lon, double alt);
  double haversine(double lat1, double lon1, double lat2, double lon2);
};