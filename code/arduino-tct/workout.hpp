#pragma once
#include <Arduino.h>
#include "TinyGPS++.h"

// ---- Window buffer for GPS grade ----
struct WP {
  TinyGPSLocation Pos;
  double alt, accumDist;
  bool valid;
};

class Workout {
 private:
  // Variables for Distance
  // ---- Slope blending ----
  float ALPHA_IMU = 0.6, GPS_GRADE_WINDOW = 25.0;
  ;
  // ---- Position tracking ----
  struct Pos {
    TinyGPSLocation Pos;
    double alt;
    bool valid;
  };
  static const int MAXW = 60;
  WP win[MAXW];
  int win_start = 0, win_end = 0;

  // ---- Timing ----
  unsigned long lastDisplay = 0, lastLog = 0;
  double slope, totalDist_m = 0;
  bool gpsGrade(double& grade);
  unsigned long elapsed_workout_ms, start_workout_ms, total_ms;
  Pos lastPos;

 public:
  void start();
  void end();
  void reset();
  char buf[10];
  double& get_slope(float& pitch);
  char* paceFromSpeed(double speed_m_s);
  void pushWP(TinyGPSLocation& Pos1, double alt);
  double haversine(TinyGPSLocation& Pos1, TinyGPSLocation& Pos2);
  void add_distance(double distance);
  double& get_total_distance();
  unsigned long& get_elapsed_workout_time();
  unsigned long& get_total_ms();
  TinyGPSLocation& get_last_location();
  bool is_last_pos_valid();
};