#pragma once
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>

// OLED Display files values
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class OLEDGPS : public Adafruit_SSD1306 {
 private:
  void draw_bmp(int16_t x, int16_t y, const uint8_t* data, int16_t d_width, int16_t d_height, bool display_value = true);
  uint8_t hour = 0;
  char time_buffer[9];
  unsigned int change_counter = 0;
  const unsigned int switch_altitude = 20;
  const unsigned int switch_elapsed_time = 2 * switch_altitude;
  const unsigned long conversion_hours_2_seconds = 3600, conversion_minutes_2_seconds = 60;
  char chrono_buf[8];

 public:
  OLEDGPS();
  void init_screen();
  void init_gps();
  void init_sd();
  void init_imu();
  void update_values(double& total_distance, double& slope_percent, char* pace, double& altitude, unsigned long& ellapsed_workout_ms);
  void draw_wait_screen(TinyGPSTime& gps_time);
  void display_total_results(double& total_distance, unsigned long& ellapsed_workout_ms);
  const char* convert_ms_to_hms(unsigned long& ellapsed_workout_ms);
};