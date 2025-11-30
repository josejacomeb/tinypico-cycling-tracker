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
public:
  OLEDGPS();
  void init_screen();
  void init_gps();
  void init_sd();
  void init_imu();
  void update_values(double& total_distance, double& slope_percent, String& pace, double& altitude);
  void draw_wait_screen(TinyGPSTime& gps_time);
};