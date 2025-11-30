#include "graphics.hpp"
#include "oled.hpp"

OLEDGPS::OLEDGPS()
  : Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
}
void OLEDGPS::draw_bmp(int16_t x, int16_t y, const uint8_t* data, int16_t d_width, int16_t d_height, bool display_value) {
  drawBitmap(x, y, data, d_width, d_height, 1);
  if (display_value)
    display();
}

void OLEDGPS::init_gps() {
  draw_bmp(3, 21, gps_data, gps_width, gps_height);
}

void OLEDGPS::init_sd() {
  draw_bmp(94, 21, sd_data, sd_width, sd_height);
}

void OLEDGPS::init_imu() {
  draw_bmp(42, 21, imu_data, imu_width, imu_height);
}

void OLEDGPS::init_screen() {
  if (!Adafruit_SSD1306::begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Clear the buffer
  clearDisplay();
  drawBitmap(1, 1, logo_data, logo_width, logo_height, 1);
  display();
  setTextColor(SSD1306_WHITE);
}

void OLEDGPS::update_values(double& total_distance, double& slope_percent, String& pace, double& altitude) {
  clearDisplay();
  draw_bmp(0, 0, active_data, active_width, active_height, false);
  setTextSize(1);
  setCursor(1, 1);
  print("h: ");
  println((int)altitude);
  setCursor(70, 3);
  println(total_distance);
  setCursor(70, 26);
  println(slope_percent);
  setCursor(70, 50);
  println(pace);
  display();
}

void OLEDGPS::draw_wait_screen(TinyGPSTime& gps_time) {
  clearDisplay();
  draw_bmp(0, 0, load_data, load_width, load_height, false);
  setCursor(55, 2);
  setTextSize(1);
  hour = gps_time.hour();
  if (hour < 5)
    hour += 24;
  hour -= 5;
  snprintf(time_buffer, sizeof(time_buffer), "%02u:%02u:%02uZ", hour, gps_time.minute(), gps_time.second());
  print(time_buffer);
  display();
}