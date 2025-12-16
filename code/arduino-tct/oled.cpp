#include "graphics.hpp"
#include "oled.hpp"

// ---- Constructor ----

OLEDGPS::OLEDGPS()
  : Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
}

// ---- Drawing Methods ----

void OLEDGPS::draw_bmp(int16_t x, int16_t y, const uint8_t* data, int16_t d_width, int16_t d_height, bool display_value) {
  drawBitmap(x, y, data, d_width, d_height, 1);
  if (display_value)
    display();
}

// ---- Initialization Methods ----

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

// ---- Conversion Methods ----

const char* OLEDGPS::convert_ms_to_hms(unsigned long& ellapsed_workout_ms) {
  long total_seconds = ellapsed_workout_ms / 1000;
  unsigned long hours = (total_seconds) / conversion_hours_2_seconds;
  unsigned long total_minutes = (total_seconds) % conversion_hours_2_seconds;
  unsigned long minutes = total_minutes / conversion_minutes_2_seconds;
  unsigned long seconds = total_minutes % conversion_minutes_2_seconds;
  sprintf(chrono_buf, "%02d:%02d:%02d", hours, minutes, seconds);
  return chrono_buf;
}

// ---- Update/Display Methods ----

void OLEDGPS::update_values(double& total_distance, double& slope_percent, char* pace, double& altitude, unsigned long& ellapsed_workout_ms) {
  clearDisplay();
  draw_bmp(0, 0, active_data, active_width, active_height, false);
  setTextSize(1);
  setCursor(1, 1);

  change_counter++;
  if (change_counter <= switch_altitude) {
    print("h: ");
    print((int)altitude);
    print("m");
  } else if (change_counter > switch_altitude && change_counter < switch_elapsed_time) {
    print(convert_ms_to_hms(ellapsed_workout_ms));
  } else {
    change_counter = 0;
  }
  setCursor(70, 3);
  print(total_distance);
  print("m");
  setCursor(70, 26);
  print(slope_percent);
  print("%");
  setCursor(70, 50);
  print(pace);
  print("/km");
  display();
}

// ---- Screen Display Methods ----

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

void OLEDGPS::display_total_results(double& total_distance, unsigned long& ellapsed_workout_ms) {
  clearDisplay();
  setTextSize(1);
  setCursor(1, 1);
  println("Total Distance: ");
  setTextSize(3);
  print(total_distance);
  println("m");
  setTextSize(1);
  println("Elapsed time: ");
  setTextSize(2);
  println(convert_ms_to_hms(ellapsed_workout_ms));
  display();
}