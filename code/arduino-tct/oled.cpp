#include "oled.hpp"

OLEDGPS::OLEDGPS()
  : Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
}

void OLEDGPS::init_gps() {
  setTextSize(2);
  setCursor(5, 45);
  print("OK");
  display();
}

void OLEDGPS::init_sd() {
  setTextSize(2);
  setCursor(54, 45);
  print("OK");
  display();
}

void OLEDGPS::init_imu() {
  setTextSize(2);
  setCursor(100, 45);
  print("OK");
  display();
}

void OLEDGPS::init_screen() {
  if (!Adafruit_SSD1306::begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Clear the buffer
  clearDisplay();
  drawBitmap((width() - splash_width) / 2, (height() - splash_height) / 2, splash_data, splash_width, splash_height, 1);
  display();
  setTextColor(SSD1306_WHITE);
}

void OLEDGPS::update_values(double& latitude, double& longitude, double& speed, double& altitude, float ypr_ang[]) {
  setCursor(1, 0);
  clearDisplay();
  setTextSize(1);
  print("GPS: ");
  print(latitude);
  print(" , ");
  print(longitude);
  print(speed);
  print(" km/h, h: ");
  println(altitude);
  println("*--- YPR ---*");
  print(ypr_ang[0]);
  print(" ");
  print(ypr_ang[1]);
  print(" ");
  println(ypr_ang[2]);
  display();
}