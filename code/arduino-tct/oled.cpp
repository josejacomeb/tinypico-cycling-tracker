#include "oled.hpp"

OLEDGPS::OLEDGPS()
  : Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 100000, 100000) {
}

void OLEDGPS::init_screen() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.drawBitmap((display.width() - LOGO_WIDTH) / 2, (display.height() - LOGO_HEIGHT) / 2, logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
}


void OLEDGPS::update_values(double& latitude, double& longitude, double& speed, double& altitude, float ypr_ang[]) {
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(1, 0);
  display.clearDisplay();
  display.setTextSize(1);
  display.print("Lat: ");
  display.print(latitude);
  display.print(" lng: ");
  display.print(longitude);
  display.print(" sp: ");
  display.print(speed);
  display.print(" h: ");
  display.println(altitude);
  display.println("*--- YPR ---*");
  display.print(ypr_ang[0]);
  display.print(" ");
  display.print(ypr_ang[1]);
  display.print(" ");
  display.println(ypr_ang[2]);
  display.display();
}