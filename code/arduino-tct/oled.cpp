#include "oled.hpp"

OLEDGPS::OLEDGPS() {
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  // Clear the buffer
  display.clearDisplay();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  display.setTextColor(SSD1306_WHITE);
}

OLEDGPS::~OLEDGPS() {
}

void OLEDGPS::update_values(double& latitude, double& longitude, double& speed, double& altitude, float ypr_ang[]) {
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