/*
* Software for the Tinypico GPS Tracker
* Written by: josejacomeb / 2025
*/
#include <I2Cdev.h>
#include <SPI.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "writter.hpp"
#include "oled.hpp"

// TinyPICO PINS
// Digital UART Pins for ESP32-PICO-D4
#define TX 25
#define RX 26
#define INPUT_PIN 33
#define PRINT 0
// SD Pins control
const unsigned int CS = 5;
// MPU Interrupt Pin
int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();

enum class State {
  LOADING,
  WAITING,
  START_WORKOUT,
  END_WORKOUT,
  SUMMARY
};
enum State m_state;

bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
MPU6050 mpu;

// The TinyGPS++ object
TinyGPSPlus gps;

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}
double lat, lng, speed_m_s, altitude;
float ypr_ang[3] = { 0.0f, 0.0f, 0.0f };
unsigned long start, elapsed_workout_ms, start_workout_ms, total_ms;
const unsigned int period = 16;
unsigned int display_results_counter = 1;
const unsigned int max_display_results_counter = 60;
OLEDGPS oled_display;

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
Pos lastPos = { 0, 0, 0, false };
double totalDist_m = 0;

// ---- Window buffer for GPS grade ----
struct WP {
  double lat, lon, alt, accumDist;
  bool valid;
};
const int MAXW = 60;
WP win[MAXW];
int win_start = 0, win_end = 0;

// ---- Timing ----
unsigned long lastDisplay = 0;
unsigned long lastLog = 0;
String pace;
double slope;

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dlat = (lat2 - lat1) * DEG_TO_RAD;
  double dlon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * sin(dlon / 2) * sin(dlon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

void pushWP(double lat, double lon, double alt) {
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

bool gpsGrade(double &grade) {
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

String paceFromSpeed(double speed_m_s) {
  if (speed_m_s < 0.5) return "--:--";
  double secpkm = 1000.0 / speed_m_s;
  int mm = int(secpkm / 60);
  int ss = int(secpkm - mm * 60);
  if (ss == 60) {
    mm++;
    ss = 0;
  }
  char buf[10];
  sprintf(buf, "%02d:%02d", mm, ss);
  return String(buf);
}

void setup() {
  tp.DotStar_SetPixelColor(255, 0, 255);
  pinMode(INPUT_PIN, INPUT);
  // Used for debug output only
  Serial.begin(115200);
  oled_display.init_screen();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("SD initialization failed!");
    while (1)
      ;
  }
  oled_display.init_sd();
  delay(1000);
  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  oled_display.init_imu();
  delay(1000);
  tp.DotStar_SetPixelColor(0, 255, 0);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial.print("Starting Serial 2...");
  while (!gps.location.isValid()) {
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
  }
  oled_display.init_gps();
  tp.DotStar_SetPixelColor(255, 0, 0);
  delay(1000);
  tp.DotStar_SetBrightness(0);
  m_state = State::WAITING;
  start = millis();
}

void loop() {
  if (!digitalRead(INPUT_PIN)) {
    delay(500);
    if (digitalRead(INPUT_PIN)) {
      if (m_state != State::START_WORKOUT) {
        start_workout_ms = millis();
        m_state = State::START_WORKOUT;
        write_header(gps);
      } else {
        total_ms = millis() - start_workout_ms;
        m_state = State::END_WORKOUT;
      }
    }
  }
  // Cycle the DotStar LED colour every 25 milliseconds
  if (!DMPReady) return;  // Stop the program if DMP programming fails.
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr_ang[0] = ypr[0] * 180 / M_PI;
    ypr_ang[1] = ypr[1] * 180 / M_PI;
    ypr_ang[2] = ypr[2] * 180 / M_PI;
  }
  float imuSlopePercent = tan(ypr_ang[1] * DEG_TO_RAD) * 100.0;
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
      altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      speed_m_s = gps.speed.isValid() ? gps.speed.mps() : 0.0;
      unsigned long ts = millis();
      // distance accumulation
      if (lastPos.valid) {
        double d = haversine(lastPos.lat, lastPos.lon, lat, lng);
        // ignore spurious large jumps
        if (d < 50.0) {
          if (m_state == State::START_WORKOUT)
            totalDist_m += d;
        }
      }
      lastPos = { lat, lng, altitude, true };
      pushWP(lat, lng, altitude);

      double ggrade = 0, slope = imuSlopePercent;
      if (gpsGrade(ggrade)) {
        slope = ALPHA_IMU * imuSlopePercent + (1.0 - ALPHA_IMU) * ggrade;
      }

      double speed_kmh = speed_m_s * 3.6;
      pace = paceFromSpeed(speed_m_s);
    }
  }
  if (millis() - start > period) {
#if PRINT
    Serial.print("YPR: ");
    Serial.print(ypr_ang[0]);
    Serial.print(", ");
    Serial.print(ypr_ang[1]);
    Serial.print(", ");
    Serial.print(ypr_ang[2]);
    Serial.print(" Lat: ");
    Serial.print(lat, 6);
    Serial.print(" Lng: ");
    Serial.print(lng, 6);
    Serial.print(" speed: ");
    Serial.print(speed);
    Serial.print(" altitude: ");
    Serial.print(altitude);
    Serial.print(" Time in UTC: ");
    Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
#endif
    switch (m_state) {
      case State::WAITING:
        oled_display.draw_wait_screen(gps.time);
        break;
      case State::START_WORKOUT:
        elapsed_workout_ms = millis() - start_workout_ms;
        oled_display.update_values(totalDist_m, slope, pace, altitude, elapsed_workout_ms);
        write_gpx(lat, lng, altitude, gps.date, gps.time);
        break;
      case State::END_WORKOUT:
        write_file(trk_end_tag);
        write_file(trk_end_segm);
        write_file("</gpx>");
        m_state = State::SUMMARY;
        break;
      case State::SUMMARY:
        display_results_counter++;
        oled_display.display_total_results(totalDist_m, total_ms);
        if (display_results_counter > max_display_results_counter) {
          display_results_counter = 1;
          m_state = State::WAITING;
        }
        break;
    }
    start = millis();
  }
}
