/*
* Software for the Tinypico GPS Tracker
* Written by: josejacomeb / 2025
*/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>  // For get no gravity results
#include <SPI.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "oled.hpp"
#include "ukf_gps_imu.hpp"
#include "writter.hpp"
#include "workout.hpp"

// Debug mode
#define DEBUG 0

// TinyPICO PINS
// Digital UART Pins for ESP32-PICO-D4
#define TX 25
#define RX 26
// Push Button Pin
#define INPUT_PIN 33
// SD Pins control
#define CS 5
// MPU Interrupt Pin
#define INTERRUPT_PIN 2
// APA102 Dotstar
#define DOTSTAR_PWR 13
#define DOTSTAR_DATA 2
#define DOTSTAR_CLK 12

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();

// ---- MPU6050 Control/Status Variables ----
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

// ---- Orientation/Motion Variables ----
Quaternion q;         // [w, x, y, z] Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];         // [yaw, pitch, roll] Yaw/Pitch/Roll container and gravity vector
MPU6050 mpu;

// ---- Interrupt detection routine ----
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high

void DMPDataReady() {
  MPUInterrupt = true;
}

// ---- GPS Variables ----
TinyGPSPlus gps;
double speed_m_s, altitude;
unsigned long start;
const unsigned int period = 16;
unsigned int display_results_counter = 1;
const unsigned int max_display_results_counter = 60;



// ---- Application Objects ----
OLEDGPS oled_display;
Workout workout;

// ---- State Machine ----
enum class State {
  LOADING,
  WAITING,
  START_WORKOUT,
  END_WORKOUT,
  SUMMARY
};

State m_state;
double vNorth, vEast;
UKFGPSIMU sf_lat, sf_lng;

const unsigned int led_time = 5000;

struct LatLonDeg {
  double lat;  // Latitude  in degrees, range [-90, +90]
  double lon;  // Longitude in degrees, range [-180, +180]

  LatLonDeg()
    : lat(0.0), lon(0.0) {}

  LatLonDeg(double latitude_deg, double longitude_deg)
    : lat(latitude_deg), lon(longitude_deg) {}
};

struct LatLonDeg getPointAhead(
  const LatLonDeg& start,
  double distance_m,
  double bearing_deg) {

  double lat1 = start.lat * M_PI / 180.0;
  double lon1 = start.lon * M_PI / 180.0;
  double bearing = bearing_deg * M_PI / 180.0;
  double delta = distance_m / EARTH_RADIUS;

  double lat2 = asin(
    sin(lat1) * cos(delta) + cos(lat1) * sin(delta) * cos(bearing));

  double y = sin(bearing) * sin(delta) * cos(lat1);
  double x = cos(delta) - sin(lat1) * sin(lat2);

  double lon2 = lon1 + atan2(y, x);

  LatLonDeg result;
  result.lat = lat2 * 180.0 / M_PI;
  result.lon = lon2 * 180.0 / M_PI;

  return result;
}
LatLonDeg ZeroPoint;

void setup() {
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(242, 57, 29);
  tp.DotStar_Show();
  pinMode(INPUT_PIN, INPUT);
  // Used for debug output only
  Serial.begin(115200);
  oled_display.init_screen();
  // Using I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
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
  delay(led_time);
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
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(237, 230, 21);
  tp.DotStar_Show();
  delay(led_time);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Starting Serial 2...");
  while (!gps.location.isValid()) {
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
  }
  oled_display.init_gps();
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(136, 237, 21);
  tp.DotStar_Show();
  delay(led_time);
  tp.DotStar_SetPower(false);
  m_state = State::WAITING;
  sf_lat.init(GPSAxis::LATITUDE);
  sf_lng.init(GPSAxis::LONGITUDE);
  start = millis();
}

void loop() {
  if (!digitalRead(INPUT_PIN)) {
    delay(500);
    if (digitalRead(INPUT_PIN)) {
      if (m_state != State::START_WORKOUT) {
        workout.start();
        m_state = State::START_WORKOUT;
        write_header(gps);
      } else {
        workout.end();
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
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // As located in the PCB
    sf_lat.add_imu_acceleration(aaWorld.y);
    sf_lng.add_imu_acceleration(aaWorld.x);
  }
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      speed_m_s = gps.speed.isValid() ? gps.speed.mps() : 0.0;
      vNorth = speed_m_s * cos(gps.course.deg() * M_PI / 180);
      vEast = speed_m_s * sin(gps.course.deg() * M_PI / 180);
      unsigned long ts = millis();
      // distance accumulation
      if (workout.is_last_pos_valid()) {
        double d = workout.haversine(workout.get_last_location(), gps.location);
        // ignore spurious large jumps
        if (d < 50.0) {
          if (m_state == State::START_WORKOUT)
            workout.add_distance(d);
        }
      }
      workout.pushWP(gps.location, altitude);
      double speed_kmh = speed_m_s * 3.6;
      sf_lat.add_gps_position_velocity(gps.location, vNorth);
      sf_lng.add_gps_position_velocity(gps.location, vEast);
    }
  }
  LatLonDeg PointEast = getPointAhead(ZeroPoint, sf_lat.get_predicted_position_meters(), 90);
  LatLonDeg PointNorthEast = getPointAhead(PointEast, sf_lng.get_predicted_position_meters(), 180);

  if (millis() - start > period) {
#if DEBUG
    Serial.print("YPR Rad: ");
    Serial.print(ypr[0]);
    Serial.print(", ");
    Serial.print(ypr[1]);
    Serial.print(", ");
    Serial.print(ypr[2]);
    Serial.print(" Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Lng: ");
    Serial.print(gps.location.lng(), 6);
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
        oled_display.update_values(workout.get_total_distance(), workout.get_slope(ypr[1]), workout.paceFromSpeed(speed_m_s), altitude, workout.get_elapsed_workout_time());
        write_gpx(gps.location, altitude, gps.date, gps.time);
        break;
      case State::END_WORKOUT:
        write_file(trk_end_tag);
        write_file(trk_end_segm);
        write_file("</gpx>");
        m_state = State::SUMMARY;
        break;
      case State::SUMMARY:
        display_results_counter++;
        oled_display.display_total_results(workout.get_total_distance(), workout.get_total_ms());
        if (display_results_counter > max_display_results_counter) {
          display_results_counter = 1;
          m_state = State::WAITING;
        }
        break;
    }
    start = millis();
  }
}

void SPEW_THE_ERROR(char const* str) {
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
  cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
  Serial.println(str);
#else
  /* Silent function */
#endif
  while (1)
    ;
}
