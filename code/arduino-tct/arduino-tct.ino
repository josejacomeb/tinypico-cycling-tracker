/*
* Software for the Tinypico GPS Tracker
* Written by: josejacomeb / 2025
*/
#include <I2Cdev.h>
#include <SPI.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "oled.hpp"
#include "writter.hpp"
#include "workout.hpp"

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
unsigned long start;
const unsigned int period = 16;
unsigned int display_results_counter = 1;
const unsigned int max_display_results_counter = 60;
String pace;
OLEDGPS oled_display;
Workout workout;

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
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr_ang[0] = ypr[0] * 180 / M_PI;
    ypr_ang[1] = ypr[1] * 180 / M_PI;
    ypr_ang[2] = ypr[2] * 180 / M_PI;
  }
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
      altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      speed_m_s = gps.speed.isValid() ? gps.speed.mps() : 0.0;
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
      pace = workout.paceFromSpeed(speed_m_s);
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
        oled_display.update_values(workout.get_total_distance(), workout.get_slope(ypr[1]), workout.paceFromSpeed(speed_m_s), altitude, workout.get_elapsed_workout_time());
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
