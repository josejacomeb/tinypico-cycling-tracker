/*
* Data Logger for the TinyPico Cycling Tracker
* Retrieves data from GPS, IMU and saves data into a .csv file
* Written by: josejacomeb / 2025
*/

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "constants.hpp"
#include "konfig.h"
#include "writer.hpp"

// See the data in the Arduino Serial Plotter https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter/
#define SERIAL_PLOTTER 1

const unsigned int led_time = 1000;
double speed_m_s, altitude, latitude, longitude, vNorth, vEast, course_rad, course_deg;
unsigned long start, wait_time, elapsed_time, start_time;
// Acceleration in m/s² (converted from raw DMP counts at read time).
// Using float keeps units consistent with vNorth/vEast (m/s) in the CSV.
float accNorth, accEast;
float elapsed_seconds;

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

bool start_writing = false;
const char* extension = "csv";
const char* header_data = "time,alt,lat,lng,vNorth,vEast,accNorth,accEast,gpsUpdate";
// GPS columns (lat, lng, vNorth, vEast, alt) contain NaN when no GPS fix
char buffer[128];

// GPS fields written as NaN when no fix arrives in the current 20 ms cycle.
// IMU acceleration is always valid and written every cycle.
bool gps_updated_this_cycle = false;

// GPS acquisition timeout to avoid hanging forever in setup().
const unsigned long GPS_TIMEOUT_MS = 300000UL;  // 5 minutes

void setup() {
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(242, 57, 29);
  tp.DotStar_Show();
  // Button should connect INPUT_PIN to GND when pressed (active-low logic).
  pinMode(INPUT_PIN, INPUT_PULLUP);
  // Used for debug output only
  Serial.begin(115200);
  // Using I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  /*Initialize device*/
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("SD initialization failed!");
    while (1)
      ;
  }
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

  // Calibration values defined in the constants.hpp
  mpu.setXAccelOffset(XAccelOffset);
  mpu.setYAccelOffset(YAccelOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setXGyroOffset(XGyroOffset);
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
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
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(237, 230, 21);
  tp.DotStar_Show();
  delay(led_time);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Starting Serial 2...");

  // DotStar blinks orange during GPS search; turns green on fix.
  unsigned long gps_wait_start = millis();
  bool blink_state = false;
  Serial.println("Waiting for GPS fix...");
  unsigned long gps_blink_start = millis();
  unsigned long gps_timeout_start = millis();
  while (!gps.location.isValid()) {
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
    // Blink orange to show we're waiting
    if (millis() - gps_blink_start > 500) {
      blink_state = !blink_state;
      if (blink_state) tp.DotStar_SetPixelColor(237, 180, 21);
      else tp.DotStar_Clear();
      tp.DotStar_Show();
      gps_blink_start = millis();
    }
    // Timeout — allow device to continue without GPS fix
    if (millis() - gps_timeout_start > GPS_TIMEOUT_MS) {
      Serial.println("GPS timeout — continuing without fix.");
      break;
    }
  }

  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(136, 237, 21);
  tp.DotStar_Show();
  delay(led_time);
  tp.DotStar_SetPower(false);
  Serial.println("Ready...");
}

void loop() {
  // Instead, only write a CSV row when GPS has actually produced a new fix this cycle.
  // The old approach wrote lat=0,lon=0 on every cycle without a GPS update.
  gps_updated_this_cycle = false;
  start = millis();

  // ---- Button debounce (active-low with INPUT_PULLUP) ----
  if (!digitalRead(INPUT_PIN)) {
    delay(50);  // debounce
    if (!digitalRead(INPUT_PIN)) {
      // Wait for release
      while (!digitalRead(INPUT_PIN))
        ;
      start_writing = !start_writing;
      if (start_writing) {
        tp.DotStar_Clear();
        tp.DotStar_SetPower(true);
        tp.DotStar_SetPixelColor(136, 237, 21);
        tp.DotStar_Show();
        set_file_name(gps, extension);
        open_log_file();  // Manually Start the log file because we're not writting to gpx
        write_file(header_data);
        start_time = millis();
      } else {
        tp.DotStar_SetPower(false);
        close_log_file();
      }
    }
  }

  // ---- IMU Update ----
  if (!DMPReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Convert raw DMP counts → m/s² immediately.
    // LSB_PER_G = 16384 counts/g at ±2g full scale (MPU6050 datasheet).
    // Multiplying by SURFACE_GRAVITY converts from g to m/s².
    accNorth = SURFACE_GRAVITY * (aaWorld.y / (float)LSB_PER_G);
    accEast = SURFACE_GRAVITY * (aaWorld.x / (float)LSB_PER_G);
  }

  // ---- GPS Update ----
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      altitude = gps.altitude.isValid() ? gps.altitude.meters() : NAN;
      if (gps.speed.isValid() && gps.course.isValid()) {
        speed_m_s = gps.speed.mps();
        course_deg = gps.course.deg();
        course_rad = course_deg * M_PI / 180.0;
        vNorth = speed_m_s * cos(course_rad);
        vEast = speed_m_s * sin(course_rad);
      } else {
        vNorth = NAN;
        vEast = NAN;
      }
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      gps_updated_this_cycle = true;
    }
  }

  // Write a row every SS_DT_MILIS ms cycle — IMU data is always present.
  // GPS columns are written as NaN when no fix arrived this cycle so that
  // sensor_fusion.py can distinguish predict-only rows from GPS update rows.
  if (start_writing) {
    elapsed_seconds = (millis() - start_time) / 1000.0f;
    if (gps_updated_this_cycle) {
      sprintf(buffer, "%.4f,%.2f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,1",
              elapsed_seconds, altitude, latitude, longitude,
              vNorth, vEast, accNorth, accEast);
    } else {
      // NaN in GPS columns; accNorth/accEast always valid
      sprintf(buffer, "%.4f,NaN,NaN,NaN,NaN,NaN,%.4f,%.4f,0",
              elapsed_seconds, accNorth, accEast);
    }
    write_file(buffer);
  }

  elapsed_time = millis() - start;
  wait_time = SS_DT_MILIS > elapsed_time ? SS_DT_MILIS - elapsed_time : 0;
  delay(wait_time);
#ifdef SERIAL_PLOTTER
  Serial.print("speed_m_s:");
  Serial.print(speed_m_s);
  Serial.print(",");
  Serial.print("course_deg:");
  Serial.print(course_deg);
  Serial.print(",");
  Serial.print("course_rad:");
  Serial.print(course_rad);
  Serial.print(",");
  Serial.print("vNorth:");
  Serial.print(vNorth);
  Serial.print(",");
  Serial.print("vEast:");
  Serial.print(vEast);
  Serial.print(",");
  Serial.print("accNorth:");
  Serial.print(accNorth);
  Serial.print(",");
  Serial.print("accEast:");
  Serial.println(accEast);
#endif
}
