/*
* Data Logger for the TinyPico Cycling Tracker
* Retrieves data from GPS, IMU and save into a .ino file
* Written by: josejacomeb / 2025
*/

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>  // For get no gravity results
#include <SPI.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "constants.hpp"
#include "konfig.h"
#include "writter.hpp"

const unsigned int led_time = 1000;
double speed_m_s, altitude, latitude, longitude, vNorth, vEast;
unsigned long start, wait_time, elapsed_time, start_time;
int16_t accNorth, accEast;

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
const char* header_data = "time,alt,lat,lng,vNorth,vEast,accNorth,accEast";
char buffer[128];

void setup() {
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(242, 57, 29);
  tp.DotStar_Show();
  pinMode(INPUT_PIN, INPUT);
  // Used for debug output only
  Serial.begin(115200);
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
  while (!gps.location.isValid()) {
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
  }
  tp.DotStar_Clear();
  tp.DotStar_SetPixelColor(136, 237, 21);
  tp.DotStar_Show();
  delay(led_time);
  tp.DotStar_SetPower(false);
}

void loop() {
  altitude = 0;
  vNorth = 0;
  vEast = 0;
  latitude = 0;
  longitude = 0;
  start = millis();
  if (!digitalRead(INPUT_PIN)) {
    delay(500);
    if (digitalRead(INPUT_PIN)) {
      start_writing = !start_writing;
      if (start_writing) {
        tp.DotStar_Clear();
        tp.DotStar_SetPower(true);
        tp.DotStar_SetPixelColor(136, 237, 21);
        tp.DotStar_Show();
        set_file_name(gps, extension);
        write_file(header_data);
        start_time = millis();
      } else
        tp.DotStar_SetPower(false);
    }
  }

  // IMU Update
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
    accNorth = aaWorld.y;
    accEast = aaWorld.x;
  }
  // GPU Update
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      speed_m_s = gps.speed.isValid() ? gps.speed.mps() : 0.0;
      vNorth = speed_m_s * cos(gps.course.deg() * M_PI / 180);
      vEast = speed_m_s * sin(gps.course.deg() * M_PI / 180);
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }
  if (start_writing) {
    sprintf(buffer, "%lu,%.2f,%.6f,%.6f,%.6f,%.6f,%d,%d",
            millis() - start_time, altitude, latitude, longitude,
            vNorth, vEast, accNorth, accEast);
    write_file(buffer);
  }
  elapsed_time = millis() - start;
  wait_time = SS_DT_MILIS > elapsed_time ? SS_DT_MILIS - elapsed_time : 0;
  delay(wait_time);
}