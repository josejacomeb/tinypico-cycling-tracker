/*
* Software for the Tinypico GPS Tracker
* Written by: josejacomeb / 2025
*/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <TinyPICO.h>

#include "oled.hpp"
#include "konfig.h"
#include "matrix.h"
#include "ukf.h"
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
bool DMPReady = false;               // Set true if DMP init was successful
uint8_t MPUIntStatus;                // Holds actual interrupt status byte from MPU
uint8_t devStatus;                   // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                 // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];              // FIFO storage buffer

// ---- Orientation/Motion Variables ----
Quaternion q;                        // [w, x, y, z] Quaternion container
VectorFloat gravity;                 // [x, y, z] Gravity vector
float ypr[3];                        // [yaw, pitch, roll] Yaw/Pitch/Roll container and gravity vector
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


/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)
/* Magnetic vector constant (align with local magnetic vector) */
float_prec IMU_MAG_B0_data[3] = {cos(0), sin(0), 0.000000};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
/* The hard-magnet bias */
float_prec HARD_IRON_BIAS_data[3] = {8.832973, 7.243323, 23.95714};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);

/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Rv_INIT     (1e-6)
#define Rn_INIT_ACC (0.0015)
#define Rn_INIT_MAG (0.0015)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec UKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix UKF_PINIT(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec UKF_RVINIT_data[SS_X_LEN*SS_X_LEN] = {Rv_INIT, 0,      0,      0,
                                                 0,      Rv_INIT, 0,      0,
                                                 0,      0,      Rv_INIT, 0,
                                                 0,      0,      0,      Rv_INIT};
Matrix UKF_RvINIT(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec UKF_RNINIT_data[SS_Z_LEN*SS_Z_LEN] = {Rn_INIT_ACC, 0,          0,          0,          0,          0,
                                                 0,          Rn_INIT_ACC, 0,          0,          0,          0,
                                                 0,          0,          Rn_INIT_ACC, 0,          0,          0,
                                                 0,          0,          0,          Rn_INIT_MAG, 0,          0,
                                                 0,          0,          0,          0,          Rn_INIT_MAG, 0,
                                                 0,          0,          0,          0,          0,          Rn_INIT_MAG};
Matrix UKF_RnINIT(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
/* UKF variables ---------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* UKF system declaration ------------------------------------------------------------------------------------------- */
UKF UKF_IMU(quaternionData, UKF_PINIT, UKF_RvINIT, UKF_RnINIT, Main_bUpdateNonlinearX, Main_bUpdateNonlinearY);

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

const unsigned int led_time = 5000;

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
  Serial.print("Starting Serial 2...");
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
  }
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
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
    }
  }
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

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;
    
    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    p = U[0][0];
    q = U[1][0];
    r = U[2][0];
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false so we can reset the UKF */
        return false;
    }
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;
    
    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0][0]
             +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1][0]
             +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2][0];

    Y[4][0] = (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0][0]
             +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1][0]
             +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2][0];

    Y[5][0] = (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0][0]
             +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1][0]
             +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2][0];
    
    return true;
}

void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
