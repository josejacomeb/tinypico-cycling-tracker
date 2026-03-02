#pragma once
#include <math.h>

// TinyPICO PINS
// Digital UART Pins for ESP32-PICO-D4
#define TX 25
#define RX 26
// Push Button Pin
#define INPUT_PIN 33
// SD Pins control
#define CS 5
// MPU Interrupt Pin
#define INTERRUPT_PIN 32
// APA102 Dotstar
#define DOTSTAR_PWR 13
#define DOTSTAR_DATA 2  // Internal TinyPICO wiring — do not drive manually
#define DOTSTAR_CLK 12

const double EARTH_RADIUS = 6371 * 1000.0;  // Meters
const float deg_2_rad_const = M_PI / 180.0f;

// This constant is MPU6050 raw ADC counts per 1g at ±2g full scale (from datasheet).
// Dividing raw counts by LSB_PER_G gives acceleration in units of g, NOT m/s².
// To convert to m/s², also multiply by SURFACE_GRAVITY:
// float acc_ms2 = (raw_counts / (float)LSB_PER_G) * SURFACE_GRAVITY;
const int LSB_PER_G = 16384;

const float SURFACE_GRAVITY = 9.771927346f;  // At Ecuador, ~2,729 m altitude

/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0 (1)

/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT (2.5f)                            // uBlox 6M horizontal position accuracy (meters), used to seed UKF covariance
const float Rv_INIT = (1e-3 * SURFACE_GRAVITY);  // Calibration Tolerance
#define Rn_INIT_ACC (0.0015f)

/* Supply your gyro offsets here, scaled for min sensitivity */
// Please see: https://github.com/ElectronicCats/mpu6050/blob/master/examples/IMU_Zero/IMU_Zero.ino
const int16_t XAccelOffset = -400;
const int16_t YAccelOffset = -2114;
const int16_t ZAccelOffset = 1568;
const int16_t XGyroOffset = -143;
const int16_t YGyroOffset = -5;
const int16_t ZGyroOffset = -329;
