#pragma once
#include <math.h>

const double EARTH_RADIUS = 6371 * 1000.0;  // Meters
const float deg_2_rad_const = M_PI / 180.0f;

/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0 (1)

/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT (10.)
#define Rv_INIT (1e-6)
#define Rn_INIT_ACC (0.0015)

/* Supply your gyro offsets here, scaled for min sensitivity */
// Please see: https://github.com/ElectronicCats/mpu6050/blob/master/examples/IMU_Zero/IMU_Zero.ino
const int16_t XAccelOffset = -383;
const int16_t YAccelOffset = -2063;
const int16_t ZAccelOffset = 1559;
const int16_t XGyroOffset = -147;
const int16_t YGyroOffset = -4;
const int16_t ZGyroOffset = -341;
