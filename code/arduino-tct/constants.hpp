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