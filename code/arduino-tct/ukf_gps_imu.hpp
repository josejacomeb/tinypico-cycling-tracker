/*
* Implementation of a Sensor Fusion via a cheap GPS + IMU
*/
#pragma once

#include <TinyGPSPlus.h>

#include "constants.hpp"
#include "konfig.h"
#include "matrix.h"
#include "ukf.h"


enum class GPSAxis {
  LATITUDE,
  LONGITUDE
};

class UKFGPSIMU {
public:
  //UKFGPSIMU();
  void init(GPSAxis ax);
  bool add_gps_position_velocity(TinyGPSLocation& coord, double& velocity);
  bool add_imu_acceleration(int16_t& acc);
  double& get_predicted_position_meters();
private:
  GPSAxis ax;
  TinyGPSLocation zeroLatLng;  // To use that calculation
  float getDistanceMetersPerAxis(TinyGPSLocation& fromPos, TinyGPSLocation& toPos, bool lat_only = false);
  /* P(k=0) variable -------------------------------------------------------------------------------------------------- */
  float_prec UKF_PINIT_data[SS_X_LEN * SS_X_LEN] = { P_INIT, 0,
                                                     0, P_INIT };
  Matrix UKF_PINIT = Matrix(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
  /* Q constant ------------------------------------------------------------------------------------------------------- */
  float_prec UKF_RVINIT_data[SS_X_LEN * SS_X_LEN] = { Rv_INIT, 0,
                                                      0, Rv_INIT };
  Matrix UKF_RvINIT = Matrix(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
  /* R constant ------------------------------------------------------------------------------------------------------- */
  float_prec UKF_RNINIT_data[SS_Z_LEN * SS_Z_LEN] = { Rn_INIT_ACC,
                                                      0, Rn_INIT_ACC };
  Matrix UKF_RnINIT = Matrix(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data);
  /* Nonlinear & linearization function ------------------------------------------------------------------------------- */
  static bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
  static bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
  /* UKF variables ---------------------------------------------------------------------------------------------------- */
  UKF* UKF_IMU;
  Matrix X = Matrix(SS_X_LEN, 1);  // State Matrix
  Matrix Y = Matrix(SS_Z_LEN, 1);  // Measurement Vector Matrix
  Matrix U = Matrix(SS_U_LEN, 1);  // Control Input
  // Varibles
  double predicted_position_meters; // Final Value


};