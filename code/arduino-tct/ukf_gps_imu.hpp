/*
 * Implementation of a Sensor Fusion via a cheap GPS + IMU
 */
#pragma once

#include "constants.hpp"
#include "data_types.hpp"
#include "konfig.h"
#include "matrix.h"
#include "ukf.h"

enum class GPSAxis
{
  LATITUDE,
  LONGITUDE
};

class UKFGPSIMU
{
public:
  // UKFGPSIMU();
  void init(GPSAxis axis, LatLonDeg &initial_coord, double &initial_velocity);
  bool add_gps_position_velocity(LatLonDeg &coord, double &velocity);
  bool add_imu_acceleration(float &acc);
  double &get_predicted_position_meters();

private:
  GPSAxis ax;
  LatLonDeg zeroLatLng; // To use that calculation
  float getDistanceMetersPerAxis(LatLonDeg &fromPos, LatLonDeg &toPos);
  // Initial covariance matrix P(k=0) variable
  float_prec UKF_PINIT_data[SS_X_LEN * SS_X_LEN] = {P_INIT, 0,
                                                    0, P_INIT};
  Matrix UKF_PINIT = Matrix(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
  // Process noise covariance Q constant
  float_prec UKF_RVINIT_data[SS_X_LEN * SS_X_LEN] = {Rv_INIT, 0,
                                                     0, Rv_INIT};
  Matrix UKF_RvINIT = Matrix(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
  // Measurement noise covariance R constant
  float_prec UKF_RNINIT_data[SS_Z_LEN * SS_Z_LEN] = {Rn_INIT_ACC,
                                                     0, Rn_INIT_ACC};
  Matrix UKF_RnINIT = Matrix(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data);
  /* Nonlinear & linearization function ------------------------------------------------------------------------------- */
  static bool Main_bUpdateNonlinearX(Matrix &X_Next, const Matrix &X, const Matrix &U);
  static bool Main_bUpdateNonlinearY(Matrix &Y, const Matrix &X, const Matrix &U);
  /* UKF variables ---------------------------------------------------------------------------------------------------- */
  UKF *UKF_IMU;
  // State vector [p, V]
  Matrix X = Matrix(SS_X_LEN, 1);
  // Measurement Vector Matrix
  Matrix Y = Matrix(SS_Z_LEN, 1);
  // Control Input Vector
  Matrix U = Matrix(SS_U_LEN, 1);
  // Variables
  double predicted_position_meters;
};