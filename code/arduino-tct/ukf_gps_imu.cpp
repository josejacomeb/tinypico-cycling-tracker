#include "ukf_gps_imu.hpp"

void UKFGPSIMU::init(GPSAxis axis) {
  /* UKF system declaration ------------------------------------------------------------------------------------------- */
  UKF_IMU = new UKF(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT, UKFGPSIMU::Main_bUpdateNonlinearX, UKFGPSIMU::Main_bUpdateNonlinearY);
  /* UKF initialization ----------------------------------------- */
  /* x(k=0) = [1 0 0 0]' */
  X.vSetToZero();
  X[0][0] = 1.0;
  UKF_IMU->vReset(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);
  ax = axis;
}

bool UKFGPSIMU::add_gps_position_velocity(TinyGPSLocation& coord, double& velocity) {
  /* ====== Inputs GPS Measurement matrix a.k.a Z(INPUT measurement (GPS) matrix) ===== */
  Y[0][0] = getDistanceMetersPerAxis(coord, zeroLatLng, true);  // Position
  Y[1][0] = velocity;
  // Update Step
  /* ============================= Update the Kalman Filter ============================== */
  if (!UKF_IMU->bUpdate(Y, U)) {
    X.vSetToZero();
    // TODO: Clean up this code
    // Conversion to Lat Long
    predicted_position_meters = UKF_IMU->GetX()[0][0];
    UKF_IMU->vReset(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);
  }
  return true;
}
bool UKFGPSIMU::add_imu_acceleration(int16_t& acc) {
  /* ======= Inputs IMU Control Matrix a.k.a. u(INPUT control (accelerometer) matrix) */
  U[0][0] = acc;
  return true;
}
double& UKFGPSIMU::get_predicted_position_meters() {
  return predicted_position_meters;
}
float UKFGPSIMU::getDistanceMetersPerAxis(TinyGPSLocation& fromPos, TinyGPSLocation& toPos, bool lat_only) {
  // Conversion from Degrees to Radians
  float to_pos_lng = toPos.lng();
  float to_pos_lat = toPos.lat();
  float from_pos_lng = fromPos.lng();
  float from_pos_lat = fromPos.lat();
  if (ax == GPSAxis::LONGITUDE) {
    to_pos_lng = 0.0;
    from_pos_lng = 0.0;
  } else {
    to_pos_lat = 0.0;
    from_pos_lat = 0.0;
  }
  float deltaLon = (to_pos_lng - from_pos_lng) * deg_2_rad_const;
  float deltaLat = (to_pos_lat - from_pos_lat) * deg_2_rad_const;
  float a = pow(sin(deltaLat / 2.0), 2) + cos(from_pos_lat * deg_2_rad_const) * cos(to_pos_lat * deg_2_rad_const) * pow(sin(deltaLon / 2.0), 2);
  return 2 * atan2(sqrt(a), sqrt(1.0f - a)) * EARTH_RADIUS;
}

bool UKFGPSIMU::Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U) {
  /* Insert the nonlinear update transformation here
  *          x(k+1) = f[x(k), u(k)]
  *   p(k+1)=p(k)+v(k)*Δt+1/2*​a(k)Δt^2
  *   v(k+1)=v(k)+a(k)*Δt
  */
  X_Next[0][0] = X[0][0] + X[0][0] * SS_DT_MILIS + U[0][0] * pow(SS_DT_MILIS, 2) / 2;
  X_Next[1][0] = X[1][0] + U[1][0] * pow(SS_DT_MILIS, 2);

  return true;
}

bool UKFGPSIMU::Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U) {
  /* Insert the nonlinear measurement transformation here
    *          y(k)   = h[x(k), u(k)]
    *   
    */
  Y[0][0] = X[0][0];
  Y[1][0] = X[1][0];
  return true;
}