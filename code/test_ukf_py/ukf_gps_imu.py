from dataclasses import dataclass

from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import numpy as np
from constants import X_LEN, Z_LEN, U_LEN, EARTH_RADIUS, P_INIT, GPS_VEL_STD
from data_types import LatLonDeg, GPSAxis
import math


def fx_axis(x, dt, a):
    """
    State transition: constant-acceleration kinematic model.
    State vector x = [position (m), velocity (m/s)]
    Input a = scalar acceleration (m/s²) along the axis.
    """
    p, v = x
    p_new = p + v * dt + 0.5 * a * dt**2
    v_new = v + a * dt
    return np.array([p_new, v_new])


def hx_axis(x):
    """Measurement function: GPS observes position and velocity directly."""
    return np.array([x[0], x[1]])


@dataclass
class UKFGPSIMU:
    """
    One-dimensional UKF fusing IMU acceleration with GPS position+velocity.

    Predict / update separation
    ---------------------------
    - add_imu_acceleration() must be called on EVERY 20 ms IMU row (50 Hz).
      It runs the UKF predict step using the kinematic model:
        p_new = p + v·dt + ½·a·dt²
        v_new = v + a·dt
    - add_gps_position_velocity() is called ONLY on rows that carry a fresh
      GPS fix (~1 Hz). It runs the UKF update (correction) step.
    - After each call to add_imu_acceleration(), the position and velocity
      properties reflect the best estimate for that instant — even between
      GPS fixes (dead-reckoning driven by IMU alone).

    Units
    -----
    position  : metres from the zero_lat_lng origin
    velocity  : m/s  (positive = North for LATITUDE axis, East for LONGITUDE)
    accstd    : m/s² (IMU noise std dev, used to build Q)
    posstd    : m    (GPS position noise std dev, used to build R)
    """

    ax: GPSAxis  # Axis this filter tracks (LATITUDE or LONGITUDE)
    posstd: float  # GPS position noise std dev (m)
    accstd: float  # IMU acceleration noise std dev (m/s²)
    dt: float = 0.1  # nominal sample period (s) — matches SS_DT_MILIS=100

    def __post_init__(self):
        self.X_LEN = X_LEN
        self.Z_LEN = Z_LEN
        self.U_LEN = U_LEN
        self.zero_lat_lng = LatLonDeg()  # Origin (0.0, 0.0)

        self.points = MerweScaledSigmaPoints(
            n=self.X_LEN, alpha=0.1, beta=2.0, kappa=1.0
        )
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.X_LEN,
            dim_z=self.Z_LEN,
            # NOTE: self.a is accessed via closure. Python closures capture
            # variables by reference, so the lambda always reads the latest
            # value of self.a set by add_imu_acceleration().
            fx=lambda x, dt: fx_axis(x, self.dt, self.a),
            hx=hx_axis,
            dt=self.dt,
            points=self.points,
        )

        self.ukf.x = np.zeros(self.X_LEN)

        # Process noise covariance (IMU uncertainty)
        self.ukf.Q = np.diag(
            [
                self.accstd * self.accstd,
                self.accstd * self.accstd,
            ]
        )
        self.ukf.R = np.diag(
            [
                self.posstd**2,
                self.posstd**2,
            ]
        )
        # Initial Covariance
        self.ukf.P = np.eye(self.X_LEN) * P_INIT

        self.a = 0.0
        self.current_timestamp = 0.0

    def init_position_velocity(self, pos: LatLonDeg, vel: float, timestamp: float):
        self.ukf.x[0] = self.get_distance_meters_per_axis(
            pos, self.zero_lat_lng, self.ax
        )
        self.ukf.x[1] = vel
        self.current_timestamp = timestamp

    @staticmethod
    def get_distance_meters_per_axis(
        from_pos: LatLonDeg, to_pos: LatLonDeg, axis: GPSAxis
    ) -> float:
        """
        Returns the SIGNED distance (meters) between two positions along one axis.
        """
        to_lng, to_lat = to_pos.lon, to_pos.lat
        from_lng, from_lat = from_pos.lon, from_pos.lat
        # Logic mirrored from C++ snippet:
        # If axis is LONGITUDE, it zeros out the longitude variables.
        if axis == GPSAxis.LONGITUDE:
            to_lat = 0.0
            from_lat = 0.0
        else:
            to_lng = 0.0
            from_lng = 0.0

        delta_lon = math.radians(to_lng - from_lng)
        delta_lat = math.radians(to_lat - from_lat)

        a = (
            math.sin(delta_lat / 2.0) ** 2
            + math.cos(math.radians(from_lat))
            * math.cos(math.radians(to_lat))
            * math.sin(delta_lon / 2.0) ** 2
        )
        distance = 2 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a)) * EARTH_RADIUS
        return distance

    @property
    def position(self) -> float:
        return self.ukf.x[0]

    @property
    def velocity(self) -> float:
        return self.ukf.x[1]

    def add_gps_position_velocity(self, gps_pos: LatLonDeg, gps_vel: float):
        """UKF update step: fuse GPS position and velocity measurement."""
        z = np.array(
            [
                self.get_distance_meters_per_axis(gps_pos, self.zero_lat_lng, self.ax),
                gps_vel,
            ]
        )
        self.ukf.update(z=z)

    def add_imu_acceleration(self, imu_acc: float, timestamp: float):
        """UKF predict step: propagate state forward using IMU acceleration."""
        self.dt = (timestamp - self.current_timestamp)
        if self.dt <= 0:
            return  # Guard against duplicate or out-of-order timestamps
        self.a = imu_acc
        self.ukf.predict(dt=self.dt)
        self.current_timestamp = timestamp
