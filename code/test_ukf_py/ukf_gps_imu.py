from dataclasses import dataclass

from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import numpy as np
from constants import X_LEN, Z_LEN, U_LEN, Rv_INIT, DEG_2_RAD, EARTH_RADIUS, LBM_2_M_S2
from data_types import LatLonDeg, GPSAxis
import math


def fx_axis(x, dt, a):
    p, v = x
    p_new = p + v * dt + 0.5 * a * dt**2
    v_new = v + a * dt
    return np.array([p_new, v_new])


def hx_axis(x):
    # GPS measures position and velocity
    return np.array([x[0], x[1]])


@dataclass
class UKFGPSIMU:
    ax: GPSAxis  # Axis
    posstd: float
    accstd: float
    dt: float = 0.2  # sample period (s)

    def __post_init__(self):
        self.X_LEN = X_LEN
        self.Z_LEN = Z_LEN
        self.U_LEN = U_LEN
        self.zero_lat_lng = LatLonDeg()  # Defaults to 0.0, 0.0

        self.points = MerweScaledSigmaPoints(
            n=self.X_LEN, alpha=0.1, beta=2.0, kappa=1.0
        )
        # Create Kalman Filter
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.X_LEN,
            dim_z=self.Z_LEN,
            fx=lambda x, dt: fx_axis(x, dt, self.a),
            hx=hx_axis,
            dt=self.dt,
            points=self.points,
        )
        # State vector [p, v]
        self.ukf.x = np.zeros((self.X_LEN))

        # Process noise covariance (IMU uncertainty)
        self.ukf.Q = np.diag(
            [
                self.accstd * self.accstd,
                self.accstd * self.accstd,
            ]
        )
        # Measurement noise covariance (GPS)
        self.ukf.R = np.diag(
            [
                self.posstd * self.posstd,
                self.posstd * self.posstd,
            ]
        )
        # Initial covariance
        self.ukf.P = np.eye(self.X_LEN) * 1.0
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
        Calculates distance isolating a specific axis based on the C++ logic provided.
        Note: Based on your snippet, choosing LONGITUDE zeros out the longitude
        difference, effectively calculating the Latitude distance (and vice versa).
        """
        to_lng, to_lat = to_pos.lon, to_pos.lat
        from_lng, from_lat = from_pos.lon, from_pos.lat

        # Logic mirrored from C++ snippet:
        # If axis is LONGITUDE, it zeros out the longitude variables.
        if axis == GPSAxis.LONGITUDE:
            to_lng = 0.0
            from_lng = 0.0
        else:
            to_lat = 0.0
            from_lat = 0.0

        delta_lon = (to_lng - from_lng) * DEG_2_RAD
        delta_lat = (to_lat - from_lat) * DEG_2_RAD

        # Haversine calculation
        a = (
            math.sin(delta_lat / 2.0) ** 2
            + math.cos(from_lat * DEG_2_RAD)
            * math.cos(to_lat * DEG_2_RAD)
            * math.sin(delta_lon / 2.0) ** 2
        )

        distance = 2 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a)) * EARTH_RADIUS
        return distance

    @property
    def position(self):
        return self.ukf.x[0]

    @property
    def velocity(self):
        return self.ukf.x[1]

    def add_gps_position_velocity(self, gps_pos: LatLonDeg, gps_vel: float):
        # Measurement (GPS positions)
        z = np.array(
            [
                self.get_distance_meters_per_axis(gps_pos, self.zero_lat_lng, self.ax),
                gps_vel,
            ]
        )
        self.ukf.update(z)

    def add_imu_acceleration(self, imu_acc: float, timestamp: float):
        delta_t = timestamp - self.current_timestamp
        self.a = imu_acc
        self.ukf.dt = delta_t
        self.ukf.predict()
        self.current_timestamp = timestamp
