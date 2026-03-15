import math

dt = 0.1  # sample period (s) — matches SS_DT_MILIS = 100 in konfig.h

X_LEN = 2
Z_LEN = 2
U_LEN = 3

# Value matches constants.hpp: uBlox 6M horizontal accuracy spec (meters).
P_INIT = 2.5

ACTUAL_GRAVITY = 9.771927346  # m/s² — Ecuador at ~2,729 m altitude

# Constants
EARTH_RADIUS = 6371 * 1000.0  # Meters

# This is LSB per g at ±2g full scale # (MPU6050 datasheet).
# Dividing raw counts by LSB_PER_G gives acceleration in g.
# Multiply by ACTUAL_GRAVITY to get m/s². See sensor_fusion.py usage.
LSB_PER_G = 16384

# ACC_STD: standard deviation of IMU accelerometer noise (m/s²).
ACC_STD_NORTH = 0.0324  # m/s²
ACC_STD_EAST = 0.0453  # m/s²

# ACC_POS: GPS horizontal position accuracy (m), used for UKF measurement noise R.
POS_STD = 2  # meters (uBlox 6M spec)

# GPS velocity noise standard deviation (m/s).
# Kept separate from ACC_POS — GPS speed accuracy is ~0.1 m/s, not 2.5 m.
GPS_VEL_STD = 0.1  # m/s (uBlox 6M typical speed accuracy)
