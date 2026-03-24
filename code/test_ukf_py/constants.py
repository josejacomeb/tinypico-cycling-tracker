dt = 0.1  # sample period (s) — matches SS_DT_MILIS = 100 in konfig.h

X_LEN = 2
Z_LEN = 2
U_LEN = 3

ACTUAL_GRAVITY = 9.771927346  # m/s² — Ecuador at ~2,729 m altitude

# Constants
EARTH_RADIUS = 6371 * 1000.0  # Meters

# You can get these values by running the data logger in a static position
# and calculating the standard deviation of the logged acceleration and GPS velocity values.
# ACC_STD: standard deviation of IMU accelerometer noise (m/s²).
ACC_STD_NORTH = 0.0559  # m/s²
ACC_STD_EAST = 0.0705  # m/s²

# ACC_POS: GPS horizontal position accuracy (m), used for UKF measurement noise R.
# P_INIT in the constants.hpp
POS_STD = 2  # meters (uBlox 6M spec)

# GPS velocity noise standard deviation (m/s).
GPS_VEL_STD_NORTH = 0.0641
GPS_VEL_STD_EAST = 0.0938
