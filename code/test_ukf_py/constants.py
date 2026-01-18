import math

dt = 0.2  # sample period (s)
X_LEN = 2
Z_LEN = 2
U_LEN = 3
P_INIT = 10
ACTUAL_GRAVITY = 9.771927346 # Ecuador
# Constants
EARTH_RADIUS = 6371 * 1000.0  # Meters
DEG_2_RAD = math.pi / 180.0
LBM_2_M_S2 = 16384  # Test - 2

Rv_INIT = ACTUAL_GRAVITY * 1e-3  # 1e-6 # measurement noise
Rn_INIT_ACC = 0.0015  # acceleration noise