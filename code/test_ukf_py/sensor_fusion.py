"""
sensor_fusion.py — GPS + IMU UKF sensor fusion

Data flow
---------
The input CSV is logged at 10 Hz (100 ms per row) by the TinyPico Cycling Tracker (TCT) on an SD.
Every row has valid IMU acceleration (accNorth, accEast in m/s²).
GPS columns (lat, lng, vNorth, vEast, alt) are NaN on rows where the
GPS receiver did not produce a new fix (~1 out of every 10 rows).

UKF loop per row
----------------
1. PREDICT  — always run, using IMU acceleration and the actual elapsed time
              (delta_t from the timestamp column, not a fixed dt assumption).
2. UPDATE   — run only when the row contains a valid GPS fix (non-NaN lat/lng).
3. OUTPUT   — emit a fused (lat, lng, vNorth, vEast) row on *every* step,
              so the output has the same 10 Hz density as the input and the
              IMU-driven dead-reckoning between GPS fixes is captured.
"""

import pathlib
import argparse

import pandas as pd

from utils import get_point_ahead
from data_types import LatLonDeg, GPSAxis
from ukf_gps_imu import UKFGPSIMU
from constants import (
    dt,
    POS_STD,
    GPS_VEL_STD_NORTH,
    GPS_VEL_STD_EAST,
    ACC_STD_NORTH,
    ACC_STD_EAST,
)


def _find_first_gps_row(df: pd.DataFrame) -> pd.Series:
    """Return the first row with a valid GPS fix, used to seed the UKF state."""
    for _, row in df.iterrows():
        if row["gpsUpdate"]:
            return row
    raise ValueError("No valid GPS fix found in the input CSV. Cannot initialise UKF.")


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "UKF GPS+IMU sensor fusion. "
            "Predicts at 10 Hz (IMU rate), updates on GPS fixes (~1 Hz). "
            "Outputs a fused position and velocity estimate on every row."
        )
    )
    parser.add_argument(
        "-i",
        "--input_csv",
        help="Input CSV logged by the TCT at 10 Hz.",
        type=pathlib.Path,
        required=True,
    )
    parser.add_argument(
        "-o",
        "--output_csv",
        help="Path to save the UKF result in CSV.",
        type=pathlib.Path,
        required=True,
    )
    return parser


def main():
    args = build_argparser().parse_args()
    df = pd.read_csv(args.input_csv)

    # ---- Seed state from the first valid GPS fix ----
    first_gps = _find_first_gps_row(df)
    origin = LatLonDeg(lat=first_gps["lat"], lon=first_gps["lng"])

    # ---- Create one UKF per axis ----
    # The Standard Devation error was calculated with the Device in a static position
    ukf_east = UKFGPSIMU(GPSAxis.LONGITUDE, POS_STD, GPS_VEL_STD_EAST, ACC_STD_EAST, dt)
    ukf_north = UKFGPSIMU(
        GPSAxis.LATITUDE, POS_STD, GPS_VEL_STD_NORTH, ACC_STD_NORTH, dt
    )

    ukf_east.init_position_velocity(origin, first_gps["vEast"], first_gps["time"])
    ukf_north.init_position_velocity(origin, first_gps["vNorth"], first_gps["time"])

    # All UKF distances are in metres relative to this origin.
    zero_point = LatLonDeg()

    results = []
    last_time = 0
    for _, row in df.iterrows():
        time = row["time"]
        gps_update = False
        # Ignore rows logged before the UKF was initialised
        if time < first_gps["time"]:
            continue

        # Predict step
        ukf_east.add_imu_acceleration(row["accEast"], time)
        ukf_north.add_imu_acceleration(row["accNorth"], time)
        # Avoid double GPS detections
        if row["gpsUpdate"] and (time - last_time > 0.5):
            # Update Step
            gps_pos = LatLonDeg(lat=row["lat"], lon=row["lng"])
            ukf_east.add_gps_position_velocity(gps_pos, row["vEast"])
            ukf_north.add_gps_position_velocity(gps_pos, row["vNorth"])
            last_time = time
            gps_update = True

        point_n = get_point_ahead(zero_point, ukf_east.position, 90.0)
        point_ne = get_point_ahead(point_n, ukf_north.position, 0.0)

        results.append(
            {
                "time": time,
                "lat": point_ne.lat,
                "lng": point_ne.lon,
                "alt": row["alt"] if "alt" in row and gps_update else None,
                "vNorth": ukf_north.velocity,
                "vEast": ukf_east.velocity,
                "gpsUpdate": gps_update,  # True on rows where GPS was fused
            }
        )

    df_out = pd.DataFrame(results)
    df_out.to_csv(args.output_csv, index=False)

    total = len(df_out)
    gps_rows = int(df_out["gpsUpdate"].sum())
    print(
        f"Wrote {total} rows → {args.output_csv}\n"
        f"  IMU predict steps : {total - gps_rows}  (~{(total - gps_rows) * dt:.1f} s)\n"
        f"  GPS update steps  : {gps_rows}"
    )


if __name__ == "__main__":
    main()
