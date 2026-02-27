import pathlib
from utils import get_point_ahead
from data_types import LatLonDeg, GPSAxis
from ukf_gps_imu import UKFGPSIMU
from constants import dt, ACTUAL_GRAVITY, LBM_2_M_S2
import pandas as pd
import argparse


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test UKF GPS+IMU sensor fusion with CSV data."
    )
    parser.add_argument(
        "-i",
        "--input_csv",
        help="Path to the input CSV file with GPS and IMU data.",
        type=pathlib.Path,
        required=True,
    )
    parser.add_argument(
        "-o",
        "--output_csv",
        help="Path to the Fused output CSV file.",
        type=pathlib.Path,
        required=True,
    )
    return parser


def main():
    args = build_argparser().parse_args()
    df = pd.read_csv(args.input_csv)
    ACC_STD = 50e-3 * ACTUAL_GRAVITY
    ACC_POS = 2.5
    ukf_sf_lat = UKFGPSIMU(GPSAxis.LATITUDE, ACC_POS, ACC_STD, dt)
    ukf_sf_lon = UKFGPSIMU(GPSAxis.LONGITUDE, ACC_POS, ACC_STD, dt)
    ukf_results = []
    GPSInitialLatLon = LatLonDeg(lat=df["lat"].iloc[0], lon=df["lng"].iloc[0])
    v_north_initial = df["vNorth"].iloc[0]
    v_east_initial = df["vEast"].iloc[0]
    timestamp_initial = df["time"].iloc[0]
    # UKF Initialization
    ukf_sf_lat.init_position_velocity(
        GPSInitialLatLon, v_north_initial, timestamp_initial
    )
    ukf_sf_lon.init_position_velocity(
        GPSInitialLatLon, v_east_initial, timestamp_initial
    )

    ZeroPoint = LatLonDeg()

    for _, row in df.iterrows():
        # Predict step
        acc_north = row["accNorth"] / LBM_2_M_S2
        acc_east = row["accEast"] / LBM_2_M_S2
        time = row["time"]
        ukf_sf_lat.add_imu_acceleration(acc_north, time)
        ukf_sf_lon.add_imu_acceleration(acc_east, time)
        if not row["lat"] == 0 and not row["lng"] == 0:
            GPSLatLon = LatLonDeg(lat=row["lat"], lon=row["lng"])
            # UKF
            ukf_sf_lat.add_gps_position_velocity(GPSLatLon, row["vNorth"])
            ukf_sf_lon.add_gps_position_velocity(GPSLatLon, row["vEast"])
            PointEastUKF = get_point_ahead(ZeroPoint, ukf_sf_lat.position, 270.0)
            PointNorthEastUKF = get_point_ahead(PointEastUKF, ukf_sf_lon.position, 180)
            ukf_results.append(
                {
                    "time": row["time"],
                    "lat": PointNorthEastUKF.lat,
                    "lng": PointNorthEastUKF.lon,
                    "vNorth": ukf_sf_lat.velocity,
                    "vEast": ukf_sf_lon.velocity,
                }
            )
    df_ukf = pd.DataFrame(ukf_results)
    df_ukf.to_csv(args.output_csv, index=False)


if __name__ == "__main__":
    main()
