import argparse
import pathlib
import json
import typing

import gpxpy
import pandas as pd
import numpy as np


def get_total_time_distance(moving_data: gpxpy.gpx.MovingData) -> dict:
    return {
        "moving_distance": moving_data.moving_distance,
        "moving_time": moving_data.moving_time,
        "stopped_distance": moving_data.stopped_distance,
        "stopped_time": moving_data.stopped_time,
        "total_distance": moving_data.moving_distance + moving_data.stopped_distance,
        "total_time": moving_data.moving_time + moving_data.stopped_time,
    }


def load_gpx_data(
    gpx_path, round: bool = False
) -> typing.Tuple[pd.DataFrame, gpxpy.gpx.MovingData]:
    with open(gpx_path, "r") as f:
        gpx = gpxpy.parse(f)
    moving_data = gpx.get_moving_data()
    moving_data.stopped_distance
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append({"time": p.time, "lat": p.latitude, "lng": p.longitude})

    df = pd.DataFrame(points)
    # Round location every second (Assumption: Speed is average of walking)
    df["time"] = pd.to_datetime(df["time"], utc=True).dt.round("s")
    df = df.set_index("time")
    return df[~df.index.duplicated(keep="last")], moving_data  # Drop duplicate times


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate the performance of GPX data processing."
    )
    parser.add_argument(
        "-i",
        "--input",
        help="Path to the GPX file to validate.",
        type=pathlib.Path,
        required=True,
    )
    parser.add_argument(
        "-g",
        "--ground_truth",
        help="Path to the reference GPX file for comparison.",
        type=pathlib.Path,
        required=True,
    )
    return parser


def main():
    args = build_argparser().parse_args()
    gpx_file: pathlib.Path = args.input
    gt_file: pathlib.Path = args.ground_truth

    df_gpx, ukf_moving_data = load_gpx_data(gpx_file)
    df_gt, ref_moving_data = load_gpx_data(gt_file)
    moving_data_gpx = get_total_time_distance(ukf_moving_data)
    moving_data_gt = get_total_time_distance(ref_moving_data)

    # --- 2. Synchronize (Resample & Interpolate) ---
    # Find overlapping time window
    start = max(df_gpx.index.min(), df_gt.index.min())
    end = min(df_gpx.index.max(), df_gt.index.max())

    # Create a common 1-second grid (modify '100L' for 100ms if you need higher precision)
    common_time = pd.date_range(start=start, end=end, freq="1s")
    # Interpolate both to the common grid
    ukf_sync = df_gpx.reindex(common_time).interpolate(method="time")
    ref_sync = df_gt.reindex(common_time).interpolate(method="time")

    # --- 3. Calculate RMSE (Degrees) ---
    lat_error_deg = ukf_sync["lat"] - ref_sync["lat"]
    lon_error_deg = ukf_sync["lng"] - ref_sync["lng"]

    rmse_lat_deg = np.sqrt((lat_error_deg**2).mean())
    rmse_lon_deg = np.sqrt((lon_error_deg**2).mean())

    # --- 4. Convert to Meters (Approximation) ---
    # 1 deg Lat ~= 111,320 meters
    # 1 deg Lon ~= 40075km * cos(lat) / 360
    mean_lat = ref_sync["lat"].mean()
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 40075000 * np.cos(np.radians(mean_lat)) / 360

    rmse_lat_m = rmse_lat_deg * meters_per_deg_lat
    rmse_lon_m = rmse_lon_deg * meters_per_deg_lon

    # Diff in the distance calculation
    distance_difference = (
        moving_data_gt["total_distance"] - moving_data_gpx["total_distance"]
    )

    # --- 5. Output Results ---
    print("=== RMSE VALIDATION ===")
    print(f"Latitude RMSE:  {rmse_lat_deg:.7f} deg  (~{rmse_lat_m:.3f} meters)")
    print(f"Longitude RMSE: {rmse_lon_deg:.7f} deg  (~{rmse_lon_m:.3f} meters)")
    print(
        f"Moving Distance GT: {moving_data_gt['total_distance']:.3f} meters,"
        f" GPX: {moving_data_gpx['total_distance']:.3f} meters, "
        f"Distance Difference: {distance_difference:.3f} meters"
    )
    print(f"GPX file: {gpx_file}")
    print(f"Reference file: {gt_file}")
    final_results = {
        "rmse_lat_deg": rmse_lat_deg,
        "rmse_lon_deg": rmse_lon_deg,
        "rmse_lat_m": rmse_lat_m,
        "rmse_lon_m": rmse_lon_m,
        "distance_difference": distance_difference,
        "gpx_file": str(gpx_file),
        "reference_file": str(gt_file),
        "total_gpx": moving_data_gpx,
        "total_gt": moving_data_gt,
    }
    output_path = gpx_file.parent / f"{gpx_file.stem}_{gt_file.stem}_results.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(final_results, f, indent=4)


if __name__ == "__main__":
    main()
