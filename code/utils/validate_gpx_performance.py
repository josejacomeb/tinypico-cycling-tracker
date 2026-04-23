import sys
import argparse
import pathlib
import json
import typing

import gpxpy
import numpy as np
import pandas as pd


# Maximum gap (seconds) allowed when interpolating between track points.
# Gaps larger than this are left as NaN rather than silently filled,
# preventing RMSE from being computed over long GPS outages.
MAX_INTERP_GAP_S = 10


def get_total_time_distance(moving_data: gpxpy.gpx.MovingData) -> dict:
    return {
        "moving_distance": float(moving_data.moving_distance),
        "moving_time": float(moving_data.moving_time),
        "stopped_distance": float(moving_data.stopped_distance),
        "stopped_time": float(moving_data.stopped_time),
        "total_distance": float(
            moving_data.moving_distance + moving_data.stopped_distance
        ),
        "total_time": float(moving_data.moving_time + moving_data.stopped_time),
    }


def load_gpx_data(
    gpx_path: pathlib.Path,
) -> typing.Tuple[pd.DataFrame, gpxpy.gpx.MovingData]:
    with open(gpx_path, "r") as f:
        gpx = gpxpy.parse(f)
    moving_data = gpx.get_moving_data()
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append(
                    {
                        "time": p.time,
                        "lat": p.latitude,
                        "lng": p.longitude,
                    }
                )
    df = pd.DataFrame(points)
    df["time"] = pd.to_datetime(df["time"], utc=True).dt.round("s")
    df = df.set_index("time")
    # Keep the last fix when two points share the same second
    return df[~df.index.duplicated(keep="last")], moving_data


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate GPX output against a ground-truth reference GPX."
    )
    parser.add_argument(
        "-i",
        "--input",
        help="Path to the GPX file to validate (UKF or RAW output).",
        type=pathlib.Path,
        required=True,
    )
    parser.add_argument(
        "-g",
        "--ground_truth",
        help="Path to the reference GPX file (e.g. Samsung recording).",
        type=pathlib.Path,
        required=True,
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    gpx_file: pathlib.Path = args.input
    gt_file: pathlib.Path = args.ground_truth

    df_gpx, ukf_moving_data = load_gpx_data(gpx_file)
    df_gt, ref_moving_data = load_gpx_data(gt_file)

    moving_data_gpx = get_total_time_distance(ukf_moving_data)
    moving_data_gt = get_total_time_distance(ref_moving_data)

    # --- Synchronise to a common 1-second grid ---
    start = max(df_gpx.index.min(), df_gt.index.min())
    end = min(df_gpx.index.max(), df_gt.index.max())

    if start >= end:
        print("ERROR: No overlapping time window between the two GPX files.")
        return 1

    common_time = pd.date_range(start=start, end=end, freq="1s")

    # gaps larger than MAX_INTERP_GAP_S are left as NaN and excluded
    # from the RMSE calculation.
    def interpolate_with_gap_limit(df: pd.DataFrame, max_gap_s: int) -> pd.DataFrame:
        df_reindexed = df.reindex(common_time)
        # Mark positions that are too far from any real observation
        is_real = df_reindexed.index.isin(df.index)
        df_interp = df_reindexed.interpolate(method="time")
        # Walk forward: if no real point within max_gap_s, set to NaN
        real_series = pd.Series(is_real, index=df_reindexed.index)
        # Build a "seconds since last real point" mask
        last_real = real_series[real_series].reindex(df_reindexed.index, method="ffill")
        gap_too_large = (
            df_reindexed.index
            - last_real.index.where(last_real.notna(), df_reindexed.index[0])
        ).total_seconds() > max_gap_s
        df_interp.loc[gap_too_large] = np.nan
        return df_interp

    ukf_sync = interpolate_with_gap_limit(df_gpx, MAX_INTERP_GAP_S)
    ref_sync = interpolate_with_gap_limit(df_gt, MAX_INTERP_GAP_S)

    # Drop rows where either track has no valid data
    valid_mask = ukf_sync["lat"].notna() & ref_sync["lat"].notna()
    ukf_sync = ukf_sync[valid_mask]
    ref_sync = ref_sync[valid_mask]
    n_valid = valid_mask.sum()

    if n_valid == 0:
        print("ERROR: No overlapping valid points after gap filtering.")
        return 1

    # --- RMSE in degrees ---
    lat_err_deg = ukf_sync["lat"] - ref_sync["lat"]
    lon_err_deg = ukf_sync["lng"] - ref_sync["lng"]
    rmse_lat_deg = float(np.sqrt((lat_err_deg**2).mean()))
    rmse_lon_deg = float(np.sqrt((lon_err_deg**2).mean()))

    # --- Convert to metres ---
    # Use the actual mean latitude of the reference track
    # for the longitude scale factor instead of a hardcoded constant.
    # meters_per_deg_lat ≈ 111,320 m/° is a reasonable global approximation
    # (error < 0.2 % at any latitude).
    mean_lat = float(ref_sync["lat"].mean())
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = 40_075_000.0 * np.cos(np.radians(mean_lat)) / 360.0

    rmse_lat_m = float(rmse_lat_deg * meters_per_deg_lat)
    rmse_lon_m = float(rmse_lon_deg * meters_per_deg_lon)

    # Combined positional RMSE (Euclidean in the local flat-earth approximation)
    rmse_2d_m = float(np.sqrt(rmse_lat_m**2 + rmse_lon_m**2))

    # Label the sign of the distance difference explicitly.
    # Positive → GPX under-estimates total distance vs ground truth.
    distance_difference = float(
        moving_data_gt["total_distance"] - moving_data_gpx["total_distance"]
    )

    # --- Print results ---
    print("=== RMSE VALIDATION ===")
    print(f"Valid comparison points : {n_valid} seconds")
    print(f"Latitude  RMSE : {rmse_lat_deg:.7f} °  (~{rmse_lat_m:.3f} m)")
    print(f"Longitude RMSE : {rmse_lon_deg:.7f} °  (~{rmse_lon_m:.3f} m)")
    print(f"2D position RMSE       : {rmse_2d_m:.3f} m")
    print()
    print(f"Total distance — GT  : {moving_data_gt['total_distance']:.3f} m")
    print(f"Total distance — GPX : {moving_data_gpx['total_distance']:.3f} m")
    if distance_difference >= 0:
        print(
            f"Distance difference    : +{distance_difference:.3f} m  (GPX under-estimates)"
        )
    else:
        print(
            f"Distance difference    : {distance_difference:.3f} m  (GPX over-estimates)"
        )
    print()
    print(f"Input file     : {gpx_file}")
    print(f"Reference file : {gt_file}")

    # --- Save JSON ---
    final_results = {
        "rmse_lat_deg": rmse_lat_deg,
        "rmse_lon_deg": rmse_lon_deg,
        "rmse_lat_m": rmse_lat_m,
        "rmse_lon_m": rmse_lon_m,
        "rmse_2d_m": rmse_2d_m,
        "n_valid_points": int(n_valid),
        "distance_difference": distance_difference,
        "distance_difference_sign": (
            "gpx_underestimates" if distance_difference >= 0 else "gpx_overestimates"
        ),
        "gpx_file": str(gpx_file),
        "reference_file": str(gt_file),
        "total_gpx": moving_data_gpx,
        "total_gt": moving_data_gt,
    }
    output_path = gpx_file.parent / f"{gpx_file.stem}_vs_{gt_file.stem}_results.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(final_results, f, indent=4)
    print(f"Results saved to: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
