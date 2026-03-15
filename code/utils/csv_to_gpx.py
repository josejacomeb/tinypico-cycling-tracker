import pathlib
import argparse
import datetime
import pytz

import pandas as pd
import gpxpy

DATETIME_FORMAT = "%Y%m%d_%H%M%S"


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert CSV files with GPS data to GPX format."
        "Note that the time column need to be in seconds!"
    )
    parser.add_argument(
        "-i",
        "--input_csv",
        help="Path to the input CSV file.",
        type=pathlib.Path,
        required=True,
    )
    parser.add_argument(
        "-o",
        "--output_gpx",
        help="Path to the output GPX file.",
        type=pathlib.Path,
    )
    parser.add_argument(
        "-dt",
        "--date_time_gpx",
        help=f"Original datetime of the file in UTC {DATETIME_FORMAT} format.",
        type=str,
        default=None,
    )
    parser.add_argument(
        "-tz",
        "--timezone_gpx",
        help="Timezone of the original datetime",
        type=str,
        default="America/Guayaquil",
    )
    return parser


def main():
    args = build_argparser().parse_args()
    input_csv: pathlib.Path = args.input_csv
    print(f"Reading CSV file from: {input_csv}")
    output_gpx: pathlib.Path = args.output_gpx
    if output_gpx is None:
        output_gpx = pathlib.Path(input_csv.parent, f"{input_csv.stem}_output.gpx")

    df = pd.read_csv(input_csv)
    print(f"Number of rows in CSV: {len(df)}")
    df = df.dropna(subset=["lat", "lng"])
    print(f"Removed rows with missing lat/lng: new total {len(df)}")
    if "gps_update" in df.columns:
        df = df[df["gps_update"] == 1]
        print(f"Filtered to rows with GPS updates: new total {len(df)}")
    # Check for time that corresponds to an actual one
    tz = pytz.timezone(args.timezone_gpx)
    print(f"Using conversion timezone: {args.timezone_gpx}")
    if df.iloc[0]["time"] < 1e12:
        print("Data is not in timestamp format, converting...")
        if args.date_time_gpx is None:
            print("No datetime for the file specified, program terminating.")
            return -1
        start_dt = datetime.datetime.strptime(
            args.date_time_gpx, DATETIME_FORMAT
        ).replace(tzinfo=datetime.timezone.utc)
        # Sum the real time to the time offsets of the microcontroller
        df["time"] = df["time"].apply(
            lambda x: int(
                (start_dt + datetime.timedelta(seconds=int(x))).timestamp() * 1e3
            )
        )
    # Convert timestamp to datetime
    df["datetime"] = pd.to_datetime(df["time"], unit="ms", utc=True)
    df = df.set_index("datetime")

    # Resample to 1 second
    df = df.resample("1s").mean(numeric_only=True).dropna(subset=["lat", "lng"])

    print(f"Rows after 1-second resampling: {len(df)}")

    # Restore time column
    df["time"] = df.index.astype("int64") // 1_000_000
    gpx = gpxpy.gpx.GPX()
    # Create GPX track
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)
    # Create a new segment in our GPX track:
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    for ts, row in df.iterrows():
        point = gpxpy.gpx.GPXTrackPoint(
            latitude=round(row["lat"], 8),
            longitude=round(row["lng"], 8),
            elevation=round(row["alt"], 2) if "alt" in row else None,
            time=ts.astimezone(tz),
        )
        gpx_segment.points.append(point)

    with open(output_gpx, "w") as f:
        f.write(gpx.to_xml())
    print(f"GPX file saved to: {output_gpx}")


if __name__ == "__main__":
    main()
