import pathlib
import argparse

import pandas as pd
import gpxpy


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert CSV files with GPS data to GPX format."
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
    return parser


def main():
    args = build_argparser().parse_args()
    input_csv: pathlib.Path = args.input_csv
    output_gpx: pathlib.Path = args.output_gpx
    if output_gpx is None:
        output_gpx = pathlib.Path(input_csv.parent, f"{input_csv.stem}_output.gpx")

    df = pd.read_csv(input_csv)
    df = df.dropna(subset=["lat", "lng"])
    gpx = gpxpy.gpx.GPX()
    # Create GPX track
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)
    # Create a new segment in our GPX track:
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    for _, row in df.iterrows():
        point = gpxpy.gpx.GPXTrackPoint(
            latitude=row["lat"],
            longitude=row["lng"],
            elevation=None,  # TODO: Calculate elevation later
        )
        gpx_segment.points.append(point)

    with open(output_gpx, "w") as f:
        f.write(gpx.to_xml())


if __name__ == "__main__":
    main()
