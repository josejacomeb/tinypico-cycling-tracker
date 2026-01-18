#!/bin/bash
# Program to fuse data from GPS and IMU sensors and output to GPX
# 1. Assign the first argument to the variable input_csv
input_csv="$1"

# 2. Check if the parameter is missing (length is zero)
if [[ -z "$input_csv" ]]; then
    echo "Error: Missing required parameter 'input_csv'."
    echo "Usage: $0 path/to/file.csv"
    exit 1
fi

# 3. Optional: Check if the file actually exists
if [[ ! -f "$input_csv" ]]; then
    echo "Error: File '$input_csv' not found."
    exit 1
fi

source .venv/bin/activate

# 4. Extract the base name from input_csv without extension for output naming
input_basename=$(basename "$input_csv" .csv)

# 5. Apply sensor fusion using the UKF implementation
python3 test_ukf_py/sensor_fusion.py --input_csv "$input_csv" --output_csv "${input_basename}_output.csv"
# 6. Convert the output CSV to GPX format
python3 utils/csv_to_gpx.py --input_csv "${input_basename}_output.csv" --output_gpx "${input_basename}_output.gpx" --date_time_gpx ${input_basename}
# 6.1 Convert original data sensor data to GPX format for comparison
python3 utils/csv_to_gpx.py --input_csv "$input_csv" --output_gpx "${input_basename}.gpx" --date_time_gpx ${input_basename}
