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
# 4. Run the Python script with the provided CSV file and output to GPX
python3 test_ukf_py/sensor_fusion.py --input_csv "$input_csv" --output_csv "fused_output.csv"

python3 utils/csv_to_gpx.py --input_csv "fused_output.csv" --output_gpx "fused_output.gpx"