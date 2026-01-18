#!/bin/bash
# Program to fuse data from GPS and IMU sensors and output to GPX
# 1. Assign the first argument to the variable INPUT_CSV
INPUT_CSV="$1"
GT_GPX="$2"

# 2. Check if the parameter is missing (length is zero)
if [[ -z "$INPUT_CSV" ]]; then
    echo "Error: Missing required parameter 'INPUT_CSV'."
    echo "Usage: $0 path/to/file.csv"
    exit 1
fi

# 3. Optional: Check if the file actually exists
if [[ ! -f "$INPUT_CSV" ]]; then
    echo "Error: File '$INPUT_CSV' not found."
    exit 1
fi

source .venv/bin/activate

# 4. Extract the base name from INPUT_CSV without extension for output naming
input_basename=$(basename "$INPUT_CSV" .csv)

# 5. Apply sensor fusion using the UKF implementation
OUTPUT_UKF_CSV="${input_basename}_output_ukf.csv"
OUTPUT_UKF_GPX="${input_basename}_output_ukf.gpx"
OUTPUT_RAW_DATA_GPX="${input_basename}.gpx"

python3 test_ukf_py/sensor_fusion.py --input_csv "$INPUT_CSV" --output_csv "$OUTPUT_UKF_CSV"
# 6. Convert the output CSV to GPX format
python3 utils/csv_to_gpx.py --input_csv "$OUTPUT_UKF_CSV" --output_gpx "$OUTPUT_UKF_GPX" --date_time_gpx ${input_basename}
# 6.1 Convert original data sensor data to GPX format for comparison
python3 utils/csv_to_gpx.py --input_csv "$INPUT_CSV" --output_gpx "$OUTPUT_RAW_DATA_GPX" --date_time_gpx ${input_basename}


# 7. Validate results if a GT is provided
if [[ -n "$GT_GPX" ]]; then
    echo "Validating the data with ground truth: $GT_GPX"
    python3 utils/validate_gpx_performance.py --input "$OUTPUT_UKF_GPX" --ground_truth "$GT_GPX"
    python3 utils/validate_gpx_performance.py --input "$OUTPUT_RAW_DATA_GPX" --ground_truth "$GT_GPX"
fi
