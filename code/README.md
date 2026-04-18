# Usage

## Cycling Tracker

The TinyPico Ciclying Tracker source code is defined in the [arduino-tct](arduino-tct/) folder

## Data Logger

Flash your device with the code to gather GPS + IMU Data, ready to be used by the Sensor Fusion models.

## Sensor Fusion Implementation

The purpose of the `test_ukf*` are to understand, code and validate the sensor fusion implementation. The final result of the conversion should be a GPX file.

### Python

#### Installation

To install the Python Libraries, please use the use the following commands

    ```bash
    python -m venv .venv
    source .venv/bin/activate
    pip install -r utils/requirements.txt
    ```

#### Sensor Fusion Usage

    ```bash
    usage: sensor_fusion.py [-h] -i INPUT_CSV -o OUTPUT_CSV

    UKF GPS+IMU sensor fusion. Predicts at 10 Hz (IMU rate), updates on GPS fixes (~1 Hz). Outputs a fused position and velocity estimate on every row.

    options:
    -h, --help            show this help message and exit
    -i INPUT_CSV, --input_csv INPUT_CSV
                            Input CSV logged by the TCT at 10 Hz.
    -o OUTPUT_CSV, --output_csv OUTPUT_CSV
                            Path to save the UKF result in CSV.
    ```

## Utils

The `utils`folder contain two useful scripts for the development for this project:

1. `csv_to_gpx`: Converts the data logger generated data to a GPX representation that is equivalent with the output of other devices, such as Garmin or Samsung Health
2. `validate_gpx_performance`: Comparte two GPX files that were generated in the same time and outputs the RMS output and the distance

## Helper Script

To use the python scripts in one command, please use the helper script, it will convert and compare your output `gpx file`

    ```bash
    ./test_sensor_fusion.sh path/to/your/sensor_data.csv path/to/your/gt.gpx
    ```
