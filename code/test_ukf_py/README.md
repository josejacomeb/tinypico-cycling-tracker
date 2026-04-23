# Sensor Fusion in Python

Implementation of the Unscented Kalman Filter(UKF) with accessible sensors such as `uBlox-Neo6M` and `MPU6050`, the scripts expect a `.csv`, consider to use the [data-logger](../data_logger/) to get the formatted data.

## Installation

1. Create a new python venv with:

    ```bash
    python -m venv .venv/
    ```

2. Source your new venv, for example in Linux

    ```bash
    source .venv/bin/activate
    ```

3. Install the prerequisites with

    ```bash
    pip install -r requirements.txt
    ```

## Usage

```bash
python3 sensor_fusion.py --help
usage: sensor_fusion.py [-h] -i INPUT_CSV -o OUTPUT_CSV

Test UKF GPS+IMU sensor fusion with CSV data.

options:
  -h, --help            show this help message and exit
  -i INPUT_CSV, --input_csv INPUT_CSV
                        Path to the input CSV file with GPS and IMU data.
  -o OUTPUT_CSV, --output_csv OUTPUT_CSV
                        Path to the Fused output CSV file.
```

## Structure

```bash
├── constants.py
├── data_types.py
├── ukf_gps_imu_graphics.ipynb
├── README.md
├── requirements.txt
├── sensor_fusion.py
├── ukf_gps_imu.py
└── utils.py
```

## Files

- `constants.py`: UKF, Accelerometer and GPS constants
- `data_types.py`: Data types used in the project
- `ukf_gps_imu.py`: Implementation of the sensor fusion GPS + IMU with UKF with filterpy
- `sensor_fusion.py`: Simulation of a real implementation that should be in the microcontroller
- `ukf_gps_imu_graphics.ipynb`: Data visualisation for comparison
