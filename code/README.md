# Usage

## Data Logger

Flash your device with the code to gather GPS + IMU Data

## Sensor Fusion

The purpose of the following programs are to understand, code and validate the sensor fusion implementation. The final result of the conversion should be a GPX file.

### Python

#### Installation

Please use the following commands

```bash
python -m venv .venv
pip install -r utils/requirements.txt
source .venv/bin/activate
```

#### Execution

Please use the helper script to accomplish this task

```bash
./test_sensor_fusion.sh path/to/your/sensor_data.csv
```
