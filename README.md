# tinypico-cycling-tracker

Tinypico based GPS + IMU + SD data logger for cycling purposes
![TinyPico Cycling Tracker](/assets/boards.png "TinyPICO Cycling Tracker")

## Components

### Software

- [Kicad v9.x.x](https://www.kicad.org/)
- [KiKit v1.7.x](https://yaqwsx.github.io/KiKit/latest/)
- [Arduino IDE v2.3.x](https://www.arduino.cc/en/software/)

### Hardware

- [TinyPICO](https://www.tinypico.com/): An ESP32 Dico D4 based development platform
- [uBlox-Neo6M](https://www.u-blox.com/en/product/neo-6-series): A versatile GPS Module with Antenna
- [Display OLED I2C 0.96" SSD1306](https://soldered.com/product/display-oled-i2c-white-0-96-ssd1306/): An 128x64px 0.96" OLED Display with I2C Communication
- [MPU6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/): An integrated gyroscope and accelerometer with I2C communication
- [MicroSD Card Adapter](https://hobbycomponents.com/adapters/578-microsd-card-adapter-with-level-shifters): A SPI module to interface microcontrollers with a microSD card.

### Communication Protocols

- SPI: SD Card Reader -> TinyPICO
- I2C: OLED Screen & MPU6050 -> TinyPICO
- UART: GPS -> TinyPICO

## Instructions

### For KiCad

1. Add the symbols via the `Preferences` > `Manage symbol Libraries` > `Project Specific Libraries`, make sure that you add the `kicad-libraries` subfolders:

    1. `usini_kicad_sensors/usini_sensors.pretty`
    2. `uBlox-Neo6M/uBLox-NEO-6M.pretty`
    3. `MicroSD-Card-Adapter/MicroSD-Card-Adapter.kicad_sym`
    4. `Soldered-SSD1306-OLED-Display/Soldered.kicad_sym`
    5. `TinyPICO/TinyPICO.kicad_sym`

2. Add the footprints via the `Preferences` > `Manage Footprint Libraries` > `Project Specific Libraries`, make sure that you add the `footprint` subfolders
    1. `usini_kicad_sensors/usini_sensors.kicad_sym`
    2. `uBlox-Neo6M/uBlox-NEO-6M.kicad_sym`
    3. `MicroSD-Card-Adapter/MicroSD-Card-Adapter.kicad_sym`
    4. `Soldered-SSD1306-OLED-Display/Soldered.pretty`
    5. `TinyPICO/TinyPICO.pretty`

3. Install the `KiKit` in your local system

    ```bash
    python -m venv .venv/
    source .venv/bin/activate
    pip3 install kikit
    ```

### For Arduino

1. Please follow the Espressif [instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) to support the `ESP32` boards on your Arduino.
2. Please install the following libraries in your Arduino IDE in your Tools > Manage Libraries section:
    1. [TinyPICO Helper Library](https://docs.arduino.cc/libraries/tinypico-helper-library/)
    2. [Adafruit SSD1306](https://docs.arduino.cc/libraries/adafruit-ssd1306/)
    3. [TinyGPSPlus](https://docs.arduino.cc/libraries/adafruit-ssd1306/)
3. Select the `UM TinyPICO` board in the Arduino IDE Tools > Board > ESP32
4. Select the Communication Port to `/dev/ttyACM0` if you're running on Linux.
5. Open the folder [code/arduino-tct/](code/arduino-tct/) with the IDE to flash the code to your TinyPICO Board.

## Modes

The following screens were converted to `.png`  and later transformed to a compatible data with the help of the Adafruit [make_splash.py](https://github.com/adafruit/Adafruit_SSD1306/blob/master/scripts/make_splash.py) script.

### Loading

![Loading Screen](/assets/screens/1.%20loading_components.svg "Loading Screen")

This screen show the loading in realtime of the three peripherals.

### Waiting

![Waiting Screen](/assets/screens/2.%20wait.svg "Waiting Screen")

This is the normal screen you'll see before pushing the start button.

### Recording workout

![Recording Screen](/assets/screens/3.%20workout.svg "Recording Screen")

After pushing the start button, you will see the elevation, pace, slope, altitude and workout time in the screen.
