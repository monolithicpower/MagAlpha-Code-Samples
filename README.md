# MagAlpha-Code-Samples
This repository contains code samples for the MPS MagAlpha position sensor family.

## License
All code samples are released under the MIT License.

# Supported Hardware
We currently have code examples for the following hardware platforms:
* [Arduino](https://www.arduino.cc/) (works best with the following 3.3V boards)
    * [Arduino ZERO](https://www.arduino.cc/en/Main/ArduinoBoardZero) (use SPI connector and USB programming port)
    * [Arduino MKRZero](https://www.arduino.cc/en/Main/ArduinoBoardMKRZero) (not tested)
    * [Adafruit Feather M0 Basic Proto](https://www.adafruit.com/products/2772) (should also works with other feather M0 boards)
    * [Arduino 101](https://www.arduino.cc/en/Main/ArduinoBoard101) (not tested)
    * [Arduino Due](https://www.arduino.cc/en/Main/ArduinoBoardDue) (use SPI connector and USB programming port)

* [Adafruit FT232H Breakout](https://www.adafruit.com/products/2264) (use Python)

| Information |
| ------- |
| Don't forget to check out the [MagAlpha Angle Sensor Arduino Library](https://github.com/monolithicpower/MagAlpha-Arduino-Library)|

# About MagAlpha Sensor Familiy
MagAlpha sensor family is based on Hall devices that are directly integrated with the signal treatment. These sensors are extremely compact and can instantaneously detects and delivers the angle value in digital format.

More information can be found on the [MPS website](https://www.monolithicpower.com/Products/Position-Sensors/Products-Overview) or directly on the [Position Sensors Design Support ](https://www.monolithicpower.com/Design-Support/Position-Sensors-Design-Support) page.
## Applications
### Potentiometer/Encoder Alternative
The MagAlpha sensor family offer a robust contactless angle encoder suitable for control buttons and knobs. The IC detects the absolute angular position of a permanent magnet, typically a diametrically magnetized cylinder attached to the rotor. The PWM digital output can be filtered to obtain an analog signal.

### Motor Commutation
A MagAlpha sensor can replace a 3 Hall switch solution (UVW) or an incremental optical encoder (ABZ, ABI) for a fraction of the initial cost. Making it a perfect fit for Brushless DC motors (BLDC). Thanks to its extremely fast acquisition time, the MagAlpha can be used at speeds from 0 to 120'000 RPM.

### Angle Measurement
The MagAlpha digital interface can be connected to any microcontroller having a SPI interface. It is extremely easy to monitor the angular position and configure sensor parameters such as the zero settings and the bias current trimming (BCT)
