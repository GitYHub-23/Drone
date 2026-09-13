# Development Log

## 6 September 2026

### Motor control reverse engineering

Today I identified and experimentally verified the motor-control GPIO pins of the original STM32-based drone flight controller.

I traced the motor driver circuits from the motor outputs through the MOSFET gate resistors to the STM32 microcontroller.

Then I wrote a minimal custom STM32 firmware and tested each suspected motor-control pin individually.

### Confirmed motor mapping

- PA8 -> M3
- PA9 -> M2
- PA10 -> M4
- PA11 -> M1

All four replacement brushed motors were tested individually with the propellers removed.

## 13 September 2026

### M540 IMU reverse engineering

The onboard IMU was identified as an M540 sensor.

After testing multiple GPIO combinations, the working I2C connection was found:

- SCL -> PB8
- SDA -> PB7
- I2C address -> 0x69

The sensor successfully responded to register reads.

WHO_AM_I register:
- Register: 0x75
- Returned value: 0x7D

Motion register testing showed that:

- 0x3B-0x40 changes with board tilt and movement, indicating accelerometer data.
- 0x43-0x48 changes with board rotation, indicating gyroscope data.

This confirms that the onboard M540 accelerometer and gyroscope can be accessed using custom STM32 firmware.
