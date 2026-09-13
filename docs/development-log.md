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

The onboard M540 motion sensor was successfully identified and accessed.

Confirmed interface:
- Interface: I2C
- SCL: PB8
- SDA: PB7
- I2C address: 0x69

Register tests:
- Register 0x75 returned 0x7D.
- Registers 0x3B-0x40 responded to board tilt and movement.
- Registers 0x43-0x48 responded to board rotation.

This strongly indicates that the first block contains accelerometer data
and the second block contains gyroscope data.

The IMU can now be accessed directly by custom STM32 firmware.
