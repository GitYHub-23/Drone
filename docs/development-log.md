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
