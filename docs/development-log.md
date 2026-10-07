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

- Radio CE confirmed: PB6.
- Complete radio interface:
  - CSN: PA15
  - SCK: PB3
  - MISO: PB4
  - MOSI: PB5
  - CE: PB6

## 3 October 2026

Successfully brought up the unknown onboard M540 IMU using the
STM32F031K4.

### Completed

- Identified IMU communication pins:
  - PB7 → SDA
  - PB8 → SCL
- Identified I2C address: `0x69`
- Confirmed `WHO_AM_I = 0x7D`
- Implemented software I2C from scratch
- Implemented multi-byte IMU register reading
- Successfully acquired accelerometer and gyroscope raw data
- Configured accelerometer for ±16 g
- Configured gyroscope for ±2000 deg/s
- Experimentally determined accelerometer orientation
- Experimentally determined gyroscope axis mapping

### Confirmed gyroscope mapping

| Raw Axis | Drone Rotation |
|----------|----------------|
| Gyro X | Roll |
| Gyro Y | Pitch |
| Gyro Z | Yaw |

### Confirmed accelerometer orientation

| Test Position | Result |
|---------------|--------|
| Board level | Z ≈ +1 g |
| Right side down | Y ≈ +1 g |
| Nose down | X ≈ -1 g |

The IMU interface is now considered operational.

# M540 IMU Bring-Up and Axis Mapping

## Overview

The original flight controller of this toy quadcopter is based on an
STM32F031K4 microcontroller.

Because the goal of this project is to write custom flight-controller
firmware from scratch, the onboard inertial measurement unit (IMU) also
had to be reverse engineered.

The chip is marked:

M540
A615E1
L1346

No dedicated driver or vendor library was used.

The sensor was brought up by directly communicating with it from the
STM32 using a custom software I2C implementation.


## Hardware Connection

The following connections were identified on the original drone PCB:

| Signal | STM32 Pin |
|--------|-----------|
| SDA | PB7 |
| SCL | PB8 |

The detected I2C address is:

0x69


## Device Identification

The sensor responds to the WHO_AM_I register at:

0x75

The returned value is:

0x7D

This confirmed reliable communication between the STM32F031 and the
onboard IMU.


## Software I2C

For the initial bring-up, I implemented I2C directly in firmware instead
of using an external library.

The SDA and SCL lines are controlled using GPIO.

Open-drain behavior is emulated by:

- driving the GPIO LOW for logic 0
- releasing the GPIO as an input for logic 1

The implementation supports:

- START condition
- STOP condition
- byte transmission
- ACK detection
- byte reception
- repeated START
- multi-register reads


## IMU Initialization

The following initialization sequence was successfully tested:

| Register | Value | Purpose |
|----------|-------|---------|
| 0x6B | 0x80 | Device reset |
| 0x6B | 0x01 | Wake / clock selection |
| 0x1C | 0x18 | Accelerometer ±16 g |
| 0x1D | 0x05 | Accelerometer filter configuration |
| 0x1B | 0x18 | Gyroscope ±2000 deg/s |
| 0x1A | 0x00 | Gyroscope filter configuration |

After initialization, 14 consecutive bytes are read starting from
register 0x3B.

The data frame contains:

Accel X
Accel Y
Accel Z
Temperature
Gyro X
Gyro Y
Gyro Z


## Accelerometer Test

The accelerometer was tested by physically rotating the flight
controller board and observing the raw measurements.

With the ±16 g configuration, approximately 2048 raw counts correspond
to 1 g.

### Board horizontal

Measured approximately:

Accel X = 38
Accel Y = 18
Accel Z = 2008

Therefore:

Raw Z ≈ +1 g


### Right side tilted down by approximately 90 degrees

Measured approximately:

Accel X = 53
Accel Y = 2046
Accel Z = 62

Therefore:

Raw Y ≈ +1 g


### Nose tilted down by approximately 90 degrees

Measured approximately:

Accel X = -2029
Accel Y = 2
Accel Z = -45

Therefore:

Raw X ≈ -1 g


## Gyroscope Axis Mapping

To identify the gyroscope axes, the firmware was modified to continuously
record the minimum and maximum raw values for all three gyro channels.

Three separate physical rotation tests were performed.


### Roll test

Recorded peaks:

| Axis | Minimum | Maximum |
|------|--------:|--------:|
| Gyro X | -8100 | +7219 |
| Gyro Y | -3136 | +3702 |
| Gyro Z | -1651 | +3095 |

Gyro X showed the dominant response.

Therefore:

Gyro X = ROLL


### Pitch test

Recorded peaks:

| Axis | Minimum | Maximum |
|------|--------:|--------:|
| Gyro X | -2092 | +1548 |
| Gyro Y | -4985 | +7194 |
| Gyro Z | -2070 | +1951 |

Gyro Y showed the dominant response.

Therefore:

Gyro Y = PITCH


### Yaw test

Recorded peaks:

| Axis | Minimum | Maximum |
|------|--------:|--------:|
| Gyro X | -5075 | +5795 |
| Gyro Y | -4554 | +3971 |
| Gyro Z | -12288 | +11014 |

Gyro Z showed the dominant response.

Therefore:

Gyro Z = YAW


## Final IMU Mapping

The experimentally determined mapping is:

| Raw Sensor Axis | Physical Meaning |
|-----------------|------------------|
| Accelerometer X | Longitudinal axis |
| Accelerometer Y | Lateral axis |
| Accelerometer Z | Vertical axis |
| Gyroscope X | Roll |
| Gyroscope Y | Pitch |
| Gyroscope Z | Yaw |


## Result

The onboard M540 IMU is now successfully communicating with the
STM32F031K4.

The firmware can continuously acquire:

- 3-axis acceleration
- 3-axis angular velocity
- temperature

The physical orientation of all accelerometer and gyroscope axes has
also been experimentally determined.

This completes the initial IMU bring-up stage.

## About controller

It was time consuming to identify the right channels, as I spent about 3 week for this, and with no result.
I used ESP32-S3, connected to the MOSI, MISO, CLK, CSN, used bit pirate web flasher, and still no progress.
So I ordered new 2xNRF24L01+ and I will use components of old controller to make a new one.

## 2026-10-07 — 3-Axis Mixer Validation

The rate controller and motor mixer were tested in dry-run mode before
allowing the firmware to drive the physical motors.

### Roll

The initial mixer direction was found to be incorrect during testing.
The Roll mixer signs were corrected and retested.

Result: PASS.

### Pitch

The initial Pitch correction direction was also found to be reversed.
The mixer was corrected based on the measured gyro response.

Result: PASS.

### Yaw

A dedicated diagnostic recorder captured the peak yaw rate and motor
commands during a manual right-yaw rotation.

Measured peak:

- Gyro Yaw: approximately -504 deg/s
- PID Yaw output: +35

Motor response:

- M1 (CW): increased
- M2 (CCW): decreased
- M3 (CW): increased
- M4 (CCW): decreased

This produces a reaction torque opposing the manually applied rotation.

Result: PASS.

All physical motor outputs remained locked at 0% during these tests.

### Conclusion

Roll, Pitch and Yaw correction directions are now verified.

The next milestone is the first propeller-free closed-loop test using the
real motor outputs.

## Current Development Status

The custom flight controller firmware is now capable of reading the IMU,
running a 200 Hz rate-control loop, calculating PID corrections and mixing
them into four virtual motor outputs.

### Completed

- [x] Identified MCU: STM32F031K4
- [x] Identified M540 IMU communication interface
- [x] Confirmed M540 I2C address: `0x69`
- [x] Confirmed `WHO_AM_I = 0x7D`
- [x] Migrated IMU communication from software I2C to hardware I2C1
- [x] Stable 100 kHz hardware I2C communication
- [x] Stable 200 Hz flight-control loop
- [x] Gyroscope calibration
- [x] Identified IMU axes:
  - X = Roll
  - Y = Pitch
  - Z = Yaw
- [x] Identified all four motor outputs
- [x] Confirmed physical motor positions and rotation directions
- [x] Verified TIM1 PWM control of all four motors
- [x] Implemented rate controller
- [x] Implemented X-quad motor mixer
- [x] Verified Roll correction direction in dry-run testing
- [x] Verified Pitch correction direction in dry-run testing
- [x] Verified Yaw correction direction in dry-run testing
- [x] Added motor output limits and IMU failure shutdown

### Motor Layout

                 FRONT
                   ↑

            M2 CCW     M1 CW
         Front-Left   Front-Right

            M3 CW      M4 CCW
          Rear-Left   Rear-Right

STM32 motor mapping:

| Motor | Position | Direction | STM32 |
|------|----------|-----------|-------|
| M1 | Front Right | CW | PA11 / TIM1_CH4 |
| M2 | Front Left | CCW | PA9 / TIM1_CH2 |
| M3 | Rear Left | CW | PA8 / TIM1_CH1 |
| M4 | Rear Right | CCW | PA10 / TIM1_CH3 |

### Verified Mixer

```c
M1 = throttle + roll + pitch + yaw;
M2 = throttle - roll + pitch - yaw;
M3 = throttle - roll - pitch + yaw;
M4 = throttle + roll - pitch - yaw;
