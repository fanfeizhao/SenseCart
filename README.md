# SenseCart

SenseCart is a foldable assistive-navigation cart designed for visually impaired users.

The cart detects obstacles at different heights and distances, then communicates their direction through vibration motors in the handle. The goal is to provide obstacle awareness while remaining lightweight, foldable, and practical for everyday use.

## Problem

Traditional white canes are useful for detecting obstacles near the ground, but they may miss obstacles at chest or head level.

Some electronic mobility devices can detect more obstacles, but they may be:

- Expensive
- Bulky
- Difficult to store
- Difficult to transport
- Complicated to operate

SenseCart was designed to combine obstacle detection, storage assistance, and intuitive haptic feedback in one foldable device.

## Solution

SenseCart uses several distance sensors positioned around the cart.

The system follows this process:

```text
Distance sensors
→ ESP32 microcontroller
→ Obstacle-processing logic
→ Left or right vibration motor
```text
## Hardware

SenseCart uses the following main electronic components:

- Arduino Nano ESP32 microcontroller
- VL53L1X time-of-flight distance sensor
- VL53L0X time-of-flight distance sensor
- TF-Nova LiDAR distance sensor
- 64 × 8 multi-zone time-of-flight sensor
- Left and right vibration motors
- Motor driver transistors or MOSFETs
- Rechargeable battery
- Voltage regulator
- Power switch
- Foldable cart frame
- Wires, connectors, mounting brackets, and fasteners

## Main Sensor Roles

### Arduino Nano ESP32

The Arduino Nano ESP32 acts as the main controller. It reads the distance sensors, processes obstacle locations, and controls the vibration motors.

### VL53L1X

The VL53L1X is used for longer-range time-of-flight distance sensing. It can help detect obstacles in front of the cart.

### VL53L0X

The VL53L0X is used for shorter-range obstacle detection and may be positioned to monitor specific areas near the cart.

### TF-Nova

The TF-Nova LiDAR sensor provides fast distance measurements over a longer range. It is used for downward obstacles/Clif fDetection

### 64 × 8 Multi-Zone ToF Sensor

The 64 × 8 time-of-flight sensor measures distance across a grid of zones instead of returning only one distance value.

This allows SenseCart to estimate:

- Whether an obstacle is on the left
- Whether an obstacle is on the right
- Whether an obstacle is in the center
- The approximate size of an obstacle
- The approximate height or position of an obstacle

## System Architecture

VL53L1X
VL53L0X
TF-Nova
64 × 8 ToF sensor
        ↓
Arduino Nano ESP32
        ↓
Obstacle-position and distance processing
        ↓
Left motor / Right motor / Both motors
