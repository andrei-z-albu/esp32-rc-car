# ESP32 RC Car Project

This project implements a remote-controlled car using ESP32 with the following features:

## Hardware Components
- ESP32-WROOM development board
- Dual motor setup with encoders
- PCA9685 PWM servo driver for steering
- SSD1306 OLED display (128x64)
- PS4/PS5 controller via Bluetooth

## Key Features
- **Bluetooth Controller Support**: PS4/PS5 controllers via Bluepad32
- **Motor Control**: PID-controlled dual motor system with RPM feedback
- **Steering**: Servo-based steering controlled by left joystick
- **Odometry**: Precise distance tracking with encoder feedback
- **OLED Display**: Real-time display of controller status and odometry
- **Multiple Control Modes**:
  - Normal mode (130 RPM max)
  - High-speed mode (200 RPM max)
  - Direct motor test mode
  - Motor diagnostics mode

## Pin Configuration
### Motor Control
- Left Motor: PWM=18, INA=19, INB=13
- Right Motor: PWM=5, INA=23, INB=12
- Left Encoder: A=25, B=26
- Right Encoder: A=14, B=27

### I2C Devices
- SDA=21, SCL=22
- OLED Display: 0x3C
- PCA9685 Servo Driver: 0x40

### Servo
- Steering servo on PCA9685 channel 0

## Controller Mapping
- **Triggers**: RT (throttle forward), LT (brake/reverse)
- **Left Stick**: Steering control
- **Buttons**: 
  - X: High-speed mode
  - Square: Direct motor test mode  
  - L1+R1: Motor diagnostics mode
- **D-pad**: 
  - Left/Right: Navigate trip computer pages (Total, Trip 1, Trip 2, Current Trip)
  - Down (hold 3s): Reset Trip 1 or Trip 2 when on those pages

## Trip Computer
- **4 Pages**: Total Odometry, Trip 1, Trip 2, Current Trip
- **Navigation**: D-pad left/right for circular page navigation
- **Reset**: Hold d-pad down for 3 seconds on Trip 1/2 pages
- **Persistence**: Total, Trip 1, and Trip 2 saved to non-volatile memory
- **Current Trip**: Session-only, resets on power cycle

## Servo Configuration
- **Safe Range**: 50° to 130° (prevents mechanical binding while providing good steering authority)
- **Center**: 90°
- **Protection**: Double-constrained to 30°-150° maximum range
- **Issue**: Original 0°-180° range caused servo stalls at steering extremes

## Build Commands
```bash
# Initialize PlatformIO project
pio project init

# Build the project
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor
```

## Motor Specifications
- Normal operation: 20-130 RPM (6V equivalent)
- High-speed mode: Up to 200 RPM (utilizing 8.25V battery)
- Encoder resolution: 3638 counts per revolution
- Wheel diameter: 120mm (Pololu Wild Thumper)

## Features Implementation
- PID motor control with anti-windup
- Load balancing between motors
- Stall detection and recovery
- Persistent odometry storage in NVS
- Real-time RPM calculation
- Controller button/joystick visualization