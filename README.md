# 🚗 ESP32 RC Car with PS4 Controller

A feature-rich RC car built with ESP32, controlled via PS4 DualShock controller over Bluetooth. Includes advanced lighting system, trip computer, battery monitoring, and real-time telemetry.

![ESP32 RC Car](https://img.shields.io/badge/Platform-ESP32-blue?style=for-the-badge&logo=espressif)
![PS4 Controller](https://img.shields.io/badge/Controller-PS4_DualShock-orange?style=for-the-badge&logo=playstation)
![Build Status](https://img.shields.io/badge/Build-Passing-green?style=for-the-badge)

## 📋 Table of Contents

- [Hardware Specifications](#-hardware-specifications)
- [Functional Features](#-functional-features)
- [Wiring Diagram](#-wiring-diagram)
- [Software Architecture](#-software-architecture)
- [Building and Uploading](#-building-and-uploading)
- [Controls](#-controls)
- [Display System](#-display-system)
- [Troubleshooting](#-troubleshooting)
- [Development](#-development)

## 🔧 Hardware Specifications

### **Main Components**
- **Microcontroller**: ESP32-WROOM-32 Development Board
- **Motors**: 2× DC Geared Motors with Quadrature Encoders (3638 counts/rev)
- **Motor Driver**: L298N-compatible Dual H-Bridge
- **Servo**: Standard servo motor for steering (controlled via PCA9685)
- **Display**: 128×64 SSD1306 OLED (I2C)
- **PWM Driver**: PCA9685 16-Channel PWM Driver (I2C)
- **Battery**: Conrad Energy 20C 3700mAh 7.4V LiPo
- **Lighting**: 8× LEDs (4× White headlights, 4× Red tail lights)

### **Power Consumption**
- **Idle**: ~360mA
- **Normal Driving**: ~1.75A
- **Aggressive Driving**: ~2.55A
- **Battery Life**: 1.5-2+ hours continuous driving

### **Physical Specifications**
- **Wheel Circumference**: 0.2199m (calibrated)
- **Encoder Resolution**: 3638 counts per revolution
- **Servo Range**: 50°-130° (mechanically limited)
- **Maximum Speed**: Variable (PID controlled)

## ⚡ Functional Features

### **🎮 Control System**
- **Wireless Control**: PS4 DualShock controller via Bluetooth HID
- **Auto-reconnection**: Automatic connection recovery
- **Safety Systems**: Motor stop on connection loss
- **Connection Health Monitoring**: Real-time connection status

### **🚗 Driving Features**
- **Differential Drive**: Independent left/right motor control
- **PID Motor Control**: Precise speed regulation
- **Encoder Feedback**: Real-time speed and distance measurement
- **Multiple Drive Modes**: Normal, High-speed, Direct control, Diagnostics

### **💡 Advanced Lighting System**
- **4-Pin Independent Control**: Separate left/right headlights and tail lights
- **Automatic Turn Signals**: Activated by steering input (±100 threshold)
- **Brake Lights**: Automatic activation on brake trigger (>50)
- **Headlight Flashing**: 2-second flash sequence (5Hz)
- **Smart Priority System**: Flashing → Turn Signals → Brake Lights → Normal

### **📊 Trip Computer**
- **4-Page Navigation**: Total ODO, Trip 1, Trip 2, Current Trip
- **Persistent Storage**: NVS-based odometry storage
- **Distance Formatting**: Automatic m/km conversion
- **Trip Reset**: Hold D-pad DOWN for 3 seconds
- **Circular Navigation**: D-pad LEFT/RIGHT page switching

### **🔋 Battery Management**
- **Real-time Monitoring**: ADC-based voltage measurement
- **Voltage Divider**: 4:1 ratio (3× 10kΩ resistors)
- **Percentage Display**: 6.4V-8.8V range mapping
- **Filtered Readings**: Stable battery percentage calculation
- **Visual Indicator**: Top-right corner percentage display

### **🖥️ Display System**
- **Startup Screen**: Battery%, Bluetooth status, Total ODO
- **Trip Computer**: 4-page odometry with navigation dots
- **Status Icons**: Bluetooth, LED status, turn signals
- **Real-time Updates**: 1-second refresh rate

## 🔌 Wiring Diagram

### **ESP32 Pin Configuration**
```
GPIO 15 → Left Headlights (2× White LEDs + 220Ω resistors)
GPIO 4  → Right Headlights (2× White LEDs + 220Ω resistors)
GPIO 16 → Left Tail Lights (2× Red LEDs + 220Ω resistors)
GPIO 17 → Right Tail Lights (2× Red LEDs + 220Ω resistors)

GPIO 18 → Left Motor PWM
GPIO 19 → Left Motor INA
GPIO 13 → Left Motor INB
GPIO 5  → Right Motor PWM
GPIO 23 → Right Motor INA
GPIO 12 → Right Motor INB

GPIO 25 → Left Encoder A
GPIO 26 → Left Encoder B
GPIO 14 → Right Encoder A
GPIO 27 → Right Encoder B

GPIO 21 → I2C SDA (OLED + PCA9685)
GPIO 22 → I2C SCL (OLED + PCA9685)

GPIO 34 → Battery Voltage (via 4:1 voltage divider)
```

### **I2C Devices**
- **0x3C**: SSD1306 OLED Display
- **0x40**: PCA9685 PWM Driver (Servo Channel 0)

### **Power Supply**
- **Main Power**: 7.4V LiPo → Motors, Servo
- **Logic Power**: ESP32 3.3V → LEDs, Logic circuits
- **Voltage Divider**: 7.4V → 1.85V (GPIO 34)

## 🏗️ Software Architecture

### **Core Systems**
- **Bluepad32 Framework**: PS4 controller HID support
- **FreeRTOS**: Dual-core task management
- **NVS Storage**: Non-volatile odometry persistence
- **I2C Communication**: Multi-device bus management

### **Control Loops**
- **Main Loop**: Controller input, lighting updates, display refresh
- **Motor Control**: PID-based speed regulation (50Hz)
- **Encoder Reading**: Interrupt-based quadrature decoding
- **Battery Monitoring**: 1Hz filtered voltage reading

### **Safety Features**
- **Watchdog Timer**: System stability monitoring
- **Connection Timeout**: 2-second controller loss detection
- **Motor Safety**: Automatic stop on disconnection
- **Servo Limiting**: Mechanical binding prevention (50°-130°)

## 🛠️ Building and Uploading

### **Prerequisites**
- **PlatformIO** (VS Code extension or CLI)
- **Python 3.9+** with pip
- **Git** for version control

### **Setup Instructions**

1. **Clone Repository**
   ```bash
   git clone <repository-url>
   cd esp32-rc-car
   ```

2. **Install PlatformIO**
   ```bash
   python3 -m pip install platformio
   ```

3. **Build Firmware**
   ```bash
   python3 -m platformio run
   ```

4. **Upload to ESP32**
   ```bash
   python3 -m platformio run --target upload
   ```

5. **Monitor Serial Output**
   ```bash
   python3 -m platformio device monitor
   ```

### **Build Configuration**
- **Platform**: espressif32@6.10.0
- **Framework**: Arduino with Bluepad32
- **Board**: ESP32 Dev Module
- **Upload Speed**: 921600 baud
- **Monitor Speed**: 115200 baud

### **Dependencies**
```ini
lib_deps = 
    adafruit/Adafruit GFX Library@^1.11.9
    adafruit/Adafruit SSD1306@^2.5.10
    adafruit/Adafruit PWM Servo Driver Library@^3.0.2
```

## 🎮 Controls

### **PS4 Controller Mapping**

#### **Driving**
- **Left Stick X**: Steering (±511 range)
- **Right Trigger (R2)**: Throttle (0-1023)
- **Left Trigger (L2)**: Brake (0-1023)

#### **Lighting**
- **D-pad UP (tap)**: Toggle all lights ON/OFF
- **D-pad UP (hold 1s)**: Flash headlights for 2 seconds
- **Steering**: Auto turn signals (±100 threshold)
- **Brake Trigger**: Auto brake lights (>50 threshold)

#### **Navigation**
- **D-pad LEFT/RIGHT**: Navigate trip computer pages
- **D-pad DOWN (hold 3s)**: Reset current trip

#### **Special Modes**
- **X Button**: High-speed mode override
- **Square**: Direct PWM motor test mode
- **L1 + R1**: Motor diagnostics mode
- **L1 + R1 + Circle**: Battery diagnostics

### **Automatic Features**
- **Turn Signals**: Activate when steering beyond ±100
- **Brake Lights**: Activate when brake > 50
- **Auto-reconnection**: Attempts reconnection every 5 seconds
- **Safety Stop**: Motors stop immediately on connection loss

## 📱 Display System

### **Startup Screen**
```
[BT][LED]                    [BATT%]
     ┌─────────────────────┐
     │     Robot Logo      │
     │     (Bitmap)        │
     └─────────────────────┘
           ODO: 123.4m
```

### **Trip Computer Pages**
```
[BT][LED]              [BATT%]

         Page Title
        
      ████████████
      █  123.4m  █
      ████████████
      
      Footer Text (if any)
      
      ●●○● (page dots)
```

### **Status Icons**
- **Bluetooth**: Connected/Disconnected state
- **LED Status**: 
  - 💡 Filled bulb: Lights ON
  - 💡 Hollow bulb: Lights OFF  
  - ◄ Left arrow: Left turn signal
  - ► Right arrow: Right turn signal

## 🔧 Troubleshooting

### **Common Issues**

#### **Controller Won't Connect**
- Ensure PS4 controller is in pairing mode (hold Share + PS buttons)
- Check Bluetooth range (<10 meters)
- Restart ESP32 and try pairing again
- Verify Bluepad32 framework is correctly installed

#### **Motors Don't Move**
- Check motor driver connections (PWM, INA, INB pins)
- Verify motor power supply (7.4V)
- Test with motor diagnostics mode (L1 + R1)
- Check encoder connections for feedback

#### **LEDs Don't Light**
- Verify GPIO pins are not input-only (avoid 34, 35, 36, 39)
- Check LED polarity (long leg to resistor)
- Test individual LEDs with multimeter
- Ensure 220Ω current-limiting resistors are used

#### **Display Issues**
- Verify I2C connections (SDA=21, SCL=22)
- Check OLED I2C address (0x3C)
- Test I2C bus with scanner code
- Ensure 3.3V power supply to OLED

#### **Battery Monitoring**
- Verify voltage divider (3× 10kΩ resistors)
- Check ADC pin connection (GPIO 34)
- Test with multimeter: expect 1.85V for 7.4V battery
- Calibrate voltage constants if needed

### **Debug Commands**
- **Battery Diagnostics**: L1 + R1 + Circle
- **Motor Tests**: L1 + R1 (hold)
- **Serial Monitor**: 115200 baud for debug output
- **Connection Status**: Displayed on OLED

## 👨‍💻 Development

### **Code Structure**
```
src/
├── main.cpp              # Main application code
├── trip_computer.cpp     # Odometry and navigation (integrated)
├── led_control.cpp       # Lighting system (integrated)
├── motor_control.cpp     # PID and motor drivers (integrated)
└── display.cpp           # OLED graphics (integrated)

platformio.ini            # Build configuration
CLAUDE.md                 # Development documentation
README.md                 # This file
```

### **Key Configuration Constants**
```cpp
// Motor Control
#define MAX_PWM 255
#define MIN_PWM 50
#define COUNTS_PER_REV 3638
#define WHEEL_CIRCUMFERENCE_M 0.2199f

// Battery Monitoring  
const float batteryMaxVoltage = 8.8f;
const float batteryMinVoltage = 6.4f;
const float voltageDividerRatio = 4.0f;

// Servo Control
#define SERVO_CENTER 90
#define SERVO_MAX_LEFT 130
#define SERVO_MAX_RIGHT 50
```

### **Adding New Features**
1. **Update pin definitions** in main.cpp header
2. **Add initialization code** in setup()
3. **Implement control logic** in processGamepad()
4. **Update display** in displayTripComputer() or displayStartupScreen()
5. **Test thoroughly** with serial monitor output

### **Performance Monitoring**
- **RAM Usage**: 27.1% (88,660 bytes)
- **Flash Usage**: 24.6% (774,165 bytes)
- **Loop Frequency**: ~50Hz (20ms cycle time)
- **I2C Frequency**: 400kHz (fast mode)

## 📄 License

This project is open source. Feel free to modify and distribute according to your needs.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 🙏 Acknowledgments

- **Bluepad32** framework for PS4 controller support
- **Adafruit** libraries for hardware drivers
- **PlatformIO** for excellent ESP32 development environment
- **ESP32** community for extensive documentation and examples

---

**🚗 Happy driving! Built with ❤️ and ESP32**