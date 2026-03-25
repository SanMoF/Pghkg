# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 embedded firmware project using PlatformIO with ESP-IDF framework. The codebase provides a collection of hardware abstraction libraries for sensors and actuators, with the main application in `src/main.cpp`.

## Build & Development Commands

```bash
# Build project
pio run

# Flash to device
pio run --target upload

# Start serial monitor (115200 baud)
pio device monitor

# Build with ESP-IDF directly (alternative)
idf.py build
idf.py flash
idf.py monitor
```

## Architecture

**Target:** ESP32 (`board = esp32dev`, `framework = espidf`)

**Structure:**
- `src/main.cpp` - Application entry point
- `lib/<module>/` - Hardware abstraction libraries, each with C++ wrapper classes:
  - **Sensors:** AS5600 (magnetic encoder, I2C), TCS34725 (color), Ultrasonic, Joystick, LineF
  - **Actuators:** SimplePWM (LED/timing), SimpleGPIO, Stepper, Servo_Stepper, HBridge, Buzzer
  - **Communication:** SimpleUART, SimpleSerialBT (Bluetooth), SimpleI2C
  - **Control:** PID_CAYETANO, SimplePID, Filter, Robotics
  - **Utilities:** SimpleLCD, SimpleRGB, SimpleTimer, SimpleADC, Keyboard, Viscometer

**Pattern:** Each library uses a C++ class with:
- `begin()` / `setup()` for initialization
- ESP-IDF native APIs (GPIO, I2C, LEDC/PWM, UART drivers)
- Header in `include/` or `Include/`, implementation in `.cpp`

**Main application** (`src/main.cpp`): Homing routine for stepper motor using AS5600 magnetic encoder feedback with deadband control.

## Key Dependencies

- ESP-IDF components: `driver`, `esp_timer`, `freertos`, `i2c`, `ledc` (PWM), `gpio`
- Unity test framework available in `.pio/libdeps/native/Unity/` (for native unit testing)
