# 🎮 Running the Examples

Example projects for Arduino, ESP32 (ESP-IDF) and STM32 are provided in the `examples` directory.

## Arduino

- 📂 Open `examples/arduino_basic_interface/main.ino` in the Arduino IDE
- 🔧 Adjust the pin numbers to match your wiring
- 🚀 Compile and upload the sketch
- 🖥️ Open the serial monitor to view angle output
  ```
  [Angle] 12345 LSB (67.89°)
  [Diag] AGC=128  MAG=16000
  ```

## ESP32 (ESP-IDF)

- 📂 Copy `examples/es32_basic_interface` into an ESP-IDF project
- 🔧 Implement an `spiBus` using ESP-IDF SPI API
- 🚀 Build and flash using `idf.py flash monitor`
  ```
  Angle: 90.0 deg
  ```

## STM32 HAL

- 📂 Import `examples/stm32_basic_interface` into STM32CubeIDE
- 🔧 Wire the sensor and adjust pins in `app_encoder_init.cpp`
- 🚀 Build and flash your board
- 🖥️ Open the UART console to see:
  ```
  Angle: 45 deg
  ```

Each example demonstrates basic angle reading and can be extended to suit your application.

---
⬅️ **Previous:** [Using the Library](usage.md) | [Back to Documentation Index](README.md) | **Next:** [Quick Start Workflow](workflow.md) ➡️
