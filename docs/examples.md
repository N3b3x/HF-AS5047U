# Running the Examples

Example projects for Arduino, ESP32 (ESP-IDF) and STM32 are provided in the `examples` directory.

## Arduino

1. Open `examples/arduino_basic_interface/main.ino` in the Arduino IDE.
2. Adjust the pin numbers at the top of the sketch to match your wiring.
3. Select your board and COM port.
4. Compile and upload the sketch.
5. Open the serial monitor to view angle and diagnostic output.
   You should see lines similar to:

   ```
   [Angle] 12345 LSB (67.89°)
   [Diag] AGC=128  MAG=16000
   ```

## ESP32 (ESP-IDF)

1. Copy the contents of `examples/es32_basic_interface` into an ESP-IDF project.
2. Implement an `spiBus` using ESP-IDF's SPI API as shown in `app_main.cpp`.
3. Build and flash using `idf.py flash monitor`.
   The monitor will print the angle repeatedly, for example:

   ```
   Angle: 90.0 deg
   ```

## STM32 HAL

1. Import `examples/stm32_basic_interface` into your STM32CubeIDE workspace.
2. Wire the sensor to an SPI peripheral and adjust the pin definitions in `app_encoder_init.cpp`.
3. Build the project and flash your board.
4. Open the UART console to observe output similar to:

   ```
   Angle: 45 deg
   ```

Each example demonstrates basic angle reading and can be extended to suit your application.
