# HF-AS5047U
Hardware Agnostic AS5047U library - as used in the HardFOC-V1 controller

# AS5047U – C++ Driver Library

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)


## 📦 Overview
**HF-AS5047U** is a portable C++20 driver for the **AS5047U** magnetic encoder from ams. It delivers fast 14‑bit absolute angle readings over SPI, optional CRC protection and advanced features like Dynamic Angle Error Compensation (DAEC) and an adaptive Dynamic Filter System (DFS™). The sensor can also output incremental (A/B/I) and 3‑phase commutation (UVW) signals or a PWM encoded angle, making it a drop‑in replacement for optical encoders in high-performance motor control and robotics.

## 🚀 Sensor Highlights
* **14‑bit absolute angle** with optional CRC check
* **DAEC** for low‑latency angle correction
* **DFS™** adaptive noise filtering
* **Stray‑field immunity** for robust operation
* **Incremental ABI** outputs up to 4096 PPR
* **UVW commutation** with programmable pole pairs
* **PWM output** option
* **Diagnostics** for AGC, magnitude and error flags
* **OTP memory** for permanent configuration

---

## 🏗️ Library Architecture
This library exposes a single `AS5047U` class that wraps all sensor functionality in a clear, type‑safe API. Key components include:

* **`AS5047U` class** – high level interface
* **Register definitions** in `AS5047U_REGISTERS.hpp`
* **`FrameFormat` enum** to select 16/24/32‑bit SPI frames
* **`spiBus` interface** – abstract SPI layer for platform independence

### SPI Bus Abstraction
Applications implement the virtual `spiBus` interface and plug it into the driver:

```cpp
class spiBus {
public:
    virtual ~spiBus() = default;
    virtual void transfer(const uint8_t *tx, uint8_t *rx, size_t len) = 0;
};
```

The driver itself contains no hardware specifics – simply implement `transfer()` for your platform.

## 🔌 Platform Integration

### ESP-IDF  
```cpp
class ESPBus : public AS5047U::spiBus {
    spi_device_handle_t dev;
public:
    ESPBus(spi_device_handle_t handle) : dev(handle) {}
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        spi_transaction_t t = {};
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        t.length = len * 8;
        spi_device_transmit(dev, &t);
    }
};
```

### STM32 HAL  
```cpp
class STM32Bus : public AS5047U::spiBus {
    SPI_HandleTypeDef *hspi;
public:
    STM32Bus(SPI_HandleTypeDef *handle) : hspi(handle) {}
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        HAL_SPI_TransmitReceive(hspi, (uint8_t*)tx, rx, len, HAL_MAX_DELAY);
    }
};
```

### Arduino  
```cpp
class ArduinoBus : public AS5047U::spiBus {
public:
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(CS_PIN, LOW);
        for (size_t i = 0; i < len; ++i) {
            uint8_t out = tx ? tx[i] : 0;
            uint8_t in  = SPI.transfer(out);
            if (rx) rx[i] = in;
        }
        digitalWrite(CS_PIN, HIGH);
        SPI.endTransaction();
    }
};
```

---

## 📂 Project Structure

```
├── Datasheet/               # AS5047U datasheet PDF
├── examples/                # Wiring and usage examples
├── src/                     # Library sources
│   ├── AS5047U.hpp          # Driver API
│   ├── AS5047U.cpp          # Implementation
│   └── AS5047U_REGISTERS.hpp# Register definitions
├── AS5047U_config.hpp       # Default build configuration
├── tests/                   # Mock-based unit tests
├── config.mk                # Makefile defaults
└── README.md                # This document
```

## 📖 Documentation
Detailed step‑by‑step guides (with example command output) are available in the [docs](docs/README.md) folder. Start with the [Quick Start Workflow](docs/workflow.md) if you're new.

---

## 🔧 Installation
1. Copy `AS5047U.hpp`, `AS5047U.cpp` and `AS5047U_REGISTERS.hpp` into your project.
2. Implement the `spiBus` interface for your platform.
3. Include the header: `#include "AS5047U.hpp"`.
4. Compile with a **C++20** or newer compiler.
5. Optionally build using the provided `Makefile`.
   Compiler flags can be overridden on the command line. Configuration
   options such as default SPI frame format and CRC retries can be set
   through `Kconfig` or by editing `AS5047U_config.hpp`.

---

## 🧠 Quick Start

```cpp
AS5047U encoder(bus, FrameFormat::SPI_24);

uint16_t angle = encoder.getAngle();
uint16_t rawAngle = encoder.getRawAngle();
int16_t vel = encoder.getVelocity();
double vel_dps = encoder.getVelocityDegPerSec();

uint8_t agc = encoder.getAGC();
uint16_t mag = encoder.getMagnitude();
uint16_t errors = encoder.getErrorFlags();
```

Configure outputs:
```cpp
encoder.setZeroPosition(8192);
encoder.setDirection(false);
encoder.setABIResolution(12);
encoder.setUVWPolePairs(5);
encoder.configureInterface(true, false, true);
```

Perform OTP programming:
```cpp
bool ok = encoder.programOTP();
```

Dump diagnostics:
```cpp
std::string status = encoder.dumpDiagnostics();
```

## ⚙️ Configuration
Projects that use a Kconfig-based build system can include the
provided `Kconfig` file. It exposes options such as the default SPI
frame format and CRC retry count and also allows enabling the unit
tests. When not using Kconfig, you can edit `AS5047U_config.hpp` to
set `AS5047U_CFG::DEFAULT_FRAME_FORMAT` and `AS5047U_CFG::CRC_RETRIES`.

## 📟 API Summary

| Function | Description |  
|----------|-------------|  
| AS5047U(spiBus &bus, FrameFormat fmt) | Constructor (SPI interface and frame format) |  
| void setFrameFormat(FrameFormat fmt) | Set SPI frame format (16/24/32-bit mode) |  
| uint16_t getAngle(uint8_t retries=0) | Read 14-bit compensated absolute angle |  
| uint16_t getRawAngle(uint8_t retries=0) | Read 14-bit raw absolute angle |  
| int16_t getVelocity(uint8_t retries=0) | Read signed 14-bit velocity (LSB units) |  
| double getVelocityDegPerSec(uint8_t retries=0) | Velocity in degrees/sec |  
| double getVelocityRadPerSec(uint8_t retries=0) | Velocity in radians/sec |  
| double getVelocityRPM(uint8_t retries=0) | Velocity in revolutions per minute (RPM) |  
| uint8_t getAGC(uint8_t retries=0) | Read AGC (0–255) value |  
| uint16_t getMagnitude(uint8_t retries=0) | Read magnetic field magnitude (0–16383) |  
| uint16_t getErrorFlags(uint8_t retries=0) | Read and clear error/status flags |  
| void dumpStatus() const | Print formatted status/diagnostics |
| uint16_t getZeroPosition(uint8_t retries=0) const | Get current zero offset |
| bool setZeroPosition(uint16_t angle, uint8_t retries=0) | Set new zero offset |  
| bool setDirection(bool clockwise, uint8_t retries=0) | Set rotation direction (CW or CCW) |  
| bool setABIResolution(uint8_t bits, uint8_t retries=0) | Set ABI output resolution (10–14 bits) |  
| bool setUVWPolePairs(uint8_t pairs, uint8_t retries=0) | Set UVW pole pairs (1–7) |  
| bool setIndexPulseLength(uint8_t lsbLen, uint8_t retries=0) | Set ABI index pulse width |  
| bool configureInterface(bool abi, bool uvw, bool pwm, uint8_t retries=0) | Enable/disable ABI, UVW, PWM |  
| bool setDynamicAngleCompensation(bool enable, uint8_t retries=0) | Enable/disable DAEC |  
| bool setAdaptiveFilter(bool enable, uint8_t retries=0) | Enable/disable adaptive filter (DFS) |  
| bool setFilterParameters(uint8_t k_min, uint8_t k_max, uint8_t retries=0) | Set DFS filter parameters |  
| bool set150CTemperatureMode(bool enable, uint8_t retries=0) | Enable 150°C (high-temp mode) |  
| bool programOTP() | Program current settings into OTP (one-time) |  
| void setPad(uint8_t pad) | Set pad byte for 32-bit SPI frames |  
| bool setHysteresis(SETTINGS3::Hysteresis hys, uint8_t retries=0) | Set incremental hysteresis level |  
| SETTINGS3::Hysteresis getHysteresis() const | Get current hysteresis setting |  
| bool setAngleOutputSource(SETTINGS2::AngleOutputSource src, uint8_t retries=0) | Select angle output source (comp/raw) |  
| SETTINGS2::AngleOutputSource getAngleOutputSource() const | Get selected angle output source |  
| AS5047U_REG::DIA getDiagnostics() const | Read full diagnostic register (DIA) |  
---

## 🧪 Unit Testing
Run the provided tests on a desktop system with:

```bash
g++ -std=c++20 src/AS5047U.cpp tests/test_as5047u.cpp -o test && ./test
```

The expected output is:

```
All tests passed
```


## C++ Features and Requirements
This library requires a **C++20 (or later)** compiler. It uses:
- `enum class`
- `constexpr`
- `std::bitset`
- `[[nodiscard]]`
- Modern structured typing




## License
**GNU General Public License v3.0**
You may use, modify, and redistribute this software under GPLv3.

---

## 🤝 Contributing
Pull requests and feature ideas are welcome!
1. Fork this repository
2. Create a feature branch
3. Commit your changes
4. Open a PR

## 🙌 Acknowledgments
Developed for the HardFOC-V1 controller.
- Developed by **Nebiyu Tadesse**, 2025.
- Based on the official [AS5047U Datasheet](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5047u-high-resolution-rotary-position-sensor) by ASM ASRAM.
- Inspired by embedded needs for a reliable, testable AS5047U library.


## 💬 Support

For questions, bug reports, or feature requests, please open an [issue](https://github.com/N3b3x/HF-AS5047U/issues) or contact the maintainer directly.
