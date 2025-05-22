# HF-AS5047U
Hardware Agnostic AS5047U library - as used in the HardFOC-V1 controller

# AS5047U C++ Driver Library
## AS5047U Sensor Overview  
The **AS5047U** is a high-resolution magnetic rotary position sensor providing fast absolute angle measurements over a full 360° rotation. It outputs a 14-bit digital angle (16384 counts per revolution) via a standard 4-wire SPI interface, with optional 8-bit CRC for reliability. Integrated Dynamic Angle Error Compensation (DAEC) and an adaptive Dynamic Filter System (DFS™) ensure low latency and reduced noise across speed ranges, while inherent immunity to homogeneous external stray magnetic fields enhances robustness.  
Beyond SPI, the AS5047U also offers configurable incremental encoder outputs (A, B, I) up to 4096 pulses per revolution, a 3-phase commutation interface (UVW) with programmable pole pairs, and a PWM-encoded absolute angle output.  
Typical applications include replacing optical encoders or resolvers in servo and BLDC/PMSM motor control, robotics, and any system demanding precise, fast shaft-angle feedback.  

**Key Sensor Features:**  
- **Absolute angle:** 14-bit digital output (0–16383 counts per rev) with optional 8-bit CRC for SPI frames.  
- **Dynamic Angle Error Compensation (DAEC):** Low-latency correction at high speed.  
- **Adaptive Filter System (DFS™):** Noise reduction at low speed.  
- **Stray-field immunity:** Robust against homogeneous external magnetic interference.  
- **Incremental encoder (ABI):** Configurable A/B/I outputs up to 4096 pulses per revolution.  
- **UVW commutation:** 3-phase outputs with 1–7 programmable pole pairs.  
- **PWM output:** Programmable PWM-encoded absolute angle on a single pin.  
- **Diagnostics:** Real-time AGC and magnitude readings plus sticky/transient error flags (CRC, framing, ECC, etc.).  
- **OTP memory:** One-time programmable non-volatile registers for zero position and settings, with guard-band verification. *

## Library Architecture  
This C++ driver implements a class `AS5047U` that encapsulates all major sensor features in a clear, type-safe API. The core components of the library are:  

- **`AS5047U` class:** High-level interface for reading angles, velocity, diagnostics, and configuring the sensor.  
- **Register definitions:** A header (`AS5047U_REGISTERS.hpp`) contains `struct` definitions for each sensor register (volatile and OTP).  
- **FrameFormat enum:** An `enum class FrameFormat { SPI_16, SPI_24, SPI_32 }` selects the SPI frame size.  
- **Virtual SPI interface:** The abstract class `spiBus` defines a single pure-virtual method `transfer(tx, rx, len)`.

## SPI Bus Abstraction  
A key feature of this driver is its **hardware-agnostic SPI interface**. The abstract class `spiBus` defines the interface:  

```cpp
class spiBus {
public:
    virtual ~spiBus() = default;
    virtual void transfer(const uint8_t *tx, uint8_t *rx, size_t len) = 0;
};
```

Users create a subclass of `spiBus` that implements `transfer()` using their platform’s SPI functions.

## Platform Integration Examples  

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

## Using the Driver API  

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

## API Reference  

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

## C++ Features and Requirements  
This library requires a **C++20 (or later)** compiler. It uses:
- `enum class`
- `constexpr`
- `std::bitset`
- `[[nodiscard]]`
- Modern structured typing

## Installation  

1. Copy `AS5047U.hpp`, `AS5047U.cpp`, and `AS5047U_REGISTERS.hpp` into your project.
2. Include the headers: `#include "AS5047U.hpp"`
3. Compile with `-std=c++20`
4. Implement your `spiBus` class.




## License  
**GNU General Public License v3.0**  
You may use, modify, and redistribute this software under GPLv3.
