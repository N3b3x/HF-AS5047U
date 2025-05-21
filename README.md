# HF-AS5047U
# AS5047U C++ Driver Library
Hardware Agnostic AS5047U library - as used in the HardFOC-V1 controller

## AS5047U Sensor Overview  
The **AS5047U** is a high‑resolution, 14‑bit on‑axis magnetic rotary position sensor that provides absolute angle measurements over a full 360° range. It features integrated **Dynamic Angle Error Compensation (DAEC)** for low latency at high speeds, plus a Dynamic Filter System (DFS) to reduce noise at low speed. The sensor outputs a 14‑bit angle (0–16383 LSBs per revolution) over a standard 4‑wire SPI interface (24‑ or 32‑bit frames include an 8‑bit CRC). In addition to SPI, the AS5047U provides **incre...

**Key Sensor Features:**  
- **Absolute angle:** 14‑bit resolution (0–16383 counts per rev) for full 360°.  
- **ABI encoder output:** Configurable incremental (A/B/I) signals up to 14‑bit resolution.  
- **UVW interface:** Three-phase commutation outputs for BLDC motors (1–7 pole pairs).  
- **PWM output:** Programmable PWM-encoded angle on a single pin.  
- **DAEC and filtering:** Dynamic Angle Error Compensation and adaptive filtering for improved accuracy.  
- **Diagnostics:** Internal gain (AGC) and magnetic field magnitude readings, plus error flags (CRC, framing, etc.).  
- **OTP memory:** Programmable nonvolatile registers for zero position, counts, settings, with verification.  

## Library Architecture  
This C++ driver implements a class `AS5047U` that encapsulates all major sensor features in a clear, type-safe API. The core components of the library are:  

- **`AS5047U` class:** High-level interface for reading angles, velocity, diagnostics, and configuring the sensor.  
- **Register definitions:** A generated header (e.g. `AS5047U_REGISTERS.hpp`) contains `struct` definitions for each sensor register (volatile and OTP).  
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
class ESPBus : public spiBus {
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
class STM32Bus : public spiBus {
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
class ArduinoBus : public spiBus {
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
