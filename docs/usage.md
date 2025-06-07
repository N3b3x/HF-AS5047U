# 📚 Using the Library

This document shows how to integrate `HF-AS5047U` into your own project.

## 1. Provide a SPI Bus Implementation

The driver uses a small `spiBus` interface. Implement the `transfer()` method for your platform. Example for Arduino:

```cpp
class ArduinoBus : public spiBus {
public:
    ArduinoBus(SPIClass& spi, uint8_t cs) : spi_(spi), csPin_(cs) {
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH);
    }
    void transfer(const uint8_t* tx, uint8_t* rx, size_t len) override {
        digitalWrite(csPin_, LOW);
        for (size_t i = 0; i < len; ++i) {
            uint8_t out = tx ? tx[i] : 0x00;
            uint8_t in  = spi_.transfer(out);
            if (rx) rx[i] = in;
        }
        digitalWrite(csPin_, HIGH);
    }
private:
    SPIClass& spi_;
    uint8_t   csPin_;
};
```

## 2. Create the Driver

Pass your bus implementation and desired frame format to the constructor:

```cpp
ArduinoBus bus(SPI, CS_PIN);                 // platform SPI wrapper
AS5047U encoder(bus, FrameFormat::SPI_24);    // use 24-bit frames
```

## 3. Read Data

Call the API methods to access sensor data:

```cpp
uint16_t angle = encoder.getAngle();
uint8_t agc    = encoder.getAGC();
```

Typical output when printing the values might look like:

```
Angle: 16384
AGC: 128
```

You can also read velocity in degrees per second:

```cpp
double velDeg = encoder.getVelocityDegPerSec(); // velocity in deg/s
```

Which might print something like `Velocity: 30.5 deg/s`.

Refer to the [API summary](../README.md#-api-summary) for the full list of methods.

---
⬅️ **Previous:** [Building and Testing](building.md) | [Back to Documentation Index](README.md) | **Next:** [Running the Examples](examples.md) ➡️
