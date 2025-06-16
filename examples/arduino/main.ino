#include <Arduino.h>
#include <SPI.h>
#include "AS5047U.hpp"

// ---------- GPIO pins (adapt to your wiring) ----------
static const uint8_t PIN_MISO = 19;
static const uint8_t PIN_MOSI = 23;
static const uint8_t PIN_SCK  = 18;
static const uint8_t PIN_CS   = 5;     // Chip‑select

// ---------- Mutex & driver globals ----------
static SemaphoreHandle_t spiMutex = nullptr;
static AS5047U* encoder = nullptr;

// ---------- spiBus wrapper ----------
class ArduinoBus : public spiBus {
public:
    ArduinoBus(SPIClass& spi, uint8_t cs) : spi_(spi), csPin_(cs) {
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH);     // deselect
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
    SPIClass&  spi_;
    uint8_t    csPin_;
};

// ---------- Tasks ----------
void angleTask(void*) {
    for (;;) {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(20))) {
            uint16_t ang = encoder->getAngle(1);
            Serial.printf("[Angle] %u LSB (%0.2f°)\n",
                          ang, ang * 360.0 / 16384.0);
            xSemaphoreGive(spiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));   // 10 Hz
    }
}

void diagTask(void*) {
    for (;;) {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50))) {
            uint8_t agc  = encoder->getAGC();
            uint16_t mag = encoder->getMagnitude();
            Serial.printf("[Diag] AGC=%u  MAG=%u\n", agc, mag);
            xSemaphoreGive(spiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));   // 2 Hz
    }
}

void errorTask(void*) {
    for (;;) {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50))) {
            AS5047U_Error e = encoder->getStickyErrorFlags();
            if (e != AS5047U_Error::None) {
                Serial.printf("[ERR] 0x%04X\n", static_cast<uint16_t>(e));
            }
            xSemaphoreGive(spiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 Hz
    }
}

// ---------- Arduino setup ----------
void setup() {
    Serial.begin(115200);
    delay(300);

    // Configure & start SPI bus
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    SPI.beginTransaction(SPISettings(4'000'000, MSBFIRST, SPI_MODE1));

    // Construct driver & mutex
    spiMutex = xSemaphoreCreateMutex();
    static ArduinoBus bus(SPI, PIN_CS);
    static AS5047U drv(bus, FrameFormat::SPI_24);
    encoder = &drv;

    // Optionally configure sensor once here (dir, resolution, etc.)
    encoder->setABIResolution(12);
    encoder->configureInterface(true, false, false); // ABI only

    // Create tasks (stack 4 kB each, default priority 1)
    xTaskCreatePinnedToCore(angleTask, "angle", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(diagTask,  "diag",  4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(errorTask, "error", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
    // nothing — work is done in the RTOS tasks
}
