#pragma once
#include "AS5047U.hpp"
#include "stm32f4xx_hal.h" // adjust series header
#include <cstdint>

class STM32Bus : public spiBus {
  public:
    STM32Bus(SPI_HandleTypeDef *h, GPIO_TypeDef *csPort, uint16_t csPin)
        : hspi(h), csPort(csPort), csPin(csPin) {}

    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        // CS low
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi,
                                const_cast<uint8_t *>(tx), // HAL wants non‑const
                                rx, len, HAL_MAX_DELAY);
        // CS high
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    }

  private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
};
