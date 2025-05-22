#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "AS5047U.hpp"

// SPI config
#define PIN_NUM_MISO  12
#define PIN_NUM_MOSI  11
#define PIN_NUM_CLK   10
#define PIN_NUM_CS     9

static const char *TAG = "AS5047U_RTOS";

// Shared driver pointer and mutex
AS5047U *encoder = nullptr;
SemaphoreHandle_t spi_mutex = nullptr;

// SPI wrapper subclass
class ESPBus : public spiBus {
    spi_device_handle_t spi;
public:
    ESPBus(spi_device_handle_t handle) : spi(handle) {}
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        spi_device_transmit(spi, &t);
    }
};

// SPI and driver initialization
void init_encoder() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_handle_t spi_dev;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev));

    auto *bus = new ESPBus(spi_dev);
    encoder = new AS5047U(*bus, FrameFormat::SPI_24);
}

// Task: read angle
void angle_task(void *param) {
    while (true) {
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(50))) {
            uint16_t angle = encoder->getAngle(1);
            ESP_LOGI(TAG, "Angle: %u", angle);
            xSemaphoreGive(spi_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task: monitor AGC and Magnitude
void diag_task(void *param) {
    while (true) {
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(50))) {
            uint8_t agc = encoder->getAGC();
            uint16_t mag = encoder->getMagnitude();
            ESP_LOGI(TAG, "AGC: %u, MAG: %u", agc, mag);
            xSemaphoreGive(spi_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task: check and print errors
void error_task(void *param) {
    while (true) {
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(50))) {
            AS5047U_Error err = encoder->getStickyErrorFlags();
            if (err != AS5047U_Error::None) {
                ESP_LOGW(TAG, "Sticky error: 0x%X", static_cast<uint16_t>(err));
            }
            xSemaphoreGive(spi_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {
    spi_mutex = xSemaphoreCreateMutex();
    init_encoder();

    xTaskCreate(angle_task, "angle_task", 4096, NULL, 5, NULL);
    xTaskCreate(diag_task, "diag_task", 4096, NULL, 4, NULL);
    xTaskCreate(error_task, "error_task", 4096, NULL, 3, NULL);
}
