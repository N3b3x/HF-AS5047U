#include "AS5047U.hpp"
#include "cmsis_os.h"
#include "spi_bus.hpp"

// Prototypes
extern void AngleTask(void *, osPriority_t, uint32_t, const char *);
extern void DiagTask(void *, osPriority_t, uint32_t, const char *);
extern void ErrTask(void *, osPriority_t, uint32_t, const char *);

// Call MX_Encoder_Init() after HAL and FreeRTOS are started (e.g. in MX_FREERTOS_Init()).
void MX_Encoder_Init() {
    // 1. Create mutex
    osMutexAttr_t attr = {.name = "spiMtx"};
    spiMtx = osMutexNew(&attr);

    // 2. Create SPI wrapper & driver
    static STM32Bus bus(&hspi1, ENCODER_CS_GPIO, ENCODER_CS_PIN);
    static AS5047U driver(bus, FrameFormat::SPI_24);
    enc = &driver;

    // Optionally configure once
    {
        Locker lock(spiMtx);
        enc->configureInterface(true, false, false); // ABI only
        enc->setABIResolution(12);
    }

    // 3. Spawn tasks (stack, priority)
    osThreadNew(
        (osThreadFunc_t)AngleTask, nullptr,
        new osThreadAttr_t{.name = "Angle", .priority = osPriorityNormal, .stack_size = 1024});

    osThreadNew((osThreadFunc_t)DiagTask, nullptr,
                new osThreadAttr_t{.name = "Diag", .priority = osPriorityLow, .stack_size = 1024});

    osThreadNew((osThreadFunc_t)ErrTask, nullptr,
                new osThreadAttr_t{.name = "Err", .priority = osPriorityLow, .stack_size = 1024});
}
