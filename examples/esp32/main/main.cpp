#include "AS5047U.hpp"
#include <cstdio>

class DummyBus : public AS5047U::spiBus {
  public:
    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) override {
        (void)tx;
        for (size_t i = 0; i < len; ++i) {
            if (rx)
                rx[i] = 0;
        }
    }
};

extern "C" void app_main(void) {
    DummyBus bus;
    AS5047U sensor(bus);
    printf("AGC: %u\n", sensor.getAGC());
}
