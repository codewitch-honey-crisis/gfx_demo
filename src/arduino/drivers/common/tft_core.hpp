#pragma once
#include <Arduino.h>
#if defined(ESP32)
   #define OPTIMIZE_ESP32
   #define OPTIMIZE_DMA
   #define ASSIGNABLE_SPI_PINS
   #define ASSIGNABLE_I2C_PINS
#endif
#if defined(ARDUINO_ARCH_STM32)
    //#define ASSIGNABLE_SPI_PINS
    #define ASSIGNABLE_I2C_PINS
#endif
#if defined(__AVR__)
    #define OPTIMIZE_AVR
#endif
#define FORCE_INLINE __attribute((always_inline))
namespace arduino {
    enum struct tft_io_type {
        spi = 0,
        i2c = 1,
        parallel8 = 2
    };
}
