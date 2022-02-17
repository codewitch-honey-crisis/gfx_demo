#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "tft_core.hpp"

#if defined(I2C_BUFFER_LENGTH)
#define HTCW_I2C_WIRE_MAX (I2C_BUFFER_LENGTH>256?256:I2C_BUFFER_LENGTH) ///< Particle or similar Wire lib
#elif defined(BUFFER_LENGTH)
#define HTCW_I2C_WIRE_MAX (BUFFER_LENGTH>256?256:BUFFER_LENGTH) ///< AVR or similar Wire lib
#elif defined(SERIAL_BUFFER_SIZE)
#define HTCW_I2C_WIRE_MAX (SERIAL_BUFFER_SIZE>255?255:SERIAL_BUFFER_SIZE-1)
#else
#define HTCW_I2C_WIRE_MAX 32 ///< Use common Arduino core default
#endif
namespace arduino {
    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        uint8_t PinSda,
        uint8_t PinScl,
#endif
        uint32_t I2CSpeed=0,
        uint32_t I2CDefaultSpeed=I2CSpeed,
        uint32_t I2CInitSpeed=I2CSpeed,
        size_t I2CBufferSize = HTCW_I2C_WIRE_MAX>
    struct tft_i2c {
        constexpr static const uint8_t i2c_host = I2CHost;
        constexpr static const tft_io_type type = tft_io_type::i2c;
        constexpr static const bool readable = true;
        constexpr static const size_t dma_size = 0;
        constexpr static const uint8_t dma_channel = 0;
        
        constexpr static const uint32_t i2c_speed = I2CSpeed;
        constexpr static const uint32_t i2c_default_speed = I2CDefaultSpeed;
        constexpr static const uint32_t i2c_init_speed = I2CInitSpeed;
        constexpr static const size_t i2c_buffer_size = I2CBufferSize;
    private:
        inline static TwoWire& i2c() FORCE_INLINE {
#if defined(ESP32)
            return ii2c;
#else
#if WIRE_INTERFACES_COUNT > 1
            if(I2CHost==1) {
                return Wire1;
            }
#endif
#if WIRE_INTERFACES_COUNT > 2
            if(I2CHost==2) {
                return Wire2;
            }
#endif
#if WIRE_INTERFACES_COUNT > 3
            if(I2CHost==3) {
                return Wire3;
            }
#endif
#if WIRE_INTERFACES_COUNT > 4
            if(I2CHost==4) {
                return Wire4;
            }
#endif
#if WIRE_INTERFACES_COUNT > 5
            if(I2CHost==5) {
                return Wire5;
            }
#endif
            
#endif
            return Wire;
        }
    public:
#ifdef ESP32
        static TwoWire ii2c;
#endif
        static uint8_t address;
        static uint8_t payload;
        static bool is_init;
        static bool initialize() {
            is_init =false;
#ifdef ASSIGNABLE_I2C_PINS
            i2c().begin(PinSda,PinScl);
#else // !ASSIGNABLE_I2C_PINS
            i2c().begin(i2c_host);
#endif // !ASSIGNABLE_I2C_PINS
            i2c().setClock(i2c_default_speed);
            return true;
                
        }
        inline static void begin_initialization() FORCE_INLINE { is_init=true;}
        inline static void end_initialization() FORCE_INLINE { is_init=false; }
        
        static void deinitialize() {
            if(is_init) {
                i2c().setClock(i2c_speed);
            }
        }
        inline static void set_address(uint8_t addr) FORCE_INLINE {
            address = addr;
        }
        inline static void set_command(uint8_t set_payload) FORCE_INLINE {
            payload = set_payload;
        }
        inline static void set_data(uint8_t set_payload) FORCE_INLINE {
            payload = set_payload;
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
            i2c().beginTransmission(address);
            i2c().write(payload);
            i2c().write(value);
            i2c().endTransmission(true);
        }
        static void write_raw8_repeat(uint16_t value, size_t count) {
            i2c().beginTransmission(address);
            i2c().write(payload);
            size_t buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(value);
                ++buf_count;
            }
            i2c().endTransmission(true);
        }
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
            i2c().beginTransmission(address);
            i2c().write(payload);
            i2c().write(uint8_t((value>>8)&0xFF));
            i2c().write(uint8_t(value&0xFF));
            i2c().endTransmission(true);
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
            i2c().beginTransmission(address);
            i2c().write(payload);
            size_t buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t((value>>8)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t(value&0xFF));
                ++buf_count;
            }
            i2c().endTransmission(true);
        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
            i2c().beginTransmission(address);
            i2c().write(payload);
            i2c().write(uint8_t((value>>24)&0xFF));
            i2c().write(uint8_t((value>>16)&0xFF));
            i2c().write(uint8_t((value>>8)&0xFF));
            i2c().write(uint8_t(value&0xFF));
            i2c().endTransmission(true);
        }
        static void write_raw32_repeat(uint32_t value, size_t count) {
            i2c().beginTransmission(address);
            i2c().write(payload);
            size_t buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t((value>>24)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t((value>>16)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t((value>>8)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(uint8_t(value&0xFF));
                ++buf_count;
            }
            i2c().endTransmission(true);
        }
        static uint8_t read_raw8() {
            i2c().beginTransmission(address);
            i2c().write(payload);
            uint8_t b=0;
            i2c().readBytes(&b,1);
            i2c().endTransmission(true);
            return b;
        }
        static void write_raw(const uint8_t* data, size_t length) {
            if(0==length) return;           
            i2c().beginTransmission(address);
            i2c().write(payload);
            size_t buf_count = 1;
            while (length--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(*data);
                ++data;
                ++buf_count;
            }
            i2c().endTransmission(true);
        }
        static void write_raw_pgm(const uint8_t* data, size_t length) {
            if(0==length) return;
            i2c().beginTransmission(address);
            i2c().write(payload);
            size_t buf_count = 1;
            while (length--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c().endTransmission(true);
                    i2c().beginTransmission(address);
                    i2c().write(payload);
                    buf_count = 1;
                }
                i2c().write(pgm_read_byte(data));
                ++data;
                ++buf_count;
            }
            i2c().endTransmission(true);
        }
        static inline void begin_write() FORCE_INLINE {
            i2c().setClock(is_init?i2c_init_speed:i2c_speed);
        }
        static inline void end_write() FORCE_INLINE {
            i2c().setClock(i2c_default_speed);
        }
        
        static inline void begin_read() FORCE_INLINE {
        }
        static inline void end_read() FORCE_INLINE {
        }
        
        static inline void direction(uint8_t direction) FORCE_INLINE {
        }
      
        inline static void cs_low() FORCE_INLINE {
        }
        inline static void cs_high() FORCE_INLINE {
        }
        
    private:
    };
    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        uint8_t PinSda,
        uint8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        uint32_t I2CSpeed,
        uint32_t I2CDefaultSpeed,
        uint32_t I2CInitSpeed,
        size_t I2CBufferSize>
        uint8_t tft_i2c<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CSpeed,I2CDefaultSpeed,I2CInitSpeed,I2CBufferSize>
    ::address = 0;
    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        uint8_t PinSda,
        uint8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        uint32_t I2CSpeed,
        uint32_t I2CDefaultSpeed,
        uint32_t I2CInitSpeed,
        size_t I2CBufferSize>
        uint8_t tft_i2c<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CSpeed,I2CDefaultSpeed,I2CInitSpeed,I2CBufferSize>
    ::payload = 0;
#if defined(ESP32)
        template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        uint8_t PinSda,
        uint8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        uint32_t I2CSpeed,
        uint32_t I2CDefaultSpeed,
        uint32_t I2CInitSpeed,
        size_t I2CBufferSize>
        TwoWire tft_i2c<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CSpeed,I2CDefaultSpeed,I2CInitSpeed,I2CBufferSize>
    ::ii2c(i2c_host);
#endif
        template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        uint8_t PinSda,
        uint8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        uint32_t I2CSpeed,
        uint32_t I2CDefaultSpeed,
        uint32_t I2CInitSpeed,
        size_t I2CBufferSize>
        bool tft_i2c<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CSpeed,I2CDefaultSpeed,I2CInitSpeed,I2CBufferSize>
    ::is_init = false;
}
#undef HTCW_I2C_WIRE_MAX