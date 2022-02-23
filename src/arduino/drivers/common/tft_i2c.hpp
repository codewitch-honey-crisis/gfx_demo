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
        int8_t PinSda=-1,
        int8_t PinScl=-1,
#endif
        size_t I2CBufferSize = HTCW_I2C_WIRE_MAX>
    struct tft_i2c_ex {
        constexpr static const uint8_t i2c_host = I2CHost;
        constexpr static const tft_io_type type = tft_io_type::i2c;
        constexpr static const bool readable = true;
        constexpr static const size_t dma_size = 0;
        constexpr static const uint8_t dma_channel = 0;
        constexpr static const uint32_t base_speed = 100*1000;
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
        static uint32_t speed;
        static uint8_t address;
        static uint8_t payload;
        static bool is_init;
        static bool initialize() {
            speed = base_speed;
            is_init =false;
#ifdef ASSIGNABLE_I2C_PINS
            i2c().begin(PinSda,PinScl);
#else // !ASSIGNABLE_I2C_PINS
            i2c().begin(i2c_host);
#endif // !ASSIGNABLE_I2C_PINS
            i2c().setClock(base_speed);
            return true;
                
        }
        inline static void begin_initialization() FORCE_INLINE { is_init=true;}
        inline static void end_initialization() FORCE_INLINE { is_init=false; }
        
        static void deinitialize() {
            if(is_init) {
                i2c().setClock(base_speed);
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
        static void read_raw(uint8_t* buffer,size_t size) {
            while(size--) {
                *buffer++=read_raw8();
            }
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
            i2c().setClock(speed);
        }
        static inline void end_write() FORCE_INLINE {
            i2c().setClock(base_speed);
        }
        
        static inline void begin_read() FORCE_INLINE {
            i2c().setClock(speed);
        }
        static inline void end_read() FORCE_INLINE {
            i2c().setClock(base_speed);
        }
        
        static inline void direction(uint8_t direction) FORCE_INLINE {
        }
      
        inline static void cs_low() FORCE_INLINE {
        }
        inline static void cs_high() FORCE_INLINE {
        }
        inline static void set_speed_multiplier(float mult) {
            speed = mult*base_speed;
        }
    private:
    };
    template<uint8_t I2CHost,
        size_t I2CBufferSize
    >
#ifdef ASSIGNABLE_I2C_PINS
    using tft_i2c = tft_i2c_ex<I2CHost,-1,-1,I2CBufferSize>;
#else
    using tft_i2c = tft_i2c_ex<I2CHost,I2CBufferSize>;
#endif

    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        int8_t PinSda,
        int8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        size_t I2CBufferSize>
        uint8_t tft_i2c_ex<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CBufferSize>
    ::address = 0;
    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        int8_t PinSda,
        int8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        size_t I2CBufferSize>
        uint8_t tft_i2c_ex<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CBufferSize>
    ::payload = 0;
#if defined(ESP32)
        template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        int8_t PinSda,
        int8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        size_t I2CBufferSize>
        TwoWire tft_i2c_ex<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CBufferSize>
    ::ii2c(i2c_host);
#endif
    template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        int8_t PinSda,
        int8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        size_t I2CBufferSize>
        uint32_t tft_i2c_ex<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CBufferSize>
    ::speed = base_speed;
        template<uint8_t I2CHost,
#ifdef ASSIGNABLE_I2C_PINS
        int8_t PinSda,
        int8_t PinScl,
#endif // ASSIGNABLE_SPI_PINS
        size_t I2CBufferSize>
        bool tft_i2c_ex<I2CHost
#ifdef ASSIGNABLE_I2C_PINS
        ,PinSda,PinScl
#endif // ASSIGNABLE_SPI_PINS
        ,I2CBufferSize>
    ::is_init = false;
}
#undef HTCW_I2C_WIRE_MAX