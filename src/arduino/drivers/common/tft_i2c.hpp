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
    template<uint8_t I2CHost,uint8_t Address,
        uint8_t PinSda,
        uint8_t PinScl,
        uint8_t PayloadCommand = 0x0,
        uint8_t PayloadData = 0x40,
        uint32_t I2CSpeed=0,
        size_t I2CBufferSize = HTCW_I2C_WIRE_MAX>
    struct tft_i2c {
        constexpr static const uint8_t i2c_host = I2CHost;
        constexpr static const tft_io_type type = tft_io_type::i2c;
        constexpr static const bool readable = true;
        constexpr static const size_t dma_size = 0;
        constexpr static const uint8_t dma_channel = 0;
        constexpr static const uint8_t payload_command = PayloadCommand;
        constexpr static const uint8_t payload_data = PayloadData;
        
        constexpr static const uint8_t i2c_address = Address;
        constexpr static const uint32_t i2c_speed = I2CSpeed;
        constexpr static const size_t i2c_buffer_size = I2CBufferSize;
    private:
        
    public:
        static TwoWire i2c;
        static uint8_t payload;
        static bool initialize() {
            if(i2c.begin(PinSda,PinScl,i2c_speed)) {
                set_data();
                return true;
            }
            return false;

        }
        static void deinitialize() {
        }
        inline static void set_command() FORCE_INLINE {
            payload = payload_command;
        }
        inline static void set_data() FORCE_INLINE {
            payload = payload_data;
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            i2c.write(value);
            i2c.endTransmission(true);
        }
        static void write_raw8_repeat(uint16_t value, size_t count) {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            int buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(value);
                ++buf_count;
            }
            i2c.endTransmission(true);
        }
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            i2c.write(uint8_t((value>>8)&0xFF));
            i2c.write(uint8_t(value&0xFF));
            i2c.endTransmission(true);
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            int buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t((value>>8)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t(value&0xFF));
                ++buf_count;
            }
            i2c.endTransmission(true);
        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            i2c.write(uint8_t((value>>24)&0xFF));
            i2c.write(uint8_t((value>>16)&0xFF));
            i2c.write(uint8_t((value>>8)&0xFF));
            i2c.write(uint8_t(value&0xFF));
            i2c.endTransmission(true);
        }
        static void write_raw32_repeat(uint32_t value, size_t count) {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            int buf_count = 1;
            while (count--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t((value>>24)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t((value>>16)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t((value>>8)&0xFF));
                ++buf_count;
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(uint8_t(value&0xFF));
                ++buf_count;
            }
            i2c.endTransmission(true);
        }
        static uint8_t read_raw8() {
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            uint8_t b=0;
            i2c.readBytes(&b,1);
            i2c.endTransmission(true);
            return b;
        }
        static void write_raw(const uint8_t* data, size_t length) {
            if(0==length) return;           
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            int buf_count = 1;
            while (length--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(*data);
                ++data;
                ++buf_count;
            }
            i2c.endTransmission(true);
        }
        static void write_raw_pgm(const uint8_t* data, size_t length) {
            if(0==length) return;
            i2c.beginTransmission(i2c_address);
            i2c.write(payload);
            int buf_count = 1;
            while (length--) {
                if (buf_count >= i2c_buffer_size) {
                    i2c.endTransmission(true);
                    i2c.beginTransmission(i2c_address);
                    i2c.write(payload);
                    buf_count = 1;
                }
                i2c.write(pgm_read_byte(data));
                ++data;
                ++buf_count;
            }
            i2c.endTransmission(true);
        }
        static inline void begin_write() FORCE_INLINE {
            
        }
        static inline void end_write() FORCE_INLINE {

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
        inline static void sclk_low() FORCE_INLINE {
        }
        inline static void sclk_high() FORCE_INLINE {
        }
    private:
    };
    
    template<uint8_t I2CHost,
        uint8_t Address,
        uint8_t PinSda,
        uint8_t PinScl,
        uint8_t PayloadCommand,
        uint8_t PayloadData,
        uint32_t I2CSpeed,
        size_t I2CBufferSize>
        uint8_t tft_i2c<I2CHost,Address,PinSda,PinScl,PayloadCommand,PayloadData,I2CSpeed,I2CBufferSize>
    ::payload = PayloadData;

    template<uint8_t I2CHost,
        uint8_t Address,
        uint8_t PinSda,
        uint8_t PinScl,
        uint8_t PayloadCommand,
        uint8_t PayloadData,
        uint32_t I2CSpeed,
        size_t I2CBufferSize>
        TwoWire tft_i2c<I2CHost,Address,PinSda,PinScl,PayloadCommand,PayloadData,I2CSpeed,I2CBufferSize>
    ::i2c(i2c_host);

}
#undef HTCW_I2C_WIRE_MAX