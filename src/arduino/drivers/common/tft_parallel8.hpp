#pragma once
#if !defined (SUPPORT_TRANSACTIONS)
  #define SUPPORT_TRANSACTIONS
#endif
#include <Arduino.h>
#include "tft_core.hpp"
// The parallel code only works on the ESP32. The generic code for Arduino that is in place is non-functional in my tests. I'm not sure why, but it might be a timing issue.

#define FORCE_INLINE __attribute((always_inline))
namespace arduino {
    template<int8_t PinCS, 
        int8_t PinWR, 
        int8_t PinRD,
        int8_t PinD0,
        int8_t PinD1,
        int8_t PinD2,
        int8_t PinD3,
        int8_t PinD4,
        int8_t PinD5,
        int8_t PinD6,
        int8_t PinD7>
    struct tft_p8 {
        constexpr static const tft_io_type type = tft_io_type::parallel8;
        constexpr static const bool readable = true;
        constexpr static const size_t dma_size = 0;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_wr = PinWR;
        constexpr static const int8_t pin_rd = PinRD;
        constexpr static const int8_t pin_d0 = PinD0;
        constexpr static const int8_t pin_d1 = PinD1;
        constexpr static const int8_t pin_d2 = PinD2;
        constexpr static const int8_t pin_d3 = PinD3;
        constexpr static const int8_t pin_d4 = PinD4;
        constexpr static const int8_t pin_d5 = PinD5;
        constexpr static const int8_t pin_d6 = PinD6;
        constexpr static const int8_t pin_d7 = PinD7;
#ifdef OPTIMIZE_ESP32
    private:
        constexpr static const bool has_data_low_pins = (pin_d0>=0 && pin_d0<32) || 
                                                        (pin_d1>=0 && pin_d1<32) ||
                                                        (pin_d2>=0 && pin_d2<32) ||
                                                        (pin_d3>=0 && pin_d3<32) ||
                                                        (pin_d4>=0 && pin_d4<32) ||
                                                        (pin_d5>=0 && pin_d5<32) ||
                                                        (pin_d6>=0 && pin_d6<32) ||
                                                        (pin_d7>=0 && pin_d7<32);
        constexpr static const bool has_data_high_pins = (pin_d0>31) || 
                                                        (pin_d1>31) || 
                                                        (pin_d2>31) || 
                                                        (pin_d3>31) || 
                                                        (pin_d4>31) || 
                                                        (pin_d5>31) || 
                                                        (pin_d6>31) || 
                                                        (pin_d7>31);
        constexpr static uint32_t get_dir_mask_low() {
            uint32_t result = 0;
            if(pin_d0>-1 && pin_d0<32) {
                result |= 1<<pin_d0;
            }
            if(pin_d1>-1 && pin_d1<32) {
                result |= 1<<pin_d1;
            }
            if(pin_d2>-1 && pin_d2<32) {
                result |= 1<<pin_d2;
            }
            if(pin_d3>-1 && pin_d3<32) {
                result |= 1<<pin_d3;
            }
            if(pin_d4>-1 && pin_d4<32) {
                result |= 1<<pin_d4;
            }
            if(pin_d5>-1 && pin_d5<32) {
                result |= 1<<pin_d5;
            }
            if(pin_d6>-1 && pin_d6<32) {
                result |= 1<<pin_d6;
            }
            if(pin_d7>-1 && pin_d7<32) {
                result |= 1<<pin_d7;
            }
            return result;
        }
        constexpr static uint32_t get_dir_mask_high() {
            uint32_t result = 0;
            if(pin_d0>31) {
                result |= 1<<((pin_d0-32)&31);
            }
            if(pin_d1>31) {
                result |= 1<<((pin_d1-32)&31);
            }
            if(pin_d2>31) {
                result |= 1<<((pin_d2-32)&31);
            }
            if(pin_d3>31) {
                result |= 1<<((pin_d3-32)&31);
            }
            if(pin_d4>31) {
                result |= 1<<((pin_d4-32)&31);
            }
            if(pin_d5>31) {
                result |= 1<<((pin_d5-32)&31);
            }
            if(pin_d6>31) {
                result |= 1<<((pin_d6-32)&31);
            }
            if(pin_d7>31) {
                result |= 1<<((pin_d7-32)&31);
            }
            return result;
        }
        
        constexpr static const uint32_t dir_mask_low = get_dir_mask_low();
        constexpr static const uint32_t dir_mask_high = get_dir_mask_high();
        
        inline static uint32_t clr_mask_low() FORCE_INLINE {
            if(pin_wr>31) {
                uint32_t result = dir_mask_low;
                wr_low();
                return result;
            } else if(pin_wr>-1) {
                return (dir_mask_low | 1<<pin_wr);
            }
            return 0;
        }
        inline static uint32_t clr_mask_high() FORCE_INLINE {
            if(pin_wr>31) {
                return (dir_mask_high | 1<<((pin_wr-32)&31));
            } else if(pin_wr>-1) {
                uint32_t result = dir_mask_high;
                wr_low();
                return result;
            }
            return 0;
        }
        static uint32_t xset_mask_low[256*has_data_low_pins];
        static uint32_t xset_mask_high[256*has_data_high_pins];
#endif // OPTIMIZE_ESP32
    public:
        static bool initialize() {
            if(pin_cs > -1) {
                pinMode(pin_cs,OUTPUT);
                digitalWrite(pin_cs,HIGH);
            }
            if(pin_wr > -1) {
                pinMode(pin_wr,OUTPUT);
                digitalWrite(pin_wr,HIGH);
            }
            if(pin_rd > -1) {
                pinMode(pin_rd,OUTPUT);
                digitalWrite(pin_rd,HIGH);
            }
            if(pin_d0 > -1) {
                pinMode(pin_d0,OUTPUT);
                digitalWrite(pin_d0,HIGH);
            }
            if(pin_d1 > -1) {
                pinMode(pin_d1,OUTPUT);
                digitalWrite(pin_d1,HIGH);
            }
            if(pin_d2 > -1) {
                pinMode(pin_d2,OUTPUT);
                digitalWrite(pin_d2,HIGH);
            }
            if(pin_d3 > -1) {
                pinMode(pin_d3,OUTPUT);
                digitalWrite(pin_d3,HIGH);
            }
            if(pin_d4 > -1) {
                pinMode(pin_d4,OUTPUT);
                digitalWrite(pin_d4,HIGH);
            }
            if(pin_d5 > -1) {
                pinMode(pin_d5,OUTPUT);
                digitalWrite(pin_d5,HIGH);
            }
            if(pin_d6 > -1) {
                pinMode(pin_d6,OUTPUT);
                digitalWrite(pin_d6,HIGH);
            }
            if(pin_d7 > -1) {
                pinMode(pin_d7,OUTPUT);
                digitalWrite(pin_d7,HIGH);
            }
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                for (int32_t c = 0; c<256; c++) {
                    xset_mask_low[c] = 0;
                    if ( pin_d0>-1 && pin_d0<32 && (c & 0x01)) xset_mask_low[c] |= (1 << pin_d0);
                    if ( pin_d1>-1 && pin_d1<32 && (c & 0x02) ) xset_mask_low[c] |= (1 << pin_d1);
                    if ( pin_d2>-1 && pin_d2<32 && (c & 0x04) ) xset_mask_low[c] |= (1 << pin_d2);
                    if ( pin_d3>-1 && pin_d3<32 && (c & 0x08) ) xset_mask_low[c] |= (1 << pin_d3);
                    if ( pin_d4>-1 && pin_d4<32 && (c & 0x10) ) xset_mask_low[c] |= (1 << pin_d4);
                    if ( pin_d5>-1 && pin_d5<32 && (c & 0x20) ) xset_mask_low[c] |= (1 << pin_d5);
                    if ( pin_d6>-1 && pin_d6<32 && (c & 0x40) ) xset_mask_low[c] |= (1 << pin_d6);
                    if ( pin_d7>-1 && pin_d7<32 && (c & 0x80) ) xset_mask_low[c] |= (1 << pin_d7);
                }
            }
            if(has_data_high_pins) {
                for (int32_t c = 0; c<256; c++) {
                    xset_mask_high[c] = 0;
                    if ( pin_d0>31 && (c & 0x01)) xset_mask_high[c] |= (1 << ((pin_d0-32)&31));
                    if ( pin_d1>31 && (c & 0x02) ) xset_mask_high[c] |= (1 << ((pin_d1-32)&31));
                    if ( pin_d2>31 && (c & 0x04) ) xset_mask_high[c] |= (1 << ((pin_d2-32)&31));
                    if ( pin_d3>31 && (c & 0x08) ) xset_mask_high[c] |= (1 << ((pin_d3-32)&31));
                    if ( pin_d4>31 && (c & 0x10) ) xset_mask_high[c] |= (1 << ((pin_d4-32)&31));
                    if ( pin_d5>31 && (c & 0x20) ) xset_mask_high[c] |= (1 << ((pin_d5-32)&31));
                    if ( pin_d6>31 && (c & 0x40) ) xset_mask_high[c] |= (1 << ((pin_d6-32)&31));
                    if ( pin_d7>31 && (c & 0x80) ) xset_mask_high[c] |= (1 << ((pin_d7-32)&31));
                }
            }
#endif // OPTIMIZE_ESP32
            // Set to output once again in case ESP8266 D6 (MISO) is used for CS
            if(pin_cs>-1) {
                pinMode(pin_cs, OUTPUT);
                digitalWrite(pin_cs, HIGH); // Chip select high (inactive)
            }
            return true;
        }
        static void deinitialize() {

        }
        inline static bool initialize_dma() FORCE_INLINE { return true; }
        inline static void deinitialize_dma() FORCE_INLINE {}
        inline static void busy_check() FORCE_INLINE {
        }
        inline static void begin_transaction() FORCE_INLINE {
         
        }
        inline static void end_transaction() FORCE_INLINE {
        }
        inline static void set_speed_multiplier(float mult) FORCE_INLINE {
            
        }
        static uint8_t read_raw8() {
            rd_low();
            uint8_t b = 0xAA;
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins && has_data_high_pins) {
                
                // 3x for bus access stabilization
                uint32_t pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                uint32_t pins_h = gpio_input_get_high();
                if(pin_d0>31) {
                    b = (((pins_h>>((pin_d0-32)&31))&1)<<0);
                } else if(pin_d0>-1) {
                    b = (((pins_l>>(pin_d0))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>31) {
                    b |= (((pins_h>>((pin_d1-32)&31))&1)<<1);
                } else if(pin_d1>-1) {
                    b |= (((pins_l>>(pin_d1))&1)<<1);
                }
                if(pin_d2>31) {
                    b |= (((pins_h>>((pin_d2-32)&31))&1)<<2);
                } else if(pin_d2>-1) {
                    b |= (((pins_l>>(pin_d2))&1)<<2);
                }
                if(pin_d3>31) {
                    b |= (((pins_h>>((pin_d3-32)&31))&1)<<3);
                } else if(pin_d3>-1) {
                    b |= (((pins_l>>(pin_d3))&1)<<3);
                }
                if(pin_d4>31) {
                    b |= (((pins_h>>((pin_d4-32)&31))&1)<<4);
                } else if(pin_d4>-1) {
                    b |= (((pins_l>>((pin_d4)&31))&1)<<4);
                }
                if(pin_d5>31) {
                    b |= (((pins_h>>((pin_d5-32)&31))&1)<<5);
                } else if(pin_d5>-1) {
                    b |= (((pins_l>>(pin_d5))&1)<<5);
                }
                if(pin_d6>31) {
                    b |= (((pins_h>>((pin_d6-32)&31))&1)<<6);
                } else if(pin_d6>-1) {
                    b |= (((pins_l>>(pin_d6))&1)<<6);
                }
                if(pin_d7>31) {
                    b |= (((pins_h>>((pin_d7-32)&31))&1)<<7);
                } else if(pin_d7>-1) {
                    b |= (((pins_l>>(pin_d7))&1)<<7);
                }
            } else if(has_data_low_pins) {
                uint32_t pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                if(pin_d0>-1) {
                    b = (((pins_l>>(pin_d0))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>-1) {
                    b |= (((pins_l>>(pin_d1))&1)<<1);
                }
                if(pin_d2>-1) {
                    b |= (((pins_l>>(pin_d2))&1)<<2);
                }
                if(pin_d3>-1) {
                    b |= (((pins_l>>(pin_d3))&1)<<3);
                }
                if(pin_d4>-1) {
                    b |= (((pins_l>>(pin_d4))&1)<<4);
                }
                if(pin_d5>-1) {
                    b |= (((pins_l>>(pin_d5))&1)<<5);
                }
                if(pin_d6>-1) {
                    b |= (((pins_l>>(pin_d6))&1)<<6);
                }
                if(pin_d7>-1) {
                    b |= (((pins_l>>(pin_d7))&1)<<7);
                }
            } else {
                uint32_t pins_h = gpio_input_get_high();
                pins_h = gpio_input_get_high();
                pins_h = gpio_input_get_high();
                if(pin_d0>-1) {
                    b = (((pins_h>>((pin_d0-32)&31))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>-1) {
                    b |= (((pins_h>>((pin_d1-32)&31))&1)<<1);
                }
                if(pin_d2>-1) {
                    b |= (((pins_h>>((pin_d2-32)&31))&1)<<2);
                }
                if(pin_d3>-1) {
                    b |= (((pins_h>>((pin_d3-32)&31))&1)<<3);
                }
                if(pin_d4>-1) {
                    b |= (((pins_h>>((pin_d4-32)&31))&1)<<4);
                }
                if(pin_d5>-1) {
                    b |= (((pins_h>>((pin_d5-32)&31))&1)<<5);
                }
                if(pin_d6>-1) {
                    b |= (((pins_h>>((pin_d6-32)&31))&1)<<6);
                }
                if(pin_d7>-1) {
                    b |= (((pins_h>>((pin_d7-32)&31))&1)<<7);
                }
            }
#else // !OPTIMIZE_ESP32
            if(pin_d0>-1) {
                b=digitalRead(pin_d0);
            } else {
                b=0;
            }
            if(pin_d1>-1) {
                b|=digitalRead(pin_d1)<<1;
            }
            if(pin_d2>-1) {
                b|=digitalRead(pin_d2)<<2;
            }
            if(pin_d3>-1) {
                b|=digitalRead(pin_d3)<<3;
            }
            if(pin_d4>-1) {
                b|=digitalRead(pin_d4)<<4;
            }
            if(pin_d5>-1) {
                b|=digitalRead(pin_d5)<<5;
            }
            if(pin_d6>-1) {
                b|=digitalRead(pin_d6)<<6;
            }
            if(pin_d7>-1) {
                b|=digitalRead(pin_d7)<<7;
            }
#endif // !OPTIMIZE_ESP32
            rd_high();
            return b;
        }
        static void read_raw(uint8_t* buffer,size_t size) {
            while(size--) {
                *buffer++=read_raw8();
            }
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[value];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[value];
            } 
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_d0,value&1);
            digitalWrite(pin_d1,(value>>1)&1);
            digitalWrite(pin_d2,(value>>2)&1);
            digitalWrite(pin_d3,(value>>3)&1);
            digitalWrite(pin_d4,(value>>4)&1);
            digitalWrite(pin_d5,(value>>5)&1);
            digitalWrite(pin_d6,(value>>6)&1);
            digitalWrite(pin_d7,(value>>7)&1);
#endif // !OPTIMIZE_ESP32
            wr_high();
        }
        inline static void begin_initialization() FORCE_INLINE {}
        inline static void end_initialization() FORCE_INLINE {}
        static inline void set_address(uint8_t address) FORCE_INLINE {}
        inline static void set_command(uint8_t payload) FORCE_INLINE {}
        inline static void set_data(uint8_t payload) FORCE_INLINE {}
        inline static bool dma_busy() FORCE_INLINE { return false; }
        inline static void dma_wait() FORCE_INLINE { }
        static void write_raw8_repeat(uint8_t value, size_t count) {
            write_raw8(value);
            while(--count) {
                wr_low(); wr_high();
            }
        }
        static void write_raw(const uint8_t* buffer, size_t length) {
            if(length) {
#if 1
                uint8_t old=*buffer;
                write_raw8(*buffer++);
                while(--length) {
                    if(old==*buffer) {
                        wr_low(); wr_high();
                    } else {
                        write_raw8(*buffer);
                    }
                    old = *buffer++;
                }
#else
                while(length--) {
                    write_raw8(*buffer++);
                }
#endif
            }
        }
        
        static void write_raw_pgm(const uint8_t* buffer, size_t length) {
            if(length) {
#if 1
                uint8_t old=*buffer;
                write_raw8(pgm_read_byte(buffer++));
                while(--length) {
                    if(old==*buffer) {
                        wr_low(); wr_high();
                    } else {
                        write_raw8(pgm_read_byte(buffer++));
                    }
                    old = *buffer++;
                }
#else
                while(length--) {
                    write_raw8(pgm_read_byte(buffer++));
                }
#endif
            }
        }
        inline static void write_raw_dma(const uint8_t* data,size_t length) FORCE_INLINE {
            write_raw(data,length);
        }
        
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
            uint8_t b = uint8_t(value >> 8);
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=value;
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_d0,b&1);
            digitalWrite(pin_d1,(b>>1)&1);
            digitalWrite(pin_d2,(b>>2)&1);
            digitalWrite(pin_d3,(b>>3)&1);
            digitalWrite(pin_d4,(b>>4)&1);
            digitalWrite(pin_d5,(b>>5)&1);
            digitalWrite(pin_d6,(b>>6)&1);
            digitalWrite(pin_d7,(b>>7)&1);
            wr_high();
            b=value;
            digitalWrite(pin_d0,b&1);
            digitalWrite(pin_d1,(b>>1)&1);
            digitalWrite(pin_d2,(b>>2)&1);
            digitalWrite(pin_d3,(b>>3)&1);
            digitalWrite(pin_d4,(b>>4)&1);
            digitalWrite(pin_d5,(b>>5)&1);
            digitalWrite(pin_d6,(b>>6)&1);
            digitalWrite(pin_d7,(b>>7)&1);
#endif // !OPTIMIZE_ESP32
            wr_high();
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
            write_raw16(value);
            if((value >> 8) == (value & 0x00FF)) {
                while(--count) {
                    wr_low(); wr_high();
                    wr_low(); wr_high();
                }
            } else {
                while(--count) {
                    write_raw16(value);
                }
            }
        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            uint8_t b = uint8_t(value >> 24);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=uint8_t(value>>16);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b = uint8_t(value >> 8);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=value;
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
#else // !OPTIMIZE_ESP32
            write_raw16(value>>16);
            write_raw16(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw32_repeat(uint32_t value, size_t count) {
            while(count--) {
                write_raw32(value);
            }
        }
        inline static void pin_mode(int8_t pin, uint8_t mode) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin<32) {
                if(mode==INPUT) {
                    GPIO.enable_w1tc = ((uint32_t)1 << pin);
                } else {
                    GPIO.enable_w1ts = ((uint32_t)1 << pin);
                }
            } else {
                if(mode==INPUT) {
                    GPIO.enable1_w1tc.val = ((uint32_t)1 << ((pin-32)&31));
                } else {
                    GPIO.enable1_w1ts.val = ((uint32_t)1 << ((pin-32)&31));
                }
            }
            ESP_REG(DR_REG_IO_MUX_BASE + esp32_gpioMux[pin].reg) // Register lookup
                = ((uint32_t)2 << FUN_DRV_S)                        // Set drive strength 2
                | (FUN_IE)                                          // Input enable
                | ((uint32_t)2 << MCU_SEL_S);                       // Function select 2
            GPIO.pin[pin].val = 1;  
#else // !OPTIMIZE_ESP32
        if(mode==INPUT) {
            digitalWrite(pin,LOW);
            pinMode(pin,INPUT);
        } else {
            digitalWrite(pin,HIGH);
            pinMode(pin,OUTPUT);
        }
#endif // !OPTIMIZE_ESP32
        }
        inline static void begin_write() FORCE_INLINE {
            cs_low();
        }
        inline static void end_write() FORCE_INLINE {
            cs_high();
        }
        inline static void begin_read() FORCE_INLINE {
            cs_low();
        }
        inline static void end_read() FORCE_INLINE {
            cs_high();
        }
        static void direction(uint8_t mode) {
            pin_mode(pin_d0,mode);
            pin_mode(pin_d1,mode);
            pin_mode(pin_d2,mode);
            pin_mode(pin_d3,mode);
            pin_mode(pin_d4,mode);
            pin_mode(pin_d5,mode);
            pin_mode(pin_d6,mode);
            pin_mode(pin_d7,mode);
        }
        inline static void cs_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1tc = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void cs_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1ts = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
        
        inline static void wr_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_wr>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_wr - 32)&31));
            } else if(pin_wr>-1) {
                GPIO.out_w1tc = (1 << (pin_wr&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_wr,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void wr_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_wr>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_wr - 32)&31));
            } else if(pin_wr>-1) {
                GPIO.out_w1ts = (1 << (pin_wr&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_wr,HIGH);
#endif // !OPTIMIZE_ESP32
        }

        inline static void rd_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_rd>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_rd - 32)&31));
            } else if(pin_rd>-1) {
                GPIO.out_w1tc = (1 << (pin_rd &31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_rd,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void rd_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_rd>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_rd - 32)&31));
            } else if(pin_rd>-1) {
                GPIO.out_w1ts = (1 << (pin_rd&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_rd,HIGH);
#endif // !OPTIMIZE_ESP32
        }

    };
#ifdef OPTIMIZE_ESP32
    template<int8_t PinCS, 
            int8_t PinWR, 
            int8_t PinRD,
            int8_t PinD0,
            int8_t PinD1,
            int8_t PinD2,
            int8_t PinD3,
            int8_t PinD4,
            int8_t PinD5,
            int8_t PinD6,
            int8_t PinD7> 
            uint32_t tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::xset_mask_low[256*tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::has_data_low_pins];
    template<int8_t PinCS, 
            int8_t PinWR, 
            int8_t PinRD,
            int8_t PinD0,
            int8_t PinD1,
            int8_t PinD2,
            int8_t PinD3,
            int8_t PinD4,
            int8_t PinD5,
            int8_t PinD6,
            int8_t PinD7> 
            uint32_t tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::xset_mask_high[256*tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::has_data_high_pins];
#endif // OPTIMIZE_ESP32
}