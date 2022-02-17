#pragma once
#include <Arduino.h>
#include "tft_io.hpp"
namespace arduino {
    template<int8_t PinDC, int8_t PinRst, int8_t PinBL, typename Bus, int16_t SoftResetCommand = 0x01,uint8_t Address=0,uint8_t CommandPayload=0x00,uint8_t DataPayload=0x40>
    struct tft_driver {
        static_assert(Bus::type!=tft_io_type::i2c || Address!=0,"I2C must have a valid address");
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_bl = PinBL;
        constexpr static const int16_t swrst_command = SoftResetCommand;
        constexpr static const uint8_t address = Address;
        constexpr static const uint8_t command_payload = CommandPayload;
        constexpr static const uint8_t data_payload = DataPayload;
        using bus = Bus;
        static bool initialize() {
            if(bus::initialize()) {
                bus::set_address(address);
                bus::set_data(data_payload);
                if(pin_dc>-1) {
                    pinMode(pin_dc, OUTPUT);
                    digitalWrite(pin_dc, HIGH);
                }
                if(pin_rst>-1) {
                    pinMode(pin_rst, OUTPUT);
                    // TODO: is this necessary? Should it be LOW?
                    digitalWrite(pin_rst, HIGH);
                }
                reset();
                return true;
            }
            return false;
        }
        static void deinitialize() {
            bus::deinitialize();
        }
        static void reset() {
            if(pin_rst > -1) {
                digitalWrite(pin_rst, HIGH);
                delay(5);
                digitalWrite(pin_rst, LOW);
                delay(20);
                digitalWrite(pin_rst, HIGH);
            } else {
                if(swrst_command!=-1) {
                    send_command(swrst_command&0xFF);
                }
            }
            // wait for reset to complete
            delay(150);

        }
        static void send_command(uint8_t command) {
            bus::begin_write();
            dc_command();
            bus::write_raw8(command);
            dc_data();
            bus::end_write();
        }
        static void send_command(const uint8_t *data, size_t length) {
            bus::begin_write();
            dc_command();
            bus::write_raw(data,length);
            dc_data();
            bus::end_write();
        }
        static void send_command_pgm(const uint8_t *data,size_t length) {
            bus::begin_write();
            dc_command();
            bus::write_raw_pgm(data,length);
            dc_data();
            bus::end_write();
        }
        static void send_data_pgm(const uint8_t *data,size_t length) {
            bus::begin_write();
            dc_data();
            bus::write_raw_pgm(data,length);
            dc_data();
            bus::cs_low(); // allow more hold time for low VDI rail
            bus::end_write();
        }
        static void send_data8(uint8_t data) {
            bus::begin_write();
            dc_data();
            bus::write_raw8(data);
            dc_data();
            bus::cs_low(); // allow more hold time for low VDI rail
            bus::end_write();
        }
        static void send_data16(uint16_t data) {
            bus::begin_write();
            dc_data();
            bus::write_raw16(data);
            dc_data();
            bus::cs_low(); // allow more hold time for low VDI rail
            bus::end_write();
        }
        static void send_data(const uint8_t *data,size_t length) {
            bus::begin_write();
            dc_data();
            bus::write_raw(data,length);
            dc_data();
            bus::cs_low(); // allow more hold time for low VDI rail
            bus::end_write();
        }
        static uint8_t recv_command8(uint8_t command, uint8_t index) {
            uint8_t result = 0;
            send_command(command);
            bus::direction(INPUT);
            bus::cs_low();
            while(index--) result = bus::read_raw8();
            bus::direction(OUTPUT);
            bus::cs_high();
            return result;
        }
        static uint8_t recv_command16(uint8_t command, uint8_t index) {
            uint8_t result = recv_command8(command, index) << 8;
            result |= recv_command8(command, index);
            return result;
        }
        static uint8_t recv_command32(uint8_t command, uint8_t index) {
            uint8_t result = recv_command8(command, index) << 24;
            result |= recv_command8(command, index) << 16;
            result |= recv_command8(command, index) << 8;
            result |= recv_command8(command, index);
            return result;
        }
        inline static void dc_command() FORCE_INLINE {
            bus::set_command(command_payload);
            if(pin_dc>-1) {
#if defined(OPTIMIZE_ESP32) 
                if(pin_dc>31) {
                    GPIO.out1_w1tc.val = (1 << ((pin_dc - 32)&31));
                } else if(pin_dc>-1) {
                    GPIO.out_w1tc = (1 << (pin_dc&31));
                }
#else // !OPTIMIZE_ESP32
                digitalWrite(pin_dc,LOW);
#endif // !OPTIMIZE_ESP32
            }
        }
        inline static void dc_data() FORCE_INLINE {
            bus::set_data(data_payload);
            if(pin_dc>-1) {
#if defined(OPTIMIZE_ESP32) 
                if(pin_dc>31) {
                    GPIO.out1_w1ts.val = (1 << ((pin_dc - 32)&31));
                } else if(pin_dc>-1) {
                    GPIO.out_w1ts = (1 << (pin_dc &31));
                }
#else // !OPTIMIZE_ESP32
                digitalWrite(pin_dc,HIGH);
#endif // !OPTIMIZE_ESP32
            }
        }
    };
}