#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "gfx_bitmap.hpp"
namespace arduino {
    
    namespace dep0290b_helpers {
         static const uint8_t display[] PROGMEM = {
            7,                             
            0x12,0x80,0xFE,      
            0x11,0x01,0x03,
            0x44,0x02,0x01,0x10,
            0x45,0x04,0x00,0x00,0x29,0x01,
            0x4E,0x01,0x01,      
            0x4F,0x02,0x00,0x00,
            0x24,0x00 
        };                 
            
    }
    template<int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy,
        size_t BitDepth=1>
    struct depg0290b final {
        enum struct result {
            success = 0,
            invalid_argument,
            io_error,
            io_busy,
            out_of_memory,
            timeout,
            not_supported
        };
        constexpr static const uint16_t width = 128;
        constexpr static const uint16_t height = 296;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        // the Busy pin
        constexpr static const int8_t pin_busy = PinBusy;

        constexpr static const uint32_t clock_speed = 4*1000*1000;

        constexpr static const size_t bit_depth = BitDepth;

        struct rect {
            uint16_t x1;
            uint16_t y1;
            uint16_t x2;
            uint16_t y2;
        };
        
        // GFX Bindings
        using type = depg0290b;
        using pixel_type = gfx::gsc_pixel<BitDepth>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        using frame_buffer_type = gfx::large_bitmap<pixel_type>;

    private: 
        bool m_initialized;
        SPIClass& m_spi;
        int m_suspend_count;
        frame_buffer_type m_frame_buffer;
        void send_commands(const uint8_t* commands) {
            uint8_t cmd_count, cmd, arg_count;
            uint16_t ms;
            cmd_count = pgm_read_byte(commands++); // Number of commands to follow
            while (cmd_count--) {              // For each command...
                cmd = *(commands++);       // Read command
                arg_count = *(commands++);   // Number of args to follow
                ms = arg_count & 0x80;       // If hibit set, delay follows args
                arg_count &= ~0x80;          // Mask out delay bit
                send_command(cmd);
                while(arg_count--) {
                    send_data(pgm_read_byte(commands++));
                }
                if (ms) {
                    ms = pgm_read_byte(commands++);
                    if(ms==254) {
                        ms=1000;
                    }
                    if (ms == 255)
                        ms = 1200;
                    wait_busy(ms);
                }
            }
        }
        void wait_busy(uint16_t ms) {
            if (pin_busy >= 0) {
                uint32_t ts = millis();
                while (true) {
                    if (!digitalRead(pin_busy)) {
                        return;
                    }
                    delay(1);
                    if (millis() - ts > 5000) {
                        return;
                    }
                }
            } 
            delay(ms);
        }
        void send_command(uint8_t command)
        {
            m_spi.beginTransaction(SPISettings(clock_speed, MSBFIRST, SPI_MODE0));
            if (pin_dc >= 0) {
                digitalWrite(pin_dc, LOW);
            }
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, LOW);
            }
            m_spi.transfer(command);
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, HIGH);
            }
            if (pin_dc >= 0) {
                digitalWrite(pin_dc, HIGH);
            }
            m_spi.endTransaction();
        }

        void send_data(uint8_t data)
        {
            m_spi.beginTransaction(SPISettings(clock_speed, MSBFIRST, SPI_MODE0));
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, LOW);
            }
            m_spi.transfer(data);
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, HIGH);
            }
            m_spi.endTransaction();
        }

        void display_update() {
            initialize();
            // don't draw if we're suspended
            if(0==m_suspend_count) {    
                send_commands(dep0290b_helpers::display);
                if(1==bit_depth) {
                    for (int y = 0; y < height; y++) {
                        for (int x = 0; x < width ; x+=8) {
                            uint8_t data = 0;
                            for(int i = 0;i<8;++i) {
                                pixel_type px;
                                gfx::gfx_result r=m_frame_buffer.point(gfx::point16(x+i,y),&px);
                                if(gfx::gfx_result::success!=r) {
                                    return;
                                }
                                data |= (1<<(7-i))*px.template channel<gfx::channel_name::L>();
                            }
                            send_data(data);
                        }
                    }
                } else {
                    int col, row;
                    for (int y = 0; y < height; y++) {
                        row = y & 15;
                        for (int x = 0; x < width ; x+=8) {
                            uint8_t data = 0;
                            for(int i = 0;i<8;++i) {
                                const int xx = x+i;
                                col = xx & 15;
                                pixel_type px;
                                gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                                if(gfx::gfx_result::success!=r) {
                                    return;
                                }
                                data |= (1<<(7-i))* (255.0*px.template channelr<gfx::channel_name::L>()>=gfx::helpers::dither::bayer_16[col+row*16]);
                            }
                            send_data(data);
                        }
                    }
                }
                send_command(0x20);
                wait_busy(1200);  
                // power down
                send_command(0x10);
                send_data(0x01);   
                m_initialized=false;   
            }
        }
    public:
     
        depg0290b(SPIClass& spi) : m_initialized(false),m_spi(spi),m_suspend_count(0),m_frame_buffer(gfx::size16(width,height),1) {

        }
        void reset() {
            if (pin_rst >= 0)
            {
                delay(10);
                digitalWrite(pin_rst, LOW);
                delay(10);
                digitalWrite(pin_rst, HIGH);
                delay(150);
            }
        }
        void initialize() {
            if(!m_initialized) {
                if (pin_cs >= 0) {
                    pinMode(pin_cs, OUTPUT);
                }
                if (pin_dc >= 0) {
                    pinMode(pin_dc, OUTPUT);
                }
                if (pin_rst >= 0) {
                    pinMode(pin_rst, OUTPUT);
                }
                reset();
                m_spi.begin();
                if (pin_busy >= 0) pinMode(pin_busy, INPUT);
                if (pin_rst >= 0) {
                    pinMode(pin_rst, OUTPUT);
                    digitalWrite(pin_rst, LOW);
                    delay(10);
                    digitalWrite(pin_rst, HIGH);
                    delay(10);
                    wait_busy(5000);
                }
                m_initialized = true;
            }
        }
       
    private:
        static gfx::gfx_result xlt_err(result r) {
            switch(r) {
                case result::io_error:
                    return gfx::gfx_result::device_error;
                case result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case result::success:
                    return gfx::gfx_result::success;
                case result::not_supported:
                    return gfx::gfx_result::not_supported;
                case result::invalid_argument:
                    return gfx::gfx_result::invalid_argument;
                default:
                    return gfx::gfx_result::unknown_error;
            }
        }
 public:
        constexpr inline gfx::size16 dimensions() const {return gfx::size16(width,height);}
        constexpr inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        // gets a point 
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            return m_frame_buffer.point(location,out_color);
       }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            return m_frame_buffer.point(location,color);
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            
            return m_frame_buffer.fill(rect,color);
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    display_update();
                }
            }
            return gfx::gfx_result::success;
        }
    };
}