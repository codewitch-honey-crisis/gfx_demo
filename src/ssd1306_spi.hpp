// NOT COMPLETE _ I'm working on this


// NOTICE OF USE:

/* Portions of this code derived from Adafruit's SSD1306 library. 
https://github.com/adafruit/Adafruit_SSD1306

The license that ships with their library is as follows:

Software License Agreement (BSD License)

Copyright (c) 2012, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_master.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
// TODO: Remove these or change them to consts
#define SSD1306_MEMORYMODE 0x20          ///< See datasheet
#define SSD1306_COLUMNADDR 0x21          ///< See datasheet
#define SSD1306_PAGEADDR 0x22            ///< See datasheet
#define SSD1306_SETCONTRAST 0x81         ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D          ///< See datasheet
#define SSD1306_SEGREMAP 0xA0            ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5        ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6       ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7       ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8        ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE          ///< See datasheet
#define SSD1306_DISPLAYON 0xAF           ///< See datasheet
#define SSD1306_COMSCANINC 0xC0          ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8          ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9        ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA          ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB       ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00  ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40  ///< See datasheet

#define SSD1306_EXTERNALVCC 0x01  ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26              ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27               ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E                    ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F                      ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3             ///< Set scroll range

namespace espidf {
    template<uint16_t Width,
            uint16_t Height,
            bool Vdc3_3,
            bool ResetBeforeInit,
            spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinDC,
            gpio_num_t PinRst,
            size_t BufferSize=64,
            size_t MaxTransactions=7,
            TickType_t Timeout=5000/portTICK_PERIOD_MS,
            bool UsePolling=true>
    struct ssd1306_spi final {
        enum struct result {
            success = 0,
            invalid_argument = 1,
            io_error = 2,
            out_of_memory = 3,
            timeout = 4,
            not_supported=5,
            io_busy =6
        };
        constexpr static const uint16_t width=Width;
        constexpr static const uint16_t height=Height;
        constexpr static const bool vdc_3_3 = Vdc3_3;
        constexpr static const bool reset_before_init = ResetBeforeInit;
        // the SPI host to use
        constexpr static const spi_host_device_t host_id = HostId;
        // the CS pin
        constexpr static const gpio_num_t pin_cs = PinCS;
        // the DC pin
        constexpr static const gpio_num_t pin_dc = PinDC;
        // the RST pin
        constexpr static const gpio_num_t pin_rst = PinRst;

        // indicates the buffer size. If specified, will end up being a multiple of 2. The minimum value is 4, for efficiency
        constexpr static const size_t buffer_size = ((BufferSize<4?4:BufferSize)/2)*2;
        // the maximum number of "in the air" transactions that can be queued
        constexpr static const size_t max_transactions = (0==MaxTransactions)?1:MaxTransactions;
        // the timeout for queued sends
        constexpr static const TickType_t timeout = Timeout;
        // indicates whether non queued transactions use polling
        constexpr static const bool use_polling = UsePolling;
private:
        unsigned int m_initialized;
        spi_device m_spi;
        unsigned int m_suspend_x1;
        unsigned int m_suspend_y1;
        unsigned int m_suspend_x2;
        unsigned int m_suspend_y2;
        unsigned int m_suspend_count;
        unsigned int m_suspend_first;
    
        struct rect {
            unsigned int x1;
            unsigned int y1;
            unsigned int x2;
            unsigned int y2;
        };
        uint8_t m_contrast;
        uint8_t m_frame_buffer[width*height/8];
        inline result command_impl(uint8_t cmd) {
            return write_bytes(&cmd,1,false);
        }
        
        result reset_impl() {
            if(GPIO_NUM_NC!=pin_rst) {
                if(ESP_OK!=gpio_set_level(pin_rst,1)) {
                    return result::io_error;
                }
                vTaskDelay(1/portTICK_PERIOD_MS);
                if(ESP_OK!=gpio_set_level(pin_rst, 0)) {
                    return result::io_error;
                }
                vTaskDelay(10/portTICK_PERIOD_MS);
                if(ESP_OK!=gpio_set_level(pin_rst,1)) {
                    return result::io_error;
                }
                return result::success;
            }
            return result::not_supported;
        }
        result initialize_impl() {
            result r;
            if(reset_before_init) {
                r=reset();
                if(result::success!=r && result::not_supported!=r) {
                    return r;
                }
            }
            // Init sequence
            static const uint8_t init1[] = {SSD1306_DISPLAYOFF,         // 0xAE
                                                    SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
                                                    0x80, // the suggested ratio 0x80
                                                    SSD1306_SETMULTIPLEX}; // 0xA8
            r=write_bytes(init1, sizeof(init1),false);

            if(result::success!=r) {
                return r;
            }
            r=command_impl(height - 1);
            if(result::success!=r) {
                return r;
            }

            static const uint8_t init2[] = {SSD1306_SETDISPLAYOFFSET, // 0xD3
                                                    0x0,                      // no offset
                                                    SSD1306_SETSTARTLINE | 0x0, // line #0
                                                    SSD1306_CHARGEPUMP};        // 0x8D
            r=write_bytes(init2, sizeof(init2),false);
            if(result::success!=r) {
                return r;
            }
            r=command_impl(!vdc_3_3 ? 0x10 : 0x14);
            if(result::success!=r) {
                return r;
            }
            static const uint8_t init3[] = {SSD1306_MEMORYMODE, // 0x20
                                                    0x00, // 0x0 act like ks0108, but we want mode 1?
                                                    SSD1306_SEGREMAP | 0x1,
                                                    SSD1306_COMSCANDEC};
            r=write_bytes(init3, sizeof(init3),false);
            if(result::success!=r) {
                return r;
            }
            uint8_t com_pins = 0x02;
            m_contrast = 0x8F;
            if ((width == 128) && (height == 32)) {
                com_pins = 0x02;
                m_contrast = 0x8F;
            } else if ((width == 128) && (height == 64)) {
                com_pins = 0x12;
                m_contrast = !vdc_3_3 ? 0x9F:0xCF;
            } else if ((width == 96) && (height == 16)) {
                com_pins = 0x2; // ada x12
                m_contrast = !vdc_3_3 ? 0x10:0xAF;
            } else {
                return result::not_supported;
            }

            r=command_impl(SSD1306_SETCOMPINS);
            if(result::success!=r) {
                return r;
            }
            r=command_impl(com_pins);
            if(result::success!=r) {
                return r;
            }
            r=command_impl(SSD1306_SETCONTRAST);
            if(result::success!=r) {
                return r;
            }
            r=command_impl(m_contrast);
            if(result::success!=r) {
                return r;
            }
            r=command_impl(SSD1306_SETPRECHARGE); // 0xd9
            if(result::success!=r) {
                return r;
            }
            r=command_impl(!vdc_3_3 ? 0x22:0xF1);
            if(result::success!=r) {
                return r;
            }
            static const uint8_t init5[] = {
                SSD1306_SETVCOMDETECT, // 0xDB
                0x40,
                SSD1306_DISPLAYALLON_RESUME, // 0xA4
                SSD1306_NORMALDISPLAY,       // 0xA6
                SSD1306_DEACTIVATE_SCROLL,
                SSD1306_DISPLAYON}; // Main screen turn on
            return write_bytes(init5, sizeof(init5),false);
        }
        static bool normalize_values(uint16_t& x1,uint16_t& y1,uint16_t& x2,uint16_t& y2,bool check_bounds=true) {
            // normalize values
            uint16_t tmp;
            if(x1>x2) {
                tmp=x1;
                x1=x2;
                x2=tmp;
            }
            if(y1>y2) {
                tmp=y1;
                y1=y2;
                y2=tmp;
            }
            if(check_bounds) {
                if(x1>=width||y1>=height)
                    return false;
                if(x2>=width)
                    x2=width-1;
                if(y2>height)
                    y2=height-1;
            }
            return true;
        }
        static result xlt_err(spi_result sr) {
            switch(sr) {
                case spi_result::timeout:
                    return result::timeout;
                case spi_result::invalid_argument:
                    return result::invalid_argument;
                case spi_result::previous_transactions_pending:
                case spi_result::host_in_use:
                case spi_result::dma_in_use:
                    return result::io_busy;
                case spi_result::out_of_memory:
                    return result::out_of_memory;
                case spi_result::success:
                    return result::success;
                default:
                    return result::io_error;
            }
        }
        result write_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(0==size) return result::success;
            esp_err_t err = gpio_set_level(pin_dc,is_data);
            if(ESP_OK!=err) {
                return result::io_error;
            }
            spi_result sr = m_spi.write(data,size);
            if(spi_result::success!=sr)
                return xlt_err(sr);
            return result::success;
        }
        result display_update(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) {
            if(!normalize_values(x1,y1,x2,y2))
                return result::success;
            result r = initialize();
            if(result::success!=r)
                return r;
            // don't draw if we're suspended
            if(0==m_suspend_count) {
                
                uint8_t dlist1[] = {
                    SSD1306_PAGEADDR,
                    uint8_t(y1/8),                   // Page start address
                    uint8_t(0xFF),                   // Page end (not really, but works here)
                    SSD1306_COLUMNADDR, uint8_t(x1)};// Column start address
                result sr=write_bytes(dlist1, sizeof(dlist1),false);
                if(result::success!=sr) {
                    return sr;
                }
                sr=command_impl(x2); // Column end address
                if(result::success!=sr) {
                    return sr;
                }
                i2c_master_command mc;
                if(x1==0&&y1==0&&x2==width-1&&y2==height-1) {
                    // special case for whole screen
                    return write_bytes(m_frame_buffer,width*height/8,true);
                }
                const int be = (y2+8)/8;
                const int w = x2-x1+1;
                for(int b=y1/8;b<be;++b) {
                    uint8_t* p = m_frame_buffer+(b*width)+x1;
                    sr = write_bytes(p,w,true);
                    if(result::success!=sr)
                        return sr;
                }
            }
            return result::success;
        }

        void buffer_fill(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2, bool color) {
            if(!normalize_values(x1,y1,x2,y2))
                return;
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = x1;
                    m_suspend_y1 = y1;
                    m_suspend_x2 = x2;
                    m_suspend_y2 = y2;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>x1)
                        m_suspend_x1=x1;
                    if(m_suspend_y1>y1)
                        m_suspend_y1=y1;
                    if(m_suspend_x2<x2)
                        m_suspend_x2=x2;
                    if(m_suspend_y2<y2)
                        m_suspend_y2=y2;
                }
            }
            int y=y1;
            int m=y%8;
            // special case when bottom and top are the same bank:
            if(y1/8==y2/8) {
                const uint8_t on_mask = uint8_t(uint8_t(0xFF<<((y1%8))) & uint8_t(0xFF>>(7-(y2%8))));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                for(int x=x1;x<=x2;++x) {
                    uint8_t* p = m_frame_buffer+(y/8*width)+x;
                    *p&=set_mask;
                    *p|=value_mask;
                }
                return;
            } 
           
            // first handle the top
            {
                const uint8_t on_mask = uint8_t(uint8_t(0xFF<<((y1%8))));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                for(int x=x1;x<=x2;++x) {
                    uint8_t* p = m_frame_buffer+(y/8*width)+x;
                    *p&=set_mask;
                    *p|=value_mask;
                    //++p;
                }
                
                y=y1+(8-m);
            }
            
            int be = y2/8;
            int b = y/8;
            
            while(b<be) {
                // we can do a faster fill for this part
                const int w = x2-x1+1;    
                uint8_t* p = m_frame_buffer+(b*width)+x1;
                memset(p,0xFF*color,w);
                
                ++b;
            }
            // now do the trailing rows
            if(b*8<y2) {
                m=y2%8;
                y=y2;
                const uint8_t on_mask = uint8_t(0xFF>>(7-(y2%8)));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                uint8_t* p = m_frame_buffer+(y/8*width)+x1;
                for(int x=x1;x<=x2;++x) {
                    *p&=set_mask;
                    *p|=value_mask;
                    ++p;
                }
            }
        }
        inline static spi_device_interface_config_t get_device_config() {
            spi_device_interface_config_t devcfg={
                .command_bits=0,
                .address_bits=0,
                .dummy_bits=0,
                .mode=0,
                .duty_cycle_pos=0,
                .cs_ena_pretrans=0,
                .cs_ena_posttrans=0,
        #ifdef HTCW_SSD1306_OVERCLOCK
                .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
        #else
                .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        #endif
                .input_delay_ns = 0,
                .spics_io_num=pin_cs,               //CS pin
                .flags =0,
                .queue_size=max_transactions,                          //We want to be able to queue 7 transactions at a time
                .pre_cb=NULL,
                .post_cb=NULL
            };
            return devcfg;
        } 
public:
        ssd1306_spi() : m_initialized(false),m_spi(host_id,get_device_config()), m_suspend_count(0),m_suspend_first(0)  {
        }
        bool initialized() const {
            return m_initialized;
        }
        result initialize() {
            if(!m_initialized) {
                result r = initialize_impl();
                if(result::success!=r)
                    return r;
                m_initialized=true;
            }
            return result::success;
        }
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
        // resets the display. The reset pin must be available and connected
        result reset() {
            if(!initialized()) {
                return initialize_impl();
            } 

            return reset_impl();
        }
        result pixel_read(uint16_t x,uint16_t y,bool* out_color) {
            result r=initialize();
            if(result::success!=r)
                return r;
            if(nullptr==out_color)
                return result::invalid_argument;
            if(x>=width || y>=height) {
                *out_color = false;
                return result::success;
            }
            uint8_t* p = m_frame_buffer+(y/8*width)+x;
            *out_color = 0!=(*p & (1<<(y&7)));
            return result::success;
        }
        result frame_fill(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,bool color) {
            result r = initialize();
            if(result::success!=r)
                return r;
            buffer_fill(x1,y1,x2,y2,color);
            return display_update(x1,y1,x2,y2);
        }
        result frame_suspend() {
            m_suspend_first=(m_suspend_count==0);
            ++m_suspend_count;
            return result::success;
        }
        result frame_resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    return display_update(m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2);
                }
                
            } 
            return result::success;
        }
        // GFX Bindings
        using type = ssd1306_spi<Width,Height,Vdc3_3,ResetBeforeInit,HostId,PinCS,PinDC,PinRst,BufferSize,MaxTransactions,Timeout,UsePolling>;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps< false,false,false,false,true,false>;
    private:
        gfx::gfx_result xlt_err(result r) {
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
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) {
            bool col=false;
            result r = pixel_read(location.x,location.y,&col);
            if(result::success!=r)
                return xlt_err(r);
            pixel_type p(!!col);
            *out_color=p;
            return gfx::gfx_result::success;
       }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            result r = frame_fill(location.x,location.y,location.x,location.y,color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            
            result r = frame_fill(rect.x1,rect.y1,rect.x2,rect.y2,color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            result r =frame_suspend();
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result resume(bool force=false) {
            result r =frame_resume(force);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
    };
}