#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "spi_master.hpp"
#include "bits.hpp"
// GFX bindings
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"

#define ALL_CHIPS 0xff
#define ALL_DIGITS 8

#define REG_DIGIT_0      (1 << 8)
#define REG_DECODE_MODE  (9 << 8)
#define REG_INTENSITY    (10 << 8)
#define REG_SCAN_LIMIT   (11 << 8)
#define REG_SHUTDOWN     (12 << 8)
#define REG_DISPLAY_TEST (15 << 8)

#define VAL_CLEAR_BCD    0x0f
#define VAL_CLEAR_NORMAL 0x00

namespace espidf {
    template<uint16_t Width,
            uint16_t Height,
            spi_host_device_t HostId,
            gpio_num_t PinCS,
            size_t MaxTransactions=1,
            bool UsePolling = true,
            size_t DmaSize = -1,
            TickType_t Timeout=5000/portTICK_PERIOD_MS>
    struct max7219 final {
        static_assert(Width==(Width/8)*8,"Width must be a multiple of 8");
        static_assert(Height==(Height/8)*8,"Height must be a multiple of 8");
        enum struct result {
            success = 0,
            invalid_argument = 1,
            io_error = 2,
            out_of_memory = 3,
            timeout = 4,
            not_supported=5,
            invalid_state=6,
            io_busy=7,
        };
        constexpr static const size_t segments = Width*Height/64;
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const spi_host_device_t host_id = HostId;
        constexpr static const gpio_num_t pin_cs = PinCS;
        constexpr static const size_t max_transactions = MaxTransactions;
        constexpr static const bool use_polling = UsePolling;
        constexpr static const size_t dma_size = DmaSize;
        constexpr static const TickType_t timeout = Timeout;
private:
        unsigned int m_initialized;
        spi_device m_spi;
        uint8_t m_frame_buffer[8*segments];
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
        
        static bool normalize_values(rect& bounds,bool check_bounds=true) {
            // normalize values
            uint16_t tmp;
            if(bounds.x1>bounds.x2) {
                tmp=bounds.x1;
                bounds.x1=bounds.x2;
                bounds.x2=tmp;
            }
            if(bounds.y1>bounds.y2) {
                tmp=bounds.y1;
                bounds.y1=bounds.y2;
                bounds.y2=tmp;
            }
            if(check_bounds) {
                if(bounds.x1>=width||bounds.y1>=height)
                    return false;
                if(bounds.x2>=width)
                    bounds.x2=width-1;
                if(bounds.y2>height)
                    bounds.y2=height-1;
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
        result send(uint8_t chip, uint16_t value)
        {
            uint16_t buf[segments] = { 0 };
            if (chip == ALL_CHIPS)
            {
                for (uint8_t i = 0; i < segments; i++)
                    buf[i] = bits::swap(value);
            }
            else buf[chip] = bits::swap(value);

            spi_result r = m_spi.write((uint8_t*)buf,segments*2);
            if(spi_result::success!=r) 
                return xlt_err(r);
            return result::success;
        }
        result display_line(uint8_t line,uint8_t value) {
            uint8_t chip = line / 8;
            uint8_t data = (line & 7);
            return send(chip, (REG_DIGIT_0 + ((uint16_t)data << 8)) | value);
        }
        void line_rect(uint8_t line,rect* out_rect) {
            out_rect->x1 = line % (width/8);
            out_rect->x2 = out_rect->x1 + 7;
            out_rect->y1=out_rect->y2= line / (width/8);
        }
        result display_update(const rect& bounds) {
            // TODO: There's a more efficient way to do 
            // this than brute force but my head is not
            // in it right now. revisit
            /*for(int i = 0;i<segments*8;++i) {
                rect lr;
                line_rect(i,&lr);
                if(bounds.x1>=lr.x1 && 
                    bounds.x2<=lr.x2 && 
                    bounds.y1>=lr.y1 && 
                    bounds.y2<=lr.y2) {
                    const uint8_t val = m_frame_buffer[(lr.y1*width+lr.x1)/8];
                    const result r = display_line(i,val);
                    if(result::success!=r) {
                        return r;
                    }
                }
            }*/
            // TODO: above isn't even working, fallback
            for(int i=0;i<segments*8;++i) {
                rect lr;
                line_rect(i,&lr);
                const uint8_t val = m_frame_buffer[(lr.y1*width+lr.x1)/8];
                const result r = display_line(i,val);
                if(result::success!=r) {
                    return r;
                }
            }
            return result::success;
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
                .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
                .input_delay_ns = 0,
                .spics_io_num=pin_cs,               //CS pin
                .flags =SPI_DEVICE_NO_DUMMY,
                .queue_size=max_transactions,
                .pre_cb=nullptr,  
                .post_cb=nullptr
            };
            return devcfg;
        } 
        void buffer_fill(const rect& bounds,bool color) {
            rect b = bounds;
            if(!normalize_values(b))
                return;
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = b.x1;
                    m_suspend_y1 = b.y1;
                    m_suspend_x2 = b.x2;
                    m_suspend_y2 = b.y2;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>b.x1)
                        m_suspend_x1=b.x1;
                    if(m_suspend_y1>b.y1)
                        m_suspend_y1=b.y1;
                    if(m_suspend_x2<b.x2)
                        m_suspend_x2=b.x2;
                    if(m_suspend_y2<b.y2)
                        m_suspend_y2=b.y2;
                }
            }
            const uint16_t w=b.x2-b.x1+1,h = b.y2-b.y1+1;
            
            for(int y = 0;y<h;++y) {
                const size_t offs = ((y+b.y1)*width+(b.x1));
                uint8_t* const pbegin = m_frame_buffer+(offs/8);
                bits::set_bits(pbegin,offs%8,w,color);
            }
        
        }
        result clear_internal()
        {
            for (uint8_t i = 0; i < 8; i++) {
                result r=send(ALL_CHIPS, (REG_DIGIT_0 + ((uint16_t)i << 8)));
                if(result::success!=r) {
                    return r;
                }
            }

            return result::success;
        }

        result initialize_impl() {
            result r;
            r=send(ALL_CHIPS,REG_SHUTDOWN);
            if(r!=result::success) {
                return r;
            }
            r=send(ALL_CHIPS, REG_DISPLAY_TEST);
            if(r!=result::success) {
                return r;
            }
            r=send(ALL_CHIPS, REG_SCAN_LIMIT | 7);
            if(r!=result::success) {
                return r;
            }
            r=send(ALL_CHIPS, REG_DECODE_MODE);
            if(r!=result::success) {
                return r;
            }
            r=clear_internal();
            if(r!=result::success) {
                return r;
            }
            r=send(ALL_CHIPS, REG_INTENSITY);
            if(r!=result::success) {
                return r;
            }
            r=send(ALL_CHIPS,REG_SHUTDOWN|1);
            if(r!=result::success) {
                return r;
            }
            return result::success;
        }
public:
        max7219() : m_initialized(false),m_spi(host_id,get_device_config()),m_suspend_count(0),m_suspend_first(0)  {
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
            uint8_t* p = m_frame_buffer+(y*width/8)+x;
            *out_color = 0!=(*p & (1<<(7-(x&7))));
            return result::success;
        }
        result frame_fill(const rect& bounds,bool color) {
            result r = initialize();
            if(result::success!=r)
                return r;
            buffer_fill(bounds,color);
            return display_update(bounds);
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
                    return display_update({m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2});
                }
                
            } 
            return result::success;
        }
        // GFX Bindings
        using type = max7219<Width,Height,HostId,PinCS,MaxTransactions,UsePolling,DmaSize,Timeout>;
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
            result r = frame_fill({location.x,location.y,location.x,location.y},color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            
            result r = frame_fill({rect.x1,rect.y1,rect.x2,rect.y2},color.native_value!=0);
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