#pragma once
#include "common/spi_master.hpp"
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"
#include "gfx_palette.hpp"
namespace espidf {
    
    namespace dep0290b_helpers {
         static const uint8_t display[] = {
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
    template<spi_host_device_t HostId,
        gpio_num_t PinCS,
        gpio_num_t PinDC,
        gpio_num_t PinRst,
        gpio_num_t PinBusy>
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
        
        constexpr static const spi_host_device_t host_id = HostId;

        constexpr static const gpio_num_t pin_cs = PinCS;
        constexpr static const gpio_num_t pin_dc = PinDC;
        // the RST pin
        constexpr static const gpio_num_t pin_rst = PinRst;
        // the Busy pin
        constexpr static const gpio_num_t pin_busy = PinBusy;

        constexpr static const uint32_t clock_speed = 4*1000*1000;
        struct rect {
            uint16_t x1;
            uint16_t y1;
            uint16_t x2;
            uint16_t y2;
        };
    private: 
        bool m_initialized;
        spi_device m_spi;
        int m_suspend_count;
        uint8_t m_frame_buffer[width*height/8];

        static result xlt_err(spi_result rr) {    
            if(spi_result::success!=rr) {
                switch(rr) {
                    case spi_result::timeout:
                        return result::timeout;
                    case spi_result::out_of_memory:
                        return result::out_of_memory;
                    case spi_result::previous_transactions_pending:
                        return result::io_busy;
                    default:
                        return result::io_error;
                }
            }
            return result::success;
        }

        result send_commands(const uint8_t* commands) {
            uint8_t cmd_count, cmd, arg_count;
            uint16_t ms;
            cmd_count = *(commands++); // Number of commands to follow
            while (cmd_count--) {              // For each command...
                cmd = *(commands++);       // Read command
                arg_count = *(commands++);   // Number of args to follow
                ms = arg_count & 0x80;       // If hibit set, delay follows args
                arg_count &= ~0x80;          // Mask out delay bit
                result r=send_command(cmd);
                if(result::success!=r) {
                    return r;
                }
                while(arg_count--) {
                    r=send_data(*commands++);
                    if(result::success!=r) {
                        return r;
                    }
                }
                if (ms) {
                    ms = *(commands++);
                    if(ms==254) {
                        ms=1000;
                    }
                    if (ms == 255)
                        ms = 1200;
                    r =wait_busy(ms);
                    if(result::success!=r) {
                        return r;
                    }
                }
            }
            return result::success;
        }
        result wait_busy(unsigned int ms=100) {
            if(GPIO_NUM_NC!=pin_busy) {
                uint32_t start = esp_timer_get_time();
                while(esp_timer_get_time()-start<5000*1000) {
                    if(0!=gpio_get_level(pin_busy)) {
                        return result::success;
                    }
                    vTaskDelay(1);
                }
                
                return result::timeout;
            }
            vTaskDelay(ms/portTICK_PERIOD_MS);
            return result::success;
        }
        result send_command(uint8_t command)
        {
            spi_result r= m_spi.write(&command,1,(void*)0,true);
            if(spi_result::success!=r) {
                return xlt_err(r);
            }
            return result::success;
        }

        result send_data(uint8_t data)
        {
            spi_result r= m_spi.write(&data,1,(void*)1,true);
            if(spi_result::success!=r) {
                return xlt_err(r);
            }
            return result::success;
        }

        static bool normalize_values(rect& r,bool check_bounds=true) {
            // normalize values
            uint16_t tmp;
            if(r.x1>r.x2) {
                tmp=r.x1;
                r.x1=r.x2;
                r.x2=tmp;
            }
            if(r.y1>r.y2) {
                tmp=r.y1;
                r.y1=r.y2;
                r.y2=tmp;
            }
            if(check_bounds) {
                if(r.x1>=width||r.y1>=height)
                    return false;
                if(r.x2>=width)
                    r.x2=width-1;
                if(r.y2>height)
                    r.y2=height-1;
            }
            return true;
        }
        void buffer_fill(const rect& bounds,bool color) {
            rect b = bounds;
            if(!normalize_values(b))
                return;
            
            const uint16_t w=b.x2-b.x1+1,h = b.y2-b.y1+1;
            
            for(int y = 0;y<h;++y) {
                const size_t offs = ((y+b.y1)*width+(b.x1));
                uint8_t* const pbegin = m_frame_buffer+(offs/8);
                bits::set_bits(pbegin,offs%8,w,color);
            }
        }
        void display_update() {
            initialize();
            // don't draw if we're suspended
            if(0==m_suspend_count) {    
               
               send_commands(dep0290b_helpers::display);
                const uint8_t* p = m_frame_buffer;
                for (uint32_t y = 0; y < height; y++) {
                    for (uint32_t x = 0; x < width / 8; x++) {
                        send_data(*p++);
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
        inline static spi_device_interface_config_t get_device_config() {
            spi_device_interface_config_t devcfg={
                .command_bits=0,
                .address_bits=0,
                .dummy_bits=0,
                .mode=0,
                .duty_cycle_pos=0,
                .cs_ena_pretrans=0,
                .cs_ena_posttrans=0,
                .clock_speed_hz=clock_speed,           //Clock out at 4 MHz
                .input_delay_ns = 0,
                .spics_io_num=pin_cs,               //CS pin
                .flags =SPI_DEVICE_NO_DUMMY,
                .queue_size=7,                          //We only need 1 at a time
                .pre_cb=[](spi_transaction_t*t){
                    int dc=(int)t->user;
                    gpio_set_level(pin_dc, dc!=0);
                },  //Specify pre-transfer callback to handle D/C line
                .post_cb=NULL
            };
            return devcfg;
        } 
    public:
        depg0290b() : m_initialized(false),m_spi(host_id,get_device_config()),m_suspend_count(0) {

        }
        void reset() {
            if (pin_rst != GPIO_NUM_NC)
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                gpio_set_level(pin_rst, 0);
                vTaskDelay(10/portTICK_PERIOD_MS);
                gpio_set_level(pin_rst, 1);
                vTaskDelay(150/portTICK_PERIOD_MS);
            }
        }
        void initialize() {
            if(!m_initialized) {
                if (pin_cs != GPIO_NUM_NC) {
                    gpio_set_direction(pin_cs, GPIO_MODE_OUTPUT);
                }
                if (pin_dc != GPIO_NUM_NC) {
                    gpio_set_direction(pin_dc, GPIO_MODE_OUTPUT);
                }
                if (pin_rst >= GPIO_NUM_NC) {
                    gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
                }
                reset();
                if (pin_busy >= GPIO_NUM_NC) {
                    gpio_set_direction(pin_busy, GPIO_MODE_OUTPUT);
                }
                if (pin_rst >= 0) {
                    gpio_set_level(pin_rst, 0);
                    vTaskDelay(10/portTICK_PERIOD_MS);
                    gpio_set_level(pin_rst, 1);
                    vTaskDelay(10/portTICK_PERIOD_MS);    
                    wait_busy(5000);
                }
                m_initialized = true;
            }
        }
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
        
        result pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return result::invalid_argument;
            if(x>=width || y>=height) {
                *out_color = false;
                return result::success;
            }
            const uint8_t* p = m_frame_buffer+(y/8*width)+x;
            *out_color = 0!=(*p & (1<<(y&7)));
            return result::success;
        }
        result frame_fill(const rect& bounds,bool color) {
            initialize();
            buffer_fill(bounds,color);
            display_update();
            return result::success;
        }
        result frame_suspend() {
            ++m_suspend_count;
            return result::success;
        }
        result frame_resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    display_update();
                }
                
            } 
            return result::success;
        }
        // GFX Bindings
        using type = depg0290b;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
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