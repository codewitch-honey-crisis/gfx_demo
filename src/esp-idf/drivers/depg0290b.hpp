#pragma once
#include <stdint.h>
#include "esp_timer.h"
#include "common/spi_master.hpp"
#include "gfx_core.hpp"
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"
#include "gfx_palette.hpp"
namespace espidf {
    namespace depg0290b_helpers {
        static const uint8_t lut_default_part[] = {
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00,
    0x00, 0x00, //0x22, 0x17, 0x41, 0xB0, 0x32,
    // 0x36,
};
    }
    template<spi_host_device_t HostId,
        gpio_num_t PinCS,
        gpio_num_t PinDC,
        gpio_num_t PinRst,
        gpio_num_t PinBusy,
        size_t MaxTransactions=7,
        bool UsePolling = true,
        size_t DmaSize = -1,
        TickType_t Timeout=5000/portTICK_PERIOD_MS,
        size_t BatchBufferSize=64
        >
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
        constexpr static const size_t width = 128;
        constexpr static const size_t height = 296;
        constexpr static const spi_host_device_t host_id = HostId;
        constexpr static const gpio_num_t pin_cs = PinCS;
        constexpr static const gpio_num_t pin_dc = PinDC;
        constexpr static const size_t clock_speed = 1*1000*1000;
        constexpr static const size_t batch_buffer_size = (BatchBufferSize/sizeof(uint16_t))*sizeof(uint16_t);
        // the RST pin
        constexpr static const gpio_num_t pin_rst = PinRst;
        // the Busy pin
        constexpr static const gpio_num_t pin_busy = PinBusy;
        // the maximum number of "in the air" transactions that can be queued
        constexpr static const size_t max_transactions = (0==MaxTransactions)?1:MaxTransactions;
        constexpr static const bool use_polling = UsePolling;
        constexpr static const size_t dma_size = DmaSize;
        constexpr static const size_t timeout = Timeout;
        struct rect {
            uint16_t x1;
            uint16_t y1;
            uint16_t x2;
            uint16_t y2;
        };
    private: 
        bool m_initialized;
        spi_device m_spi;
        uint16_t m_suspend_x1;
        uint16_t m_suspend_y1;
        uint16_t m_suspend_x2;
        uint16_t m_suspend_y2;
        int m_suspend_first;
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
        result write_window(uint16_t xs, uint16_t ys, uint16_t xd, uint16_t yd, uint16_t w, uint16_t h) {
            uint16_t xds_d8 = xd / 8;
            uint16_t xde_d8 = (xd + w - 1) / 8;
            uint16_t yde = yd + h - 1;
            
            uint16_t xse_d8 = xs / 8 + xde_d8 - xds_d8;
            uint16_t yse = ys + h - 1;
            result r = set_ram_area(xds_d8, xde_d8, yd % 256, yd / 256, yde % 256, yde / 256); // X-source area,Y-gate area
            if(result::success!=r) {
                return r;
            }
            r=set_ram_pointer(xds_d8, yd % 256, yd / 256); // set ram
            if(result::success!=r) {
                return r;
            }
            r=wait_busy(100); // needed ?
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x24);
            if(result::success!=r) {
                return r;
            }
            for (int16_t y1 = ys; y1 <= yse; ++y1) {
                for (int16_t x1 = xs / 8; x1 <= xse_d8; ++x1) {
                    uint16_t idx = y1 * (width / 8) + x1;
                    uint8_t data = (idx < (width*height/8)) ? m_frame_buffer[idx] : 0x00;
                    r=send_data(data);
                    if(result::success!=r) {
                        return r;
                    }
                }
            }
            return result::success;
        }
        result display_update(const rect& bounds) {
            result r = initialize();
            if(result::success!=r)
                return r;
            rect b(bounds);
            if(!normalize_values(b))
                return result::success;
            // don't draw if we're suspended
            if(0==m_suspend_count) {
                r=init_full(0x01);
                if(result::success!=r) {
                    return r;
                }
                r=send_command(0x24);
                if(result::success!=r) {
                    return r;
                }
                r=send_data(m_frame_buffer,width*height/8);
                if(result::success!=r) {
                    return r;
                }
                r=update_full();
                if(result::success!=r) {
                    return r;
                }
                // power off?
               
            }
            return result::success;
        }
        result send_command(uint8_t cmd,bool init=false) {
            printf("send_command(0x%02X,%s);\r\n",(int)cmd,init?"true":"false");
            if(init && !m_initialized) {
                result r=initialize();
                if(result::success!=r) {
                    return r;
                }
            }
            result rr=wait_busy(100);
            if(result::success!=rr) {
                return rr;
            }
            spi_result r = m_spi.write(&cmd,1,(void*)0);
            if(spi_result::success!=r)
                return xlt_err(r);
            return result::success;
        }
        result send_data(const uint8_t* data, size_t size,bool init=false,bool silent=false) {
            if(!silent) {
                if(1==size) {
                    printf("send_data(0x%02X, %s);\r\n",(int)*data,init?"true":"false");
                } else {
                    printf("send_data(..., %d, %s);\r\n",(int)size, init?"true":"false");
                }
            }
            if(init && !m_initialized) {
                result r=initialize();
                if(result::success!=r) {
                    return r;
                }
            }
            spi_result r = m_spi.write(data,size,(void*)1);
            if(spi_result::success!=r)
                return xlt_err(r);
            return result::success;
        }
        inline result send_data(const uint8_t data,bool init=false) {
            return send_data(&data,1,init);
        }
        result init_display(uint8_t em) {
            result r=send_command(0x12,true);
            if(result::success!=r) {
                return r;
            }
            r=wait_busy(1000);
            if(result::success!=r) {
                return r;
            }
            return set_ram_data_entry_mode(em);
        }
        result set_ram_area(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1) {
            result r = send_command(0x44,true);
            r=send_data(Xstart + 1 );
            if(result::success!=r) {
                return r;
            }
            r=send_data(Xend + 1);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x45);
            if(result::success!=r) {
                return r;
            }
            r=send_data(Ystart);
            if(result::success!=r) {
                return r;
            }
            r=send_data(Ystart1);
            if(result::success!=r) {
                return r;
            }
            r=send_data(Yend);
            if(result::success!=r) {
                return r;
            }
            return send_data(Yend1);
        }

        result set_ram_pointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
        {
            result r=send_command(0x4e,true);
            if(result::success!=r) {
                return r;
            }
            r=send_data(addrX + 1);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x4f);
            if(result::success!=r) {
                return r;
            }
            send_data(addrY);
            if(result::success!=r) {
                return r;
            }
            send_data(addrY1);
            if(result::success!=r) {
                return r;
            }
            return result::success;
        }
        result init_full(uint8_t em) {
            result r=init_display(em);
            if(result::success!=r) {
                return r;
            }
            // power on
            return result::success;
        }
        result update_full() {
            result r = send_command(0x20);
            if(result::success!=r) {
                return r;
            }
            return wait_busy(1200);
        }
        result update_partial() {
            result r = send_command(0x22);
            r=send_data(0xCF);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x20);
            if(result::success!=r) {
                return r;
            }
            return wait_busy(300);
            // _writeCommand(0xff);
        }
        result init_partial(uint8_t em) {
            result r=init_display(em);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x32);
            if(result::success!=r) {
                return r;
            }
            r=send_data(depg0290b_helpers::lut_default_part,sizeof(depg0290b_helpers::lut_default_part));
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x3f);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x17);
            if(result::success!=r) {
                return r;
            }
            
            r=send_command(0x04);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x41);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0xB0);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x32);
            if(result::success!=r) {
                return r;
            }
            
            r=send_command(0x2C);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x36);
            if(result::success!=r) {
                return r;
            }
            
            r=send_command(0x37);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x40);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x00);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x3C);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0x80);
            if(result::success!=r) {
                return r;
            }
            
            r=send_command(0x22);
            if(result::success!=r) {
                return r;
            }
            r=send_data(0xC0);
            if(result::success!=r) {
                return r;
            }
            r=send_command(0x20);
            if(result::success!=r) {
                return r;
            }
            r=wait_busy(1000);
            if(result::success!=r) {
                return r;
            }
            // power on
            return result::success;
        }
        result set_ram_data_entry_mode(uint8_t em) {
            const uint16_t x2 = width - 1;
            const uint16_t y2 = height - 1;
            result r=send_command(0x11);
            if(result::success!=r) {
                return r;
            }
            r=send_data(em);
            if(result::success!=r) {
                return r;
            }
            switch (em) {
            case 0x00: // x decrease, y decrease
                r=set_ram_area(x2 / 8, 0x00, y2 % 256, y2 / 256, 0x00, 0x00);  // X-source area,Y-gate area
                if(result::success!=r) {
                    return r;
                }
                r=set_ram_pointer(x2 / 8, y2 % 256, y2 / 256); // set ram
                if(result::success!=r) {
                    return r;
                }
                break;
            case 0x01: // x increase, y decrease
                r=set_ram_area(0x00, x2 / 8, y2 % 256, y2 / 256, 0x00, 0x00);  // X-source area,Y-gate area
                if(result::success!=r) {
                    return r;
                }r=set_ram_pointer(0x00, y2 % 256, y2 / 256); // set ram
                if(result::success!=r) {
                    return r;
                }
                break;
            case 0x02: // x decrease, y increase
                r=set_ram_area(x2 / 8, 0x00, 0x00, 0x00, y2 % 256, y2 / 256);  // X-source area,Y-gate area
                if(result::success!=r) {
                    return r;
                }
                r=set_ram_pointer(x2 / 8, 0x00, 0x00); // set ram
                if(result::success!=r) {
                    return r;
                }
                break;
            case 0x03: // x increase, y increase : normal mode
                r=set_ram_area(0x00, x2 / 8, 0x00, 0x00, y2 % 256, y2 / 256);  // X-source area,Y-gate area
                if(result::success!=r) {
                    return r;
                }
                r=set_ram_pointer(0x00, 0x00, 0x00); // set ram
                if(result::success!=r) {
                    return r;
                }
                break;
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
                .clock_speed_hz=clock_speed,           //Clock out at 4 MHz
                .input_delay_ns = 0,
                .spics_io_num=pin_cs,               //CS pin
                .flags =0,//SPI_DEVICE_HALFDUPLEX,
                .queue_size=max_transactions,                          //We want to be able to queue 7 transactions at a time
                .pre_cb=[](spi_transaction_t*t){
                    int dc=(int)t->user;
                    gpio_set_level(pin_dc, dc!=0);
                },  //Specify pre-transfer callback to handle D/C line
                .post_cb=NULL
            };
            return devcfg;
        } 
        result wait_busy(unsigned int ms=100) {
            if(GPIO_NUM_NC!=pin_busy) {
                uint32_t start = esp_timer_get_time();
                while(esp_timer_get_time()-start<timeout*1000) {
                    if(0!=gpio_get_level(pin_busy)) {
                        return result::success;
                    }
                    vTaskDelay(1);
                }
                printf("timeout\r\n");
                return result::timeout;
            }
            vTaskDelay(ms/portTICK_PERIOD_MS);
            return result::success;
        }
                
    public:
        depg0290b() : m_initialized(false),
            m_spi(host_id,get_device_config()) {
        }
        inline bool initialized() const {
 
            return m_initialized;
        }
        result initialize() {
            if(!m_initialized) {
                static const TickType_t t10ms = 10/portTICK_RATE_MS;                
                //static const TickType_t t100ms = 100/portTICK_RATE_MS;                
                //Initialize non-SPI GPIOs
                /*gpio_config_t pc;
                pc.pin_bit_mask = (1<<(int)pin_rst) | (1<<(int)pin_dc);
                pc.mode=GPIO_MODE_OUTPUT;
                pc.intr_type = GPIO_INTR_DISABLE;
                pc.pull_down_en = GPIO_PULLDOWN_DISABLE;
                pc.pull_up_en = GPIO_PULLUP_DISABLE;
                gpio_config(&pc);
                */
                ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(pin_cs,GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(pin_rst,GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(pin_dc,GPIO_MODE_OUTPUT));
                if(GPIO_NUM_NC!=pin_busy) {
                    /*pc.pin_bit_mask = (1<<(int)pin_busy);
                    pc.mode=GPIO_MODE_INPUT;
                    pc.intr_type = GPIO_INTR_DISABLE;
                    pc.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    pc.pull_up_en = GPIO_PULLUP_DISABLE;
                    gpio_config(&pc);
                    */
                    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(pin_busy,GPIO_MODE_INPUT));

                }
                if(GPIO_NUM_NC!=pin_rst) {
                    gpio_set_level(pin_rst,0);
                    vTaskDelay(t10ms);
                    gpio_set_level(pin_rst,1);
                    vTaskDelay(t10ms);
                    wait_busy(timeout);
                }
                m_initialized=true;
            }
            return result::success;
        }
        void test(uint8_t em) {
            printf("initialize\r\n");
            initialize();
            printf("init_full(0x%02X);\r\n",(int)em);
            init_full(em);
            send_command(0x24);
            printf("sending data\r\n");
            for (uint32_t y = 0; y < height; y++) {
                for (uint32_t x = 0; x < width / 8; x++) {
                    uint8_t data = 0xFF;
                    if ((x < 1) && (y < 8)) data = 0x00;
                    if ((x > width / 8 - 3) && (y < 16)) data = 0x00;
                    if ((x > width / 8 - 4) && (y > height - 25)) data = 0x00;
                    if ((x < 4) && (y > height - 33)) data = 0x00;
                    //data = ~data;
                    send_data(&data,1,false,true);
                }
            }
            update_full();
            // power off
        }
        result pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return result::invalid_argument;
            if(x>=width || y>=height) {
                *out_color = false;
                return result::success;
            }
            const uint8_t* p = m_frame_buffer+(y*width/8)+x;
            *out_color = 0!=(*p & (1<<(7-(x&7))));
            return result::success;
        }
        result pixel_write(uint16_t x,uint16_t y,bool color) {
            return frame_fill({x,y,x,y},color);
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
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            
            result r = frame_fill({bounds.x1,bounds.y1,bounds.x2,bounds.y2},color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type p;
            return fill(bounds,p);
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