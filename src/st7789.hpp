#pragma once
#define HTCW_ST7789_OVERCLOCK
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_driver.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"

namespace espidf {
     namespace st7789_helpers {
     struct init_cmd {
            uint8_t cmd;
            uint8_t data[16];
            uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
        };
        DRAM_ATTR static const init_cmd init_cmds[] = {
            //Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
    /* Memory Data Access Control */
    {0x36, {0xA0}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x45}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x2B}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
    // display inversion ON - not sure why this is necessary
    {0x21, {},0},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01, 0xff}, 2},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x11}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 1},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}

        };
     }
    // the driver for the ST7789 display
    template<uint16_t Width,
            uint16_t Height,
            spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinDC,
            gpio_num_t PinRst,
            gpio_num_t PinBacklight,
            size_t MaxTransactions=7,
            bool UsePolling = true,
            size_t DmaSize = -1,
            TickType_t Timeout=5000/portTICK_PERIOD_MS,
            size_t BatchBufferSize=64
            >
    struct st7789 : 
            public spi_driver<Width,
                            Height,
                            HostId,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7789_OVERCLOCK
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            MaxTransactions,
                            UsePolling,
                            DmaSize,
                            Timeout,
                            BatchBufferSize> {
        using base_type = spi_driver<Width,
                            Height,
                            HostId,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7789_OVERCLOCK
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            MaxTransactions,
                            UsePolling,
                            DmaSize,
                            Timeout,
                            BatchBufferSize>;
        // the RST pin
        constexpr static const gpio_num_t pin_rst = PinRst;
        // the BL pin
        constexpr static const gpio_num_t pin_backlight = PinBacklight;
        // the maximum number of "in the air" transactions that can be queued
        constexpr static const size_t max_transactions = (0==MaxTransactions)?1:MaxTransactions;
    private:
       
        spi_driver_result write_window_impl(const spi_driver_rect& win,bool queued,spi_driver_set_window_flags set_flags) {
            //printf("(%d, %d)-(%d, %d)\r\n",win.x1,win.y1,win.x2,win.y2);
            
            spi_driver_result r;
            uint8_t tx_data[4];
            //Column Address Set
            r=this->send_next_command(0x2A,queued);
            if(spi_driver_result::success!=r)
                return r;
            if(set_flags.x1 || set_flags.x2) {
                tx_data[0]=win.x1>>8;             //Start Col High
                tx_data[1]=win.x1&0xFF;           //Start Col Low
                tx_data[2]=win.x2>>8;             //End Col High
                tx_data[3]=win.x2&0xff;           //End Col Low
                r=this->send_next_data(tx_data,4,queued,true);
                if(spi_driver_result::success!=r)
                    return r;
            }
            if(set_flags.y1 || set_flags.y2 || !(set_flags.x1 || set_flags.x2)) {
                //Page address set
                r=this->send_next_command(0x2B,queued,true);
                if(spi_driver_result::success!=r)
                    return r;
                tx_data[0]=win.y1>>8;        //Start page high
                tx_data[1]=win.y1&0xff;      //start page low
                tx_data[2]=win.y2>>8;        //end page high
                tx_data[3]=win.y2&0xff;      //end page low
                r=this->send_next_data(tx_data,4,queued,true);
                if(spi_driver_result::success!=r)
                    return r;
            }
            // Memory write
            return this->send_next_command(0x2C,queued,true);
        }
    public:
        // constructs a new instance of the driver
        st7789() {
            
        }
        virtual ~st7789() {}
        // forces initialization of the driver
        spi_driver_result initialize()
        {
            if(!this->initialized()) {
                static const TickType_t ts = 100/portTICK_RATE_MS;

                int cmd=0;
                
                //Initialize non-SPI GPIOs
                gpio_set_direction(base_type::pin_dc, GPIO_MODE_OUTPUT);
                gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
                gpio_set_direction(pin_backlight, GPIO_MODE_OUTPUT);

                //Reset the display
                gpio_set_level(pin_rst, 0);
                vTaskDelay(ts);
                gpio_set_level(pin_rst, 1);
                vTaskDelay(ts);
                
                //Send all the commands
                while (st7789_helpers::init_cmds[cmd].databytes!=0xff) {
                    spi_driver_result r = this->send_init_command(st7789_helpers::init_cmds[cmd].cmd);
                    if(spi_driver_result::success!= r) {
                        return r;
                    }
                    r=this->send_init_data(st7789_helpers::init_cmds[cmd].data,st7789_helpers::init_cmds[cmd].databytes&0x1F);
                    if(spi_driver_result::success!= r) {
                        return spi_driver_result::io_error;
                    }
                    if (st7789_helpers::init_cmds[cmd].databytes&0x80) {
                        vTaskDelay(ts);
                    }
                    ++cmd;
                }
                ///Enable backlight
                gpio_set_level(pin_backlight, 0);
            }
            return spi_driver_result::success;
        }
protected:
        virtual spi_driver_result write_window(const spi_driver_rect& bounds,spi_driver_set_window_flags set_flags) {
            return write_window_impl(bounds,false,set_flags);
        }
        virtual spi_driver_result queued_write_window(const spi_driver_rect& bounds,spi_driver_set_window_flags set_flags) {
            return write_window_impl(bounds,true,set_flags);
        }

        // GFX bindings
 public:
        // indicates the type, itself
        using type = st7789<Width,Height,HostId,PinCS,PinDC,PinRst,PinBacklight,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,true,true,true,false,false>;
 
 private:
        gfx::gfx_result xlt_err(spi_driver_result r) {
            switch(r) {
                case spi_driver_result::io_error:
                    return gfx::gfx_result::device_error;
                case spi_driver_result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case spi_driver_result::success:
                    return gfx::gfx_result::success;
                default:
                    return gfx::gfx_result::invalid_argument;
            }
        }
        template<typename Source>
        gfx::gfx_result copy_from_impl(const gfx::rect16& src_rect,const Source& src,gfx::point16 location,bool async) {
            spi_driver_result r;
            gfx::rect16 srcr = src_rect.normalize().crop(src.bounds());
            gfx::rect16 dstr(location,src_rect.dimensions());
            dstr=dstr.crop(bounds());
            if(srcr.width()>dstr.width()) {
                srcr.x2=srcr.x1+dstr.width()-1;
            }
            if(srcr.height()>dstr.height()) {
                srcr.y2=srcr.y1+dstr.height()-1;
            }
            if(gfx::helpers::is_same<pixel_type,typename Source::pixel_type>::value && Source::caps::blt) {
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    spi_driver_rect dr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                    if(!async)
                        r=this->frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    else
                        r=this->queued_frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    if(spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                while(yy<hh) {
                    spi_driver_rect dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.x2+yy)};
                    if(!async)
                        r = this->frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    else
                        r = this->queued_frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    if(spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                    ++yy;
                }
                return gfx::gfx_result::success;
            }
            uint16_t w = dstr.dimensions().width;
            uint16_t h = dstr.dimensions().height;
            spi_driver_rect drr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
            if(!async)
                r=this->batch_write_begin(drr);
            else
                r=this->queued_batch_write_begin(drr);
            if(spi_driver_result::success!=r) {
                return xlt_err(r);
            }
            for(uint16_t y=0;y<h;++y) {
                for(uint16_t x=0;x<w;++x) {
                    typename Source::pixel_type pp;
                    gfx::gfx_result rr=src.point(gfx::point16(x+srcr.x1,y+srcr.y1), &pp);
                    if(rr!=gfx::gfx_result::success)
                        return rr;
                    pixel_type p;
                    if(!pp.convert(&p)) {
                        return gfx::gfx_result::invalid_format;
                    }
                    uint16_t pv = p.value();
                    if(!async)
                        r = this->batch_write(&pv,1);
                    else
                        r = this->queued_batch_write(&pv,1);
                    if(spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                }
            }
            if(!async)
                r=this->batch_write_commit();
            else
                r=this->queued_batch_write_commit();
            if(spi_driver_result::success!=r) {
                return xlt_err(r);
            }
            return gfx::gfx_result::success;
        }
 public:
        // retrieves the dimensions of the screen
        constexpr inline gfx::size16 dimensions() const {
            return gfx::size16(base_type::width,base_type::height);
        }
        // retrieves the bounds of the screen
        constexpr inline gfx::rect16 bounds() const {
            return gfx::rect16(gfx::point16(0,0),dimensions());
        }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type pixel) {
            spi_driver_result r = this->pixel_write(location.x,location.y,pixel.value());
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // asynchronously sets a point to the specified pixel
        gfx::gfx_result point_async(gfx::point16 location,pixel_type pixel) {
            spi_driver_result r = this->queued_pixel_write(location.x,location.y,pixel.value());
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // fills the specified rectangle with the specified pixel
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            spi_driver_result r=this->frame_fill(b,color.value());
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // asynchronously fills the specified rectangle with the specified pixel
        gfx::gfx_result fill_async(const gfx::rect16& bounds,pixel_type color) {
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            spi_driver_result r=this->queued_frame_fill(b,color.value());
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type p;
            return fill(bounds,p);
        }
        // asynchronously clears the specified rectangle
        inline gfx::gfx_result clear_async(const gfx::rect16& bounds) {
            pixel_type p;
            return fill_async(bounds,p);
        }
        // begins a batch operation for the specified rectangle
        gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            if(spi_driver_result::success!= this->batch_write_begin(b))
                return gfx::gfx_result::device_error;
            return gfx::gfx_result::success;
        }
        // asynchronously begins a batch operation for the specified rectangle
        gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            spi_driver_result r = this->queued_batch_write_begin(b);
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // writes a pixel to a pending batch
        gfx::gfx_result write_batch(pixel_type color) {
            uint16_t p = color.value();
            spi_driver_result r = this->batch_write(&p,1);
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // asynchronously writes a pixel to a pending batch
        gfx::gfx_result write_batch_async(pixel_type color) {
            uint16_t p = color.value();
            spi_driver_result r = this->queued_batch_write(&p,1);
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // commits a pending batch
        gfx::gfx_result commit_batch() {
            spi_driver_result r=this->batch_write_commit();
            if(spi_driver_result::success!=r)
                return xlt_err(r);
                
            return gfx::gfx_result::success;
        }
        // asynchronously commits a pending batch
        gfx::gfx_result commit_batch_async() {
            spi_driver_result r=this->queued_batch_write_commit();
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // copies source data to a frame
        template<typename Source> inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            return copy_from_impl(src_rect,src,location,false);
        }
        // asynchronously writes source data to a frame
        template<typename Source>
        inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            return copy_from_impl(src_rect,src,location,true);
        }
        // waits for all pending asynchronous operations to complete
        gfx::gfx_result wait_all_async() {
            spi_driver_result r=this->queued_wait_all();
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        
    };

}