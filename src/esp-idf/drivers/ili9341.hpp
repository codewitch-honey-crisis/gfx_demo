#pragma once
#define HTCW_ILI9341_OVERCLOCK_26
//#define HTCW_ILI9341_OVERCLOCK_40
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "common/tft_spi_driver.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"

namespace espidf {
    // the driver for an ILI9341 display
    template<spi_host_device_t HostId,
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
    struct ili9341 final : 
            public tft_spi_driver<320,
                            240,
                            HostId,
                            PinCS,
                            PinDC,
#if defined(HTCW_ILI9341_OVERCLOCK_40)
                            40*1000*1000,
#elif defined(HTCW_ILI9341_OVERCLOCK_26)
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            MaxTransactions,
                            UsePolling,
                            DmaSize,
                            Timeout,
                            BatchBufferSize> {
        using base_type = tft_spi_driver<320,
                            240,
                            HostId,
                            PinCS,
                            PinDC,
#if defined(HTCW_ILI9341_OVERCLOCK_40)
                            40*1000*1000,
#elif defined(HTCW_ILI9341_OVERCLOCK_26)
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
        struct init_cmd {
            uint8_t cmd;
            uint8_t data[16];
            uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
        };
        DRAM_ATTR static const init_cmd s_init_cmds[];
        tft_spi_driver_result write_window_impl(const tft_spi_driver_rect& win,bool queued,tft_spi_driver_set_window_flags set_flags) {
            //printf("(%d, %d)-(%d, %d)\r\n",win.x1,win.y1,win.x2,win.y2);
            
            tft_spi_driver_result r;
            uint8_t tx_data[4];
            //Column Address Set
            r=this->send_next_command(0x2A,queued);
            if(tft_spi_driver_result::success!=r)
                return r;
            if(set_flags.x1 || set_flags.x2) {
                tx_data[0]=win.x1>>8;             //Start Col High
                tx_data[1]=win.x1&0xFF;           //Start Col Low
                tx_data[2]=win.x2>>8;             //End Col High
                tx_data[3]=win.x2&0xff;           //End Col Low
                r=this->send_next_data(tx_data,4,queued,true);
                if(tft_spi_driver_result::success!=r)
                    return r;
            }
            if(set_flags.y1 || set_flags.y2 || !(set_flags.x1 || set_flags.x2)) {
                //Page address set
                r=this->send_next_command(0x2B,queued,true);
                if(tft_spi_driver_result::success!=r)
                    return r;
                tx_data[0]=win.y1>>8;        //Start page high
                tx_data[1]=win.y1&0xff;      //start page low
                tx_data[2]=win.y2>>8;        //end page high
                tx_data[3]=win.y2&0xff;      //end page low
                r=this->send_next_data(tx_data,4,queued,true);
                if(tft_spi_driver_result::success!=r)
                    return r;
            }
            // Memory write
            return this->send_next_command(0x2C,queued,true);
        }
        
    public:
        // constructs a new instance of the driver
        ili9341() {
            
        }
        virtual ~ili9341() {}
        // forces initialization of the driver
        tft_spi_driver_result initialize()
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
                while (s_init_cmds[cmd].databytes!=0xff) {
                    tft_spi_driver_result r = this->send_init_command(s_init_cmds[cmd].cmd);
                    if(tft_spi_driver_result::success!= r) {
                        return r;
                    }
                    r=this->send_init_data(s_init_cmds[cmd].data,s_init_cmds[cmd].databytes&0x1F);
                    if(tft_spi_driver_result::success!= r) {
                        return tft_spi_driver_result::io_error;
                    }
                    if (s_init_cmds[cmd].databytes&0x80) {
                        vTaskDelay(ts);
                    }
                    ++cmd;
                }
                ///Enable backlight
                gpio_set_level(pin_backlight, 0);
            }
            return tft_spi_driver_result::success;
        }
protected:
        virtual tft_spi_driver_result write_window(const tft_spi_driver_rect& bounds,tft_spi_driver_set_window_flags set_flags) {
            return write_window_impl(bounds,false,set_flags);
        }
        virtual tft_spi_driver_result queued_write_window(const tft_spi_driver_rect& bounds,tft_spi_driver_set_window_flags set_flags) {
            return write_window_impl(bounds,true,set_flags);
        }
        virtual tft_spi_driver_result read_window(const tft_spi_driver_rect& win) {
            //printf("(%d, %d)-(%d, %d)\r\n",win.x1,win.y1,win.x2,win.y2);
            
            tft_spi_driver_result r;
            uint8_t tx_data[4];
            //Column Address Set
            r=this->send_next_command(0x2A,false);
            if(tft_spi_driver_result::success!=r)
                return r;
            
            tx_data[0]=win.x1>>8;             //Start Col High
            tx_data[1]=win.x1&0xFF;           //Start Col Low
            tx_data[2]=win.x2>>8;             //End Col High
            tx_data[3]=win.x2&0xff;           //End Col Low
            r=this->send_next_data(tx_data,4,false,true);
            if(tft_spi_driver_result::success!=r)
                return r;
        
            //Page address set
            r=this->send_next_command(0x2B,false,true);
            if(tft_spi_driver_result::success!=r)
                return r;
            tx_data[0]=win.y1>>8;        //Start page high
            tx_data[1]=win.y1&0xff;      //start page low
            tx_data[2]=win.y2>>8;        //end page high
            tx_data[3]=win.y2&0xff;      //end page low
            r=this->send_next_data(tx_data,4,false,true);
            if(tft_spi_driver_result::success!=r)
                return r;
    
            // Memory read
            r= this->send_next_command(0x2E,false,true);
            if(tft_spi_driver_result::success!=r)
                return r;
            // dummy read (1st parameter)
            uint8_t val;
            return this->read_next_data(&val,1);
        }
        // GFX bindings
 public:
        // indicates the type, itself
        using type = ili9341<HostId,PinCS,PinDC,PinRst,PinBacklight,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,true,true,true,false,false,false>;
 
 private:
        static gfx::gfx_result xlt_err(tft_spi_driver_result r) {
            switch(r) {
                case tft_spi_driver_result::io_error:
                    return gfx::gfx_result::device_error;
                case tft_spi_driver_result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case tft_spi_driver_result::success:
                    return gfx::gfx_result::success;
                default:
                    return gfx::gfx_result::invalid_argument;
            }
        }
        template<typename Source,bool Blt> 
        struct copy_from_helper {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async)  {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                tft_spi_driver_rect drr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                tft_spi_driver_result r;
                if(!async)
                    r=this_->batch_write_begin(drr);
                else
                    r=this_->queued_batch_write_begin(drr);
                if(tft_spi_driver_result::success!=r) {
                    return xlt_err(r);
                }
                for(uint16_t y=0;y<h;++y) {
                    for(uint16_t x=0;x<w;++x) {
                        typename Source::pixel_type pp;
                        gfx::gfx_result rr=src.point(gfx::point16(x+srcr.x1,y+srcr.y1), &pp);
                        if(rr!=gfx::gfx_result::success)
                            return rr;
                        pixel_type p;
                        rr=gfx::convert_palette_to(src,pp,&p);
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                        uint16_t pv = p.value();
                        if(!async)
                            r = this_->batch_write(&pv,1);
                        else
                            r = this_->queued_batch_write(&pv,1);
                        if(tft_spi_driver_result::success!=r) {
                            return xlt_err(r);
                        }
                    }
                }
                if(!async)
                    r=this_->batch_write_commit();
                else
                    r=this_->queued_batch_write_commit();
                if(tft_spi_driver_result::success!=r) {
                    return xlt_err(r);
                }
                return gfx::gfx_result::success;
            }
        };
        
        template<typename Source> 
        struct copy_from_helper<Source,true> {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async) {
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    tft_spi_driver_rect dr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                    tft_spi_driver_result r;
                    if(!async)
                        r=this_->frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    else
                        r=this_->queued_frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    if(tft_spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                while(yy<hh) {
                    tft_spi_driver_rect dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.x2+yy)};
                    tft_spi_driver_result r;
                    if(!async)
                        r = this_->frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    else
                        r = this_->queued_frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    if(tft_spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                    ++yy;
                }
                return gfx::gfx_result::success;
            }
        };
        template<typename Source>
        gfx::gfx_result copy_from_impl(const gfx::rect16& src_rect,const Source& src,gfx::point16 location,bool async) {
            gfx::rect16 srcr = src_rect.normalize().crop(src.bounds());
            gfx::rect16 dstr(location,src_rect.dimensions());
            dstr=dstr.crop(bounds());
            if(srcr.width()>dstr.width()) {
                srcr.x2=srcr.x1+dstr.width()-1;
            }
            if(srcr.height()>dstr.height()) {
                srcr.y2=srcr.y1+dstr.height()-1;
            }
            return copy_from_helper<Source,gfx::helpers::is_same<pixel_type,typename Source::pixel_type>::value && Source::caps::blt>
            ::do_draw(this,dstr,src,srcr,async);
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
            tft_spi_driver_result r = this->pixel_write(location.x,location.y,pixel.value());
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        /*
        // gets a pixel from the specified point
        gfx::gfx_result point(gfx::point16 location,pixel_type* pixel) {
            if(nullptr==pixel)
                return gfx::gfx_result::invalid_argument;
            uint16_t pv;
            tft_spi_driver_result r = this->pixel_read(location.x,location.y,&pv);
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            pixel->value(pv);
            return gfx::gfx_result::success;
        }
        */
        // asynchronously sets a point to the specified pixel
        gfx::gfx_result point_async(gfx::point16 location,pixel_type pixel) {
            tft_spi_driver_result r = this->queued_pixel_write(location.x,location.y,pixel.value());
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // fills the specified rectangle with the specified pixel
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            tft_spi_driver_result r=this->frame_fill(b,color.value());
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // asynchronously fills the specified rectangle with the specified pixel
        gfx::gfx_result fill_async(const gfx::rect16& bounds,pixel_type color) {
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            tft_spi_driver_result r=this->queued_frame_fill(b,color.value());
            if(tft_spi_driver_result::success!=r)
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
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            if(tft_spi_driver_result::success!= this->batch_write_begin(b))
                return gfx::gfx_result::device_error;
            return gfx::gfx_result::success;
        }
        // asynchronously begins a batch operation for the specified rectangle
        gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            tft_spi_driver_result r = this->queued_batch_write_begin(b);
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // writes a pixel to a pending batch
        gfx::gfx_result write_batch(pixel_type color) {
            uint16_t p = color.value();
            tft_spi_driver_result r = this->batch_write(&p,1);
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // asynchronously writes a pixel to a pending batch
        gfx::gfx_result write_batch_async(pixel_type color) {
            uint16_t p = color.value();
            tft_spi_driver_result r = this->queued_batch_write(&p,1);
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        // commits a pending batch
        gfx::gfx_result commit_batch() {
            tft_spi_driver_result r=this->batch_write_commit();
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
                
            return gfx::gfx_result::success;
        }
        // asynchronously commits a pending batch
        gfx::gfx_result commit_batch_async() {
            tft_spi_driver_result r=this->queued_batch_write_commit();
            if(tft_spi_driver_result::success!=r)
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
            tft_spi_driver_result r=this->queued_wait_all();
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        
    };
     template<spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinDC,
            gpio_num_t PinRst,
            gpio_num_t PinBacklight,
            size_t MaxTransactions,
            bool UsePolling,
            size_t DmaSize,
            TickType_t Timeout,
            size_t BatchBufferSize
            >
    const typename ili9341<HostId,PinCS,PinDC,PinRst,PinBacklight,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>::init_cmd ili9341<HostId,PinCS,PinDC,PinRst,PinBacklight,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>::s_init_cmds[]={
            /* Power contorl B, power control = 0, DC_ENA = 1 */
            {0xCF, {0x00, 0x83, 0X30}, 3},
            /* Power on sequence control,
            * cp1 keeps 1 frame, 1st frame enable
            * vcl = 0, ddvdh=3, vgh=1, vgl=2
            * DDVDH_ENH=1
            */
            {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
            /* Driver timing control A,
            * non-overlap=default +1
            * EQ=default - 1, CR=default
            * pre-charge=default - 1
            */
            {0xE8, {0x85, 0x01, 0x79}, 3},
            /* Power control A, Vcore=1.6V, DDVDH=5.6V */
            {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
            /* Pump ratio control, DDVDH=2xVCl */
            {0xF7, {0x20}, 1},
            /* Driver timing control, all=0 unit */
            {0xEA, {0x00, 0x00}, 2},
            /* Power control 1, GVDD=4.75V */
            {0xC0, {0x26}, 1},
            /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
            {0xC1, {0x11}, 1},
            /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
            {0xC5, {0x35, 0x3E}, 2},
            /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
            {0xC7, {0xBE}, 1},
            /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
            {0x36, {0x28}, 1},
            /* Pixel format, 16bits/pixel for RGB/MCU interface */
            {0x3A, {0x55}, 1},
            /* Frame rate control, f=fosc, 70Hz fps */
            {0xB1, {0x00, 0x1B}, 2},
            /* Enable 3G, disabled */
            {0xF2, {0x08}, 1},
            /* Gamma set, curve 1 */
            {0x26, {0x01}, 1},
            /* Positive gamma correction */
            {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
            /* Negative gamma correction */
            {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
            /* Column address set, SC=0, EC=0xEF */
            {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
            /* Page address set, SP=0, EP=0x013F */
            {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
            /* Memory write */
            {0x2C, {0}, 0},
            /* Entry mode set, Low vol detect disabled, normal display */
            {0xB7, {0x07}, 1},
            /* Display function control */
            {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
            /* Sleep out */
            {0x11, {0}, 0x80},
            /* Display on */
            {0x29, {0}, 0x80},
            {0, {0}, 0xff},
        };
}