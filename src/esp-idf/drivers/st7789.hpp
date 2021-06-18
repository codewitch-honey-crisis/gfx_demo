#pragma once
#define HTCW_ST7789_OVERCLOCK
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "common/tft_spi_driver.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"
#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1 0xDA
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD
namespace espidf {
     namespace st7789_helpers {
        DRAM_ATTR static const uint8_t generic_st7789[] =  {                // Init commands for 7789 screens
    9,                              //  9 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      150,                          //     ~150 ms delay
    ST77XX_SLPOUT ,   ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      10,                          //      10 ms delay
    ST77XX_COLMOD , 1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x55,                         //     16-bit color
      10,                           //     10 ms delay
    ST77XX_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST77XX_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
      0x00,
      0,        //     XSTART = 0
      0,
      240,  //     XEND = 240
    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
      0x00,
      0,             //     YSTART = 0
      320>>8,
      320&0xFF,  //     YEND = 320
    ST77XX_INVON  ,   ST_CMD_DELAY,  //  7: hack
      10,
    ST77XX_NORON  ,   ST_CMD_DELAY, //  8: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON ,   ST_CMD_DELAY, //  9: Main screen turn on, no args, delay
      10 };                          //    10 ms delay
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
            public tft_spi_driver<Width,
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
        using base_type = tft_spi_driver<Width,
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
        constexpr static const uint16_t column_start = (Width<240)?((240-Width+1)/2):(Width==240&&Height==280)?0:(240-Width);
        constexpr static const uint16_t column_start2 = (Width<240)?((240-Width)/2):(Width==240&&Height==280)?0:(240-Width);
        constexpr static const uint16_t row_start = (Width<240)?((320-Height+1)/2):(Width==240&&Height==280)?20:(320-Height);
        constexpr static const uint16_t row_start2 = (Width<240)?((320-Height+1)/2):0;
         
        // the RST pin
        constexpr static const gpio_num_t pin_rst = PinRst;
        // the BL pin
        constexpr static const gpio_num_t pin_backlight = PinBacklight;
        // the maximum number of "in the air" transactions that can be queued
        constexpr static const size_t max_transactions = (0==MaxTransactions)?1:MaxTransactions;
    private:
       
        tft_spi_driver_result write_window_impl(const tft_spi_driver_rect& w,bool queued,tft_spi_driver_set_window_flags set_flags) {
            //printf("(%d, %d)-(%d, %d)\r\n",win.x1,win.y1,win.x2,win.y2);
            tft_spi_driver_rect win;
            const uint16_t offsx = (320-Width)/2;
            const uint16_t offsy = (240-Height)/2;
            win.x1=w.x1+offsx;
            win.x2=w.x2+offsx;
            win.y1=w.y1+offsy;
            win.y2=w.y2+offsy;
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
        st7789() {
            
        }
        virtual ~st7789() {}
        // forces initialization of the driver
        tft_spi_driver_result initialize()
        {
            if(!this->initialized()) {
                static const TickType_t ts = 100/portTICK_RATE_MS;

                
                //Initialize non-SPI GPIOs
                gpio_set_direction(base_type::pin_dc, GPIO_MODE_OUTPUT);
                gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
                gpio_set_direction(pin_backlight, GPIO_MODE_OUTPUT);

                //Reset the display
                gpio_set_level(pin_rst, 0);
                vTaskDelay(ts);
                gpio_set_level(pin_rst, 1);
                vTaskDelay(ts);
                
                uint8_t numCommands, cmd, numArgs;
                uint16_t ms;
                const uint8_t* addr = st7789_helpers::generic_st7789;
                tft_spi_driver_result r;
                numCommands = *(addr++); // Number of commands to follow
                while (numCommands--) {              // For each command...
                    cmd = *(addr++);       // Read command
                    numArgs = *(addr++);   // Number of args to follow
                    ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
                    numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
                    r= this->send_init_command(cmd);
                    if(tft_spi_driver_result::success!=r) {
                        return r;
                    }
                    r= this->send_init_data(addr, numArgs);
                    if(tft_spi_driver_result::success!=r) {
                        return r;
                    }
                    addr += numArgs;
                    if (ms) {
                        ms = *(addr++); // Read post-command delay time (ms)
                        if (ms == 255)
                            ms = 500; // If 255, delay for 500 ms
                        vTaskDelay(ms/portTICK_PERIOD_MS);
                    }
                }
                r= this->send_init_command(ST77XX_RASET);  //  6: Row addr set, 4 args, no delay:
                if(tft_spi_driver_result::success!= r) {
                    return r;
                }
                uint8_t init_data[] ={0,0,Width>>8,Height&0xFF};
                r=this->send_init_data(init_data,4);
                if(tft_spi_driver_result::success!= r) {
                    return r;
                }
                r= this->send_init_command(ST77XX_MADCTL);  
                cmd = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;//ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
                r=this->send_init_data(&cmd,1);
                if(tft_spi_driver_result::success!=r) {
                    return r;
                }
                ///Enable backlight
                gpio_set_level(pin_backlight, 1);
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

        // GFX bindings
 public:
        // indicates the type, itself
        using type = st7789<Width,Height,HostId,PinCS,PinDC,PinRst,PinBacklight,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,true,true,true,false,false,false>;
 
 private:
        gfx::gfx_result xlt_err(tft_spi_driver_result r) {
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
        template<typename Source>
        gfx::gfx_result copy_from_impl(const gfx::rect16& src_rect,const Source& src,gfx::point16 location,bool async) {
            tft_spi_driver_result r;
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
                    tft_spi_driver_rect dr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                    if(!async)
                        r=this->frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    else
                        r=this->queued_frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
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
                    if(!async)
                        r = this->frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    else
                        r = this->queued_frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    if(tft_spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                    ++yy;
                }
                return gfx::gfx_result::success;
            }
            uint16_t w = dstr.dimensions().width;
            uint16_t h = dstr.dimensions().height;
            tft_spi_driver_rect drr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
            if(!async)
                r=this->batch_write_begin(drr);
            else
                r=this->queued_batch_write_begin(drr);
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
                        r = this->batch_write(&pv,1);
                    else
                        r = this->queued_batch_write(&pv,1);
                    if(tft_spi_driver_result::success!=r) {
                        return xlt_err(r);
                    }
                }
            }
            if(!async)
                r=this->batch_write_commit();
            else
                r=this->queued_batch_write_commit();
            if(tft_spi_driver_result::success!=r) {
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
            tft_spi_driver_result r = this->pixel_write(location.x,location.y,pixel.value());
            if(tft_spi_driver_result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
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

}