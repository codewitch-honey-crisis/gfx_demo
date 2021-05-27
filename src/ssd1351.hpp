#pragma once
//#define HTCW_SSD1351_OVERCLOCK
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_master.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "spi_driver.hpp"

#define SSD1351_CMD_SETCOLUMN 0x15      ///< See datasheet
#define SSD1351_CMD_SETROW 0x75         ///< See datasheet
#define SSD1351_CMD_WRITERAM 0x5C       ///< See datasheet
#define SSD1351_CMD_READRAM 0x5D        ///< Not currently used
#define SSD1351_CMD_SETREMAP 0xA0       ///< See datasheet
#define SSD1351_CMD_STARTLINE 0xA1      ///< See datasheet
#define SSD1351_CMD_DISPLAYOFFSET 0xA2  ///< See datasheet
#define SSD1351_CMD_DISPLAYALLOFF 0xA4  ///< Not currently used
#define SSD1351_CMD_DISPLAYALLON 0xA5   ///< Not currently used
#define SSD1351_CMD_NORMALDISPLAY 0xA6  ///< See datasheet
#define SSD1351_CMD_INVERTDISPLAY 0xA7  ///< See datasheet
#define SSD1351_CMD_FUNCTIONSELECT 0xAB ///< See datasheet
#define SSD1351_CMD_DISPLAYOFF 0xAE     ///< See datasheet
#define SSD1351_CMD_DISPLAYON 0xAF      ///< See datasheet
#define SSD1351_CMD_PRECHARGE 0xB1      ///< See datasheet
#define SSD1351_CMD_DISPLAYENHANCE 0xB2 ///< Not currently used
#define SSD1351_CMD_CLOCKDIV 0xB3       ///< See datasheet
#define SSD1351_CMD_SETVSL 0xB4         ///< See datasheet
#define SSD1351_CMD_SETGPIO 0xB5        ///< See datasheet
#define SSD1351_CMD_PRECHARGE2 0xB6     ///< See datasheet
#define SSD1351_CMD_SETGRAY 0xB8        ///< Not currently used
#define SSD1351_CMD_USELUT 0xB9         ///< Not currently used
#define SSD1351_CMD_PRECHARGELEVEL 0xBB ///< Not currently used
#define SSD1351_CMD_VCOMH 0xBE          ///< See datasheet
#define SSD1351_CMD_CONTRASTABC 0xC1    ///< See datasheet
#define SSD1351_CMD_CONTRASTMASTER 0xC7 ///< See datasheet
#define SSD1351_CMD_MUXRATIO 0xCA       ///< See datasheet
#define SSD1351_CMD_COMMANDLOCK 0xFD    ///< See datasheet
#define SSD1351_CMD_HORIZSCROLL 0x96    ///< Not currently used
#define SSD1351_CMD_STOPSCROLL 0x9E     ///< Not currently used
#define SSD1351_CMD_STARTSCROLL 0x9F    ///< Not currently used

namespace espidf {
    // the driver for an SSD1351 display
    template<spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinDC,
            gpio_num_t PinRst,
            size_t MaxTransactions=7,
            bool UsePolling = true,
            size_t DmaSize = -1,
            TickType_t Timeout=5000/portTICK_PERIOD_MS,
            size_t BatchBufferSize=64
            >
    struct ssd1351 : 
            public spi_driver<128,
                            128,
                            HostId,
                            PinCS,
                            PinDC,
#ifdef HTCW_SSD1351_OVERCLOCK
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            MaxTransactions,
                            UsePolling,
                            DmaSize,
                            Timeout,
                            BatchBufferSize> {
        using base_type = spi_driver<128,
                            128,
                            HostId,
                            PinCS,
                            PinDC,
#ifdef HTCW_SSD1351_OVERCLOCK
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
        // the maximum number of "in the air" transactions that can be queued
        constexpr static const size_t max_transactions = (0==MaxTransactions)?1:MaxTransactions;
    private:
        DRAM_ATTR static const uint8_t s_init_cmds[];
        spi_driver_result write_window_impl(const spi_driver_rect& win,bool queued,spi_driver_set_window_flags set_flags) {
            spi_driver_result r;
            uint8_t tx_data[2];

            r=this->send_next_command(SSD1351_CMD_SETCOLUMN,queued); // X range
            if(spi_driver_result::success!=r)
                return r;
            tx_data[0]=win.x1;
            tx_data[1]=win.x2;
            r=this->send_next_data(tx_data,2,queued);
            if(spi_driver_result::success!=r)
                return r;
            r=this->send_next_command(SSD1351_CMD_SETROW,queued); // Y range
            if(spi_driver_result::success!=r)
                return r;
            tx_data[0]=win.y1;
            tx_data[1]=win.y2;
            r=this->send_next_data(tx_data,2,queued);
            if(spi_driver_result::success!=r)
                return r;
            return this->send_next_command(SSD1351_CMD_WRITERAM,queued);
        }
    public:
        // constructs a new instance of the driver
        ssd1351() {
            
        }
        virtual ~ssd1351() {}
        // forces initialization of the driver
        spi_driver_result initialize()
        {
            if(!this->initialized()) {
                static const TickType_t ts = 100/portTICK_RATE_MS;

                
                //Initialize non-SPI GPIOs
                gpio_set_direction(base_type::pin_dc, GPIO_MODE_OUTPUT);
                gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
                
                //Reset the display
                gpio_set_level(pin_rst, 0);
                vTaskDelay(ts);
                gpio_set_level(pin_rst, 1);
                vTaskDelay(ts);
                
                const uint8_t *addr = s_init_cmds;
                uint8_t cmd, x, numArgs;
                spi_driver_result r;
                while ((cmd = *(addr++)) > 0) { // '0' command ends list
                    x = *(addr++);
                    numArgs = x & 0x7F;
                    if (cmd != 0xFF) { // '255' is ignored
                        r=this->send_init_command(cmd);
                        if(spi_driver_result::success!=r)
                            return r;
                        r=this->send_init_data(addr, numArgs);
                        if(spi_driver_result::success!=r)
                            return r;
                    }
                    addr += numArgs;
                }
                
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
        using type = ssd1351<HostId,PinCS,PinDC,PinRst,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,true,true,true,false,false,false>;
 
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
     template<spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinDC,
            gpio_num_t PinRst,
            size_t MaxTransactions,
            bool UsePolling,
            size_t DmaSize,
            TickType_t Timeout,
            size_t BatchBufferSize
            >
    const uint8_t ssd1351<HostId,PinCS,PinDC,PinRst,MaxTransactions,UsePolling,DmaSize,Timeout,BatchBufferSize>::s_init_cmds[]={
     SSD1351_CMD_COMMANDLOCK,
    1, // Set command lock, 1 arg
    0x12,
    SSD1351_CMD_COMMANDLOCK,
    1, // Set command lock, 1 arg
    0xB1,
    SSD1351_CMD_DISPLAYOFF,
    0, // Display off, no args
    SSD1351_CMD_CLOCKDIV,
    1,
    0xF1, // 7:4 = Oscillator Freq, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    SSD1351_CMD_MUXRATIO,
    1,
    127,
    SSD1351_CMD_DISPLAYOFFSET,
    1,
    0x0,
    SSD1351_CMD_SETGPIO,
    1,
    0x00,
    SSD1351_CMD_FUNCTIONSELECT,
    1,
    0x01, // internal (diode drop)
    SSD1351_CMD_PRECHARGE,
    1,
    0x32,
    SSD1351_CMD_VCOMH,
    1,
    0x05,
    SSD1351_CMD_NORMALDISPLAY,
    0,
    SSD1351_CMD_SETREMAP,
    1,
    0b01110100,
    SSD1351_CMD_STARTLINE,
    1,
    128,
    SSD1351_CMD_CONTRASTABC,
    3,
    0xC8,
    0x80,
    0xC8,
    SSD1351_CMD_CONTRASTMASTER,
    1,
    0x0F,
    SSD1351_CMD_SETVSL,
    3,
    0xA0,
    0xB5,
    0x55,
    SSD1351_CMD_PRECHARGE2,
    1,
    0x01,
    SSD1351_CMD_DISPLAYON,
    0,  // Main screen turn on
    0}; // END OF COMMAND LIST
}