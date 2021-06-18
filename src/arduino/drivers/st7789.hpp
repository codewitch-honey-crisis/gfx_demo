#pragma once
#define HTCW_ST7789_OVERCLOCK
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
namespace arduino {
    namespace st7789_helpers {
        static const uint8_t generic_st7789[] =  {                // Init commands for 7789 screens
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
            10 
        };                          //    10 ms delay
    }
    
    // the driver for an ST7789 display
    template<uint16_t Width,
            uint16_t Height,
            int8_t PinCS,
            int8_t PinDC,
            int8_t PinRst,
            int8_t PinBacklight,
            size_t BatchBufferSize=64
            >
    struct st7789 final : 
            public tft_spi_driver<Width,
                            Height,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7789_OVERCLOCK
                            40*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize> {
        using base_type = tft_spi_driver<Width,
                            Height,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7789_OVERCLOCK
                            40*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize>;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        // the BL pin
        constexpr static const int8_t pin_backlight = PinBacklight;
        
    private:
    protected:
        virtual void initialize_impl() {
            if(pin_rst>=0) {
                pinMode(pin_rst,OUTPUT);
            }
            if(pin_backlight>=0) {
                pinMode(pin_backlight,OUTPUT);
                digitalWrite(pin_backlight,HIGH);
            }
            reset();
            uint8_t numCommands, cmd, numArgs;
            uint16_t ms;
            const uint8_t* addr = st7789_helpers::generic_st7789;
            numCommands = *(addr++); // Number of commands to follow
            while (numCommands--) {              // For each command...
                cmd = *(addr++);       // Read command
                numArgs = *(addr++);   // Number of args to follow
                ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
                numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
                this->send_command_init(cmd);
                this->send_data_init(addr, numArgs);
                addr += numArgs;
                if (ms) {
                    ms = *(addr++); // Read post-command delay time (ms)
                    if (ms == 255)
                        ms = 500; // If 255, delay for 500 ms
                    delay(ms);
                }
            }
            this->send_command_init(ST77XX_RASET);  //  6: Row addr set, 4 args, no delay:
            uint8_t init_data[] ={0,0,Width>>8,Height&0xFF};
            this->send_data_init(init_data,4);
            this->send_command_init(ST77XX_MADCTL);  
            cmd = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;//ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
            this->send_data_init(&cmd,1);
            //Enable backlight
            if(pin_backlight>=0) {
                digitalWrite(pin_backlight, HIGH);
            }
        
        }
        virtual void write_window(const tft_spi_driver_rect& w, tft_spi_driver_set_window_flags flags) {
            const uint16_t offsx = (320-Width)/2;
            const uint16_t offsy = (240-Height)/2;
            tft_spi_driver_rect bounds;
            bounds.x1=w.x1+offsx;
            bounds.x2=w.x2+offsx;
            bounds.y1=w.y1+offsy;
            bounds.y2=w.y2+offsy;

            uint8_t tx_data[4];
            //Column Address Set
            this->send_next_command(0x2A);
            if(flags.x1 || flags.x2) {
                tx_data[0]=bounds.x1>>8;             //Start Col High
                tx_data[1]=bounds.x1&0xFF;           //Start Col Low
                tx_data[2]=bounds.x2>>8;             //End Col High
                tx_data[3]=bounds.x2&0xff;           //End Col Low
                this->send_next_data(tx_data,4,true);
            }
            if(flags.y1 || flags.y2 || !(flags.x1 || flags.x2)) {
                //Page address set
                this->send_next_command(0x2B,true);
                tx_data[0]=bounds.y1>>8;        //Start page high
                tx_data[1]=bounds.y1&0xff;      //start page low
                tx_data[2]=bounds.y2>>8;        //end page high
                tx_data[3]=bounds.y2&0xff;      //end page low
                this->send_next_data(tx_data,4,true);
            }
            // Memory write
            this->send_next_command(0x2C,true);
        }
    public:
        st7789(SPIClass& spi) : base_type(spi) {

        }
        void reset() {
            if (pin_rst >= 0)
            {
                if(!this->initialized()) {
                    pinMode(pin_rst,OUTPUT);
                }
                delay(20);
                digitalWrite(pin_rst, LOW);
                delay(20);
                digitalWrite(pin_rst, HIGH);
                delay(200);
            }
        }
        // GFX bindings
 public:
        // indicates the type, itself
        using type = st7789;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,false,true,true,false,false,false>;
 
 private:
        template<typename Source,bool Blt> 
        struct copy_from_helper {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr)  {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                tft_spi_driver_rect drr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                this_->batch_write_begin(drr);
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
                        this_->batch_write(&pv,1);
                    }
                }
                this_->batch_write_commit();
                return gfx::gfx_result::success;
            }
        };
        
        template<typename Source> 
        struct copy_from_helper<Source,true> {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr) {
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    tft_spi_driver_rect dr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                    this_->frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                while(yy<hh) {
                    tft_spi_driver_rect dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.x2+yy)};
                    this_->frame_write(dr,src.begin()+(ww*(srcr.y1+yy)+srcr.x1));
                    ++yy;
                }
                return gfx::gfx_result::success;
            }
        };
        template<typename Source>
        gfx::gfx_result copy_from_impl(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
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
            ::do_draw(this,dstr,src,srcr);
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
            this->pixel_write(location.x,location.y,pixel.value());
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
        // fills the specified rectangle with the specified pixel
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            this->frame_fill(b,color.value());
            return gfx::gfx_result::success;
        }
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type p;
            return fill(bounds,p);
        }
        // begins a batch operation for the specified rectangle
        inline gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
            tft_spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
            this->batch_write_begin(b);
            return gfx::gfx_result::success;
        }
        // writes a pixel to a pending batch
        gfx::gfx_result write_batch(pixel_type color) {
            uint16_t p = color.value();
            this->batch_write(&p,1);
            return gfx::gfx_result::success;
        }
        // commits a pending batch
        inline gfx::gfx_result commit_batch() {
            this->batch_write_commit();
            return gfx::gfx_result::success;
        }
        // copies source data to a frame
        template<typename Source> inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            return copy_from_impl(src_rect,src,location);
        }
        
    };
}