#pragma once
#define HTCW_ST7735_OVERCLOCK
#include "common/spi_driver.hpp"
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

#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1
namespace arduino {
    namespace st7735_helpers {
        static const uint8_t generic_st7735[] =  {                // Init commands for 7735 screens
                                              // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
    ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      0xC8,                         //     row/col addr, bottom-top refresh
    ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 };                       //     16-bit color

     static const uint8_t generic_st7735_2[] = {                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay
    }
    
    // the driver for an ST7735 display
    template<uint16_t Width,
            uint16_t Height,
            int8_t PinCS,
            int8_t PinDC,
            int8_t PinRst,
            int8_t PinBacklight,
            size_t BatchBufferSize=64
            >
    struct st7735 final : 
            public spi_driver<Width,
                            Height,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7735_OVERCLOCK
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize> {
        using base_type = spi_driver<Width,
                            Height,
                            PinCS,
                            PinDC,
#ifdef HTCW_ST7735_OVERCLOCK
                            26*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize>;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        // the BL pin
        constexpr static const int8_t pin_backlight = PinBacklight;
        
    private:
        void send_init_commands(const uint8_t* addr) {
            uint8_t numCommands, cmd, numArgs;
            uint16_t ms;
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
        }
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
            send_init_commands(st7735_helpers::generic_st7735);
            this->send_command_init(ST77XX_CASET);
            uint8_t c_init_data[] = {0,0,0,Width-1};
            this->send_data_init(c_init_data,4);
            this->send_command_init(ST77XX_RASET);  //  6: Row addr set, 4 args, no delay:
            uint8_t r_init_data[] ={0,0,0,Height-1};
            this->send_data_init(r_init_data,4);
            send_init_commands(st7735_helpers::generic_st7735_2);
            //Enable backlight
            if(pin_backlight>=0) {
                digitalWrite(pin_backlight, HIGH);
            }
        
        }
        virtual void write_window(const spi_driver_rect& w, spi_driver_set_window_flags flags) {
            const uint16_t offsx = 2;
            const uint16_t offsy = 3;
            spi_driver_rect bounds;
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
        st7735(SPIClass& spi) : base_type(spi) {

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
        using type = st7735;
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
                spi_driver_rect drr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
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
                    spi_driver_rect dr = {dstr.x1,dstr.y1,dstr.x2,dstr.y2};
                    this_->frame_write(dr,src.begin()+(srcr.y1*src.dimensions().width*2));
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                while(yy<hh) {
                    spi_driver_rect dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.x2+yy)};
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
            spi_driver_result r = this->pixel_read(location.x,location.y,&pv);
            if(spi_driver_result::success!=r)
                return xlt_err(r);
            pixel->value(pv);
            return gfx::gfx_result::success;
        }
        */
        // fills the specified rectangle with the specified pixel
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
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
            spi_driver_rect b = {bounds.x1,bounds.y1,bounds.x2,bounds.y2};
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