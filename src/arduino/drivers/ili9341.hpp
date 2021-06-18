#pragma once
#define HTCW_ILI9341_OVERCLOCK
#include "common/tft_spi_driver.hpp"
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"

namespace arduino {
    namespace ili9341_helpers {
    struct init_cmd {
            uint8_t cmd;
            uint8_t data[16];
            uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
        };
    const init_cmd s_init_cmds[]={
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
    // the driver for an ILI9341 display
    template<int8_t PinCS,
            int8_t PinDC,
            int8_t PinRst,
            int8_t PinBacklight,
            size_t BatchBufferSize=64
            >
    struct ili9341 final : 
            public tft_spi_driver<320,
                            240,
                            PinCS,
                            PinDC,
#ifdef HTCW_ILI9341_OVERCLOCK
                            40*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize> {
        using base_type = tft_spi_driver<320,
                            240,
                            PinCS,
                            PinDC,
#ifdef HTCW_ILI9341_OVERCLOCK
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
            //Send all the commands
            int cmd=0;
            while (ili9341_helpers::s_init_cmds[cmd].databytes!=0xff) {
                this->send_command_init(ili9341_helpers::s_init_cmds[cmd].cmd);
                this->send_data_init(ili9341_helpers::s_init_cmds[cmd].data,ili9341_helpers::s_init_cmds[cmd].databytes&0x1F);
                if (ili9341_helpers::s_init_cmds[cmd].databytes&0x80) {
                    delay(100);
                }
                ++cmd;
            }
            //Enable backlight
            if(pin_backlight>=0) {
                digitalWrite(pin_backlight, HIGH);
            }
        
        }
        virtual void write_window(const tft_spi_driver_rect& bounds, tft_spi_driver_set_window_flags flags) {
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
        ili9341(SPIClass& spi) : base_type(spi) {

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
        using type = ili9341;
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