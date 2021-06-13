#include <Arduino.h>
#include "common/spi_driver.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"
#include "gfx_positioning.hpp"
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
namespace arduino {
namespace ssd1351_helpers {
        static const uint8_t generic_ssd1351[] =  {                
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
    0}; // END OF COMMAND LIST                         //    10 ms delay
    }
    
    // the driver for an SSD1351 display
    template<int8_t PinCS,
            int8_t PinDC,
            int8_t PinRst,
            size_t BatchBufferSize=64
            >
    struct ssd1351 final : 
            public spi_driver<128,
                            128,
                            PinCS,
                            PinDC,
#ifdef HTCW_SSD1351_OVERCLOCK
                            40*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize> {
        using base_type = spi_driver<128,
                            128,
                            PinCS,
                            PinDC,
#ifdef HTCW_SSD1351_OVERCLOCK
                            40*1000*1000,
#else
                            10*1000*1000,
#endif
                            BatchBufferSize>;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        
    private:
    protected:
        virtual void initialize_impl() {
            if(pin_rst>=0) {
                pinMode(pin_rst,OUTPUT);
            }
            
            reset();
            const uint8_t *addr = ssd1351_helpers::generic_ssd1351;
            uint8_t cmd, x, numArgs;
            while ((cmd = *(addr++)) > 0) { // '0' command ends list
                x = *(addr++);
                numArgs = x & 0x7F;
                if (cmd != 0xFF) { // '255' is ignored
                    this->send_command_init(cmd);
                    this->send_data_init(addr, numArgs);
                    
                }
                addr += numArgs;
            }
            
        }
        virtual void write_window(const spi_driver_rect& win, spi_driver_set_window_flags flags) {
            uint8_t tx_data[2];

            this->send_next_command(SSD1351_CMD_SETCOLUMN); // X range
            tx_data[0]=win.x1;
            tx_data[1]=win.x2;
            this->send_next_data(tx_data,2);
            this->send_next_command(SSD1351_CMD_SETROW); // Y range
            tx_data[0]=win.y1;
            tx_data[1]=win.y2;
            this->send_next_data(tx_data,2);
            this->send_next_command(SSD1351_CMD_WRITERAM);
        }
    public:
        ssd1351(SPIClass& spi) : base_type(spi) {

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
        using type = ssd1351;
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