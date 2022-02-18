#include <Arduino.h>
#include "common/tft_driver.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"
#include "gfx_positioning.hpp"

namespace arduino {
namespace ssd1351_helpers {
    }
    
    // the driver for an SSD1351 display
    template<int8_t PinDC,
            int8_t PinRst,
            typename Bus
            >
    struct ssd1351 final {
        // the DC pin
        constexpr static const int8_t pin_dc = PinDC;    
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const uint8_t rotation = 0;
        constexpr static const size_t max_dma_size = 128*128*2;
        using type = ssd1351;
        using driver = tft_driver<PinDC, PinRst,-1,Bus>;
        using bus = Bus;
        using pixel_type = gfx::rgb_pixel<16>;
        using caps = gfx::gfx_caps<false,(bus::dma_size>0),true,true,false,bus::readable,false>;
        
    private:
        bool m_initialized;
        bool m_dma_initialized;
        bool m_in_batch;
        static void set_window(const gfx::rect16& bounds, bool read=false) {
            bus::busy_check();
            driver::dc_command();
            bus::write_raw8(0x15);
            driver::dc_data();
            bus::write_raw8(uint8_t(bounds.x1));
            bus::write_raw8(uint8_t(bounds.x2));
            driver::dc_command();
            bus::write_raw8(0x75);
            driver::dc_data();
            bus::write_raw8(uint8_t(bounds.y1));
            bus::write_raw8(uint8_t(bounds.y2));
            driver::dc_command();
            bus::write_raw8(read?0x5D:0x5C);
            driver::dc_data();
        }
        template<typename Source,bool Blt> 
        struct copy_from_helper {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async)  {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                gfx::gfx_result rr;
               
                rr=this_->begin_batch(dstr);
                
                if(gfx::gfx_result::success!=rr) {
                    return rr;
                }
                for(uint16_t y=0;y<h;++y) {
                    for(uint16_t x=0;x<w;++x) {
                        typename Source::pixel_type pp;
                        rr=src.point(gfx::point16(x+srcr.x1,y+srcr.y1), &pp);
                        if(rr!=gfx::gfx_result::success)
                            return rr;
                        pixel_type p;
                        rr=gfx::convert_palette_to(src,pp,&p,nullptr);
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                       
                        rr = this_->write_batch(p);
                        
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                    }
                }
                
                rr=this_->batch_commit();
                
                return rr;
            }
        };
        
        template<typename Source> 
        struct copy_from_helper<Source,true> {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async) {
                if(async) {
                    bus::dma_wait();
                }
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    bus::begin_write();
                    set_window(dstr);
                    if(async) {
                        bus::write_raw_dma(src.begin()+(srcr.y1*src.dimensions().width*2),(srcr.y2-srcr.y1+1)*src.dimensions().width*2);
                    } else {
                        bus::write_raw(src.begin()+(srcr.y1*src.dimensions().width*2),(srcr.y2-srcr.y1+1)*src.dimensions().width*2);
                    }

                    bus::end_write();
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                uint16_t pitch = (srcr.x2 - srcr.x1+1)*2;
                bus::begin_write();
                bus::begin_transaction();
                while(yy<hh-!!async) {
                    gfx::rect16 dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.y1+yy)};
                    set_window(dr);
                    bus::write_raw(src.begin()+2*(ww*(srcr.y1+yy)+srcr.x1),pitch);
                    ++yy;
                }
                if(async) {
                    gfx::rect16 dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.y1+yy)};
                    set_window(dr);
                    bus::write_raw_dma(src.begin()+2*(ww*(srcr.y1+yy)+srcr.x1),pitch);
                }
                bus::end_transaction();
                bus::end_write();
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
        static void apply_rotation() {
            bus::begin_write();
            uint8_t madctl = 0x64;

            switch (rotation) {
                case 0:
                madctl |= 0x10;
                break;
                case 1:
                madctl |= 0x13;
                break;
                case 2:
                madctl |= 0x02;
                break;
                case 3:
                madctl |= 0x01;
                break;
            }
            driver::send_command(0xA0);
            driver::send_data8(madctl);
            driver::send_command(0xA1);
            driver::send_data8(uint8_t(rotation < 2 ? 128 : 0));
            delayMicroseconds(20);
            bus::end_write();
        }
    public:
        ssd1351() : m_initialized(false), m_dma_initialized(false),m_in_batch(false) {

        }
        ~ssd1351() {
            if(m_dma_initialized) {
                bus::deinitialize_dma();
            }
            if(m_initialized) {
                driver::deinitialize();
            }
        }
        inline bool initialized() const {
            return m_initialized;
        }
        bool initialize() {
            if(!m_initialized) {

                static const uint8_t generic_ssd1351[] PROGMEM =  {                
     0xFD,
    1, // Set command lock, 1 arg
    0x12,
    0xFD,
    1, // Set command lock, 1 arg
    0xB1,
    0xAE,
    0, // Display off, no args
    0xB3,
    1,
    0xF1, // 7:4 = Oscillator Freq, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    0xCA,
    1,
    127,
    0xA2,
    1,
    0x0,
    0xB5,
    1,
    0x00,
    0xAB,
    1,
    0x01, // internal (diode drop)
    0xB1,
    1,
    0x32,
    0xBE,
    1,
    0x05,
    0xA6,
    0,
    0xA0,
    1,
    0b01110100,
    0xA1,
    1,
    128,
    0xC1,
    3,
    0xC8,
    0x80,
    0xC8,
    0xC7,
    1,
    0x0F,
    0xB4,
    3,
    0xA0,
    0xB5,
    0x55,
    0xB6,
    1,
    0x01,
    0xAF,
    0,  // Main screen turn on
    0}; // END OF COMMAND LIST                         //    10 ms delay

                if(pin_rst>=0) {
                    pinMode(pin_rst,OUTPUT);
                }
                if(pin_dc>=0) {
                    pinMode(pin_dc,OUTPUT);
                }
                if(!driver::initialize()) {
                    return false;
                }
                reset();
                bus::begin_initialization();
                bus::begin_write();
                bus::begin_transaction();
                const uint8_t *addr = generic_ssd1351;
                uint8_t cmd, x, numArgs;
                while ((cmd = pgm_read_byte(addr++)) > 0) { // '0' command ends list
                    x = pgm_read_byte(addr++);
                    numArgs = x & 0x7F;
                    if (cmd != 0xFF) { // '255' is ignored
                        driver::send_command(cmd);
                        driver::send_data_pgm(addr, numArgs);
                    }
                    addr += numArgs;
                }
                delay(20);
                bus::end_transaction();
                bus::end_write();
                bus::end_initialization();
                bus::begin_write();
                bus::begin_transaction();
                apply_rotation();
                bus::end_transaction();
                bus::end_write();
                m_initialized = true;
                
            }
            return true;
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
        inline gfx::size16 dimensions() const {
            return {128,128};
        }
        inline gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
        
        inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
            return fill({location.x,location.y,location.x,location.y},color);
        }
        inline gfx::gfx_result point_async(gfx::point16 location, pixel_type color) {
            return point(location,color);
        }
        gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
            if(out_color==nullptr) return gfx::gfx_result::invalid_argument;
            if(!m_initialized || m_in_batch) return gfx::gfx_result::invalid_state;
            if(!bounds().intersects(location)) {
                *out_color = pixel_type();
                return gfx::gfx_result::success;
            }
            bus::dma_wait();
            bus::cs_low();
            set_window({location.x,location.y,location.x,location.y},true);
            bus::direction(INPUT);
            bus::read_raw8(); // throw away
            out_color->native_value = ((bus::read_raw8() & 0xF8) << 8) | ((bus::read_raw8() & 0xFC) << 3) | (bus::read_raw8() >> 3);
            bus::cs_high();
            bus::direction(OUTPUT);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
            if(!initialize()) return gfx::gfx_result::device_error;
            else bus::dma_wait();
            gfx::gfx_result rr = commit_batch();

            if(rr!=gfx::gfx_result::success) {
                return rr;
            }
            if(!bounds.intersects(this->bounds())) return gfx::gfx_result::success;
            const gfx::rect16 r = bounds.normalize().crop(this->bounds());
            bus::begin_write();
            bus::begin_transaction();
            set_window(r);
            bus::write_raw16_repeat(color.native_value,(r.x2-r.x1+1)*(r.y2-r.y1+1));
            bus::end_transaction();
            bus::end_write();
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result fill_async(const gfx::rect16& bounds, pixel_type color) {
            return fill(bounds,color);
        }
        inline gfx::gfx_result clear(const gfx::rect16& bounds) {
            return fill(bounds,pixel_type());
        }
        inline gfx::gfx_result clear_async(const gfx::rect16& bounds) {
            return clear(bounds);
        }
        template<typename Source>
        inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            return copy_from_impl(src_rect,src,location,false);
        }
        template<typename Source>
        inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            if(!m_dma_initialized) {
                if(!bus::initialize_dma()) return gfx::gfx_result::device_error;
                m_dma_initialized = true;
            }
            return copy_from_impl(src_rect,src,location,true);
        }
        gfx::gfx_result commit_batch() {
            if(m_in_batch) {
                bus::end_transaction();
                bus::end_write();
                m_in_batch = false;
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result commit_batch_async() {
            return commit_batch();
        }
        gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr!=gfx::gfx_result::success) {
                return rr;
            }
            const gfx::rect16 r = bounds.normalize();
            bus::begin_write();
            bus::begin_transaction();
            set_window(r);
            m_in_batch = true;
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
            return begin_batch(bounds);
        }
        gfx::gfx_result write_batch(pixel_type color) {
            bus::write_raw16(color.native_value);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result write_batch_async(pixel_type color) {
            return write_batch(color);
        }
        inline gfx::gfx_result wait_all_async() {
            bus::dma_wait();
            return gfx::gfx_result::success;
        }
        
        
    };
}