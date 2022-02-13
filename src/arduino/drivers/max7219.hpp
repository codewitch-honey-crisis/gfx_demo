#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "gfx_core.hpp"
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"
#include "common/tft_driver.hpp"
namespace arduino {
    template<uint8_t WidthSegments,uint8_t HeightSegments, int8_t PinCS,typename Bus>
    struct max7219 final {
        
        static_assert(WidthSegments>0 && HeightSegments>0,"Not enough segments");
        static_assert(WidthSegments*HeightSegments<=256,"Too many segments");
        
        constexpr static const uint16_t width_segments = WidthSegments;
        constexpr static const uint16_t height_segments = HeightSegments;
        constexpr static const uint8_t segments = WidthSegments*HeightSegments;
        constexpr static const uint16_t width = WidthSegments*8;
        constexpr static const uint16_t height = HeightSegments*8;
        constexpr static const int8_t pin_cs = PinCS;
        using bus = Bus;
        using driver = tft_driver<-1,-1,-1,bus,-1>;
    private:
        bool m_initialized;
        uint8_t m_frame_buffer[segments*8];
        uint16_t m_suspend_x1;
        uint16_t m_suspend_y1;
        uint16_t m_suspend_x2;
        uint16_t m_suspend_y2;
        int m_suspend_first;
        int m_suspend_count;
        static inline uint16_t shuffle(uint16_t val)
        {
            return (val >> 8) | (val << 8);
        }
        void spi_write(const uint8_t* data,size_t size) {
            driver::send_data(data,size);
        }
        void spi_start() {
            bus::begin_write();
            bus::start_transaction();
        }

        void spi_end() {
            bus::end_transaction();
            bus::end_write();
        }
        inline void spi_write(uint8_t data) {
            driver::send_data8(data);
        }
        inline void spi_write(uint16_t data) {
            driver::send_data16(data);
        }
        static bool normalize_values(gfx::rect16& bounds,bool check_bounds=true) {
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
        void line_rect(uint8_t line,gfx::rect16* out_rect) {
            out_rect->x1 = line % (width/8);
            out_rect->x2 = out_rect->x1 + 7;
            out_rect->y1=out_rect->y2= line / (width/8);
        }
        bool display_update(const gfx::rect16& bounds) {
            int line = 0;
            for(int x = 0;x<width;x+=8) {        
                for(int y=0;y<height;++y) {
                    if(x<=bounds.x2&&x+7>=bounds.x1&&y<=bounds.y2&&y>=bounds.y1) {
                        const uint8_t* p = m_frame_buffer+(y*width+x)/8;
                        if(!set_line(line,*p)) {
                            return false;
                        }
                    }
                    ++line;
                }
            }
            return true;
        }
        void buffer_fill(const gfx::rect16& bounds,bool color) {
            gfx::rect16 b = bounds;
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
        bool disable_decode_mode()
        {
            if(!send( 0xFF, (9 << 8) )) {
                return false;
            }
            return clear();
        }

        bool set_brightness(uint8_t value)
        {
            if(value > 15) {
                return false;
            }

            return send( 0xFF, (10 << 8) | value);

        }

        inline bool set_enabled(bool enabled)
        {
            return send(0xFF, (12 << 8) | enabled);
        }


        bool clear()
        {
            for (uint8_t i = 0; i < 8; i++) {
                if(!send( 0xFF, ((1 << 8) + ((uint16_t)i << 8)))) {
                    return false;
                }
            }
            return true;
        }

        bool send(uint8_t seg, uint16_t value)
        {
      
            uint16_t buf[8] = { 0 };
            value = shuffle(value);
            if (seg == 0xFF)
            {
                for (uint8_t i = 0; i < segments; i++)
                    buf[i] = value;
            }
            else {
                buf[seg] = value;
            }
            spi_start();
            spi_write((uint8_t*)buf,segments*2);
            spi_end();
            return true;
            
        }
        bool set_line(uint8_t line, uint8_t val)
        {
            if(!initialize()) {
                return false;
            }
            if (line >= segments*8)
            {
                return false;
            }

            uint8_t c = line / 8;
            uint8_t d = line % 8;

            return send(c, ((1 << 8) + ((uint16_t)d << 8)) | val);
        }
        bool pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return false;
            if(x>=width || y>=height) {
                *out_color = false;
                return true;
            }
            const uint8_t* p = m_frame_buffer+(y*width/8)+x;
            *out_color = 0!=(*p & (1<<(7-(x&7))));
            return true;
        }
        bool frame_fill(const gfx::rect16& bounds,bool color) {
            if(!initialize()) {
                return false;
            }
            buffer_fill(bounds,color);
            return display_update(bounds);
        }
        inline bool frame_suspend() {
            m_suspend_first=(m_suspend_count==0);
            ++m_suspend_count;
            return true;
        }
        bool frame_resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    return display_update({m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2});
                }
                
            } 
            return true;
        }
        max7219(const max7219& rhs)=delete;
        max7219& operator=(const max7219& rhs)=delete;
    public:
        max7219() : 
            m_initialized(false),
            m_suspend_first(0),
            m_suspend_count(0) {
        }
        inline bool initialized() const {
            return m_initialized;
        }
        bool initialize() {
            if(!m_initialized) {
                if(!driver::initialize()) {
                    return false;
                }
                if(!set_enabled(false)) {
                    return false;
                }
                if(!send(0xFF, (15 << 8))) {
                    return false;
                }
                if(!send(0xFF, (11 << 8) | 7)) {
                    return false;
                }
                if(!disable_decode_mode()) {
                    return false;
                }
                if(!set_brightness(0)) {
                    return false;
                }
                if(!set_enabled(true)) {
                    return false;
                }
                
                m_initialized=true;
            }
            return true;
        }
        
        ~max7219() {
            if(m_initialized) {
                driver::deinitialize();
            }
        }
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
    public:
        // GFX Bindings
        using type = max7219;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps< false,false,false,false,true,true,false>;
    
        constexpr inline gfx::size16 dimensions() const {return gfx::size16(width,height);}
        constexpr inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        // gets a point 
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            bool col=false;
            if(!pixel_read(location.x,location.y,&col)) {
                return gfx::gfx_result::io_error;
            }
            pixel_type p(!!col);
            *out_color=p;
            return gfx::gfx_result::success;
       }
        // sets a point to the specified pixel
        inline gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            if(!frame_fill({location.x,location.y,location.x,location.y},color.native_value!=0)) {
                return gfx::gfx_result::io_error;
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            if(!frame_fill(rect,color.native_value!=0)) {
                return gfx::gfx_result::io_error;
            }
            return gfx::gfx_result::success;
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            if(!frame_suspend()) {
                return gfx::gfx_result::io_error;
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result resume(bool force=false) {
            if(!frame_resume(force)) {
                return gfx::gfx_result::io_error;
            }
            return gfx::gfx_result::success;
        }
    };   
}