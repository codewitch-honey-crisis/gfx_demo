#pragma once
#include "bits.hpp"
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"
#include "common/spi_master.hpp"
namespace espidf {
    template<uint8_t WidthSegments,uint8_t HeightSegments, spi_host_device_t HostId,gpio_num_t PinCS>
    struct max7219 final {
        enum struct result {
            success = 0,
            invalid_argument = 1,
            io_error=2,
            out_of_memory=3
        };
        struct rect {
            uint16_t x1;
            uint16_t y1;
            uint16_t x2;
            uint16_t y2;
        };     
        static_assert(WidthSegments>0 && HeightSegments>0,"Not enough segments");
        static_assert(WidthSegments*HeightSegments<=256,"Too many segments");
        
        constexpr static const uint16_t width_segments = WidthSegments;
        constexpr static const uint16_t height_segments = HeightSegments;
        constexpr static const uint8_t segments = WidthSegments*HeightSegments;
        constexpr static const uint16_t width = WidthSegments*8;
        constexpr static const uint16_t height = HeightSegments*8;
        constexpr static const spi_host_device_t host_id = HostId;
        constexpr static const gpio_num_t pin_cs = PinCS;
    private:
        bool m_initialized;
        spi_device m_spi;
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
        static bool normalize_values(rect& bounds,bool check_bounds=true) {
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
        void line_rect(uint8_t line,rect* out_rect) {
            out_rect->x1 = line % (width/8);
            out_rect->x2 = out_rect->x1 + 7;
            out_rect->y1=out_rect->y2= line / (width/8);
        }
        result display_update(const rect& bounds) {
            result r;
            int line = 0;
            for(int x = 0;x<width;x+=8) {        
                for(int y=0;y<height;++y) {
                    if(x<=bounds.x2&&x+7>=bounds.x1&&y<=bounds.y2&&y>=bounds.y1) {
                        const uint8_t* p = m_frame_buffer+(y*width+x)/8;
                        r=set_line(line,*p);
                        if(result::success!=r) {
                            return r;
                        }
                    }
                    ++line;
                }
            }
            return result::success;
        }
        void buffer_fill(const rect& bounds,bool color) {
            rect b = bounds;
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
        result disable_decode_mode()
        {
            result r = send( 0xFF, (9 << 8) );
            if(result::success!=r) {
                return r;
            }
            return clear();
        }

        result set_brightness(uint8_t value)
        {
            if(value > 15) {
                return result::invalid_argument;
            }

            return send( 0xFF, (10 << 8) | value);

        }

        result set_enabled(bool enabled)
        {
            return send(0xFF, (12 << 8) | enabled);
        }


        result clear()
        {
            for (uint8_t i = 0; i < 8; i++) {
                result r = send( 0xFF, ((1 << 8) + ((uint16_t)i << 8)));
                if(result::success!=r)
                    return r;
            }
            return result::success;
        }

        result send(uint8_t seg, uint16_t value)
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
            if(spi_result::success!=m_spi.write((uint8_t*)buf,segments*2)) {
                return result::io_error;
            }
            return result::success;
            
        }
        
        inline static spi_device_interface_config_t get_spi_config() {
            spi_device_interface_config_t spi_cfg;
            memset(&spi_cfg, 0, sizeof(spi_cfg));
            spi_cfg.spics_io_num = pin_cs;
            spi_cfg.clock_speed_hz = 10*1000*1000;
            spi_cfg.mode = 0;
            spi_cfg.queue_size = 1;
            spi_cfg.flags = SPI_DEVICE_NO_DUMMY;    
            return spi_cfg;        
        }
        max7219(const max7219& rhs)=delete;
        max7219& operator=(const max7219& rhs)=delete;
    public:
        max7219() : 
            m_initialized(false),
            m_spi(host_id,get_spi_config()),
            m_suspend_first(0),
            m_suspend_count(0) {
        }
        inline bool initialized() const {
            return m_initialized;
        }
        result initialize() {
            if(!m_initialized) {
                result r;
                r=set_enabled(false);
                if(result::success!=r) {
                    return r;
                }
                r=send(0xFF, (15 << 8));
                if(result::success!=r) {
                    return r;
                }
                r=send(0xFF, (11 << 8) | 7);
                if(result::success!=r) {
                    return r;
                }
                r=disable_decode_mode();
                if(result::success!=r) {
                    return r;
                }
                r=set_brightness(0);
                if(result::success!=r) {
                    return r;
                }
                r=set_enabled(true);
                if(result::success!=r) {
                    return r;
                }
                m_initialized=true;
            }
            return result::success;
        }
        
        ~max7219() {}
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
        result set_line(uint8_t line, uint8_t val)
        {
            if (line >= segments*8)
            {
                return result::invalid_argument;
            }

            uint8_t c = line / 8;
            uint8_t d = line % 8;

            return send(c, ((1 << 8) + ((uint16_t)d << 8)) | val);
        }
        result pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return result::invalid_argument;
            if(x>=width || y>=height) {
                *out_color = false;
                return result::success;
            }
            const uint8_t* p = m_frame_buffer+(y*width/8)+x;
            *out_color = 0!=(*p & (1<<(7-(x&7))));
            return result::success;
        }
        result frame_fill(const rect& bounds,bool color) {
            result r = initialize();
            if(result::success!=r)
                return r;
            buffer_fill(bounds,color);
            return display_update(bounds);
        }
        result frame_suspend() {
            m_suspend_first=(m_suspend_count==0);
            ++m_suspend_count;
            return result::success;
        }
        result frame_resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    return display_update({m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2});
                }
                
            } 
            return result::success;
        }
        // GFX Bindings
        using type = max7219<WidthSegments,HeightSegments,HostId,PinCS>;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps< false,false,false,false,true,true,false>;
    private:
        static gfx::gfx_result xlt_err(result r) {
            switch(r) {
                case result::io_error:
                    return gfx::gfx_result::device_error;
                case result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case result::success:
                    return gfx::gfx_result::success;
                case result::invalid_argument:
                    return gfx::gfx_result::invalid_argument;
                default:
                    return gfx::gfx_result::unknown_error;
            }
        }
 public:
        constexpr inline gfx::size16 dimensions() const {return gfx::size16(width,height);}
        constexpr inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        // gets a point 
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            bool col=false;
            result r = pixel_read(location.x,location.y,&col);
            if(result::success!=r)
                return xlt_err(r);
            pixel_type p(!!col);
            *out_color=p;
            return gfx::gfx_result::success;
       }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            result r = frame_fill({location.x,location.y,location.x,location.y},color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            
            result r = frame_fill({rect.x1,rect.y1,rect.x2,rect.y2},color.native_value!=0);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            result r =frame_suspend();
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result resume(bool force=false) {
            result r =frame_resume(force);
            if(result::success!=r)
                return xlt_err(r);
            return gfx::gfx_result::success;
        }
    };   
}