#include "common/tft_driver.hpp"
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"

namespace arduino {
    template<uint16_t Width,
            uint16_t Height,
            typename Bus,
            bool Vdc3_3=true,
            int8_t PinDC=-1,
            int8_t PinRst=-1,
            bool ResetBeforeInit=false>
    struct ssd1306 final {
        
        constexpr static const uint16_t width=Width;
        constexpr static const uint16_t height=Height;
        constexpr static const bool vdc_3_3 = Vdc3_3;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const bool reset_before_init = ResetBeforeInit;
private:
        using bus = Bus;
        using driver = tft_driver<PinDC,PinRst,-1,Bus,-1>;
        unsigned int m_initialized;
        unsigned int m_suspend_x1;
        unsigned int m_suspend_y1;
        unsigned int m_suspend_x2;
        unsigned int m_suspend_y2;
        unsigned int m_suspend_count;
        unsigned int m_suspend_first;
    
        struct rect {
            unsigned int x1;
            unsigned int y1;
            unsigned int x2;
            unsigned int y2;
        };
        uint8_t m_contrast;
        uint8_t m_frame_buffer[width*height/8];
        
        inline void write_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(is_data) {
                driver::send_data(data,size);
            } else {
                driver::send_command(data,size);
            }
        }
        inline void write_pgm_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(is_data) {
                driver::send_data_pgm(data,size);
            } else {
                driver::send_command_pgm(data,size);
            }
        }
        static bool normalize_values(rect& r,bool check_bounds=true) {
            // normalize values
            uint16_t tmp;
            if(r.x1>r.x2) {
                tmp=r.x1;
                r.x1=r.x2;
                r.x2=tmp;
            }
            if(r.y1>r.y2) {
                tmp=r.y1;
                r.y1=r.y2;
                r.y2=tmp;
            }
            if(check_bounds) {
                if(r.x1>=width||r.y1>=height)
                    return false;
                if(r.x2>=width)
                    r.x2=width-1;
                if(r.y2>height)
                    r.y2=height-1;
            }
            return true;
        }
        void buffer_fill(const rect& bounds, bool color) {
            rect r = bounds;
            if(!normalize_values(r))
                return;
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = r.x1;
                    m_suspend_y1 = r.y1;
                    m_suspend_x2 = r.x2;
                    m_suspend_y2 = r.y2;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>r.x1)
                        m_suspend_x1=r.x1;
                    if(m_suspend_y1>r.y1)
                        m_suspend_y1=r.y1;
                    if(m_suspend_x2<r.x2)
                        m_suspend_x2=r.x2;
                    if(m_suspend_y2<r.y2)
                        m_suspend_y2=r.y2;
                }
            }
            int y=r.y1;
            int m=y%8;
            // special case when bottom and top are the same bank:
            if(r.y1/8==r.y2/8) {
                const uint8_t on_mask = uint8_t(uint8_t(0xFF<<((r.y1%8))) & uint8_t(0xFF>>(7-(r.y2%8))));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                for(int x=r.x1;x<=r.x2;++x) {
                    uint8_t* p = m_frame_buffer+(y/8*width)+x;
                    *p&=set_mask;
                    *p|=value_mask;
                }
                return;
            } 
           
            // first handle the top
            {
                const uint8_t on_mask = uint8_t(uint8_t(0xFF<<((r.y1%8))));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                for(int x=r.x1;x<=r.x2;++x) {
                    uint8_t* p = m_frame_buffer+(y/8*width)+x;
                    *p&=set_mask;
                    *p|=value_mask;
                }
                
                y=r.y1+(8-m);
            }
            
            int be = r.y2/8;
            int b = y/8;
            
            while(b<be) {
                // we can do a faster fill for this part
                const int w = r.x2-r.x1+1;    
                uint8_t* p = m_frame_buffer+(b*width)+r.x1;
                memset(p,0xFF*color,w);
                ++b;
            }
            // now do the trailing rows
            if(b*8<r.y2) {
                m=r.y2%8;
                y=r.y2;
                const uint8_t on_mask = uint8_t(0xFF>>(7-(r.y2%8)));
                const uint8_t set_mask = uint8_t(~on_mask);
                const uint8_t value_mask = uint8_t(on_mask*color);
                uint8_t* p = m_frame_buffer+(y/8*width)+r.x1;
                for(int x=r.x1;x<=r.x2;++x) {
                    *p&=set_mask;
                    *p|=value_mask;
                    ++p;
                }
            }
        }
        void display_update(const rect& bounds) {
            rect b = bounds;
            if(!normalize_values(b))
                return;
            initialize();
            // don't draw if we're suspended
            if(0==m_suspend_count) {    
                uint8_t dlist1[] = {
                    0x22,
                    uint8_t(b.y1/8),                   // Page start address
                    uint8_t(0xFF),                   // Page end (not really, but works here)
                    0x21, uint8_t(b.x1)};// Column start address
                write_bytes(dlist1, sizeof(dlist1),false);
                uint8_t cmd = b.x2;
                write_bytes(&cmd,1,false); // Column end address
                if(b.x1==0&&b.y1==0&&b.x2==width-1&&b.y2==height-1) {
                    // special case for whole screen
                    return write_bytes(m_frame_buffer,width*height/8,true);
                }
                const int be = (b.y2+8)/8;
                const int w = b.x2-b.x1+1;
                for(int bb=b.y1/8;bb<be;++bb) {
                    uint8_t* p = m_frame_buffer+(bb*width)+b.x1;
                    write_bytes(p,w,true);
                }
            }
        }
        bool pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return false;
            if(x>=width || y>=height) {
                *out_color = false;
                return true;
            }
            const uint8_t* p = m_frame_buffer+(y/8*width)+x;
            *out_color = 0!=(*p & (1<<(y&7)));
            return true;
        }
        bool frame_fill(const rect& bounds,bool color) {
            if(!initialize()) {
                return false;
            }
            buffer_fill(bounds,color);
            display_update(bounds);
            return true;
        }
        bool frame_suspend() {
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
                    display_update({m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2});
                }
                
            } 
            return true;
        }
public:
        ssd1306() : 
                    m_initialized(false),
                    m_suspend_count(0),
                    m_suspend_first(0) {
            
        }
        inline bool initialized() const {
            return m_initialized;
        }
        void reset() {
            if(pin_rst>=0) {
                digitalWrite(pin_rst,HIGH);
                delay(1);
                digitalWrite(pin_rst,LOW);
                delay(10);
                digitalWrite(pin_rst,HIGH);
            }
        }
        bool initialize() {
            if(!m_initialized) {
                if(!driver::initialize()) {
                    return false;
                }
                if(reset_before_init) {
                    reset();
                }
                bus::begin_initialization();
                bus::begin_write();
                uint8_t cmd;
                // Init sequence
                static const uint8_t init1[] PROGMEM = {0xAE,
                                                        0xD5,
                                                        0x80, // the suggested ratio 0x80
                                                        0xA8};
                write_pgm_bytes(init1, sizeof(init1),false);

                cmd=height-1;
                write_bytes(&cmd,1,false);
                
                static const uint8_t init2[] PROGMEM = {0xD3,
                                                        0x00,                      // no offset
                                                        0x40 | 0x00, // line #0
                                                        0x8D};
                write_pgm_bytes(init2, sizeof(init2),false);
                
                cmd=!vdc_3_3 ? 0x10 : 0x14;
                write_bytes(&cmd,1,false);

                static const uint8_t init3[] PROGMEM = { 0x20,
                                                        0x00, // 0x0 act like ks0108
                                                        0xA0 | 0x1,
                                                        0xC8};
                write_pgm_bytes(init3, sizeof(init3),false);
                uint8_t com_pins = 0x02;
                m_contrast = 0x8F;
                if ((width == 128) && (height == 32)) {
                    com_pins = 0x02;
                    m_contrast = 0x8F;
                } else if ((width == 128) && (height == 64)) {
                    com_pins = 0x12;
                    m_contrast = !vdc_3_3 ? 0x9F:0xCF;
                } else if ((width == 96) && (height == 16)) {
                    com_pins = 0x2; // ada x12
                    m_contrast = !vdc_3_3 ? 0x10:0xAF;
                } else
                    return false;
                cmd=0xDA;
                write_bytes(&cmd,1,false);
                write_bytes(&com_pins,1,false);
                cmd=0x81;
                write_bytes(&cmd,1,false);
                write_bytes(&m_contrast,1,false);
                cmd=0xD9;
                write_bytes(&cmd,1,false);
                cmd=!vdc_3_3 ? 0x22:0xF1;
                write_bytes(&cmd,1,false);
                static const uint8_t init5[] PROGMEM = {
                    0xDB,
                    0x40,
                    0xA4,
                    0xA6,
                    0x2E,
                    0xAF}; // Main screen turn on
                write_pgm_bytes(init5, sizeof(init5),false);
                bus::end_write();
                bus::end_initialization();
                m_initialized = true;
            }
            return true;
        }
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
        
        
        // GFX Bindings
        using type = ssd1306;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
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
            if(!frame_fill({rect.x1,rect.y1,rect.x2,rect.y2},color.native_value!=0)) {
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