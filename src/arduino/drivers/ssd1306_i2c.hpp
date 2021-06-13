#include <Arduino.h>
#include <Wire.h>
#include "gfx_pixel.hpp"
#include "gfx_positioning.hpp"
#define SSD1306_MEMORYMODE 0x20          ///< See datasheet
#define SSD1306_COLUMNADDR 0x21          ///< See datasheet
#define SSD1306_PAGEADDR 0x22            ///< See datasheet
#define SSD1306_SETCONTRAST 0x81         ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D          ///< See datasheet
#define SSD1306_SEGREMAP 0xA0            ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5        ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6       ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7       ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8        ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE          ///< See datasheet
#define SSD1306_DISPLAYON 0xAF           ///< See datasheet
#define SSD1306_COMSCANINC 0xC0          ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8          ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9        ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA          ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB       ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00  ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40  ///< See datasheet

#define SSD1306_EXTERNALVCC 0x01  ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26              ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27               ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E                    ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F                      ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3             ///< Set scroll range
#if defined(I2C_BUFFER_LENGTH)
#define SSD1306_WIRE_MAX min(256, I2C_BUFFER_LENGTH) ///< Particle or similar Wire lib
#elif defined(BUFFER_LENGTH)
#define SSD1306_WIRE_MAX min(256, BUFFER_LENGTH) ///< AVR or similar Wire lib
#elif defined(SERIAL_BUFFER_SIZE)
#define SSD1306_WIRE_MAX                                                               \
  min(255, SERIAL_BUFFER_SIZE - 1) ///< Newer Wire uses RingBuffer
#else
#define SSD1306_WIRE_MAX 32 ///< Use common Arduino core default
#endif

namespace arduino {
    template<uint16_t Width,
            uint16_t Height,
            uint8_t Address=0x3C,
            bool Vdc3_3=true,
            int8_t PinRst=-1,
            bool ResetBeforeInit=false>
    struct ssd1306_i2c final {
        enum struct result {
            success = 0,
            invalid_argument = 1,
            io_error = 2,
            out_of_memory = 3,
            timeout = 4,
            not_supported=5,
            i2c_not_initalized=6,
            invalid_state=7
        };
        constexpr static const uint16_t width=Width;
        constexpr static const uint16_t height=Height;
        constexpr static const uint8_t address = Address;
        constexpr static const bool vdc_3_3 = Vdc3_3;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const bool reset_before_init = ResetBeforeInit;
private:
        unsigned int m_initialized;
        TwoWire& m_i2c;
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
        void start_trans(uint8_t payload) {
            m_i2c.beginTransmission(address);
            m_i2c.write(payload);
        }
        void write_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(0==size) return;
            const uint8_t trans_code = 0x40*is_data;
            start_trans(trans_code);
            int buf_count = 1;
            while (size--) {
                if (buf_count >= SSD1306_WIRE_MAX) {
                    m_i2c.endTransmission(true);
                    start_trans(trans_code);
                    buf_count = 1;
                }

                m_i2c.write(*data);
                ++data;
                ++buf_count;
            }
            m_i2c.endTransmission(true);
        }
        void write_pgm_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(0==size) return;
            const uint8_t trans_code = 0x40*is_data;
            
            start_trans(trans_code);
            int buf_count = 1;
            while (size--) {
                if (buf_count >= SSD1306_WIRE_MAX) {
                    m_i2c.endTransmission(true);
                    start_trans(trans_code);
                    buf_count = 1;
                }

                m_i2c.write(pgm_read_byte(data));
                ++data;
                ++buf_count;
            }
            m_i2c.endTransmission(true);
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
                    //++p;
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
                    SSD1306_PAGEADDR,
                    uint8_t(b.y1/8),                   // Page start address
                    uint8_t(0xFF),                   // Page end (not really, but works here)
                    SSD1306_COLUMNADDR, uint8_t(b.x1)};// Column start address
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
public:
        ssd1306_i2c(TwoWire& i2c) : 
                    m_initialized(false),
                    m_i2c(i2c),
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
        void initialize() {
            if(!m_initialized) {
                if(reset_before_init) {
                    reset();
                }
                uint8_t cmd;
                // Init sequence
                static const uint8_t init1[] PROGMEM = {SSD1306_DISPLAYOFF,         // 0xAE
                                                        SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
                                                        0x80, // the suggested ratio 0x80
                                                        SSD1306_SETMULTIPLEX}; // 0xA8
                write_pgm_bytes(init1, sizeof(init1),false);

                cmd=height-1;
                write_bytes(&cmd,1,false);
                
                static const uint8_t init2[] PROGMEM = {SSD1306_SETDISPLAYOFFSET, // 0xD3
                                                        0x0,                      // no offset
                                                        SSD1306_SETSTARTLINE | 0x0, // line #0
                                                        SSD1306_CHARGEPUMP};        // 0x8D
                write_pgm_bytes(init2, sizeof(init2),false);
                
                cmd=!vdc_3_3 ? 0x10 : 0x14;
                write_bytes(&cmd,1,false);

                static const uint8_t init3[] PROGMEM = {SSD1306_MEMORYMODE, // 0x20
                                                        0x00, // 0x0 act like ks0108, but we want mode 1?
                                                        SSD1306_SEGREMAP | 0x1,
                                                        SSD1306_COMSCANDEC};
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
                    return;
                cmd=SSD1306_SETCOMPINS;
                write_bytes(&cmd,1,false);
                write_bytes(&com_pins,1,false);
                cmd=SSD1306_SETCONTRAST;
                write_bytes(&cmd,1,false);
                write_bytes(&m_contrast,1,false);
                cmd=SSD1306_SETPRECHARGE; // 0xd9
                write_bytes(&cmd,1,false);
                cmd=!vdc_3_3 ? 0x22:0xF1;
                write_bytes(&cmd,1,false);
                static const uint8_t init5[] PROGMEM = {
                    SSD1306_SETVCOMDETECT, // 0xDB
                    0x40,
                    SSD1306_DISPLAYALLON_RESUME, // 0xA4
                    SSD1306_NORMALDISPLAY,       // 0xA6
                    SSD1306_DEACTIVATE_SCROLL,
                    SSD1306_DISPLAYON}; // Main screen turn on
                write_pgm_bytes(init5, sizeof(init5),false);
                m_initialized = true;
            }
        }
        const uint8_t* frame_buffer() const {
            return m_frame_buffer;
        }
        
        result pixel_read(uint16_t x,uint16_t y,bool* out_color) const {
            if(nullptr==out_color)
                return result::invalid_argument;
            if(x>=width || y>=height) {
                *out_color = false;
                return result::success;
            }
            const uint8_t* p = m_frame_buffer+(y/8*width)+x;
            *out_color = 0!=(*p & (1<<(y&7)));
            return result::success;
        }
        result frame_fill(const rect& bounds,bool color) {
            initialize();
            buffer_fill(bounds,color);
            display_update(bounds);
            return result::success;
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
                    display_update({m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2});
                }
                
            } 
            return result::success;
        }
        // GFX Bindings
        using type = ssd1306_i2c;
        using pixel_type = gfx::gsc_pixel<1>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
    private:
        static gfx::gfx_result xlt_err(result r) {
            switch(r) {
                case result::io_error:
                case result::i2c_not_initalized:
                    return gfx::gfx_result::device_error;
                case result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case result::success:
                    return gfx::gfx_result::success;
                case result::not_supported:
                    return gfx::gfx_result::not_supported;
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