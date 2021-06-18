#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "common/epaper_spi_driver.hpp"
// unlike most of the other drivers, which simply expose 
// GFX binding which may be removed, this driver 
// requires GFX in order to function due to the 
// complexity of indexed pixel translation involved
// there was no sense in duplicating the code for
// bit mangling.
namespace arduino {
    
    struct gdeh0154z90_palette {
        
    private:
        constexpr static gfx::gfx_result index_to_mapped(int idx,gfx::rgb_pixel<16>* result) {
            uint8_t red = 31*(idx>0), green=63*(idx==2), blue=green>>1;
            result->native_value = gfx::rgb_pixel<16>(red,green,blue).native_value;
            return gfx::gfx_result::success;
        }
    public:
        using type = gdeh0154z90_palette;
        using pixel_type = gfx::pixel<gfx::channel_traits<gfx::channel_name::index,2,0,2>>;
        using mapped_pixel_type = gfx::rgb_pixel<16>;
        constexpr static const bool writable = false;
        constexpr static const size_t size = 3;
        gfx::gfx_result map(pixel_type pixel,mapped_pixel_type* mapped_pixel) const {
            return index_to_mapped(pixel.channel<gfx::channel_name::index>(),mapped_pixel);
        }
        gfx::gfx_result nearest(mapped_pixel_type mapped_pixel,pixel_type* pixel) const {
            
            if(nullptr==pixel) {
                return gfx::gfx_result::invalid_argument;
            }
            mapped_pixel_type mpx;
            gfx::gfx_result r = index_to_mapped(0,&mpx);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            double least = mpx.difference(mapped_pixel);
            if(0.0==least) {
                pixel->native_value = 0;
                return gfx::gfx_result::success;
            }
            int ii=0;
            for(int i = 1;i<size;++i) {
                r=index_to_mapped(i,&mpx);
                if(gfx::gfx_result::success!=r) {
                    return r;
                }
                double cmp = mpx.difference(mapped_pixel);
                if(0.0==cmp) {
                    ii=i;
                    least = 0.0;
                    break;
                }
                if(cmp<least) {
                    least = cmp;
                    ii=i;
                }
            }
            pixel->channel<gfx::channel_name::index>(ii);
            return gfx::gfx_result::success;
        }
    };
    
    template<int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy>
    struct gdeh0154z90 final {
        enum struct result {
            success = 0,
            invalid_argument,
            io_error,
            io_busy,
            out_of_memory,
            timeout,
            not_supported
        };
        constexpr static const uint16_t width = 200;
        constexpr static const uint16_t height = 200;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        // the Busy pin
        constexpr static const int8_t pin_busy = PinBusy;

        constexpr static const uint32_t clock_speed = 4*1000*1000;
        struct rect {
            uint16_t x1;
            uint16_t y1;
            uint16_t x2;
            uint16_t y2;
        };
        using palette_type = gdeh0154z90_palette;
        using pixel_type = typename palette_type::pixel_type;
        using frame_buffer_type = gfx::bitmap<pixel_type,palette_type>;
    private: 
        
        bool m_initialized;
        SPIClass& m_spi;
        int m_suspend_count;
        const gdeh0154z90_palette m_palette;
        uint8_t m_frame_buffer[frame_buffer_type::sizeof_buffer(width,height)];
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
        
        void send_command(uint8_t command)
        {
            m_spi.beginTransaction(SPISettings(clock_speed, MSBFIRST, SPI_MODE0));
            if (pin_dc >= 0) {
                digitalWrite(pin_dc, LOW);
            }
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, LOW);
            }
            m_spi.transfer(command);
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, HIGH);
            }
            if (pin_dc >= 0) {
                digitalWrite(pin_dc, HIGH);
            }
            m_spi.endTransaction();
        }

        void send_data(uint8_t data)
        {
            m_spi.beginTransaction(SPISettings(clock_speed, MSBFIRST, SPI_MODE0));
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, LOW);
            }
            m_spi.transfer(data);
            if (pin_cs >= 0) {
                digitalWrite(pin_cs, HIGH);
            }
            m_spi.endTransaction();
        }
        void wait_busy(uint16_t ms) {
            if (pin_busy >= 0) {
                uint32_t ts = millis();
                while (true) {
                    if (!digitalRead(pin_busy)) {
                        return;
                    }
                    delay(1);
                    if (millis() - ts > 5000) {
                        return;
                    }
                }
            } 
            delay(ms);
        }
        gfx::gfx_result display_update() {
            initialize();
            if(0==m_suspend_count) {
                send_command(0x24);   //write RAM for black(0)/white (1)
                frame_buffer_type fb(dimensions(),m_frame_buffer,&m_palette);
                for(int y=0;y<height;++y) {
                    for(int x=0;x<width;x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            pixel_type px;
                            gfx::point16 pt(x+i,y);
                            gfx::gfx_result r = fb.point(pt,&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            bool is_white = px.channel<0>()==2;
                            data |= (1<<(7-i))*is_white;
                        }
                        send_data(data);
                    }
                }
                send_command(0x26);   //write RAM for black(0)/red (1)
                for(int y=0;y<height;++y) {
                    for(int x=0;x<width;x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            pixel_type px;
                            gfx::point16 pt(x+i,y);
                            gfx::gfx_result r = fb.point(pt,&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            bool is_red = px.channel<0>()==1;
                            data |= (1<<(7-i))*is_red;
                        }
                        send_data(data);
                    }
                }
                send_command(0x22); //Display Update Control
                send_data(0xF7);   
                send_command(0x20);  //Activate Display Update Sequence
                wait_busy(1200);
            }
            return gfx::gfx_result::success;
        }
    public:
        gdeh0154z90(SPIClass& spi) : m_initialized(false),m_spi(spi),m_palette() {

        }
        void reset() {
            if(pin_rst>=0) {
                digitalWrite(pin_rst,LOW);
                delay(10);
                digitalWrite(pin_rst,HIGH);
                delay(10);
            }
        }
        void initialize() {
            if(!m_initialized) {
                if (pin_cs >= 0) {
                    pinMode(pin_cs, OUTPUT);
                }
                if (pin_dc >= 0) {
                    pinMode(pin_dc, OUTPUT);
                }
                if (pin_rst >= 0) {
                    pinMode(pin_rst, OUTPUT);
                }
                reset();  
                wait_busy(300);
                m_spi.begin();
                send_command(0x12);  //SWRESET
                wait_busy(100) ;
                    
                send_command(0x01); //Driver output control      
                send_data(0xC7);
                send_data(0x00);
                send_data(0x00);

                send_command(0x11); //data entry mode       
                send_data(0x00);

                send_command(0x44); //set Ram-X address start/end position   
                send_data(0x18);    //0x18-->(24+1)*8=200
                send_data(0x00);
                
                send_command(0x45); //set Ram-Y address start/end position          
                send_data(0xC7);    //0xC7-->(199+1)=200
                send_data(0x00);
                send_data(0x00);
                send_data(0x00); 
                
                send_command(0x4E); // set Ram pointer
                send_data(0x18); // X

                send_data(0x4F);
                send_data(0xC7); // Y1
                send_data(0x00); // Y2

                send_command(0x3C); //BorderWavefrom
                send_data(0x05);  
                    
                send_command(0x18); //Read built-in temperature sensor
                send_data(0x80);  
                
                send_command(0x4E);   // set RAM x address count to 0x199;
                send_data(0x18);
                send_command(0x4F);   // set RAM y address count to 0X199;    
                send_data(0xC7);
                send_data(0x00);
                
                wait_busy(300);
                m_initialized = true;
                
            }
        }
    // GFX Bindings
    public:
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
    private:
        static gfx::gfx_result xlt_err(result r) {
            switch(r) {
                case result::io_error:
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
            return frame_buffer_type::point(dimensions(),m_frame_buffer,location,out_color);    
        }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            frame_buffer_type fb(dimensions(),m_frame_buffer,&m_palette);
            gfx::gfx_result r=fb.point(location,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            return display_update();
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            frame_buffer_type fb(dimensions(),m_frame_buffer,&m_palette);
            gfx::gfx_result r=fb.fill(rect,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            return display_update();
        }
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result resume(bool force=false) {
            if(force) {
                m_suspend_count=0;
            }
            if(0<m_suspend_count)
                --m_suspend_count;
            return display_update();
            
            return gfx::gfx_result::success;
        }
        inline const palette_type* palette() const {
            return &m_palette;
        }
        
    };
    
    
    
}