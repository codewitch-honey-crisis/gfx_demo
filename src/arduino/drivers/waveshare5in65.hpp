#pragma once
#include <Arduino.h>
#include <gfx_bitmap.hpp>
#include "common/tft_driver.hpp"
namespace arduino {
    template<typename PixelType>
    struct waveshare5in65_palette {
    private:
        constexpr static gfx::gfx_result index_to_mapped(int idx,PixelType* result) {
            switch(idx) {
                case 0:
                    return gfx::convert(gfx::rgb_pixel<16>(0,0,0),result);
                case 1:
                    return gfx::convert(gfx::rgb_pixel<16>(31,63,31),result);
                case 2:
                    return gfx::convert(gfx::rgb_pixel<16>(0,63,0),result);
                case 3:
                    return gfx::convert(gfx::rgb_pixel<16>(0,0,31),result);
                case 4:
                    return gfx::convert(gfx::rgb_pixel<16>(31,0,0),result);
                case 5:
                    return gfx::convert(gfx::rgb_pixel<16>(31,63,0),result);
                default: //case 6:
                    return gfx::convert(gfx::rgb_pixel<16>(31,31,0),result);
            }
        }
    public:
        using type = waveshare5in65_palette;
        using pixel_type = gfx::pixel<gfx::channel_traits<gfx::channel_name::index,4,0,6>>;
        using mapped_pixel_type = PixelType;
        constexpr static const bool writable = false;
        constexpr static const size_t size = 7;
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
    
    template<int8_t PinDC, 
            int8_t PinRst, 
            int8_t PinWait, 
            typename Bus, 
            unsigned int WriteSpeedPercent = 100,
            unsigned int ReadSpeedPercent = WriteSpeedPercent>
    struct waveshare5in65 final {
        constexpr static const uint16_t width = 600;
        constexpr static const uint16_t height = 448;
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_wait = PinWait;
        constexpr static const float write_speed_multiplier = (WriteSpeedPercent/100.0);
        constexpr static const float read_speed_multiplier = (ReadSpeedPercent/100.0);
        using bus = Bus;
        using bus_driver = tft_driver<pin_dc,pin_rst,-1,bus>;
        using pixel_type = gfx::pixel<gfx::channel_traits<gfx::channel_name::index,4,0,6>>;
        using palette_type = waveshare5in65_palette<gfx::rgb_pixel<16>>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        using frame = gfx::bitmap<pixel_type,palette_type>;
        
    private:
        void*(*m_allocator)(size_t);
        void(*m_deallocator)(void*);
        uint8_t* m_frame_buffer;
        int m_suspend_count;
        palette_type m_palette;
        bool m_sleep;
        waveshare5in65(const waveshare5in65& rhs)=delete;
        waveshare5in65& operator=(const waveshare5in65& rhs)=delete;
        static void busy_high() { // If BUSYN=0 then waiting 
            while(!(digitalRead(pin_wait)));
        }
        static void busy_low() { // If BUSYN=1 then waiting 
            while(digitalRead(pin_wait));
        }
        gfx::gfx_result update_display() {
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            unsigned long i,j;
            bus_driver::send_command(0x61);//Set Resolution setting
            bus_driver::send_data8(0x02);
            bus_driver::send_data8(0x58);
            bus_driver::send_data8(0x01);
            bus_driver::send_data8(0xC0);
            bus_driver::send_command(0x10);
            for(i=0; i<height; i++) {
                for(j=0; j< width/2; j++) {
                    bus_driver::send_data8(m_frame_buffer[j + (width/2*i)]);
                }
            }
            bus_driver::send_command(0x04);
            busy_high();
            bus_driver::send_command(0x12);
            busy_high();
            bus_driver::send_command(0x02);
            busy_low();
            delay(200);
            
            return gfx::gfx_result::success;
        }
        gfx::gfx_result initialize_display() {
            reset();
            busy_high(); reset();
            busy_high();
            bus_driver::send_command(0x00);
            bus_driver::send_data8(0xEF);
            bus_driver::send_data8(0x08);
            bus_driver::send_command(0x01);
            bus_driver::send_data8(0x37);
            bus_driver::send_data8(0x00);
            bus_driver::send_data8(0x23);
            bus_driver::send_data8(0x23);
            bus_driver::send_command(0x03);
            bus_driver::send_data8(0x00);
            bus_driver::send_command(0x06);
            bus_driver::send_data8(0xC7);
            bus_driver::send_data8(0xC7);
            bus_driver::send_data8(0x1D);
            bus_driver::send_command(0x30);
            bus_driver::send_data8(0x3C);
            bus_driver::send_command(0x40);
            bus_driver::send_data8(0x00);
            bus_driver::send_command(0x50);
            bus_driver::send_data8(0x37);
            bus_driver::send_command(0x60);
            bus_driver::send_data8(0x22);
            bus_driver::send_command(0x61);
            bus_driver::send_data8(0x02);
            bus_driver::send_data8(0x58);
            bus_driver::send_data8(0x01);
            bus_driver::send_data8(0xC0);
            bus_driver::send_command(0xE3);
            bus_driver::send_data8(0xAA);

            delay(100);
            bus_driver::send_command(0x50);
            bus_driver::send_data8(0x37);
            return gfx::gfx_result::success;
        }
    public:
        waveshare5in65(waveshare5in65&& rhs) {
            m_allocator = rhs.m_allocator;
            m_deallocator = rhs.m_deallocator;
            m_frame_buffer = rhs.m_frame_buffer;
            m_suspend_count = rhs.m_suspend_count;
            rhs.m_deallocator = nullptr;
        }
        waveshare5in65& operator=(waveshare5in65&& rhs) {
            deinitialize();
            m_allocator = rhs.m_allocator;
            m_deallocator = rhs.m_deallocator;
            m_frame_buffer = rhs.m_frame_buffer;
            m_suspend_count = rhs.m_suspend_count;
            rhs.m_deallocator = nullptr;
            return *this;
        }
        inline gfx::size16 dimensions() const { return {width,height}; }
        inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        gfx::gfx_result initialize() {
            if(m_frame_buffer==nullptr) {
                const size_t sz = frame::sizeof_buffer(dimensions());
                m_frame_buffer = (uint8_t*)m_allocator(sz);
                if(m_frame_buffer==nullptr) {
                    return gfx::gfx_result::out_of_memory;
                }
                bus_driver::initialize();
                pinMode(pin_wait, INPUT);
                pinMode(pin_rst, OUTPUT);
                pinMode(pin_dc, OUTPUT);
                gfx::gfx_result r=initialize_display();
                if(r!=gfx::gfx_result::success) {
                    deinitialize();
                    return r;
                }
                m_sleep=false;
                return gfx::gfx_result::success;
            } else if(m_sleep) {
                gfx::gfx_result r=initialize_display();
                if(r!=gfx::gfx_result::success) {
                    deinitialize();
                    return r;
                }
                m_sleep=false;
            }
            return gfx::gfx_result::success;
        }
        void deinitialize() {
            m_sleep=false;
            if(m_deallocator!=nullptr) {
                m_deallocator = nullptr;
                if(m_frame_buffer!=nullptr) {
                    m_deallocator(m_frame_buffer);
                    m_frame_buffer = nullptr;
                    bus_driver::deinitialize();
                }
            }
        }
        inline bool initialized() const { return m_frame_buffer; }
        gfx::gfx_result sleep() {
            if(!m_frame_buffer) {
                return gfx::gfx_result::invalid_state;
            }
            if(!m_sleep) {
                delay(500);
                bus_driver::send_command(0x07); //DEEP_SLEEP
                bus_driver::send_data8(0xA5);
                digitalWrite(pin_rst, LOW);
                m_sleep=true;
            }
            return gfx::gfx_result::success;
        }
        void reset() {
            digitalWrite(pin_rst, HIGH);
            delay(600);
            digitalWrite(pin_rst, LOW);
            delay(2);
            digitalWrite(pin_rst, HIGH);
            delay(200);
        }
        waveshare5in65(void*(allocator)(size_t)=::malloc,void(deallocator)(void*)=::free) :
                m_allocator(allocator),
                m_deallocator(deallocator),
                m_frame_buffer(nullptr),
                m_suspend_count(0),
                m_sleep(false) {
        }
        ~waveshare5in65() {
            deinitialize();
        }
        inline const palette_type* palette() const {return &m_palette;}
        gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            int os = m_suspend_count;
            if(force || 0==--m_suspend_count) {
                m_suspend_count = 0;
                gfx::gfx_result r = update_display(); 
                if(r!=gfx::gfx_result::success) {
                    m_suspend_count=os; // undo decrement
                    return r;
                }
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result point(gfx::point16 location, pixel_type color) {
            if(!bounds().intersects(location)) {
                return gfx::gfx_result::success;
            }
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            frame bmp(dimensions(),m_frame_buffer,palette());
            bmp.point(location,color);
            if(!m_suspend_count) {
                return update_display();
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(!this->bounds().intersects(bounds)) {
                return gfx::gfx_result::success;
            }
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            frame bmp(dimensions(),m_frame_buffer,palette());
            bmp.fill(bounds,color);
            if(!m_suspend_count) {
                return update_display();
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            if(!m_frame_buffer) {
                return gfx::gfx_result::invalid_state;
            }
            if(!out_color) {
                return gfx::gfx_result::invalid_argument;
            }
            if(!this->bounds().intersects(location)) {
                return gfx::gfx_result::success;
            }
            return frame::point(dimensions(),m_frame_buffer,location,out_color);
        }
    };
}