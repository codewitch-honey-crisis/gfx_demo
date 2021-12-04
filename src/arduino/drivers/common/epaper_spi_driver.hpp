#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "gfx_bitmap.hpp"
namespace arduino {
    enum struct epaper_spi_driver_result {
        success = 0,
        invalid_argument,
        io_error,
        io_busy,
        out_of_memory,
        timeout,
        not_supported
    };
    namespace epaper_spi_driver_helpers {
        template<typename PixelTypeLhs,typename PixelTypeRhs,bool Dithered> struct convert_helper {
            static gfx::gfx_result convert(PixelTypeLhs pixel,PixelTypeRhs* out_pixel) {
                return gfx::convert(pixel,out_pixel);
            }
        };
        template<typename PixelTypeLhs,typename PixelTypeRhs> struct convert_helper<PixelTypeLhs,PixelTypeRhs,false> {
            static gfx::gfx_result convert(PixelTypeLhs pixel,PixelTypeRhs* out_pixel) {
                out_pixel->native_value = pixel.native_value;
                return gfx::gfx_result::success;
            }
        };
        template<typename Target,bool Dithered,bool Dithering> struct write_pixel_helper {
            static epaper_spi_driver_result write_pixel(Target* target,typename Target::mapped_pixel_type pixel) {

                return epaper_spi_driver_result::success;
            }
            static epaper_spi_driver_result write_pixel(Target* target,typename Target::native_pixel_type pixel) {
                return epaper_spi_driver_result::success;
            }
        };
        template<typename Target> struct write_pixel_helper<Target,true,true> {
            static epaper_spi_driver_result write_pixel(Target* target,typename Target::native_pixel_type pixel) {
                return target->write_next_pixel(pixel);
            }
        };
        template<typename Target> struct write_pixel_helper<Target,false,false> {
            static epaper_spi_driver_result write_pixel(Target* target,typename Target::native_pixel_type pixel) {
                return target->write_next_pixel(pixel);
            }
        };
    }
    // for faster function calls:
    struct epaper_spi_driver_rect {
        uint16_t x1;
        uint16_t y1;
        uint16_t x2;
        uint16_t y2;
    };
    template<uint16_t Width,
        uint16_t Height,
        int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy,
        uint32_t ClockSpeed>
    struct epaper_spi_driver_base {
        
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_busy = PinBusy;
        constexpr static const uint32_t clock_speed = ClockSpeed;
    private:
        bool m_initialized;
        SPIClass& m_spi;
    protected:
        epaper_spi_driver_base(SPIClass& spi) : m_initialized(false),m_spi(spi) {
            
        }
        static bool normalize_values(epaper_spi_driver_rect& r,bool check_bounds=true) {
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
        
        virtual epaper_spi_driver_result initialize_impl() {
            return epaper_spi_driver_result::success;
        }
        virtual epaper_spi_driver_result deinitialize_impl() {
            return epaper_spi_driver_result::success;
        }
       
    public:
        epaper_spi_driver_result deinitialize() {
            if(m_initialized) {
                epaper_spi_driver_result r= deinitialize_impl();
                if(epaper_spi_driver_result::success!=r) {
                    return r;
                }
                m_initialized = false;
            }
            return epaper_spi_driver_result::success;
        }
        inline bool initialized() const {
            return m_initialized;
        }
        epaper_spi_driver_result initialize() {
            if(!m_initialized) {
                epaper_spi_driver_result r=initialize_impl();
                if(epaper_spi_driver_result::success!=r) {
                    return r;
                }
                m_initialized = true;
            }
            return epaper_spi_driver_result::success;
        }

        void reset() {
            if(pin_rst>=0) {
                digitalWrite(pin_rst,LOW);
                delay(10);
                digitalWrite(pin_rst,HIGH);
                delay(10);
            }
        }
    };
    template<uint16_t Width,
        uint16_t Height,
        typename NativePaletteType, 
        typename PixelType, 
        int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy,
        uint32_t ClockSpeed>
    struct epaper_spi_driver : public epaper_spi_driver_base<Width,Height,PinCS,PinDC,PinRst,PinBusy,ClockSpeed> {
        static_assert(!PixelType::template has_channel_names<typename gfx::channel_name::index>::value || gfx::helpers::is_same<PixelType,typename NativePaletteType::pixel_type>::value,"PixelType must not be indexed or it must be the native indexed pixel type.");
        friend class write_pixel_helper;
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_busy = PinBusy;
        constexpr static const uint32_t clock_speed = ClockSpeed;
        constexpr static const bool dithered = !gfx::helpers::is_same<PixelType,typename NativePaletteType::pixel_type>::value;
        using palette_type = NativePaletteType;
        using mapped_pixel_type = typename NativePaletteType::mapped_pixel_type;
        using native_pixel_type = typename NativePaletteType::pixel_type;
        using pixel_type = PixelType;
        using frame_buffer_type = gfx::large_bitmap<pixel_type,palette_type>;
        
    private:
        frame_buffer_type m_frame_buffer;
        int m_suspend_count;
        const palette_type m_native_palette;
        
    protected:
        using base_type = epaper_spi_driver_base<Width,Height,PinCS,PinDC,PinRst,PinBusy,ClockSpeed>;
        
        inline const frame_buffer_type* frame_buffer() const {
            return &m_frame_buffer;
        };
        virtual epaper_spi_driver_result initialize_impl() {
            m_frame_buffer = frame_buffer_type(gfx::size16(width,height),1,&m_native_palette);
            if(!m_frame_buffer.initialized()) {
                return epaper_spi_driver_result::out_of_memory;
            }
            return epaper_spi_driver_result::success;
        }
        virtual epaper_spi_driver_result deinitialize_impl() {
            m_frame_buffer = frame_buffer_type();
            return epaper_spi_driver_result::success;
            
        }
        virtual epaper_spi_driver_result display_update_impl()=0;
        
        epaper_spi_driver_result process_frame_buffer() {
            this->initialize();
            if(dithered) {
                const bool use_fast = true;

                gfx::gfx_result r=gfx::helpers::dither_prepare(&m_native_palette);
                if(gfx::gfx_result::success!=r) {
                    return epaper_spi_driver_result::out_of_memory;
                }
                if(use_fast) {
                    for(int y = 0; y < height; ++y) {
                        for(int x = 0; x<width; ++x) {
                            double map_value = gfx::helpers::threshold_map_fast[(x & 7) + ((y & 7) << 3)];
                            pixel_type px;
                            typename palette_type::mapped_pixel_type mpx;
                            gfx::gfx_result r = m_frame_buffer.point(gfx::point16(x,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return epaper_spi_driver_result::not_supported;
                            }
                            r=epaper_spi_driver_helpers::convert_helper<pixel_type,typename palette_type::mapped_pixel_type,dithered>::convert(px,&mpx);
                            if(gfx::gfx_result::success!=r) {
                                return epaper_spi_driver_result::not_supported;
                            }
                            gfx::helpers::mixing_plan_data_fast plan;
                            gfx::helpers::dither_mixing_plan_fast(&m_native_palette,mpx,&plan);
                            epaper_spi_driver_result rr=epaper_spi_driver_helpers::write_pixel_helper<epaper_spi_driver,dithered,true>::write_pixel(this,plan.colors[map_value<plan.ratio?1:0]);
                            if(epaper_spi_driver_result::success!=rr) {
                                return rr;
                            }
                        }
                    }
                     
                } else {   
                    native_pixel_type plan[64];
                    for(int y = 0; y < height; ++y) {
                        for(int x = 0; x<width; ++x) {
                            gfx::point16 pt(x,y);
                            pixel_type px;
                            typename palette_type::mapped_pixel_type mpx;
                            gfx::gfx_result r = m_frame_buffer.point(pt,&px);
                            if(gfx::gfx_result::success!=r) {
                                return epaper_spi_driver_result::not_supported;
                            }
                            r=epaper_spi_driver_helpers::convert_helper<pixel_type,typename palette_type::mapped_pixel_type,dithered>::convert(px,&mpx);
                            if(gfx::gfx_result::success!=r) {
                                return epaper_spi_driver_result::not_supported;
                            }
                            unsigned map_value = gfx::helpers::threshold_map[(x & 7) + ((y & 7) << 3)];
                            r=gfx::helpers::dither_mixing_plan(&m_native_palette,mpx,plan);
                            if(gfx::gfx_result::success!=r) {
                                return epaper_spi_driver_result::not_supported;
                            }
                            epaper_spi_driver_result rr=epaper_spi_driver_helpers::write_pixel_helper<epaper_spi_driver,dithered,true>::write_pixel(this,plan[map_value]);
                            //epaper_spi_driver_result rr =write_next_pixel(plan[map_value]);
                            if(epaper_spi_driver_result::success!= rr) {
                                return rr;
                            }
                        }
                    }
                    r=gfx::helpers::unprepare();
                    if(gfx::gfx_result::success!=r) {
                        return epaper_spi_driver_result::out_of_memory;
                    }
                }
            } else {
                for(int y = 0; y < height; ++y) {
                    for(int x = 0; x<width; ++x) {
                        gfx::point16 pt(x,y);
                        pixel_type px;
                        gfx::gfx_result r = m_frame_buffer.point(pt,&px);
                        if(gfx::gfx_result::success!=r) {
                            return epaper_spi_driver_result::not_supported;
                        }
                        epaper_spi_driver_result rr=epaper_spi_driver_helpers::write_pixel_helper<epaper_spi_driver,dithered,false>::write_pixel(this,px);
                        
                        //epaper_spi_driver_result rr =write_next_pixel(px);
                        if(epaper_spi_driver_result::success!= rr) {
                            return rr;
                        }
                    }
                }
            }
            return epaper_spi_driver_result::success;
        }
    private:
        epaper_spi_driver_result display_update() {
            if(0==m_suspend_count) {
                
                epaper_spi_driver_result r = this->initialize();
                if(epaper_spi_driver_result::success!=r) {
                    return r;
                }
                return display_update_impl();
            }
            return epaper_spi_driver_result::success;
        }
    public:
        epaper_spi_driver(SPIClass& spi) : base_type(spi),m_native_palette() {

        }
        // HACK: This shouldn't be public but long story short I can't get  a specialization to call it if it's not public
        virtual epaper_spi_driver_result write_next_pixel(native_pixel_type pixel)=0;
        
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
    private:
        static gfx::gfx_result xlt_err(epaper_spi_driver_result r) {
            switch(r) {
                case epaper_spi_driver_result::io_error:
                    return gfx::gfx_result::device_error;
                case epaper_spi_driver_result::out_of_memory:
                    return gfx::gfx_result::out_of_memory;
                case epaper_spi_driver_result::success:
                    return gfx::gfx_result::success;
                case epaper_spi_driver_result::not_supported:
                    return gfx::gfx_result::not_supported;
                case epaper_spi_driver_result::invalid_argument:
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
            return m_frame_buffer.point(location,out_color);    
        }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            gfx::gfx_result r=m_frame_buffer.point(location,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            epaper_spi_driver_result rr= display_update();
            if(epaper_spi_driver_result::success!=rr) {
                return xlt_err(rr);
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type color) {
            gfx::gfx_result r=m_frame_buffer.fill(rect,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            epaper_spi_driver_result rr = display_update();
            if(epaper_spi_driver_result::success!=rr) {
                return xlt_err(rr);
            }
            return gfx::gfx_result::success;
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
            epaper_spi_driver_result rr = display_update();
            if(epaper_spi_driver_result::success!=rr) {
                return xlt_err(rr);
            }
            return gfx::gfx_result::success;
            
        }
        inline const palette_type* palette() const {
            return &m_native_palette;
        }
    
    };
}