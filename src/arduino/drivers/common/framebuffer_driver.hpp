#pragma once
#include <Arduino.h>
#include <gfx_core.hpp>
#include <gfx_pixel.hpp>
#include <gfx_palette.hpp>
#include <gfx_positioning.hpp>
#include <gfx_draw_helpers.hpp>
#include <gfx_bitmap.hpp>
namespace arduino {
    template<uint16_t Width,uint16_t Height,typename PixelType,typename PaletteType = gfx::palette<PixelType,PixelType>>
    class framebuffer_driver {
        void*(*m_allocator)(size_t size);
        void(*m_deallocator)(void* pointer);
        uint8_t* m_framebuffer;
        gfx::rect16 m_suspend_bounds;
        int m_suspend_count;
    public:
        using type = framebuffer_driver;
        using pixel_type = PixelType;
        using palette_type = PaletteType;
        using bitmap_type = gfx::bitmap<PixelType,PaletteType>;
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const size_t framebuffer_size = ((width*height*pixel_type::bit_depth)+7)/8;
    private:
        const palette_type* m_palette;
        void update_suspend_rect(const gfx::rect16& bounds) {
            if(uint16_t(-1)==m_suspend_bounds.x1==m_suspend_bounds.x2==m_suspend_bounds.y1==m_suspend_bounds.y2) {
                m_suspend_bounds = bounds;
                return;
            }
            if(bounds.x1<m_suspend_bounds.x1) {
                m_suspend_bounds.x1 = bounds.x1;
            }
            if(bounds.y1<m_suspend_bounds.y1) {
                m_suspend_bounds.y1 = bounds.y1;
            }
            if(bounds.x2>m_suspend_bounds.x2) {
                m_suspend_bounds.x2 = bounds.x2;
            }
            if(bounds.y2>m_suspend_bounds.y2) {
                m_suspend_bounds.y2 = bounds.y2;
            }
        }
    protected:
        framebuffer_driver(void*(allocator)(size_t size)=::malloc,void(deallocator)(void*)=::free,const palette_type* palette=nullptr) :
                m_allocator(allocator),
                m_deallocator(deallocator),
                m_framebuffer(nullptr),
                m_suspend_bounds(uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)),
                m_suspend_count(0),
                m_palette(palette)
                 {
        }
        inline bool intialized() const { return m_framebuffer; }
        bool initialize() {
            if(!m_framebuffer) {
                m_framebuffer = (uint8_t*)m_allocator(framebuffer_size);
                if(m_framebuffer) {
                    if(!initialize_driver()) {
                        m_deallocator(m_framebuffer);
                        m_framebuffer = nullptr;
                    }
                }
            }
            return m_framebuffer;
        }
        void deinitialize() {
            if(m_framebuffer) {
                deinitialize_driver();
                m_deallocator(m_framebuffer);
                m_framebuffer = nullptr;
            }
        }
        virtual gfx::gfx_result update_display_framebuffer(const gfx::rect16& bounds)=0;
        virtual bool initialize_driver() = 0;
        virtual void deinitialize_driver() {}
    public:
        virtual ~framebuffer_driver() {
            deinitialize();
        }
        
        using caps = gfx::gfx_caps<false,false,false,false,true,true,true>;
        inline uint8_t* begin() const { return m_framebuffer;}
        // indicates just past the end of the bitmap buffer
        inline uint8_t* end() const {
            return begin()+framebuffer_size;
        }
        inline const palette_type* palette() const {
            return m_palette;
        }
        inline gfx::size16 dimensions() const { return {width,height}; }
        inline gfx::rect16 bounds() const { return dimensions().bounds();}
        
        gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type px;
            return fill(bounds,px);
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(!initialize()) {
                return gfx::gfx_result::out_of_memory;
            }
            if(bounds.intersects(this->bounds())) {
                bitmap_type bmp(dimensions(),m_framebuffer,m_palette);
                bmp.fill(bounds,color);
                const gfx::rect16 rr = bounds.normalize().crop(this->bounds());
                if(!m_suspend_count) {
                    return update_display_framebuffer(rr);
                } else {
                    update_suspend_rect(rr);
                }
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            if(!initialize()) {
                return gfx::gfx_result::out_of_memory;
            }
            if(bounds().intersects(location)) {
                bitmap_type bmp(dimensions(),m_framebuffer,m_palette);
                bmp.point(location,color);
                if(!m_suspend_count) {
                    return update_display_framebuffer({location.x,location.y,location.x,location.y});
                } else {
                    update_suspend_rect({location.x,location.y,location.x,location.y});
                }
            }
            return gfx::gfx_result::success;
        }
        template<typename Source>
        inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            if(!initialize()) {
                return gfx::gfx_result::out_of_memory;
            }
            return copy_from_impl(src_rect,src,location);
        }
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            if(m_framebuffer==nullptr) {
                return gfx::gfx_result::invalid_state;
            }
            if(bounds().intersects(location)) {
                bitmap_type bmp(dimensions(),m_framebuffer,m_palette);
                bmp.point(dimensions(),m_framebuffer,location,out_color);
            }
            return gfx::gfx_result::success;
        }
        template<typename Destination>
        inline gfx::gfx_result copy_to(const gfx::rect16& src_rect,Destination& dst,gfx::point16 location) const {
            if(!this->initialize()) {
                return gfx::gfx_result::out_of_memory;
            }
            bitmap_type bmp(dimensions(),m_framebuffer,m_palette);
            gfx::gfx_result r = bmp.copy_to(src_rect,dst,location);
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    this->update_display_framebuffer(m_suspend_bounds);
                }
                
            } 
            return gfx::gfx_result::success;
        }
    };
}