#pragma once
#include <gfx_cpp14.hpp>
#define WIDL_EXPLICIT_AGGREGATE_RETURNS
#include <d2d1.h>
namespace windows {
    class dxd2d final {
        size_t m_suspend_count;
        ID2D1RenderTarget* m_prender;
        static gfx::gfx_result xlt_err(HRESULT hr) {
            switch(hr) {
                case 0:
                    return gfx::gfx_result::success;
                case D2DERR_RECREATE_TARGET:
                    return gfx::gfx_result::canceled;
                default:
                    return gfx::gfx_result::unknown_error;
            }
        }
    public:
        using pixel_type = gfx::rgba_pixel<32>;
        using caps = gfx::gfx_caps<false,false,false,false,true,false,false>;
        dxd2d() : m_suspend_count(0),m_prender(nullptr) {
        }
        dxd2d(ID2D1RenderTarget* target) : m_suspend_count(0), m_prender(target) {
            if(target != nullptr) {
                target->AddRef();
            }
        }
        virtual ~dxd2d() {
            if(m_prender != nullptr) {
                m_prender->Release();
            }
        }
        void render_target(ID2D1RenderTarget* target) {
            if(m_prender != nullptr) {
                m_prender->Release();
            }
            if(target != nullptr) {
                target->AddRef();
            }
            m_prender = target;
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(m_prender!=nullptr) {
                D2D1_COLOR_F col;
                col.r = color.channelr<gfx::channel_name::R>();
                col.g = color.channelr<gfx::channel_name::G>();
                col.b = color.channelr<gfx::channel_name::B>();
                col.a = color.channelr<gfx::channel_name::A>();
                ID2D1SolidColorBrush* brush;
                gfx::gfx_result gr = xlt_err(m_prender->CreateSolidColorBrush(col,&brush));
                if(gr!=gfx::gfx_result::success) {
                    return gr;
                }
                D2D1_RECT_F rf;
                gfx::rect16 sr = bounds.normalize();
                rf.left = sr.x1;
                rf.top = sr.y1;
                rf.right = sr.x2+.99999;
                rf.bottom = sr.y2+.99999;
                suspend();
                m_prender->FillRectangle(rf,brush);
                gr=resume();
                brush->Release();
                return gr;
            }
            return gfx::gfx_result::invalid_state;
        }
        inline gfx::gfx_result suspend() {
            if(m_prender==nullptr) {
                return gfx::gfx_result::invalid_state;
            }
            if(m_suspend_count==0) {
                m_prender->BeginDraw();
            }
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result resume(bool force = false) {
            if(m_prender==nullptr) {
                return gfx::gfx_result::invalid_state;
            }
            if(m_suspend_count==0) {
                return gfx::gfx_result::success;
            }
            --m_suspend_count;
            if(m_suspend_count==0) {
                return xlt_err(m_prender->EndDraw());
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            return fill({location.x,location.y,location.x,location.y},color);
        }
        gfx::size16 dimensions() const {
            if(m_prender!=nullptr) {
                D2D1_SIZE_F s = m_prender->GetSize();
                return {uint16_t(s.width),uint16_t(s.height)};
            }
            return {0,0};
        }
        inline gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
    };
}