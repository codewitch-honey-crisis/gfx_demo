#pragma once
// enable this once TFT_eSPI is updated to support it
// #define HTCW_TFT_ESPI_ASYNC
#include <gfx_core.hpp>
#include <gfx_positioning.hpp>
#include <gfx_pixel.hpp>
#include <TFT_eSPI.h>
namespace arduino {
    template<bool CanRead=false, bool Async=
#if (defined(ESP32_DMA) || defined(RP2040_DMA) || defined(STM32_DMA)) && defined(HTCW_TFT_ESPI_ASYNC) && !defined(TFT_PARALLEL_8_BIT)
        true
#else
        false
#endif
    >
    class gfx_tft_espi final {
        TFT_eSPI& m_tft_espi;
        bool m_batch;
    public:
        gfx_tft_espi() : m_batch(false) {
            
        }
        using type = gfx_tft_espi;
        using pixel_type = gfx::rgb_pixel<16>;
        using caps = gfx::gfx_caps<false,Async,true,true,false,CanRead,false>;
    private:
        template<typename Source,bool Blt> 
        struct copy_from_helper {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async)  {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                gfx::gfx_result rr;
                if(Async) {
                    if(!async)
                        rr=this_->write_batch(dstr);
                    else
                        rr=this_->write_batch_async(dstr);
                } else {
                    rr=this_->write_batch(dstr);
                }
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
                        rr=gfx::convert_palette_to(src,pp,&p);
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                        if(Async) {
                            if(!async)
                                rr = this_->write_batch(p);
                            else
                                rr = this_->write_batch_async(p);
                        } else {
                            rr = this_->write_batch(p);
                        }
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                    }
                }
                if(Async) {
                    if(!async)
                        rr=this_->batch_commit();
                    else
                        rr=this_->batch_commit_async();
                } else {
                    rr=this_->batch_commit();
                }
                return rr;
            }
        };
        
        template<typename Source> 
        struct copy_from_helper<Source,true> {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async) {
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    if(Async) {
                        // TODO: wait for Bodmer to update TFT_eSPI to fix pushImageDMA so it can be used here
                        if(!async) {
                            this_->m_tft_espi.pushImage(dstr.x1,dstr.y1,dstr.x2-dstr.x1+1,dstr.y2-dstr.y1+1,(const uint16_t*)(src.begin()+(srcr.y1*src.dimensions().width*2)));
                        } else {
                            this_->m_tft_espi.pushImageDMA(dstr.x1,dstr.y1,dstr.x2-dstr.x1+1,dstr.y2-dstr.y1+1,(const uint16_t*)(src.begin()+(srcr.y1*src.dimensions().width*2)));
                        }
                    } else {
                        this_->m_tft_espi.pushImage(dstr.x1,dstr.y1,dstr.x2-dstr.x1+1,dstr.y2-dstr.y1+1,(const uint16_t*)(src.begin()+(srcr.y1*src.dimensions().width*2)));
                    }
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                while(yy<hh) {
                    gfx::rect16 dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.x2+yy)};
                    if(Async) {
                        // TODO: wait for Bodmer to update TFT_eSPI to fix pushImageDMA so it can be used here
                        if(!async) {
                            this_->m_tft_espi.pushImage(dr.x1,dr.y1,dr.x2-dr.x1+1,dr.y2-dr.y1+1,(const uint16_t*)(src.begin()+(ww*(srcr.y1+yy)+srcr.x1)));
                        } else {
                            this_->m_tft_espi.pushImageDMA(dr.x1,dr.y1,dr.x2-dr.x1+1,dr.y2-dr.y1+1,(const uint16_t*)(src.begin()+(ww*(srcr.y1+yy)+srcr.x1)));
                        }
                    } else {
                        this_->m_tft_espi.pushImage(dr.x1,dr.y1,dr.x2-dr.x1+1,dr.y2-dr.y1+1,(const uint16_t*)(src.begin()+(ww*(srcr.y1+yy)+srcr.x1)));
                    }
                    ++yy;
                }
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
public:
        inline gfx_tft_espi(TFT_eSPI& tft_espi) : m_tft_espi(tft_espi) {
        }
        inline gfx::size16 dimensions() const {
            return {uint16_t(m_tft_espi.width()),uint16_t(m_tft_espi.height())};
        }
        inline gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
        inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            m_tft_espi.drawPixel(location.x,location.y,color.native_value);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result point_async(gfx::point16 location, pixel_type color) {
            return point(location,color);
        }
        inline gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
            if(out_color==nullptr) return gfx::gfx_result::invalid_argument;
            if(m_batch) return gfx::gfx_result::invalid_state;
            if(Async) {
                m_tft_espi.dmaWait();
            }
            out_color->native_value=m_tft_espi.readPixel(location.x,location.y);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            const gfx::rect16 r = bounds.normalize();
            m_tft_espi.fillRect(r.x1,r.y1,r.x2-r.x1+1,r.y2-r.y1+1,color.native_value);
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
        gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            m_batch = true;
            gfx::rect16 r = bounds.normalize();
            m_tft_espi.setAddrWindow(r.x1,r.y1,r.x2-r.x1+1,r.y2-r.y1+1);
            m_batch = true;
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
            return begin_batch(bounds);
        }
        gfx::gfx_result write_batch(pixel_type color)  {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            if(!m_batch) return gfx::gfx_result::invalid_state;
            m_tft_espi.pushColor(color.native_value);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result write_batch_async(pixel_type color)  {
            return write_batch(color);
        }
        gfx::gfx_result commit_batch() {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            m_batch = false;    
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result commit_batch_async() {
            return commit_batch();
        }
        template<typename Source> 
        inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            return copy_from_impl(src_rect,src,location,false);
        }
        template<typename Source>
        inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            return copy_from_impl(src_rect,src,location,true);
        }
        gfx::gfx_result wait_all_async() {
            if(Async) {
                m_tft_espi.dmaWait();
            }
            return gfx::gfx_result::success;
        }
    };
}