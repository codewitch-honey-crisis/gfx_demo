#include <Arduino.h>
#include "common/tft_driver.hpp"
#include <gfx_core.hpp>
#include <gfx_pixel.hpp>
#include <gfx_palette.hpp>
#include <gfx_positioning.hpp>
#include <gfx_draw_helpers.hpp>
namespace arduino {
    template<int8_t PinDC, 
            int8_t PinRst, 
            int8_t PinBL, 
            typename Bus, 
            uint8_t Rotation = 0, 
            bool BacklightHigh=false, 
            unsigned int WriteSpeedPercent = 200,
            unsigned int ReadSpeedPercent = WriteSpeedPercent>
    struct ili9341 final {
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_bl = PinBL;
        constexpr static const uint8_t rotation = Rotation & 3;
        constexpr static const size_t max_dma_size = 320*240*2;
        constexpr static const bool backlight_high = BacklightHigh;
        constexpr static const float write_speed_multiplier = (WriteSpeedPercent/100.0);
        constexpr static const float read_speed_multiplier = (ReadSpeedPercent/100.0);
        using type = ili9341;
        using driver = tft_driver<PinDC, PinRst, PinBL, Bus>;
        using bus = Bus;
        using pixel_type = gfx::rgb_pixel<16>;
        using caps = gfx::gfx_caps<false,(bus::dma_size>0),true,true,false,bus::readable,true>;
        ili9341() : m_initialized(false), m_dma_initialized(false), m_in_batch(false) {
        }
        ~ili9341() {
            if(m_dma_initialized) {
                bus::deinitialize_dma();
            }
            if(m_initialized) {
                driver::deinitialize();
            }
        }
        bool initialize() {
            if(!m_initialized) {
                if(driver::initialize()) {
                    bus::begin_initialization();
                    bus::set_speed_multiplier(write_speed_multiplier);
                    bus::begin_write();
                    bus::begin_transaction();
                    driver::send_command(0xEF);
                    driver::send_data8(0x03);
                    driver::send_data8(0x80);
                    driver::send_data8(0x02);

                    driver::send_command(0xCF);
                    driver::send_data8(0x00);
                    driver::send_data8(0XC1);
                    driver::send_data8(0X30);

                    driver::send_command(0xED);
                    driver::send_data8(0x64);
                    driver::send_data8(0x03);
                    driver::send_data8(0X12);
                    driver::send_data8(0X81);

                    driver::send_command(0xE8);
                    driver::send_data8(0x85);
                    driver::send_data8(0x00);
                    driver::send_data8(0x78);

                    driver::send_command(0xCB);
                    driver::send_data8(0x39);
                    driver::send_data8(0x2C);
                    driver::send_data8(0x00);
                    driver::send_data8(0x34);
                    driver::send_data8(0x02);

                    driver::send_command(0xF7);
                    driver::send_data8(0x20);

                    driver::send_command(0xEA);
                    driver::send_data8(0x00);
                    driver::send_data8(0x00);
                    driver::send_command(0xC0);    //Power control
                    driver::send_data8(0x23);   //VRH[5:0]

                    driver::send_command(0xC1);    //Power control
                    driver::send_data8(0x10);   //SAP[2:0];BT[3:0]

                    driver::send_command(0xC5);    //VCM control
                    driver::send_data8(0x3e);
                    driver::send_data8(0x28);

                    driver::send_command(0xC7);    //VCM control2
                    driver::send_data8(0x86);  //--

                    driver::send_command(0x36);    // Memory Access Control
                    driver::send_data8(0x40 | 0x08); // Rotation 0 (portrait mode)

                    driver::send_command(0x3A);
                    driver::send_data8(0x55);

                    driver::send_command(0xB1);
                    driver::send_data8(0x00);
                    driver::send_data8(0x13); // 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz

                    driver::send_command(0xB6);    // Display Function Control
                    driver::send_data8(0x08);
                    driver::send_data8(0x82);
                    driver::send_data8(0x27);

                    driver::send_command(0xF2);    // 3Gamma Function Disable
                    driver::send_data8(0x00);

                    driver::send_command(0x26);    //Gamma curve selected
                    driver::send_data8(0x01);

                    driver::send_command(0xE0);    //Set Gamma
                    driver::send_data8(0x0F);
                    driver::send_data8(0x31);
                    driver::send_data8(0x2B);
                    driver::send_data8(0x0C);
                    driver::send_data8(0x0E);
                    driver::send_data8(0x08);
                    driver::send_data8(0x4E);
                    driver::send_data8(0xF1);
                    driver::send_data8(0x37);
                    driver::send_data8(0x07);
                    driver::send_data8(0x10);
                    driver::send_data8(0x03);
                    driver::send_data8(0x0E);
                    driver::send_data8(0x09);
                    driver::send_data8(0x00);

                    driver::send_command(0xE1);    //Set Gamma
                    driver::send_data8(0x00);
                    driver::send_data8(0x0E);
                    driver::send_data8(0x14);
                    driver::send_data8(0x03);
                    driver::send_data8(0x11);
                    driver::send_data8(0x07);
                    driver::send_data8(0x31);
                    driver::send_data8(0xC1);
                    driver::send_data8(0x48);
                    driver::send_data8(0x08);
                    driver::send_data8(0x0F);
                    driver::send_data8(0x0C);
                    driver::send_data8(0x31);
                    driver::send_data8(0x36);
                    driver::send_data8(0x0F);

                    driver::send_command(0x11);    //Exit Sleep
                    bus::end_transaction();
                    bus::end_write();
                    delay(120);
                    bus::begin_write();
                    bus::begin_transaction();
                    driver::send_command(0x29);    //Display on
                    bus::end_transaction();
                    bus::end_write();
                    bus::end_initialization();
                    bus::begin_write();
                    bus::begin_transaction();
                    apply_rotation();
                    bus::end_transaction();
                    bus::end_write();
                    if(pin_bl>-1) {
                        pinMode(pin_bl,OUTPUT);
                        digitalWrite(pin_bl,backlight_high);
                    }
                    
                    m_initialized = true;
                    
                }
            }
            return m_initialized;
        }
        
        inline gfx::size16 dimensions() const {
            return rotation&1?gfx::size16(320, 240):gfx::size16(240, 320);
        }
        inline gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
        
        inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
            return fill({location.x,location.y,location.x,location.y},color);
        }
        inline gfx::gfx_result point_async(gfx::point16 location, pixel_type color) {
            return point(location,color);
        }
        gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
            if(out_color==nullptr) return gfx::gfx_result::invalid_argument;
            if(!m_initialized || m_in_batch) return gfx::gfx_result::invalid_state;
            if(!bounds().intersects(location)) {
                *out_color = pixel_type();
                return gfx::gfx_result::success;
            }
            bus::dma_wait();
            bus::set_speed_multiplier(read_speed_multiplier);
            bus::begin_read();
            bus::cs_low();
            set_window({location.x,location.y,location.x,location.y},true);
            bus::direction(INPUT);
            bus::read_raw8(); // throw away
            out_color->native_value = ((bus::read_raw8() & 0xF8) << 8) | ((bus::read_raw8() & 0xFC) << 3) | (bus::read_raw8() >> 3);
            bus::cs_high();
            bus::end_read();
            bus::set_speed_multiplier(write_speed_multiplier);
            bus::direction(OUTPUT);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
            if(!initialize()) return gfx::gfx_result::device_error;
            else bus::dma_wait();
            gfx::gfx_result rr = commit_batch();

            if(rr!=gfx::gfx_result::success) {
                return rr;
            }
            if(!bounds.intersects(this->bounds())) return gfx::gfx_result::success;
            const gfx::rect16 r = bounds.normalize().crop(this->bounds());
            bus::begin_write();
            bus::begin_transaction();
            set_window(r);
            bus::write_raw16_repeat(color.native_value,(r.x2-r.x1+1)*(r.y2-r.y1+1));
            bus::end_transaction();
            bus::end_write();
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
        template<typename Source>
        inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            return copy_from_impl(src_rect,src,location,false);
        }
        template<typename Source>
        inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect,const Source& src,gfx::point16 location) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr != gfx::gfx_result::success) {
                return rr;
            }
            if(!m_dma_initialized) {
                if(!bus::initialize_dma()) return gfx::gfx_result::device_error;
                m_dma_initialized = true;
            }
            return copy_from_impl(src_rect,src,location,true);
        }
        template<typename Destination>
        inline gfx::gfx_result copy_to(const gfx::rect16& src_rect,Destination& dst,gfx::point16 location) const {
            if(!src_rect.intersects(bounds())) return gfx::gfx_result::success;
            gfx::rect16 srcr = src_rect.crop(bounds());
            gfx::rect16 dstr= gfx::rect16(location,srcr.dimensions()).crop(dst.bounds());
            srcr=gfx::rect16(srcr.location(),dstr.dimensions());
            return copy_to_helper<Destination,!(pixel_type::template has_channel_names<gfx::channel_name::A>::value)>::copy_to(*this,srcr,dst,dstr);
        }
        template<typename Destination>
        inline gfx::gfx_result copy_to_async(const gfx::rect16& src_rect,Destination& dst,gfx::point16 location) const {
            return copy_to(src_rect,dst,location);
        }
        
        gfx::gfx_result commit_batch() {
            if(m_in_batch) {
                bus::end_transaction();
                bus::end_write();
                m_in_batch = false;
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result commit_batch_async() {
            return commit_batch();
        }
        gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
            if(!initialize()) return gfx::gfx_result::device_error;
            gfx::gfx_result rr = commit_batch();
            if(rr!=gfx::gfx_result::success) {
                return rr;
            }
            const gfx::rect16 r = bounds.normalize();
            bus::begin_write();
            bus::begin_transaction();
            set_window(r);
            m_in_batch = true;
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
            return begin_batch(bounds);
        }
        gfx::gfx_result write_batch(pixel_type color) {
            bus::write_raw16(color.native_value);
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result write_batch_async(pixel_type color) {
            return write_batch(color);
        }
        inline gfx::gfx_result wait_all_async() {
            bus::dma_wait();
            return gfx::gfx_result::success;
        }
    private:
        bool m_initialized;
        bool m_dma_initialized;
        bool m_in_batch;
        static void set_window(const gfx::rect16& bounds, bool read=false) {
            bus::busy_check();
            driver::dc_command();
            bus::write_raw8(0x2A);
            driver::dc_data();
            bus::write_raw16(bounds.x1);
            bus::write_raw16(bounds.x2);
            driver::dc_command();
            bus::write_raw8(0x2B);
            driver::dc_data();
            bus::write_raw16(bounds.y1);
            bus::write_raw16(bounds.y2);
            driver::dc_command();
            bus::write_raw8(read?0x2E:0x2C);
            driver::dc_data();
            
        }
        template<typename Destination, bool AllowBlt=true>
        struct copy_to_helper {
            inline static gfx::gfx_result copy_to(const type& src, const gfx::rect16& srcr,Destination& dst,const gfx::rect16& dstr) {
                if(gfx::helpers::template is_same<typename Destination::pixel_type,pixel_type>::value && dstr.top_left()==gfx::point16(0,0)) {
                    if(AllowBlt && dstr.width()==srcr.width() && dstr.height()==srcr.height()) {
                        if(dstr.top_left()==gfx::point16(0,0)) {
                            return copy_to_fast_helper<Destination>::do_copy(srcr,dst);
                        }
                    } 
                }
                    
                size_t dy=0,dye=dstr.height();
                size_t dx,dxe = dstr.width();
                gfx::gfx_result r;
                gfx::helpers::suspender<Destination,Destination::caps::suspend,false> sustok(dst);
                r = gfx::helpers::batcher<Destination,Destination::caps::batch,false>::begin_batch(dst,dstr,false);
                if(gfx::gfx_result::success!=r) {
                    return r;
                }
                int sox = srcr.left(),soy=srcr.top();
                int dox = dstr.left(),doy=dstr.top();
                while(dy<dye) {
                    dx=0;
                    
                    while(dx<dxe) {
                        pixel_type spx;
                        r=src.point(gfx::point16(sox+dx,soy+dy),&spx);
                        if(gfx::gfx_result::success!=r)
                            return r;
                        typename Destination::pixel_type dpx;
                        if(pixel_type::template has_channel_names<gfx::channel_name::A>::value) {
                            r=gfx::helpers::blend_helper<type,Destination,Destination::caps::read>::do_blend(src,spx,dst,gfx::point16(dox+dx,doy+dy),&dpx);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                        } else {    
                            r=convert_palette(dst,src,spx,&dpx,nullptr);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                             
                        }
                        r = gfx::helpers::batcher<Destination,Destination::caps::batch,false>::write_batch(dst,gfx::point16(dox+dx,doy+dy),dpx,false);
                        if(gfx::gfx_result::success!=r)
                            return r;
                        ++dx;
                    }
                    ++dy;
                }
                return gfx::helpers::batcher<Destination,Destination::caps::batch,false>::commit_batch(dst,false);
                
            }
        };
        
        template<typename Destination> struct copy_to_fast_helper {
            static gfx::gfx_result do_copy(const gfx::rect16& src_rect,Destination& dst) {
                gfx::rect16 r = src_rect.normalize();
                bool entire = false;
                gfx::size16 bssz = r.dimensions();
                size_t bsz = bssz.width*bssz.height*3;
                uint8_t* buf=(uint8_t*)malloc(bsz);
                if(buf!=nullptr) {
                    entire=true;
                } else {
                    bsz = bssz.width*3;
                    buf=(uint8_t*)malloc(bsz);
                }
                if(buf!=nullptr) {
                    free(buf);
                    buf = nullptr;
                }
                using batch=gfx::helpers::batcher<Destination,Destination::caps::batch,Destination::caps::async>;
                if(buf==nullptr) {
                    gfx::helpers::suspender<Destination,Destination::caps::suspend,Destination::caps::async> stok(dst,false);
                    gfx::gfx_result rr= batch::begin_batch(dst,{0,0,uint16_t(r.width()-1),uint16_t(r.height()-1)} ,false);
                    if(rr!=gfx::gfx_result::success) {
                        return rr;
                    }
                    pixel_type px;
                    for(int y = r.y1;y<=r.y2;++y) {
                        for(int x = r.x1;x<=r.x2;++x) {
                            uint8_t buf3[3];
                            buf = buf3;
                            fetch_buffer({uint16_t(x),uint16_t(y),uint16_t(x),uint16_t(y)},buf,3);
                            px.native_value = (((*buf) & 0xF8) << 8);
                            ++buf;
                            px.native_value|=(((*buf) & 0xFC) << 3);
                            ++buf;
                            px.native_value|= (*buf >> 3);
                            batch::write_batch(dst,{uint16_t(x-r.x1),uint16_t(y-r.y1)},px,false);
                        }
                    }
                    rr=batch::commit_batch(dst,false);
                    if(rr!=gfx::gfx_result::success) {
                        return rr;
                    }
                    buf = nullptr;
                } else {
                    if(entire) {
            
                        fetch_buffer(r,buf,bsz);
                        gfx::helpers::suspender<Destination,Destination::caps::suspend,Destination::caps::async> stok(dst,false);
                        gfx::gfx_result rr= batch::begin_batch(dst,{0,0,uint16_t(r.width()-1),uint16_t(r.height()-1)} ,false);
                        if(rr!=gfx::gfx_result::success) {
                            free(buf);
                            return rr;
                        }
                        uint8_t* bbuf = buf;
                        while(bsz) {
                            pixel_type px;
                            uint16_t x,y;
                            x=0;y=0;
                            px.native_value = (((*bbuf) & 0xF8) << 8);
                            ++bbuf;
                            px.native_value|=(((*bbuf) & 0xFC) << 3);
                            ++bbuf;
                            px.native_value|= (*bbuf >> 3);
                            ++bbuf;
                            ++x;
                            if(x>r.x2-r.x1) {
                                x=0;
                                ++y;
                            }
                            batch::write_batch(dst,{x,y},px,false);
                            bsz-=3;
                        }
                        rr=batch::commit_batch(dst,false);
                        if(rr!=gfx::gfx_result::success) {
                            free(buf);
                            return rr;
                        }
                    } else {
                        gfx::helpers::suspender<Destination,Destination::caps::suspend,Destination::caps::async> stok(dst,false);
                        for(int y = r.y1;y<=r.y2;++y) {
                            fetch_buffer(r,buf,bsz);
                            gfx::gfx_result rr= batch::begin_batch(dst,{0,0,uint16_t(r.width()-1),uint16_t(r.height()-1)} ,false);
                            if(rr!=gfx::gfx_result::success) {
                                free(buf);
                                return rr;
                            }   
                            size_t bbsz = bsz;
                            uint8_t* bbuf = buf;
                            while(bbsz) {
                                pixel_type px;
                                uint16_t x=0;
                                px.native_value = (((*bbuf) & 0xF8) << 8);
                                ++bbuf;
                                px.native_value|=(((*bbuf) & 0xFC) << 3);
                                ++bbuf;
                                px.native_value|= (*bbuf >> 3);
                                ++bbuf;
                                ++x;
                                batch::write_batch(dst,{x,uint16_t(y-r.y1)},px,false);
                                bbsz-=3;
                            }   
                            rr=batch::commit_batch(dst,false);
                            if(rr!=gfx::gfx_result::success) {
                                free(buf);
                                return rr;
                            }
                        }
                    }
                }
                if(buf!=nullptr) {
                    free(buf);
                }
                return gfx::gfx_result::success;
            }
            
            static void fetch_buffer(const gfx::rect16& r, uint8_t* buf,size_t len) {
         
                bus::dma_wait();
                bus::set_speed_multiplier(read_speed_multiplier);
                bus::begin_read();
                bus::cs_low();
                set_window(r,true);
                bus::direction(INPUT);
                bus::read_raw8(); // throw away
                bus::read_raw(buf,len);
                bus::cs_high();
                bus::end_read();
                bus::set_speed_multiplier(write_speed_multiplier);
                bus::direction(OUTPUT);
            }
        };
        template<typename Source,bool Blt> 
        struct copy_from_helper {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async)  {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                gfx::gfx_result rr;
               
                rr=this_->begin_batch(dstr);
                
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
                        rr=gfx::convert_palette_to(src,pp,&p,nullptr);
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                       
                        rr = this_->write_batch(p);
                        
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                    }
                }
                
                rr=this_->batch_commit();
                
                return rr;
            }
        };
        
        template<typename Source> 
        struct copy_from_helper<Source,true> {
            static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,const Source& src,gfx::rect16 srcr,bool async) {
                if(async) {
                    bus::dma_wait();
                }
                // direct blt
                if(src.bounds().width()==srcr.width() && srcr.x1==0) {
                    bus::begin_write();
                    set_window(dstr);
                    if(async) {
                        bus::write_raw_dma(src.begin()+(srcr.y1*src.dimensions().width*2),(srcr.y2-srcr.y1+1)*src.dimensions().width*2);
                    } else {
                        bus::write_raw(src.begin()+(srcr.y1*src.dimensions().width*2),(srcr.y2-srcr.y1+1)*src.dimensions().width*2);
                    }

                    bus::end_write();
                    return gfx::gfx_result::success;
                }
                // line by line blt
                uint16_t yy=0;
                uint16_t hh=srcr.height();
                uint16_t ww = src.dimensions().width;
                uint16_t pitch = (srcr.x2 - srcr.x1+1)*2;
                bus::begin_write();
                bus::begin_transaction();
                while(yy<hh-!!async) {
                    gfx::rect16 dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.y1+yy)};
                    set_window(dr);
                    bus::write_raw(src.begin()+2*(ww*(srcr.y1+yy)+srcr.x1),pitch);
                    ++yy;
                }
                if(async) {
                    gfx::rect16 dr = {dstr.x1,uint16_t(dstr.y1+yy),dstr.x2,uint16_t(dstr.y1+yy)};
                    set_window(dr);
                    bus::write_raw_dma(src.begin()+2*(ww*(srcr.y1+yy)+srcr.x1),pitch);
                }
                bus::end_transaction();
                bus::end_write();
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
        
        static void apply_rotation() {   
            bus::begin_write();       
            driver::send_command(0x36);
            switch (rotation) {
                case 0:
                // portrait
                driver::send_data8(0x40 | 0x08);
                break;
                case 1:
                // landscape
                driver::send_data8(0x20 | 0x08);
                break;
                case 2:
                // portrait
                driver::send_data8(0x80 | 0x08);
                break;
                case 3:
                // landscape
                driver::send_data8(0x20 | 0x40 | 0x80 | 0x08);
                break;

            }
            delayMicroseconds(10);

            bus::end_write();
        }
        
    };
}