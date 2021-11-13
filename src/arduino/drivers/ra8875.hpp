#pragma once
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"
#include <Arduino.h>
#include <SPI.h>
namespace arduino {
    template<uint16_t Width,
            uint16_t Height,
            int8_t PinCS,
            int8_t PinRst,
            uint32_t ClockSpeed = 20*1000*1000, uint32_t InitClockSpeed = 1*1000*1000> 
    struct ra8875 final {
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const uint32_t clock_speed = ClockSpeed;
        constexpr static const uint32_t init_clock_speed = InitClockSpeed;
    private:
        bool m_initialized;
        uint8_t m_voffset;
        bool m_in_batch;
        size_t m_batch_offset;
        gfx::rect16 m_batch_bounds;
        SPIClass& m_spi;
        SPISettings m_spi_settings;
        uint8_t reg(uint8_t reg) const {
            send_command(reg);
            return recv_data();
        }
        void reg(uint8_t reg, uint8_t value) const {
            send_command(reg);
            send_data(value);
        }
        void sendx(uint8_t x, uint8_t value) const {
            digitalWrite(pin_cs, LOW);
            m_spi.beginTransaction(m_spi_settings);
            m_spi.transfer(x);
            m_spi.transfer(value);
            m_spi.endTransaction();
            digitalWrite(pin_cs, HIGH);
        }
        uint8_t recvx(uint8_t x) const {
            digitalWrite(pin_cs, LOW);
            m_spi.beginTransaction(m_spi_settings);
            m_spi.transfer(x);
            uint8_t result = m_spi.transfer(0);

            m_spi.endTransaction();
            digitalWrite(pin_cs, HIGH);
            return result;
        }
        inline void send_command(uint8_t value) const {
            static const uint8_t RA8875_CMDWRITE = 0x80;
            sendx(RA8875_CMDWRITE,value);
        }
        inline void send_data(uint8_t value) const {
            static const uint8_t RA8875_DATAWRITE = 0x00;
            sendx(RA8875_DATAWRITE,value);
        }
        inline uint8_t recv_data() const {
            static const uint8_t RA8875_DATAREAD = 0x40;
            return recvx(RA8875_DATAREAD);
        }
        inline uint8_t recv_command() const {
            static const uint8_t RA8875_CMDREAD = 0xC0;
            return recvx(RA8875_CMDREAD);
        }
        // waits for an operation to complete
        void poll(uint8_t rg, uint8_t flag) const {
            while (1) {
                uint8_t result = reg(rg);
                if (!(result & flag))
                return;
            }
        }
        bool initialize_pll() {
            static const uint8_t RA8875_PLLC1 = 0x88;
            static const uint8_t  RA8875_PLLC1_PLLDIV1 = 0x00;
            static const uint8_t  RA8875_PLLC2 = 0x89;
            static const uint8_t  RA8875_PLLC2_DIV4 = 0x02;
            if ((width == 480 && height == 80) || (width==480 && height==128) || (width==480 && height==272))
            {
                reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
                delay(1);
                reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
                delay(1);
                return true;
            }
            else if(width==800 && height==480)
            {
                reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
                delay(1);
                reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
                delay(1);
                return true;
            }
            return false;
        }
        bool initialize_display() {
            static const uint8_t RA8875_SYSR = 0x10;
            static const uint8_t RA8875_SYSR_16BPP = 0x0C;
            static const uint8_t RA8875_SYSR_MCU8 = 0x00;
            static const uint8_t RA8875_PCSR = 0x04;
            static const uint8_t RA8875_PCSR_PDATL = 0x80;
            static const uint8_t RA8875_PCSR_2CLK = 0x01;
            static const uint8_t RA8875_PCSR_4CLK = 0x02;
            static const uint8_t RA8875_HDWR = 0x14;
            static const uint8_t RA8875_HNDFTR = 0x15;
            static const uint8_t RA8875_HNDFTR_DE_HIGH = 0x00;
            static const uint8_t RA8875_HNDR = 0x16;
            static const uint8_t RA8875_HSTR = 0x17;
            static const uint8_t RA8875_HPWR = 0x18;
            static const uint8_t RA8875_HPWR_LOW = 0x00;
            static const uint8_t RA8875_VDHR0 = 0x19;
            static const uint8_t RA8875_VDHR1 = 0x1A;
            static const uint8_t RA8875_VNDR0 = 0x1B;
            static const uint8_t RA8875_VNDR1 = 0x1C;
            static const uint8_t RA8875_VSTR0 = 0x1D;
            static const uint8_t RA8875_VSTR1 = 0x1E;
            static const uint8_t RA8875_VPWR = 0x1F;
            static const uint8_t RA8875_VPWR_LOW = 0x00;
            static const uint8_t RA8875_HSAW0 = 0x30;
            static const uint8_t RA8875_HSAW1 = 0x31;
            static const uint8_t RA8875_VSAW0 = 0x32;
            static const uint8_t RA8875_VSAW1 = 0x33;
            static const uint8_t RA8875_HEAW0 = 0x34;
            static const uint8_t RA8875_HEAW1 = 0x35;
            static const uint8_t RA8875_VEAW0 = 0x36;
            static const uint8_t RA8875_VEAW1 = 0x37;
            static const uint8_t RA8875_MCLR = 0x8E;         
            static const uint8_t RA8875_MCLR_START = 0x80;
            static const uint8_t RA8875_MCLR_FULL = 0x00;
            
            reg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

            uint8_t pixclk;
            uint8_t hsync_start;
            uint8_t hsync_pw;
            uint8_t hsync_finetune;
            uint8_t hsync_nondisp;
            uint8_t vsync_pw;
            uint16_t vsync_nondisp;
            uint16_t vsync_start;

            if (width == 480 && height == 80) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
                hsync_nondisp = 10;
                hsync_start = 8;
                hsync_pw = 48;
                hsync_finetune = 0;
                vsync_nondisp = 3;
                vsync_start = 8;
                vsync_pw = 10;
                m_voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
            } else if (width == 480 && (height == 128 || height == 272)) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
                hsync_nondisp = 10;
                hsync_start = 8;
                hsync_pw = 48;
                hsync_finetune = 0;
                vsync_nondisp = 3;
                vsync_start = 8;
                vsync_pw = 10;
                m_voffset = 0;
            } else if (width == 800 && height == 480) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
                hsync_nondisp = 26;
                hsync_start = 32;
                hsync_pw = 96;
                hsync_finetune = 0;
                vsync_nondisp = 32;
                vsync_start = 23;
                vsync_pw = 2;
                m_voffset = 0;
            } else {
                return false;
            }

            reg(RA8875_PCSR, pixclk);
            delay(1);


            reg(RA8875_HDWR, (width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
            reg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
            reg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                                    8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
            reg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
            reg(RA8875_HPWR,
                    RA8875_HPWR_LOW +
                        (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

            reg(RA8875_VDHR0, (uint16_t)(height - 1 + m_voffset) & 0xFF);
            reg(RA8875_VDHR1, (uint16_t)(height - 1 + m_voffset) >> 8);
            reg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
            reg(RA8875_VNDR1, vsync_nondisp >> 8);
            reg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
            reg(RA8875_VSTR1, vsync_start >> 8);
            reg(RA8875_VPWR,
                    RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

            /* Set active window X */
            reg(RA8875_HSAW0, 0); // horizontal start point
            reg(RA8875_HSAW1, 0);
            reg(RA8875_HEAW0, (uint16_t)(width - 1) & 0xFF); // horizontal end point
            reg(RA8875_HEAW1, (uint16_t)(width - 1) >> 8);

            /* Set active window Y */
            reg(RA8875_VSAW0, 0 + m_voffset); // vertical start point
            reg(RA8875_VSAW1, 0 + m_voffset);
            reg(RA8875_VEAW0,
                    (uint16_t)(height - 1 + m_voffset) & 0xFF); // vertical end point
            reg(RA8875_VEAW1, (uint16_t)(height - 1 + m_voffset) >> 8);

            /* ToDo: Setup touch panel? */

            /* Clear the entire window */
            reg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
            delay(500);
            
            return true;
        }
        
    public:
        // indicates the type, itself
        using type = ra8875;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,false,true,false,false,false,false>;

        ra8875(SPIClass& spi) : m_initialized(false), m_voffset(0),m_in_batch(false),m_batch_offset(0), m_batch_bounds(0,0,0,0), m_spi(spi),m_spi_settings(init_clock_speed,MSBFIRST,0) {

        }
        
        bool initialize() {
            static const uint8_t RA8875_PWRR = 0x01;
            static const uint8_t RA8875_PWRR_DISPON = 0x80;
            static const uint8_t RA8875_PWRR_NORMAL = 0x00;
            static const uint8_t RA8875_GPIOX = 0xC7;
            static const uint8_t RA8875_PWM_CLK_DIV1024 = 0x0A;
            static const uint8_t RA8875_P1CR = 0x8A;
            static const uint8_t RA8875_P1CR_ENABLE = 0x80;
            static const uint8_t RA8875_P1DCR = 0x8B;

            if(!m_initialized) {

                pinMode(pin_cs, OUTPUT);
                digitalWrite(pin_cs, HIGH);
                pinMode(pin_rst, OUTPUT);

                digitalWrite(pin_rst, LOW);
                delay(100);
                digitalWrite(pin_rst, HIGH);
                delay(100);
                m_spi.begin();
                uint8_t result = reg(0);
                if (result != 0x75) {
                    return false;
                }
                if(!initialize_pll()) {
                    return false;
                }
                
                if(!initialize_display()) {
                    return false;
                }
                // turn on the display
                reg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);

                // GPIOX on
                reg(RA8875_GPIOX, 1);

                // enable backlight
                reg(RA8875_P1CR, RA8875_P1CR_ENABLE | (RA8875_PWM_CLK_DIV1024 & 0xF));
                reg(RA8875_P1DCR, 0xFF);
                m_spi_settings._clock = clock_speed;
                m_initialized = true;
            
            }
            return true;
        }
        // retrieves the dimensions of the screen
        constexpr inline gfx::size16 dimensions() const {
            return gfx::size16(width,height);
        }
        // retrieves the bounds of the screen
        constexpr inline gfx::rect16 bounds() const {
            return gfx::rect16(gfx::point16(0,0),dimensions());
        }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type pixel) {
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_DATAWRITE = 0x00;

            if(location.x>=width || location.y>=height)
                return gfx::gfx_result::success;
            if(!m_initialized) {
                if(!initialize())
                    return gfx::gfx_result::device_error;
            }
            reg(RA8875_CURH0, location.x);
            reg(RA8875_CURH1, location.x >> 8);
            reg(RA8875_CURV0, location.y);
            reg(RA8875_CURV1, location.y >> 8);
            send_command(RA8875_MRWC);
            digitalWrite(pin_cs, LOW);
            auto v = pixel.value();
            m_spi.transfer(RA8875_DATAWRITE);
            m_spi.transfer(v);
            m_spi.transfer(v >> 8);
            digitalWrite(pin_cs, HIGH);
            return gfx::gfx_result::success;
        }
        // fills the specified rectangle
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type pixel) {
            static const uint8_t RA8875_DCR = 0x90;
            static const uint8_t RA8875_DCR_LINESQUTRI_STATUS = 0x80;
            // it's a point
            if(rect.x1==rect.x2&&rect.y1==rect.y2) {
                return point(rect.point1(),pixel);
            }
            if(!rect.intersects(bounds())) return gfx::gfx_result::success;
            gfx::rect16 r = rect.crop(bounds()).normalize();
            if(!m_initialized) {
                if(!initialize())
                    return gfx::gfx_result::device_error;
            }
            // it's a line
            if(r.x1==r.x2 || r.y1==r.y2) {    
                send_command(0x91);
                send_data(r.x1);
                send_command(0x92);
                send_data(r.x1 >> 8);

                send_command(0x93);
                send_data(r.y1);
                send_command(0x94);
                send_data(r.y1 >> 8);

                send_command(0x95);
                send_data(r.x2);
                send_command(0x96);
                send_data((r.x2) >> 8);

                send_command(0x97);
                send_data(r.y2);
                send_command(0x98);
                send_data((r.y2) >> 8);

                send_command(0x63);
                send_data(pixel.channel_unchecked<0>());
                send_command(0x64);
                send_data(pixel.channel_unchecked<1>());
                send_command(0x65);
                send_data(pixel.channel_unchecked<2>());

                send_command(RA8875_DCR);
                send_data(0x80);

                poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
                return gfx::gfx_result::success;
            }
            // set x
            send_command(0x91);
            send_data(r.x1);
            send_command(0x92);
            send_data(r.x1 >> 8);
            // set y
            send_command(0x93);
            send_data(r.y1);
            send_command(0x94);
            send_data(r.y1 >> 8);

            // set width
            send_command(0x95);
            send_data(r.x2);
            send_command(0x96);
            send_data(r.x2 >> 8);

            // set height
            send_command(0x97);
            send_data(r.y2);
            send_command(0x98);
            send_data(r.y2 >> 8);

            // set color 
            send_command(0x63);
            send_data(pixel.channel_unchecked<0>());
            send_command(0x64);
            send_data(pixel.channel_unchecked<1>());
            send_command(0x65);
            send_data(pixel.channel_unchecked<2>());

            send_command(RA8875_DCR);
            send_data(0xB0);

            // wait
            poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);

            return gfx::gfx_result::success;
        }
        gfx::gfx_result clear(const gfx::rect16& rect) {
            return fill(rect,pixel_type());
        }
        gfx::gfx_result begin_batch(const gfx::rect16& rect) {
            static const uint8_t RA8875_DATAWRITE = 0x00;
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_MWCR0 = 0x40;
            static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
            static const uint8_t RA8875_MWCR0_LRTD = 0x00;

            gfx::rect16 r = rect.normalize();
            gfx::gfx_result rr = commit_batch();
            if(gfx::gfx_result::success != rr) {
                return rr;
            }
            reg(RA8875_CURH0, r.x1);
            reg(RA8875_CURH1, r.x1 >> 8);
            reg(RA8875_CURV0, r.y1);
            reg(RA8875_CURV1, r.y1 >> 8);
            reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | RA8875_MWCR0_LRTD);
            send_command(RA8875_MRWC);
            digitalWrite(pin_cs, LOW);
            SPI.transfer(RA8875_DATAWRITE);
            m_batch_bounds = r;
            m_batch_offset = 0;
            m_in_batch = true;
            return gfx::gfx_result::success;
        }
        // writes a pixel to a pending batch
        gfx::gfx_result write_batch(pixel_type color) {
            if(!m_in_batch) return gfx::gfx_result::invalid_state;
            static const uint8_t RA8875_DATAWRITE = 0x00;
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_MWCR0 = 0x40;
            static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
            static const uint8_t RA8875_MWCR0_LRTD = 0x00;
            uint16_t v = color.channel_unchecked<2>() | (color.channel_unchecked<1>()<<5) | (color.channel_unchecked<0>()<<11);
            m_spi.transfer16(v);
            ++m_batch_offset;
            gfx::point16 pt(m_batch_offset%m_batch_bounds.width()+m_batch_bounds.x1, m_batch_offset/m_batch_bounds.width()+m_batch_bounds.y1);
            if(pt.x==m_batch_bounds.x1) {
                digitalWrite(pin_cs, HIGH);
                reg(RA8875_CURH0, m_batch_bounds.x1);
                reg(RA8875_CURH1, m_batch_bounds.x1 >> 8);
                reg(RA8875_CURV0, pt.y);
                reg(RA8875_CURV1, pt.y >> 8);
                reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | RA8875_MWCR0_LRTD);
                send_command(RA8875_MRWC);
                digitalWrite(pin_cs, LOW);
                SPI.transfer(RA8875_DATAWRITE);
            }
            return gfx::gfx_result::success;
        }
        // commits a pending batch
        gfx::gfx_result commit_batch() {
            if(!m_in_batch) {
                return gfx::gfx_result::success;
            }
            digitalWrite(pin_cs, HIGH);
            m_in_batch = false;
            return gfx::gfx_result::success;
        }
    };
}