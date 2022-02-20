#include <Arduino.h>

#include <gfx_core.hpp>
#include <gfx_palette.hpp>
#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>
#include <gfx_draw_helpers.hpp>
#include "common/tft_driver.hpp"

namespace arduino {
enum struct st7735_flags {
    green = 0x00,
    red = 0x01,
    black = 0x02,
    green_18 = green,
    red_18 = red,
    black_18 = black,
    green_144 = 0x01,
    mini_160x80 = 0x04,
    hallowing = 0x05
};
template <int8_t PinDC, int8_t PinRst, int8_t PinBL, typename Bus,
          st7735_flags TabFlags = st7735_flags::green, uint8_t Rotation = 0,unsigned int WriteSpeedPercent = 200,unsigned int ReadSpeedPercent = WriteSpeedPercent>
struct st7735 final {
    constexpr static const int8_t pin_dc = PinDC;
    constexpr static const int8_t pin_rst = PinRst;
    constexpr static const int8_t pin_bl = PinBL;
    constexpr static const uint8_t rotation = Rotation & 3;
    constexpr static const st7735_flags tab_flags = TabFlags;
    constexpr static const float write_speed_multiplier = (WriteSpeedPercent/100.0);
    constexpr static const float read_speed_multiplier = (ReadSpeedPercent/100.0);
    using type = st7735;
    using driver = tft_driver<PinDC, PinRst, PinBL, Bus>;
    using bus = Bus;
    using pixel_type = gfx::rgb_pixel<16>;
    using caps = gfx::gfx_caps<false, (bus::dma_size > 0), true, true, false,
                               bus::readable, bus::readable>;
    st7735()
        : m_initialized(false), m_dma_initialized(false), m_in_batch(false) {}
    ~st7735() {
        if (m_dma_initialized) {
            bus::deinitialize_dma();
        }
        if (m_initialized) {
            driver::deinitialize();
        }
    }

    bool initialize() {
        if (!m_initialized) {
            if (driver::initialize()) {
                static const uint8_t generic_st7735[] PROGMEM =
                    {             // Init commands for 7735 screens
                                  // 7735R init, part 1 (red or green tab)
                     15,          // 15 commands in list:
                     0x01, 0x80,  //  1: Software reset, 0 args, w/delay
                     150,         //     150 ms delay
                     0x11, 0x80,  //  2: Out of sleep mode, 0 args, w/delay
                     255,         //     500 ms delay
                     0xB1, 3,     //  3: Framerate ctrl - normal mode, 3 arg:
                     0x01, 0x2C,
                     0x2D,     //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
                     0xB2, 3,  //  4: Framerate ctrl - idle mode, 3 args:
                     0x01, 0x2C,
                     0x2D,     //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
                     0xB3, 6,  //  5: Framerate - partial mode, 6 args:
                     0x01, 0x2C,
                     0x2D,  //     Dot inversion mode
                     0x01, 0x2C,
                     0x2D,     //     Line inversion mode
                     0xB4, 1,  //  6: Display inversion ctrl, 1 arg:
                     0x07,     //     No inversion
                     0xC0, 3,  //  7: Power control, 3 args, no delay:
                     0xA2,
                     0x02,     //     -4.6V
                     0x84,     //     AUTO mode
                     0xC1, 1,  //  8: Power control, 1 arg, no delay:
                     0xC5,     //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
                     0xC2, 2,  //  9: Power control, 2 args, no delay:
                     0x0A,     //     Opamp current small
                     0x00,     //     Boost frequency
                     0xC3, 2,  // 10: Power control, 2 args, no delay:
                     0x8A,     //     BCLK/2,
                     0x2A,     //     opamp current small & medium low
                     0xC4, 2,  // 11: Power control, 2 args, no delay:
                     0x8A, 0xEE,
                     0xC5, 1,  // 12: Power control, 1 arg, no delay:
                     0x0E, 0x20,
                     0,        // 13: Don't invert display, no args
                     0x36, 1,  // 14: Mem access ctl (directions), 1 arg:
                     0xC8,     //     row/col addr, bottom-top refresh
                     0x3A, 1,  // 15: set color mode, 1 arg, no delay:
                     0x05};    //     16-bit color

                static const uint8_t generic_st7735_2[] PROGMEM = {
                    // 7735R init, part 3 (red or green tab)
                    4,         //  4 commands in list:
                    0xE0, 16,  //  1: Gamma Adjustments (pos. polarity), 16 args
                               //  + delay:
                    0x02, 0x1c, 0x07, 0x12,  //     (Not entirely necessary, but
                                             //     provides
                    0x37, 0x32, 0x29, 0x2d,  //      accurate colors)
                    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01,
                    0x03, 0x10, 0xE1, 16,    //  2: Gamma Adjustments (neg.
                                             //  polarity), 16 args + delay:
                    0x03, 0x1d, 0x07, 0x06,  //     (Not entirely necessary, but
                                             //     provides
                    0x2E, 0x2C, 0x29, 0x2D,  //      accurate colors)
                    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00,
                    0x02, 0x10, 0x13, 0x80,  //  3: Normal display on, no args,
                                             //  w/delay
                    10,                      //     10 ms delay
                    0x29, 0x80,  //  4: Main screen turn on, no args w/delay
                    100};        //     100 ms delay

                if (pin_dc >= 0) {
                    pinMode(pin_dc, OUTPUT);
                }
                if (pin_rst >= 0) {
                    pinMode(pin_rst, OUTPUT);
                }
                if (pin_bl >= 0) {
                    pinMode(pin_bl, OUTPUT);
                    digitalWrite(pin_bl, HIGH);
                }
                bus::set_speed_multiplier(write_speed_multiplier);
                bus::begin_initialization();
                bus::begin_write();
                bus::begin_transaction();

                driver::reset();
                send_init_commands(generic_st7735);
                driver::send_command(0x2A);
                uint8_t c_init_data[] = {0, 0, 0, native_width - 1};
                driver::send_data(c_init_data, 4);
                driver::send_command(
                    0x2B);  //  6: Row addr set, 4 args, no delay:
                uint8_t r_init_data[] = {0, 0, 0, native_height - 1};
                driver::send_data(r_init_data, 4);
                send_init_commands(generic_st7735_2);
                bus::end_transaction();
                bus::end_write();
                bus::end_initialization();
                bus::begin_write();
                bus::begin_transaction();
                apply_rotation();
                bus::end_transaction();
                bus::end_write();

                if (pin_bl > -1) {
                    pinMode(pin_bl, OUTPUT);
                    digitalWrite(pin_bl, HIGH);
                }

                m_initialized = true;
            }
        }
        return m_initialized;
    }

    inline gfx::size16 dimensions() const {
        return rotation & 1 ? gfx::size16(native_width, native_height)
                            : gfx::size16(native_height, native_width);
    }
    inline gfx::rect16 bounds() const { return dimensions().bounds(); }

    inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        return fill({location.x, location.y, location.x, location.y}, color);
    }
    inline gfx::gfx_result point_async(gfx::point16 location,
                                       pixel_type color) {
        return point(location, color);
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
        if (out_color == nullptr) return gfx::gfx_result::invalid_argument;
        if (!m_initialized || m_in_batch) return gfx::gfx_result::invalid_state;
        if (!bounds().intersects(location)) {
            *out_color = pixel_type();
            return gfx::gfx_result::success;
        }
        bus::dma_wait();
        bus::set_speed_multiplier(read_speed_multiplier);
        bus::begin_read();
        bus::cs_low();
        set_window({location.x, location.y, location.x, location.y}, true);
        bus::direction(INPUT);
        bus::read_raw8();  // throw away
        out_color->native_value = ((bus::read_raw8() & 0xF8) << 8) |
                                  ((bus::read_raw8() & 0xFC) << 3) |
                                  (bus::read_raw8() >> 3);
        bus::cs_high();
        bus::end_read();
        bus::set_speed_multiplier(write_speed_multiplier);
        bus::direction(OUTPUT);
        return gfx::gfx_result::success;
    }
    gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
        if (!initialize())
            return gfx::gfx_result::device_error;
        else
            bus::dma_wait();
        gfx::gfx_result rr = commit_batch();

        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        if (!bounds.intersects(this->bounds())) return gfx::gfx_result::success;
        const gfx::rect16 r = bounds.normalize().crop(this->bounds());
        bus::begin_write();
        bus::begin_transaction();
        set_window(r);
        bus::write_raw16_repeat(color.native_value,
                                (r.x2 - r.x1 + 1) * (r.y2 - r.y1 + 1));
        bus::end_transaction();
        bus::end_write();
        return gfx::gfx_result::success;
    }
    inline gfx::gfx_result fill_async(const gfx::rect16& bounds,
                                      pixel_type color) {
        return fill(bounds, color);
    }
    inline gfx::gfx_result clear(const gfx::rect16& bounds) {
        return fill(bounds, pixel_type());
    }
    inline gfx::gfx_result clear_async(const gfx::rect16& bounds) {
        return clear(bounds);
    }
    template <typename Source>
    inline gfx::gfx_result copy_from(const gfx::rect16& src_rect,
                                     const Source& src, gfx::point16 location) {
        if (!initialize()) return gfx::gfx_result::device_error;
        gfx::gfx_result rr = commit_batch();
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        return copy_from_impl(src_rect, src, location, false);
    }
    template <typename Source>
    inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect,
                                           const Source& src,
                                           gfx::point16 location) {
        if (!initialize()) return gfx::gfx_result::device_error;
        gfx::gfx_result rr = commit_batch();
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        if (!m_dma_initialized) {
            if (!bus::initialize_dma()) return gfx::gfx_result::device_error;
            m_dma_initialized = true;
        }
        return copy_from_impl(src_rect, src, location, true);
    }
    gfx::gfx_result commit_batch() {
        if (m_in_batch) {
            bus::end_transaction();
            bus::end_write();
            m_in_batch = false;
        }
        return gfx::gfx_result::success;
    }
    template <typename Destination>
    inline gfx::gfx_result copy_to(const gfx::rect16& src_rect,
                                   Destination& dst,
                                   gfx::point16 location) const {
        if (!src_rect.intersects(bounds())) return gfx::gfx_result::success;
        gfx::rect16 srcr = src_rect.crop(bounds());
        gfx::rect16 dstr =
            gfx::rect16(location, srcr.dimensions()).crop(dst.bounds());
        srcr = gfx::rect16(srcr.location(), dstr.dimensions());
        return copy_to_helper<Destination,
                              !(pixel_type::template has_channel_names<
                                  gfx::channel_name::A>::value)>::copy_to(*this,
                                                                          srcr,
                                                                          dst,
                                                                          dstr);
    }
    template <typename Destination>
    inline gfx::gfx_result copy_to_async(const gfx::rect16& src_rect,
                                         Destination& dst,
                                         gfx::point16 location) const {
        return copy_to(src_rect, dst, location);
    }
    inline gfx::gfx_result commit_batch_async() { return commit_batch(); }
    gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
        if (!initialize()) return gfx::gfx_result::device_error;
        gfx::gfx_result rr = commit_batch();
        if (rr != gfx::gfx_result::success) {
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
   template <typename Destination, bool AllowBlt = true>
    struct copy_to_helper {
        inline static gfx::gfx_result copy_to(const type& src,
                                              const gfx::rect16& srcr,
                                              Destination& dst,
                                              const gfx::rect16& dstr) {
            if (gfx::helpers::template is_same<typename Destination::pixel_type,
                                               pixel_type>::value &&
                dstr.top_left() == gfx::point16(0, 0)) {
                if (AllowBlt && dstr.width() == srcr.width() &&
                    dstr.height() == srcr.height()) {
                    if (dstr.top_left() == gfx::point16(0, 0)) {
                        return copy_to_fast_helper<Destination>::do_copy(srcr,
                                                                         dst);
                    }
                }
            }

            size_t dy = 0, dye = dstr.height();
            size_t dx, dxe = dstr.width();
            gfx::gfx_result r;
            gfx::helpers::suspender<Destination, Destination::caps::suspend,
                                    false>
                sustok(dst);
            r = gfx::helpers::batcher<Destination, Destination::caps::batch,
                                      false>::begin_batch(dst, dstr, false);
            if (gfx::gfx_result::success != r) {
                return r;
            }
            int sox = srcr.left(), soy = srcr.top();
            int dox = dstr.left(), doy = dstr.top();
            while (dy < dye) {
                dx = 0;

                while (dx < dxe) {
                    pixel_type spx;
                    r = src.point(gfx::point16(sox + dx, soy + dy), &spx);
                    if (gfx::gfx_result::success != r) return r;
                    typename Destination::pixel_type dpx;
                    if (pixel_type::template has_channel_names<
                            gfx::channel_name::A>::value) {
                        r = gfx::helpers::blend_helper<
                            type, Destination,
                            Destination::caps::read>::do_blend(src, spx, dst,
                                                               gfx::point16(
                                                                   dox + dx,
                                                                   doy + dy),
                                                               &dpx);
                        if (gfx::gfx_result::success != r) {
                            return r;
                        }
                    } else {
                        r = convert_palette(dst, src, spx, &dpx, nullptr);
                        if (gfx::gfx_result::success != r) {
                            return r;
                        }
                    }
                    r = gfx::helpers::batcher<
                        Destination, Destination::caps::batch,
                        false>::write_batch(dst,
                                            gfx::point16(dox + dx, doy + dy),
                                            dpx, false);
                    if (gfx::gfx_result::success != r) return r;
                    ++dx;
                }
                ++dy;
            }
            return gfx::helpers::batcher<Destination, Destination::caps::batch,
                                         false>::commit_batch(dst, false);
        }
    };
    template <typename Destination>
    struct copy_to_fast_helper {
        static gfx::gfx_result do_copy(const gfx::rect16& src_rect,
                                       Destination& dst) {
            gfx::rect16 r = src_rect.normalize();
            bool entire = false;
            gfx::size16 bssz = r.dimensions();
            size_t bsz = bssz.width * bssz.height * 3;
            uint8_t* buf = (uint8_t*)malloc(bsz);
            if (buf != nullptr) {
                entire = true;
            } else {
                bsz = bssz.width * 3;
                buf = (uint8_t*)malloc(bsz);
            }
            if (buf != nullptr) {
                free(buf);
                buf = nullptr;
            }
            using batch =
                gfx::helpers::batcher<Destination, Destination::caps::batch,
                                      Destination::caps::async>;
            if (buf == nullptr) {
                gfx::helpers::suspender<Destination, Destination::caps::suspend,
                                        Destination::caps::async>
                    stok(dst, false);
                gfx::gfx_result rr = batch::begin_batch(
                    dst,
                    {0, 0, uint16_t(r.width() - 1), uint16_t(r.height() - 1)},
                    false);
                if (rr != gfx::gfx_result::success) {
                    return rr;
                }
                pixel_type px;
                for (int y = r.y1; y <= r.y2; ++y) {
                    for (int x = r.x1; x <= r.x2; ++x) {
                        uint8_t buf3[3];
                        buf = buf3;
                        fetch_buffer({uint16_t(x), uint16_t(y), uint16_t(x),
                                      uint16_t(y)},
                                     buf, 3);
                        px.native_value = (((*buf) & 0xF8) << 8);
                        ++buf;
                        px.native_value |= (((*buf) & 0xFC) << 3);
                        ++buf;
                        px.native_value |= (*buf >> 3);
                        batch::write_batch(
                            dst, {uint16_t(x - r.x1), uint16_t(y - r.y1)}, px,
                            false);
                    }
                }
                rr = batch::commit_batch(dst, false);
                if (rr != gfx::gfx_result::success) {
                    return rr;
                }
                buf = nullptr;
            } else {
                if (entire) {
                    fetch_buffer(r, buf, bsz);
                    gfx::helpers::suspender<Destination,
                                            Destination::caps::suspend,
                                            Destination::caps::async>
                        stok(dst, false);
                    gfx::gfx_result rr =
                        batch::begin_batch(dst,
                                           {0, 0, uint16_t(r.width() - 1),
                                            uint16_t(r.height() - 1)},
                                           false);
                    if (rr != gfx::gfx_result::success) {
                        free(buf);
                        return rr;
                    }
                    uint8_t* bbuf = buf;
                    while (bsz) {
                        pixel_type px;
                        uint16_t x, y;
                        x = 0;
                        y = 0;
                        px.native_value = (((*bbuf) & 0xF8) << 8);
                        ++bbuf;
                        px.native_value |= (((*bbuf) & 0xFC) << 3);
                        ++bbuf;
                        px.native_value |= (*bbuf >> 3);
                        ++bbuf;
                        ++x;
                        if (x > r.x2 - r.x1) {
                            x = 0;
                            ++y;
                        }
                        batch::write_batch(dst, {x, y}, px, false);
                        bsz -= 3;
                    }
                    rr = batch::commit_batch(dst, false);
                    if (rr != gfx::gfx_result::success) {
                        free(buf);
                        return rr;
                    }
                } else {
                    gfx::helpers::suspender<Destination,
                                            Destination::caps::suspend,
                                            Destination::caps::async>
                        stok(dst, false);
                    for (int y = r.y1; y <= r.y2; ++y) {
                        fetch_buffer(r, buf, bsz);
                        gfx::gfx_result rr =
                            batch::begin_batch(dst,
                                               {0, 0, uint16_t(r.width() - 1),
                                                uint16_t(r.height() - 1)},
                                               false);
                        if (rr != gfx::gfx_result::success) {
                            free(buf);
                            return rr;
                        }
                        size_t bbsz = bsz;
                        uint8_t* bbuf = buf;
                        while (bbsz) {
                            pixel_type px;
                            uint16_t x = 0;
                            px.native_value = (((*bbuf) & 0xF8) << 8);
                            ++bbuf;
                            px.native_value |= (((*bbuf) & 0xFC) << 3);
                            ++bbuf;
                            px.native_value |= (*bbuf >> 3);
                            ++bbuf;
                            ++x;
                            batch::write_batch(dst, {x, uint16_t(y - r.y1)}, px,
                                               false);
                            bbsz -= 3;
                        }
                        rr = batch::commit_batch(dst, false);
                        if (rr != gfx::gfx_result::success) {
                            free(buf);
                            return rr;
                        }
                    }
                }
            }
            if (buf != nullptr) {
                free(buf);
            }
            return gfx::gfx_result::success;
        }

        static void fetch_buffer(const gfx::rect16& r, uint8_t* buf,
                                 size_t len) {
            bus::dma_wait();
            bus::set_speed_multiplier(read_speed_multiplier);
            bus::begin_read();
            bus::cs_low();
            set_window(r, true);
            bus::direction(INPUT);
            bus::read_raw8();  // throw away
            bus::read_raw(buf, len);
            bus::cs_high();
            bus::end_read();
            bus::set_speed_multiplier(write_speed_multiplier);
            bus::direction(OUTPUT);
        }
    };
    constexpr static inline uint16_t compute_native_width() {
        uint16_t result = 128;
        switch (rotation) {
            case 0:
            case 2:
                if (tab_flags == st7735_flags::mini_160x80) {
                    result = 80;
                }
                break;
            case 1:
            case 3:
                if (tab_flags == st7735_flags::green_144) {
                    // 128
                } else {
                    result = 160;
                }
                break;
        }
        return result;
    }
    constexpr static inline uint16_t compute_native_height() {
        uint16_t result = 160;
        switch (rotation) {
            case 0:
            case 2:
                if (tab_flags == st7735_flags::green_144) {
                    result = 128;
                }
                break;
            case 1:
            case 3:
                if (tab_flags == st7735_flags::mini_160x80) {
                    result = 80;
                } else {
                    result = 128;
                }
                break;
        }
        return result;
    }
    constexpr static const uint16_t native_width = compute_native_width();
    constexpr static const uint16_t native_height = compute_native_height();
    constexpr static inline uint16_t compute_row_start() {
        uint16_t result = 0;
        if (tab_flags == st7735_flags::green) {
            result = 1;
        } else if (tab_flags == st7735_flags::green_144 ||
                   tab_flags == st7735_flags::hallowing) {
            result = rotation < 2 ? 3 : 1;
        } else if (tab_flags == st7735_flags::mini_160x80) {
            result = 0;
        }
        return result;
    }
    constexpr static inline uint16_t compute_column_start() {
        uint16_t result = 0;
        if (tab_flags == st7735_flags::green) {
            result = 2;
        } else if (tab_flags == st7735_flags::green_144 ||
                   tab_flags == st7735_flags::hallowing) {
            result = 2;
        } else if (tab_flags == st7735_flags::mini_160x80) {
            result = 24;
        }
        return result;
    }
    constexpr static const uint16_t row_start = compute_row_start();
    constexpr static const uint16_t column_start = compute_column_start();
    constexpr static const uint16_t y_start =
        rotation & 1 ? column_start : row_start;
    constexpr static const uint16_t x_start =
        rotation & 1 ? row_start : column_start;

    bool m_initialized;
    bool m_dma_initialized;
    bool m_in_batch;
    void send_init_commands(const uint8_t* addr) {
        uint8_t numCommands, cmd, numArgs;
        uint16_t ms;
        numCommands = pgm_read_byte(addr++);  // Number of commands to follow
        while (numCommands--) {               // For each command...
            cmd = pgm_read_byte(addr++);      // Read command
            numArgs = pgm_read_byte(addr++);  // Number of args to follow
            ms = numArgs & 0x80;  // If hibit set, delay follows args
            numArgs &= ~0x80;     // Mask out delay bit
            driver::send_command(cmd);
            driver::send_data_pgm(addr, numArgs);
            addr += numArgs;
            if (ms) {
                ms =
                    pgm_read_byte(addr++);  // Read post-command delay time (ms)
                if (ms == 255) ms = 500;    // If 255, delay for 500 ms
                delay(ms);
            }
        }
    }
    static void set_window(const gfx::rect16& bounds, bool read = false) {
       driver::dc_command();
        bus::write_raw8(0x2A);
        driver::dc_data();
        bus::write_raw16(column_start+bounds.x1);
        bus::write_raw16(column_start+bounds.x2);
        driver::dc_command();
        bus::write_raw8(0x2B);
        driver::dc_data();
        bus::write_raw16(row_start+bounds.y1);
        bus::write_raw16(row_start+bounds.y2);
        driver::dc_command();
        bus::write_raw8(read ? 0x2E : 0x2C);
        driver::dc_data();
    }
    template <typename Source, bool Blt>
    struct copy_from_helper {
        static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,
                                       const Source& src, gfx::rect16 srcr,
                                       bool async) {
            uint16_t w = dstr.dimensions().width;
            uint16_t h = dstr.dimensions().height;
            gfx::gfx_result rr;

            rr = this_->begin_batch(dstr);

            if (gfx::gfx_result::success != rr) {
                return rr;
            }
            for (uint16_t y = 0; y < h; ++y) {
                for (uint16_t x = 0; x < w; ++x) {
                    typename Source::pixel_type pp;
                    rr = src.point(gfx::point16(x + srcr.x1, y + srcr.y1), &pp);
                    if (rr != gfx::gfx_result::success) return rr;
                    pixel_type p;
                    rr = gfx::convert_palette_to(src, pp, &p, nullptr);
                    if (gfx::gfx_result::success != rr) {
                        return rr;
                    }

                    rr = this_->write_batch(p);

                    if (gfx::gfx_result::success != rr) {
                        return rr;
                    }
                }
            }

            rr = this_->batch_commit();

            return rr;
        }
    };

    template <typename Source>
    struct copy_from_helper<Source, true> {
        static gfx::gfx_result do_draw(type* this_, const gfx::rect16& dstr,
                                       const Source& src, gfx::rect16 srcr,
                                       bool async) {
            if (async) {
                bus::dma_wait();
            }
            // direct blt
            if (src.bounds().width() == srcr.width() && srcr.x1 == 0) {
                bus::begin_write();
                set_window(dstr);
                if (async) {
                    bus::write_raw_dma(
                        src.begin() + (srcr.y1 * src.dimensions().width * 2),
                        (srcr.y2 - srcr.y1 + 1) * src.dimensions().width * 2);
                } else {
                    bus::write_raw(
                        src.begin() + (srcr.y1 * src.dimensions().width * 2),
                        (srcr.y2 - srcr.y1 + 1) * src.dimensions().width * 2);
                }

                bus::end_write();
                return gfx::gfx_result::success;
            }
            // line by line blt
            uint16_t yy = 0;
            uint16_t hh = srcr.height();
            uint16_t ww = src.dimensions().width;
            uint16_t pitch = (srcr.x2 - srcr.x1 + 1) * 2;
            bus::begin_write();
            bus::begin_transaction();
            while (yy < hh - !!async) {
                gfx::rect16 dr = {dstr.x1, uint16_t(dstr.y1 + yy), dstr.x2,
                                  uint16_t(dstr.y1 + yy)};
                set_window(dr);
                bus::write_raw(
                    src.begin() + 2 * (ww * (srcr.y1 + yy) + srcr.x1), pitch);
                ++yy;
            }
            if (async) {
                gfx::rect16 dr = {dstr.x1, uint16_t(dstr.y1 + yy), dstr.x2,
                                  uint16_t(dstr.y1 + yy)};
                set_window(dr);
                bus::write_raw_dma(
                    src.begin() + 2 * (ww * (srcr.y1 + yy) + srcr.x1), pitch);
            }
            bus::end_transaction();
            bus::end_write();
            return gfx::gfx_result::success;
        }
    };
    template <typename Source>
    gfx::gfx_result copy_from_impl(const gfx::rect16& src_rect,
                                   const Source& src, gfx::point16 location,
                                   bool async) {
        gfx::rect16 srcr = src_rect.normalize().crop(src.bounds());
        gfx::rect16 dstr(location, src_rect.dimensions());
        dstr = dstr.crop(bounds());
        if (srcr.width() > dstr.width()) {
            srcr.x2 = srcr.x1 + dstr.width() - 1;
        }
        if (srcr.height() > dstr.height()) {
            srcr.y2 = srcr.y1 + dstr.height() - 1;
        }
        return copy_from_helper < Source,
               gfx::helpers::is_same<pixel_type,
                                     typename Source::pixel_type>::value &&
                   Source::caps::blt > ::do_draw(this, dstr, src, srcr, async);
    }
    static void apply_rotation() {
        bus::begin_write();
        uint8_t madctl;
        switch (rotation) {
            case 0:
                // portrait
                if ((tab_flags == st7735_flags::black) ||
                    (tab_flags == st7735_flags::mini_160x80)) {
                    madctl = 0x40 | 0x80 | 0x00;
                } else {
                    madctl = 0x40 | 0x80 | 0x08;
                }
                break;
            case 1:
                // landscape
                if ((tab_flags == st7735_flags::black) ||
                    (tab_flags == st7735_flags::mini_160x80)) {
                    madctl = 0x80 | 0x20 | 0x00;
                } else {
                    madctl = 0x80 | 0x20 | 0x08;
                }
                break;
            case 2:
                // portrait
                if ((tab_flags == st7735_flags::black) ||
                    (tab_flags == st7735_flags::mini_160x80)) {
                    madctl = 0x00;
                } else {
                    madctl = 0x08;
                }
                break;
            case 3:
                // landscape
                if ((tab_flags == st7735_flags::black) ||
                    (tab_flags == st7735_flags::mini_160x80)) {
                    madctl = 0x40 | 0x20 | 0x00;
                } else {
                    madctl = 0x40 | 0x20 | 0x08;
                }
                break;
        }
        driver::send_command(0x36);
        driver::send_data8(madctl);
        delayMicroseconds(10);

        bus::end_write();
    }
};
}  // namespace arduino