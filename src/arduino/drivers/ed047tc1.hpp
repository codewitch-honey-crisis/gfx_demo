#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_attr.h>
#include <esp_idf_version.h>
#if ESP_IDF_VERSION_MAJOR >= 4
#include <hal/rmt_ll.h>
#endif
#include <Arduino.h>
#include <esp_timer.h>
#include <string.h>
#include <xtensa/core-macros.h>

#include <gfx_core.hpp>
#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>

#include "ed047tc1_i2s.hpp"

namespace arduino {
    namespace ed047tc1_helpers {
        typedef struct {
            bool ep_latch_enable : 1;
            bool power_disable : 1;
            bool pos_power_enable : 1;
            bool neg_power_enable : 1;
            bool ep_stv : 1;
            bool ep_scan_direction : 1;
            bool ep_mode : 1;
            bool ep_output_enable : 1;

        } epd_config_register_t;
        struct arrays {
            static const /*DRAM_ATTR*/ uint32_t *lut_1bpp;
            static const int *contrast_cycles_4;  // = {30, 30, 20, 20, 30,  30,  30, 40,
                                            // 40, 50, 50, 50, 100, 200, 300};
            static const int *contrast_cycles_4_white;  // = {10, 10, 8, 8, 8,  8,  8, 10,
                                                    // 10, 15, 15, 20, 20, 100, 300};
        };
        /// The image drawing mode.
        enum struct DrawMode {
            /// Draw black / grayscale image on a white display.
            BLACK_ON_WHITE = 1 << 0,
            /// "Draw with white ink" on a white display.
            WHITE_ON_WHITE = 1 << 1,
            /// Draw with white ink on a black display.
            WHITE_ON_BLACK = 1 << 2,
        };

    }  // namespace ed047tc1_helpers
    template <uint16_t Width, uint16_t Height, int8_t PinCfgData, int8_t PinCfgClk,
            int8_t PinCfgStr, int8_t PinCkv, int8_t PinSth, int8_t PinCkh,
            int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
            int8_t PinD5, int8_t PinD6, int8_t PinD7>
    class ed047tc1 final {
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const int8_t pin_cfg_data = PinCfgData;
        constexpr static const int8_t pin_cfg_clk = PinCfgClk;
        constexpr static const int8_t pin_cfg_str = PinCfgStr;
        constexpr static const int8_t pin_ckv = PinCkv;
        constexpr static const int8_t pin_sth = PinSth;
        constexpr static const int8_t pin_ckh = PinCkh;
        constexpr static const int8_t pin_d0 = PinD0;
        constexpr static const int8_t pin_d1 = PinD1;
        constexpr static const int8_t pin_d2 = PinD2;
        constexpr static const int8_t pin_d3 = PinD3;
        constexpr static const int8_t pin_d4 = PinD4;
        constexpr static const int8_t pin_d5 = PinD5;
        constexpr static const int8_t pin_d6 = PinD6;
        constexpr static const int8_t pin_d7 = PinD7;
        using pixel_type = gfx::gsc_pixel<4>;
    #ifndef ESP32
        static_assert(false, "This driver only works with an ESP32");
    #endif
        using bus = ed047tc1_helpers::ed047tc1_i2s<width + 32, pin_cfg_clk, pin_sth,
                                                pin_d0, pin_d1, pin_d2, pin_d3,
                                                pin_d4, pin_d5, pin_d6, pin_d7>;

    private:
        bool m_initialized;

    public:
        ed047tc1() : m_initialized(false) {}
        ~ed047tc1() {
            if (m_initialized) {
                bus::deinitialize();
            }
        }
        inline bool initialized() const { return m_initialized; }
        bool initialize() {
            if (!m_initialized) {
                bus::initialize();
                m_initialized = true;
            }
            return m_initialized;
        }

    private:
        static intr_handle_t gRMT_intr_handle;  // = NULL;

        // the RMT channel configuration object
        static rmt_config_t row_rmt_config;

        // keep track of wether the current pulse is ongoing
        static volatile bool rmt_tx_done;  // = true;
        static ed047tc1_helpers::epd_config_register_t config_reg;
        /**
         * Remote peripheral interrupt. Used to signal when transmission is done.
         */
        static void IRAM_ATTR rmt_interrupt_handler(void *arg) {
            rmt_tx_done = true;
            RMT.int_clr.val = RMT.int_st.val;
        }

        static void rmt_pulse_init(gpio_num_t pin) {
            row_rmt_config.rmt_mode = RMT_MODE_TX;
            // currently hardcoded: use channel 0
            row_rmt_config.channel = RMT_CHANNEL_1;

            row_rmt_config.gpio_num = pin;
            row_rmt_config.mem_block_num = 2;

            // Divide 80MHz APB Clock by 8 -> .1us resolution delay
            row_rmt_config.clk_div = 8;

            row_rmt_config.tx_config.loop_en = false;
            row_rmt_config.tx_config.carrier_en = false;
            row_rmt_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
            row_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
            row_rmt_config.tx_config.idle_output_en = true;

    #if ESP_IDF_VERSION_MAJOR >= 4
            rmt_isr_register(rmt_interrupt_handler, 0, ESP_INTR_FLAG_LEVEL3,
                            &gRMT_intr_handle);
    #else
            esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_LEVEL3,
                        rmt_interrupt_handler, 0, &gRMT_intr_handle);
    #endif

            rmt_config(&row_rmt_config);
    #if ESP_IDF_VERSION_MAJOR >= 4
            rmt_ll_enable_tx_end_interrupt(&RMT, row_rmt_config.channel, true);
    #else
            rmt_set_tx_intr_en(row_rmt_config.channel, true);
    #endif
        }

        static void IRAM_ATTR pulse_ckv_ticks(uint16_t high_time_ticks,
                                            uint16_t low_time_ticks, bool wait) {
            while (!rmt_tx_done) {
            };
            volatile rmt_item32_t *rmt_mem_ptr =
                &(RMTMEM.chan[row_rmt_config.channel].data32[0]);
            if (high_time_ticks > 0) {
                rmt_mem_ptr->level0 = 1;
                rmt_mem_ptr->duration0 = high_time_ticks;
                rmt_mem_ptr->level1 = 0;
                rmt_mem_ptr->duration1 = low_time_ticks;
            } else {
                rmt_mem_ptr->level0 = 1;
                rmt_mem_ptr->duration0 = low_time_ticks;
                rmt_mem_ptr->level1 = 0;
                rmt_mem_ptr->duration1 = 0;
            }
            RMTMEM.chan[row_rmt_config.channel].data32[1].val = 0;
            rmt_tx_done = false;
            RMT.conf_ch[row_rmt_config.channel].conf1.mem_rd_rst = 1;
            RMT.conf_ch[row_rmt_config.channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
            RMT.conf_ch[row_rmt_config.channel].conf1.tx_start = 1;
            while (wait && !rmt_tx_done) {
            };
        }

        static void IRAM_ATTR pulse_ckv_us(uint16_t high_time_us,
                                        uint16_t low_time_us, bool wait) {
            pulse_ckv_ticks(10 * high_time_us, 10 * low_time_us, wait);
        }

        static bool IRAM_ATTR rmt_busy() { return !rmt_tx_done; }

        /*
        * Write bits directly using the registers.
        * Won't work for some pins (>= 32).
        */
        inline static void fast_gpio_set_hi(int8_t gpio_num) {
            GPIO.out_w1ts = (1 << ((gpio_num_t)gpio_num));
        }

        inline static void fast_gpio_set_lo(int8_t gpio_num) {
            GPIO.out_w1tc = (1 << ((gpio_num_t)gpio_num));
        }

        void IRAM_ATTR busy_delay(uint32_t cycles) {
            volatile unsigned long counts = XTHAL_GET_CCOUNT() + cycles;
            while (XTHAL_GET_CCOUNT() < counts) {
            };
        }

        inline static void IRAM_ATTR push_cfg_bit(bool bit) {
            fast_gpio_set_lo(pin_cfg_clk);
            if (bit) {
                fast_gpio_set_hi(pin_cfg_data);
            } else {
                fast_gpio_set_lo(pin_cfg_data);
            }
            fast_gpio_set_hi(pin_cfg_clk);
        }

        static void IRAM_ATTR
        push_cfg(ed047tc1_helpers::epd_config_register_t *cfg) {
            fast_gpio_set_lo(pin_cfg_str);

            // push config bits in reverse order
            push_cfg_bit(cfg->ep_output_enable);
            push_cfg_bit(cfg->ep_mode);
            push_cfg_bit(cfg->ep_scan_direction);
            push_cfg_bit(cfg->ep_stv);

            push_cfg_bit(cfg->neg_power_enable);
            push_cfg_bit(cfg->pos_power_enable);
            push_cfg_bit(cfg->power_disable);
            push_cfg_bit(cfg->ep_latch_enable);

            fast_gpio_set_hi(pin_cfg_str);
        }

        void epd_base_init(uint32_t epd_row_width) {
            config_reg.ep_latch_enable = false;
            config_reg.power_disable = true;
            config_reg.pos_power_enable = false;
            config_reg.neg_power_enable = false;
            config_reg.ep_stv = true;
            config_reg.ep_scan_direction = true;
            config_reg.ep_mode = false;
            config_reg.ep_output_enable = false;

            /* Power Control Output/Off */
            gpio_set_direction((gpio_num_t)pin_cfg_data, GPIO_MODE_OUTPUT);
            gpio_set_direction((gpio_num_t)pin_cfg_clk, GPIO_MODE_OUTPUT);
            gpio_set_direction((gpio_num_t)pin_cfg_str, GPIO_MODE_OUTPUT);
            fast_gpio_set_lo(pin_cfg_str);

            push_cfg(&config_reg);

            if (!bus::initialize()) return;

            rmt_pulse_init(pin_ckv);
        }

        void epd_poweron() {
            // POWERON
            config_reg.ep_scan_direction = true;
            config_reg.power_disable = false;
            push_cfg(&config_reg);
            busy_delay(100 * 240);
            config_reg.neg_power_enable = true;
            push_cfg(&config_reg);
            busy_delay(500 * 240);
            config_reg.pos_power_enable = true;
            push_cfg(&config_reg);
            busy_delay(100 * 240);
            config_reg.ep_stv = true;
            push_cfg(&config_reg);
            fast_gpio_set_hi(pin_sth);
            // END POWERON
        }

        void epd_poweroff() {
            // POWEROFF
            config_reg.pos_power_enable = false;
            push_cfg(&config_reg);
            busy_delay(10 * 240);
            config_reg.neg_power_enable = false;
            push_cfg(&config_reg);
            busy_delay(100 * 240);
            config_reg.power_disable = true;
            push_cfg(&config_reg);

            config_reg.ep_stv = false;
            push_cfg(&config_reg);

            //   config_reg.ep_scan_direction = false;
            //   push_cfg(&config_reg);

            // END POWEROFF
        }

        void epd_poweroff_all() {
            memset(&config_reg, 0, sizeof(config_reg));
            push_cfg(&config_reg);
        }

        void epd_start_frame() {
            while (bus::is_busy() || rmt_busy()) {
            };
            config_reg.ep_mode = true;
            push_cfg(&config_reg);

            pulse_ckv_us(1, 1, true);

            // This is very timing-sensitive!
            config_reg.ep_stv = false;
            push_cfg(&config_reg);
            busy_delay(240);
            pulse_ckv_us(10, 10, false);
            config_reg.ep_stv = true;
            push_cfg(&config_reg);
            pulse_ckv_us(0, 10, true);

            config_reg.ep_output_enable = true;
            push_cfg(&config_reg);

            pulse_ckv_us(1, 1, true);
        }

        static inline void latch_row() {
            config_reg.ep_latch_enable = true;
            push_cfg(&config_reg);

            config_reg.ep_latch_enable = false;
            push_cfg(&config_reg);
        }

        void IRAM_ATTR epd_skip() {
    #if defined(CONFIG_EPD_DISPLAY_TYPE_ED097TC2)
            pulse_ckv_ticks(2, 2, false);
    #else
            // According to the spec, the OC4 maximum CKV frequency is 200kHz.
            pulse_ckv_ticks(45, 5, false);
    #endif
        }

        void IRAM_ATTR epd_output_row(uint32_t output_time_dus) {
            while (bus::is_busy() || rmt_busy()) {
            };
            latch_row();

            pulse_ckv_ticks(output_time_dus, 50, false);

            bus::start_line_output();
            bus::switch_buffer();
        }

        void epd_end_frame() {
            config_reg.ep_output_enable = false;
            push_cfg(&config_reg);
            config_reg.ep_mode = false;
            push_cfg(&config_reg);
            pulse_ckv_us(1, 1, true);
            pulse_ckv_us(1, 1, true);
        }

        void IRAM_ATTR epd_switch_buffer() { bus::switch_buffer(); }
        uint8_t IRAM_ATTR *epd_get_current_buffer() {
            return (uint8_t *)bus::current_buffer();
        };

        // number of bytes needed for one line of EPD pixel data.
        constexpr static const size_t EPD_LINE_BYTES = width / 4;
        constexpr static uint8_t CLEAR_BYTE = 0B10101010;
        constexpr static uint8_t DARK_BYTE = 0B01010101;

        uint32_t skipping;
        // Heap space to use for the EPD output lookup table, which
        // is calculated for each cycle.
        uint8_t *conversion_lut;
        QueueHandle_t output_queue;

        // output a row to the display.
        void write_row(uint32_t output_time_dus) {
            // avoid too light output after skipping on some displays
            if (skipping) {
                // vTaskDelay(20);
            }
            skipping = 0;
            epd_output_row(output_time_dus);
        }

        void epd_init() {
            skipping = 0;
            epd_base_init(width);

            conversion_lut = (uint8_t *)heap_caps_malloc(1 << 16, MALLOC_CAP_8BIT);
            assert(conversion_lut != NULL);
            output_queue = xQueueCreate(64, width / 2);
        }

        // skip a display row
        void skip_row(uint8_t pipeline_finish_time) {
            // output previously loaded row, fill buffer with no-ops.
            if (skipping == 0) {
                epd_switch_buffer();
                memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
                epd_switch_buffer();
                memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
                epd_output_row(pipeline_finish_time);
                // avoid tainting of following rows by
                // allowing residual charge to dissipate
                // vTaskDelay(10);
                /*
                unsigned counts = XTHAL_GET_CCOUNT() + 50 * 240;
                while (XTHAL_GET_CCOUNT() < counts) {
                };
                */
            } else if (skipping < 2) {
                epd_output_row(10);
            } else {
                // epd_output_row(5);
                epd_skip();
            }
            ++skipping;
        }
        void epd_push_pixels(gfx::rect16 area, short time, int color) {
            uint8_t row[EPD_LINE_BYTES] = {0};
            int w = area.width();
            int h = area.height();
            for (uint32_t i = 0; i < w; i++) {
                uint32_t position = i + area.x1 % 4;
                uint8_t mask = (color ? CLEAR_BYTE : DARK_BYTE) &
                            (0b00000011 << (2 * (position % 4)));
                row[area.x1 / 4 + position / 4] |= mask;
            }
            reorder_line_buffer((uint32_t *)row);

            epd_start_frame();

            for (int i = 0; i < height; i++) {
                // before are of interest: skip
                if (i < area.y1) {
                    skip_row(time);
                    // start area of interest: set row data
                } else if (i == area.y1) {
                    epd_switch_buffer();
                    memcpy(epd_get_current_buffer(), row, EPD_LINE_BYTES);
                    epd_switch_buffer();
                    memcpy(epd_get_current_buffer(), row, EPD_LINE_BYTES);

                    write_row(time * 10);
                    // load nop row if done with area
                } else if (i >= area.y1 + h) {
                    skip_row(time);
                    // output the same as before
                } else {
                    write_row(time * 10);
                }
            }
            // Since we "pipeline" row output, we still have to latch out the last
            // row.
            write_row(time * 10);

            epd_end_frame();
        }

        void epd_clear_area(gfx::rect16 area) {
            epd_clear_area_cycles(area, 4, 50);
        }

        void epd_clear_area_cycles(gfx::rect16 area, int cycles, int cycle_time) {
            const short white_time = cycle_time;
            const short dark_time = cycle_time;

            for (int c = 0; c < cycles; c++) {
                for (int i = 0; i < 4; i++) {
                    epd_push_pixels(area, dark_time, 0);
                }
                for (int i = 0; i < 4; i++) {
                    epd_push_pixels(area, white_time, 1);
                }
            }
        }

        gfx::rect16 epd_full_screen() {
            return {.x1 = 0, .y1 = 0, .x2 = width - 1, .y2 = height - 1};
        }

        void epd_clear() { epd_clear_area(epd_full_screen()); }

        /*
        * Reorder the output buffer to account for I2S FIFO order.
        */
        void reorder_line_buffer(uint32_t *line_data) {
            for (uint32_t i = 0; i < EPD_LINE_BYTES / 4; i++) {
                uint32_t val = *line_data;
                *(line_data++) = val >> 16 | ((val & 0x0000FFFF) << 16);
            }
        }

        void IRAM_ATTR calc_epd_input_4bpp(uint32_t *line_data, uint8_t *epd_input,
                                        uint8_t k, uint8_t *conversion_lut) {
            uint32_t *wide_epd_input = (uint32_t *)epd_input;
            uint16_t *line_data_16 = (uint16_t *)line_data;

            // this is reversed for little-endian, but this is later compensated
            // through the output peripheral.
            for (uint32_t j = 0; j < width / 16; j++) {
                uint16_t v1 = *(line_data_16++);
                uint16_t v2 = *(line_data_16++);
                uint16_t v3 = *(line_data_16++);
                uint16_t v4 = *(line_data_16++);
                uint32_t pixel = conversion_lut[v1] << 16 |
                                conversion_lut[v2] << 24 | conversion_lut[v3] |
                                conversion_lut[v4] << 8;
                wide_epd_input[j] = pixel;
            }
        }

        void IRAM_ATTR calc_epd_input_1bpp(uint8_t *line_data, uint8_t *epd_input,
                                        ed047tc1_helpers::DrawMode mode) {
            uint32_t *wide_epd_input = (uint32_t *)epd_input;
            
            // this is reversed for little-endian, but this is later compensated
            // through the output peripheral.
            for (uint32_t j = 0; j < width / 16; j++) {
                uint8_t v1 = *(line_data++);
                uint8_t v2 = *(line_data++);
                wide_epd_input[j] = (ed047tc1_helpers::arrays::lut_1bpp[v1] << 16) | ed047tc1_helpers::arrays::lut_1bpp[v2];
            }
        }

        static void IRAM_ATTR reset_lut(uint8_t *lut_mem, ed047tc1_helpers::DrawMode mode) {
            switch (mode) {
                case ed047tc1_helpers::DrawMode::BLACK_ON_WHITE:
                    memset(lut_mem, 0x55, (1 << 16));
                    break;
                case ed047tc1_helpers::DrawMode::WHITE_ON_BLACK:
                case ed047tc1_helpers::DrawMode::WHITE_ON_WHITE:
                    memset(lut_mem, 0xAA, (1 << 16));
                    break;
                default:
                    ESP_LOGW("epd_driver", "unknown draw mode %d!", mode);
                    break;
            }
        }

        static void IRAM_ATTR update_LUT(uint8_t *lut_mem, uint8_t k,
                                        ed047tc1_helpers::DrawMode mode) {
            if (mode == ed047tc1_helpers::DrawMode::BLACK_ON_WHITE || mode == ed047tc1_helpers::DrawMode::WHITE_ON_WHITE) {
                k = 15 - k;
            }

            // reset the pixels which are not to be lightened / darkened
            // any longer in the current frame
            for (uint32_t l = k; l < (1 << 16); l += 16) {
                lut_mem[l] &= 0xFC;
            }

            for (uint32_t l = (k << 4); l < (1 << 16); l += (1 << 8)) {
                for (uint32_t p = 0; p < 16; p++) {
                    lut_mem[l + p] &= 0xF3;
                }
            }
            for (uint32_t l = (k << 8); l < (1 << 16); l += (1 << 12)) {
                for (uint32_t p = 0; p < (1 << 8); p++) {
                    lut_mem[l + p] &= 0xCF;
                }
            }
            for (uint32_t p = (k << 12); p < ((k + 1) << 12); p++) {
                lut_mem[p] &= 0x3F;
            }
        }

        void IRAM_ATTR nibble_shift_buffer_right(uint8_t *buf, uint32_t len) {
            uint8_t carry = 0xF;
            for (uint32_t i = 0; i < len; i++) {
                uint8_t val = buf[i];
                buf[i] = (val << 4) | carry;
                carry = (val & 0xF0) >> 4;
            }
        }

        /*
        * bit-shift a buffer `shift` <= 7 bits to the right.
        */
        void IRAM_ATTR bit_shift_buffer_right(uint8_t *buf, uint32_t len,
                                            int shift) {
            uint8_t carry = 0x00;
            for (uint32_t i = 0; i < len; i++) {
                uint8_t val = buf[i];
                buf[i] = (val << shift) | carry;
                carry = val >> (8 - shift);
            }
        }
    };
    template <uint16_t Width, uint16_t Height, int8_t PinCfgData, int8_t PinCfgClk,
            int8_t PinCfgStr, int8_t PinCkv, int8_t PinSth, int8_t PinCkh,
            int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
            int8_t PinD5, int8_t PinD6, int8_t PinD7>
    intr_handle_t ed047tc1<Width, Height, PinCfgData, PinCfgClk, PinCfgStr, PinCkv,
                        PinSth, PinCkh, PinD0, PinD1, PinD2, PinD3, PinD4, PinD5,
                        PinD6, PinD7>::gRMT_intr_handle = nullptr;
    template <uint16_t Width, uint16_t Height, int8_t PinCfgData, int8_t PinCfgClk,
            int8_t PinCfgStr, int8_t PinCkv, int8_t PinSth, int8_t PinCkh,
            int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
            int8_t PinD5, int8_t PinD6, int8_t PinD7>
    rmt_config_t ed047tc1<Width, Height, PinCfgData, PinCfgClk, PinCfgStr, PinCkv,
                        PinSth, PinCkh, PinD0, PinD1, PinD2, PinD3, PinD4, PinD5,
                        PinD6, PinD7>::row_rmt_config = {0};
    template <uint16_t Width, uint16_t Height, int8_t PinCfgData, int8_t PinCfgClk,
            int8_t PinCfgStr, int8_t PinCkv, int8_t PinSth, int8_t PinCkh,
            int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
            int8_t PinD5, int8_t PinD6, int8_t PinD7>
    volatile bool ed047tc1<Width, Height, PinCfgData, PinCfgClk, PinCfgStr, PinCkv,
                        PinSth, PinCkh, PinD0, PinD1, PinD2, PinD3, PinD4, PinD5,
                        PinD6, PinD7>::rmt_tx_done = true;
    template <uint16_t Width, uint16_t Height, int8_t PinCfgData, int8_t PinCfgClk,
            int8_t PinCfgStr, int8_t PinCkv, int8_t PinSth, int8_t PinCkh,
            int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
            int8_t PinD5, int8_t PinD6, int8_t PinD7>
    ed047tc1_helpers::epd_config_register_t ed047tc1<
        Width, Height, PinCfgData, PinCfgClk, PinCfgStr, PinCkv, PinSth, PinCkh,
        PinD0, PinD1, PinD2, PinD3, PinD4, PinD5, PinD6, PinD7>::config_reg = {
        0, 0, 0, 0, 0, 0, 0, 0};

}  // namespace arduino