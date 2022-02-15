#include <driver/rmt.h>
#include <esp_idf_version.h>
#include <driver/gpio.h>
#include <esp_attr.h>
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
}  // namespace ed047tc1_helpers
template <int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
          int8_t PinD5, int8_t PinD6, int8_t PinD7, int8_t PinClk, int8_t PinWR,
          int8_t PinDC,int8_t PinSth,int>
class ed047tc1 final {
    constexpr static const int8_t pin_d0 = PinD0;
    constexpr static const int8_t pin_d1 = PinD1;
    constexpr static const int8_t pin_d2 = PinD2;
    constexpr static const int8_t pin_d3 = PinD3;
    constexpr static const int8_t pin_d4 = PinD4;
    constexpr static const int8_t pin_d5 = PinD5;
    constexpr static const int8_t pin_d6 = PinD6;
    constexpr static const int8_t pin_d7 = PinD7;
    constexpr static const int8_t pin_clk = PinClk;
    constexpr static const int8_t pin_wr = PinWR;
    constexpr static const int8_t pin_dc = PinDC;
    constexpr static const int8_t pin_sth = PinSth;
    using pixel_type = gfx::gsc_pixel<4>;
#ifndef ESP32
    static_assert(false, "This driver only works with an ESP32");
#endif
    using bus = ed047tc1_helpers::ed047tc1_i2s<960, pin_d0, pin_d1, pin_d2,
                                               pin_d3, pin_d4, pin_d5, pin_d6,
                                               pin_d7, pin_clk, pin_wr>;

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
    inline static void fast_gpio_set_hi(gpio_num_t gpio_num) {
        GPIO.out_w1ts = (1 << gpio_num);
    }

    inline static void fast_gpio_set_lo(gpio_num_t gpio_num) {
        GPIO.out_w1tc = (1 << gpio_num);
    }

    void IRAM_ATTR busy_delay(uint32_t cycles) {
        volatile unsigned long counts = XTHAL_GET_CCOUNT() + cycles;
        while (XTHAL_GET_CCOUNT() < counts) {
        };
    }

    inline static void IRAM_ATTR push_cfg_bit(bool bit) {
        fast_gpio_set_lo(pin_clk);
        if (bit) {
            fast_gpio_set_hi(CFG_DATA);
        } else {
            fast_gpio_set_lo(CFG_DATA);
        }
        fast_gpio_set_hi(CFG_CLK);
    }

    static void IRAM_ATTR push_cfg(ed047tc1_helpers::epd_config_register_t *cfg) {
        fast_gpio_set_lo(CFG_STR);

        // push config bits in reverse order
        push_cfg_bit(cfg->ep_output_enable);
        push_cfg_bit(cfg->ep_mode);
        push_cfg_bit(cfg->ep_scan_direction);
        push_cfg_bit(cfg->ep_stv);

        push_cfg_bit(cfg->neg_power_enable);
        push_cfg_bit(cfg->pos_power_enable);
        push_cfg_bit(cfg->power_disable);
        push_cfg_bit(cfg->ep_latch_enable);

        fast_gpio_set_hi(CFG_STR);
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
        gpio_set_direction(pin_dc, GPIO_MODE_OUTPUT);
        gpio_set_direction(pin_clk, GPIO_MODE_OUTPUT);
        gpio_set_direction(pin_wr, GPIO_MODE_OUTPUT);
        fast_gpio_set_lo(pin_wr);

        push_cfg(&config_reg);

        // Setup I2S
        i2s_bus_config i2s_config;
        // add an offset off dummy bytes to allow for enough timing headroom
        i2s_config.epd_row_width = width + 32;
        i2s_config.clock = (gpio_num_t)pin_clk;
        i2s_config.start_pulse = (gpio_num_t)pin_wr;
        i2s_config.data_0 = (gpio_num_t)pin_d0;
        i2s_config.data_1 = (gpio_num_t)pin_d1;
        i2s_config.data_2 = (gpio_num_t)pin_d2;
        i2s_config.data_3 = (gpio_num_t)pin_d3;
        i2s_config.data_4 = (gpio_num_t)pin_d4;
        i2s_config.data_5 = (gpio_num_t)pin_d5;
        i2s_config.data_6 = (gpio_num_t)pin_d6;
        i2s_config.data_7 = (gpio_num_t)pin_d7;

        i2s_bus_init(&i2s_config);

        rmt_pulse_init(CKV);
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
        fast_gpio_set_hi(STH);
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
        while (i2s_is_busy() || rmt_busy()) {
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
        while (i2s_is_busy() || rmt_busy()) {
        };
        latch_row();

        pulse_ckv_ticks(output_time_dus, 50, false);

        i2s_start_line_output();
        i2s_switch_buffer();
    }

    void epd_end_frame() {
        config_reg.ep_output_enable = false;
        push_cfg(&config_reg);
        config_reg.ep_mode = false;
        push_cfg(&config_reg);
        pulse_ckv_us(1, 1, true);
        pulse_ckv_us(1, 1, true);
    }

    void IRAM_ATTR epd_switch_buffer() { i2s_switch_buffer(); }
    uint8_t IRAM_ATTR *epd_get_current_buffer() {
        return (uint8_t *)i2s_get_current_buffer();
    };
};
template <int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
          int8_t PinD5, int8_t PinD6, int8_t PinD7, int8_t PinClk, int8_t PinWR,
          int8_t PinDC, int8_t PinSth>
intr_handle_t ed047tc1<PinD0, PinD1, PinD2, PinD3, PinD4, PinD5, PinD6, PinD7,
                       PinClk, PinWR, PinDC, PinSth>::gRMT_intr_handle = nullptr;
template <int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
          int8_t PinD5, int8_t PinD6, int8_t PinD7, int8_t PinClk, int8_t PinWR,
          int8_t PinDC, int8_t PinSth>
rmt_config_t ed047tc1<PinD0, PinD1, PinD2, PinD3, PinD4, PinD5, PinD6, PinD7,
                      PinClk, PinWR, PinDC, PinSth>::row_rmt_config = {0};
template <int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
          int8_t PinD5, int8_t PinD6, int8_t PinD7, int8_t PinClk, int8_t PinWR,
          int8_t PinDC, int8_t PinSth>
volatile bool ed047tc1<PinD0, PinD1, PinD2, PinD3, PinD4, PinD5, PinD6, PinD7,
                       PinClk, PinWR, PinDC, PinSth>::rmt_tx_done = true;
template <int8_t PinD0, int8_t PinD1, int8_t PinD2, int8_t PinD3, int8_t PinD4,
          int8_t PinD5, int8_t PinD6, int8_t PinD7, int8_t PinClk, int8_t PinWR,
          int8_t PinDC, int8_t PinSth>
ed047tc1_helpers::epd_config_register_t
    ed047tc1<PinD0, PinD1, PinD2, PinD3, PinD4, PinD5, PinD6, PinD7, PinClk,
             PinWR, PinDC, PinSth>::config_reg = {0, 0, 0, 0, 0, 0, 0, 0};
}  // namespace arduino