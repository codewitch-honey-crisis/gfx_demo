/**
 * Implements a 8bit parallel interface to transmit pixel
 * data to the display, based on the I2S peripheral.
 */

#pragma once

#include <stdint.h>

#include <driver/gpio.h>
#include <esp_attr.h>

#include <driver/periph_ctrl.h>
#include <esp_heap_caps.h>

// #include <esp_idf_version.h>
// #if ESP_IDF_VERSION_MAJOR >= 4
// #include <esp32/rom/lldesc.h>
// #else

#include <rom/lldesc.h>

// #endif
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>
#include <soc/rtc.h>
namespace arduino {
    namespace ed047tc1_helpers {
        typedef struct {
            volatile lldesc_t *dma_desc_a;
            volatile lldesc_t *dma_desc_b;

            /// Front and back line buffer.
            uint8_t *buf_a;
            uint8_t *buf_b;
        } i2s_parallel_state_t;

        template<uint32_t Width,
                int8_t PinD0, 
                int8_t PinD1, 
                int8_t PinD2,
                int8_t PinD3,
                int8_t PinD4,
                int8_t PinD5,
                int8_t PinD6,
                int8_t PinD7,
                int8_t PinClk,
                int8_t PinWR>
        struct ed047tc1_i2s final {
            constexpr static const uint32_t width = Width;
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

            static bool initialize() {
                // TODO: Why?
                gpio_num_t I2S_GPIO_BUS[] = {(gpio_num_t)pin_d6, (gpio_num_t)pin_d7, (gpio_num_t)pin_d4,
                                            (gpio_num_t)pin_d5, (gpio_num_t)pin_d2, (gpio_num_t)pin_d3,
                                            (gpio_num_t)pin_d0, (gpio_num_t)pin_d1
                                            };

                gpio_set_direction((gpio_num_t)pin_wr, GPIO_MODE_OUTPUT);
                gpio_set_level((gpio_num_t)pin_wr, 1);
                
                // Use I2S1 with no signal offset (for some reason the offset seems to be
                // needed in 16-bit mode, but not in 8 bit mode.
                int signal_base = I2S1O_DATA_OUT0_IDX;

                // Setup and route GPIOS
                for (int x = 0; x < 8; x++) {
                    gpio_setup_out(I2S_GPIO_BUS[x], signal_base + x, false);
                }
                // Invert word select signal
                gpio_setup_out(pin_clk, I2S1O_WS_OUT_IDX, true);

                periph_module_enable(PERIPH_I2S1_MODULE);

                i2s_dev_t *dev = &I2S1;

                // Initialize device
                dev->conf.tx_reset = 1;
                dev->conf.tx_reset = 0;

                // Reset DMA
                dev->lc_conf.in_rst = 1;
                dev->lc_conf.in_rst = 0;
                dev->lc_conf.out_rst = 1;
                dev->lc_conf.out_rst = 0;

                // Setup I2S config. See section 12 of Technical Reference Manual
                // Enable LCD mode
                dev->conf2.val = 0;
                dev->conf2.lcd_en = 1;

                // Enable FRAME1-Mode (See technical reference manual)
                dev->conf2.lcd_tx_wrx2_en = 1;
                dev->conf2.lcd_tx_sdx2_en = 0;

                // Set to 8 bit parallel output
                dev->sample_rate_conf.val = 0;
                dev->sample_rate_conf.tx_bits_mod = 8;

                // Half speed of bit clock in LCD mode.
                // (Smallest possible divider according to the spec).
                dev->sample_rate_conf.tx_bck_div_num = 2;

            //#if defined(CONFIG_EPD_DISPLAY_TYPE_ED097OC4_LQ)
                // Initialize Audio Clock (APLL) for 120 Mhz.
                rtc_clk_apll_enable(1, 0, 0, 8, 0);
            //#else
                // Initialize Audio Clock (APLL) for 80 Mhz.
            // rtc_clk_apll_enable(1, 0, 0, 8, 1);
            //#endif


                // Set Audio Clock Dividers
                dev->clkm_conf.val = 0;
                dev->clkm_conf.clka_en = 1;
                dev->clkm_conf.clkm_div_a = 1;
                dev->clkm_conf.clkm_div_b = 0;
                // 2 is the smallest possible divider according to the spec.
                dev->clkm_conf.clkm_div_num = 2;

                // Set up FIFO
                dev->fifo_conf.val = 0;
                dev->fifo_conf.tx_fifo_mod_force_en = 1;
                dev->fifo_conf.tx_fifo_mod = 1;
                dev->fifo_conf.tx_data_num = 32;
                dev->fifo_conf.dscr_en = 1;

                // Stop after transmission complete
                dev->conf1.val = 0;
                dev->conf1.tx_stop_en = 1;
                dev->conf1.tx_pcm_bypass = 1;

                // Configure TX channel
                dev->conf_chan.val = 0;
                dev->conf_chan.tx_chan_mod = 1;
                dev->conf.tx_right_first = 1;

                dev->timing.val = 0;

                // Allocate DMA descriptors
                i2s_state.buf_a = (uint8_t*)heap_caps_malloc(width / 4, MALLOC_CAP_DMA);
                if(i2s_state.buf_a==nullptr) {
                    return false;
                }
                i2s_state.buf_b = (uint8_t*)heap_caps_malloc(width / 4, MALLOC_CAP_DMA);
                if(i2s_state.buf_b==nullptr) {
                    return false;
                }
                i2s_state.dma_desc_a = (lldesc_t*)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
                if(i2s_state.dma_desc_a==nullptr) {
                    return false;
                }
                i2s_state.dma_desc_b = (lldesc_t*)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
                if(i2s_state.dma_desc_b==nullptr) {
                    return false;
                }
                // and fill them
                fill_dma_desc(i2s_state.dma_desc_a, i2s_state.buf_a);
                fill_dma_desc(i2s_state.dma_desc_b, i2s_state.buf_b);

                // enable "done" interrupt
                SET_PERI_REG_BITS(I2S_INT_ENA_REG(1), I2S_OUT_DONE_INT_ENA_V, 1,
                                I2S_OUT_DONE_INT_ENA_S);
                // register interrupt
                esp_intr_alloc(ETS_I2S1_INTR_SOURCE, 0, i2s_int_hdl, 0, &gI2S_intr_handle);

                // Reset FIFO/DMA
                dev->lc_conf.in_rst = 1;
                dev->lc_conf.out_rst = 1;
                dev->lc_conf.ahbm_rst = 1;
                dev->lc_conf.ahbm_fifo_rst = 1;
                dev->lc_conf.in_rst = 0;
                dev->lc_conf.out_rst = 0;
                dev->lc_conf.ahbm_rst = 0;
                dev->lc_conf.ahbm_fifo_rst = 0;
                dev->conf.tx_reset = 1;
                dev->conf.tx_fifo_reset = 1;
                dev->conf.rx_fifo_reset = 1;
                dev->conf.tx_reset = 0;
                dev->conf.tx_fifo_reset = 0;
                dev->conf.rx_fifo_reset = 0;

                // Start dma on front buffer
                dev->lc_conf.val =
                    I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
                dev->out_link.addr = ((uint32_t)(i2s_state.dma_desc_a));
                dev->out_link.start = 1;

                dev->int_clr.val = dev->int_raw.val;

                dev->int_ena.val = 0;
                dev->int_ena.out_done = 1;

                dev->conf.tx_start = 0;
                return true;
            }
            static bool deinitialize() {
                esp_intr_free(gI2S_intr_handle);
                if(i2s_state.buf_a!=nullptr) {
                    free(i2s_state.buf_a);
                }
                if(i2s_state.buf_b!=nullptr) {
                    free(i2s_state.buf_b);
                }
                if(nullptr != (void*)i2s_state.dma_desc_a) {
                    free((void *)i2s_state.dma_desc_a);    
                }
                if(nullptr != (void*)i2s_state.dma_desc_b) {
                    free((void *)i2s_state.dma_desc_b);    
                }
                
                periph_module_disable(PERIPH_I2S1_MODULE);
                return true;
            }
            static volatile uint8_t IRAM_ATTR *buffer() {
                return current_buffer ? i2s_state.dma_desc_a->buf : i2s_state.dma_desc_b->buf;
            }
            static void IRAM_ATTR switch_buffer() {
                // either device is done transmitting or the switch must be away from the
                // buffer currently used by the DMA engine.
                while (is_busy() && dma_desc_addr() != I2S1.out_link.addr) {
                };
                current_buffer = !current_buffer;
            }
            static void IRAM_ATTR start_line_output() {
                output_done = false;

                i2s_dev_t *dev = &I2S1;
                dev->conf.tx_start = 0;
                dev->conf.tx_reset = 1;
                dev->conf.tx_fifo_reset = 1;
                dev->conf.rx_fifo_reset = 1;
                dev->conf.tx_reset = 0;
                dev->conf.tx_fifo_reset = 0;
                dev->conf.rx_fifo_reset = 0;
                dev->out_link.addr = dma_desc_addr();
                dev->out_link.start = 1;

                // sth is pulled up through peripheral interrupt
                gpio_set_level(pin_wr, 0);
                dev->conf.tx_start = 1;
            }
            bool IRAM_ATTR is_busy() {
                // DMA and FIFO must be done
                return !output_done || !I2S1.state.tx_idle;
            }
            private:
            typedef struct {
                volatile lldesc_t *dma_desc_a;
                volatile lldesc_t *dma_desc_b;

                /// Front and back line buffer.
                uint8_t *buf_a;
                uint8_t *buf_b;
            } parallel_state_t;
            static int current_buffer; // = 0;

            /// The I2S state instance.
            static i2s_parallel_state_t i2s_state;

            static intr_handle_t gI2S_intr_handle; // = NULL;

            /// Indicates the device has finished its transmission and is ready again.
            static volatile bool output_done; // = true;
        private:
            static void fill_dma_desc(volatile lldesc_t *dmadesc, uint8_t *buf) {
                dmadesc->size = dmadesc->length = width / 4;
                dmadesc->buf = buf;
                dmadesc->eof = 1;
                dmadesc->sosf = 1;
                dmadesc->owner = 1;
                dmadesc->qe.stqe_next = 0;
                dmadesc->offset = 0;
            }
            /// Address of the currently front DMA descriptor,
            /// which uses only the lower 20bits (according to TRM)
            static uint32_t dma_desc_addr() {
                return (uint32_t)(current_buffer ? i2s_state.dma_desc_a
                                : i2s_state.dma_desc_b) &
                    0x000FFFFF;
            }
            /// Set up a GPIO as output and route it to a signal.
            static void gpio_setup_out(int gpio, int sig, bool invert) {
                if (gpio == -1)
                    return;
                PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
                gpio_set_direction((gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
                gpio_matrix_out((gpio_num_t)gpio, sig, invert, false);
            }
            /// Resets "Start Pulse/WR" signal when the current row output is done.
            static void IRAM_ATTR i2s_int_hdl(void *arg)
            {
                i2s_dev_t *dev = &I2S1;
                if (dev->int_st.out_done) {
                    gpio_set_level((gpio_num_t)pin_wr, 1);
                    output_done = true;
                }
                // Clear the interrupt. Otherwise, the whole device would hang.
                dev->int_clr.val = dev->int_raw.val;
            }
        };
        template<uint32_t Width,
                int8_t PinD0, 
                int8_t PinD1, 
                int8_t PinD2,
                int8_t PinD3,
                int8_t PinD4,
                int8_t PinD5,
                int8_t PinD6,
                int8_t PinD7,
                int8_t PinClk,
                int8_t PinWR> int ed047tc1_i2s<Width,
                                            PinD0,
                                            PinD1,
                                            PinD2,
                                            PinD3,
                                            PinD4,
                                            PinD5,
                                            PinD6,
                                            PinD7,
                                            PinClk,
                                            PinWR>::current_buffer = 0;
        template<uint32_t Width,
                int8_t PinD0, 
                int8_t PinD1, 
                int8_t PinD2,
                int8_t PinD3,
                int8_t PinD4,
                int8_t PinD5,
                int8_t PinD6,
                int8_t PinD7,
                int8_t PinClk,
                int8_t PinWR> i2s_parallel_state_t ed047tc1_i2s<Width,
                                            PinD0,
                                            PinD1,
                                            PinD2,
                                            PinD3,
                                            PinD4,
                                            PinD5,
                                            PinD6,
                                            PinD7,
                                            PinClk,
                                            PinWR>::i2s_state = {nullptr,nullptr,nullptr,nullptr};
        template<uint32_t Width,
                int8_t PinD0, 
                int8_t PinD1, 
                int8_t PinD2,
                int8_t PinD3,
                int8_t PinD4,
                int8_t PinD5,
                int8_t PinD6,
                int8_t PinD7,
                int8_t PinClk,
                int8_t PinWR> intr_handle_t ed047tc1_i2s<Width,
                                            PinD0,
                                            PinD1,
                                            PinD2,
                                            PinD3,
                                            PinD4,
                                            PinD5,
                                            PinD6,
                                            PinD7,
                                            PinClk,
                                            PinWR>::gI2S_intr_handle = nullptr;

        template<uint32_t Width,
                int8_t PinD0, 
                int8_t PinD1, 
                int8_t PinD2,
                int8_t PinD3,
                int8_t PinD4,
                int8_t PinD5,
                int8_t PinD6,
                int8_t PinD7,
                int8_t PinClk,
                int8_t PinWR> volatile bool ed047tc1_i2s<Width,
                                            PinD0,
                                            PinD1,
                                            PinD2,
                                            PinD3,
                                            PinD4,
                                            PinD5,
                                            PinD6,
                                            PinD7,
                                            PinClk,
                                            PinWR>::output_done = true;
    }
}