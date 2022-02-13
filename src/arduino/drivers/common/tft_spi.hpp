#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "tft_core.hpp"
#ifdef ESP32
    #include "soc/spi_reg.h"
    #include "driver/spi_master.h"
#endif

namespace arduino {
    template<uint8_t SpiHost,
        int8_t PinCS, 
        int8_t PinMosi, 
        int8_t PinMiso, 
        int8_t PinSClk, 
        uint8_t SpiMode = 0, 
        uint32_t SpiWriteSpeed=26*1000*1000, 
        uint32_t SpiReadSpeed=20*1000*1000, 
        bool SdaRead = (PinMiso < 0)
#ifdef OPTIMIZE_DMA
    , size_t DmaSize = 4120
    , uint8_t DmaChannel = 
    #ifdef ESP32
        1
    #else
    0
    #endif // ESP32
#endif // OPTIMIZE_DMA
    >
    struct tft_spi {
        constexpr static const tft_io_type type = tft_io_type::spi;
        constexpr static const bool readable = PinMiso > -1 || SdaRead;
        constexpr static const bool sda_read = PinMiso < 0 && SdaRead;
        constexpr static const size_t dma_size =
#ifdef OPTIMIZE_DMA
        DmaSize
#else
        0
#endif
        ;
constexpr static const uint8_t dma_channel =
#ifdef OPTIMIZE_DMA
        DmaChannel
#else
        0
#endif
        ;
        constexpr static const uint8_t spi_host = SpiHost;
        
        constexpr static const uint8_t spi_mode = SpiMode;
        constexpr static const uint32_t spi_write_speed = SpiWriteSpeed;
        constexpr static const uint32_t spi_read_speed = SpiReadSpeed;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_mosi = PinMosi;
        constexpr static const int8_t pin_miso = PinMiso;
        constexpr static const int8_t pin_sclk = PinSClk;
    private:
        constexpr static const uint8_t spi_host_real = (HSPI==SpiHost)?1:(VSPI==SpiHost)?2:0;
        static SPIClass spi;
        static bool lock_transaction;
        static bool in_transaction;
        static bool locked;
#if defined(OPTIMIZE_ESP32)
        // Volatile for register reads:
        volatile static uint32_t* _spi_cmd;
        volatile static uint32_t* _spi_user;
        // Register writes only:
        volatile static uint32_t* _spi_mosi_dlen;
        volatile static uint32_t* _spi_w;
        #ifdef OPTIMIZE_DMA
        static spi_device_handle_t dma_hal;
        static uint8_t spi_busy_check;
        #endif
#endif // OPTIMIZE_ESP32
    public:
        static bool initialize() {
            pinMode(pin_cs,OUTPUT);
            digitalWrite(pin_cs,HIGH);
            spi.begin(pin_sclk,pin_miso,pin_mosi,-1);
            lock_transaction = false;
            in_transaction = false;
            locked = true;
            return true;
        }
        static void deinitialize() {
            lock_transaction = false;
            in_transaction = false;
            locked = true;
        }
        static bool initialize_dma() {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size>0) {
                esp_err_t ret;
                spi_bus_config_t buscfg = {
                    .mosi_io_num = pin_mosi,
                    .miso_io_num = pin_miso,
                    .sclk_io_num = pin_sclk,
                    .quadwp_io_num = -1,
                    .quadhd_io_num = -1,
                    .max_transfer_sz = dma_size,
                    .flags = 0,
                    .intr_flags = 0
                };
                spi_device_interface_config_t devcfg = {
                    .command_bits = 0,
                    .address_bits = 0,
                    .dummy_bits = 0,
                    .mode = spi_mode,
                    .duty_cycle_pos = 0,
                    .cs_ena_pretrans = 0,
                    .cs_ena_posttrans = 0,
                    .clock_speed_hz = spi_write_speed,
                    .input_delay_ns = 0,
                    .spics_io_num = -1,
                    .flags = SPI_DEVICE_NO_DUMMY, //0,
                    .queue_size = 1,
                    .pre_cb = 0,
                    // callback to handle DMA chaining: (not working)
                    .post_cb = 0 //dma_chain_callback
                };
                ret = spi_bus_initialize((spi_host_device_t)spi_host_real, &buscfg, dma_channel);
                ESP_ERROR_CHECK(ret);
                ret = spi_bus_add_device((spi_host_device_t)spi_host_real, &devcfg, &dma_hal);
                ESP_ERROR_CHECK(ret);
                spi_busy_check = 0;

            }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            return true;
        
        }
        static inline void set_command() FORCE_INLINE {

        }
        static inline void set_data() FORCE_INLINE {

        }
        static void deinitialize_dma() {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size>0) {
                spi_bus_remove_device(dma_hal);
                spi_bus_free((spi_host_device_t)spi_host_real);
            }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)

        }
        inline static void busy_check() FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            while (*_spi_cmd&SPI_USR);
#endif
        }
        inline static void start_transaction() FORCE_INLINE {
            in_transaction = true;
        }
        inline static void end_transaction() FORCE_INLINE {
            in_transaction = lock_transaction;
        }
        static void write_raw(const uint8_t* data, size_t length) {
            while(length--) {
                write_raw8(*data++);
            }
        }
        static void write_raw_pgm(const uint8_t* data, size_t length) {
            while(length--) {
                write_raw8(pgm_read_byte(data++));
            }
        }
        static void write_raw_dma(const uint8_t* data,size_t length) {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size == 0) {
                write_raw(data,length);
            } else {
                if(length==0) return;
                dma_wait();
                static spi_transaction_t trans;
                memset(&trans,0,sizeof(spi_transaction_t));
                trans.user=(void*)length;
                trans.tx_buffer = data;
                
                trans.length = ((length>dma_size)?dma_size:length) * 8;
                trans.flags = 0;
                esp_err_t ret = spi_device_queue_trans(dma_hal,&trans,portMAX_DELAY);
                assert(ret==ESP_OK);
                ++spi_busy_check;
            }
#else // !defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            write_raw(data,length);
#endif // !defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            *_spi_mosi_dlen = 7;
            *_spi_w = value;
            *_spi_cmd = SPI_USR;
            while (*_spi_cmd & SPI_USR);
#elif defined(OPTIMIZE_AVR)
            SPDR=(C); 
            while (!(SPSR&_BV(SPIF)));
#else // !OPTIMIZE_ESP32
            spi.transfer(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw8_repeat(uint16_t value, size_t count) {
#if defined(OPTIMIZE_ESP32)
            volatile uint32_t* spi_w = _spi_w;
            uint32_t val32 = (value<<24) | (value <<16) | (value<<8) | value;
            uint32_t i = 0;
            uint32_t rem = count & 0x1F;
            count =  count - rem;

            // Start with partial buffer pixels
            if (rem)
            {
            while (*_spi_cmd&SPI_USR);
            for (i=0; i < rem; ++i) *spi_w++ = val32;
            *_spi_mosi_dlen = (rem << 3) - 1;
            *_spi_cmd = SPI_USR;
            if (!count) return; //{while (*_spi_cmd&SPI_USR); return; }
            i = i>>1; while(i++<8) *spi_w++ = val32;
            }

            while (*_spi_cmd&SPI_USR);
            if (!rem) while (i++<8) *spi_w++ = val32;
            *_spi_mosi_dlen =  511;

            // End with full buffer to maximise useful time for downstream code
            while(count)
            {
            while (*_spi_cmd&SPI_USR);
            *_spi_cmd = SPI_USR;
                count -= 32;
            }
#else // !OPTIMIZE_ESP32
            while(count--) write_raw8(value);
#endif // !OPTIMIZE_ESP32
        }
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            *_spi_mosi_dlen = 15;
            *_spi_w = (value<<8)|(value>>8);
            *_spi_cmd = SPI_USR;
            while (*_spi_cmd & SPI_USR);
#elif defined(__AVR__)
            // AVR does not have 16 bit write
            write_raw8(value>>8);
            write_raw8(value);
#else // !OPTIMIZE_ESP32
            spi.transfer16(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
#if defined(OPTIMIZE_ESP32)
            volatile uint32_t* spi_w = _spi_w;
            uint32_t val32 = (value<<8 | value >>8)<<16 | (value<<8 | value >>8);  
            uint32_t i = 0;
            uint32_t rem = count & 0x1F;
            count =  count - rem;

            // Start with partial buffer pixels
            if (rem)
            {
            while (*_spi_cmd&SPI_USR);
            for (i=0; i < rem; i+=2) *spi_w++ = val32;
            *_spi_mosi_dlen = (rem << 4) - 1;
            *_spi_cmd = SPI_USR;
            if (!count) return; //{while (*_spi_cmd&SPI_USR); return; }
            i = i>>1; while(i++<16) *spi_w++ = val32;
            }

            while (*_spi_cmd&SPI_USR);
            if (!rem) while (i++<16) *spi_w++ = val32;
            *_spi_mosi_dlen =  511;

            // End with full buffer to maximise useful time for downstream code
            while(count)
            {
            while (*_spi_cmd&SPI_USR);
            *_spi_cmd = SPI_USR;
                count -= 32;
            }
#else // !OPTIMIZE_ESP32
            while(count--) {
                write_raw16(value);
            }
#endif // !OPTIMIZE_ESP32

        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
            write_raw16(value>>16);
            write_raw16(value);
        }
        static uint8_t read_raw8() {
#if defined(OPTIMIZE_ESP32)
            return spi.transfer(0);
#else // !OPTIMIZE_ESP32
            if(sda_read) {
                uint8_t  ret = 0;
                for (uint8_t i = 0; i < 8; i++) {  // read results
                    ret <<= 1;
                    sclk_low();
                    if (digitalRead(pin_mosi)) ret |= 1;
                    sclk_high();
                }
                return ret;
            } else {
                return spi.transfer(0);
            }
#endif // !OPTIMIZE_ESP32
        }
        static void begin_write(bool lock=false) {
            if(lock) {
                lock_transaction  =true;
                in_transaction = true;
            }
            if (locked) {
                locked = false; // Flag to show SPI access now unlocked
                spi.beginTransaction(SPISettings(spi_write_speed, MSBFIRST, spi_mode)); // RP2040 SDK -> 68us delay!
                cs_low();
#if defined(OPTIMIZE_ESP32)
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
        static void end_write(bool unlock=false) {
            if(unlock) {
                dma_wait();
                lock_transaction  =false;
                in_transaction = false;
            }
            if(!in_transaction) {
                if(!locked) {
                    locked = true;
#if defined(OPTIMIZE_ESP32)
                    while (*_spi_cmd&SPI_USR);
#endif // OPTIMIZE_ESP32
                    cs_high();
                    spi.endTransaction();
                }
#if defined(OPTIMIZE_ESP32)
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
                }
#endif // OPTIMIZE_ESP32
            }
            
        }
        
        static void begin_read() {
            dma_wait();
            if (locked) {
                locked = false;
                spi.beginTransaction(SPISettings(spi_read_speed, MSBFIRST, spi_mode)); // RP2040 SDK -> 68us delay!
                cs_low();
            }
#if defined(OPTIMIZE_ESP32)
            if(spi_mode==1||spi_mode==2) {
                *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
            } else {
                *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
            }
#endif // OPTIMIZE_ESP32
        }
        static void end_read() {
            if(!in_transaction) {
                if(!locked) {
                    locked = true;
                    cs_high();
                    spi.endTransaction();
                }
#if defined(OPTIMIZE_ESP32)
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
        
        inline static void direction(uint8_t direction) FORCE_INLINE {
            if(direction==INPUT) {
                begin_sda_read();
            } else {
                end_sda_read();
            }
        }
        static void dma_wait() {
            if(dma_size>0) {
#if defined(OPTIMIZE_ESP32)
    #ifdef OPTIMIZE_DMA
                if(!spi_busy_check) return;
                spi_transaction_t* rtrans;
                esp_err_t ret;
                for (int i = 0; i < spi_busy_check; ++i) {
                    ret = spi_device_get_trans_result(dma_hal, &rtrans, portMAX_DELAY);
                    assert(ret == ESP_OK);
                }
                spi_busy_check = 0;
    #endif // OPTIMIZE_DMA
#endif // OPTIMIZE_ESP32
            }
        }
        static bool dma_busy() {
            if(dma_size>0) {
#if defined(OPTIMIZE_ESP32)
    #ifdef OPTIMIZE_DMA
                if(!spi_busy_check) return false;
                spi_transaction_t* rtrans;
                esp_err_t ret;
                uint8_t checks = spi_busy_check;
                for (int i = 0; i < checks; ++i) {
                    ret = spi_device_get_trans_result(dma_hal, &rtrans, portMAX_DELAY);
                    if(ret == ESP_OK) spi_busy_check--;
                }
                if(spi_busy_check == 0) return false;
                return true;
    #endif // OPTIMIZE_DMA
#endif // OPTIMIZE_ESP32
            }
            return false;
        }
        inline static void cs_low() FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            if(pin_cs>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_cs - 32)&31));
                GPIO.out1_w1tc.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1tc = (1 << (pin_cs&31));
                GPIO.out_w1tc = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void cs_high() FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            if(pin_cs>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1ts = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
        inline static void sclk_low() FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            if(pin_sclk>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_sclk - 32)&31));
            } else if(pin_sclk>-1) {
                GPIO.out_w1tc = (1 << (pin_sclk&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_sclk,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void sclk_high() FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            if(pin_sclk>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_sclk - 32)&31));
            } else if(pin_sclk>-1) {
                GPIO.out_w1ts = (1 << (pin_sclk&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
    private:
/*
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
        static void IRAM_ATTR dma_chain_callback(spi_transaction_t* ptrans) {
            size_t read = (ptrans->length/8);
            size_t remaining = ((size_t)ptrans->user)-read;
            if(remaining>0) {
                static spi_transaction_t trans;
                memset(&trans,0,sizeof(spi_transaction_t));
                size_t ns = (remaining>dma_size)?dma_size:remaining;
                trans.user=(void*)remaining;
                trans.tx_buffer = ((const uint8_t*)ptrans->tx_buffer)+read;
                trans.length = ns * 8;
                trans.flags = 0;
                esp_err_t ret = spi_device_queue_trans(dma_hal,&trans,portMAX_DELAY);
                assert(ret==ESP_OK);
            }
        }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
*/
        static void begin_sda_read() {
            if(sda_read) {
#if defined(OPTIMIZE_ESP32)
                pinMatrixOutDetach(pin_mosi, false, false);
                pinMode(pin_mosi, INPUT);
                pinMatrixInAttach(pin_mosi, VSPIQ_IN_IDX, false);
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
                }
#endif // OPTIMIZE_ESP32
            }
            cs_low();
        }
        static void end_sda_read() {
            if(sda_read) {
#if defined(OPTIMIZE_ESP32)
                pinMode(pin_mosi, OUTPUT);
                pinMatrixOutAttach(pin_mosi, VSPID_OUT_IDX, false, false);
                pinMode(pin_miso, INPUT);
                pinMatrixInAttach(pin_miso, VSPIQ_IN_IDX, false);
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
    };
    
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::lock_transaction = false;

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::in_transaction = false;

    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::locked = true;

#if defined(OPTIMIZE_ESP32)
template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_cmd = (volatile uint32_t*)(SPI_CMD_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_user = (volatile uint32_t*)(SPI_USER_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode,uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed,  bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_mosi_dlen = (volatile uint32_t*)(SPI_MOSI_DLEN_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_w = (volatile uint32_t*)(SPI_W0_REG(SpiHost));
#endif // OPTIMIZE_ESP32

#ifndef OPTIMIZE_ESP32
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > uint32_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::cs_pin_mask = digitalPinToBitMask(pin_cs);
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > uint32_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::sclk_pin_mask = digitalPinToBitMask(pin_sclk);
#endif

    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > SPIClass tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::spi(SpiHost);

#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead, size_t DmaSize, uint8_t DmaChannel> uint8_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead,DmaSize,DmaChannel>::spi_busy_check = 0;
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead, size_t DmaSize, uint8_t DmaChannel> spi_device_handle_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead,DmaSize,DmaChannel>::dma_hal = {0};
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)

}