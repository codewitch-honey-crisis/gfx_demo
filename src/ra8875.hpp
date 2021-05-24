#pragma once
#include <stdint.h>
#include "spi_master.hpp"

// Command/Data pins for SPI
#define RA8875_DATAWRITE 0x00 
#define RA8875_DATAREAD 0x40  
#define RA8875_CMDWRITE 0x80  
#define RA8875_CMDREAD 0xC0   

// Registers & bits
#define RA8875_PWRR 0x01           
#define RA8875_PWRR_DISPON 0x80    
#define RA8875_PWRR_DISPOFF 0x00   
#define RA8875_PWRR_SLEEP 0x02     
#define RA8875_PWRR_NORMAL 0x00    
#define RA8875_PWRR_SOFTRESET 0x01 

#define RA8875_MRWC 0x02 

#define RA8875_GPIOX 0xC7 

#define RA8875_PLLC1 0x88         
#define RA8875_PLLC1_PLLDIV2 0x80 
#define RA8875_PLLC1_PLLDIV1 0x00 

#define RA8875_PLLC2 0x89        
#define RA8875_PLLC2_DIV1 0x00   
#define RA8875_PLLC2_DIV2 0x01   
#define RA8875_PLLC2_DIV4 0x02   
#define RA8875_PLLC2_DIV8 0x03   
#define RA8875_PLLC2_DIV16 0x04  
#define RA8875_PLLC2_DIV32 0x05  
#define RA8875_PLLC2_DIV64 0x06  
#define RA8875_PLLC2_DIV128 0x07 

#define RA8875_SYSR 0x10       
#define RA8875_SYSR_8BPP 0x00  
#define RA8875_SYSR_16BPP 0x0C 
#define RA8875_SYSR_MCU8 0x00  
#define RA8875_SYSR_MCU16 0x03 

#define RA8875_PCSR 0x04       
#define RA8875_PCSR_PDATR 0x00 
#define RA8875_PCSR_PDATL 0x80 
#define RA8875_PCSR_CLK 0x00   
#define RA8875_PCSR_2CLK 0x01  
#define RA8875_PCSR_4CLK 0x02  
#define RA8875_PCSR_8CLK 0x03  

#define RA8875_HDWR 0x14 

#define RA8875_HNDFTR 0x15         
#define RA8875_HNDFTR_DE_HIGH 0x00 
#define RA8875_HNDFTR_DE_LOW 0x80  

#define RA8875_HNDR 0x16      
#define RA8875_HSTR 0x17      
#define RA8875_HPWR 0x18      
#define RA8875_HPWR_LOW 0x00  
#define RA8875_HPWR_HIGH 0x80 

#define RA8875_VDHR0 0x19     
#define RA8875_VDHR1 0x1A     
#define RA8875_VNDR0 0x1B     
#define RA8875_VNDR1 0x1C     
#define RA8875_VSTR0 0x1D     
#define RA8875_VSTR1 0x1E     
#define RA8875_VPWR 0x1F      
#define RA8875_VPWR_LOW 0x00  
#define RA8875_VPWR_HIGH 0x80 

#define RA8875_HSAW0 0x30 
#define RA8875_HSAW1 0x31 
#define RA8875_VSAW0 0x32 
#define RA8875_VSAW1 0x33 

#define RA8875_HEAW0 0x34 
#define RA8875_HEAW1 0x35 
#define RA8875_VEAW0 0x36 
#define RA8875_VEAW1 0x37 

#define RA8875_MCLR 0x8E            
#define RA8875_MCLR_START 0x80      
#define RA8875_MCLR_STOP 0x00       
#define RA8875_MCLR_READSTATUS 0x80 
#define RA8875_MCLR_FULL 0x00       
#define RA8875_MCLR_ACTIVE 0x40     

#define RA8875_DCR 0x90                   
#define RA8875_DCR_LINESQUTRI_START 0x80  
#define RA8875_DCR_LINESQUTRI_STOP 0x00   
#define RA8875_DCR_LINESQUTRI_STATUS 0x80 
#define RA8875_DCR_CIRCLE_START 0x40      
#define RA8875_DCR_CIRCLE_STATUS 0x40     
#define RA8875_DCR_CIRCLE_STOP 0x00       
#define RA8875_DCR_FILL 0x20              
#define RA8875_DCR_NOFILL 0x00            
#define RA8875_DCR_DRAWLINE 0x00          
#define RA8875_DCR_DRAWTRIANGLE 0x01      
#define RA8875_DCR_DRAWSQUARE 0x10        

#define RA8875_ELLIPSE 0xA0        
#define RA8875_ELLIPSE_STATUS 0x80 

#define RA8875_MWCR0 0x40         
#define RA8875_MWCR0_GFXMODE 0x00 
#define RA8875_MWCR0_TXTMODE 0x80 
#define RA8875_MWCR0_CURSOR 0x40  
#define RA8875_MWCR0_BLINK 0x20   

#define RA8875_MWCR0_DIRMASK 0x0C ///< Bitmask for Write Direction
#define RA8875_MWCR0_LRTD 0x00    ///< Left->Right then Top->Down
#define RA8875_MWCR0_RLTD 0x04    ///< Right->Left then Top->Down
#define RA8875_MWCR0_TDLR 0x08    ///< Top->Down then Left->Right
#define RA8875_MWCR0_DTLR 0x0C    ///< Down->Top then Left->Right

#define RA8875_BTCR 0x44  
#define RA8875_CURH0 0x46 
#define RA8875_CURH1 0x47 
#define RA8875_CURV0 0x48 
#define RA8875_CURV1 0x49 

#define RA8875_P1CR 0x8A         
#define RA8875_P1CR_ENABLE 0x80  
#define RA8875_P1CR_DISABLE 0x00 
#define RA8875_P1CR_CLKOUT 0x10  
#define RA8875_P1CR_PWMOUT 0x00  

#define RA8875_P1DCR 0x8B 

#define RA8875_P2CR 0x8C         
#define RA8875_P2CR_ENABLE 0x80  
#define RA8875_P2CR_DISABLE 0x00 
#define RA8875_P2CR_CLKOUT 0x10  
#define RA8875_P2CR_PWMOUT 0x00  

#define RA8875_P2DCR 0x8D 

#define RA8875_PWM_CLK_DIV1 0x00     
#define RA8875_PWM_CLK_DIV2 0x01     
#define RA8875_PWM_CLK_DIV4 0x02     
#define RA8875_PWM_CLK_DIV8 0x03     
#define RA8875_PWM_CLK_DIV16 0x04    
#define RA8875_PWM_CLK_DIV32 0x05    
#define RA8875_PWM_CLK_DIV64 0x06    
#define RA8875_PWM_CLK_DIV128 0x07   
#define RA8875_PWM_CLK_DIV256 0x08   
#define RA8875_PWM_CLK_DIV512 0x09   
#define RA8875_PWM_CLK_DIV1024 0x0A  
#define RA8875_PWM_CLK_DIV2048 0x0B  
#define RA8875_PWM_CLK_DIV4096 0x0C  
#define RA8875_PWM_CLK_DIV8192 0x0D  
#define RA8875_PWM_CLK_DIV16384 0x0E 
#define RA8875_PWM_CLK_DIV32768 0x0F 

#define RA8875_TPCR0 0x70               
#define RA8875_TPCR0_ENABLE 0x80        
#define RA8875_TPCR0_DISABLE 0x00       
#define RA8875_TPCR0_WAIT_512CLK 0x00   
#define RA8875_TPCR0_WAIT_1024CLK 0x10  
#define RA8875_TPCR0_WAIT_2048CLK 0x20  
#define RA8875_TPCR0_WAIT_4096CLK 0x30  
#define RA8875_TPCR0_WAIT_8192CLK 0x40  
#define RA8875_TPCR0_WAIT_16384CLK 0x50 
#define RA8875_TPCR0_WAIT_32768CLK 0x60 
#define RA8875_TPCR0_WAIT_65536CLK 0x70 
#define RA8875_TPCR0_WAKEENABLE 0x08    
#define RA8875_TPCR0_WAKEDISABLE 0x00   
#define RA8875_TPCR0_ADCCLK_DIV1 0x00   
#define RA8875_TPCR0_ADCCLK_DIV2 0x01   
#define RA8875_TPCR0_ADCCLK_DIV4 0x02   
#define RA8875_TPCR0_ADCCLK_DIV8 0x03   
#define RA8875_TPCR0_ADCCLK_DIV16 0x04  
#define RA8875_TPCR0_ADCCLK_DIV32 0x05  
#define RA8875_TPCR0_ADCCLK_DIV64 0x06  
#define RA8875_TPCR0_ADCCLK_DIV128 0x07 

#define RA8875_TPCR1 0x71            
#define RA8875_TPCR1_AUTO 0x00       
#define RA8875_TPCR1_MANUAL 0x40     
#define RA8875_TPCR1_VREFINT 0x00    
#define RA8875_TPCR1_VREFEXT 0x20    
#define RA8875_TPCR1_DEBOUNCE 0x04   
#define RA8875_TPCR1_NODEBOUNCE 0x00 
#define RA8875_TPCR1_IDLE 0x00       
#define RA8875_TPCR1_WAIT 0x01       
#define RA8875_TPCR1_LATCHX 0x02     
#define RA8875_TPCR1_LATCHY 0x03     

#define RA8875_TPXH 0x72  
#define RA8875_TPYH 0x73  
#define RA8875_TPXYL 0x74 

#define RA8875_INTC1 0xF0     
#define RA8875_INTC1_KEY 0x10 
#define RA8875_INTC1_DMA 0x08 
#define RA8875_INTC1_TP 0x04  
#define RA8875_INTC1_BTE 0x02 

#define RA8875_INTC2 0xF1     
#define RA8875_INTC2_KEY 0x10 
#define RA8875_INTC2_DMA 0x08 
#define RA8875_INTC2_TP 0x04  
#define RA8875_INTC2_BTE 0x02 

#define RA8875_SCROLL_BOTH 0x00   
#define RA8875_SCROLL_LAYER1 0x40 
#define RA8875_SCROLL_LAYER2 0x80 
#define RA8875_SCROLL_BUFFER 0xC0 
namespace espidf {
    template<uint16_t Width,
            uint16_t Height,
            spi_host_device_t HostId,
            gpio_num_t PinCS,
            gpio_num_t PinBacklight,
            gpio_num_t PinRst=GPIO_NUM_NC,
            gpio_num_t PinInt=GPIO_NUM_NC,
            size_t MaxTransactions=7,
            bool UsePolling = true,
            size_t DmaSize = -1,
            TickType_t Timeout=5000/portTICK_PERIOD_MS,
            size_t BatchBufferSize=64
            >
    struct ra8875 final {
        static_assert((Width==800&&Height==480)||
                        (Width==480&&(
                            Height==80||
                            Height==128||
                            Height==272
                        )),"Invalid screen resolution");
        enum struct result {
            success = 0,
            invalid_argument = 1,
            not_supported = 2,
            out_of_memory = 3,
            io_error = 4,
            timeout = 5,
            busy = 6,
            device_not_detected=7
        };

        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const spi_host_device_t host_id = HostId;
        constexpr static const gpio_num_t pin_cs = PinCS;
        constexpr static const gpio_num_t pin_backlight = PinBacklight;
        constexpr static const gpio_num_t pin_rst = PinRst;
        constexpr static const gpio_num_t pin_int = PinInt;
        constexpr static const size_t max_transactions = MaxTransactions;
        constexpr static const bool use_polling = UsePolling;
        constexpr static const size_t dma_size = DmaSize;
        constexpr static const TickType_t timeout = Timeout;
        constexpr static const size_t batch_buffer_size = BatchBufferSize;
    private:
        enum codes : uint8_t {
            // Command/Data pins for SPI
            DATAWRITE = 0x00 ,
            DATAREAD = 0x40  ,
            CMDWRITE = 0x80  ,
            CMDREAD = 0xC0   ,

            // Registers & bits
            PWRR = 0x01           ,
            PWRR_DISPON = 0x80    ,
            PWRR_DISPOFF = 0x00   ,
            PWRR_SLEEP = 0x02     ,
            PWRR_NORMAL = 0x00    ,
            PWRR_SOFTRESET = 0x01 ,

            MRWC = 0x02 ,

            GPIOX = 0xC7 ,

            PLLC1 = 0x88         ,
            PLLC1_PLLDIV2 = 0x80 ,
            PLLC1_PLLDIV1 = 0x00 ,

            PLLC2 = 0x89        ,
            PLLC2_DIV1 = 0x00   ,
            PLLC2_DIV2 = 0x01   ,
            PLLC2_DIV4 = 0x02   ,
            PLLC2_DIV8 = 0x03   ,
            PLLC2_DIV16 = 0x04  ,
            PLLC2_DIV32 = 0x05  ,
            PLLC2_DIV64 = 0x06  ,
            PLLC2_DIV128 = 0x07 ,

            SYSR = 0x10       ,
            SYSR_8BPP = 0x00  ,
            SYSR_16BPP = 0x0C ,
            SYSR_MCU8 = 0x00  ,
            SYSR_MCU16 = 0x03 ,

            PCSR = 0x04       ,
            PCSR_PDATR = 0x00 ,
            PCSR_PDATL = 0x80 ,
            PCSR_CLK = 0x00   ,
            PCSR_2CLK = 0x01  ,
            PCSR_4CLK = 0x02  ,
            PCSR_8CLK = 0x03  ,

            HDWR = 0x14 ,

            HNDFTR = 0x15         ,
            HNDFTR_DE_HIGH = 0x00 ,
            HNDFTR_DE_LOW = 0x80  ,

            HNDR = 0x16      ,
            HSTR = 0x17      ,
            HPWR = 0x18      ,
            HPWR_LOW = 0x00  ,
            HPWR_HIGH = 0x80 ,

            VDHR0 = 0x19     ,
            VDHR1 = 0x1A     ,
            VNDR0 = 0x1B     ,
            VNDR1 = 0x1C     ,
            VSTR0 = 0x1D     ,
            VSTR1 = 0x1E     ,
            VPWR = 0x1F      ,
            VPWR_LOW = 0x00  ,
            VPWR_HIGH = 0x80 ,

            HSAW0 = 0x30 ,
            HSAW1 = 0x31 ,
            VSAW0 = 0x32 ,
            VSAW1 = 0x33 ,

            HEAW0 = 0x34 ,
            HEAW1 = 0x35 ,
            VEAW0 = 0x36 ,
            VEAW1 = 0x37 ,

            MCLR = 0x8E            ,
            MCLR_START = 0x80      ,
            MCLR_STOP = 0x00       ,
            MCLR_READSTATUS = 0x80 ,
            MCLR_FULL = 0x00       ,
            MCLR_ACTIVE = 0x40     ,

            DCR = 0x90                   ,
            DCR_LINESQUTRI_START = 0x80  ,
            DCR_LINESQUTRI_STOP = 0x00   ,
            DCR_LINESQUTRI_STATUS = 0x80 ,
            DCR_CIRCLE_START = 0x40      ,
            DCR_CIRCLE_STATUS = 0x40     ,
            DCR_CIRCLE_STOP = 0x00       ,
            DCR_FILL = 0x20              ,
            DCR_NOFILL = 0x00            ,
            DCR_DRAWLINE = 0x00          ,
            DCR_DRAWTRIANGLE = 0x01      ,
            DCR_DRAWSQUARE = 0x10        ,

            ELLIPSE = 0xA0        ,
            ELLIPSE_STATUS = 0x80 ,

            MWCR0 = 0x40         ,
            MWCR0_GFXMODE = 0x00 ,
            MWCR0_TXTMODE = 0x80 ,
            MWCR0_CURSOR = 0x40  ,
            MWCR0_BLINK = 0x20   ,

            MWCR0_DIRMASK = 0x0C ,///< Bitmask for Write Direction
            MWCR0_LRTD = 0x00    ,///< Left->Right then Top->Down
            MWCR0_RLTD = 0x04    ,///< Right->Left then Top->Down
            MWCR0_TDLR = 0x08    ,///< Top->Down then Left->Right
            MWCR0_DTLR = 0x0C    ,///< Down->Top then Left->Right

            BTCR = 0x44  ,
            CURH0 = 0x46 ,
            CURH1 = 0x47 ,
            CURV0 = 0x48 ,
            CURV1 = 0x49 ,

            P1CR = 0x8A         ,
            P1CR_ENABLE = 0x80  ,
            P1CR_DISABLE = 0x00 ,
            P1CR_CLKOUT = 0x10  ,
            P1CR_PWMOUT = 0x00  ,

            P1DCR = 0x8B ,

            P2CR = 0x8C         ,
            P2CR_ENABLE = 0x80  ,
            P2CR_DISABLE = 0x00 ,
            P2CR_CLKOUT = 0x10  ,
            P2CR_PWMOUT = 0x00  ,

            P2DCR = 0x8D ,

            PWM_CLK_DIV1 = 0x00     ,
            PWM_CLK_DIV2 = 0x01     ,
            PWM_CLK_DIV4 = 0x02     ,
            PWM_CLK_DIV8 = 0x03     ,
            PWM_CLK_DIV16 = 0x04    ,
            PWM_CLK_DIV32 = 0x05    ,
            PWM_CLK_DIV64 = 0x06    ,
            PWM_CLK_DIV128 = 0x07   ,
            PWM_CLK_DIV256 = 0x08   ,
            PWM_CLK_DIV512 = 0x09   ,
            PWM_CLK_DIV1024 = 0x0A  ,
            PWM_CLK_DIV2048 = 0x0B  ,
            PWM_CLK_DIV4096 = 0x0C  ,
            PWM_CLK_DIV8192 = 0x0D  ,
            PWM_CLK_DIV16384 = 0x0E ,
            PWM_CLK_DIV32768 = 0x0F ,

            TPCR0 = 0x70               ,
            TPCR0_ENABLE = 0x80        ,
            TPCR0_DISABLE = 0x00       ,
            TPCR0_WAIT_512CLK = 0x00   ,
            TPCR0_WAIT_1024CLK = 0x10  ,
            TPCR0_WAIT_2048CLK = 0x20  ,
            TPCR0_WAIT_4096CLK = 0x30  ,
            TPCR0_WAIT_8192CLK = 0x40  ,
            TPCR0_WAIT_16384CLK = 0x50 ,
            TPCR0_WAIT_32768CLK = 0x60 ,
            TPCR0_WAIT_65536CLK = 0x70 ,
            TPCR0_WAKEENABLE = 0x08    ,
            TPCR0_WAKEDISABLE = 0x00   ,
            TPCR0_ADCCLK_DIV1 = 0x00   ,
            TPCR0_ADCCLK_DIV2 = 0x01   ,
            TPCR0_ADCCLK_DIV4 = 0x02   ,
            TPCR0_ADCCLK_DIV8 = 0x03   ,
            TPCR0_ADCCLK_DIV16 = 0x04  ,
            TPCR0_ADCCLK_DIV32 = 0x05  ,
            TPCR0_ADCCLK_DIV64 = 0x06  ,
            TPCR0_ADCCLK_DIV128 = 0x07 ,

            TPCR1 = 0x71            ,
            TPCR1_AUTO = 0x00       ,
            TPCR1_MANUAL = 0x40     ,
            TPCR1_VREFINT = 0x00    ,
            TPCR1_VREFEXT = 0x20    ,
            TPCR1_DEBOUNCE = 0x04   ,
            TPCR1_NODEBOUNCE = 0x00 ,
            TPCR1_IDLE = 0x00       ,
            TPCR1_WAIT = 0x01       ,
            TPCR1_LATCHX = 0x02     ,
            TPCR1_LATCHY = 0x03     ,

            TPXH = 0x72  ,
            TPYH = 0x73  ,
            TPXYL = 0x74 ,

            INTC1 = 0xF0     ,
            INTC1_KEY = 0x10 ,
            INTC1_DMA = 0x08 ,
            INTC1_TP = 0x04  ,
            INTC1_BTE = 0x02 ,

            INTC2 = 0xF1     ,
            INTC2_KEY = 0x10 ,
            INTC2_DMA = 0x08 ,
            INTC2_TP = 0x04  ,
            INTC2_BTE = 0x02 ,

            SCROLL_BOTH = 0x00   ,
            SCROLL_LAYER1 = 0x40 ,
            SCROLL_LAYER2 = 0x80 ,
            SCROLL_BUFFER = 0xC0 
        };
        bool m_initialized;
        spi_device m_spi;
        inline static spi_device_interface_config_t get_spi_config() {
            spi_device_interface_config_t spi_cfg;
            memset(&spi_cfg, 0, sizeof(spi_cfg));
            spi_cfg.spics_io_num = pin_cs;
            spi_cfg.clock_speed_hz = 20*1000*1000;
            spi_cfg.mode = 0;
            spi_cfg.queue_size = max_transactions;
            spi_cfg.flags = SPI_DEVICE_NO_DUMMY;    
            return spi_cfg;        
        }
        static result xlt_err(spi_result r) {
            switch(r) {
                case spi_result::success:
                    return result::success;
                case spi_result::timeout:
                    return result::timeout;
                case spi_result::previous_transactions_pending:
                    return result::busy;
                case spi_result::invalid_argument:
                    return result::invalid_argument;
                case spi_result::out_of_memory:
                    return result::out_of_memory;
                default:
                    return result::io_error;
            }
        }
        void read_data(uint8_t* read_buf, uint32_t read_len) {
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));  //Zero out the transaction

            /* required since read_buf is unlikely aligned to 32bit carrier */
            uint8_t* tmp_read = (uint8_t*)malloc(read_len);
            if (tmp_read == NULL) {
                puts("SPI: Couldn't malloc");
                return;
            }
            t.length = 8 * read_len;  //length is in bits.
            //t.flags = SPI_TRANS_USE_TXDATA;
            t.tx_buffer = NULL;		  // Pointer to transmit buffer, or NULL for no MOSI phase
            t.rx_buffer = tmp_read;
            t.rxlength = 8*read_len;							//0 forces it to be the same as 'length'
            t.user = (void*)0;						//context or random pointer. Perhaps use for callback function if used for async?
            esp_err_t ret = spi_device_transmit(m_spi.handle(), &t);	//blocking
            memcpy(read_buf, tmp_read, read_len);
            free(tmp_read);
            return;
        }
        inline static void delay(int ms) {
            vTaskDelay(ms/portTICK_PERIOD_MS);
        }
        result init_pll() {
            result r;
            if (width == 480) {
                r=set_register(PLLC1, PLLC1_PLLDIV1 + 10);
                if(result::success!=r) {
                    return r;
                }
                delay(1);
                r=set_register(PLLC2, PLLC2_DIV4);
                if(result::success!=r) {
                    return r;
                }
                delay(1);
            } else if(width==800) {
                r=set_register(PLLC1, PLLC1_PLLDIV1 + 11);
                if(result::success!=r) {
                    return r;
                }
                delay(1);
                r=set_register(PLLC2, PLLC2_DIV4);
                if(result::success!=r) {
                    return r;
                }
                delay(1);
            } else {
                return result::invalid_argument;
            }
            return result::success;
        }
        result send(uint8_t type,uint8_t value) {
            // TODO: use a 16-bit word here instead
            uint8_t data[2];
            data[0]=type;
            data[1]=value;
            spi_result r = m_spi.write(data,2);
            if(spi_result::success!=r)
                return xlt_err(r);
            return result::success;
        }
        result retr(uint8_t type,uint8_t* out_value) {
            if(nullptr==out_value)
                return result::invalid_argument;
            // TODO: use a 16-bit word here instead
            spi_transaction_t tran;
            uint8_t data[2];
            data[0]=type;
            data[1]=0;
            spi_device::make_read_write(&tran,data,2,nullptr,2);
            spi_result r = m_spi.transaction(&tran,use_polling);
            if(spi_result::success!=r)
                return xlt_err(r);
            if(tran.rxlength>0) {
                printf("0x%04X\r\n",(unsigned int)tran.rx_buffer);
                *out_value = tran.rx_data[0];
            }
            return result::success;
        }
        inline result get_status(uint8_t* out_value) {
            return retr(CMDREAD,out_value);
        }
        inline result send_command(uint8_t cmd) {
            return send(CMDWRITE,cmd);
        }
        inline result send_data(uint8_t data) {
            return send(DATAWRITE,data);
        }
        result set_register(uint8_t reg,uint8_t data) {
            result r = send_command(reg);
            if(result::success!=r) {
                return r;
            }
            return send_data(data);
        }
        result get_register(uint8_t reg,uint8_t* out_data) {
            if(nullptr==out_data) {
                return result::invalid_argument;
            }
            result r = send_command(reg);
            if(result::success!=r) {
                return r;
            }
            return retr(DATAREAD,out_data);
        }
    public:
        ra8875() : m_initialized(false), m_spi(host_id,get_spi_config()) {

        }
        bool initialized() const {
            return m_initialized;
        }
        result initialize() {
            if(!m_initialized) {
                gpio_config_t pc;
                
                pc.pin_bit_mask = (1<<(int)pin_backlight) | (1<<(int)pin_cs) | (GPIO_NUM_NC!=pin_rst?(1<<(int)pin_rst):0);
                pc.mode=GPIO_MODE_OUTPUT;
                pc.intr_type = GPIO_INTR_DISABLE;
                pc.pull_down_en = GPIO_PULLDOWN_DISABLE;
                pc.pull_up_en = GPIO_PULLUP_DISABLE;
                if(ESP_OK!=gpio_config(&pc)) {
                    return result::io_error;
                }
                if(ESP_OK!=gpio_set_level(pin_cs,1)) {
                    return result::io_error;
                }
                if(ESP_OK!=gpio_set_level(pin_rst,1)) { 
                    return result::io_error;
                }
                if(ESP_OK!=gpio_set_level(pin_rst,0)) {
                    return result::io_error;
                }
                delay(100);
                if(ESP_OK!=gpio_set_level(pin_rst, 1)) {
                    return result::io_error;
                }
                delay(100);
                uint8_t reg=0;
                result r = send_command(0);
                if(result::success!=r)
                    return r;
                //,&reg);
                read_data(&reg,1);
                if (0x75!=reg) {
                    printf("register 0 value was: %02X\r\n",reg);
                    return result::device_not_detected;
                }
                gpio_set_level(pin_backlight,1);;
                m_initialized = true;
            }
            return result::success;
        }
    };
}