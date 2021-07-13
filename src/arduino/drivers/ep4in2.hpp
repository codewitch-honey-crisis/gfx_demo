#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "gfx_bitmap.hpp"
// unlike most of the other drivers, which simply expose 
// GFX binding which may be removed, this driver 
// requires GFX in order to function due to the 
// complexity of indexed pixel translation involved
// there was no sense in duplicating the code for
// bit mangling.
namespace arduino {
    namespace ep4in2_helpers {
        static const unsigned char lut_vcom0[] PROGMEM = {
            44, // count
            0x00, 0x17, 0x00, 0x00, 0x00, 0x02,
            0x00, 0x17, 0x17, 0x00, 0x00, 0x02,
            0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,
            0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        static const unsigned char lut_ww[] PROGMEM = {
            42, // count
            0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
            0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
            0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
            0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        static const unsigned char lut_bw[] PROGMEM = {
            42, // count
            0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
            0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
            0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
            0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        static const unsigned char lut_wb[] PROGMEM = {
            42, // count
            0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
            0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
            0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
            0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        static const unsigned char lut_bb[] PROGMEM = {
            42, // count
            0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
            0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
            0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
            0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };

        /******************************partial screen update LUT*********************************/
        const unsigned char partial_lut_vcom1[] PROGMEM = {
        44, // count
        0x00	,0x19	,0x01	,0x00	,0x00	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00,	};

        const unsigned char partial_lut_ww1[] PROGMEM = {
        42, // count
        0x00	,0x19	,0x01	,0x00	,0x00	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,};

        const unsigned char partial_lut_bw1[] PROGMEM = {
        42, // count
        0x80	,0x19	,0x01	,0x00	,0x00	,0x01,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	};

        const unsigned char partial_lut_wb1[] PROGMEM = {
        42, // count
        0x40	,0x19	,0x01	,0x00	,0x00	,0x01,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	};

        const unsigned char partial_lut_bb1[] PROGMEM = {
        42, // count
        0x00	,0x19	,0x01	,0x00	,0x00	,0x01,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,	};

        /******************************gray*********************************/
        //0~3 gray
        const unsigned char gray4_lut_vcom[] PROGMEM = {
        42, // count
        0x00	,0x0A	,0x00	,0x00	,0x00	,0x01,
        0x60	,0x14	,0x14	,0x00	,0x00	,0x01,
        0x00	,0x14	,0x00	,0x00	,0x00	,0x01,
        0x00	,0x13	,0x0A	,0x01	,0x00	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00
                        
        };
        //R21
        const unsigned char gray4_lut_ww[] PROGMEM = {
        42, // count
        0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
        0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
        0x10	,0x14	,0x0A	,0x00	,0x00	,0x01,
        0xA0	,0x13	,0x01	,0x00	,0x00	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        };
        //R22H	r
        const unsigned char gray4_lut_bw[] PROGMEM = {
        42, // count
        0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
        0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
        0x00	,0x14	,0x0A	,0x00	,0x00	,0x01,
        0x99	,0x0C	,0x01	,0x03	,0x04	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        };
        //R23H	w
        const unsigned char gray4_lut_wb[] PROGMEM = {
        42, // count
        0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
        0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
        0x00	,0x14	,0x0A	,0x00	,0x00	,0x01,
        0x99	,0x0B	,0x04	,0x04	,0x01	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        };
        //R24H	b
        const unsigned char gray4_lut_bb[] PROGMEM = {
        42, // count
        0x80	,0x0A	,0x00	,0x00	,0x00	,0x01,
        0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
        0x20	,0x14	,0x0A	,0x00	,0x00	,0x01,
        0x50	,0x13	,0x01	,0x00	,0x00	,0x01,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
        };
    }
    template<int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy>
    struct ep4in2;
    template<typename DriverType> struct ep4in2_mode_base {
        template<int8_t PinCS,
            int8_t PinDC,
            int8_t PinRst,
            int8_t PinBusy>
        friend class ep4in2;
        constexpr static const uint16_t width=400;
        constexpr static const uint16_t height=300;
        
    protected:
        virtual void invalidate()=0;
        static bool normalize_values(gfx::rect16& r,bool check_bounds=true) {
            // normalize values
            uint16_t tmp;
            if(r.x1>r.x2) {
                tmp=r.x1;
                r.x1=r.x2;
                r.x2=tmp;
            }
            if(r.y1>r.y2) {
                tmp=r.y1;
                r.y1=r.y2;
                r.y2=tmp;
            }
            if(check_bounds) {
                if(r.x1>=width||r.y1>=height)
                    return false;
                if(r.x2>=width)
                    r.x2=width-1;
                if(r.y2>height)
                    r.y2=height-1;
            }
            return true;
        }
        static void send_command(SPIClass& spi,uint8_t cmd) {
            spi.beginTransaction(SPISettings(DriverType::clock_speed,MSBFIRST,SPI_MODE0));
            digitalWrite(DriverType::pin_dc,LOW);
            digitalWrite(DriverType::pin_cs,LOW);
            spi.transfer(cmd);
            digitalWrite(DriverType::pin_dc,HIGH);
            digitalWrite(DriverType::pin_cs,HIGH);
            spi.endTransaction();
        }
        static void send_data(SPIClass& spi,uint8_t data) {
            spi.beginTransaction(SPISettings(DriverType::clock_speed,MSBFIRST,SPI_MODE0));
            digitalWrite(DriverType::pin_cs,LOW);
            spi.transfer(data);
            digitalWrite(DriverType::pin_cs,HIGH);
            spi.endTransaction();
        }
        static void wait_busy(SPIClass& spi) {
            uint32_t start = millis();
            send_command(spi,0x71);
            while(LOW==digitalRead(DriverType::pin_busy) && millis()-start<DriverType::timeout) {
                send_command(spi,0x71);
                delay(100);
            }
        }
        static void send_lut_data(SPIClass& spi,const uint8_t* data) {
            size_t count = *data++;
            while(count-->0) {
                send_data(spi,*data++);
            }
        }
        static void do_reset() {
            digitalWrite(DriverType::pin_rst, HIGH);
            delay(20);
            digitalWrite(DriverType::pin_rst, LOW);
            delay(2);
            digitalWrite(DriverType::pin_rst, HIGH);
            delay(20);
            digitalWrite(DriverType::pin_rst, LOW);
            delay(2);
            digitalWrite(DriverType::pin_rst, HIGH);
            delay(20);
            digitalWrite(DriverType::pin_rst, LOW);
            delay(2);
            digitalWrite(DriverType::pin_rst, HIGH);
            delay(20);
        }
        virtual ~ep4in2_mode_base() {

        }
    public:
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        inline constexpr gfx::size16 dimensions() const {
            return gfx::size16(width,height);
        }
        inline constexpr gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
    };
    template<typename DriverType,size_t BitDepth,size_t VirtualBitDepth>  
    struct ep4in2_mode final {
    };
    template<typename DriverType,size_t VirtualBitDepth>
    struct ep4in2_mode<DriverType,1,VirtualBitDepth> : public ep4in2_mode_base<DriverType> {
        using type = ep4in2_mode;
        using base_type = ep4in2_mode_base<DriverType>;
        using driver_type = DriverType;
        using pixel_type = gfx::gsc_pixel<VirtualBitDepth>;
        using native_pixel_type = gfx::gsc_pixel<1>;
        using frame_buffer_type = gfx::large_bitmap<pixel_type>;
    private:
        SPIClass& m_spi;
        frame_buffer_type m_frame_buffer;
        bool m_initialized;
        void(*m_deinitialize_self)(DriverType*);
        unsigned int m_suspend_x1;
        unsigned int m_suspend_y1;
        unsigned int m_suspend_x2;
        unsigned int m_suspend_y2;
        unsigned int m_suspend_count;
        unsigned int m_suspend_first;
        bool m_wash;
        void initialize() {
            if(!m_initialized) {
                base_type::do_reset();
                base_type::send_command(m_spi,0x01); // POWER SETTING
                base_type::send_data(m_spi,0x03);
                base_type::send_data(m_spi,0x00);
                base_type::send_data(m_spi,0x2b);
                base_type::send_data(m_spi,0x2b);

                base_type::send_command(m_spi,0x06); // boost soft start
                base_type::send_data(m_spi,0x17);		//A
                base_type::send_data(m_spi,0x17);		//B
                base_type::send_data(m_spi,0x17);		//C

                base_type::send_command(m_spi,0x04); // POWER_ON
                base_type::wait_busy(m_spi);

                base_type::send_command(m_spi,0x00); // panel setting
                base_type::send_data(m_spi,0xbf); // KW-BF   KWR-AF	BWROTP 0f	BWOTP 1f
                base_type::send_data(m_spi,0x0d);

                base_type::send_command(m_spi,0x30); // PLL setting
                base_type::send_data(m_spi,0x3C); // 3A 100HZ   29 150Hz 39 200HZ	31 171HZ

                base_type::send_command(m_spi,0x61); // resolution setting
                base_type::send_data(m_spi,0x01);
                base_type::send_data(m_spi,0x90); //128
                base_type::send_data(m_spi,0x01); //
                base_type::send_data(m_spi,0x2c);

                base_type::send_command(m_spi,0x82); // vcom_DC setting
                base_type::send_data(m_spi,0x28);

                base_type::send_command(m_spi,0x50); // VCOM AND DATA INTERVAL SETTING
                base_type::send_data(m_spi,0x97); // 97white border 77black border		VBDF 17|D7 VBDW 97 VBDB 57		VBDF F7 VBDW 77 VBDB 37  VBDR B7

                base_type::send_command(m_spi,0x20);         //g vcom
                base_type::send_lut_data(m_spi,ep4in2_helpers::lut_vcom0);
                
                base_type::send_command(m_spi,0x21);
                base_type::send_lut_data(m_spi,ep4in2_helpers::lut_ww);
                
                base_type::send_command(m_spi,0x22);
                base_type::send_lut_data(m_spi,ep4in2_helpers::lut_bw);
                
                base_type::send_command(m_spi,0x23);
                base_type::send_lut_data(m_spi,ep4in2_helpers::lut_wb);
                
                base_type::send_command(m_spi,0x24);
                base_type::send_lut_data(m_spi,ep4in2_helpers::lut_bb);
                m_initialized = true;

                
            }
        }
        gfx::gfx_result wash_display() {
            initialize();
            base_type::send_command(m_spi,0x10);				 //writes old data to SRAM.
            for (int y = 0; y < base_type::height; ++y) {
                for (int x = 0; x <base_type::width; x+=8) {    
                    base_type::send_data(m_spi, 0x00);
                }
            }

            base_type::send_command(m_spi,0x13);				 //writes New data to SRAM.
            for (int y = 0; y < base_type::height; ++y) {
                for (int x = 0; x <base_type::width; x+=8) {    
                    base_type::send_data(m_spi, 0xFF);
                }
            }

            base_type::send_command(m_spi,0x12);
            delay(100);
            base_type::wait_busy(m_spi);
            return gfx::gfx_result::success;
        }
        gfx::gfx_result display_update_full() {
            if(0==m_suspend_count) {
                initialize();
                int col, row;
                base_type::send_command(m_spi,0x10);				 //writes old data to SRAM.
                for (int y = 0; y < base_type::height; y++) {
                    row = y & 15;
                    for (int x = 0; x < base_type::width ; x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i;
                            col = xx & 15;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            data |= (1<<(7-i))* (255.0*px.template channelr<gfx::channel_name::L>()>=gfx::helpers::dither_bayer_16[col][row]);
                        }
                        base_type::send_data(m_spi,~data);
                    }
                }
                base_type::send_command(m_spi,0x13);				 //writes New data to SRAM.
                for (int y = 0; y < base_type::height; y++) {
                    row = y & 15;
                    for (int x = 0; x < base_type::width ; x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i;
                            col = xx & 15;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            data |= (1<<(7-i))* (255.0*px.template channelr<gfx::channel_name::L>()>=gfx::helpers::dither_bayer_16[col][row]);
                        }
                        base_type::send_data(m_spi,data);
                    }
                }
            
                base_type::send_command(m_spi,0x12);
                delay(100);
                base_type::wait_busy(m_spi);
                
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result display_update() {
            
            return display_update_partial();
        }
        gfx::gfx_result display_update_partial() {
            if(0==m_suspend_count) {
                if(!m_initialized) {
                    return display_update_full();
                }
                if(m_wash) {
                    wash_display();
                    m_wash =false;
                }
                base_type::send_command(m_spi, 0x50);
                base_type::send_data(m_spi, 0xf7);
                delay(100);
                
                base_type::send_command(m_spi, 0x82);			//vcom_DC setting  	
                base_type::send_data(m_spi,0x08);	
                base_type::send_command(m_spi, 0X50);
                base_type::send_data(m_spi, 0x47);		
                
                base_type::send_command(m_spi, 0x20);
                base_type::send_lut_data(m_spi, ep4in2_helpers::partial_lut_vcom1);
                
                base_type::send_command(m_spi, 0x21);
                base_type::send_lut_data(m_spi, ep4in2_helpers::partial_lut_ww1);
                
                base_type::send_command(m_spi, 0x22);
                base_type::send_lut_data(m_spi, ep4in2_helpers::partial_lut_bw1);
                
                base_type::send_command(m_spi, 0x23);
                base_type::send_lut_data(m_spi, ep4in2_helpers::partial_lut_wb1);
                
                base_type::send_command(m_spi, 0x24);
                base_type::send_lut_data(m_spi, ep4in2_helpers::partial_lut_bb1);
                
                base_type::send_command(m_spi, 0x91);		//This command makes the display enter partial mode
                base_type::send_command(m_spi, 0x90);		//resolution setting
                //Serial.printf("(%d, %d)-(%d, %d)\r\n",m_suspend_x1,m_suspend_y1,m_suspend_x2,m_suspend_y2);
                int x1 = m_suspend_x1/8*8,
                    y1 = m_suspend_y1,
                    x2 = m_suspend_x2 = (m_suspend_x2 % 8 == 0)? m_suspend_x2:m_suspend_x2/8*8+8,
                    y2 = m_suspend_y2;
                
                base_type::send_data(m_spi, x1/256);
                base_type::send_data(m_spi, x1%256);   //x-start    

                base_type::send_data(m_spi, x2/256);		
                base_type::send_data(m_spi, x2%256);  //x-end

                base_type::send_data(m_spi, y1/256);
                base_type::send_data(m_spi, y1%256);   //y-start    

                base_type::send_data(m_spi, y2/256);		
                base_type::send_data(m_spi, y2%256);  //y-end
                base_type::send_data(m_spi, 0x28);	

                int col, row;
                base_type::send_command(m_spi, 0x10);	       //writes Old data to SRAM for programming
                for (int y = 0; y <= y2 - y1; ++y) {
                    row = (y+y1) & 15;
                    for (int x = 0; x <= x2-x1; x+=8) {    
                        uint8_t data=0;                    
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i+x1;
                            col = xx & 15;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            data |= (1<<(7-i))* (255.0*px.template channelr<gfx::channel_name::L>()>=gfx::helpers::dither_bayer_16[col][row]);
                        }
                        base_type::send_data(m_spi, data);
                    }
                }

                base_type::send_command(m_spi, 0x13);	       //writes new data to SRAM
                for (int y = 0; y <= y2 - y1; ++y) {
                    row = (y+y1) & 15;
                    for (int x = 0; x <= x2-x1; x+=8) {    
                        uint8_t data=0;                    
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i+x1;
                            col = xx & 15;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            data |= (1<<(7-i))* (255.0*px.template channelr<gfx::channel_name::L>()>=gfx::helpers::dither_bayer_16[col][row]);
                        }
                        base_type::send_data(m_spi, ~data);
                    }
                }
                
                base_type::send_command(m_spi,0x12);		 //DISPLAY REFRESH 		             
                delay(20);
                base_type::send_command(m_spi,0x12);
                delay(100);
                base_type::wait_busy(m_spi);
            }
            return gfx::gfx_result::success;
        }
    protected:
        driver_type* m_driver;
        virtual void invalidate() {
            m_frame_buffer = frame_buffer_type();
        }
    public:
        ep4in2_mode(driver_type* driver,void(*deinitialize_self)(driver_type*),SPIClass& spi,void*(*frame_buffer_allocator)(size_t)=malloc,void(*frame_buffer_deallocator)(void*)=free) : 
            m_spi(spi),
            m_frame_buffer(this->dimensions(),1,nullptr,frame_buffer_allocator,frame_buffer_deallocator),
            m_initialized(false),
            m_deinitialize_self(deinitialize_self),
            m_suspend_x1(0),
            m_suspend_y1(0),
            m_suspend_x2(base_type::width-1),
            m_suspend_y2(base_type::height-1),
            m_suspend_count(0),
            m_suspend_first(0),
            m_wash(false),
            m_driver(driver) {
        }
        ep4in2_mode(ep4in2_mode&& rhs) : 
            m_spi(rhs.m_spi),
            m_initialized(false),
            m_deinitialize_self(rhs.m_deinitialize_self),
            m_suspend_x1(0),
            m_suspend_y1(0),
            m_suspend_x2(base_type::width-1),
            m_suspend_y2(base_type::height-1),
            m_suspend_count(0),
            m_suspend_first(0),
            m_wash(false),
            m_driver(rhs.m_driver) {
                rhs.m_frame_buffer = frame_buffer_type();
        }
        ep4in2_mode& operator=(ep4in2_mode&& rhs) {
            m_spi=rhs.m_spi;
            m_initialized=false;
            m_deinitialize_self=rhs.m_deinitialize_self;
            m_suspend_x1(0);
            m_suspend_y1(0);
            m_suspend_x2(base_type::width-1);
            m_suspend_y2(base_type::height-1);
            m_suspend_count=0;
            m_suspend_first=0;
            m_wash=false;
            m_driver=rhs.m_driver;
            rhs.m_frame_buffer = frame_buffer_type();
            return *this;
        }
        virtual ~ep4in2_mode() {
            m_deinitialize_self(m_driver);
        }
        gfx::gfx_result wash() {
            m_wash = true;
            return display_update_full();
        }
        gfx::gfx_result point(const gfx::point16& location,pixel_type color) {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            if(location.x>=this->width||location.y>=this->height) {
                return gfx::gfx_result::success;
            }
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = location.x;
                    m_suspend_y1 = location.y;
                    m_suspend_x2 = location.x;
                    m_suspend_y2 = location.y;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>location.x)
                        m_suspend_x1=location.x;
                    if(m_suspend_y1>location.y)
                        m_suspend_y1=location.y;
                    if(m_suspend_x2<location.x)
                        m_suspend_x2=location.x;
                    if(m_suspend_y2<location.y)
                        m_suspend_y2=location.y;
                }
            }
            gfx::gfx_result r= m_frame_buffer.point(location,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            return display_update();
        }
        gfx::gfx_result point(const gfx::point16& location,pixel_type* out_color) const {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            return m_frame_buffer.point(location,out_color);
        }
        gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type px;
            px.native_value = 0;
            return fill(bounds,px);
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            gfx::rect16 r = bounds;
            if(!base_type::normalize_values(r,true)) {
                return gfx::gfx_result::success;
            }
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = r.x1;
                    m_suspend_y1 = r.y1;
                    m_suspend_x2 = r.x2;
                    m_suspend_y2 = r.y2;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>r.x1)
                        m_suspend_x1=r.x1;
                    if(m_suspend_y1>r.y1)
                        m_suspend_y1=r.y1;
                    if(m_suspend_x2<r.x2)
                        m_suspend_x2=r.x2;
                    if(m_suspend_y2<r.y2)
                        m_suspend_y2=r.y2;
                }
            }
            gfx::gfx_result rr=m_frame_buffer.fill(r,color);
            if(gfx::gfx_result::success!=rr) {
                return rr;
            }
            return display_update();
        }
        gfx::gfx_result suspend() {
            m_suspend_first=(m_suspend_count==0);
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    return display_update();
                }
                
            } 
            return gfx::gfx_result::success;
        }
    };

    template<typename DriverType,size_t VirtualBitDepth>
    struct ep4in2_mode<DriverType,2,VirtualBitDepth> : public ep4in2_mode_base<DriverType> {
        static_assert(VirtualBitDepth>1,"VirtualBitDepth must be at least 2.");
        using type = ep4in2_mode;
        using base_type = ep4in2_mode_base<DriverType>;
        using driver_type = DriverType;
        using pixel_type = gfx::gsc_pixel<VirtualBitDepth>;
        using native_pixel_type = gfx::gsc_pixel<2>;
        using frame_buffer_type = gfx::large_bitmap<pixel_type>;
    private:
        SPIClass& m_spi;
        frame_buffer_type m_frame_buffer;
        bool m_initialized;
        void(*m_deinitialize_self)(DriverType*);
        unsigned int m_suspend_x1;
        unsigned int m_suspend_y1;
        unsigned int m_suspend_x2;
        unsigned int m_suspend_y2;
        unsigned int m_suspend_count;
        unsigned int m_suspend_first;
        void initialize() {
            if(!m_initialized) {
                base_type::do_reset();
                base_type::send_command(m_spi, 0x01);			//POWER SETTING
                base_type::send_data(m_spi, 0x03);
                base_type::send_data(m_spi, 0x00);       //VGH=20V,VGL=-20V
                base_type::send_data(m_spi, 0x2b);		//VDH=15V															 
                base_type::send_data(m_spi, 0x2b);		//VDL=-15V
                base_type::send_data(m_spi, 0x13);

                base_type::send_command(m_spi, 0x06);         //booster soft start
                base_type::send_data(m_spi, 0x17);		//A
                base_type::send_data(m_spi, 0x17);		//B
                base_type::send_data(m_spi, 0x17);		//C 

                base_type::send_command(m_spi, 0x04);
                base_type::wait_busy(m_spi);

                base_type::send_command(m_spi, 0x00);			//panel setting
                base_type::send_data(m_spi, 0x3f);		//KW-3f   KWR-2F	BWROTP 0f	BWOTP 1f

                base_type::send_command(m_spi, 0x30);			//PLL setting
                base_type::send_data(m_spi, 0x3c);      	//100hz 

                base_type::send_command(m_spi, 0x61);			//resolution setting
                base_type::send_data(m_spi, 0x01);		//400
                base_type::send_data(m_spi, 0x90);     	 
                base_type::send_data(m_spi, 0x01);		//300
                base_type::send_data(m_spi, 0x2c);

                base_type::send_command(m_spi, 0x82);			//vcom_DC setting
                base_type::send_data(m_spi, 0x12);

                base_type::send_command(m_spi, 0X50);			//VCOM AND DATA INTERVAL SETTING			
                base_type::send_data(m_spi, 0x97);
                m_initialized = true;

                
            }
        }
        
        gfx::gfx_result display_update_full() {
            if(0==m_suspend_count) {
                initialize();
                base_type::send_command(m_spi,0x10);				 //writes old data to SRAM.
                for (int y = 0; y < base_type::height; y++) {
                    for (int x = 0; x < base_type::width ; x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            auto ii = px.template channel<gfx::channel_name::L>();
                            data |= (1<<(7-i))*(ii==0||ii==1);
                            
                        }
                        base_type::send_data(m_spi,~data);
                    }
                }
                base_type::send_command(m_spi,0x13);				 //writes New data to SRAM.
                for (int y = 0; y < base_type::height; y++) {
                    for (int x = 0; x < base_type::width ; x+=8) {
                        uint8_t data = 0;
                        for(int i = 0;i<8;++i) {
                            const int xx = x+i;
                            pixel_type px;
                            gfx::gfx_result r=m_frame_buffer.point(gfx::point16(xx,y),&px);
                            if(gfx::gfx_result::success!=r) {
                                return r;
                            }
                            auto ii = px.template channel<gfx::channel_name::L>();
                            data |= (1<<(7-i))* (ii==3||ii==3);
                            
                        }
                        base_type::send_data(m_spi,data);
                    }
                }
                base_type::send_command(m_spi,0x20);							//vcom
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_vcom);
                
                base_type::send_command(m_spi,0x21);							//red (not used)
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_ww);

                base_type::send_command(m_spi,0x22);							// bw r
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_bw);

                base_type::send_command(m_spi,0x23);							// wb w
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_wb);
                    
                base_type::send_command(m_spi,0x24);							// bb b
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_bb);
                
                base_type::send_command(m_spi,0x25);							// vcom
                base_type::send_lut_data(m_spi,ep4in2_helpers::gray4_lut_ww);
                
                base_type::send_command(m_spi,0x12);
                delay(100);
                base_type::wait_busy(m_spi);
                
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result display_update() {
            
            return display_update_full();
        }
        
    protected:
        driver_type* m_driver;
        virtual void invalidate() {
            m_frame_buffer = frame_buffer_type();
        }
    public:
        ep4in2_mode(driver_type* driver,void(*deinitialize_self)(driver_type*),SPIClass& spi,void*(*frame_buffer_allocator)(size_t)=malloc,void(*frame_buffer_deallocator)(void*)=free) : 
            m_spi(spi),
            m_frame_buffer(this->dimensions(),1,nullptr,frame_buffer_allocator,frame_buffer_deallocator),
            m_initialized(false),
            m_deinitialize_self(deinitialize_self),
            m_suspend_x1(0),
            m_suspend_y1(0),
            m_suspend_x2(base_type::width-1),
            m_suspend_y2(base_type::height-1),
            m_suspend_count(0),
            m_suspend_first(0),
            m_driver(driver) {
        }
        ep4in2_mode(ep4in2_mode&& rhs) : 
            m_spi(rhs.m_spi),
            m_initialized(false),
            m_deinitialize_self(rhs.m_deinitialize_self),
            m_suspend_x1(0),
            m_suspend_y1(0),
            m_suspend_x2(base_type::width-1),
            m_suspend_y2(base_type::height-1),
            m_suspend_count(0),
            m_suspend_first(0),
            m_driver(rhs.m_driver) {
                rhs.m_frame_buffer = frame_buffer_type();
        }
        ep4in2_mode& operator=(ep4in2_mode&& rhs) {
            m_spi=rhs.m_spi;
            m_initialized=false;
            m_deinitialize_self=rhs.m_deinitialize_self;
            m_suspend_x1(0);
            m_suspend_y1(0);
            m_suspend_x2(base_type::width-1);
            m_suspend_y2(base_type::height-1);
            m_suspend_count=0;
            m_suspend_first=0;
            m_driver=rhs.m_driver;
            rhs.m_frame_buffer = frame_buffer_type();
            return *this;
        }
        virtual ~ep4in2_mode() {
            m_deinitialize_self(m_driver);
        }
        gfx::gfx_result point(const gfx::point16& location,pixel_type color) {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            if(location.x>=this->width||location.y>=this->height) {
                return gfx::gfx_result::success;
            }
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = location.x;
                    m_suspend_y1 = location.y;
                    m_suspend_x2 = location.x;
                    m_suspend_y2 = location.y;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>location.x)
                        m_suspend_x1=location.x;
                    if(m_suspend_y1>location.y)
                        m_suspend_y1=location.y;
                    if(m_suspend_x2<location.x)
                        m_suspend_x2=location.x;
                    if(m_suspend_y2<location.y)
                        m_suspend_y2=location.y;
                }
            }
            gfx::gfx_result r= m_frame_buffer.point(location,color);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            return display_update();
        }
        gfx::gfx_result point(const gfx::point16& location,pixel_type* out_color) const {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            return m_frame_buffer.point(location,out_color);
        }
        gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type px;
            px.native_value = 0;
            return fill(bounds,px);
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            gfx::rect16 r = bounds;
            if(!base_type::normalize_values(r,true)) {
                return gfx::gfx_result::success;
            }
            if(0!=m_suspend_count) {
                if(0!=m_suspend_first) {
                    m_suspend_first = 0;
                    m_suspend_x1 = r.x1;
                    m_suspend_y1 = r.y1;
                    m_suspend_x2 = r.x2;
                    m_suspend_y2 = r.y2;
                } else {
                    // if we're suspended update the suspended extents
                    if(m_suspend_x1>r.x1)
                        m_suspend_x1=r.x1;
                    if(m_suspend_y1>r.y1)
                        m_suspend_y1=r.y1;
                    if(m_suspend_x2<r.x2)
                        m_suspend_x2=r.x2;
                    if(m_suspend_y2<r.y2)
                        m_suspend_y2=r.y2;
                }
            }
            gfx::gfx_result rr=m_frame_buffer.fill(r,color);
            if(gfx::gfx_result::success!=rr) {
                return rr;
            }
            return display_update();
        }
        gfx::gfx_result suspend() {
            m_suspend_first=(m_suspend_count==0);
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            if(0!=m_suspend_count) {
                --m_suspend_count;
                if(force)
                    m_suspend_count = 0;
                if(0==m_suspend_count) {
                    return display_update();
                }
                
            } 
            return gfx::gfx_result::success;
        }
    };
    
    template<int8_t PinCS,
        int8_t PinDC,
        int8_t PinRst,
        int8_t PinBusy>
    struct ep4in2 final {
        
        enum struct result {
            success = 0,
            invalid_argument,
            io_error,
            io_busy,
            out_of_memory,
            timeout,
            not_supported
        };
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        // the RST pin
        constexpr static const int8_t pin_rst = PinRst;
        // the Busy pin
        constexpr static const int8_t pin_busy = PinBusy;

        constexpr static const uint32_t clock_speed = 4*1000*1000;
        constexpr static const uint32_t timeout = 5000;
    private: 
        
        SPIClass& m_spi;
        ep4in2_mode_base<ep4in2>* m_previous_mode;
        static void deinitialize_mode(ep4in2* this_) {
            if(nullptr!=this_->m_previous_mode) {
                this_->m_previous_mode->invalidate();
                this_->m_previous_mode = nullptr;
            }
        }
    public:
        ep4in2(SPIClass& spi) : m_spi(spi),m_previous_mode(nullptr) {
            pinMode(pin_cs,OUTPUT);
            pinMode(pin_dc,OUTPUT);
            pinMode(pin_rst,OUTPUT);
            pinMode(pin_busy,INPUT);
        }

        template<size_t NativeBitDepth,size_t VirtualBitDepth = NativeBitDepth>
        ep4in2_mode<ep4in2,NativeBitDepth,VirtualBitDepth> mode() {
            static_assert(NativeBitDepth==1||NativeBitDepth==2,"The supported native bit depths are 1 and 2.");
            return ep4in2_mode<ep4in2,NativeBitDepth,VirtualBitDepth>(this,deinitialize_mode,m_spi);
        }
    };
       
}