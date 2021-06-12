#pragma once
#include <Arduino.h>
#include <SPI.h>
namespace arduino {
    enum struct spi_driver_result {
        success = 0,
        invalid_argument,
        io_error,
        io_busy,
        out_of_memory,
        timeout,
        not_supported
    };
    // for faster function calls:
    struct spi_driver_rect {
        uint16_t x1;
        uint16_t y1;
        uint16_t x2;
        uint16_t y2;
    };
    struct spi_driver_set_window_flags {
        int x1 : 1;
        int y1 : 1;
        int x2 : 1;
        int y2 : 1;
    };
    template<uint16_t Width, 
        uint16_t Height,
        int8_t PinCS, 
        int8_t PinDC, 
        uint32_t ClockSpeed=10*1000*1000, 
        TickType_t Timeout=portMAX_DELAY,
        size_t BatchBufferSize=64>
    struct spi_driver {
        constexpr static const uint16_t width = Width;
        constexpr static const uint16_t height = Height;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const uint32_t clock_speed = ClockSpeed;
        constexpr static const size_t batch_buffer_size = (BatchBufferSize/sizeof(uint16_t))*sizeof(uint16_t);
        constexpr static const size_t timeout = Timeout;
    private:
        bool m_initialized;
        SPIClass& m_spi;
        SPISettings m_spi_settings;
        size_t m_batch_left;
        uint16_t m_batch_buffer[batch_buffer_size/sizeof(uint16_t)];
        spi_driver_rect m_batch_window;
        
        void spi_write(const uint8_t* data,size_t size) {
#if defined(ESP8266) || defined(ESP32)
            m_spi.writeBytes(data, size);
#else
            while (size > 0) {
                m_spi.transfer(*data);
                ++data;
                --size;
            }
#endif
        }
        void spi_start() {
            m_spi.beginTransaction(m_spi_settings);
            if (pin_cs >= 0) digitalWrite(pin_cs, LOW);
        }

        void spi_end() {
            if (pin_cs >= 0) digitalWrite(pin_cs, HIGH);
            m_spi.endTransaction();
        }
        inline void spi_write(uint8_t data) {
            m_spi.transfer(data);
        }
        inline void spi_write(uint16_t data) {
            m_spi.transfer16(data);
        }
        void send_next_internal(const uint8_t* data,size_t size,bool dc,bool skip_batch_commit) {
            if(!initialized()) {
                initialize();
            }
            if(!skip_batch_commit&&m_batch_left!=0) {
                commit_batch_internal();
            }
            spi_start();
            digitalWrite(pin_dc,dc);
            spi_write(data,size);
            spi_end();
        }
        void batch_write_commit_impl() {
            if(m_batch_left==0)  {
                return;
            }
           send_next_data((uint8_t*)m_batch_buffer,m_batch_left*sizeof(uint16_t),true);
            m_batch_left=0;
        }
        void batch_write_begin_impl(const spi_driver_rect& r,spi_driver_set_window_flags set_flags) {
             // normalize values
            spi_driver_rect b;
            if(r.x1>r.x2) {
                b.x2=r.x1;
                b.x1=r.x2;
            } else {
                b.x1=r.x1;
                b.x2=r.x2;
            }
            if(r.y1>r.y2) {
                b.y2=r.y1;
                b.y1=r.y2;
            } else {
                b.y1=r.y1;
                b.y2=r.y2;
            }
            
            write_window(b,set_flags);
            if(set_flags.x1) {
                m_batch_window.x1 = b.x1;
            }
            if(set_flags.y1) {
                m_batch_window.y1 = b.y1;
            }
            if(set_flags.x2) {
                m_batch_window.x2 = b.x2;
            }
            if(set_flags.y2) {
                m_batch_window.y2 = b.y2;
            }
        }       
        void batch_write_impl(const uint16_t* pixels,
                                size_t count) {
            if(!m_initialized)
                return;
            size_t index = m_batch_left;
            if(index==batch_buffer_size/2) {
                 send_next_data((uint8_t*)m_batch_buffer,batch_buffer_size,true);
                m_batch_left=0;
                index = 0;
            }
            uint16_t* p=m_batch_buffer+index;
            while(0<count) {    
                *p=*pixels;
                --count;
                ++m_batch_left;
                ++pixels;
                ++p;
                if(m_batch_left==(batch_buffer_size/2)) {
                    send_next_data((uint8_t*)m_batch_buffer,batch_buffer_size,true);
                    p=m_batch_buffer;
                    m_batch_left=0;
                }
            }
        }
        void frame_fill_impl(const spi_driver_rect& b,
                            uint16_t color) {
            
            // normalize values
            spi_driver_rect br;
            if(b.x1>b.x2) {
                br.x2=b.x1;
                br.x1=b.x2;
            } else {
                br.x1=b.x1;
                br.x2=b.x2;
            }
            if(b.y1>b.y2) {
                br.y2=b.y1;
                br.y1=b.y2;
            } else {
                br.y1=b.y1;
                br.y2=b.y2;
            }
           
            uint16_t w = br.x2-br.x1+1;
            uint16_t h = br.y2-br.y1+1;
            if(w==1&&h==1) {
                pixel_write_impl(br.x1,br.y1,color);
                return;
            }
            
            batch_write_begin_impl(br,{1,1,1,1});
            size_t pc=w*h;
            while(pc>0) {
                batch_write_impl(&color,1);
                --pc;
            }
            batch_write_commit_impl();
        }
        void pixel_write_impl(uint16_t x,
                                uint16_t y,
                                uint16_t color) {
            // check values
            if(x>=width || y>=height)
                return;
            
            // set the address window. we're not
            // actually batching here.
            spi_driver_rect b;
            b.x1=b.x2=x;
            b.y1=b.y2=y;
            spi_driver_set_window_flags f;
            f.x1 = m_batch_window.x1!=x;
            f.y1 = m_batch_window.y1!=y;
           
            batch_write_begin_impl(b,f);
            send_next_data((uint8_t*)&color,2,false);
        }
    protected:
        void send_command_init(uint8_t cmd) {
            spi_start();
            digitalWrite(pin_dc,LOW);
            spi_write(cmd);
            spi_end();
        }
        void send_data_init(uint8_t data) {
            spi_start();
            digitalWrite(pin_dc,HIGH);
            spi_write(data);
            spi_end();
        }
        void send_data_init(const uint8_t* data, size_t size) {
            spi_start();
            digitalWrite(pin_dc,HIGH);
            spi_write(data,size);
            spi_end();
        }
        virtual void write_window(const spi_driver_rect& bounds,spi_driver_set_window_flags flags)=0;
        virtual void read_window(const spi_driver_rect& bounds) {

        }
        virtual void initialize_impl() {

        }
        void commit_batch_internal() {
            send_next_data((uint8_t*)m_batch_buffer,m_batch_left*sizeof(uint16_t),true);
            m_batch_left=0;
        }
        
        inline void send_next_command(uint8_t cmd,bool skip_batch_commit=false) {
            send_next_internal(&cmd,1,false,skip_batch_commit);
        }
        inline void send_next_data(const uint8_t* data,size_t size,bool skip_batch_commit=false) {
            send_next_internal(data,size,true,skip_batch_commit);
        }
        
    public:
        spi_driver(SPIClass& spi) : 
                m_initialized(false),
                m_spi(spi),
                m_spi_settings(clock_speed,MSBFIRST,SPI_MODE0),
                m_batch_left(0) {
            
        }
        virtual ~spi_driver() {
            
        }
        inline bool initialized() const {
            return m_initialized;
        }
        void initialize()
        {
            if(!m_initialized) {
                if (pin_cs >= 0) {
                    digitalWrite(pin_cs, HIGH);
                    pinMode(pin_cs, OUTPUT);
                }
                if (pin_dc >= 0) {
                    digitalWrite(pin_dc, HIGH);
                    pinMode(pin_dc, OUTPUT);
                }
                m_spi.begin();
                initialize_impl();
                m_initialized = true;
            }
        }
        // writes bitmap data to the frame buffer
        void frame_write(const spi_driver_rect& bounds,const uint8_t* bmp_data) {
            // normalize values
            spi_driver_rect b;
            if(bounds.x1>bounds.x2) {
                b.x2=bounds.x1;
                b.x1=bounds.x2;
            } else {
                b.x1=bounds.x1;
                b.x2=bounds.x2;
            }
            if(bounds.y1>bounds.y2) {
                b.y2=bounds.y1;
                b.y1=bounds.y2;
            } else {
                b.y1=bounds.y1;
                b.y2=bounds.y2;
            }
            if(b.x1>=width || b.y1>=height)
                return;
          
            batch_write_begin_impl(b,{1,1,1,1});
            send_next_data(bmp_data,(b.x2-b.x1+1)*(b.y2-b.y1+1)*2,true);
        }
        // fills the target rectangle of the frame buffer with a pixel
        void frame_fill(const spi_driver_rect& bounds,uint16_t color) {
            if(bounds.x1==bounds.x2&&bounds.y1==bounds.y2)
               return pixel_write_impl(bounds.x1,bounds.y1,color);
            return frame_fill_impl(bounds,color);
        }

        // begins a batch write for the given target coordinates
        inline void batch_write_begin(const spi_driver_rect& bounds) {
            batch_write_begin_impl(bounds,{1,1,1,1});
        }
        // does a batch write. the batch operation must have been started with
        // batch_write_begin() or queued_batch_write_begin()
        inline void batch_write(const uint16_t* pixels,size_t count) {
            batch_write_impl(pixels,count);
        }
        // commits any pending batch
        inline void batch_write_commit() {
            return batch_write_commit_impl();
        }
        // writes a pixel at the specified coordinates
        inline void pixel_write(uint16_t x,uint16_t y,uint16_t color) {
            pixel_write_impl(x,y,color);
        }

    };
}