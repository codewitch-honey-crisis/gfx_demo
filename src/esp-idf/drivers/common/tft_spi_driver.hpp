#include "spi_master.hpp"
namespace espidf {
    enum struct tft_spi_driver_result {
        success = 0,
        invalid_argument,
        io_error,
        io_busy,
        out_of_memory,
        timeout,
        not_supported
    };
    // for faster function calls:
    struct tft_spi_driver_rect {
        uint16_t x1;
        uint16_t y1;
        uint16_t x2;
        uint16_t y2;
    };
    struct tft_spi_driver_set_window_flags {
        int x1 : 1;
        int y1 : 1;
        int x2 : 1;
        int y2 : 1;
    };
    template<size_t Width, 
            size_t Height,
            spi_host_device_t HostId, 
            gpio_num_t PinCS, 
            gpio_num_t PinDC, 
            size_t ClockSpeed=10*1000*1000, 
            size_t MaxTransactions=7,
            bool UsePolling=true, 
            size_t DmaSize=(size_t)-1, 
            TickType_t Timeout=portMAX_DELAY,
            size_t BatchBufferSize=64>
    struct tft_spi_driver {
        constexpr static const size_t width = Width;
        constexpr static const size_t height = Height;
        constexpr static const spi_host_device_t host_id = HostId;
        constexpr static const gpio_num_t pin_cs = PinCS;
        constexpr static const gpio_num_t pin_dc = PinDC;
        constexpr static const size_t clock_speed = ClockSpeed;
        constexpr static const size_t batch_buffer_size = (BatchBufferSize/sizeof(uint16_t))*sizeof(uint16_t);
        constexpr static const size_t max_transactions = MaxTransactions;
        constexpr static const bool use_polling = UsePolling;
        constexpr static const size_t dma_size = DmaSize;
        constexpr static const size_t timeout = Timeout;
        bool m_initialized;
        spi_device m_spi;
        spi_device_manager<max_transactions,timeout> m_spi_mgr;
        size_t m_batch_left;
        uint16_t m_batch_buffer[batch_buffer_size/sizeof(uint16_t)];
        tft_spi_driver_rect m_batch_window;
        
        static tft_spi_driver_result xlt_err(spi_result rr) {    
            if(spi_result::success!=rr) {
                switch(rr) {
                    case spi_result::timeout:
                        return tft_spi_driver_result::timeout;
                    case spi_result::out_of_memory:
                        return tft_spi_driver_result::out_of_memory;
                    case spi_result::previous_transactions_pending:
                        return tft_spi_driver_result::io_busy;
                    default:
                        return tft_spi_driver_result::io_error;
                }
            }
            return tft_spi_driver_result::success;
        }
       
        tft_spi_driver_result commit_batch_internal(bool queued) {
            tft_spi_driver_result r=send_next_data((uint8_t*)m_batch_buffer,m_batch_left*2,queued,true);
            if(tft_spi_driver_result::success!=r)
                return r;
            m_batch_left=0;
            return tft_spi_driver_result::success;
        }
        tft_spi_driver_result send_next_internal(const uint8_t* data,size_t size,bool queued,bool user,bool skip_batch_commit) {
            if(!initialized()) {
                tft_spi_driver_result r = initialize();
                if(tft_spi_driver_result::success!=r)
                    return r;
                m_initialized=true;
            }
            spi_result rr;
            if(!skip_batch_commit&&m_batch_left!=0) {
                tft_spi_driver_result r=commit_batch_internal(queued);
                if(tft_spi_driver_result::success!=r)
                    return r;
            }
            rr = m_spi_mgr.write(data,size,(void*)user,(queued)?spi_transaction_type::queued:(use_polling)?spi_transaction_type::polling:spi_transaction_type::interrupt);
            if(spi_result::success!=rr)
                return xlt_err(rr);
            return tft_spi_driver_result::success;
        }
        
        tft_spi_driver_result retr_next_internal(uint8_t* data,size_t size,bool user) {
            if(!initialized()) {
                tft_spi_driver_result r = initialize();
                if(tft_spi_driver_result::success!=r)
                    return r;
                m_initialized=true;
            }
            spi_result rr;
            if(m_batch_left!=0) {
                tft_spi_driver_result r=commit_batch_internal(false);
                if(tft_spi_driver_result::success!=r)
                    return r;
            }
            rr = m_spi_mgr.read(data,size,(void*)user,use_polling?spi_transaction_type::polling:spi_transaction_type::interrupt,false);
            if(spi_result::success!=rr)
                return xlt_err(rr);
            return tft_spi_driver_result::success;
        }
        tft_spi_driver_result batch_write_commit_impl(bool queued) {
            if(m_batch_left==0)  {
                return tft_spi_driver_result::success;
            }
           tft_spi_driver_result r = send_next_data((uint8_t*)m_batch_buffer,m_batch_left*2,queued,true);
            if(tft_spi_driver_result::success!=r)
                return r;
            m_batch_left=0;
            return tft_spi_driver_result::success;
        }
        tft_spi_driver_result batch_write_begin_impl(const tft_spi_driver_rect& r,
                                    bool queued,tft_spi_driver_set_window_flags set_flags) {
             // normalize values
            tft_spi_driver_rect b;
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
            
            tft_spi_driver_result rr;
            if(queued)
                rr= write_window(b,set_flags);
            else
                rr= queued_write_window(b,set_flags);
            if(tft_spi_driver_result::success!=rr)
                return rr;
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
            return tft_spi_driver_result::success;
        }       
        tft_spi_driver_result batch_write_impl(const uint16_t* pixels,
                                size_t count,
                                bool queued) {
            if(!m_initialized)
                return tft_spi_driver_result::io_error;
            tft_spi_driver_result r;
            size_t index = m_batch_left;
            if(index==batch_buffer_size/2) {
                 r=send_next_data((uint8_t*)m_batch_buffer,batch_buffer_size,queued,true);
                if(tft_spi_driver_result::success!=r) {
                    return r;
                }
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
                    r=send_next_data((uint8_t*)m_batch_buffer,batch_buffer_size,queued,true);
                    if(tft_spi_driver_result::success!=r)
                        return r;
                    p=m_batch_buffer;
                    m_batch_left=0;
                }
            }
            return tft_spi_driver_result::success;
        }
        tft_spi_driver_result frame_fill_impl(const tft_spi_driver_rect& b,
                            uint16_t color,
                            bool queued) {
            
            // normalize values
            tft_spi_driver_rect br;
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
            tft_spi_driver_result r;
            if(w==1&&h==1) {
                return pixel_write_impl(br.x1,br.y1,color,queued);
            }
            spi_result sr = m_spi.acquire_bus(portMAX_DELAY);
            if(spi_result::success!=sr)
                return xlt_err(sr);
           
            r=batch_write_begin_impl(br,queued,{1,1,1,1});
            if(tft_spi_driver_result::success!=r) {
                m_spi.release_bus();
                return r;
            }
            size_t pc=w*h;
            while(pc>0) {
                r=batch_write_impl(&color,1,queued);
                if(tft_spi_driver_result::success!=r) {
                    m_spi.release_bus();
                    return r;
                }
                --pc;
            }
            r=batch_write_commit_impl(queued);
            m_spi.release_bus();
            return r;           
        }
        tft_spi_driver_result pixel_write_impl(uint16_t x,
                                uint16_t y,
                                uint16_t color,
                                bool queued) {
            // check values
            if(x>=width || y>=height)
                return tft_spi_driver_result::success;
            
            // set the address window. we're not
            // actually batching here.
            tft_spi_driver_rect b;
            b.x1=b.x2=x;
            b.y1=b.y2=y;
            tft_spi_driver_set_window_flags f;
            f.x1 = m_batch_window.x1!=x;
            f.y1 = m_batch_window.y1!=y;
           
            tft_spi_driver_result r=batch_write_begin_impl(b,queued,f);
            if(tft_spi_driver_result::success!=r)
                return r;
            return send_next_data((uint8_t*)&color,2,queued,false);
        }
        inline static spi_device_interface_config_t get_device_config() {
            uint32_t flags = 0;
            if(clock_speed>26*1000*1000) {
                flags = SPI_DEVICE_NO_DUMMY;
            }
            spi_device_interface_config_t devcfg={
                .command_bits=0,
                .address_bits=0,
                .dummy_bits=0,
                .mode=0,
                .duty_cycle_pos=0,
                .cs_ena_pretrans=0,
                .cs_ena_posttrans=0,
                .clock_speed_hz=clock_speed,           //Clock out at 26 MHz
                .input_delay_ns = 0,
                .spics_io_num=pin_cs,               //CS pin
                
                .flags =flags,
                .queue_size=max_transactions,                          //We want to be able to queue 7 transactions at a time
                .pre_cb=[](spi_transaction_t*t){
                    int dc=(int)t->user;
                    gpio_set_level(pin_dc, dc!=0);
                },  //Specify pre-transfer callback to handle D/C line
                .post_cb=NULL
            };
            return devcfg;
        } 
    protected:
        virtual tft_spi_driver_result read_window(const tft_spi_driver_rect& bounds) {
            return tft_spi_driver_result::not_supported;
        }
        virtual tft_spi_driver_result write_window(const tft_spi_driver_rect& bounds,tft_spi_driver_set_window_flags set_flags)=0;
        virtual tft_spi_driver_result queued_write_window(const tft_spi_driver_rect& bounds,tft_spi_driver_set_window_flags set_flags)=0;

        inline tft_spi_driver_result send_next_command(uint8_t cmd,bool queued,bool skip_batch_commit=false) {
            return send_next_internal(&cmd,1,queued,0,skip_batch_commit);
        }
        inline tft_spi_driver_result send_next_data(const uint8_t* data,size_t size,bool queued,bool skip_batch_commit=false) {
            return send_next_internal(data,size,queued,1,skip_batch_commit);
        }
        inline tft_spi_driver_result read_next_data(uint8_t* data,size_t size) {
            return retr_next_internal(data,size,1);
        }
        inline tft_spi_driver_result send_init_command(uint8_t cmd) {
            spi_result r = m_spi.write(&cmd,1,(void*)0);
            if(spi_result::success!=r)
                return xlt_err(r);
            return tft_spi_driver_result::success;
        }
        inline tft_spi_driver_result send_init_data(const uint8_t* data, size_t size) {
            spi_result r = m_spi.write(data,size,(void*)1);
            if(spi_result::success!=r)
                return xlt_err(r);
            return tft_spi_driver_result::success;
        }
       
    public:
            tft_spi_driver(tft_spi_driver_result* out_result = nullptr) : 
            m_initialized(false),
            m_spi(host_id,get_device_config()),
            m_spi_mgr(m_spi),
            m_batch_left(0),
            m_batch_window({(uint16_t)-1,(uint16_t)-1,(uint16_t)-1,(uint16_t)-1}) {
            if(!m_spi.initialized()) {
                if(nullptr!=out_result) {
                    *out_result=tft_spi_driver_result::io_error;
                }
                return;
            }
        }
        // indicates if the driver is initialized
        inline bool initialized() const {
            return m_initialized;
        }
        // forces initialization of the driver if not already initialized
        virtual tft_spi_driver_result initialize()=0;

        // writes bitmap data to the frame buffer
        tft_spi_driver_result frame_write(const tft_spi_driver_rect& bounds,const uint8_t* bmp_data) {
            // normalize values
            tft_spi_driver_rect b;
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
                return tft_spi_driver_result::success;
          
            tft_spi_driver_result r=batch_write_begin_impl(b,false,{1,1,1,1});
            if(tft_spi_driver_result::success!=r)
                return r;
            return send_next_data(bmp_data,(b.x2-b.x1+1)*(b.y2-b.y1+1)*2,false,true);
        }
        tft_spi_driver_result frame_read(const tft_spi_driver_rect& bounds,uint8_t* bmp_data) {
            // normalize values
            tft_spi_driver_rect b;
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
                return tft_spi_driver_result::success;
            m_batch_window.x1=m_batch_window.y1=m_batch_window.x2=m_batch_window.y2=(uint16_t)-1;
            tft_spi_driver_result r=read_window(b);
            
            if(tft_spi_driver_result::success!=r) {
                return r;
            
            }
            return read_next_data(bmp_data,(b.x2-b.x1+1)*(b.y2-b.y1+1)*2);
        }
        inline tft_spi_driver_result pixel_read(uint16_t x,uint16_t y,uint16_t* out_color) {
            return frame_read({x,y,x,y},(uint8_t*)out_color);
        }
        // queues a frame write operation. The bitmap data must be valid 
        // for the duration of the operation (until queued_wait_all())
        tft_spi_driver_result queued_frame_write(const tft_spi_driver_rect& bounds,
                                uint8_t* bmp_data,
                                bool preflush=false) {
            // normalize values
            tft_spi_driver_rect b;
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
                return tft_spi_driver_result::success;
            tft_spi_driver_result r;
            if(preflush) {
                // flush any pending batches or 
                // transactions if necessary:
                r=batch_write_commit_impl(true);
                if(tft_spi_driver_result::success!=r) {
                    return r;
                }
                r=queued_wait_all();
                if(tft_spi_driver_result::success!=r)
                    return r;
            }
            // set the address window - we don't actually do a batch
            // here, but we use this for our own purposes
            
            r=batch_write_begin_impl(b,true,{1,1,1,1});
            if(tft_spi_driver_result::success!=r)
                return r;
            
            r=send_next_data(bmp_data,
                            (b.x2-b.x1+1)*(b.y2-b.y1+1)*2,
                            true);
            
            // When we are here, the SPI driver is busy (in the background) 
            // getting the transactions sent. That happens mostly using DMA, 
            // so the CPU doesn't have much to do here. We're not going to 
            // wait for the transaction to finish because we may as well spend
            // the time doing something else. When that is done, we can call
            // queued_wait_all(), which will wait for the transfers to be done.
            // otherwise, the transactions will be queued as the old ones finish
            return r;  
        }
        // fills the target rectangle of the frame buffer with a pixel
        tft_spi_driver_result frame_fill(const tft_spi_driver_rect& bounds,uint16_t color) {
            if(bounds.x1==bounds.x2&&bounds.y1==bounds.y2)
               return pixel_write_impl(bounds.x1,bounds.y1,color,false);
            return frame_fill_impl(bounds,color,false);
        }
        // queues the fill of a target rectangle with the specified pixel
        tft_spi_driver_result queued_frame_fill(const tft_spi_driver_rect& bounds,uint16_t color) {
            if(bounds.x1==bounds.x2&&bounds.y1==bounds.y2)
                return pixel_write_impl(bounds.x1,bounds.y1,color,false);
            return frame_fill_impl(bounds,color,true);
        }

        // begins a batch write for the given target coordinates
        tft_spi_driver_result batch_write_begin(const tft_spi_driver_rect& bounds) {
           
            return batch_write_begin_impl(bounds,false,{1,1,1,1});
        }
        // queues the beginning of a batch write for the target coordinates
        tft_spi_driver_result queued_batch_write_begin(const tft_spi_driver_rect& bounds) {
           
            return batch_write_begin_impl(bounds,true,{1,1,1,1});
        }
        // does a batch write. the batch operation must have been started with
        // batch_write_begin() or queued_batch_write_begin()
        tft_spi_driver_result batch_write(const uint16_t* pixels,size_t count) {
            return batch_write_impl(pixels,count,false);
        }
        // does a queued batch write. the batch operation must have been started with
        // batch_write_begin() or queued_batch_write_begin()
        tft_spi_driver_result queued_batch_write(const uint16_t* pixels,size_t count) {
            return batch_write_impl(pixels,count,true);
        }
        // commits any pending batch
        tft_spi_driver_result batch_write_commit() {
            return batch_write_commit_impl(false);
        }
        // queues the commit of any pending batch
        tft_spi_driver_result queued_batch_write_commit() {
            return batch_write_commit_impl(true);
        }
        // writes a pixel at the specified coordinates
        inline tft_spi_driver_result pixel_write(uint16_t x,uint16_t y,uint16_t color) {
            return pixel_write_impl(x,y,color,false);
        }
        // queues a write of a pixel at the specified coordinates
        inline tft_spi_driver_result queued_pixel_write(uint16_t x,uint16_t y,uint16_t color) {
            return pixel_write_impl(x,y,color,true);
        }
        // waits for all pending queued operations
        tft_spi_driver_result queued_wait_all()
        {
            spi_result rr = m_spi_mgr.wait_all();
            if(spi_result::success!=rr)
                return xlt_err(rr);
            return tft_spi_driver_result::success;
        }
    };
}