#ifndef HTCW_ESPIDF_SPI_MASTER_HPP
#define HTCW_ESPIDF_SPI_MASTER_HPP
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
namespace espidf {
    enum struct spi_result {
        success = 0,
        invalid_argument=1,
        host_in_use = 2,
        dma_in_use = 3,
        out_of_memory=4,
        no_more_cs_slots=5,
        previous_transactions_pending=6,
        timeout=7
    };
    class spi_master final {
        bool m_initialized;
        spi_host_device_t m_host_id;
        size_t m_dma_transfer_size;
        spi_master(const spi_master& rhs)=delete;
        spi_master& operator=(const spi_master& rhs)=delete;
    public:
        
        spi_master(spi_result* out_result = nullptr, 
            spi_host_device_t host_id=SPI2_HOST,
            gpio_num_t sclk=GPIO_NUM_18,
            gpio_num_t miso=GPIO_NUM_19,
            gpio_num_t mosi=GPIO_NUM_23,
            gpio_num_t d2=GPIO_NUM_NC,
            gpio_num_t d3=GPIO_NUM_NC,
            size_t max_transfer_size=0,
            int dma_chan = 0,
            uint32_t flags=0,
            int intr_flags = 0) : m_initialized(false),m_host_id(host_id),m_dma_transfer_size((dma_chan!=0)?max_transfer_size:0) {
            
            spi_bus_config_t config;
            config.sclk_io_num=sclk;
            config.miso_io_num=miso;
            config.mosi_io_num=mosi;
            config.quadwp_io_num=d2;
            config.quadhd_io_num=d3;
            config.max_transfer_sz=m_dma_transfer_size;
            config.flags=flags;
            config.intr_flags = intr_flags;
            esp_err_t res = spi_bus_initialize(host_id, &config, dma_chan);
            if(res==ESP_OK) {
                m_initialized=true;
                if(nullptr!=out_result) {
                    *out_result=spi_result::success;
                }
                return;
            }
            spi_result r;
            switch(res) {
                case ESP_ERR_INVALID_ARG:
                    r=spi_result::invalid_argument;
                    break;
                case ESP_ERR_INVALID_STATE:
                    r=spi_result::host_in_use;
                    break;
                case ESP_ERR_NOT_FOUND:
                    r=spi_result::dma_in_use;
                    break;
                default:
                    r=spi_result::out_of_memory;
                    break;
            }
            if(nullptr!=out_result)
                *out_result = r;
        }
        spi_master(spi_master&& rhs) : m_initialized(rhs.m_initialized), m_host_id(rhs.m_host_id){
            rhs.m_initialized=false;
        }
        spi_master& operator=(spi_master&& rhs) {
            m_initialized=rhs.m_initialized;
            m_host_id=rhs.m_host_id;
            rhs.m_initialized=false;
            return *this;
        }
        ~spi_master() {
            if(m_initialized) {
                spi_bus_free(m_host_id);
            }   
        }
        inline bool initialized() const {
            return m_initialized;
        }
        inline spi_host_device_t host_id() const {
            return m_host_id;
        }
        inline size_t dma_transfer_size() const {
            return m_dma_transfer_size;
        }
    };
    class spi_device final {
        spi_device_handle_t m_handle;
        size_t m_max_transactions;
        spi_device(const spi_device& rhs)=delete;
        spi_device& operator=(const spi_device& rhs)=delete;
    public:
        spi_device(spi_host_device_t host_id,const spi_device_interface_config_t& config,spi_result* out_result=nullptr) : m_handle(nullptr) {
            m_max_transactions = config.queue_size;
            esp_err_t res = spi_bus_add_device(host_id,&config,&m_handle);
            if(ESP_OK==res) {
                if(nullptr!=out_result) {
                    *out_result = spi_result::success;
                }
                return;
            }
            spi_result r;
            switch(res) {
                case ESP_ERR_INVALID_ARG:
                    r=spi_result::invalid_argument;
                    break;
                case ESP_ERR_NOT_FOUND:
                    r=spi_result::no_more_cs_slots;
                    break;
                default:
                    r=spi_result::out_of_memory;
                    break;
            }
            if(nullptr!=out_result) {
                *out_result = r;
            }
        }
        spi_device(spi_device&& rhs) : m_handle(rhs.m_handle) {
            rhs.m_handle=nullptr;
        }
        spi_device& operator=(spi_device&& rhs) {
            if(nullptr!=m_handle) {
                spi_bus_remove_device(m_handle);
            }
            m_handle=rhs.m_handle;
            rhs.m_handle=nullptr;
            return *this;
        }
        ~spi_device() {
            if(nullptr!=m_handle) {
                spi_bus_remove_device(m_handle);
            }
        }
        inline spi_device_handle_t handle() const {
            return m_handle;
        }
        inline bool initialized() const {
            return nullptr!=m_handle;
        }
        inline size_t max_transactions() const {
            return m_max_transactions;
        }
        // make a read transaction
        static void make_read(spi_transaction_t* trans, uint8_t* data, size_t size,void* user = nullptr,bool use_rxdata=true) {
            trans->addr = 0;
            trans->cmd = 0;
            trans->length = trans->rxlength=size*8;
            trans->tx_buffer = nullptr;
            if(size>4 || !use_rxdata) {
                trans->flags = 0;
                trans->rx_buffer = data;
            } else {
                trans->flags = SPI_TRANS_USE_RXDATA;
            }
            trans->user = user;
        }
        // make a write transaction
        static void make_write(spi_transaction_t* trans, const uint8_t* data, size_t size,void* user=nullptr) {
            trans->addr = 0;
            trans->cmd = 0;
            trans->length = size * 8;
            trans->rx_buffer = nullptr;
            trans->rxlength = 0;
            if(size>4) {
                trans->flags = 0;
                trans->tx_buffer = data;
            } else {
                trans->flags = SPI_TRANS_USE_TXDATA;
                memcpy(trans->tx_data,data,size);
            }
            trans->user = user;
        }
        // makes a full duplex transaction - size_in must be less than or equal to size_out! (it's a restriction of SPI)
        static void make_read_write(spi_transaction_t* trans, const uint8_t *data_out,size_t size_out,uint8_t* data_in,size_t size_in,void* user=nullptr,bool use_rxdata=true) {
            trans->addr = 0;
            trans->cmd = 0;
            trans->length = size_out*8;
            if(size_out>4) {
                trans->flags = 0;
                trans->tx_buffer = data_out;
                trans->rx_buffer = data_in;
            } else {
                trans->flags = SPI_TRANS_USE_TXDATA;
                memcpy(trans->tx_data,data_out,size_out);
            }
            trans->rxlength = size_in*8;;
            if(size_in>4 || !use_rxdata) {
                trans->rx_buffer = data_in;
            } else {
                trans->flags |= SPI_TRANS_USE_RXDATA;
                trans->rx_buffer = nullptr;
            }
            trans->user = user;
        }
        spi_result acquire_bus(TickType_t timeout=portMAX_DELAY) {
            esp_err_t res = spi_device_acquire_bus(m_handle,timeout);
            if(ESP_OK==res) return spi_result::success;
            return spi_result::invalid_argument;
        }
        void release_bus() {
            spi_device_release_bus(m_handle);
        }
        spi_result queue_transaction(spi_transaction_t* ptrans,TickType_t timeout=portMAX_DELAY) {
            esp_err_t res = spi_device_queue_trans(m_handle,ptrans,timeout);
            if(ESP_OK==res) return spi_result::success;
            switch(res) {
                case ESP_ERR_INVALID_ARG:
                    return spi_result::invalid_argument;
                case ESP_ERR_INVALID_STATE:
                    return spi_result::previous_transactions_pending;
                case ESP_ERR_NO_MEM:
                    return spi_result::out_of_memory;
                default:
                    return spi_result::timeout;
            }
        }
        spi_result get_next_queued_result(spi_transaction_t** out_ptrans,TickType_t timeout=portMAX_DELAY) {
            esp_err_t res = spi_device_get_trans_result(m_handle,out_ptrans,timeout);
            if(ESP_OK==res) {
                return spi_result::success;
            }
            switch(res) {
                case ESP_ERR_INVALID_ARG:
                    return spi_result::invalid_argument;
                default:
                    return spi_result::timeout;
            }
        }
        spi_result transaction(spi_transaction_t* ptrans,bool polling = false) {
            esp_err_t res = polling?spi_device_polling_transmit(m_handle,ptrans) :spi_device_transmit(m_handle,ptrans);
            if(ESP_OK==res) return spi_result::success;
            return spi_result::invalid_argument;
        }
        spi_result polling_transaction(spi_transaction_t* ptrans,TickType_t timeout=portMAX_DELAY) {
            esp_err_t res = spi_device_polling_start(m_handle,ptrans,timeout);
            if(ESP_OK==res) return spi_result::success;
            switch(res) {
                case ESP_ERR_INVALID_ARG:
                    return spi_result::invalid_argument;
                case ESP_ERR_INVALID_STATE:
                    return spi_result::previous_transactions_pending;
                case ESP_ERR_NO_MEM:
                    return spi_result::out_of_memory;
                default:
                    return spi_result::timeout;
            }
        }
        
        spi_result read(uint8_t* data, size_t size,void* user=nullptr,bool use_polling=true) {
            spi_transaction_t trans;
            spi_result r;
            if(0<size) {
                make_read(&trans,data,size,user);
                r=transaction(&trans,use_polling);
                if(spi_result::success!=r) {
                    return r;
                }
                if(SPI_TRANS_USE_RXDATA==(trans.flags & SPI_TRANS_USE_RXDATA)) {
                    memcpy(data,trans.rx_data,size);
                }
            }
            return spi_result::success;
        }
        spi_result write(const uint8_t* data, size_t size,void* user=nullptr,bool use_polling=true) {
            spi_transaction_t trans;
            spi_result r;
            if(0<size) {
                make_write(&trans,data,size,user);
                r=transaction(&trans,use_polling);
                if(spi_result::success!=r) {
                    return r;
                }
            }
            return spi_result::success;
        }
        spi_result queue_read(spi_transaction_t* trans, uint8_t* data, size_t size,TickType_t timeout=portMAX_DELAY,void* user=nullptr) {
            spi_result r;
            if(0<size) {
                make_read(trans,data,size,user);
                r=queue_transaction(trans,timeout);
                if(spi_result::success!=r) {
                    return r;
                }                
            }
            return spi_result::success;
        }
        spi_result queue_write(spi_transaction_t* trans, const uint8_t* data, size_t size,TickType_t timeout=portMAX_DELAY,void* user=nullptr) {
            spi_result r;
            if(0<size) {
                make_write(trans,data,size,user);
                r=queue_transaction(trans,timeout);
                if(spi_result::success!=r) {
                    return r;
                }
            }
            return spi_result::success;
        }
    };
    enum struct spi_transaction_type {
        any = 0,
        polling = 1,
        interrupt = 2,
        queued = 3
    };
    template<size_t MaxTransactions=7,TickType_t Timeout = portMAX_DELAY>
    struct spi_device_manager final {
        
    private:
        constexpr static const size_t max_transactions = MaxTransactions;
        constexpr static const TickType_t timeout = Timeout;
        spi_device& m_device;
        spi_transaction_t m_queued_transactions[MaxTransactions];
        size_t m_queue_head;
        size_t m_queue_tail;
    public:
        spi_device_manager(spi_device& device) : m_device(device) {
        }
        inline bool has_queued_transactions() const {
            return m_queue_head!=m_queue_tail;
        }
        spi_result wait_one(spi_transaction_t **out_ptrans=nullptr) {
            spi_result r;
            spi_transaction_t* ptrans;
            if(m_queue_head!=m_queue_tail){
                r=m_device.get_next_queued_result(&ptrans);
                if(spi_result::success!=r)
                    return r;
                m_queue_head=(m_queue_head+1)%max_transactions;    
                if(out_ptrans!=nullptr)
                    *out_ptrans = ptrans;
                return spi_result::success;
            }
            if(out_ptrans!=nullptr)
                *out_ptrans = nullptr;
            return spi_result::success;
        }
        spi_result wait_all() {
            spi_result r;
            spi_transaction_t* ptrans;
            while(m_queue_tail!=m_queue_head) {
                r=m_device.get_next_queued_result(&ptrans);
                if(spi_result::success!=r)
                    return r;
                m_queue_head=(m_queue_head+1)%max_transactions;    
            }
            return spi_result::success;
        }
        spi_result ensure_free_queue() {
            if(m_queue_head==(m_queue_tail+1)%max_transactions) {
                return wait_one();
            }
            return spi_result::success;
        }
        spi_result write(const uint8_t* data, size_t size,void* user,spi_transaction_type type=spi_transaction_type::any) {
            spi_result r;
            if(type==spi_transaction_type::any) {
                // if there are pending transactions queued, use interrupt instead of polling
                type = (m_queue_head==m_queue_tail)?spi_transaction_type::polling:spi_transaction_type::interrupt;
            }
            spi_transaction_t t;
            spi_device::make_write(&t,data,size,user);
        
            if(type==spi_transaction_type::polling) {
                return m_device.transaction(&t,true);
            } else if(type==spi_transaction_type::interrupt) {
                return m_device.transaction(&t,false);
            }
            spi_transaction_t* ptrans;
            r=ensure_free_queue();
            if(spi_result::success!=r)
                return r;
            ptrans = &m_queued_transactions[m_queue_tail];
            m_queue_tail=(m_queue_tail+1)%max_transactions;
            spi_device::make_write(ptrans,data,size,user);
            return m_device.queue_transaction(ptrans,timeout);
        }
        spi_result read(uint8_t* data, size_t size,void* user,spi_transaction_type type=spi_transaction_type::any,bool use_rxdata=true) {
            spi_result r;
            if(type==spi_transaction_type::any) {
                // if there are pending transactions queued, use interrupt instead of polling
                type = (m_queue_head==m_queue_tail)?spi_transaction_type::polling:spi_transaction_type::interrupt;
            }
            spi_transaction_t t;
            spi_device::make_read(&t,data,size,user,use_rxdata);
            if(type==spi_transaction_type::polling) {
                return m_device.transaction(&t,true);
            } else if(type==spi_transaction_type::interrupt) {
                return m_device.transaction(&t,false);
            }
            spi_transaction_t* ptrans;
            r=ensure_free_queue();
            if(spi_result::success!=r)
                return r;
            ptrans = &m_queued_transactions[m_queue_tail];
            m_queue_tail=(m_queue_tail+1)%max_transactions;
            spi_device::make_read(ptrans,data,size,user,use_rxdata);
            return m_device.queue_transaction(ptrans,timeout);
        }
        spi_result read_write(const uint8_t* write_data,size_t write_size, uint8_t* read_data, size_t read_size,void* user,spi_transaction_type type=spi_transaction_type::any,bool use_rxdata=true) {
            spi_result r;
            if(type==spi_transaction_type::any) {
                // if there are pending transactions queued, use interrupt instead of polling
                type = (m_queue_head==m_queue_tail)?spi_transaction_type::polling:spi_transaction_type::interrupt;
            }
            spi_transaction_t t;
            spi_device::make_read_write(&t,write_data,write_size, read_data,read_size,user,use_rxdata);
            if(type==spi_transaction_type::polling) {
                return m_device.transaction(&t,true);
            } else if(type==spi_transaction_type::interrupt) {
                return m_device.transaction(&t,false);
            }
            spi_transaction_t* ptrans;
            r=ensure_free_queue();
            if(spi_result::success!=r)
                return r;
            ptrans = &m_queued_transactions[m_queue_tail];
            m_queue_tail=(m_queue_tail+1)%max_transactions;
            spi_device::make_read_write(ptrans,write_data,write_size, read_data,read_size,user,use_rxdata);
            return m_device.queue_transaction(ptrans,timeout);
        }
    };
}
#endif