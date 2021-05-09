#ifndef HTCW_ESPIDF_I2C_MASTER_HPP
#define HTCW_ESPIDF_I2C_MASTER_HPP
#include <atomic>
#include <stdint.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "soc/gpio_sig_map.h"

#define MATRIX_DETACH_OUT_SIG 0x100
#define MATRIX_DETACH_IN_LOW_PIN 0x30
#define MATRIX_DETACH_IN_LOW_HIGH 0x38

namespace espidf {
    // indicates the result of an i2c operation
    enum struct i2c_result {
        // the operation completed successfully
        success = 0,
        // one or more arguments is invalid
        invalid_argument=1,
        // the driver could not be installed
        driver_installation_error=2,
        // the driver is not in the right state, maybe slave but master commands are being sent
        invalid_state=3,
        // no ack was received upon transmit or receive
        io_error=4,
        // a timeout occurred because the bus was busy
        bus_timeout=5,
        // the handle is not valid
        invalid_handle=6
    };

    class i2c_master;
    class i2c_master_command;
    class i2c_master_command final {
        i2c_cmd_handle_t m_handle;
        i2c_master_command(const i2c_master_command& rhs)=delete;
        i2c_master_command& operator=(const i2c_master_command& rhs)=delete;
    public:
        i2c_master_command() : m_handle(nullptr) {
            m_handle = i2c_cmd_link_create();
        }
        i2c_master_command(i2c_master_command&& rhs) : m_handle(rhs.m_handle) {
            rhs.m_handle=nullptr;
        }
        i2c_master_command& operator=(i2c_master_command&& rhs)  {
            if(nullptr!=m_handle)
                i2c_cmd_link_delete(m_handle);
            m_handle=rhs.m_handle;
            rhs.m_handle=nullptr;
            return *this;
        }
        ~i2c_master_command() {
            if(nullptr!=m_handle)
                i2c_cmd_link_delete(m_handle);
        }
        i2c_cmd_handle_t handle() const {
            return m_handle;
        }
        bool initialized() const {
            return nullptr!=m_handle;
        }
        i2c_result start() {
            esp_err_t res = i2c_master_start(m_handle);
            if(ESP_OK!=res) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result stop() {
            esp_err_t res = i2c_master_stop(m_handle);
            if(ESP_OK!=res) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result read(void* data,size_t size,i2c_ack_type_t ack=i2c_ack_type_t::I2C_MASTER_ACK) {
            if(nullptr==data) {
                return i2c_result::invalid_argument;
            }
            esp_err_t res = i2c_master_read(m_handle,(uint8_t*)data,size,ack);
            if(ESP_OK!=res) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result read(uint8_t* data,i2c_ack_type_t ack=i2c_ack_type_t::I2C_MASTER_ACK) {
            if(nullptr==data) {
                return i2c_result::invalid_argument;
            }
            esp_err_t res = i2c_master_read_byte(m_handle,data,ack);
            if(ESP_OK!=res) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result write(const void* data,size_t size,bool ack=false) {
            if(nullptr==data) {
                return i2c_result::invalid_argument;
            }
            esp_err_t res = i2c_master_write(m_handle,(uint8_t*)data,size,ack);
            if(res!=ESP_OK) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result write(uint8_t data,bool ack=false) {
            esp_err_t res = i2c_master_write_byte(m_handle,data,ack);
            if(res!=ESP_OK) {
                return i2c_result::invalid_handle;
            }
            return i2c_result::success;
        }
        i2c_result begin_read(uint8_t address,bool ack=false) {
            return write(address<<1|I2C_MASTER_READ,ack);
        }
        i2c_result begin_write(uint8_t address,bool ack=false) {
           return write(address<<1|I2C_MASTER_WRITE,ack);
        }
        i2c_result read_register(uint8_t address,uint8_t regist, void* result,size_t size) {
            if(nullptr==result || 0==size) {
                return i2c_result::invalid_argument;
            }
            i2c_result r;
            r=begin_write(address,true);
            if(r!=i2c_result::success)
                return r;
                
            r=write(regist,true);
            if(r!=i2c_result::success)
                return r;
                
            r=start();
            if(r!=i2c_result::success)
                return r;
            r=begin_read(address,true);
            if(r!=i2c_result::success)
                return r;
            if (size > 1) {
                r=read(result, size - 1, I2C_MASTER_ACK);
                if(r!=i2c_result::success)
                    return r;
            
            }
            r=read(((uint8_t*)result + size - 1), I2C_MASTER_NACK);
            if(r!=i2c_result::success)
                return r;
            
            return stop();
        }
        i2c_result write_register(uint8_t address,uint8_t regist, const void* data,size_t size) {
            if(nullptr==data || 0==size) {
                return i2c_result::invalid_argument;
            }
            i2c_result r;
            r=begin_write(address,true);
            if(r!=i2c_result::success)
                return r;
            
            r=write(regist,true);
            if(r!=i2c_result::success)
                return r;
            
            r=write(data,size,true);
            if(r!=i2c_result::success)
                return r;
            
            return stop();
        }
    };
    class i2c_master final {
        bool m_initialized;
        i2c_port_t m_port;
        i2c_config_t m_configuration;
        i2c_master(const i2c_master& rhs)=delete;
        i2c_master& operator=(const i2c_master& rhs)=delete;
    public:
        constexpr static const int buffer_length = 128;

        i2c_master(
            i2c_result* out_result=nullptr,
            i2c_port_t i2c_port=I2C_NUM_0, 
            gpio_num_t sda=GPIO_NUM_21,
            gpio_num_t scl=GPIO_NUM_22,
            bool sda_pullup=true,
            bool scl_pullup=true, 
            uint32_t frequency=100000,
            int interrupt_flags=0
            ) : m_initialized(false) {
            m_configuration.mode=i2c_mode_t::I2C_MODE_MASTER;
            m_configuration.sda_io_num = (int)sda;
            m_configuration.scl_io_num = (int)scl;
            m_configuration.sda_pullup_en = sda_pullup;
            m_configuration.scl_pullup_en = scl_pullup;
            m_configuration.master.clk_speed=frequency;
            esp_err_t res = i2c_param_config(i2c_port, &m_configuration);
            if (res != ESP_OK) {
                if(nullptr!=out_result) {
                    *out_result = i2c_result::invalid_argument;
                }
                return;
            }
            res = i2c_driver_install(i2c_port, m_configuration.mode, 0, 0, interrupt_flags);
            if (res != ESP_OK) {
                if(nullptr!=out_result) {
                    if(res==ESP_ERR_INVALID_ARG)
                        *out_result = i2c_result::invalid_argument;
                    else
                        *out_result = i2c_result::driver_installation_error;
                }
                return;
            } 
            m_port=i2c_port;
            m_initialized=true;
        }
        i2c_master(i2c_master&& rhs) : m_initialized(rhs.m_initialized),m_configuration(rhs.m_configuration) {
            rhs.m_initialized=false;
        }
        i2c_master& operator=(i2c_master&& rhs) {
            m_initialized=rhs.m_initialized;
            m_configuration = rhs.m_configuration;
            rhs.m_initialized=false;
            return *this;
        }
        ~i2c_master() {m_initialized=false;}
        bool initialized() const {
            return m_initialized;
        }
        i2c_result execute(const i2c_master_command& command,TickType_t timeout_ticks=portMUX_NO_TIMEOUT) {
            return execute(m_port,command,timeout_ticks);
        }
        static i2c_result execute(i2c_port_t port,const i2c_master_command& command,TickType_t timeout_ticks=portMUX_NO_TIMEOUT) {
            esp_err_t res = i2c_master_cmd_begin(port,command.handle(),timeout_ticks);
            if(ESP_OK!=res) { 
                switch(res) {
                    case ESP_ERR_TIMEOUT:
                        return i2c_result::bus_timeout;
                    case ESP_ERR_INVALID_STATE:
                        return i2c_result::invalid_state;
                    case ESP_ERR_INVALID_ARG:
                        return i2c_result::invalid_handle;
                }
                return i2c_result::io_error;
            }
            return i2c_result::success;
        }

    };
}
#endif