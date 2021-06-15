extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "drivers/depg0290b.hpp"
#include "drivers/ra8875.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace espidf;
using namespace gfx;
//io(SPI, EPD_CS=5, EPD_DC=19, EPD_RSET=12);
//display(io, EPD_RSET=12, EPD_BUSY=4);
//SPI.begin(EPD_SCLK=18, EPD_MISO=2, EPD_MOSI=23);
#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_2
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_BUSY GPIO_NUM_4
#define PIN_NUM_RST  GPIO_NUM_12
#define PIN_NUM_DC GPIO_NUM_19

#define LCD_WIDTH 128
#define LCD_HEIGHT 296

spi_master spi_host(nullptr,
                LCD_HOST,
                PIN_NUM_CLK,
                PIN_NUM_MISO,
                PIN_NUM_MOSI,
                GPIO_NUM_NC,
                GPIO_NUM_NC,
                4096+8,
                DMA_CHAN);

using lcd_type = depg0290b<LCD_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BUSY>;

lcd_type lcd;

void app_main() {
    // check to make sure SPI was initialized successfully
    if(!spi_host.initialized()) {
        printf("SPI host initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
    draw::suspend(lcd);
    
    lcd.clear(lcd.bounds());
    draw::line(lcd,{0,0,lcd.width-1,lcd.height-1},color<typename lcd_type::pixel_type>::white);
    draw::line(lcd,srect16(0,0,lcd.width-1,lcd.height-1).flip_horizontal(),color<typename lcd_type::pixel_type>::black);
    draw::resume(lcd);
    
}