extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "drivers/ra8875.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace espidf;
using namespace gfx;

#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_INT   GPIO_NUM_NC
#define PIN_NUM_RST  GPIO_NUM_4
#define PIN_NUM_BCKL GPIO_NUM_15

#define PARALLEL_LINES 8

#define LCD_WIDTH 800
#define LCD_HEIGHT 480

spi_master spi_host(nullptr,
                LCD_HOST,
                PIN_NUM_CLK,
                PIN_NUM_MISO,
                PIN_NUM_MOSI,
                GPIO_NUM_NC,
                GPIO_NUM_NC,
                PARALLEL_LINES*LCD_WIDTH*2,
                DMA_CHAN);

using lcd_type = ra8875<LCD_WIDTH,
                        LCD_HEIGHT,
                        LCD_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_BCKL,
                        PIN_NUM_RST,
                        PIN_NUM_INT>;

lcd_type lcd;

void app_main() {
    // check to make sure SPI was initialized successfully
    if(!spi_host.initialized()) {
        printf("SPI host initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
    lcd_type::result lr=lcd.initialize();
    if(lcd_type::result::success!=lr) {
        printf("LCD not initialized - error: 0x%02X\r\n",(int)lr);
    }
}