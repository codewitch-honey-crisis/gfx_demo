extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_master.hpp"
#include "esp_spiffs.h"
#include "max7219.hpp"
#include "gfx_drawing.hpp"
#include "gfx_image.hpp"
#include "gfx_drawing.hpp"
#include "stream.hpp"
#include "gfx_color_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace espidf;
using namespace io;
using namespace gfx;

#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_15

#define LCD_WIDTH 8
#define LCD_HEIGHT 8

spi_master spi_host(nullptr,
                LCD_HOST,
                PIN_NUM_CLK,
                PIN_NUM_MISO,
                PIN_NUM_MOSI,
                GPIO_NUM_NC,
                GPIO_NUM_NC,
                1024+8, // we don't need much DMA for this display
                DMA_CHAN);

using matrix_type = max7219<LCD_WIDTH,LCD_HEIGHT,LCD_HOST,PIN_NUM_CS> ;
using matrix_color = color<typename matrix_type::pixel_type>;
matrix_type matrix;
void dump_frame_buffer() {
    
    for(int y = 0;y<matrix.height;++y) {
        for(int x = 0;x<matrix.width;x+=8) {
            const uint8_t* p = matrix.frame_buffer()+(y*matrix.width/8)+(x/8);
            for(int j=0;j<8;++j) {
                printf("%c",((1<<(7-j))&*p)?'#':' ');
            }
        }
        printf("\r\n");
    }
}
void app_main() {
    // check to make sure SPI was initialized successfully
    if(!spi_host.initialized()) {
        printf("SPI host initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
    matrix_type::result r= matrix.initialize();
    if(matrix_type::result::success!=r) {
        printf("Could not initialize driver.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "ESP32 GFX Demo - MAX7219 **        ";
    ssize16 sz = f.measure_text(ssize16(strlen(text)*f.average_width(),f.height()),text);
    while(true) {
        for(int i = 0;i<sz.width;++i) {
            draw::suspend(matrix);
            //draw::filled_rectangle(matrix,(srect16)matrix.bounds(),matrix_color::black);
            draw::text(matrix,sz.bounds().offset(-i,0),text,f,matrix_color::white,matrix_color::black,false,4);
            draw::resume(matrix);
            vTaskDelay(1);
        }
    }
}