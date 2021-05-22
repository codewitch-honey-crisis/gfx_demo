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
// prints a source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;*+!=1%O@$#";
    using gsc4 = pixel<channel_traits<channel_name::L,4>>;
    for(int y = 0;y<src.dimensions().height;++y) {
        for(int x = 0;x<src.dimensions().width;++x) {
            typename Source::pixel_type px;
            src.point(point16(x,y),&px);
            const auto px2 = px.template convert<gsc4>();
            size_t i =px2.template channel<0>();
            printf("%c",col_table[i]);
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
    const char* text = "ESP32 GFX Demo - MAX7219 **     ";
    size_t len = strlen(text);
    using bmp_type = bitmap<typename matrix_type::pixel_type>;
    const font& f= Bm437_Acer_VGA_8x8_FON;
    size16 bmp_size = (size16)f.measure_text({int16_t(len*f.average_width()),(int16_t)f.height()}, text);
    uint8_t* bmp_buf = (uint8_t*)malloc(bmp_type::sizeof_buffer(bmp_size));
    bmp_type bmp(bmp_size,bmp_buf);
    bmp.clear(bmp.bounds());
    draw::text(bmp,(srect16)bmp.bounds(),text,f,matrix_color::white);
    print_source(bmp);   
    int x = 0;
    while(true) {
            draw::suspend(matrix);
            draw::bitmap(matrix,srect16(spoint16(0,0),(ssize16)matrix.dimensions()),bmp,rect16(point16(x,0),matrix.dimensions()));
            draw::resume(matrix);
            vTaskDelay(50/portTICK_PERIOD_MS);
        	if( ++x > bmp.dimensions().width ) {
                x = 0;
            }
    }
    
}