#include <Arduino.h>
#include <SPIFFS.h>
#include "drivers/common/tft_io.hpp"
#include "gfx_cpp14.hpp"
#include "drivers/max7219.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   15

#define LCD_WIDTH 32
#define LCD_HEIGHT 8

using bus_type = tft_spi<LCD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,0,false>;
using matrix_type = max7219<LCD_WIDTH/8,LCD_HEIGHT/8,PIN_NUM_CS,bus_type> ;
using matrix_color = color<typename matrix_type::pixel_type>;
using bmp_type = bitmap<typename matrix_type::pixel_type>;
    
matrix_type matrix;
void setup() {
    Serial.begin(115200);
    
    const char* text = "ESP32 GFX Demo - MAX7219 ** Copyright (C) 2021 by honey the codewitch   ";
    size_t len = strlen(text);
    const font& f= Bm437_Acer_VGA_8x8_FON;
    size16 bmp_size = size16(f.measure_text({int16_t(len*f.average_width()),(int16_t)f.height()}, text).width,matrix.dimensions().height);
    uint8_t* bmp_buf = (uint8_t*)malloc(bmp_type::sizeof_buffer(bmp_size));
    bmp_type bmp(bmp_size,bmp_buf);
    bmp.clear(bmp.bounds());
    int cy = (bmp.dimensions().height-f.height())/2;
    draw::text(bmp,srect16(spoint16(0,cy),ssize16(bmp.dimensions().width,f.height())),text,f,matrix_color::white);
    int x = 0;
    while(true) {
        draw::bitmap(matrix,srect16(spoint16(0,0),(ssize16)matrix.dimensions()),bmp,rect16(point16(x,0),matrix.dimensions()));
        delay(50);
        if( ++x > bmp.dimensions().width ) {
            x = 0;
        }
    }    
}
void loop() {
    

}