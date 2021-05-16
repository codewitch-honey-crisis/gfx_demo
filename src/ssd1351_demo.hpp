extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "ssd1351.hpp"
#include "gfx_drawing.hpp"
#include "gfx_image.hpp"
#include "gfx_drawing.hpp"
#include "stream.hpp"
#include "gfx_color_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ATI_9x16.h"
using namespace espidf;
using namespace io;
using namespace gfx;

// the following is configured for the ESP-WROVER-KIT
// make sure to set the pins to your set up.
#define LCD_WIDTH 128
#define LCD_HEIGHT 128
#ifdef CONFIG_IDF_TARGET_ESP32
#if defined(ESP_WROVER_KIT)
#define PARALLEL_LINES 16
#define LCD_HOST    HSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_25
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_19
#define PIN_NUM_CS   GPIO_NUM_22

#define PIN_NUM_DC   GPIO_NUM_21
#define PIN_NUM_RST  GPIO_NUM_18
#define PIN_NUM_BCKL GPIO_NUM_5
#elif defined(ESP32_TTGO)
#define LCD_WIDTH 240
#define LCD_HEIGHT 135
#define PARALLEL_LINES 16
#define LCD_HOST    HSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_19
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_DC   GPIO_NUM_16
#define PIN_NUM_RST  GPIO_NUM_NC
#define PIN_NUM_BCKL GPIO_NUM_4
#else
#define PARALLEL_LINES 16
#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_DC   GPIO_NUM_2
#define PIN_NUM_RST  GPIO_NUM_4
#define PIN_NUM_BCKL GPIO_NUM_15
#endif

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define PARALLEL_LINES 16
#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    LCD_HOST

#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34

#define PIN_NUM_DC   GPIO_NUM_4
#define PIN_NUM_RST  GPIO_NUM_5
#define PIN_NUM_BCKL GPIO_NUM_6
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_ILI9341
#define PARALLEL_LINES 16
#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    LCD_HOST

#define PIN_NUM_MISO GPIO_NUM_2
#define PIN_NUM_MOSI GPIO_NUM_7
#define PIN_NUM_CLK  GPIO_NUM_6
#define PIN_NUM_CS   GPIO_NUM_10

#define PIN_NUM_DC   GPIO_NUM_9
#define PIN_NUM_RST  GPIO_NUM_18
#define PIN_NUM_BCKL GPIO_NUM_19
#endif


// enable this to dump the jpeg images as ascii upon load
//#define ASCII_JPEGS
// To speed up transfers, every SPI transfer sends as much data as possible. 

// configure the spi bus. Must be done before the driver
spi_master spi_host(nullptr,
                    LCD_HOST,
                    PIN_NUM_CLK,
                    PIN_NUM_MISO,
                    PIN_NUM_MOSI,
                    GPIO_NUM_NC,
                    GPIO_NUM_NC,
                    PARALLEL_LINES*LCD_WIDTH*2+8,
                    DMA_CHAN);

// we use the default, modest buffer - it makes things slower but uses less
// memory. it usually works fine at default but you can change it for performance 
// tuning. It's the final parameter: Note that it shouldn't be any bigger than 
// the DMA size
using lcd_type = ssd1351<LCD_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST>;



lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

using bmp_type = bitmap<typename lcd_type::pixel_type>;
using bmp_color = color<typename lcd_type::pixel_type>;
// declare the bitmap
constexpr static const size16 bmp_size(16,16);
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size,bmp_buf);
void bmp_demo() {
    lcd.clear(lcd.bounds());
    
    // draw stuff
    bmp.clear(bmp.bounds());

    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1));
    rect16 ubounds(0,0,bounds.x2,bounds.y2);

    // draw the face
    draw::filled_ellipse(bmp,bounds,bmp_color::yellow);
    
    // draw the left eye
    srect16 eye_bounds_left(spoint16(bounds.width()/5,
                                    bounds.height()/5),
                                    ssize16(bounds.width()/5,
                                            bounds.height()/3));
    draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
    
    // draw the right eye
    srect16 eye_bounds_right(
        spoint16(
            bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
            eye_bounds_left.y1
        ),eye_bounds_left.dimensions());
    draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
    
    // draw the mouth
    srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,
                                        -bounds.height()/8).normalize();
    // we need to clip part of the circle we'll be drawing
    srect16 mouth_clip(mouth_bounds.x1,
                    mouth_bounds.y1+mouth_bounds.height()/(float)1.6,
                    mouth_bounds.x2,
                    mouth_bounds.y2);

    draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);
    int dx = 1;
    int dy=2;
    int i =0;
    lcd.clear(lcd.bounds());
    // now we're going to draw the bitmap to the lcd instead, animating it
    i=0;
    srect16 r =(srect16)bmp.bounds().center(lcd.bounds());
    while(i<150) {
        draw::filled_rectangle(lcd,r,lcd_color::black);
        srect16 r2 = r.offset(dx,dy);
        if(!((srect16)lcd.bounds()).contains(r2)) {
            if(r2.x1<0 || r2.x2>lcd.bounds().x2)
                dx=-dx;
            if(r2.y1<0 || r2.y2>lcd.bounds().y2)
                dy=-dy;
            r=r.offset(dx,dy);
        } else
            r=r2;
        draw::bitmap(lcd,r,bmp,bmp.bounds());
        vTaskDelay(10/portTICK_PERIOD_MS);
        ++i;
        
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
}
// produced by request
void scroll_text_demo() {
    lcd.clear(lcd.bounds());
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "(C) 2021\r\nby HTCW";
    ssize16 text_size = f.measure_text((ssize16)lcd.dimensions(),text);
    srect16 text_rect = srect16(spoint16((lcd.dimensions().width-text_size.width)/2,(lcd.dimensions().height-text_size.height)/2),text_size);
    int16_t text_start = text_rect.x1;
    bool first=true;
    while(true) {
        draw::filled_rectangle(lcd,text_rect,lcd_color::black);
        if(text_rect.x2>=320) {
            draw::filled_rectangle(lcd,text_rect.offset(-lcd.dimensions().width,0),lcd_color::black);
        }

        text_rect=text_rect.offset(2,0);
        draw::text(lcd,text_rect,text,f,lcd_color::old_lace,lcd_color::black,false);
        if(text_rect.x2>=lcd.dimensions().width){
            draw::text(lcd,text_rect.offset(-lcd.dimensions().width,0),text,f,lcd_color::old_lace,lcd_color::black,false);
        }
        if(text_rect.x1>=lcd.dimensions().width) {
            text_rect=text_rect.offset(-lcd.dimensions().width,0);
            first=false;
        }
        if(!first && text_rect.x1>=text_start)
            break;
        vTaskDelay(1);
    }
}

void lines_demo() {
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "ESP32 GFX";
    srect16 text_rect = srect16(spoint16(0,0),
                            f.measure_text((ssize16)lcd.dimensions(),
                            text));
                            
    draw::text(lcd,text_rect.center((srect16)lcd.bounds()),text,f,lcd_color::dark_blue);
    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd_type::width/100.0),
                i*(lcd_type::height/100.0),
                lcd_type::width-i*(lcd_type::width/100.0)-1,
                lcd_type::height-i*(lcd_type::height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd_type::height-1),lcd_color::light_blue);
        draw::line(lcd,srect16(r.x2,0,lcd_type::width-1,r.y2),lcd_color::hot_pink);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::pale_green);
        draw::line(lcd,srect16(lcd_type::width-1,r.y1,r.x2,lcd_type::height-1),lcd_color::yellow);
        // the ESP32 wdt will get tickled
        // unless we do this:
        vTaskDelay(1);
    }
    
    vTaskDelay(500/portTICK_PERIOD_MS);
}

void app_main(void)
{
    // check to make sure host or port was initialized successfully
    if(!spi_host.initialized()) {
        printf("SPI host initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }

    while(true) {
     lines_demo();
     scroll_text_demo();
       bmp_demo();
    }
}