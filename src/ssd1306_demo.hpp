extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "i2c_master.hpp"
#include "esp_spiffs.h"
#include "ssd1306.hpp"
#include "gfx_drawing.hpp"
#include "gfx_image.hpp"
#include "gfx_drawing.hpp"
#include "stream.hpp"
#include "gfx_color_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ACM_VGA_9x16.h"
#include "../fonts/Bm437_ATI_9x16.h"
using namespace espidf;
using namespace io;
using namespace gfx;

// ensure the following is configured for your setup
#define LCD_PORT I2C_NUM_0
#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22
#define PIN_RST GPIO_NUM_NC
#define I2C_FREQ 400000
#define LCD_WIDTH 128
#define LCD_HEIGHT 64
i2c_master i2c(nullptr,LCD_PORT,PIN_SDA,PIN_SCL,true,true,I2C_FREQ);
//#define LCD_VDC_5
#if defined(LCD_VDC_5)
#define LCD_VDC_3_3 false
#else
#define LCD_VDC_3_3 true
#endif

// drivers that support double buffering expose it
// through suspend and resume calls
// GFX will use it automatically, but it can't know
// about your own series of line draws for example,
// only about individual lines, so if you want to
// extend the scope and extents of the buffered
// drawing you can use draw::suspend<>() and 
// draw::resume<>().

// Undefine this to see
// the difference
#define SUSPEND_RESUME

using lcd_type = ssd1306_i2c<LCD_WIDTH,LCD_HEIGHT,LCD_PORT,0x3C,LCD_VDC_3_3,PIN_RST>;

lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

using bmp_type = bitmap<typename lcd_type::pixel_type>;
using bmp_color = color<typename lcd_type::pixel_type>;
// declare the bitmap
static const size16 bmp_size(16,16);
uint8_t bmp_buf[16*16/8];
bmp_type bmp(bmp_size,bmp_buf);
void bmp_demo() {
    lcd.clear(lcd.bounds());
    
    // draw stuff
    bmp.clear(bmp.bounds()); // comment this out and check out the uninitialized RAM. It looks neat.

    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1));
    rect16 ubounds(0,0,bounds.x2,bounds.y2);

    // draw the face
    draw::filled_ellipse(bmp,bounds,bmp_color::yellow);
    
    // draw the left eye
    srect16 eye_bounds_left(spoint16(bounds.width()/5,bounds.height()/5),ssize16(bounds.width()/5,bounds.height()/3));
    draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
    
    // draw the right eye
    srect16 eye_bounds_right(
        spoint16(
            bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
            eye_bounds_left.y1
        ),eye_bounds_left.dimensions());
    draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
    
    // draw the mouth
    srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,-bounds.height()/8).normalize();
    // we need to clip part of the circle we'll be drawing
    srect16 mouth_clip(mouth_bounds.x1,mouth_bounds.y1+mouth_bounds.height()/(float)1.6,mouth_bounds.x2,mouth_bounds.y2);
    draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);
    int dx = 1;
    int dy=2;
    int i =0;
    srect16 r =(srect16)bmp.bounds().center(lcd.bounds());
    while(i<150) {
#ifdef SUSPEND_RESUME
        draw::suspend(lcd);
#endif
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
#ifdef SUSPEND_RESUME
        draw::resume(lcd);
#endif
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
#ifdef SUSPEND_RESUME
        draw::suspend(lcd);
#endif
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
#ifdef SUSPEND_RESUME
        draw::resume(lcd);
#endif
        if(!first && text_rect.x1>=text_start)
            break;
        vTaskDelay(1);
    }
}
void dump_frame_buffer() {
    
    for(int i=0;i<lcd_type::width*lcd_type::height/8;++i) {
        if(0==i%lcd_type::width) {
            printf("\r\n");
        }
        printf("%02X",(int)lcd.frame_buffer()[i]);
    }
    printf("\r\n");
}
void lines_demo() {
    lcd.clear(lcd.bounds());
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "ESP32 GFX";
    srect16 text_rect = srect16(spoint16(0,0),f.measure_text((ssize16)lcd.dimensions(),text));
    text_rect=text_rect.offset((lcd.dimensions().width-text_rect.width())/2,(lcd.dimensions().height-text_rect.height())/2);
    draw::text(lcd,text_rect,text,f,lcd_color::white);
    
    for(int i = 1;i<100;i+=10) {
#ifdef SUSPEND_RESUME
        draw::suspend(lcd);
#endif
        draw::line(lcd,srect16(0,i*(lcd.dimensions().height/100.0),i*(lcd.dimensions().width/100.0),lcd.dimensions().height-1),lcd_color::white);
        draw::line(lcd,srect16(lcd.dimensions().width-i*(lcd.dimensions().width/100.0)-1,0,lcd.dimensions().width-1,lcd.dimensions().height-i*(lcd.dimensions().height/100.0)-1),lcd_color::white);
        draw::line(lcd,srect16(0,lcd.dimensions().height-i*(lcd.dimensions().height/100.0),i*(lcd.dimensions().width/100.0)-1,0),lcd_color::white);
        draw::line(lcd,srect16(lcd.dimensions().width-1,i*(lcd.dimensions().height/100.0),lcd.dimensions().width-i*(lcd.dimensions().width/100.0)-1,lcd.dimensions().height-1),lcd_color::white);
        vTaskDelay(1);
#ifdef SUSPEND_RESUME
        draw::resume(lcd);
#endif
    }
    
    vTaskDelay(500/portTICK_PERIOD_MS);
}

void app_main(void)
{
    // check to make sure SPI was initialized successfully
    if(!i2c.initialized()) {
        printf("I2C host initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
    
    lcd_type::result r= lcd.initialize();
    
    if(lcd_type::result::success!=r) {
        printf("display initialization error.\r\n");
        vTaskDelay(portMAX_DELAY);
    }
#ifdef SUSPEND_RESUME
    lcd.suspend();
#endif
    lcd.clear(lcd.bounds());
    for(int i=0;i<lcd.dimensions().width;i+=4) {
        draw::line(lcd,srect16(i,0,i+1,lcd.dimensions().height-1),lcd_color::white);
    }
#ifdef SUSPEND_RESUME
    lcd.resume();
#endif
    vTaskDelay(100/portTICK_PERIOD_MS);
#ifdef SUSPEND_RESUME
    lcd.suspend();
#endif
    for(int i=2;i<lcd.dimensions().width;i+=4) {
        draw::line(lcd,srect16(i,0,i+1,lcd.dimensions().height-1),lcd_color::white);
    }
#ifdef SUSPEND_RESUME
    lcd.resume();
#endif
    while(true) {
       lines_demo();
       scroll_text_demo();
       bmp_demo();
    }
}