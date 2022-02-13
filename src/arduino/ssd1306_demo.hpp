#include <Arduino.h>
#include "drivers/common/tft_io.hpp"
#include "drivers/ssd1306.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ATI_9x16.h"
using namespace arduino;
using namespace gfx;

// ensure the following is configured for your setup
#ifdef I2C
#define LCD_PORT 0
#define PIN_NUM_SDA 21
#define PIN_NUM_SCL 22
#define PIN_NUM_RST -1
#define PIN_NUM_DC -1
#define I2C_FREQ 400000
#else
#define LCD_HOST VSPI
#define PIN_NUM_CS 5
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO -1
#define PIN_NUM_CLK 18
#define PIN_NUM_DC 2
#define PIN_NUM_RST 4
#endif


#define LCD_WIDTH 128
#define LCD_HEIGHT 64


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

#ifdef I2C
using bus_type = tft_i2c<LCD_PORT,0x3C,PIN_NUM_SDA,PIN_NUM_SCL,0x0,0x40,I2C_FREQ>;
#else
using bus_type = tft_spi<LCD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0,20*1000*1000,20*1000*1000,false
#ifdef OPTIMIZE_DMA
,(LCD_WIDTH*LCD_HEIGHT)/8+8
#endif
>;
#endif

using lcd_type = ssd1306<LCD_WIDTH,LCD_HEIGHT,bus_type,LCD_VDC_3_3,PIN_NUM_DC,PIN_NUM_RST,true>;
lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

using bmp_type = bitmap<rgb_pixel<16>>;
using bmp_color = color<typename bmp_type::pixel_type>;
// declare the bitmap
constexpr static const size16 bmp_size(16,16);
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size,bmp_buf);
void bmp_demo() {
    lcd.clear(lcd.bounds());
    
    // draw stuff

    // fill with the transparent color
    typename bmp_type::pixel_type tpx = bmp_color::cyan;
    bmp.fill(bmp.bounds(),tpx);

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
    lcd.clear(lcd.bounds());
    // now we're going to draw the bitmap to the lcd instead, animating it
    int i=0;
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
        draw::bitmap(lcd,r,bmp,bmp.bounds(),bitmap_resize::crop,&tpx);
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
       
    }
}

void lines_demo() {
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::black);
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "ESP32 GFX";
    srect16 text_rect = srect16(spoint16(0,0),
                            f.measure_text((ssize16)lcd.dimensions(),
                            text));
                            
    draw::text(lcd,text_rect.center((srect16)lcd.bounds()),text,f,lcd_color::white);

    for(int i = 1;i<100;i+=10) {
#ifdef SUSPEND_RESUME
        draw::suspend(lcd);
#endif
        // calculate our extents
        srect16 r(i*(lcd_type::width/100.0),
                i*(lcd_type::height/100.0),
                lcd_type::width-i*(lcd_type::width/100.0)-1,
                lcd_type::height-i*(lcd_type::height/100.0)-1);

        draw::line(lcd,srect16(0,r.y1,r.x1,lcd_type::height-1),lcd_color::white);
        draw::line(lcd,srect16(r.x2,0,lcd_type::width-1,r.y2),lcd_color::white);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::white);
        draw::line(lcd,srect16(lcd_type::width-1,r.y1,r.x2,lcd_type::height-1),lcd_color::white);
#ifdef SUSPEND_RESUME
        draw::resume(lcd);
#endif
        
    }
    
    delay(500);
}
void intro() {
    
#ifdef SUSPEND_RESUME
    lcd.suspend();
#endif

    lcd.clear(lcd.bounds());
    const char* text = "presenting...";
    const font& f = Bm437_ATI_9x16_FON;
    srect16 sr = f.measure_text((ssize16)lcd.dimensions(),text).bounds();
    sr=sr.center((srect16)lcd.bounds());
    draw::text(lcd,sr,text,f,lcd_color::white);
#ifdef SUSPEND_RESUME
    lcd.resume();
#endif
    vTaskDelay(1500/portTICK_PERIOD_MS);
#ifdef SUSPEND_RESUME
    lcd.suspend();
#endif
    for(int i=0;i<lcd.dimensions().width;i+=4) {
        draw::line(lcd,srect16(i,0,i+1,lcd.dimensions().height-1),lcd_color::white);
        delay(50);
    }
#ifdef SUSPEND_RESUME
    lcd.resume();
#endif
    delay(100);
    
#ifdef SUSPEND_RESUME
    lcd.suspend();
#endif
    for(int i=2;i<lcd.dimensions().width;i+=4) {
        draw::line(lcd,srect16(i,0,i+1,lcd.dimensions().height-1),lcd_color::white);
        delay(50);
    }
#ifdef SUSPEND_RESUME
    lcd.resume();
#endif
    delay(1000);
}
void setup() {
    Serial.begin(115200);
    intro();
    while(true) {
        lines_demo();
        scroll_text_demo();
        bmp_demo();
    }
}
void loop() {

}
