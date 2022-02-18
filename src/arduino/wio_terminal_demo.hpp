#include <Arduino.h>
#include "gfx_cpp14.hpp"
#include "arduino/drivers/common/tft_io.hpp"
#include "arduino/drivers/ili9341.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

using bus_type = tft_spi<3,LCD_SS_PIN,SPI_MODE0>;
using lcd_type = ili9341<LCD_DC,LCD_RESET,LCD_BACKLIGHT,bus_type,3,true,400,200>;

using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd;

// prints a source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;+=x!1%$O@#";
    using gsc4 = pixel<channel_traits<channel_name::L,4>>;
    for(int y = 0;y<src.dimensions().height;++y) {
        for(int x = 0;x<src.dimensions().width;++x) {
            typename Source::pixel_type px;
            src.point(point16(x,y),&px);
            const auto px2 = convert<typename Source::pixel_type,gsc4>(px);
            size_t i =px2.template channel<0>();
            printf("%c",col_table[i]);
        }
        printf("\r\n");
    }
}
static const size16 bmp_size(32,32);
using bmp_type = bitmap<typename lcd_type::pixel_type>;
using bmp_color = color<typename lcd_type::pixel_type>;

// declare the bitmap
uint8_t bmp_buf[2048];
bmp_type bmp(bmp_size,bmp_buf);

// produced by request
void scroll_text_demo() {
    lcd.clear(lcd.bounds());
    
    // draw stuff
    bmp.clear(bmp.bounds()); // comment this out and check out the uninitialized RAM. It looks neat.

    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1)/(4/3.0));
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
    draw::bitmap(lcd,(srect16)bmp.bounds().center_horizontal(lcd.bounds()),bmp,bmp.bounds());
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "Copyright (C) 2022\r\nby honey the codewitch";
    ssize16 text_size = f.measure_text((ssize16)lcd.dimensions(),text);
    srect16 text_rect = srect16(spoint16((lcd.dimensions().width-text_size.width)/2,(lcd.dimensions().height-text_size.height)/2),text_size);
    int16_t text_start = text_rect.x1;
    bool first=true;
    print_source(bmp);
    while(true) {

       draw::filled_rectangle(lcd,text_rect,lcd_color::black);
        if(text_rect.x2>=lcd.dimensions().width) {
           draw::filled_rectangle(lcd,text_rect.offset(-lcd.dimensions().width,0),lcd_color::black);
        }

        text_rect=text_rect.offset(1,0);
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
    }
}
void lines_demo() {
    const font& f = Bm437_Acer_VGA_8x8_FON;
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const char* text = "Wio GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();

    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            lcd_color::dark_blue);

    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd.dimensions().width/100.0),
                i*(lcd.dimensions().height/100.0),
                lcd.dimensions().width-i*(lcd.dimensions().width/100.0)-1,
                lcd.dimensions().height-i*(lcd.dimensions().height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd.dimensions().height-1),lcd_color::light_blue);
        draw::line(lcd,srect16(r.x2,0,lcd.dimensions().width-1,r.y2),lcd_color::hot_pink);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::pale_green);
        draw::line(lcd,srect16(lcd.dimensions().width-1,r.y1,r.x2,lcd.dimensions().height-1),lcd_color::yellow);
        
    }
}

void setup() {
  Serial.begin(115200);
  
}

void loop() {
  lines_demo();
  scroll_text_demo();
}