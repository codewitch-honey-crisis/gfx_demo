#include <Arduino.h>
#include <SPIFFS.h>
#include "drivers/gdeh0154z90.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_BUSY 15
#define PIN_NUM_RST  4
#define PIN_NUM_DC 2

SPIClass spi(LCD_HOST);

using lcd_type = gdeh0154z90<PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BUSY
                        ,rgb_pixel<16>>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd(spi);
void lines_demo() {
    draw::suspend(lcd);
    const font& f = Bm437_Acer_VGA_8x8_FON;
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),color_max::white);
    const char* text = "GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();
    text_rect.offset_inplace(1,1);
    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            color_max::red);
    text_rect.offset_inplace(-1,-1);
    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            color_max::black);
    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd_type::width/100.0),
                i*(lcd_type::height/100.0),
                lcd_type::width-i*(lcd_type::width/100.0)-1,
                lcd_type::height-i*(lcd_type::height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd_type::height-1),color_max::black);
        draw::line(lcd,srect16(r.x2,0,lcd_type::width-1,r.y2),color_max::black);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),color_max::red);
        draw::line(lcd,srect16(lcd_type::width-1,r.y1,r.x2,lcd_type::height-1),color_max::red);
    }
    draw::resume(lcd);
}
void setup() {
    Serial.begin(115200);
    spi.begin(PIN_NUM_CLK,PIN_NUM_MISO,PIN_NUM_MOSI,PIN_NUM_CS);
    SPIFFS.begin(false);

    // with e-paper displays it's a good idea to
    // suspend for the entire frame because the 
    // refresh rate is terribly slow.
    while(true) {
        draw::suspend(lcd);
        draw::filled_rectangle(lcd,(srect16)lcd.bounds(),color_max::white);
        rect16 image_bounds(0,0,199,199);
        rect16 crop_bounds(0,0,199,199);
        File fs = SPIFFS.open("/image_200.jpg");
        draw::image(lcd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        const font& f = Bm437_Acer_VGA_8x8_FON;
        const char* text = "GFX Demo by\r\n honey the\r\n codewitch";
        ssize16 fd=f.measure_text({200,200},text);
        srect16 tr=srect16(spoint16(0,lcd.height-f.height()*3-1),fd).center_horizontal((srect16)lcd.bounds());
        draw::text(lcd,tr,text,f,color_max::black);
        draw::resume(lcd);
        delay(15000);
        lines_demo();
        delay(15000);
        draw::suspend(lcd);
        image_bounds = rect16(0,0,335,255);
        fs = SPIFFS.open("/image2.jpg");
        draw::image(lcd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        draw::resume(lcd);
        delay(15000);
    }
}
void loop() {

}