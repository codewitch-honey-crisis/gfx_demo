#include <Arduino.h>
#include <SPIFFS.h>
#include "drivers/depg0290b.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
#ifdef T5_22
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_BUSY 4
#define PIN_NUM_RST  12
#define PIN_NUM_DC 19
#else 
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_BUSY 15
#define PIN_NUM_RST  4
#define PIN_NUM_DC 2
#endif

#define LCD_WIDTH 128
#define LCD_HEIGHT 296

SPIClass spi(LCD_HOST);

using lcd_type = depg0290b<PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BUSY,
                        8>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd(spi);
void lines_demo() {
    draw::suspend(lcd);
    const font& f = Bm437_Acer_VGA_8x8_FON;
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const char* text = "GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();

    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            lcd_color::black);

    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd_type::width/100.0),
                i*(lcd_type::height/100.0),
                lcd_type::width-i*(lcd_type::width/100.0)-1,
                lcd_type::height-i*(lcd_type::height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd_type::height-1),lcd_color::red);
        draw::line(lcd,srect16(r.x2,0,lcd_type::width-1,r.y2),lcd_color::red);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::black);
        draw::line(lcd,srect16(lcd_type::width-1,r.y1,r.x2,lcd_type::height-1),lcd_color::black);
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
        draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
        rect16 image_bounds(0,0,335,255);
        rect16 crop_bounds(0,0,127,127);
        File fs = SPIFFS.open("/image3.jpg");
        draw::image(lcd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        fs = SPIFFS.open("/image3.jpg");
        draw::image(lcd,{0,lcd.height-128,127,lcd.height-1},&fs,crop_bounds.center(image_bounds));
        fs.close();
        const font& f = Bm437_Acer_VGA_8x8_FON;
        const char* text = "GFX Demo by\r\n honey the\r\n codewitch";
        ssize16 fd=f.measure_text({128,128},text);
        draw::text(lcd,fd.bounds().center((srect16)lcd.bounds()),text,f,lcd_color::black);
        draw::resume(lcd);
        delay(5000);
        lines_demo();
        delay(5000);
    }
}
void loop() {

}