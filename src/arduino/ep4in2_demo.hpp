#include <Arduino.h>
#include <SPIFFS.h>
#include "drivers/ep4in2.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

#define LCD_HOST    VSPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_BUSY 15
#define PIN_NUM_RST  4
#define PIN_NUM_DC 2

SPIClass spi(LCD_HOST);

using epaper_driver_type = ep4in2<PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BUSY>;

epaper_driver_type epaper(spi);
template<typename Destination>
void lines_demo(Destination& target) {
    draw::suspend(target);
    const font& f = Bm437_Acer_VGA_8x8_FON;
    draw::filled_rectangle(target,(srect16)target.bounds(),color_max::white);
    const char* text = "GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)target.dimensions(),
                            text).bounds();

    draw::text(target,
            text_rect.center((srect16)target.bounds()),
            text,
            f,
            color_max::black);

    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(target.dimensions().width/100.0),
                i*(target.dimensions().height/100.0),
                target.dimensions().width-i*(target.dimensions().width/100.0)-1,
                target.dimensions().height-i*(target.dimensions().height/100.0)-1);
        // draw the four lines
        draw::line(target,srect16(0,r.y1,r.x1,target.dimensions().height-1),color_max::hot_pink);
        draw::line(target,srect16(r.x2,0,target.dimensions().width-1,r.y2),color_max::hot_pink);
        draw::line(target,srect16(0,r.y2,r.x1,0),color_max::black);
        draw::line(target,srect16(target.dimensions().width-1,r.y1,r.x2,target.dimensions().height-1),color_max::black);
    }

    draw::resume(target);
   
}
template<typename Destination>
void image_demo(Destination& target) {
        draw::suspend(target);
        draw::filled_rectangle(target,(srect16)target.bounds(),color_max::white);
        rect16 image_bounds(0,0,335,255);
        rect16 crop_bounds(0,0,127,127);
        File fs = SPIFFS.open("/image3.jpg");
        draw::image(target,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        fs = SPIFFS.open("/image3.jpg");
        draw::image(target,{0,target.height-128,127,target.height-1},&fs,crop_bounds.center(image_bounds));
        fs.close();
        fs = SPIFFS.open("/image3.jpg");
        draw::image(target,{target.width-128,0,target.width-1,127},&fs,crop_bounds.center(image_bounds));
        fs.close();
        fs = SPIFFS.open("/image3.jpg");
        draw::image(target,{target.width-128,target.height-128,target.width-1,target.height-1},&fs,crop_bounds.center(image_bounds));
        fs.close();
        
        const font& f = Bm437_Acer_VGA_8x8_FON;
        const char* text = "GFX Demo by\r\n honey the\r\n codewitch";
        ssize16 fd=f.measure_text({128,128},text);
        draw::text(target,fd.bounds().center((srect16)target.bounds()),text,f,color_max::black);
        draw::resume(target);
}
void setup() {
    Serial.begin(115200);
    spi.begin(PIN_NUM_CLK,PIN_NUM_MISO,PIN_NUM_MOSI,PIN_NUM_CS);
    SPIFFS.begin(false);
    
      
    // with e-paper displays it's a good idea to
    // suspend for the entire frame because the 
    // refresh rate is terribly slow.
    int i = 0;
    while(true) {
        auto mode2 = epaper.mode<2>(); // black and white
        for(int j=0;j<2;++j) {
            image_demo(mode2);
            delay(5000);
            lines_demo(mode2);
            delay(5000);
            ++i;
        }
        auto mode1 = epaper.mode<1>(); // black and white
        for(int j=0;j<2;++j) {
            image_demo(mode1);
            delay(5000);
            lines_demo(mode1);
            delay(5000);
            ++i;
        }
        auto mode1_4 = epaper.mode<1,4>(); // black and white with 4-bit grayscale virtualization
        // note that "mode1" from above is no longer valid ever since we switched
        for(int j=0;j<2;++j) {
            image_demo(mode1_4);
            delay(5000);
            lines_demo(mode1_4);
            delay(5000);
            ++i;
        }
        
    }
}
void loop() {

}