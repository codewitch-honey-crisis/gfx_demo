#include <Arduino.h>
#include <SPIFFS.h>
#include <tft_io.hpp>
#include <waveshare1in54bv2.hpp>
#include <gfx_cpp14.hpp>
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace arduino;
using namespace gfx;

#define EPD_HOST    VSPI
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_WAIT 15
#define PIN_NUM_RST  4
#define PIN_NUM_DC 2

using epd_bus = tft_spi_ex<EPD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0>;
waveshare1in54bv2<PIN_NUM_DC,PIN_NUM_RST,PIN_NUM_WAIT,epd_bus,16> epd;
using epd_color = color<rgb_pixel<16>>;

void lines_demo() {
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd_color::white);
  const char* text = "GFX";
  SPIFFS.begin(false);
  File file = SPIFFS.open("/Maziro.ttf","rb");
  file_stream fs(file);
  open_font f;
  open_font::open(&fs,&f);
  float scale = f.scale(40);
  srect16 text_rect = f.measure_text((ssize16)epd.dimensions(),{0,0},
                          text,scale).bounds();
  draw::text(epd,
          text_rect.center((srect16)epd.bounds()),
          {0,0},
          text,
          f,scale,
          epd_color::red,epd_color::white,false,true /* anti-aliasing doesn't work great with this display due to no grays*/);
  file.close();
  for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),rgb_pixel<16>(10,0,0));
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd_color::red);
      draw::line(epd,srect16(0,r.y2,r.x1,0),rgb_pixel<16>(15,31,15));
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd_color::black);
      
  }
  draw::resume(epd);
}
void setup() {
    Serial.begin(115200);
    SPIFFS.begin(false);

    // with e-paper displays it's a good idea to
    // suspend for the entire frame because the 
    // refresh rate is terribly slow.
    while(true) {
        draw::suspend(epd);
        draw::filled_rectangle(epd,(srect16)epd.bounds(),color_max::white);
        rect16 image_bounds(0,0,199,199);
        rect16 crop_bounds(0,0,199,199);
        File fs = SPIFFS.open("/image_200.jpg");
        draw::image(epd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        const font& f = Bm437_Acer_VGA_8x8_FON;
        const char* text = "GFX Demo by\r\n honey the\r\n codewitch";
        ssize16 fd=f.measure_text({200,200},text);
        srect16 tr=srect16(spoint16(0,epd.height-f.height()*3-1),fd).center_horizontal((srect16)epd.bounds());
        draw::text(epd,tr,text,f,color_max::black);
        draw::resume(epd);
        delay(15000);
        lines_demo();
        delay(15000);
        draw::suspend(epd);
        image_bounds = rect16(0,0,335,255);
        fs = SPIFFS.open("/image2.jpg");
        draw::image(epd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        draw::resume(epd);
        delay(15000);
    }
}
void loop() {

}