
#define EPD_HOST    VSPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_WAIT 15

#include <Arduino.h>
#include <SPIFFS.h>
#include <tft_io.hpp>
#include <waveshare2in13b.hpp>
#include <gfx_cpp14.hpp>
using namespace arduino;
using namespace gfx;
using epd_bus = tft_spi_ex<EPD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0>;
waveshare2in13b<PIN_NUM_DC,PIN_NUM_RST,PIN_NUM_WAIT,epd_bus,16> epd;

void setup() {
  Serial.begin(115200);
  if(gfx_result::success!=epd.initialize()) {
    Serial.println("Error initializing");
  }

}

void loop() {
  using epd0_color = color<rgb_pixel<16>>;
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd0_color::white);
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
          epd0_color::red,epd0_color::white,false,true /* anti-aliasing doesn't work great with this display due to no grays*/);
  file.close();
  for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),rgb_pixel<16>(10,0,0));
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd0_color::red);
      draw::line(epd,srect16(0,r.y2,r.x1,0),rgb_pixel<16>(15,31,15));
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd0_color::black);
      
  }
  draw::resume(epd);
  epd.sleep(); // saves power, but takes longer to draw the next frame
  delay(2000);
  epd.dithering(!epd.dithering());
  
}