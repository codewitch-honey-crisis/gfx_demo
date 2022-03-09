#ifndef BOARD_HAS_PSRAM
static_assert(false,"PSRAM must be enabled to use this device");
#endif

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
#include "drivers/common/tft_io.hpp"
#include "drivers/waveshare5in65.hpp"
#include <gfx_cpp14.hpp>
using namespace arduino;
using namespace gfx;
using epd_bus = tft_spi_ex<EPD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0>;
// ::ps_malloc means use PSRAM for framebuffer - this is required:
waveshare5in65<PIN_NUM_DC,PIN_NUM_RST,PIN_NUM_WAIT,epd_bus> epd(::ps_malloc);

void setup() {
  Serial.begin(115200);
  

}

void loop() {
  using epd0_color = color<rgb_pixel<16>>;
  // use indexed colors:
  //using epd0ipt = decltype(epd)::pixel_type;
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd0_color::white);
  const char* text = "ESP32 GFX";
  SPIFFS.begin(false);
  File file = SPIFFS.open("/Maziro.ttf","rb");
  file_stream fs(file);
  open_font f;
  open_font::open(&fs,&f);
  float scale = f.scale(60);
  srect16 text_rect = f.measure_text((ssize16)epd.dimensions(),{0,0},
                          text,scale).bounds();
  draw::text(epd,
          text_rect.center((srect16)epd.bounds()),
          {0,0},
          text,
          f,scale,
          epd0_color::blue,epd0_color::white,false,true /* anti-aliasing doesn't work great with this display due to no grays*/);
  file.close();
  for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),epd0_color::blue);
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd0_color::red);
      draw::line(epd,srect16(0,r.y2,r.x1,0),epd0_color::yellow);
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd0_color::orange);
      
  }
  /* // use indexed colors:
    for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),epd0ipt(2));
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd0ipt(3));
      draw::line(epd,srect16(0,r.y2,r.x1,0),epd0ipt(4));
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd0ipt(6));
      
  }
  */
  draw::resume(epd);
  epd.sleep(); // saves power, but takes longer to draw the next frame
  delay(2000);
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd0_color::black);
  
  for(int y = 0;y<epd.dimensions().height;y+=16) {
      for(int x = 0;x<epd.dimensions().width;x+=16) {
          if(0!=((x+y)%32)) {
              draw::filled_rectangle(epd,
                                  srect16(
                                      spoint16(x,y),
                                      ssize16(16,16)),
                                  epd0_color::white);
          }
      }
  }
  file = SPIFFS.open("/image.jpg");
  file_stream fs2(file);
  draw::image(epd,(srect16)epd.bounds(),&fs2);
  fs2.close();
  draw::resume(epd);
  delay(2000);
}