#include <Arduino.h>
#include <SPIFFS.h>
#ifdef PARALLEL8
#include "drivers/common/tft_parallel8.hpp"
#else
#include "drivers/common/tft_spi.hpp"
#endif
#include "drivers/st7789.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ACM_VGA_9x16.h"
#include "../fonts/Bm437_Verite_9x16.h"
#include "../fonts/Bm437_ATI_9x16.h"
#include "pretty_effect.hpp"
using namespace arduino;
using namespace gfx;

#if defined(ESP32_TTGO)
#define LCD_WIDTH 135
#define LCD_HEIGHT 240
#define LCD_HOST    VSPI
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   16
#define PIN_NUM_RST  23
#define PIN_NUM_BCKL 4
#elif defined(ESP32_TWATCH)
#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define LCD_HOST    VSPI
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   27
#define PIN_NUM_RST  -1
#define PIN_NUM_BCKL 12
#else
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_HOST    VSPI
#define DMA_CHAN    2
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 1
#endif

#define PARALLEL_LINES 16

// enable this to dump the jpeg images as ascii upon load
//#define ASCII_JPEGS
// To speed up transfers, every SPI transfer sends as much data as possible. 
#ifdef PARALLEL8
using bus_type = tft_p8<PIN_NUM_CS,PIN_NUM_WR,PIN_NUM_RD,PIN_NUM_D0,PIN_NUM_D1,PIN_NUM_D2,PIN_NUM_D3,PIN_NUM_D4,PIN_NUM_D5,PIN_NUM_D6,PIN_NUM_D7>;
#else
using bus_type = tft_spi<VSPI,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0,
true
#ifdef OPTIMIZE_DMA
,LCD_WIDTH*LCD_HEIGHT*2+8
#endif
>;
#endif
using lcd_type = st7789<LCD_WIDTH,LCD_HEIGHT,PIN_NUM_DC,PIN_NUM_RST,PIN_NUM_BCKL,bus_type,1>;

lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

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
    draw::bitmap(lcd,(srect16)bmp.bounds().center_horizontal(lcd.bounds()),bmp,bmp.bounds());
    const font& f = Bm437_ATI_9x16_FON;
    const char* text = "copyright (C) 2021\r\nby honey the codewitch";
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

    }
}
void lines_demo() {
    const font& f=Bm437_Verite_9x16_FON;
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const char* text = "ESP32 GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();

    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            lcd_color::dark_blue);

    for(int i = 1;i<100;++i) {
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

//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors()
{
    uint16_t *lines[2];
    //Allocate memory for the pixel buffers
    for (int i=0; i<2; i++) {
        lines[i]=(uint16_t*)heap_caps_malloc(lcd.dimensions().width*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(lines[i]!=NULL);
    }
    using lines_bmp_type = bitmap<typename lcd_type::pixel_type>;
    lines_bmp_type line_bmps[2] {
        lines_bmp_type(size16(lcd.dimensions().width,PARALLEL_LINES),lines[0]),
        lines_bmp_type(size16(lcd.dimensions().width,PARALLEL_LINES),lines[1])
    };
    
    int frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
 
#ifdef ASCII_JPEGS
    bool print=true;
#endif
    while(true) {
        if(0==frame%150) {
            lines_demo();
            scroll_text_demo();
        }
        ++frame;

        for (int y=0; y<lcd.dimensions().height; y+=PARALLEL_LINES) {
            //Calculate a line.
            pretty_effect_calc_lines(lcd.dimensions().width,lcd.dimensions().height, line_bmps[calc_line], y, frame, PARALLEL_LINES);
            // wait for the last frame to finish. Don't need this unless transactions are > 7
            if(-1!=sending_line)
                draw::wait_all_async(lcd);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            // draw::bitmap_async works better the larger the transfer size. Here ours is pretty big
            const lines_bmp_type& sending_bmp = line_bmps[sending_line];
            rect16 src_bounds = sending_bmp.bounds();
#ifdef ASCII_JPEGS
            if(print) {
                if(y+PARALLEL_LINES>=lcd.dimensions().height)
                    print=false;
                print_source(sending_bmp);
            }
#endif
            draw::bitmap_async(lcd,(srect16)src_bounds.offset(0,y),sending_bmp,src_bounds);
            //The line set is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line set as long as we do not
            //touch lines[sending_line] or the bitmap for it; 
            // the SPI sending process is still reading from that.
        }
        
        if(0==frame%50) {
            using lcd_color = color<rgb_pixel<16>>;
            int pid = (frame/50)%3;
            
            if(pid==1) {
                for(int i=0;i<60;++i) {
                    srect16 sr(spoint16(rand()%lcd.dimensions().width,rand()%lcd.dimensions().height),rand()%(lcd.dimensions().width/4));
                    draw::filled_ellipse(lcd, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));

                }
            } else if(pid==2) {
                for(int i=0;i<90;++i) {
                    srect16 sr(spoint16(rand()%lcd.dimensions().width,rand()%lcd.dimensions().height),rand()%(lcd.dimensions().width/4));
                    if(0!=(rand()%2)) {
                        draw::filled_rectangle(lcd, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    } else {
                        draw::filled_rounded_rectangle(lcd, sr,(rand()%10)/10.0,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    }
   
                }
            } else {
                for(int i = 1;i<120;++i) {
                    draw::line(lcd,srect16(0,i*(lcd.dimensions().height/240.0),lcd.dimensions().width-1,i*(lcd.dimensions().height/240.0)),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(i*(lcd.dimensions().width/240.0),0,i*(lcd.dimensions().width/240.0),lcd.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(lcd.dimensions().width-i*(lcd.dimensions().width/240.0)-1,0,lcd.dimensions().width-i*(lcd.dimensions().width/240.0)-1,lcd.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(0,lcd.dimensions().height-i*(lcd.dimensions().height/240.0)-1,lcd.dimensions().width-1,lcd.dimensions().height-i*(lcd.dimensions().height/240.0)-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
               
                }
                for(int i = 1;i<120;++i) {
                    draw::line(lcd,srect16(0,i*(lcd.dimensions().height/240.0),lcd.dimensions().width-1,i*(lcd.dimensions().height/240.0)),lcd_color::black);
                    draw::line(lcd,srect16(i*(lcd.dimensions().width/240.0),0,i*(lcd.dimensions().width/240.0),lcd.dimensions().height-1),lcd_color::black);
                    draw::line(lcd,srect16(lcd.dimensions().width-i*(lcd.dimensions().width/240.0)-1,0,lcd.dimensions().width-i*(lcd.dimensions().width/240.0)-1,lcd.dimensions().height-1),lcd_color::black);
                    draw::line(lcd,srect16(0,lcd.dimensions().height-i*(lcd.dimensions().height/240.0)-1,lcd.dimensions().width-1,lcd.dimensions().height-i*(lcd.dimensions().height/240.0)-1),lcd_color::black);
                  
                }
            }
            
            File fs = SPIFFS.open(
#if defined(ESP32_TTGO)
                "/image_240.jpg"
#elif defined(ESP32_TWATCH)
                "/image_240_240.jpg"
#else
                ((0==pid)?"/image.jpg":
                (1==pid)?"/image2.jpg":
                "/image3.jpg")
#endif
                );
                draw::image(pixels,(srect16)pixels.bounds(),&fs,rect16(0,0,-1,-1));
#ifdef ASCII_JPEGS
            print=true;
#endif
        }
    }
}


void setup() {
    Serial.begin(115200);
    SPIFFS.begin(false);
    pretty_effect_init(
#if defined(ESP32_TTGO)
    "/image_240.jpg",
    256,
    151,
#elif defined(ESP32_TWATCH)
    "/image_240_240.jpg",
    256,
    256,
#else
    "/image.jpg",
    336,
    256,
#endif
    lcd.dimensions().width,
    lcd.dimensions().height
);    
    display_pretty_colors();

}
void loop() {

}
