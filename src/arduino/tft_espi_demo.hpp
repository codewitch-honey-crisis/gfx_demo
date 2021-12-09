#include <User_Setup.h>
#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>
#include <gfx_cpp14.hpp>
#include "drivers/gfx_tft_espi.hpp"
#include "pretty_effect.hpp"
#include "../fonts/Bm437_ATI_9x16.h"
#include "../fonts/Bm437_Verite_9x16.h"
//#include "../fonts/Maziro.h"

using namespace gfx;
using namespace arduino;
using tft_type = gfx_tft_espi<true>;
using tft_color = color<typename tft_type::pixel_type>;
// create the TFT_eSPI driver so we wrap it.
TFT_eSPI tft_espi = TFT_eSPI();
tft_type tft(tft_espi);

// should be an even multiple of the screen height:
#define PARALLEL_LINES 16 

using frame_buffer_type = large_bitmap<rgb_pixel<16>>;
using fb_color = color<typename frame_buffer_type::pixel_type>;


// prints a source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;+=x!1%$O@#";
    for(int y = 0;y<src.dimensions().height;++y) {
        for(int x = 0;x<src.dimensions().width;++x) {
            typename Source::pixel_type px;
            src.point(point16(x,y),&px);
            const auto px2 = convert<typename Source::pixel_type,gsc_pixel<4>>(px);
            size_t i =px2.template channel<0>();
            char sz[2] = {col_table[i],0};
            Serial.print(sz);
        }
        Serial.println();
    }
}

constexpr static const size16 bmp_size(16,16);
// you can use YbCbCr for example. It's lossy, so you'll want extra bits
//using bmp_type = bitmap<ycbcr_pixel<HTCW_MAX_WORD>>;
using bmp_type = bitmap<typename tft_type::pixel_type,palette<typename tft_type::pixel_type,typename tft_type::pixel_type>>;
using bmp_color = color<typename bmp_type::pixel_type>;
using bmpa_pixel_type = rgba_pixel<HTCW_MAX_WORD>;
using bmpa_color = color<bmpa_pixel_type>;
// declare the bitmap
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size,bmp_buf);


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors()
{
    using lines_bmp_type = bitmap<typename tft_type::pixel_type>;
    lines_bmp_type line_bmps[2] {
        lines_bmp_type(size16(320,PARALLEL_LINES),malloc(320*PARALLEL_LINES*sizeof(uint16_t))),
        lines_bmp_type(size16(320,PARALLEL_LINES),malloc(320*PARALLEL_LINES*sizeof(uint16_t)))
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
        if(0!=(frame%10))
            vTaskDelay(1);
        for (int y=0; y<240; y+=PARALLEL_LINES) {
            //Calculate a line.
            pretty_effect_calc_lines(tft.dimensions().width,tft.dimensions().height, line_bmps[calc_line], y, frame, PARALLEL_LINES);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            const lines_bmp_type& sending_bmp = line_bmps[sending_line];
            rect16 src_bounds = sending_bmp.bounds();
#ifdef ASCII_JPEGS
            if(print) {
                if(y+PARALLEL_LINES>=tft.dimensions().height)
                    print=false;
                print_source(sending_bmp);
            }
#endif
            // TFT_eSPI supports asynchronous operations, but only one in the queue at a time.
            draw::wait_all_async();
            draw::bitmap_async(tft,(srect16)src_bounds.offset(0,y),sending_bmp,src_bounds);
            
        }
        if(0==frame%50) {
            using tft_color = color<rgb_pixel<16>>;
            int pid = (frame/50)%3;
            
            if(pid==1) {
                for(int i=0;i<60;++i) {
                    srect16 sr(spoint16(rand()%tft.dimensions().width,rand()%tft.dimensions().height),rand()%(tft.dimensions().width/4));
                    draw::filled_ellipse(tft, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                }
            } else if(pid==2) {
                for(int i=0;i<90;++i) {
                    srect16 sr(spoint16(rand()%tft.dimensions().width,rand()%tft.dimensions().height),rand()%(tft.dimensions().width/4));
                    if(0!=(rand()%2)) {
                        draw::filled_rectangle(tft, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    } else {
                        draw::filled_rounded_rectangle(tft, sr,(rand()%10)/10.0,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    }
                }
            } else {
                for(int i = 1;i<120;++i) {
                    draw::line(tft,srect16(0,i*(tft.dimensions().height/240.0),tft.dimensions().width-1,i*(tft.dimensions().height/240.0)),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(i*(tft.dimensions().width/240.0),0,i*(tft.dimensions().width/240.0),tft.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,0,tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,tft.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(0,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1,tft.dimensions().width-1,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                }
                for(int i = 1;i<120;++i) {
                    draw::line(tft,srect16(0,i*(tft.dimensions().height/240.0),tft.dimensions().width-1,i*(tft.dimensions().height/240.0)),tft_color::black);
                    draw::line(tft,srect16(i*(tft.dimensions().width/240.0),0,i*(tft.dimensions().width/240.0),tft.dimensions().height-1),tft_color::black);
                    draw::line(tft,srect16(tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,0,tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,tft.dimensions().height-1),tft_color::black);
                    draw::line(tft,srect16(0,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1,tft.dimensions().width-1,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1),tft_color::black);
                }
            }
            
            File fs = SPIFFS.open((0==pid)?"/image.jpg":(1==pid)?"/image2.jpg":"/image3.jpg");
            draw::image(pixels,(srect16)pixels.bounds(),&fs,rect16(0,0,-1,-1));
            fs.close();
#ifdef ASCII_JPEGS
            print=true;
#endif
        }
    }
}

void setup() {
    Serial.begin(115200);
    SPIFFS.begin(false);
    // tft_espi does not initialize automatically.
    tft_espi.begin();
    // tft_espi tends to start in portrait mode.
    // switch to landscape.
    tft_espi.setRotation(1);
    // if your screen size is different than 320x240 
    // you may want a different JPEG
    pretty_effect_init("/image.jpg",336,256,320,240);
    
    display_pretty_colors();

}
void loop() {

}

