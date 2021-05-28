extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_master.hpp"
#include "esp_spiffs.h"
#include "ssd1351.hpp"
#include "gfx_drawing.hpp"
#include "gfx_image.hpp"
#include "gfx_drawing.hpp"
#include "stream.hpp"
#include "gfx_color_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ACM_VGA_9x16.h"
#include "../fonts/Bm437_ATI_9x16.h"
#include "pretty_effect.hpp"
using namespace espidf;
using namespace io;
using namespace gfx;

#define LCD_WIDTH 128
#define LCD_HEIGHT 128
#define PARALLEL_LINES 16
#define LCD_HOST    VSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_DC   GPIO_NUM_2
#define PIN_NUM_RST  GPIO_NUM_4

// enable this to dump the jpeg images as ascii upon load
//#define ASCII_JPEGS
// To speed up transfers, every SPI transfer sends as much data as possible. 

// configure the spi bus. Must be done before the driver
spi_master spi_host(nullptr,
                    LCD_HOST,
                    PIN_NUM_CLK,
                    PIN_NUM_MISO,
                    PIN_NUM_MOSI,
                    GPIO_NUM_NC,
                    GPIO_NUM_NC,
                    PARALLEL_LINES*LCD_WIDTH*2+8,
                    DMA_CHAN);

// we use the default, modest buffer - it makes things slower but uses less
// memory. it usually works fine at default but you can change it for performance 
// tuning. It's the final parameter: Note that it shouldn't be any bigger than 
// the DMA size
using lcd_type = ssd1351<LCD_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST>;

lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

// prints a source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;*+!=1%O@$#";
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
    bmp.clear(bmp.bounds());
    
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
    const char* text = "(C) 2021\r\nby HTCW";
    ssize16 text_size = f.measure_text((ssize16)lcd.dimensions(),text);
    srect16 text_rect = srect16(spoint16((lcd_type::width-text_size.width)/2,(lcd_type::height-text_size.height)/2),text_size);
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
        vTaskDelay(1);
    }
}
void lines_demo() {
    const font& f = Bm437_Acer_VGA_8x8_FON;
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const char* text = "ESP32 GFX Demo";
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();

    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            lcd_color::dark_blue);

    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd_type::width/100.0),
                i*(lcd_type::height/100.0),
                lcd_type::width-i*(lcd_type::width/100.0)-1,
                lcd_type::height-i*(lcd_type::height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd_type::height-1),lcd_color::light_blue);
        draw::line(lcd,srect16(r.x2,0,lcd_type::width-1,r.y2),lcd_color::hot_pink);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::pale_green);
        draw::line(lcd,srect16(lcd_type::width-1,r.y1,r.x2,lcd_type::height-1),lcd_color::yellow);
        // the ESP32 wdt will get tickled
        // unless we do this:
        vTaskDelay(1);
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
            pretty_effect_calc_lines(lcd.dimensions().width,lcd.dimensions().height, lines[calc_line], y, frame, PARALLEL_LINES);
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
                    if(0==(i%3))
                        vTaskDelay(1);
                }
            } else if(pid==2) {
                for(int i=0;i<90;++i) {
                    srect16 sr(spoint16(rand()%lcd.dimensions().width,rand()%lcd.dimensions().height),rand()%(lcd.dimensions().width/4));
                    if(0!=(rand()%2)) {
                        draw::filled_rectangle(lcd, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    } else {
                        draw::filled_rounded_rectangle(lcd, sr,(rand()%10)/10.0,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    }
                    if(0==(i%5))
                        vTaskDelay(1);
                }
            } else {
                for(int i = 1;i<120;++i) {
                    draw::line(lcd,srect16(0,i*(lcd_type::height/240.0),lcd_type::width-1,i*(lcd_type::height/240.0)),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(i*(lcd_type::width/240.0),0,i*(lcd_type::width/240.0),lcd_type::height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(lcd_type::width-i*(lcd_type::width/240.0)-1,0,lcd_type::width-i*(lcd_type::width/240.0)-1,lcd_type::height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(lcd,srect16(0,lcd_type::height-i*(lcd_type::height/240.0)-1,lcd_type::width-1,lcd_type::height-i*(lcd_type::height/240.0)-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    if(0==(i%12))
                        vTaskDelay(1);
                }
                for(int i = 1;i<120;++i) {
                    draw::line(lcd,srect16(0,i*(lcd_type::height/240.0),lcd_type::width-1,i*(lcd_type::height/240.0)),lcd_color::black);
                    draw::line(lcd,srect16(i*(lcd_type::width/240.0),0,i*(lcd_type::width/240.0),lcd_type::height-1),lcd_color::black);
                    draw::line(lcd,srect16(lcd_type::width-i*(lcd_type::width/240.0)-1,0,lcd_type::width-i*(lcd_type::width/240.0)-1,lcd_type::height-1),lcd_color::black);
                    draw::line(lcd,srect16(0,lcd_type::height-i*(lcd_type::height/240.0)-1,lcd_type::width-1,lcd_type::height-i*(lcd_type::height/240.0)-1),lcd_color::black);
                    if(0==(i%12))
                        vTaskDelay(1);
                }
            }
            
            file_stream fs(
                "/spiffs/image_128.jpg"
                );
                
                gfx::jpeg_image::load(&fs,[](const typename gfx::jpeg_image::region_type& region,gfx::point16 location,void* state) {
                uint16_t** out = (uint16_t**)state;
                // to go as fast as possible, we access the bmp
                // as raw memory
                uint8_t *in = region.begin();
                gfx::rect16 r = region.bounds().offset(location.x,location.y);
                gfx::point16 pt;
                for (pt.y = r.y1; pt.y <= r.y2; ++pt.y) {
                    for (pt.x = r.x1; pt.x <= r.x2; ++pt.x) {
                        //We need to convert the 3 bytes in `in` to a rgb565 value.
                        // we could use convert<> and it's almost as efficient
                        // but it's actually more lines of code because we have to
                        // convert to and from raw values
                        // so we may as well just keep it raw
                        
                        uint16_t v = 0;
                        v |= ((in[0] >> 3) <<  11);
                        v |= ((in[1] >> 2) << 5);
                        v |= ((in[2] >> 3) );
                        //The LCD wants the 16-bit value in big-endian, so swap bytes
                        v=gfx::helpers::order_guard(v);
                        out[pt.y][pt.x] = v;
                        in+=3;
                    }
                }
                return gfx_result::success;

            },pixels);
#ifdef ASCII_JPEGS
            print=true;
#endif
        }
    }
}

void app_main(void)
{

    // check to make sure SPI was initialized successfully
    if(!spi_host.initialized()) {
        printf("SPI host initialization error.\r\n");
        abort();
    }
    // mount SPIFFS
    esp_err_t ret;
    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path="/spiffs";
    conf.format_if_mount_failed=false;
    conf.max_files=5;
    conf.partition_label="storage";
    ret=esp_vfs_spiffs_register(&conf);
    ESP_ERROR_CHECK(ret);   
    gfx_result rr;
    rr=pretty_effect_init(
    "/spiffs/image_128.jpg",
    144,
    144,
    lcd.dimensions().width,
    lcd.dimensions().height
);
    if(gfx_result::success!=rr) {
        printf("Error loading demo: %d\r\n",(int)rr);
    }
    display_pretty_colors();
}