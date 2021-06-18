extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spiffs.h"

#include "drivers/gdeh0154z90.hpp"
#include "gfx_cpp14.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
using namespace espidf;
using namespace gfx;

#define LCD_HOST    VSPI_HOST
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_BUSY GPIO_NUM_15
#define PIN_NUM_RST  GPIO_NUM_4
#define PIN_NUM_DC GPIO_NUM_2

#define DMA_CHAN 2

// configure the spi bus. Must be done before the driver
spi_master spi_host(nullptr,
                    LCD_HOST,
                    PIN_NUM_CLK,
                    PIN_NUM_MISO,
                    PIN_NUM_MOSI,
                    GPIO_NUM_NC,
                    GPIO_NUM_NC,
                    4096+8,
                    DMA_CHAN);

using lcd_type = gdeh0154z90<LCD_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_DC,
                        PIN_NUM_RST,
                        PIN_NUM_BUSY>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd;
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

void app_main() {
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
    // with e-paper displays it's a good idea to
    // suspend for the entire frame because the 
    // refresh rate is terribly slow.
    while(true) {
        draw::suspend(lcd);
        draw::filled_rectangle(lcd,(srect16)lcd.bounds(),color_max::white);
        rect16 image_bounds(0,0,199,199);
        rect16 crop_bounds(0,0,199,199);
        file_stream fs("/spiffs/image_200.jpg");
        draw::image(lcd,(srect16)crop_bounds,&fs,crop_bounds.center(image_bounds));
        fs.close();
        const font& f = Bm437_Acer_VGA_8x8_FON;
        const char* text = "GFX Demo by\r\n honey the\r\n codewitch";
        ssize16 fd=f.measure_text({200,200},text);
        srect16 tr=srect16(spoint16(0,lcd.height-f.height()*3-1),fd).center_horizontal((srect16)lcd.bounds());
        tr.offset_inplace(1,1);
        draw::text(lcd,tr,text,f,color_max::red);
        tr.offset_inplace(-1,-1);
        draw::text(lcd,tr,text,f,color_max::black);
        draw::resume(lcd);
        vTaskDelay(30000/portTICK_PERIOD_MS);
        lines_demo();
        vTaskDelay(30000/portTICK_PERIOD_MS);
    }
}
