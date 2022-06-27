#if defined(ARDUINO)
    #if defined(ILI9341)
        #include "arduino/ili9341_demo.hpp"
    #elif defined(ST7789)
        #include "arduino/st7789_demo.hpp"
    #elif defined(SSD1306)
        #include "arduino/ssd1306_demo.hpp"
    #elif defined(SSD1351)
        #include "arduino/ssd1351_demo.hpp"
    #elif defined(ST7735)
        #include "arduino/st7735_demo.hpp"
    #elif defined(MAX7219)
        #include "arduino/max7219_demo.hpp"
    #elif defined(RA8875)
        #include "arduino/ra8875_demo.hpp"
    #elif defined(WS5IN65)
        #include "arduino/waveshare5in65_demo.hpp"
    #elif defined(WS2IN13)
        #include "arduino/waveshare2in13_demo.hpp"
    #elif defined(WS1IN54BV2)
        #include "arduino/waveshare1in54bv2_demo.hpp"
    #elif defined(WS4IN2GSC)
        #include "arduino/waveshare4in2gsc_demo.hpp"
    #elif defined(TFT_ESPI)
        #include "arduino/tft_espi_demo.hpp"
    #elif defined(WIO_TERMINAL)
        #include "arduino/wio_terminal_demo.hpp"
    #endif
#elif defined(FRAMEWORK_ESP_IDF)
    #if defined(ILI9341)
        #include "esp-idf/ili9341_demo.hpp"
    #elif defined(SSD1306)
        #include "esp-idf/ssd1306_demo.hpp"
    #elif defined(ST7789)
        #include "esp-idf/st7789_demo.hpp"
    #elif defined(SSD1351)
        #include "esp-idf/ssd1351_demo.hpp"
    #elif defined(ST7735)
        #include "esp-idf/st7735_demo.hpp"
    #elif defined(MAX7219)
        #include "esp-idf/max7219_demo.hpp"
    #elif defined(GDEH0154Z90)
        #include "esp-idf/gdeh0154z90_demo.hpp"
    #elif defined(DEPG0290B)
        // not working yet
        #include "esp-idf/depg0290b_demo.hpp"
    #elif defined(RA8875)
        // not working yet
        #include "esp-idf/ra8875_demo.hpp"
    #endif
#elif defined(DIRECTX)
    #include "windows/directx_demo.hpp"
#endif
