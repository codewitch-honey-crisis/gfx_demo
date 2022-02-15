#include <Arduino.h>

//#include "arduino/drivers/ed047tc1.hpp"
#include <gfx_cpp14.hpp>
//using namespace arduino;
using namespace gfx;
/* Config Reggister Control */
#define PIN_NUM_DC 23
#define PIN_NUM_CLK 18
#define PIN_NUM_WR 0

/* Control Lines */
//#define CKV GPIO_NUM_25
//#define STH GPIO_NUM_26

/* Edges */
#define CKH GPIO_NUM_5

/* Data Lines */
#define PIN_NUM_D7 22
#define PIN_NUM_D6 21
#define PIN_NUM_D5 27
#define PIN_NUM_D4 2
#define PIN_NUM_D3 19
#define PIN_NUM_D2 4
#define PIN_NUM_D1 32
#define PIN_NUM_D0 33
//using ep_type = ed047tc1<PIN_NUM_D0,PIN_NUM_D1,PIN_NUM_D2,PIN_NUM_D3,PIN_NUM_D4,PIN_NUM_D5,PIN_NUM_D6,PIN_NUM_D7,PIN_NUM_CLK,PIN_NUM_WR,PIN_NUM_DC>;
//ep_type ep;

// better to keep these global rather than waste stack
font my_font;

void setup() {
    Serial.begin(115200);

}

void loop() {

}