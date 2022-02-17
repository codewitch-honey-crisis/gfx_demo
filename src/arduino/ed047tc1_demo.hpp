#include <Arduino.h>

#include "arduino/drivers/ed047tc1.hpp"
#include <gfx_cpp14.hpp>
using namespace arduino;
using namespace gfx;

/* Config Register Control */
#define PIN_NUM_CFG_DATA 23
#define PIN_NUM_CFG_CLK 18
#define PIN_NUM_CFG_STR 0

/* Control Lines */
#define PIN_NUM_CKV 25
#define PIN_NUM_STH 26

/* Edges */
#define PIN_NUM_CKH 5
/* Data Lines */
#define PIN_NUM_D7 22
#define PIN_NUM_D6 21
#define PIN_NUM_D5 27
#define PIN_NUM_D4 2
#define PIN_NUM_D3 19
#define PIN_NUM_D2 4
#define PIN_NUM_D1 32
#define PIN_NUM_D0 33
using ep_type = ed047tc1<960,540,PIN_NUM_CFG_DATA,PIN_NUM_CFG_CLK,PIN_NUM_CFG_STR,PIN_NUM_CKV,PIN_NUM_STH,PIN_NUM_CKH,PIN_NUM_D0,PIN_NUM_D1,PIN_NUM_D2,PIN_NUM_D3,PIN_NUM_D4,PIN_NUM_D5,PIN_NUM_D6,PIN_NUM_D7>;
ep_type ep;

// better to keep these global rather than waste stack
font my_font;

void setup() {
    Serial.begin(115200);

}

void loop() {

}