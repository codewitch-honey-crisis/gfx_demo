/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <SPIFFS.h>
#else
#include <stdint.h>
#include <string.h>
#endif

#include "gfx_cpp14.hpp"
//Define the height and width of the jpeg file. Make sure this matches the actual jpeg
//dimensions.
#define IMAGE_W 336
#define IMAGE_H 256

using pixels_type = gfx::large_bitmap<gfx::rgb_pixel<16>>;
//using pixels_type = gfx::large_bitmap<gfx::indexed_pixel<4>,gfx::ega_palette<gfx::rgb_pixel<16>>>;
//using pixels_palette_type = typename pixels_type::palette_type;
//Decode the embedded image into pixel lines that can be used with the rest of the logic.
gfx::gfx_result decode_image(const char* image, uint16_t image_width, uint16_t image_height,pixels_type *pixels/*,pixels_palette_type* palette*/)
{
    
    *pixels = pixels_type(gfx::size16(image_width,image_height),1/*,palette*/);
    if(!pixels->initialized()) {
        return gfx::gfx_result::out_of_memory;
    }
    gfx::gfx_result ret = gfx::gfx_result::success;
    if(nullptr!=image) {
#ifndef ARDUINO
        gfx::file_stream fs(image);
        if(!fs.caps().read)
        {
            return gfx::gfx_result::io_error;
        }
#else
        File fs = SPIFFS.open(image);
#endif
        ret=gfx::draw::image(*pixels,(gfx::srect16)pixels->bounds(),&fs,gfx::rect16(0,0,-1,-1));
        if(gfx::gfx_result::success!=ret) {
            printf("Bad result %d\r\n",(int)ret);
            vTaskDelay(portMAX_DELAY);
        }
    }
    return ret;

}