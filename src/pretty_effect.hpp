/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include <stdint.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "gfx_pixel.hpp"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "decode_image.hpp"

uint16_t **pixels;

//Grab a rgb16 pixel from the esp32_tiles image
static inline uint16_t get_bgnd_pixel(int x, int y)
{
    //Image has an 8x8 pixel margin, so we can also resolve e.g. [-3, 243]
    const uint16_t wpx = pixels[y+8][x+8];
    return wpx;
    
}
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
//esp32s2/c3 doesn't have enough memory to hold the decoded image, calculate instead
static inline uint16_t get_bgnd_pixel(int x, int y)
{
    return ((x<<3)^(y<<3)^(x*y));
}
#endif

//This variable is used to detect the next frame.
static int prev_frame=-1;

//Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
//these as we go through all the pixels in the frame. This is much, much faster.
static int8_t xofs[320], yofs[240];
static int8_t xcomp[320], ycomp[240];

//Calculate the pixel data for a set of lines (with implied line size of 320). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.
/**
 * @brief Calculate the effect for a bunch of lines.
 *
 * @param dest Destination for the pixels. Assumed to be LINECT * 320 16-bit pixel values.
 * @param line Starting line of the chunk of lines.
 * @param frame Current frame, used for animation
 * @param linect Amount of lines to calculate
 */
void pretty_effect_calc_lines(uint16_t *dest, int line, int frame, int linect)
{
    if (frame!=prev_frame) {
        //We need to calculate a new set of offset coefficients. Take some random sines as offsets to make everything
        //look pretty and fluid-y.
        for (int x=0; x<320; x++) xofs[x]=sin(frame*0.15+x*0.06)*4;
        for (int y=0; y<240; y++) yofs[y]=sin(frame*0.1+y*0.05)*4;
        for (int x=0; x<320; x++) xcomp[x]=sin(frame*0.11+x*0.12)*4;
        for (int y=0; y<240; y++) ycomp[y]=sin(frame*0.07+y*0.15)*4;
        prev_frame=frame;
    }
    for (int y=line; y<line+linect; y++) {
        for (int x=0; x<320; x++) {
            *dest++=get_bgnd_pixel(x+yofs[y]+xcomp[x], y+xofs[x]+ycomp[y]);
        }
    }
}

/**
 * @brief Initialize the effect
 *
 * @return ESP_OK on success, an error from the jpeg decoder otherwise.
 */
gfx::gfx_result pretty_effect_init(void)
{
#ifdef CONFIG_IDF_TARGET_ESP32
    return decode_image(&pixels);
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
    //esp32s2/c3 doesn't have enough memory to hold the decoded image, calculate instead
    return gfx::gfx_result::success;
#endif
}