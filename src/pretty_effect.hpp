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
#include "gfx_bitmap.hpp"
#include "gfx_palette.hpp"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "decode_image.hpp"

using pixels_type = gfx::large_bitmap<gfx::rgb_pixel<16>>;
//using pixels_type = gfx::large_bitmap<gfx::indexed_pixel<4>,gfx::ega_palette<gfx::rgb_pixel<16>>>;
//using pixels_palette_type = typename pixels_type::palette_type;
//pixels_palette_type pixels_palette;
pixels_type pixels;
//Grab a rgb16 pixel from the esp32_tiles image
static inline typename gfx::rgb_pixel<16> get_bgnd_pixel(int x, int y)
{
    //Image has an 8x8 pixel margin, so we can also resolve e.g. [-3, 243]
    typename pixels_type::pixel_type px;
    pixels.point(gfx::point16(x+8,y+8),&px);
    gfx::rgb_pixel<16> result;
    // in case this is indexed, we need to convert it here
    // while we still have access to the palette
    convert_palette_to(pixels,px,&result);
    return result;
    
}
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
//esp32s2/c3 doesn't have enough memory to hold the decoded image, calculate instead
static inline pixels_type::pixel_type get_bgnd_pixel(int x, int y)
{
    typename pixels_type::pixel_type result;
    result.value((x<<3)^(y<<3)^(x*y));
    return result;
}
#endif

//This variable is used to detect the next frame.
static int prev_frame=-1;

//Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
//these as we go through all the pixels in the frame. This is much, much faster.
static int8_t *xofs, *yofs;
static int8_t *xcomp, *ycomp;

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
template<typename Destination>
void pretty_effect_calc_lines(uint16_t width,uint16_t height,Destination& dest, int line, int frame, int linect)
{
    if (frame!=prev_frame) {
        //We need to calculate a new set of offset coefficients. Take some random sines as offsets to make everything
        //look pretty and fluid-y.
        for (int x=0; x<width; x++) xofs[x]=sin(frame*0.15+x*0.06)*4;
        for (int y=0; y<height; y++) yofs[y]=sin(frame*0.1+y*0.05)*4;
        for (int x=0; x<width; x++) xcomp[x]=sin(frame*0.11+x*0.12)*4;
        for (int y=0; y<height; y++) ycomp[y]=sin(frame*0.07+y*0.15)*4;
        prev_frame=frame;
    }
    for (int y=line; y<line+linect; y++) {
        for (int x=0; x<width; x++) {
            gfx::draw::point(dest,gfx::spoint16(x,y-line),get_bgnd_pixel(x+yofs[y]+xcomp[x], y+xofs[x]+ycomp[y]));
            //dest.point(gfx::point16(x,y-line),get_bgnd_pixel(x+yofs[y]+xcomp[x], y+xofs[x]+ycomp[y]));
        }
    }
}

/**
 * @brief Initialize the effect
 *
 * @return ESP_OK on success, an error from the jpeg decoder otherwise.
 */
gfx::gfx_result pretty_effect_init(const char* image,uint16_t image_width,uint16_t image_height,uint16_t width,uint16_t height)
{
    xofs = (int8_t*)malloc(width);
    assert(xofs!=nullptr);
    yofs = (int8_t*)malloc(height);
    assert(yofs!=nullptr);
    xcomp = (int8_t*)malloc(width);
    assert(xcomp!=nullptr);
    ycomp = (int8_t*)malloc(height);
    assert(ycomp!=nullptr);

#ifdef CONFIG_IDF_TARGET_ESP32
    return decode_image(image,image_width,image_height, &pixels/*,&pixels_palette*/);
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
    //esp32s2/c3 doesn't have enough memory to hold the decoded image, calculate instead
    return gfx::gfx_result::success;
#endif
}