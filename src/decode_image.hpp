/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "stream.hpp"
#include "gfx_drawing.hpp"
#include "gfx_image.hpp"
//Define the height and width of the jpeg file. Make sure this matches the actual jpeg
//dimensions.
#define IMAGE_W 336
#define IMAGE_H 256

const char *TAG = "ImageDec";

//Data that is passed from the decoder function to the infunc/outfunc functions.
typedef struct {
    FILE* fd;
    const unsigned char *inData; //Pointer to jpeg data
    uint16_t inPos;              //Current position in jpeg data
    uint16_t **outData;          //Array of IMAGE_H pointers to arrays of IMAGE_W 16-bit pixel values
    int outW;                    //Width of the resulting file
    int outH;                    //Height of the resulting file
} JpegDev;

//Input function for jpeg decoder. Just returns bytes from the inData field of the JpegDev structure.
/*
//Output function. Re-encodes the RGB888 data from the decoder as big-endian RGB565 and
//stores it in the outData array of the JpegDev structure.
static int outfunc(JDEC *decoder, void *bitmap, JRECT *rect)
{
    JpegDev *jd = (JpegDev *)decoder->device;
    uint8_t *in = (uint8_t *)bitmap;
    for (int y = rect->top; y <= rect->bottom; y++) {
        for (int x = rect->left; x <= rect->right; x++) {
            //We need to convert the 3 bytes in `in` to a rgb565 value.
            uint16_t v = 0;
            v |= ((in[0] >> 3) << 11);
            v |= ((in[1] >> 2) << 5);
            v |= ((in[2] >> 3) << 0);
            //The LCD wants the 16-bit value in big-endian, so swap bytes
            v = (v >> 8) | (v << 8);
            jd->outData[y][x] = v;
            in += 3;
        }
    }
    return 1;
}
*/
using pixels_type = gfx::large_bitmap<gfx::rgb_pixel<16>>;
//Decode the embedded image into pixel lines that can be used with the rest of the logic.
/**
 * @brief Decode the jpeg ``image.jpg`` embedded into the program file into pixel data.
 *
 * @param pixels A pointer to a pointer for an array of rows, which themselves are an array of pixels.
 *        Effectively, you can get the pixel data by doing ``decode_image(&myPixels); pixelval=myPixels[ypos][xpos];``
 * @return - ESP_ERR_NOT_SUPPORTED if image is malformed or a progressive jpeg file
 *         - ESP_ERR_NO_MEM if out of memory
 *         - ESP_OK on succesful decode
 */
gfx::gfx_result decode_image(const char* image, uint16_t image_width, uint16_t image_height,pixels_type *pixels)
{
    *pixels = pixels_type(gfx::size16(image_width,image_height),1);
    if(!pixels->initialized()) {
        ESP_LOGE(TAG, "Error allocating memory for lines");
        return gfx::gfx_result::out_of_memory;
    }
    gfx::gfx_result ret = gfx::gfx_result::success;
    io::file_stream fs(image);
    if(!fs.caps().read)
    {
        return gfx::gfx_result::io_error;
    }
    ret = gfx::jpeg_image::load(&fs,[](const typename gfx::jpeg_image::region_type& region,gfx::point16 location,void* state) {
        pixels_type* out = (pixels_type*)state;
        gfx::rect16 r = region.bounds().offset(location.x,location.y);
        gfx::draw::bitmap(*out,(gfx::srect16)r,region,region.bounds());
        return gfx::gfx_result::success;
    },pixels);
    if(gfx::gfx_result::success!=ret) {
        printf("Bad result %d\r\n",(int)ret);
        vTaskDelay(portMAX_DELAY);
    }
    return ret;

}