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
gfx::gfx_result decode_image(const char* image, uint16_t image_width, uint16_t image_height,uint16_t ***pixels)
{
    char *work = NULL;
    *pixels = NULL;
    gfx::gfx_result ret = gfx::gfx_result::success;
    io::file_stream fs(image);
    if(!fs.caps().read)
    {
        ret =gfx::gfx_result::io_error;
        goto err;
    }
    //Alocate pixel memory. Each line is an array of IMAGE_W 16-bit pixels; the `*pixels` array itself contains pointers to these lines.
    *pixels = (uint16_t**)calloc(image_height, sizeof(uint16_t *));
    if (*pixels == NULL) {
        ESP_LOGE(TAG, "Error allocating memory for lines");
        ret = gfx::gfx_result::out_of_memory;
        goto err;
    }
    for (int i = 0; i < image_height; i++) {
        (*pixels)[i] = (uint16_t*)malloc(image_width * sizeof(uint16_t));
        if ((*pixels)[i] == NULL) {
            ESP_LOGE(TAG, "Error allocating memory for line %d", i);
            ret = gfx::gfx_result::out_of_memory;
            goto err;
        }
    }

    
    ret = gfx::jpeg_image::load(&fs,[](const typename gfx::jpeg_image::region_type& region,gfx::point16 location,void* state) {
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
        return gfx::gfx_result::success;
    },*pixels);
    if(gfx::gfx_result::success!=ret) {
        printf("Bad result %d\r\n",(int)ret);
        vTaskDelay(portMAX_DELAY);
    }
    return ret;
err:
    //Something went wrong! Exit cleanly, de-allocating everything we allocated.
    if (*pixels != NULL) {
        for (int i = 0; i < image_height; i++) {
            free((*pixels)[i]);
        }
        free(*pixels);
    }
    free(work);
    return ret;
}