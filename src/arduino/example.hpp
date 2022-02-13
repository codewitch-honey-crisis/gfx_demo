#include "drivers/common/tft_io.hpp"
using namespace arduino;
#if defined(PARALLEL8)
#define PIN_NUM_BCKL -1
#define PIN_NUM_CS   33  // Chip select control pin (library pulls permanently low
#define PIN_NUM_DC   22  // (RS) Data Command control pin - must use a pin in the range 0-31
#define PIN_NUM_RST  32  // Reset pin, toggles on startup
#define PIN_NUM_WR    21  // Write strobe control pin - must use a pin in the range 0-31
#define PIN_NUM_RD    15  // Read strobe control pin
#define PIN_NUM_D0   2  // Must use pins in the range 0-31 for the data bus
#define PIN_NUM_D1   13  // so a single register write sets/clears all bits.
#define PIN_NUM_D2   26  // Pins can be randomly assigned, this does not affect
#define PIN_NUM_D3   25  // TFT screen update performance.
#define PIN_NUM_D4   27
#define PIN_NUM_D5   12
#define PIN_NUM_D6   14
#define PIN_NUM_D7   4
#elif defined(I2C)
#define TFT_PORT 0
#define PIN_NUM_SDA 21
#define PIN_NUM_SCL 22
#define PIN_NUM_RST -1
#define PIN_NUM_DC -1
#define TFT_ADDR 0x3C
#define TFT_CMD 0x00
#define TFT_DATA 0x40
#define I2C_FREQ (400*1000)
#else
#define TFT_HOST VSPI
#define PIN_NUM_CS 5
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK 18
#define PIN_NUM_DC 2
#define PIN_NUM_RST 4
#define SPI_WRITE_FREQ (26*1000*1000)
#define SPI_READ_FREQ (20*1000*1000)
#endif

#define TFT_WIDTH 200
#define TFT_HEIGHT 320

#ifdef PARALLEL8
using bus_type = tft_p8<PIN_NUM_CS,
                        PIN_NUM_WR,
                        PIN_NUM_RD,
                        PIN_NUM_D0,
                        PIN_NUM_D1,
                        PIN_NUM_D2,
                        PIN_NUM_D3,
                        PIN_NUM_D4,
                        PIN_NUM_D5,
                        PIN_NUM_D6,
                        PIN_NUM_D7>;
#elif defined(I2C)
using bus_type = tft_i2c<TFT_PORT,
                        TFT_ADDR,
                        PIN_NUM_SDA,
                        PIN_NUM_SCL,
                        TFT_CMD,
                        TFT_DATA,
                        I2C_FREQ>;
#else
using bus_type = tft_spi<TFT_HOST,
                        PIN_NUM_CS,
                        PIN_NUM_MOSI,
                        PIN_NUM_MISO,
                        PIN_NUM_CLK,
                        SPI_MODE0,
                        SPI_WRITE_FREQ,
                        SPI_READ_FREQ,
                        PIN_NUM_MISO<0
#ifdef OPTIMIZE_DMA
                        ,(TFT_WIDTH*TFT_HEIGHT)/8+8
#endif
>;
#endif

