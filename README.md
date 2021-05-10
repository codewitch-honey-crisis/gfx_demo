# ILI9341 and SSD1306 Display Drivers and demo for GFX

This code is a playground for driver development, but right now it supports the ILI9341 as write only, but it's very snappy in terms of performance, implementing nearly every performance capability of GFX

Also there is an SSD1306 display driver as well.

GFX meanwhile, is the drawing and graphics library that can interface with these or other drivers. They are independent of each other, although for a driver to work with GFX it must expose GFX bindings and thus have a dependency on GFX

This is set up for an ESP_WROVER_KIT by default. You'll have to modify the pin settings for your configuration. It's set up to use the HSPI SPI host as well. You might use VSPI instead in which case you'll have to change that setting as well.

Build this with platform io. if the ide pukes, try typing 'pio run' inside a Platform IO CLI terminal window