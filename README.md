# X-NUCLEO-GNSS1A1
Arduino library to support the X-NUCLEO-GNSS1A1 expansion board using the TESEO-LIV3F module

## API

This device uses alternatively I2C or UART to communicate. An I2C or UART instance is required to access the device.
  
## Examples

There are 2 examples with the  X-NUCLEO-GNSS1A1 library.

* X_NUCLEO_GNSS1A1_HelloWorld_I2C: This example code provides a simple command line interface
  to communicate with the sensor via I2C protocol

* X_NUCLEO_GNSS1A1_HelloWorld_UART: This example code provides a simple command line interface
  to communicate with the sensor via UART protocol

## Note

The device works only in an outdoor enviroment with a clear view of the sky.
In order to prevent data loss the update function should be called at least 20 times per second.
Both the examples need at least 16KB of RAM in order to work properly.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-GNSS1A1

The TESEO-LIV3F datasheet is available at  
https://www.st.com/content/st_com/en/products/positioning/gnss-modules/teseo-liv3f.html
