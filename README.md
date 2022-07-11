# C-SPI-Interface-to-DAC1220-DACs-on-Raspberry-Pi-Computers
A C++ class to communicate with the DAC1220 digital-to-analog (D/A) converters over the SPI protocol on Raspberry Pi (Zero, 4, 400) computers.

## Synopsis
[DAC1220](https://www.ti.com/product/DAC1220) is a 20-bit, low-power digital-to-analog converter that can be operated using the SPI communication protocol available on Raspberry Pi (Zero, 4, 400) computers.

## Requirements
To be able to compile this code, you need to install the latest version of the [C library for Broadcom BCM 2835 as used in Raspberry Pi](http://www.airspayce.com/mikem/bcm2835/) by Mike McCauley.

## Compilation
Use the Makefile to compile the test example or modify it to suit your needs:
- To compile the test example, run `make all`
- To generate documentation, run `make doc`. You need to install [Doxygen](https://www.doxygen.nl/manual/index.html) to generate documentation.

## Test Code
The file `test_dac1220.cpp` constains an example. After building the code, run it as `sudo ./build/test_dac1220` or `make test`.
