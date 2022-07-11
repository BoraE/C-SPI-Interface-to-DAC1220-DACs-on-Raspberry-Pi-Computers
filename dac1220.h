/**
 * Class and variable definitions for the DAC 1220 digital-to-analog converters.
 *
 * Copyright (C) 2022 Bora Eryilmaz
 */

#ifndef DAC1220_H
#define DAC1220_H

#include <bcm2835.h>
#include <stdint.h>

namespace dac1220 {
  // Command Byte
  constexpr uint8_t CB_RW = 7;
  constexpr uint8_t CB_MB = 5;
  constexpr uint8_t CB_ADR = 0;

  constexpr uint8_t CB_RW_R = 1;
  constexpr uint8_t CB_RW_W = 0;

  constexpr uint8_t CB_MB_1 = 0b00;
  constexpr uint8_t CB_MB_2 = 0b01;
  constexpr uint8_t CB_MB_3 = 0b10;

  // Registers addresses
  constexpr uint8_t DIR_ADR = 0;
  constexpr uint8_t CMR_ADR = 4;
  constexpr uint8_t OCR_ADR = 8;
  constexpr uint8_t FCR_ADR = 12;

  // Command Register
  constexpr uint32_t CMR_ADPT = 15;
  constexpr uint32_t CMR_CALPIN = 14;
  constexpr uint32_t CMR_CRST = 9;
  constexpr uint32_t CMR_RES = 7;
  constexpr uint32_t CMR_CLR = 6;
  constexpr uint32_t CMR_DF = 5;
  constexpr uint32_t CMR_DISF = 4;
  constexpr uint32_t CMR_BD = 3;
  constexpr uint32_t CMR_MSB = 2;
  constexpr uint32_t CMR_MD = 0;

  constexpr uint32_t CMR_MD_NORMAL = 0b00;
  constexpr uint32_t CMR_MD_CAL = 0b01;
  constexpr uint32_t CMR_MD_SLEEP = 0b10;

  // Clock Divider
  //constexpr uint16_t CLK_DIVIDER = 8000; // 50 kHz for 400 MHz core clock of RPi Zero.
  //constexpr uint16_t CLK_DIVIDER = 11000; // 50 kHz for ??? MHz core clock of RPi 400.
  constexpr uint16_t CLK_DIVIDER = 10000; // 50 kHz for ??? MHz core clock of RPi 4.

  // GPIO pins
  constexpr uint8_t MOSI_PIN = RPI_V2_GPIO_P1_19; ///< GPIO 10 (MOSI)
  constexpr uint8_t MISO_PIN = RPI_V2_GPIO_P1_21; ///< GPIO 09 (MISO)
  constexpr uint8_t CLK_PIN  = RPI_V2_GPIO_P1_23; ///< GPIO 11 (CLK)
  constexpr uint8_t CE0_PIN  = RPI_V2_GPIO_P1_24; ///< GPIO 08 (CE0)

  // Timing
  constexpr uint32_t tXIN = 400; // ns for 2.5 MHz clock
  constexpr uint32_t t10 =   20*tXIN; //  minimum 11*tXIN
  constexpr uint32_t t14 =   50*tXIN; //  minimum 41*tXIN
  constexpr uint32_t t15 =   30*tXIN; //  minimum 22*tXIN
  constexpr uint32_t t16 =  600*tXIN; //   [512-800]*tXIN
  constexpr uint32_t t17 =   20*tXIN; //  minimum 10*tXIN
  constexpr uint32_t t18 = 1200*tXIN; // [1024-1800]*tXIN
  constexpr uint32_t t19 = 2200*tXIN; // [2048-2400]*tXIN

  // Voltage reference
  constexpr double Vref = 2.500; // V

  // Modes
  enum class Mode {Sleep, Normal};
}

class DAC1220 {
public:
  DAC1220();
  ~DAC1220();

  static void startup();
  static void reset();
  static void calibrate(bool output_on = 0);
  static void set_mode(dac1220::Mode mode);

  static void set_value(uint32_t value);
  static void set_value(double value);

  static void set_command_register(uint32_t cmr);
  static void set_data_input_register(uint32_t dir = 0);
  static void set_offset_calibration_register(uint32_t ocr = 0);
  static void set_full_scale_calibration_register(uint32_t fcr = 0x800000);

  static uint32_t read_command_register();
  static uint32_t read_data_input_register();
  static uint32_t read_offset_calibration_register();
  static uint32_t read_full_scale_calibration_register();

private:
  static void spi_begin();
  static void spi_end();

  static void write_register(uint8_t cmd, uint32_t reg);
  static uint32_t read_register(uint8_t cmd);
};

#endif
