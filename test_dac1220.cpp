/**
 * Tests for the DAC 1220 digital-to-analog converter chip.
 *
 * Copyright (C) 2022 Bora Eryilmaz
 */

#include <dac1220.h>
#include <stdio.h>
#include <assert.h>

void test() {
  using namespace dac1220;

  /**
   * After reset, the DAC goes to Normal mode and the OCR, FSC, and DIR registers
   * are set to their default values.
   */
  printf("Testing reset state...\n");
  uint32_t dir = 0x0ABFFC; // random value
  uint32_t ocr = 0x01FAAA; // random value
  uint32_t fcr = 0x805280; // random value
  DAC1220 dac0;
  dac0.set_data_input_register(dir);
  dac0.set_offset_calibration_register(ocr);
  dac0.set_full_scale_calibration_register(fcr);
  assert(dac0.read_data_input_register() == dir);
  assert(dac0.read_offset_calibration_register() == ocr);
  assert(dac0.read_full_scale_calibration_register() == fcr);

  dac0.reset();
  // Reset will set the register values to their defaults.
  dir = 0x000000;
  ocr = 0x000000;
  fcr = 0x800000;
  assert(dac0.read_data_input_register() == dir);
  assert(dac0.read_offset_calibration_register() == ocr);
  assert(dac0.read_full_scale_calibration_register() == fcr);

  auto mode = dac0.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_NORMAL << CMR_MD));
  printf("Done.\n\n");


  /**
   * After calibration, the DAC goes to Normal mode and the OCR and FSC registers
   * hold new calibration values.
   */
  printf("Testing calibration procedure...\n");
  ocr = 0x02EB82; // random value
  fcr = 0x4A7180; // random value
  DAC1220 dac1;
  dac1.set_offset_calibration_register(ocr);
  dac1.set_full_scale_calibration_register(fcr);
  assert(dac1.read_offset_calibration_register() == ocr);
  assert(dac1.read_full_scale_calibration_register() == fcr);

  dac1.calibrate(true);
  // Calibration will almost certainly change the random calibration values.
  assert(dac1.read_offset_calibration_register() != ocr);
  assert(dac1.read_full_scale_calibration_register() != fcr);

  mode = dac1.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_NORMAL << CMR_MD));
  printf("Done.\n\n");


  /**
   * After startup, the DAC is configured as a 20-bit, straight-binary converter,
   * and is in Normal mode.
   */
  printf("Testing startup configuration...\n");
  DAC1220 dac2;
  dac2.startup();

  // 20-bit, straight binary mode.
  auto config = dac2.read_command_register() & ((1 << CMR_RES) | (1 << CMR_DF));
  assert(config == ((1 << CMR_RES) | (1 << CMR_DF)));

  mode = dac2.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_NORMAL << CMR_MD));
  printf("Done.\n\n");


  /**
   * In Normal mode, writing to CRST and CLR registers reset the OCR & FCR,
   * and DIR registers, respectively.
   */
  printf("Testing Normal mode...\n");
  dir = 0xFFFAB0; // random value
  ocr = 0x012B80; // random value
  fcr = 0x8A7180; // random value
  DAC1220 dac3;
  dac3.set_data_input_register(dir);
  dac3.set_offset_calibration_register(ocr);
  dac3.set_full_scale_calibration_register(fcr);
  assert(dac3.read_data_input_register() == dir);
  assert(dac3.read_offset_calibration_register() == ocr);
  assert(dac3.read_full_scale_calibration_register() == fcr);

  dac3.set_mode(Mode::Normal);
  mode = dac3.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_NORMAL << CMR_MD));

  auto cmr = dac3.read_command_register();
  cmr |= (1 << CMR_CRST) | (1 << CMR_CLR); // Set reset bits.
  dac3.set_command_register(cmr);
  dir = 0x000000;
  ocr = 0x000000;
  fcr = 0x800000;
  assert(dac3.read_data_input_register() == dir);
  assert(dac3.read_offset_calibration_register() == ocr);
  assert(dac3.read_full_scale_calibration_register() == fcr);
  printf("Done.\n\n");


  /**
   * In Sleep mode, writing to CRST and CLR registers have no effect on
   * the OCR & FCR, and DIR registers.
   */
  printf("Testing Sleep mode...\n");
  dir = 0x800000; // random value
  ocr = 0x01FB80; // random value
  fcr = 0x807180; // random value
  DAC1220 dac4;
  dac4.set_data_input_register(dir);
  dac4.set_offset_calibration_register(ocr);
  dac4.set_full_scale_calibration_register(fcr);
  assert(dac4.read_data_input_register() == dir);
  assert(dac4.read_offset_calibration_register() == ocr);
  assert(dac4.read_full_scale_calibration_register() == fcr);

  dac4.set_mode(Mode::Sleep);
  mode = dac4.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_SLEEP << CMR_MD));

  cmr = dac4.read_command_register();
  cmr |= (1 << CMR_CRST) | (1 << CMR_CLR); // Attempt to reset registers.
  dac4.set_command_register(cmr);
  assert(dac4.read_data_input_register() == dir);
  assert(dac4.read_offset_calibration_register() == ocr);
  assert(dac4.read_full_scale_calibration_register() == fcr);

  // Back to Normal mode.
  dac4.set_mode(Mode::Normal);
  mode = dac4.read_command_register() & (0b11 << CMR_MD);
  assert(mode == (CMR_MD_NORMAL << CMR_MD));
  // Going to Normal mode with reset bits already set will reset the registers.
  dir = 0x000000;
  ocr = 0x000000;
  fcr = 0x800000;
  assert(dac4.read_data_input_register() == dir);
  assert(dac4.read_offset_calibration_register() == ocr);
  assert(dac4.read_full_scale_calibration_register() == fcr);
  printf("Done.\n\n");


  /**
   * In 20-bit, straight-binary mode, the DAC values can be specified as
   * double or integer values
   */
  printf("Testing analog voltage generation...\n");
  DAC1220 dac5;

  dir = 0xFFFFF0; // 5 V as 20-bit integer.
  dac5.set_value(dir);
  assert(dac5.read_data_input_register() == dir);
  bcm2835_delay(200);

  dir = 0x800000; // 2.5 V as 20-bit integer.
  dac5.set_value(2.5); // Set as a floating-point value.
  assert(dac5.read_data_input_register() == dir);
  bcm2835_delay(200);

  dir = 0x000000; // 0 V
  dac5.set_value(dir);
  assert(dac5.read_data_input_register() == dir);
  bcm2835_delay(200);

  dir = 0xCCCCC0; // 4.0 V as 20-bit integer.
  dac5.set_value(4.0); // Set as a floating-point value.
  assert(dac5.read_data_input_register() == dir);
  bcm2835_delay(200);

  dir = 0x400000; // 1.25 V as 20-bit integer.
  dac5.set_value(1.25); // Set as a floating-point value.
  assert(dac5.read_data_input_register() == dir);
  bcm2835_delay(200);
  printf("Done.\n");
}


int main() {
  if (!bcm2835_init()) {
    printf("bcm2835_init failed. Are you running as root?\n");
    return 1;
  }

  // Run tests
  test();

  // Clean up
  bcm2835_close();
  return 0;
}
