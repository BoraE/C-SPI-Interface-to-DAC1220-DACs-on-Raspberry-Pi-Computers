/**
 * Driver code for the DAC 1220 digital-to-analog converters.
 *
 * Copyright (C) 2022 Bora Eryilmaz
 */

#include <dac1220.h>
#include <stdio.h>
#include <stdexcept>

using namespace dac1220;

DAC1220::DAC1220() {
  spi_begin();
  startup();
}

DAC1220::~DAC1220() {
  spi_end();
}


/**
 * Applies the DAC startup procedure to make sure that the device is
 * in a stable state before generating analog voltages.
 */
void DAC1220::startup() {
  // Wait for the crystal oscillator to start.
  bcm2835_delay(25);

  // Wait 200 ms for I/O recovery timeout in case of a firmware problem.
  bcm2835_delay(200);

  // Apply the reset pattern, which enters the Normal mode when complete.
  reset();

  // Set configuration to 20-bit, straight binary mode.
  uint32_t cmr = read_command_register();
  cmr |= (1 << CMR_RES) | (1 << CMR_DF);
  set_command_register(cmr);

  // Calibrate with the output connected.
  calibrate(true);
}


/**
 * Applies the special reset pattern to the clock pin while the enable pin
 * is held low. The DAC switches automatically to Normal mode when done.
 */
void DAC1220::reset() {
  spi_end();

  // Temporarily switch CLK and CE0 pins to be outputs.
  bcm2835_gpio_fsel(CLK_PIN, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(CE0_PIN, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(CLK_PIN, LOW);
  bcm2835_gpio_write(CE0_PIN, LOW);
  bcm2835_delayMicroseconds(t10/1000);

  // Apply the reset pattern.
  bcm2835_gpio_write(CLK_PIN, HIGH);
  bcm2835_delayMicroseconds(t16/1000);
  bcm2835_gpio_write(CLK_PIN, LOW);
  bcm2835_delayMicroseconds(t17/1000);
  bcm2835_gpio_write(CLK_PIN, HIGH);
  bcm2835_delayMicroseconds(t18/1000);
  bcm2835_gpio_write(CLK_PIN, LOW);
  bcm2835_delayMicroseconds(t17/1000);
  bcm2835_gpio_write(CLK_PIN, HIGH);
  bcm2835_delayMicroseconds(t19/1000);
  bcm2835_gpio_write(CLK_PIN, LOW); // DAC resets here.

  // Get ready to switch back to SPI mode.
  bcm2835_delayMicroseconds(t14/1000);
  bcm2835_gpio_write(CE0_PIN, HIGH);
  bcm2835_delayMicroseconds(t15/1000);
}


/**
 * Calibrates the DAC and switches automatically to Normal mode when complete.
 */
void DAC1220::calibrate(bool output_on) {
  // Set desired output state during calibration.
  uint32_t cmr = read_command_register();
  if (output_on) {
    cmr |= (1 << CMR_CALPIN);
  } else {
    cmr &= ~(1 << CMR_CALPIN);
  }
  set_command_register(cmr);

  // Start calibration, in a separate call.
  cmr |= (CMR_MD_CAL << CMR_MD);
  set_command_register(cmr);

  // Wait 500 ms for calibration to complete.
  bcm2835_delay(500);
}


void DAC1220::set_mode(Mode mode) {
  // Clean MD bits first.
  uint32_t cmr = read_command_register() & ~(0b11 << CMR_MD);
  switch (mode) {
  case Mode::Sleep:
    cmr |= (CMR_MD_SLEEP << CMR_MD);
    break;

  case Mode::Normal:
    cmr |= (CMR_MD_NORMAL << CMR_MD);
    break;
  }
  set_command_register(cmr);
}


/**
 * Generates output voltages specified as floating-point values between
 * 0 and 2*Vref volts.
 */
void DAC1220::set_value(double value) {
  // Range check to prevent overflow.
  if (value < 0.0) {
    value = 0.0;
  } else if (value >= 2*Vref) {
    const double LSB = Vref / (double)0x80000;
    value = 2*Vref - LSB;
  }

  // Assumes 20-bit, straight-binary code.
  uint32_t code = (uint32_t) ((double)0x80000 * (value / Vref)); // Table 8 in datasheet.
  code = (code << 4); // Make it a 24-bit value.
  set_value(code);
}


/**
 * Generates output voltages specified as left-justified 20-bit numbers
 * within a right-aligned 24-bit (xxFFFFFxh) integer value.
 */
void DAC1220::set_value(uint32_t value) {
  // Assumes 20-bit, straight-binary code.
  set_data_input_register(value);
}


void DAC1220::set_command_register(uint32_t cmr) {
  // Register value set as a right-aligned 16-bit number (xxxxFFFFh).
  uint8_t cmd = (CB_RW_W << CB_RW) | (CB_MB_2 << CB_MB) | (CMR_ADR << CB_ADR);
  write_register(cmd, cmr);
}


void DAC1220::set_data_input_register(uint32_t dir) {
  // Register value set as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_W << CB_RW) | (CB_MB_3 << CB_MB) | (DIR_ADR << CB_ADR);
  write_register(cmd, dir);
}


void DAC1220::set_offset_calibration_register(uint32_t ocr) {
  // Register value set as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_W << CB_RW) | (CB_MB_3 << CB_MB) | (OCR_ADR << CB_ADR);
  write_register(cmd, ocr);
}


void DAC1220::set_full_scale_calibration_register(uint32_t fcr) {
  // Register value set as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_W << CB_RW) | (CB_MB_3 << CB_MB) | (FCR_ADR << CB_ADR);
  write_register(cmd, fcr);
}


uint32_t DAC1220::read_command_register() {
  // Value returned as a right-aligned 16-bit number (xxxxFFFFh).
  uint8_t cmd = (CB_RW_R << CB_RW) | (CB_MB_2 << CB_MB) | (CMR_ADR << CB_ADR);
  return read_register(cmd);
}


uint32_t DAC1220::read_data_input_register() {
  // Value returned as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_R << CB_RW) | (CB_MB_3 << CB_MB) | (DIR_ADR << CB_ADR);
  return read_register(cmd);
}


uint32_t DAC1220::read_offset_calibration_register() {
  // Value returned as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_R << CB_RW) | (CB_MB_3 << CB_MB) | (OCR_ADR << CB_ADR);
  return read_register(cmd);
}


uint32_t DAC1220::read_full_scale_calibration_register() {
  // Value returned as a right-aligned 24-bit number (xxFFFFFFh).
  uint8_t cmd = (CB_RW_R << CB_RW) | (CB_MB_3 << CB_MB) | (FCR_ADR << CB_ADR);
  return read_register(cmd);
}


void DAC1220::spi_begin() {
  if (!bcm2835_spi_begin()) {
    throw std::runtime_error("Could not configure SPI for communicating with DAC1220.");
  }

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1); // CPOL = 0, CPHA = 1
  bcm2835_spi_setClockDivider(CLK_DIVIDER); // 50 kHz clock.
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0); // Chip wired to CS0.
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
}


void DAC1220::spi_end() {
  bcm2835_spi_end();
}


void DAC1220::write_register(uint8_t cmd, uint32_t reg) {
  spi_begin();

  // Transmit the command byte and the correct number of register bytes.
  char buffer[4] = {cmd, 0, 0, 0};
  switch (cmd & ((uint8_t)0b11 << CB_MB)) {
  case (CB_MB_1 << CB_MB): // 1 byte
    buffer[1] = (char)(reg & 0xFF);
    bcm2835_spi_writenb(buffer, 2);
    break;
  case (CB_MB_2 << CB_MB): // 2 bytes
    buffer[1] = (char)((reg >> 8) & 0xFF);
    buffer[2] = (char)(reg & 0xFF);
    bcm2835_spi_writenb(buffer, 3);
    break;
  case (CB_MB_3 << CB_MB): // 3 bytes
    buffer[1] = (char)((reg >> 16) & 0xFF);
    buffer[2] = (char)((reg >> 8) & 0xFF);
    buffer[3] = (char)(reg & 0xFF);
    bcm2835_spi_writenb(buffer, 4);
    break;
  }
  bcm2835_delayMicroseconds(t15/1000);
}


uint32_t DAC1220::read_register(uint8_t cmd) {
  spi_begin();

  // Send read command.
  bcm2835_spi_transfer(cmd);

  // Set SPI_REN in SPI_CS before writing junk data to trigger the transfer.
  // This will turn the bus around: MOSI disconnected, MISO wired to MOSI pin.
  volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
  bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_REN, BCM2835_SPI0_CS_REN);

  uint32_t value = 0;
  char buffer[3] = {0, 0, 0};
  switch (cmd & ((uint8_t)0b11 << CB_MB)) {
  case (CB_MB_1 << CB_MB): // 1 byte
    bcm2835_spi_transfern(buffer, 1);
    value = (uint32_t)buffer[0];
    break;
  case (CB_MB_2 << CB_MB): // 2 bytes
    bcm2835_spi_transfern(buffer, 2);
    value = ((uint32_t)buffer[0] << 8) | (uint32_t)buffer[1];
    break;
  case (CB_MB_3 << CB_MB): // 3 bytes
    bcm2835_spi_transfern(buffer, 3);
    value = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
    break;
  }

  // Clear SPI_REN in SPI_CS to switch back to write mode.
  bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_REN);
  bcm2835_delayMicroseconds(t15/1000);

  return value;
}
