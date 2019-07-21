/*
 * lis3mdl.c
 *
 *  Created on: Jul 20, 2019
 *      Author: Kyle
 */


#include "lis3mdl.h"

static GPIO_TypeDef* gpio_bank;
static uint16_t gpio_pin;
static SPI_HandleTypeDef* hspi;

static int _read_reg_u8(uint8_t address, uint8_t* value)
{
  uint8_t txData[2] = {0x80, 0x00};
  uint8_t rxData[2];

  txData[0] |= address;

  HAL_GPIO_WritePin(gpio_bank, gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 100);

  while( hspi->State == HAL_SPI_STATE_BUSY );  // wait xmission complete
  HAL_GPIO_WritePin(gpio_bank, gpio_pin, GPIO_PIN_SET);

  *value = rxData[1];

  return 0;
}

static int _write_reg(uint8_t address, uint8_t value)
{
  uint8_t txData[2] = {0x00, 0x00};

  txData[0] |= address;
  txData[1] |= value;

  HAL_GPIO_WritePin(gpio_bank, gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t*)&txData, sizeof(txData), 100);

  while( hspi->State == HAL_SPI_STATE_BUSY );  // wait xmission complete
  HAL_GPIO_WritePin(gpio_bank, gpio_pin, GPIO_PIN_SET);

  return 0;
}

static int read_reg_u16(uint8_t address, uint16_t* val)
{
  uint8_t msb = 0;
  uint8_t lsb = 0;

  _read_reg_u8(address, &lsb);
  _read_reg_u8(address + 1, &msb);

  *val = ( (msb << 8) | lsb);

  return 0;
}

static int read_reg_16(uint8_t address, int16_t* val)
{
  read_reg_u16(address, (uint16_t*)val);
  return 0;
}

int lis3mdl_read_mag_x(int16_t* x)
{
  read_reg_16(0x28, x);
  return 0;
}

int lis3mdl_read_mag_y(int16_t* y)
{
  read_reg_16(0x2A, y);
  return 0;
}

int lis3mdl_read_mag_z(int16_t* z)
{
  read_reg_16(0x2C, z);
  return 0;
}

int lis3mdl_initialize(SPI_HandleTypeDef* spi_port, GPIO_TypeDef* cs_gpio_bank, uint16_t cs_gpio_pin)
{
  uint8_t who_am_i = 0;

  hspi = spi_port;
  gpio_bank = cs_gpio_bank;
  gpio_pin = cs_gpio_pin;

  _read_reg_u8(0x0F, &who_am_i);

  if (who_am_i != 0x3D)
    return -1;

  /* turn on mag */
  _write_reg(0x20, 0x70);
  _write_reg(0x21, 0x00);
  _write_reg(0x22, 0x00);
  _write_reg(0x23, 0x0C);

  return 0;
}
