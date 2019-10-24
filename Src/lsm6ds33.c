/*
 * lsm6ds33.c
 *
 *  Created on: Jul 20, 2019
 *      Author: Kyle
 */

#include "lsm6ds33.h"
#include <cstring>
#include <cstdio>

static GPIO_TypeDef* gpio_bank;
static uint16_t gpio_pin;
static SPI_HandleTypeDef* hspi;
//int32_t accel_x_cal;
//int32_t accel_y_cal;
//int32_t accel_z_cal;

extern uint8_t buffer[];
extern UART_HandleTypeDef huart2;

static int _read_reg_u8(uint8_t address, uint8_t* value)
{
  uint8_t txData[2] = {0x80, 0x00};
  uint8_t rxData[2];

  txData[0] |= address;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 100);

  while( hspi->State == HAL_SPI_STATE_BUSY );  // wait xmission complete
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  *value = rxData[1];

  return 0;
}

static int _write_reg(uint8_t address, uint8_t value)
{
  uint8_t txData[2] = {0x00, 0x00};

  txData[0] |= address;
  txData[1] |= value;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, (uint8_t*)&txData, sizeof(txData), 100);

  while( hspi->State == HAL_SPI_STATE_BUSY );  // wait xmission complete
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

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

int lsm6ds33_read_gyro_x(int16_t* x)
{
  read_reg_16(0x22, x);
  return 0;
}

int lsm6ds33_read_gyro_y(int16_t* y)
{
  read_reg_16(0x24, y);
  return 0;
}

int lsm6ds33_read_gyro_z(int16_t* z)
{
  read_reg_16(0x26, z);
  return 0;
}

int lsm6ds33_read_accel_x(int16_t* x)
{
  read_reg_16(0x28, x);
  //*x -= accel_x_cal;
  return 0;
}

int lsm6ds33_read_accel_y(int16_t* y)
{
  read_reg_16(0x2A, y);
  //*y -= accel_y_cal;
  return 0;
}

int lsm6ds33_read_accel_z(int16_t* z)
{
  read_reg_16(0x2C, z);
  //*z -= accel_z_cal;
  return 0;
}

int lsm6ds33_initialize(SPI_HandleTypeDef* spi_port, GPIO_TypeDef* cs_gpio_bank, uint16_t cs_gpio_pin)
{
  uint8_t who_am_i = 0;

  hspi = spi_port;
  gpio_bank = cs_gpio_bank;
  gpio_pin = cs_gpio_pin;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  _read_reg_u8(0x0F, &who_am_i);

  if (who_am_i != 0x69) {
    sprintf((char*)buffer, "LIS3MDL whoami read back: 0x%x (expected 0x69)\r\n", who_am_i);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  }
  //  return -1;

  /* turn on accel */
  _write_reg(0x10, 0x20);
  /* turn on gyro */
  _write_reg(0x11, 0x20);

  _read_reg_u8(0x0F, &who_am_i);

  sprintf((char*)buffer, "LIS3MDL whoami after powerup read back: 0x%x (expected 0x69)\r\n", who_am_i);
  HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);

//  for (int i = 0; i < 20; i++) {
//    int16_t temp_x = 0;
//    int16_t temp_y = 0;
//    int16_t temp_z = 0;
//
//    lsm6ds33_read_accel_x(&temp_x);
//    accel_x_cal += temp_x;
//    printf("x cal point %d: %d\r\n", i, temp_x);
//    lsm6ds33_read_accel_y(&temp_y);
//    accel_y_cal += temp_y;
//    lsm6ds33_read_accel_z(&temp_z);
//    accel_z_cal += temp_z;
//
//    HAL_Delay(100);
//  }
//
//  accel_x_cal /= 20;
//  accel_y_cal /= 20;
//  accel_z_cal /= 20;
//
//  printf("cal: x: %lu  y: %lu  z: %lu\r\n", accel_x_cal, accel_y_cal, accel_z_cal);

  return 0;
}
