/*
 * lsm6ds33.h
 *
 *  Created on: Jul 20, 2019
 *      Author: Kyle
 */

#ifndef LSM6DS33_H_
#define LSM6DS33_H_

#include "stm32f4xx_hal.h"

int lsm6ds33_read(int16_t* x);
int lsm6ds33_read_gyro_x(int16_t* x);
int lsm6ds33_read_gyro_y(int16_t* y);
int lsm6ds33_read_gyro_z(int16_t* z);

int lsm6ds33_read_accel_x(int16_t* x);
int lsm6ds33_read_accel_y(int16_t* y);
int lsm6ds33_read_accel_z(int16_t* z);

int lsm6ds33_initialize(SPI_HandleTypeDef* spi_port, GPIO_TypeDef* cs_gpio_bank, uint16_t cs_gpio_pin);

#endif /* LSM6DS33_H_ */
