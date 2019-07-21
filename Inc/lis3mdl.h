/*
 * lis3mdl.h
 *
 *  Created on: Jul 20, 2019
 *      Author: Kyle
 */

#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "stm32f4xx_hal.h"

int lis3mdl_read_mag_x(int16_t* x);
int lis3mdl_read_mag_y(int16_t* y);
int lis3mdl_read_mag_z(int16_t* z);

int lis3mdl_initialize(SPI_HandleTypeDef* spi_port, GPIO_TypeDef* cs_gpio_bank, uint16_t cs_gpio_pin);

#endif /* LIS3MDL_H_ */
