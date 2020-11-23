/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "can_fix.h"
#include "lsm6ds33.h"
#include "lis3mdl.h"
#include "kalman.h"
#include "pressure.h"
#include <cstring>
#include "stm32f4xx_hal.h"
#include "utils.h"

#define ACCEL_SF 0.061/1000.0*9.81   // convert to m/s2
#define GYRO_SF 4.375/1000.0*3.141592653589793/180.0   // convert to rad/sec
#define MAG_SF  1.0/6842*100               // convert to uT

#define NUM_INIT 1000  // Number of points to average to determine gyro bias and inital mag vector

#define SEND_CANFIX_MSGS
#define RX_CAN_MSGS
#define SEND_CAN_DEBUG_MSGS
//#define SEND_UART_DEBUG_MSGS

uint8_t buffer[200];
uint8_t buffer2[11];


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  float baro = 29.92;  // inHg
  float temperature = 25; // degC
  float abs_press, diff_press; // Pascal
  float abs_press_temp, diff_press_temp;
  float tas = 0;
  float ias, altitude;
  Kalman k;
  float w[3];
  float a[3];
  float m[3];
  int init_ctr = 0;
  int32_t w_offset[3] = {0, 0, 0};
  int32_t a_offset[3] = {0, 0, 0};
  int32_t mag_init[3] = {0, 0, 0};
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  initialize_CAN();

  /* sensors fail if we try to initialize right away */
  HAL_Delay(1500);

  if (lsm6ds33_initialize(&hspi1, GPIOA, GPIO_PIN_1)) {
    sprintf((char*)buffer, "lsm6ds3 init error\r\n");
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  } else {
    sprintf((char*)buffer, "lsm6ds3 init success\r\n");
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  }

  if (lis3mdl_initialize(&hspi2, GPIOC, GPIO_PIN_0)) {
    sprintf((char*)buffer, "LIS3MDL init failure!!!\r\n");
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  } else {
    sprintf((char*)buffer, "LIS3MDL init success\r\n");
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  }

  HAL_Delay(1500);

  // Blink test
  //while(1) {
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  //  HAL_Delay(2);
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  //  HAL_Delay(2);
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  //  k.predict(.038);
  //  k.update_accel(Vector3f(.1, .2, .3));
  //  k.update_gyro(Vector3f(.4, .5, .6));
  //  k.update_mag(Vector3f(.7,.8, .9));
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  //  HAL_Delay(2);
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  //  HAL_Delay(2);
  //  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  //  HAL_Delay(50);
  //  
  //}

  while (1)
  {
    int16_t gyro[3];
    int16_t accel[3];
    int16_t mag[3];
    uint32_t tick_ctr = HAL_GetTick();
    float dt;
    
    // SPI1 = LSM6DS33
    // SPI2 = LIS3MDL
    // TODO: 
    //   1. Single SPI transaction to read all 6 axes of LSM6DS3; Single transaction to read mag
    
    lsm6ds33_read_accel_x(&accel[0]);
    lsm6ds33_read_accel_y(&accel[1]);
    lsm6ds33_read_accel_z(&accel[2]);
    lsm6ds33_read_gyro_x(&gyro[0]);
    lsm6ds33_read_gyro_y(&gyro[1]);
    lsm6ds33_read_gyro_z(&gyro[2]);
    lis3mdl_read_mag_x(&mag[0]);
    lis3mdl_read_mag_y(&mag[1]);
    lis3mdl_read_mag_z(&mag[2]);    

    if (init_ctr < NUM_INIT) {
      for (int i=0; i < 3; i ++) {
        w_offset[i] += gyro[i];
        a_offset[i] += accel[i];
        mag_init[i] += mag[i];
      }

      init_ctr++;
      continue;
    } else if (init_ctr == NUM_INIT) {
      //k.init_mag(Matrix<float, 3, 1>(mag_init_x/NUM_INIT, mag_init_y/NUM_INIT, mag_init_z/NUM_INIT));
      //k.normalize_yaw();
      for (int i=0; i < 3; i++) {
        w_offset[i] /= NUM_INIT;
        a_offset[i] /= NUM_INIT;
      }
      init_ctr++;
    }
    
    // Rotate sensors to z axis down
    a[0] = (float) (accel[0] - a_offset[0])*ACCEL_SF;
    a[1] = (float)-(accel[1] - a_offset[1])*ACCEL_SF;
    a[2] = (float)-(accel[2])*ACCEL_SF;

    w[0] = (float)(gyro[0] -  w_offset[0])*GYRO_SF;
    w[1] = (float)-(gyro[1] - w_offset[1])*GYRO_SF;
    w[2] = (float)-(gyro[2] - w_offset[2])*GYRO_SF;

    m[0] = (float)mag[0]*MAG_SF;
    m[1] = (float)-mag[1]*MAG_SF;
    m[2] = (float)-mag[2]*MAG_SF;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    //dt = (HAL_GetTick() - tick_ctr)*1e-3;
    dt = (HAL_GetTick() - tick_ctr);  // Doesn't seem to be ms as expected. Off by about a factor of 20
    k.predict(.044, tas*K2ms);
    //k.update_accel(Vector3f(ax, ay, az));
    k.update_accel(Vector3f(Map<Vector3f>(a)));
    k.update_gyro(Vector3f(Map<Vector3f>(w)));
    //k.update_mag(Vector3f(mag_x, mag_y, mag_z));
    k.update_mag(Vector3f(20, 0, 50));
    tick_ctr = HAL_GetTick();
    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

    abs_pressure(&abs_press, &abs_press_temp);
    diff_pressure(&diff_press, &diff_press_temp);

    #ifdef SEND_UART_DEBUG_MSGS
    uart_debug(&k, a, w, m, 
               &abs_press, &abs_press_temp,
               &diff_press, &diff_press_temp);
    #endif

    #ifdef SEND_CAN_DEBUG_MSGS
    can_debug(&k, a, w, m, 
              &abs_press, &abs_press_temp,
              &diff_press, &diff_press_temp, &dt, &baro);    
    #endif              

    #ifdef RX_CAN_MSGS
    rx_canfix_msgs(&baro);  //&temperature, &baro);
    #endif

    // air data computations
    airspeed_altitude(abs_press, diff_press, baro, temperature,
                      &altitude, &ias, &tas);

    #ifdef SEND_CANFIX_MSGS
    send_canfix_msgs(&k, &ias, &tas, &altitude);
    #endif

    //HAL_Delay(150);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
