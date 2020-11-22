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
#include "utils.h"

#define ACCEL_SF 0.061/1000.0*9.81   // convert to m/s2
#define GYRO_SF 4.375/1000.0*3.141592653589793/180.0   // convert to rad/sec
#define MAG_SF  1.0/6842*100               // convert to uT

#define NUM_INIT 1000  // Number of points to average to determine gyro bias and inital mag vector

#define PRINT_GYRO
#define PRINT_ACCEL
#define PRINT_MAG
#define PRINT_PRESSURE
#define PRINT_KF_STATE
#define PRINT_CAN_RX_DEBUG
#define PRINT_CAN_RX
#define SEND_CAN_MSGS
//#define RX_CAN_MSGS

#define CANID_BARO 0x190
#define CANID_OAT  0x406

#define PSI2PASCAL 6894.76

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
  float wx, wy, wz, ax, ay, az, mx, my, mz;
  int init_ctr = 0;
  int32_t wx_offset = 0;
  int32_t wy_offset = 0;
  int32_t wz_offset = 0;

  int32_t ax_offset = 0;
  int32_t ay_offset = 0;
  int32_t az_offset = 0;

  int32_t mag_init_x = 0;
  int32_t mag_init_y = 0;
  int32_t mag_init_z = 0;

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
    //printf("LSM6DS33 Initilization Error\r\n");
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
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    // SPI1 = LSM6DS33
    // SPI2 = LIS3MDL

    lsm6ds33_read_gyro_x(&gyro_x);
    lsm6ds33_read_gyro_y(&gyro_y);
    lsm6ds33_read_gyro_z(&gyro_z);

    lsm6ds33_read_accel_x(&accel_x);
    lsm6ds33_read_accel_y(&accel_y);
    lsm6ds33_read_accel_z(&accel_z);

    lis3mdl_read_mag_x(&mag_x);
    lis3mdl_read_mag_y(&mag_y);
    lis3mdl_read_mag_z(&mag_z);

    if (init_ctr < NUM_INIT) {
      wx_offset += gyro_x;
      wy_offset += gyro_y;
      wz_offset += gyro_z;

      ax_offset += accel_x;
      ay_offset += accel_y;
      az_offset += accel_z;

      // Mag
      mag_init_x += mag_x;
      mag_init_y += mag_y;
      mag_init_z += mag_z;

      init_ctr++;
      continue;
    } else if (init_ctr == NUM_INIT) {
      //k.init_mag(Matrix<float, 3, 1>(mag_init_x/NUM_INIT, mag_init_y/NUM_INIT, mag_init_z/NUM_INIT));
      //k.normalize_yaw();

      wx_offset /= NUM_INIT;
      wy_offset /= NUM_INIT;
      wz_offset /= NUM_INIT;

      ax_offset /= NUM_INIT;
      ay_offset /= NUM_INIT;
      az_offset /= NUM_INIT;

      init_ctr++;
    }

    
    ax = (float) (accel_x - ax_offset)*ACCEL_SF;
    ay = (float)-(accel_y - ay_offset)*ACCEL_SF;
    az = (float)-(accel_z)*ACCEL_SF;

    wx = (float)(gyro_x -  wx_offset)*GYRO_SF;
    wy = (float)-(gyro_y - wy_offset)*GYRO_SF;
    wz = (float)-(gyro_z - wz_offset)*GYRO_SF;

    mx = (float)mag_x*MAG_SF;
    my = (float)-mag_y*MAG_SF;
    mz = (float)-mag_z*MAG_SF;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    k.predict(.042, tas*K2ms);
    k.update_accel(Vector3f(ax, ay, az));
    k.update_gyro(Vector3f(wx, wy, wz));
    //k.update_mag(Vector3f(mag_x, mag_y, mag_z));
    k.update_mag(Vector3f(20, 0, 50));
    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

    /* Static Pressure Port Sensor */
    uint8_t abs_press_data[4] = {0, 0, 0, 0};
    uint8_t error = HAL_OK;
    error =  HAL_I2C_Master_Receive(&hi2c1, 0x71, abs_press_data, 4, 0xFFFF);
    if (HAL_OK != error) {
      sprintf((char*)buffer, "Pressure read error: %d\r\n", error);
      HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
	    printf("error: %u\n\r", error);
    }

    uint8_t status = (abs_press_data[0] & 0xC0) >> 6;
    uint16_t bridge_data = ((abs_press_data[0] & 0x3F) << 8) + abs_press_data[1];

    abs_press_temp =  (float)((abs_press_data[2] << 3) + ((abs_press_data[3] & 0xE0) >> 5))/2047.0*200.0 - 50.0;
    //uint16_t temperature_data = (abs_press_data[2] << 3) + ((abs_press_data[3] & 0xE0) >> 5);
    //uint16_t temperature = (temperature_data * 200)/2047 - 50;
    //uint16_t pressure = ((bridge_data - 1638) * 15 * 1000) / (14745-1638);
    abs_press = ((float)(bridge_data - 1638)*15.0) / ((float)(14745-1638)) * PSI2PASCAL;

    uint8_t diff_press_data[4] = {0, 0, 0, 0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi3, diff_press_data, sizeof(diff_press_data), 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    //temperature_data = (diff_press_data[2] << 3) + ((diff_press_data[3] & 0xE0) >> 5);
    //temperature = (temperature_data * 200)/2047 - 50;
    // +/- 100mbar
    //
    bridge_data = ((diff_press_data[0] & 0x3F) << 8) + diff_press_data[1];
    diff_press = ((float)(bridge_data - 1638)*200.0) / ((float)(14745-1638)) - 100.0;
    diff_press_temp = (float)((diff_press_data[2] << 3) + ((diff_press_data[3] & 0xE0) >> 5))/2047.0*200.0 - 50.0;
    #ifdef PRINT_PRESSURE
    sprintf((char*)buffer, "PA: %f, %f\r\n", abs_press, abs_press_temp);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);

    sprintf((char*)buffer, "PD: %f, %f\r\n", diff_press, diff_press_temp);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    #endif

    #ifdef PRINT_KF_STATE
    sprintf((char*)buffer, "P, R, Y: %f, %f, %f\r\n", k.pitch()*180.0/M_PI,
                                                   k.roll()*180.0/M_PI,
                                                   positive_heading(k.heading())*180.0/M_PI);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    sprintf((char*)buffer, "KFA: %f, %f, %f\r\n", k.x(I_AX,0), k.x(I_AY,0), k.x(I_AZ,0));
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    sprintf((char*)buffer, "KFG: %f, %f, %f\r\n", k.x(I_WX,0)*180.0/M_PI, k.x(I_WY,0)*180.0/M_PI, k.x(I_WZ,0)*180.0/M_PI);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    //sprintf((char*)buffer, "KFGB: %f, %f, %f\r\n", k.x(I_WBX,0)*180.0/M_PI, k.x(I_WBY,0)*180.0/M_PI, k.x(I_WBZ,0)*180.0/M_PI);
    //HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    #endif

    #ifdef PRINT_ACCEL
    sprintf((char*)buffer, "A: %f, %f, %f\r\n", ax,ay,az);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    #endif

    #ifdef PRINT_GYRO
    sprintf((char*)buffer, "G: %f, %f, %f\r\n", wx*180.0/M_PI,wy*180.0/M_PI,wz*180.0/M_PI);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    #endif

    #ifdef PRINT_MAG
    sprintf((char*)buffer, "M: %f, %f, %f\r\n", mx, my, mz);
    //sprintf((char*)buffer, "M: %d, %d, %d\r\n", mag_x, mag_y, mag_z);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    #endif

    #ifdef RX_CAN_MSGS
    uint16_t can_rx_code;
    uint32_t can_rx_data;
    if (CAN_rx(&can_rx_code, &can_rx_data)) {
      #ifdef PRINT_CAN_RX
      sprintf((char*)buffer, "CAN MSG Received: 0x%02x with value %d\r\n", can_rx_code, can_rx_data);
      HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
      #endif

      if (can_rx_code == CANID_OAT) {
        temperature = ((float)can_rx_data) / 100.0;
        #ifdef PRINT_CAN_RX
        sprintf((char*)buffer, "OAT Set to: %f\r\n", temperature);
        HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
        #endif
      } else if (can_rx_code == CANID_BARO) {
        baro = ((float)can_rx_data) / 1000.0;
        #ifdef PRINT_CAN_RX
        sprintf((char*)buffer, "BARO Set to: %f\r\n", baro);
        HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
        #endif
      }
    }
    #endif

    // air data computations
    airspeed_altitude(abs_press, diff_press, baro, temperature,
                      &altitude, &ias, &tas);

    #ifdef SEND_CAN_MSGS
  /* START CAN BUS REPORTS */
    normal_data payload;
    payload.node = 0x12;
    payload.index = 0;
    payload.status_code = 0;

    //payload.data = (int32_t)(k.roll()*180.0/M_PI * 100.0);
    //send_can_fix_msg(0x181, &payload, 2);
    send_can_fix_msg(CANFIX_ROLL, (uint16_t)(k.roll()*180.0/M_PI * 100.0));

    payload.data = (int32_t)(k.pitch()*180.0/M_PI * 100.0);
    send_can_fix_msg(0x180, &payload, 2);
// 
    // // 0x183 is IAS
    // // 0x184 is indicated altitude
    // // 0x185 is heading
    // // 0x186 is VS
    // // 0x18d is TAS
// 
    payload.data = (int32_t)(ias * 10.0);
    send_can_fix_msg(0x183, &payload, 2);
// 
    payload.data = (int32_t)altitude;
    send_can_fix_msg(0x184, &payload, 4);
// 
    payload.data = (int32_t)(k.heading()*180.0/M_PI * 10.0);
    send_can_fix_msg(0x185, &payload, 2);

    float foo = 123.456;
    //*((float*)&payload.data) = foo;
    //send_can_fix_msg(0x600, &payload, 4);
    send_can_msg(CAN_KF_WX, &foo);
// 
    // payload.data = (uint32_t)(tas * 10.0);
    // send_can_fix_msg(0x18d, &payload);

    #endif

    //HAL_Delay(150);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
