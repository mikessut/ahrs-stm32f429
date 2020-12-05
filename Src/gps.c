
#include "gps.h"
#include <stdio.h>
#include <cstring>

extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;

void read_gps() {
  uint8_t txData[100];
  uint8_t rxData[100];
  uint8_t buffer[100];

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 100);

  while( hspi4.State == HAL_SPI_STATE_BUSY );  // wait xmission complete

  for (int i=0; i < 100; i++) {
    sprintf((char*)buffer, "%c", rxData[i]);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
  }

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
