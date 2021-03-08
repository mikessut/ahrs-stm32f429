
#include "gps.h"
#include <stdio.h>
#include <cstring>

extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;

/**
 * read_gps
 *
 * head (like the CANfix message) is in tenths of a degree
 */
 // $GNVTG,,T,,M,0.420,N,0.778,K,A*3
void read_gps(float *lng, float *lat, uint16_t* head) {
  uint8_t txData[100];
  uint8_t rxData[100];
  uint8_t buffer[150];
  char rmc_msg[150];
  char *pch;
  char *rmc_end;

  uint8_t msg_complete = 0;

  while (!msg_complete)
  {
    // sprintf((char*)buffer, "*\n");
    // HAL_UART_Transmit(&huart2, buffer, 1, 0xFFFF);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 99);

    // Make rxData null terminated
    rxData[99] = '\0';

    while( hspi4.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    for (int i=0; i < 100; i++) {
      sprintf((char*)buffer, "%c", rxData[i]);
      HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    }

    pch = strstr((char*)rxData, "$GNVTG");
    if (pch) {
      // Found $GPRMC in rxData

      // Is end of RMC msg also in rxData?
      rmc_end = strstr(pch, "\r\n");
      if (rmc_end) {
        *(rmc_end+1) = '\0';
        strcpy(rmc_msg, pch);
        //strncpy(rmc_msg, pch, rmc_end+2-pch);
        //*(rmc_msg + (rmc_end+2-pch+1)) = '\0';
        msg_complete = 1;
      } else {
        strcpy(rmc_msg, pch);
        // Need to read again to finish msg
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 99);
        // Make rxData null terminated
        rxData[99] = '\0';
        while( hspi4.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

        rmc_end = strstr((char*)rxData, "\r\n");
        *(rmc_end+1) = '\0';
        // Handle case that it's not in there?
        strcat(rmc_msg, (char*)rxData);
        //*(rmc_msg + (rmc_end+2-pch+1)) = '\0';
        msg_complete = 1;
      }

    }
  } // end while !complete

  sprintf((char*)buffer, "parsed: %s", rmc_msg);
  HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
}
