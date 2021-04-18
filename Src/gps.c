
#include "gps.h"
#include <stdio.h>
#include <cstring>
#include <stdlib.h>

extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;

#define UART_DEBUG_GPS

/**
 * read_gps
 *
 * head (like the CANfix message) is in tenths of a degree
 */
 // $GNVTG,,T,,M,0.420,N,0.778,K,A*3
bool read_gps(float* lat, float* lng, uint16_t* head, float* spd) {
  uint8_t txData[51];
  uint8_t rxData[51];
  uint8_t buffer[150];
  char rmc_msg[150];
  char *pch;
  char *rmc_end;

  uint8_t msg_complete = 0;

  // 0xff bytes are eventually ignored by gps
  memset(txData, 0xff, 51);

  while (!msg_complete)
  {
    // sprintf((char*)buffer, "*\n");
    // HAL_UART_Transmit(&huart2, buffer, 1, 0xFFFF);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 50);
    while( hspi4.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // No data is available if 50 bytes == 0xff
    if (memcmp(rxData, txData, 50) == 0)
      return false;
    // Make rxData null terminated
    rxData[49] = '\0';

#ifdef UART_DEBUG_GPS
    for (int i=0; i < 50; i++) {
      sprintf((char*)buffer, "%c", rxData[i]);
      HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    }
#endif    

    pch = strstr((char*)rxData, "$GNRMC");
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
        HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&txData, (uint8_t*)&rxData, sizeof(rxData), 50);
        while( hspi4.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        
        // Make rxData null terminated
        rxData[99] = '\0';

#ifdef UART_DEBUG_GPS
        for (int i=0; i < 50; i++) {
          sprintf((char*)buffer, "%c", rxData[i]);
          HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
        }
#endif

        rmc_end = strstr((char*)rxData, "\r\n");
        *(rmc_end+1) = '\0';
        // Handle case that it's not in there?
        strcat(rmc_msg, (char*)rxData);
        //*(rmc_msg + (rmc_end+2-pch+1)) = '\0';
        msg_complete = 1;
      }

    }
  } // end while !complete

  // sprintf((char*)buffer, "parsed: %s \n head: %d\n", rmc_msg, *head);
  // HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);

  pch = strtok(rmc_msg, ",");  // "$GNRMC"  8th item is COG
  for (int i=0; i < 2; i++)
    pch = strtok(NULL, ",");

  // pointing at status
  if (strcmp(pch, "V") == 0)
    // No fix
    return false;

  pch = strtok(NULL, ",");
  // Pointing at lat formated as ddmm.mmmmm
  // use rxData as a temporary buffer to get dd substring
  strncpy((char*)rxData, pch, 2);
  rxData[2] = '\0';
  //*lat = strtof(pch, NULL);
  *lat = strtof((char*)rxData, NULL) + strtof(pch+2, NULL) / 60.0;

  pch = strtok(NULL, ",");
  // Pointing at NS
  if (strcmp(pch, "S") == 0)
    *lat *= -1;
  
  pch = strtok(NULL, ",");
  // Pointing at lon formatted as dddmm.mmmmm
  strncpy((char*)rxData, pch, 3);
  rxData[3] = '\0';
  //*lng = strtof(pch, NULL);
  *lng = strtof((char*)rxData, NULL) + strtof(pch+3, NULL) / 60.0;
  
  pch = strtok(NULL, ",");
  // Pointing at EW
  if (strcmp(pch, "W") == 0)
    *lng *= -1;

  pch = strtok(NULL, ",");
  *spd = strtof(pch, NULL);

  pch = strtok(NULL, ",");
  // cog
  // strtok skips zero length fields (e.g. ,,dfk,,,) replaces , with \0.  If the msg isn't valid, pch-2 will be \0
  if (*(pch-2) != '\0')
    // make sure cog is in message
    *head = strtof(pch, NULL) * 10;

#ifdef UART_DEBUG_GPS
  sprintf((char*)buffer, "head: %d\nlat: %f\nlng: %f\nspd: %f\n", *head, *lat, *lng, *spd);
  HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
#endif
  return true;
}
