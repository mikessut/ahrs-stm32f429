#include "main.h"
#include "stm32f4xx_hal.h"
#include "canfix.h"


CAN_HandleTypeDef hcan1;


uint32_t flash_read(uint32_t address){
    return *(uint32_t*)address;
}

void flash_write(uint32_t address, uint32_t data){
    //HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data);
    //HAL_FLASH_Lock();
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}


/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void initialize_CAN()
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    //printf("Error configuring CAN filter\r\n");
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    //printf("Error starting CAN\r\n");
  }
}

int send_can_msg(uint32_t msg_id, uint8_t *msg, int len)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t tx_mailbox;

  tx_header.StdId = msg_id;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = len;
  tx_header.TransmitGlobalTime = DISABLE;

  for (int i = 0; i < len; i++) {
      tx_data[i] = *(msg + i);
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {}
  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
    //printf("Error queing CAN TX msg\r\n");
    return -1;
  }
  return 0;
}

int CAN_rx(uint32_t *id, uint8_t *data, uint8_t *len) 
{
  uint32_t nfifo = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);

  if (nfifo > 0) {
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data);
    *id = rx_header.StdId;
    *len = rx_header.DLC;
    return 1;
  } else {
    return 0;
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE15  STATUS (goes to LED)*/
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
//#if defined(STM32F7) || defined(STM32H7)
    // disable caches on F7 before starting program
    __DSB();
    __ISB();
    //SCB_DisableDCache();
    //SCB_DisableICache();
//#endif

    //chSysLock();    

    // we set sp as well as msp to avoid an issue with loading NuttX
    asm volatile(
        "mov sp, %0	\n"
        "msr msp, %0	\n"
        "bx	%1	\n"
        : : "r"(stacktop), "r"(entrypoint) :);
}

#define APP_START_ADDRESS 0x8020000U


typedef enum statetype {
  IDLE,
  ERASE_SECTOR,
  UPDATE_FW_SET_ADDR,
  UPDATE_FW_UPDATE_FLASH,
  DO_JUMP
} statetype;

int main(void) {
  
  statetype state = IDLE;
  uint32_t canid;
  uint8_t canmsglen;
  uint8_t candata[8];
  uint8_t hostnode;
  uint8_t channel;

  uint32_t flash_addr;
  uint32_t flash_len;
  uint32_t flash_ctr;

  uint8_t led;
  uint32_t ticks;
  uint16_t led_delay = 500;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  initialize_CAN();

  HAL_Delay(1000);

  led = 1;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  ticks = HAL_GetTick();

  while (1) {
    switch (state) {
      case IDLE:
        if (HAL_GetTick() > 10000) {
          state = DO_JUMP;
          break;
        }
        if (CAN_rx(&canid, candata, &canmsglen)) {
          if ((canid >= CANFIX_NODE_MSGS_OFFSET) && (canid <= (CANFIX_NODE_MSGS_OFFSET+256)) &&
                (candata[0] == CANFIX_CONTROLCODE_UPDATE_FW) && 
                (candata[1] == CANFIX_NODE_ID) &&
                (canmsglen == 5)) {
            // Update Firmware command.  Reply and change state
            led_delay = 250;
            HAL_FLASH_Unlock();
            channel = candata[4];
            hostnode = canid - CANFIX_NODE_MSGS_OFFSET;
            candata[1] = hostnode;
            candata[2] = 0x0;
            send_can_msg(CANFIX_NODE_MSGS_OFFSET + CANFIX_NODE_ID, candata, 3);
            state = ERASE_SECTOR;
          }
        }
        break;
      case ERASE_SECTOR:
        if (CAN_rx(&canid, candata, &canmsglen)) {
          if ((canid == (CANFIX_TWOWAY_MSGS_OFFSET + channel*2)) && (canmsglen == 1)) {
            if (candata[0] == 0xff) {
              // msg indicates done erasing, jump to update_FW_SET_ADDR
              candata[0] = 0x2;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
              state = UPDATE_FW_SET_ADDR;
            } else {
              FLASH_Erase_Sector(FLASH_SECTOR_0 + candata[0],VOLTAGE_RANGE_3);
              candata[0] = 0x1;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
            }
          }
        }
        break;
      case UPDATE_FW_SET_ADDR:
        if (CAN_rx(&canid, candata, &canmsglen)) {
          if ((canid == (CANFIX_TWOWAY_MSGS_OFFSET + channel*2)) && (canmsglen == 8)) {
            flash_addr = *((uint32_t*)candata);
            flash_len = *(((uint32_t*)candata) + 1);
            if ((flash_addr ==0) && (flash_len == 0)) {
              // We're done. Go ahead and jump
              HAL_FLASH_Lock();
              candata[0] = 0x6;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
              HAL_Delay(1000);
              state = DO_JUMP;  
            } else {
              flash_ctr = 0;
              candata[0] = 0x3;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
              state = UPDATE_FW_UPDATE_FLASH;
            }
          }
        }
        break;
      case UPDATE_FW_UPDATE_FLASH:
        if (CAN_rx(&canid, candata, &canmsglen)) {
          if ((canid == (CANFIX_TWOWAY_MSGS_OFFSET + channel*2)) && (canmsglen == 8)) {
            // write to flash
            flash_write(flash_addr + flash_ctr, *((uint32_t*)candata));
            flash_write(flash_addr + flash_ctr + 4, *(((uint32_t*)candata) + 1));
            // Update ctr
            flash_ctr += 8;
            if (flash_ctr >= flash_len) {
              // Written everthing in this section. Acknowledge and change state to SET_ADDR
              candata[0] = 0x5;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
              state = UPDATE_FW_SET_ADDR;
            } else {
              // acknowledge receipt of frame, stay in same state
              candata[0] = 0x4;
              send_can_msg(CANFIX_TWOWAY_MSGS_OFFSET + channel*2 + 1, candata, 1);
            }
          }
        }
        break;
      case DO_JUMP:
        const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);
        do_jump(app_base[0], app_base[1]);
        break;
    }

    // LED handler
    if ((HAL_GetTick() - ticks) > led_delay) {
      if (led) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        led = 0;
      } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        led = 1;
      }
      ticks = HAL_GetTick();
    }
  }
}