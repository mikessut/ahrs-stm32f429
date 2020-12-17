#include "main.h"
#include "stm32f4xx_hal.h"


CAN_HandleTypeDef hcan1;


uint32_t flash_read(uint32_t address){
    return *(uint32_t*)address;
}

void flash_write(uint32_t address, uint32_t data){
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_1);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data);
    HAL_FLASH_Lock();
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

    // sprintf((char*)buffer, "CAN MSG RCVD: %d len: %d\r\n", rx_header.StdId, rx_header.DLC);
    // HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    // for (uint8_t i=0; i < rx_header.DLC; i++) {
    //   sprintf((char*)buffer, "[%d]: 0x%x\r\n", i, data[i]);
    //   HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    // }  
    return 1;
    /* Example:
    CAN MSG RCVD:0x771 0xf823f001 len: 5
    data: 0xc 0x90 0x1 0xcc 0x74 0x0 0x0 0x0
    This message is sent for BARO: 29.90.
    0xcc 0x74 makes sense: [hex(x) for x in struct.unpack('B'*4, struct.pack('I', 29900))]
    0x0c  Same for difference messages
    0x90
    0x01  These two bytes are the message ID (0x190)
    Spec would lead me to expect and index and and function code, but don't seem them...
    Not sure what the first 3 bytes are. I think they should be Node, Index, and function code
    */

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

#define APP_START_ADDRESS 0x8008000U

int main(void) {
  
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  initialize_CAN();

  HAL_Delay(1000);

  //typedef void (*func_ptr_type)(void);
  //typedef void (func_type)(void);
  //func_ptr_type fp;
  //func_ptr_type fun_ptr = (void(*)())0x8008004;
  uint8_t buf[4] = {1,2,3,4};
  // buf[0] = *((uint8_t*)fun_ptr);
  // buf[1] = *(((uint8_t*)fun_ptr) + 1);
  // buf[2] = *(((uint8_t*)fun_ptr) + 2);
  // buf[3] = *(((uint8_t*)fun_ptr) + 3);
  
  //*fp = (func_type)0x8017ae0;
  send_can_msg(0x123, buf, 4);

  while (1) {
    for (int i=0; i < 5; i++) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_Delay(1000);
    }
    
    //HAL_DeInit();
      /* Disable all interrupts */
    //RCC->CIR = 0x00000000;

    //(*fun_ptr)();
    //((void (*)(void))0x8017ae0)();

    const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);
    do_jump(app_base[0], app_base[1]);

    for (int i=0; i < 5; i++) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_Delay(500);
    }
  }
}