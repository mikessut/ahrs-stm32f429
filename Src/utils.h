

#include "main.h"
#include "can_fix.h"


void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI2_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_CAN1_Init(void);
void MX_I2C3_Init(void);
void MX_SPI1_Init(void);
void MX_SPI3_Init(void);
void MX_SPI4_Init(void);
void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_I2C1_Init(void);

void initialize_CAN();
int send_can_fix_msg(uint32_t msg_id, normal_data *msg, int msglen); // maybe obsolete?
int send_can_fix_msg(uint32_t msg_id, uint8_t status, uint8_t *msg, int msglen);
int send_can_fix_msg(uint32_t msg_id, uint32_t);
int send_can_fix_msg(uint32_t msg_id, uint16_t);

// Just bare can data. (Not the 3 extra CANFIX bytes.)  Used for debug.
int send_can_msg(uint32_t msg_id, float *msg);


extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

