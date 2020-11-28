

#include "main.h"
#include "can_fix.h"
#include "kalman.h"

// UART debug flags
#define PRINT_GYRO
#define PRINT_ACCEL
#define PRINT_MAG
#define PRINT_PRESSURE
#define PRINT_KF_STATE
#define PRINT_CAN_RX_DEBUG
#define PRINT_CAN_RX

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
int CAN_rx(uint32_t *id, uint8_t *data, uint8_t *len);
int send_can_fix_msg(uint32_t msg_id, uint8_t status, uint8_t *msg, int msglen);
int send_can_fix_msg(uint32_t msg_id, uint32_t);
int send_can_fix_msg(uint32_t msg_id, int32_t);
int send_can_fix_msg(uint32_t msg_id, uint16_t);
int send_can_fix_msg(uint32_t msg_id, int16_t msg); 
int send_canfix_msgs(Kalman *k, float *ias, float *tas, float *altitude, float *vs, float *ay);

// Just bare can data. (Not the 3 extra CANFIX bytes.)  Used for debug.
int send_can_msg(uint32_t msg_id, uint8_t *msg, int len);

int rx_canfix_msgs(float *baro, float *temperature, float *hard_iron,
                   int32_t *wb, int32_t *ab, float *q);

void uart_debug(Kalman *k, float *a, float *w, float *m, 
                float *abs_press, float *abs_press_temp,
                float *diff_press, float *diff_press_temp);

void can_debug(Kalman *k, float *a, float *w, float *m, 
               float *abs_press, float *abs_press_temp,
               float *diff_press, float *diff_press_temp,
               float *dt, float *baro, float *temperature);                


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

// Assumes a first order IIR with a0 = 1 and single a1 value
typedef struct {
  float b[2] = {0.01546629, 0.01546629};
  float x = 0;  // previous input
  float y = 0;  // previous output
  float a = -0.96906742;
} IIRFilterDef;

float iir_filter(IIRFilterDef *filter, float y);

void rotate_sensors(float *q, float *a, float *w, float *m);