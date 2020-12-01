#include "canfix.h"


extern uint8_t buffer[200];
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern float dpress_offset;


int send_canfix_msg(uint32_t msg_id, uint32_t msg) 
{
  return send_canfix_msg(msg_id, 0, (uint8_t*)&msg, 4);
}

int send_canfix_msg(uint32_t msg_id, int32_t msg) 
{
  return send_canfix_msg(msg_id, 0, (uint8_t*)&msg, 4);
}

int send_canfix_msg(uint32_t msg_id, uint16_t msg) 
{
  return send_canfix_msg(msg_id, 0, (uint8_t*)&msg, 2);
}

int send_canfix_msg(uint32_t msg_id, int16_t msg) 
{
  return send_canfix_msg(msg_id, 0, (uint8_t*)&msg, 2);
}

int send_canfix_msg(uint32_t msg_id, uint8_t status, uint8_t *msg, int msglen)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox;
  int i = 3;

  tx_header.StdId = msg_id;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = msglen + 3;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = CANFIX_NODE_ID;
  tx_data[1] = 0;   // index
  tx_data[2] = status;

  for (i = 0; i < msglen; i++) {
      tx_data[i + 3] = *(msg + i);
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {}
  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
    //printf("Error queing CAN TX msg\r\n");
    return -1;
  }
  return 0;
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

int rx_canfix_msgs(float *baro, float *temperature, float *hard_iron,
                   float *wb, float *ab, float *q, uint8_t *status) {
  uint32_t id;
  uint8_t data[10];
  uint8_t len;
  if (CAN_rx(&id, data, &len)) {
    // sprintf((char*)buffer, "CAN rcvd: id: %d\r\n", id);
    // HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    if ((id >= CANFIX_NODE_MSGS_OFFSET) && (id <= 2047)) {
      // Node specific message data 
      // Byte
      // 0: Control code
      // 1-7: code specific
      if ((data[0] >= CANFIX_CONTROLCODE_PARAM_SET_MIN) && (data[0] <= CANFIX_CONTROLCODE_PARAM_SET_MAX)) {
        // Node set command
        // sprintf((char*)buffer, "Node set command rcvd\r\n");
        // HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
        if ((*((uint16_t*)(&data[1]))) == CANFIX_ALT_SET) {
          // Altimeter setting 
          // sprintf((char*)buffer, "Baro set\r\n");
          // HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
          *baro = (float)(*((uint16_t*)(&data[3]))) / 1000.0;
        }
      } else if ((data[0] == CANFIX_CONTROLCODE_CFG_SET) && (data[1] == CANFIX_NODE_ID)) {
        // Configure messages
        // Packet format: DATA
        // ID                0             1     2            3...
        // sending+1760   0x09   Destination   Key   Key specific data fmt
        if ((data[2] >= CANFIX_CFG_KEY_HARDIRON_X) && (data[2] <= CANFIX_CFG_KEY_HARDIRON_Z) && (len == 7)) {
          canfix_cfg_set(hard_iron, data[2] - CANFIX_CFG_KEY_HARDIRON_X, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        } else if ((data[2] >= CANFIX_CFG_KEY_W_X) && (data[2] <= CANFIX_CFG_KEY_W_Z) && (len == 7)) {
          canfix_cfg_set(wb, data[2] - CANFIX_CFG_KEY_W_X, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        } else if ((data[2] >= CANFIX_CFG_KEY_A_X) && (data[2] <= CANFIX_CFG_KEY_A_Z) && (len == 7)) {
          canfix_cfg_set(ab, data[2] - CANFIX_CFG_KEY_A_X, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        } else if ((data[2] >= CANFIX_CFG_KEY_Q0) && (data[2] <= CANFIX_CFG_KEY_Q3) && (len == 7)) {
          canfix_cfg_set(q, data[2] - CANFIX_CFG_KEY_Q0, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        } else if ((data[2] == CANFIX_CFG_KEY_STATUS) && (len == 4)) {
          canfix_cfg_set(status, 0, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        } else if ((data[2] == CANFIX_CFG_KEY_DPRESS) && (len == 7)) {
          canfix_cfg_set(&dpress_offset, 0, data, (uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), 0);
        }
      } else if ((data[0] == CANFIX_CONTROLCODE_CFG_QRY) && (data[1] == CANFIX_NODE_ID)) {
        // Configure query messages
        if ((data[2] >= CANFIX_CFG_KEY_HARDIRON_X) && (data[2] <= CANFIX_CFG_KEY_HARDIRON_Z) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), hard_iron[data[2] - CANFIX_CFG_KEY_HARDIRON_X]);    
        } else if ((data[2] >= CANFIX_CFG_KEY_W_X) && (data[2] <= CANFIX_CFG_KEY_W_Z) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), wb[data[2] - CANFIX_CFG_KEY_W_X]);  
        } else if ((data[2] >= CANFIX_CFG_KEY_A_X) && (data[2] <= CANFIX_CFG_KEY_A_Z) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), ab[data[2] - CANFIX_CFG_KEY_A_X]);  
        } else if ((data[2] >= CANFIX_CFG_KEY_Q0) && (data[2] <= CANFIX_CFG_KEY_Q3) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), q[data[2] - CANFIX_CFG_KEY_Q0]);    
        } else if ((data[2] == CANFIX_CFG_KEY_STATUS) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), *status);
        } else if ((data[2] == CANFIX_CFG_KEY_DPRESS) && (len == 3)) {
          canfix_cfg_qry((uint8_t)(id-CANFIX_NODE_MSGS_OFFSET), dpress_offset);    
        }
      } 
    } else if (id == CANFIX_SAT) {
      // normal CANFIX msg for temperature
      *temperature = (float)(*((int16_t*)(&data[3]))) / 100.0;
    }
  }
}

/* Return 1 if message received */
// Passed data buffer has to be large enough for CAN message.
int CAN_rx(uint32_t *id, uint8_t *data, uint8_t *len) 
{
  uint32_t nfifo = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);

  if (nfifo > 0) {
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data);
    *id = rx_header.StdId;
    *len = rx_header.DLC;

    sprintf((char*)buffer, "CAN MSG RCVD: %d len: %d\r\n", rx_header.StdId, rx_header.DLC);
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    for (uint8_t i=0; i < rx_header.DLC; i++) {
      sprintf((char*)buffer, "[%d]: 0x%x\r\n", i, data[i]);
      HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
    }  
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


int send_canfix_msgs(Kalman *k, float *ias, float *tas, float *altitude, float *vs, float *ay) {
  
    send_canfix_msg(CANFIX_ROLL, (int16_t)(k->roll()*180.0/M_PI * 100.0));
    send_canfix_msg(CANFIX_PITCH, (int16_t)(k->pitch()*180.0/M_PI * 100.0));
    send_canfix_msg(CANFIX_HEAD, (uint16_t)(positive_heading(k->heading())*180.0/M_PI * 10.0));
    send_canfix_msg(CANFIX_IAS, (uint16_t)(*ias * 10.0));
    send_canfix_msg(CANFIX_TAS, (uint16_t)(*tas * 10.0));
    send_canfix_msg(CANFIX_ALT, (int32_t)(*altitude));
    send_canfix_msg(CANFIX_VS, (int16_t)(*vs));
    send_canfix_msg(CANFIX_ACLAT, (int16_t)(((k->x(I_AY)) / G) * 1000.0));
}

void can_debug(Kalman *k, float *a, float *w, float *m, 
               float *abs_press, float *abs_press_temp,
               float *diff_press, float *diff_press_temp,
               float *dt, float *baro, float *temperature)
{
  for (int i=0; i < 3; i++) {
    send_can_msg(CAN_KF_WX+i, (uint8_t*)&k->x(I_WX+i), sizeof(float));
    send_can_msg(CAN_WX+i, (uint8_t*)(w + i), sizeof(float));

    send_can_msg(CAN_KF_AX+i, (uint8_t*)&k->x(I_AX+i), sizeof(float));
    send_can_msg(CAN_AX+i, (uint8_t*)(a + i), sizeof(float));

    send_can_msg(CAN_MAGX+i, (uint8_t*)(m + i), sizeof(float));
  }
  send_can_msg(CAN_PRESSA, (uint8_t*)abs_press, sizeof(float));
  send_can_msg(CAN_PRESSD, (uint8_t*)diff_press, sizeof(float));
  send_can_msg(CAN_DT, (uint8_t*)dt, sizeof(float));
  send_canfix_msg(CANFIX_ALT_SET, (uint16_t)((*baro)*1000));
  send_canfix_msg(CANFIX_SAT, (int16_t)((*temperature)*100));
}

void canfix_cfg_qry(uint8_t destination, float data) {
  uint8_t reply[7] = {CANFIX_CONTROLCODE_CFG_QRY, destination, 0, 0, 0, 0, 0};
  *((float*)(&reply[3])) = data;
  send_can_msg(CANFIX_NODE_MSGS_OFFSET + CANFIX_NODE_ID, reply, 7);
}

void canfix_cfg_qry(uint8_t destination, uint8_t data) {
  uint8_t reply[4] = {CANFIX_CONTROLCODE_CFG_QRY, destination, 0, 0};
  reply[3] = data;
  send_can_msg(CANFIX_NODE_MSGS_OFFSET + CANFIX_NODE_ID, reply, 4);
}

void canfix_cfg_set(float *destination, uint8_t idx, uint8_t *can_buffer, 
                    uint8_t reply_destination, uint8_t error_code) {
  destination[idx] = *((float*)(&can_buffer[3]));
  uint8_t reply[3] = {CANFIX_CONTROLCODE_CFG_SET, reply_destination, error_code};
  send_can_msg(CANFIX_NODE_MSGS_OFFSET + CANFIX_NODE_ID, reply, 3);
}

void canfix_cfg_set(uint8_t *destination, uint8_t idx, uint8_t *can_buffer, 
                    uint8_t reply_destination, uint8_t error_code) {
  destination[idx] = can_buffer[3];
  uint8_t reply[3] = {CANFIX_CONTROLCODE_CFG_SET, reply_destination, error_code};
  send_can_msg(CANFIX_NODE_MSGS_OFFSET + CANFIX_NODE_ID, reply, 3);
}