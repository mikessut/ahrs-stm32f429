/*
 * can_fix.h
 *
 *  CANFIX specific values
 *  
 */

#ifndef CAN_FIX_H_
#define CAN_FIX_H_

#include "main.h"
#include "kalman.h"

#define CANFIX_NODE_ID        0x12

// Message id at which node specific messages begin. See spec for details. 
// Message id is determined from sender's node id + this offset.
#define CANFIX_NODE_MSGS_OFFSET   0x6E0
    
#define CANFIX_TACH           0x200
#define CANFIX_PROP           0x202
#define CANFIX_TORQUE         0x204
#define CANFIX_TIT            0x206
#define CANFIX_ITT            0x208
#define CANFIX_TOT            0x20A
#define CANFIX_FUELPS         0x20C
#define CANFIX_OILPS          0x20E
#define CANFIX_OILTS          0x210
#define CANFIX_H2OTS          0x212
#define CANFIX_FUELS          0x214
#define CANFIX_OILLS          0x216
#define CANFIX_H2OLS          0x218
#define CANFIX_FUELF          0x21A
#define CANFIX_FUELP          0x21C
#define CANFIX_MAP            0x21E
#define CANFIX_OILP           0x220
#define CANFIX_OILT           0x222
#define CANFIX_FUELQ_L        0x226
#define CANFIX_FUELQ_R        0x227

#define CANFIX_PITCH_RATE     0x400
#define CANFIX_ROLL_RATE      0x401
#define CANFIX_YAW_RATE       0x402
#define CANFIX_TURN_RATE      0x403
#define CANFIX_STATIC_PRES    0x404
#define CANFIX_PITOT_PRES     0x405
#define CANFIX_TAT            0x406
#define CANFIX_SAT            0x407

#define CANFIX_CHT_1          0x500
#define CANFIX_CHT_2          0x502
#define CANFIX_EGT_1          0x502
#define CANFIX_EGT_2          0x503
#define CANFIX_CARB_TEMP_1    0x50C
#define CANFIX_CARB_TEMP_2    0x50D
#define CANFIX_VOLT           0x50E
#define CANFIX_CURRENT        0x512
#define CANFIX_HOBBS          0x520
#define CANFIX_TACHTM         0x522
#define CANFIX_FTIME          0x545

// AHRS
#define CANFIX_PITCH          0x180
#define CANFIX_ROLL           0x181
#define CANFIX_AOA            0x182
#define CANFIX_IAS            0x183
#define CANFIX_ALT            0x184
#define CANFIX_HEAD           0x185
#define CANFIX_VS             0x186
#define CANFIX_YAW            0x189  // ?
#define CANFIX_ACNOR          0x18A  // "gs ?"
#define CANFIX_ACLAT          0x18B
#define CANFIX_ACLONG         0x18C
#define CANFIX_TAS            0x18D
#define CANFIX_CAS            0x18E
#define CANFIX_MACH           0x18F
#define CANFIX_ALT_SET        0x190
#define CANFIX_PALT           0x191  // ?

// Kalman Filter debug
#define CAN_KF_WX             0x600
#define CAN_KF_WY             0x601
#define CAN_KF_WZ             0x602
#define CAN_KF_WBX            0x603
#define CAN_KF_WBY            0x604
#define CAN_KF_WBZ            0x605
#define CAN_KF_AX             0x606
#define CAN_KF_AY             0x607
#define CAN_KF_AZ             0x608
#define CAN_KF_ABX            0x609
#define CAN_KF_ABY            0x60A
#define CAN_KF_ABZ            0x60B
#define CAN_WX                0x60C
#define CAN_WY                0x60D
#define CAN_WZ                0x60E
#define CAN_AX                0x60F
#define CAN_AY                0x610
#define CAN_AZ                0x611
#define CAN_MAGX              0x612
#define CAN_MAGY              0x613
#define CAN_MAGZ              0x614
#define CAN_PRESSA            0x615
#define CAN_PRESSD            0x616
#define CAN_DT                0x617
#define CAN_HEAD_VEC_X        0x618
#define CAN_HEAD_VEC_Y        0x619

// CANFIX Status
#define CANFIX_STATUS_CODE_ANNUC   0x1
#define CANFIX_STATUS_CODE_QUALITY 0x2
#define CANFIX_STATUS_CODE_FAILURE 0x4

// CANFIX Node Specific Message Control Codes
#define CANFIX_CONTROLCODE_ID              0
#define CANFIX_CONTROLCODE_BITRATE_SET     1
#define CANFIX_CONTROLCODE_NODEID_SET      2
#define CANFIX_CONTROLCODE_DISABLE_PARAM   3
#define CANFIX_CONTROLCODE_ENABLE_PARAM    4
#define CANFIX_CONTROLCODE_NODE_REPORT     5
#define CANFIX_CONTROLCODE_NODE_STATUS     6
#define CANFIX_CONTROLCODE_UPDATE_FW       7
#define CANFIX_CONTROLCODE_TWOWAY          8
#define CANFIX_CONTROLCODE_CFG_SET         9
#define CANFIX_CONTROLCODE_CFG_QRY        10
#define CANFIX_CONTROLCODE_NODE_DESC      11
#define CANFIX_CONTROLCODE_PARAM_SET_MIN  12
#define CANFIX_CONTROLCODE_PARAM_SET_MAX  19

// Control code CFG_SET/QRY key values
// Mag hard iron offsets.  Represented as float  in uT (after sensor scaling)
#define CANFIX_CFG_KEY_HARDIRON_X   0
#define CANFIX_CFG_KEY_HARDIRON_Y   1
#define CANFIX_CFG_KEY_HARDIRON_Z   2
#define CANFIX_CFG_KEY_W_X          3
#define CANFIX_CFG_KEY_W_Y          4
#define CANFIX_CFG_KEY_W_Z          5
#define CANFIX_CFG_KEY_A_X          6
#define CANFIX_CFG_KEY_A_Y          7
#define CANFIX_CFG_KEY_A_Z          8
#define CANFIX_CFG_KEY_Q0           9   // float
#define CANFIX_CFG_KEY_Q1           10
#define CANFIX_CFG_KEY_Q2           11
#define CANFIX_CFG_KEY_Q3           12
// Bit
// 0:  mag update
// 7:  reset
#define CANFIX_CFG_KEY_STATUS       13  // uint8_t
#define CANFIX_CFG_KEY_DPRESS       14  // float pressure offset

void can_debug(Kalman *k, float *a, float *w, float *m, 
               float *abs_press, float *abs_press_temp,
               float *diff_press, float *diff_press_temp,
               float *dt, float *baro, float *temperature);                

int CAN_rx(uint32_t *id, uint8_t *data, uint8_t *len);

// CANfix normal messages
int send_canfix_msg(uint32_t msg_id, uint8_t status, uint8_t *msg, int msglen);
int send_canfix_msg(uint32_t msg_id, uint32_t);
int send_canfix_msg(uint32_t msg_id, int32_t);
int send_canfix_msg(uint32_t msg_id, uint16_t);
int send_canfix_msg(uint32_t msg_id, int16_t msg); 
int send_canfix_msgs(Kalman *k, float *ias, float *tas, float *altitude, float *vs, float *ay);

// CANFIX Node Specific support functions
void canfix_cfg_qry(uint8_t destination, float data);
void canfix_cfg_qry(uint8_t destination, uint8_t data);
void canfix_cfg_set(float *destination, uint8_t idx, uint8_t *can_buffer, 
                    uint8_t reply_destination, uint8_t error_code);
void canfix_cfg_set(uint8_t *destination, uint8_t idx, uint8_t *can_buffer, 
                    uint8_t reply_destination, uint8_t error_code);


// Just bare can data. (Not the 3 extra CANFIX bytes.)  Used for debug.
int send_can_msg(uint32_t msg_id, uint8_t *msg, int len);

int rx_canfix_msgs(float *baro, float *temperature, float *hard_iron,
                   float *wb, float *ab, float *q, uint8_t *status);


#endif /* CAN_FIX_H_ */
