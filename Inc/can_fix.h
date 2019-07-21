/*
 * can_fix.h
 *
 *  Created on: Jul 17, 2019
 *      Author: Kyle
 */

#ifndef CAN_FIX_H_
#define CAN_FIX_H_

#define TACH 0x200
#define PROP 0x202
#define TORQUE 0x204
#define TIT 0x206
#define ITT 0x208
#define TOT 0x20A
#define FUELPS 0x20C
#define OILPS 0x20E
#define OILTS 0x210
#define H2OTS 0x212
#define FUELS 0x214
#define OILLS 0x216
#define H2OLS 0x218
#define FUELF 0x21A
#define FUELP 0x21C
#define MAP 0x21E
#define OILP 0x220
#define OILT 0x222
#define FUELQ_L 0x226
#define FUELQ_R 0x227

#define CHT_1_FIX_ID 0x500
#define CHT_2_FIX_ID 0x502
#define EGT_1_FIX_ID 0x502
#define EGT_2_FIX_ID 0x503
#define CARB_TEMP_1_FIX_ID 0x50C
#define CARB_TEMP_2_FIX_ID 0x50D
#define VOLT_FIX_ID 0x50E
#define CURRENT_FIX_ID 0x512
#define HOBBS_FIX_ID 0x520
#define TACHTM_FIX_ID 0x522
#define FTIME_FIX_ID 0x545

#define STATUS_CODE_ANNUC 0x1
#define STATUS_CODE_QUALITY 0x2
#define STATUS_CODE_FAILURE 0x4


typedef struct {
  uint16_t alarm_code;
  uint8_t data[6];
} node_alarm_data;

typedef struct {
  uint8_t node;
  uint8_t index;
  uint8_t status_code;
  uint32_t data;
} normal_data;

typedef struct {
  uint8_t control_code;
  uint8_t data[7];
} node_specific_data;

/* returns size of array if successfully created. Otherwise returns negative error code */
int32_t can_fix_construct_normal_data_msg(normal_data data, uint8_t *byte_stream, uint8_t size)
{
  return 0;
}

#endif /* CAN_FIX_H_ */
