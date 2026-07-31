/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_SBzdatou.c/h
  * @brief      CAN发送函数，通过CAN信号控制DJI电机 (SB步兵自定义大头)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_SBZDATOU_H
#define CAN_CMD_SBZDATOU_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define SBZDATOU1_EXT_ID    0x100U
#define SBZDATOU2_EXT_ID    0x200U
#define SBZDATOU3_EXT_ID    0x300U
#define SBZDATOU4_EXT_ID    0x400U
#define SBZDATOU5_EXT_ID    0x500U
#define SBZDATOU6_EXT_ID    0x600U

#define SBZDATOU1_RX_EXT_ID    SBZDATOU1_EXT_ID
#define SBZDATOU2_RX_EXT_ID    SBZDATOU2_EXT_ID
#define SBZDATOU3_RX_EXT_ID    SBZDATOU3_EXT_ID
#define SBZDATOU4_RX_EXT_ID    SBZDATOU4_EXT_ID
#define SBZDATOU5_RX_EXT_ID    SBZDATOU5_EXT_ID
#define SBZDATOU6_RX_EXT_ID    SBZDATOU6_EXT_ID

typedef struct
{
    uint32_t ext_id;
    uint16_t ecd;
    uint32_t last_fdb_time;
} SBzdatouMeasure_s;
#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

extern void SBzdatou_ecd_get(uint8_t can, uint32_t ext_id);
extern void SBzdatou_torque_control(
    uint8_t can, uint32_t ext_id, uint8_t direction, int32_t torque);

extern void SBzdatouFdbData(SBzdatouMeasure_s * sbzdatou_measure, uint32_t ext_id, uint8_t rx_data[8]);
extern const SBzdatouMeasure_s * GetSBzdatouMeasurePoint(uint8_t can);

#endif  // CAN_CMD_SBZDATOU_H

/************************ END OF FILE ************************/
