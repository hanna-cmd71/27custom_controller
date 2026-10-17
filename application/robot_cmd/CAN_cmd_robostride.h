/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_robostride.c/h
  * @brief      CAN发送函数，通过CAN信号控制RobStride电机（MIT模式）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-23-2026     李喆            1. 完成MIT模式控制。
  *
  @verbatim
  ==============================================================================
    RobStride MIT协议：CAN2.0标准帧，默认波特率1Mbps。
    StdId = 电机CAN_ID（bit10~8为模式，使能/控制/停止时为0）。
    反馈帧（应答指令1）：Byte0为电机CAN_ID。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_ROBOSTRIDE_H
#define CAN_CMD_ROBOSTRIDE_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

/* MIT模式各电机CAN_ID（可按实际配置修改） */
#define ROBOSTRIDE1_ID    0x01U
#define ROBOSTRIDE2_ID    0x02U
#define ROBOSTRIDE3_ID    0x03U
#define ROBOSTRIDE4_ID    0x04U
#define ROBOSTRIDE5_ID    0x05U
#define ROBOSTRIDE6_ID    0x06U

/* MIT反馈帧标识（应答指令1）使用主机CAN_ID，Byte0携带电机CAN_ID */
#define ROBOSTRIDE_MASTER_ID    0x15

/* MIT模式参数范围（详见使用手册第6章） */
#define ROBOSTRIDE_P_MIN    (-12.57f)
#define ROBOSTRIDE_P_MAX    (12.57f)
#define ROBOSTRIDE_V_MIN    (-33.0f)
#define ROBOSTRIDE_V_MAX    (33.0f)
#define ROBOSTRIDE_KP_MIN   (0.0f)
#define ROBOSTRIDE_KP_MAX   (500.0f)
#define ROBOSTRIDE_KD_MIN   (0.0f)
#define ROBOSTRIDE_KD_MAX   (5.0f)
#define ROBOSTRIDE_T_MIN    (-14.0f)
#define ROBOSTRIDE_T_MAX    (14.0f)

typedef struct
{
    uint8_t motor_id;
    float pos;
    float vel;
    float tor;
    float temp;
    uint8_t mode_state;
    uint8_t fault;
    uint8_t warning;
    uint32_t last_fdb_time;
} RobostrideMeasure_s;
#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

extern void Robostride_enable(uint8_t can, uint16_t id);
extern void Robostride_mit_control(
    uint8_t can, uint16_t id, float pos, float vel, float kp, float kd, float torque);

extern void RobostrideFdbData(RobostrideMeasure_s * robostride_measure, uint8_t rx_data[8]);
extern const RobostrideMeasure_s * GetRobostrideMeasurePoint(uint8_t can);

#endif  // CAN_CMD_ROBOSTRIDE_H

/************************ END OF FILE ************************/
