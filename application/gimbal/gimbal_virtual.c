/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_virtual.c/h
  * @brief      虚拟云台缓冲器。
  * @note       对外发送函数缓冲接口，在选择GIMBAL_NONE时启用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-04      Harry_Wong      1. 初始化项目，填写对外函数
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include"gimbal_virtual.h"
#if (GIMBAL_TYPE == GIMBAL_NONE)
#include "CAN_receive.h"

Motor_s yaw_motor;
/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{ 
  yaw_motor.type = GIMBAL_DIRECT_YAW_MOTOR_TYPE;
  yaw_motor.can = GIMBAL_YAW_CAN;
  yaw_motor.id = GIMBAL_DIRECT_YAW_ID;
  GetMotorMeasure(&yaw_motor);
  return loop_fp32_constrain(yaw_motor.fdb.pos-yaw_mid_date, -M_PI, M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
  return false ;
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
  return 0.0f ;
}

#endif
