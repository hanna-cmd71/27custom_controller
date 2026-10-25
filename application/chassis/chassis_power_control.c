/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_power_control.c/h
  * @brief      底盘功率控制
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "CAN_receive.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         无
  */
fp32 chassis_power_control()                                                                                                                                                                                                                                                                                                                                                                                                 
 {
  //  fp32 chassis_power = 0.0f;
  //  fp32 chassis_power_buffer = 0.0f;
   fp32 ratio = 0.0f;
 
  //  get_chassis_power_and_buffer(chassis_power,chassis_power_buffer); //从裁判系统中读取底盘功率 和 能量缓冲
  //  ratio = chassis_power_buffer * ( 1.0f / WARNING_POWER_BUFF); //计算能量缓冲比例
   return ratio;
 }