/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef SHOOT_H
#define SHOOT_H

#include "robot_param.h"

#if SHOOT_TYPE == SHOOT_NONE

#include "shoot_fric_trigger.h"
#include "struct_typedef.h"

// 任务相关宏定义
#define SHOOT_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define SHOOT_CONTROL_TIME 1      // 任务控制间隔 1ms

typedef struct
{
  int temp;
} Shoot_s;

#endif  // SHOOT_TYPE
#endif
/*------------------------------ End of File ------------------------------*/
