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

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_NONE)
#ifndef GIMBAL_VIRTUAL_H
#define GIMBAL_VIRTUAL_H
#include "gimbal.h"
#include  "user_lib.h"

typedef struct
{
  int temp;
} Gimbal_PID_t;

#endif
#endif
