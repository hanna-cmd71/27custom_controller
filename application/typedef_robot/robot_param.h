/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H


#include "robot_typedef.h"
#include "struct_typedef.h"

// 导入具体的机器人参数配置文件
#include "robot_param_omni_sentry_two.h"
// #include "robot_param_omni_sentry.h"
// #include "robot_param_omni_mini.h"
// #include "robot_param_mecannum_hero.h"
// #include "robot_param_fly.h"

// 选择机器人的各种类型
#define __RC_TYPE RC_DT7         // 遥控器类型
#define __DEBUG 0                  // 调试模式
#define __TUNING 1               // 调参模式
#define __MUSIC_ON 0               // 开启音乐
#define __TUNING_MODE TUNING_GIMBAL_YAW_UP  // 调参模式
#define __HEAT_IMU 1  // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __CALI_DATA_FROM CALI_FROM_FLASH  // 校准数据来源(部分flash损坏的C板需要采用其他数据源)

// USB通信的部分选项
#define __USB_SEND_DEBUG 0  // 发送DEBUG数据

// 模块检查
#ifndef BOARD_CURRENT
#define BOARD_CURRENT C_BOARD_DEFAULT
#endif

#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_NONE
#endif

#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_NONE
#endif

#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_NONE
#endif

#ifndef MECHANICAL_ARM_TYPE
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE
#endif

#ifndef CUSTOM_CONTROLLER_TYPE
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_NONE
#endif

#endif /* ROBOT_PARAM_H */
