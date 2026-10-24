/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      机器人参数配置文件入口
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *  V1.1.0     Oct-24-2026     Pengui          1. 工程精简：本工程仅服务于自定义控制器
  *
  @verbatim
  ==============================================================================
  本工程只保留自定义控制器(Custom Controller)相关的模块：
    遥控器 / 裁判系统 / IMU / 自定义控制器 / 电机与 CAN 驱动 / USB 上位机通信。
  底盘、云台、发射机构、机械臂、校准、音乐等模块已移除，因此这里只保留
  CUSTOM_CONTROLLER_TYPE 一个模块类型选择。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H


#include "robot_typedef.h"
#include "struct_typedef.h"

// 导入具体的机器人参数配置文件（本工程只有自定义控制器参数文件）
#include "robot_param_engineer_controller.h"

// 选择机器人的各种类型
#define __RC_TYPE RC_DT7           // 遥控器类型
#define __DEBUG 1                  // 调试模式
#define __MUSIC_ON 0               // 开启音乐
#define __TUNING_MODE TUNING_F00T  // 调参模式

#define __TUNING __TUNING_MODE
#define __HEAT_IMU 1                       // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __CALI_DATA_FROM CALI_FROM_FLASH   // 校准数据来源(部分flash损坏的C板需要采用其他数据源)

// USB通信的部分选项
#define __USB_SEND_DEBUG 0  // 发送DEBUG数据

// 模块检查
#ifndef BOARD_CURRENT
#define BOARD_CURRENT C_BOARD_DEFAULT
#endif

#ifndef CUSTOM_CONTROLLER_TYPE
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_ENGINEER
#endif

#endif /* ROBOT_PARAM_H */
