/**
  * @file       robot_param_engineer_controller.h
  * @brief      这里是工程自定义控制器参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "motor.h"
#include "robot_typedef.h"

#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_ENGINEER  // 选择自定义控制器类型

// 机器人物理参数
/*-------------------- Custom Controller --------------------*/
//motor parameters ---------------------

#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_1_ID 2
#define JOINT_MOTOR_2_ID 3
#define JOINT_MOTOR_3_ID 1
#define JOINT_MOTOR_4_ID 2
#define JOINT_MOTOR_5_ID 3

#define JOINT_MOTOR_0_CAN 1
#define JOINT_MOTOR_1_CAN 1
#define JOINT_MOTOR_2_CAN 1
#define JOINT_MOTOR_3_CAN 1
#define JOINT_MOTOR_4_CAN 1
#define JOINT_MOTOR_5_CAN 1

#define JOINT_MOTOR_0_TYPE DJI_M6020
#define JOINT_MOTOR_1_TYPE DJI_M6020
#define JOINT_MOTOR_2_TYPE DJI_M6020
#define JOINT_MOTOR_3_TYPE DJI_M3508
#define JOINT_MOTOR_4_TYPE DJI_M3508
#define JOINT_MOTOR_5_TYPE DJI_M2006

#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION (1)
#define JOINT_MOTOR_2_DIRECTION (-1)
#define JOINT_MOTOR_3_DIRECTION (-1)
#define JOINT_MOTOR_4_DIRECTION (1)
#define JOINT_MOTOR_5_DIRECTION 1

#define JOINT_MOTOR_0_MODE 0
#define JOINT_MOTOR_1_MODE 0
#define JOINT_MOTOR_2_MODE 0
#define JOINT_MOTOR_3_MODE 0
#define JOINT_MOTOR_4_MODE 0
#define JOINT_MOTOR_5_MODE 0

#define J0_ANGLE_TRANSFORM 0.0f
#define J1_ANGLE_TRANSFORM (M_PI_2)
#define J2_ANGLE_TRANSFORM (M_PI / 3)
#define J3_ANGLE_TRANSFORM 0.0f
#define J4_ANGLE_TRANSFORM 0.0f
#define J5_ANGLE_TRANSFORM 0.0f

// clang-format off
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION  M_PI * 2
#define MAX_JOINT_1_POSITION  1.20f
#define MAX_JOINT_2_POSITION  0.00f
#define MAX_JOINT_3_POSITION  M_PI * 4
#define MAX_JOINT_4_POSITION  M_PI_2
#define MAX_JOINT_5_POSITION  M_PI * 4

#define MAX_VIRTUAL_JOINT_4_POSITION  M_PI_2
#define MAX_VIRTUAL_JOINT_5_POSITION  M_PI * 4 
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION -MAX_JOINT_0_POSITION
#define MIN_JOINT_1_POSITION -1.10f
#define MIN_JOINT_2_POSITION -2.50f
#define MIN_JOINT_3_POSITION -MAX_JOINT_3_POSITION
#define MIN_JOINT_4_POSITION -MAX_JOINT_4_POSITION
#define MIN_JOINT_5_POSITION -MAX_JOINT_5_POSITION

#define MIN_VIRTUAL_JOINT_4_POSITION -MAX_VIRTUAL_JOINT_4_POSITION
#define MIN_VIRTUAL_JOINT_5_POSITION -MAX_VIRTUAL_JOINT_5_POSITION
//clang-format on

//LPF parameters ---------------------
#define J0_LPF_ALPHA 0.0f
#define J1_LPF_ALPHA 0.0f
#define J2_LPF_ALPHA 0.0f
#define J3_LPF_ALPHA 0.0f
#define J4_LPF_ALPHA 0.0f
#define J5_LPF_ALPHA 0.0f
//PID parameters ---------------------
//J0速度环PID参数
#define KP_JOINT_0_VELOCITY 0.0f
#define KI_JOINT_0_VELOCITY 0.0f
#define KD_JOINT_0_VELOCITY 0.0f
#define MAX_IOUT_JOINT_0_VELOCITY 0.0f
#define MAX_OUT_JOINT_0_VELOCITY 0.0f
//J1速度环PID参数
#define KP_JOINT_1_VELOCITY 0.0f
#define KI_JOINT_1_VELOCITY 0.0f
#define KD_JOINT_1_VELOCITY 0.0f
#define MAX_IOUT_JOINT_1_VELOCITY 0.0f
#define MAX_OUT_JOINT_1_VELOCITY 0.0f
//J2速度环PID参数
#define KP_JOINT_2_VELOCITY 0.0f
#define KI_JOINT_2_VELOCITY 0.0f
#define KD_JOINT_2_VELOCITY 0.0f
#define MAX_IOUT_JOINT_2_VELOCITY 0.0f
#define MAX_OUT_JOINT_2_VELOCITY 0.0f
//J3速度环PID参数
#define KP_JOINT_3_VELOCITY 0.0f
#define KI_JOINT_3_VELOCITY 0.0f
#define KD_JOINT_3_VELOCITY 0.0f
#define MAX_IOUT_JOINT_3_VELOCITY 0.0f
#define MAX_OUT_JOINT_3_VELOCITY 0.0f
//J4速度环PID参数
#define KP_JOINT_4_VELOCITY 0.0f
#define KI_JOINT_4_VELOCITY 0.0f
#define KD_JOINT_4_VELOCITY 0.0f
#define MAX_IOUT_JOINT_4_VELOCITY 0.0f
#define MAX_OUT_JOINT_4_VELOCITY 0.0f
//J5速度环PID参数
#define KP_JOINT_5_VELOCITY 0.0f
#define KI_JOINT_5_VELOCITY 0.0f
#define KD_JOINT_5_VELOCITY 0.0f
#define MAX_IOUT_JOINT_5_VELOCITY 0.0f
#define MAX_OUT_JOINT_5_VELOCITY 0.0f
// Init parameters ---------------------
// Other parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
