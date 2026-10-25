/**
  * @file       robot_param_engineer.h
  * @brief      这里是工程机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "macro_typedef.h"
#include "motor.h"
#include "robot_typedef.h"

#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_ENGINEER_ARM  // 选择机械臂类型

// 机器人物理参数

/*-------------------- Chassis --------------------*/
//motor type
//physical parameters ---------------------
//upper_limit parameters ---------------------
//lower_limit parameters ---------------------
//PID parameters ---------------------

/*-------------------- Mechanical arm --------------------*/
#define ARM_DJI_CAN 1
#define ARM_DM_CAN 2
//rc parameters ---------------------

#define MECHANICAL_ARM_MODE_CHANNEL DT7_SW_LEFT  // 机械臂模式切换通道
#define PUMP_CHANNEL DT7_SW_RIGHT  // 气泵切换通道

//motor parameters ---------------------

#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_1_ID 2
#define JOINT_MOTOR_2_ID 3
#define JOINT_MOTOR_3_ID 1
#define JOINT_MOTOR_4_ID 7
#define JOINT_MOTOR_5_ID 8

#define JOINT_MOTOR_0_CAN ARM_DM_CAN
#define JOINT_MOTOR_1_CAN ARM_DM_CAN
#define JOINT_MOTOR_2_CAN ARM_DM_CAN
#define JOINT_MOTOR_3_CAN ARM_DJI_CAN
#define JOINT_MOTOR_4_CAN ARM_DJI_CAN
#define JOINT_MOTOR_5_CAN ARM_DJI_CAN

#define JOINT_MOTOR_0_TYPE DM_4310
#define JOINT_MOTOR_1_TYPE DM_8009
#define JOINT_MOTOR_2_TYPE DM_8009
#define JOINT_MOTOR_3_TYPE DJI_M6020
#define JOINT_MOTOR_4_TYPE DJI_M2006
#define JOINT_MOTOR_5_TYPE DJI_M2006

#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION 1
#define JOINT_MOTOR_2_DIRECTION 1
#define JOINT_MOTOR_3_DIRECTION 1
#define JOINT_MOTOR_4_DIRECTION 1
#define JOINT_MOTOR_5_DIRECTION 1

#define JOINT_MOTOR_0_REDUCATION_RATIO 2.24f
#define JOINT_MOTOR_1_REDUCATION_RATIO 2.0f
#define JOINT_MOTOR_2_REDUCATION_RATIO 1
#define JOINT_MOTOR_3_REDUCATION_RATIO 1
#define JOINT_MOTOR_4_REDUCATION_RATIO 36
#define JOINT_MOTOR_5_REDUCATION_RATIO 36

#define JOINT_MOTOR_0_MODE DM_MODE_MIT
#define JOINT_MOTOR_1_MODE DM_MODE_MIT
#define JOINT_MOTOR_2_MODE DM_MODE_MIT
#define JOINT_MOTOR_3_MODE 0
#define JOINT_MOTOR_4_MODE 0
#define JOINT_MOTOR_5_MODE 0

#define J0_ANGLE_TRANSFORM 0.0f
#define J1_ANGLE_TRANSFORM 0.0f
#define J2_ANGLE_TRANSFORM 0.0f
#define J3_ANGLE_TRANSFORM 0.0f
#define J4_ANGLE_TRANSFORM 0.0f
#define J5_ANGLE_TRANSFORM 0.0f

// clang-format off
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION  M_PI * 3 / 2
#define MAX_JOINT_1_POSITION  1.20f
#define MAX_JOINT_2_POSITION  0.00f
#define MAX_JOINT_3_POSITION  M_PI * 4
#define MAX_JOINT_4_POSITION  M_PI
#define MAX_JOINT_5_POSITION  M_PI
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION -MAX_JOINT_0_POSITION
#define MIN_JOINT_1_POSITION -1.10f
#define MIN_JOINT_2_POSITION -2.50f
#define MIN_JOINT_3_POSITION -MAX_JOINT_3_POSITION
#define MIN_JOINT_4_POSITION -MAX_JOINT_3_POSITION
#define MIN_JOINT_5_POSITION  0.0f
//clang-format on

//LPF parameters ---------------------
#define J0_LPF_ALPHA 0.0f
#define J1_LPF_ALPHA 0.0f
#define J2_LPF_ALPHA 0.0f
#define J3_LPF_ALPHA 0.985f
#define J4_LPF_ALPHA 0.9f
#define J5_LPF_ALPHA 0.0f

//PID parameters ---------------------
//J0角度环PID参数
#define KP_JOINT_0_ANGLE 10.0f
#define KI_JOINT_0_ANGLE 0.1f
#define KD_JOINT_0_ANGLE 0.0f
#define MAX_IOUT_JOINT_0_ANGLE 0.3f
#define MAX_OUT_JOINT_0_ANGLE 5.0f
//J0速度环PID参数
#define KP_JOINT_0_VELOCITY 0.0f
#define KI_JOINT_0_VELOCITY 0.0f
#define KD_JOINT_0_VELOCITY 0.0f
#define MAX_IOUT_JOINT_0_VELOCITY 0.0f
#define MAX_OUT_JOINT_0_VELOCITY 0.0f

//J1角度环PID参数
#define KP_JOINT_1_ANGLE 10.0f
#define KI_JOINT_1_ANGLE 0.1f
#define KD_JOINT_1_ANGLE 0.0f
#define MAX_IOUT_JOINT_1_ANGLE 0.3f
#define MAX_OUT_JOINT_1_ANGLE 5.0f
//J1速度环PID参数
#define KP_JOINT_1_VELOCITY 0.0f
#define KI_JOINT_1_VELOCITY 0.0f
#define KD_JOINT_1_VELOCITY 0.0f
#define MAX_IOUT_JOINT_1_VELOCITY 0.0f
#define MAX_OUT_JOINT_1_VELOCITY 0.0f

//J2角度环PID参数
#define KP_JOINT_2_ANGLE 10.0f
#define KI_JOINT_2_ANGLE 0.1f
#define KD_JOINT_2_ANGLE 0.0f
#define MAX_IOUT_JOINT_2_ANGLE 0.3f
#define MAX_OUT_JOINT_2_ANGLE 5.0f
//J2速度环PID参数
#define KP_JOINT_2_VELOCITY 0.0f
#define KI_JOINT_2_VELOCITY 0.0f
#define KD_JOINT_2_VELOCITY 0.0f
#define MAX_IOUT_JOINT_2_VELOCITY 0.0f
#define MAX_OUT_JOINT_2_VELOCITY 0.0f

//J3角度环PID参数
#define KP_JOINT_3_ANGLE 10.0f
#define KI_JOINT_3_ANGLE 0.1f
#define KD_JOINT_3_ANGLE 0.0f
#define MAX_IOUT_JOINT_3_ANGLE 0.3f
#define MAX_OUT_JOINT_3_ANGLE 10.0f
//J3速度环PID参数
#define KP_JOINT_3_VELOCITY 4000.0f
#define KI_JOINT_3_VELOCITY 0.5f
#define KD_JOINT_3_VELOCITY 10.0f
#define MAX_IOUT_JOINT_3_VELOCITY 1000.0f
#define MAX_OUT_JOINT_3_VELOCITY 30000.0f

//J4角度环PID参数
#define KP_JOINT_4_ANGLE 3.0f
#define KI_JOINT_4_ANGLE 0.0f
#define KD_JOINT_4_ANGLE 0.0f
#define MAX_IOUT_JOINT_4_ANGLE 0.0f
#define MAX_OUT_JOINT_4_ANGLE 10.0f
//J4速度环PID参数
#define KP_JOINT_4_VELOCITY 1000.0f
#define KI_JOINT_4_VELOCITY 0.0f
#define KD_JOINT_4_VELOCITY 10.0f
#define MAX_IOUT_JOINT_4_VELOCITY 0.0f
#define MAX_OUT_JOINT_4_VELOCITY 1200.0f //10000.0f

//J5角度环PID参数
#define KP_JOINT_5_ANGLE 3.0f
#define KI_JOINT_5_ANGLE 0.0f
#define KD_JOINT_5_ANGLE 0.0f
#define MAX_IOUT_JOINT_5_ANGLE 0.0f
#define MAX_OUT_JOINT_5_ANGLE 10.0f
//J5速度环PID参数
#define KP_JOINT_5_VELOCITY 1000.0f
#define KI_JOINT_5_VELOCITY 0.0f
#define KD_JOINT_5_VELOCITY 10.0f
#define MAX_IOUT_JOINT_5_VELOCITY 0.0f
#define MAX_OUT_JOINT_5_VELOCITY 1200.0f //10000.0f

// Init parameters ---------------------
// Other parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
