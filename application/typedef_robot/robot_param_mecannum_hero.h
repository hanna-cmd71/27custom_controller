/**
 * @file       robot_param_omni_sentry_two.h
 * @brief      这里是麦克纳姆轮英雄机器人参数配置文件，包括物理参数、PID参数等
 */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define GIMBAL_LOWER_LIMIT_YAW (-M_PI)
#define GIMBAL_UPPER_LIMIT_YAW (M_PI)





//快速选择对应的板子
// #define BOARD_CURRENT C_BOARD_DEFAULT
// #define BOARD_CURRENT C_BOARD_DOWN
#define BOARD_CURRENT C_BOARD_UP

// 快速设置双板的配置
#if(BOARD_CURRENT == C_BOARD_DOWN)
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL              // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                         // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                           // 选择发射机构类型
#define BOARD_OTHER C_BOARD_UP                          // 选择其他板子类型

#elif(BOARD_CURRENT == C_BOARD_UP)
#define CHASSIS_TYPE CHASSIS_NONE                       // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT             // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                   // 选择发射机构类型
#define BOARD_OTHER C_BOARD_DOWN                        // 选择其他板子类型

#elif(BOARD_CURRENT == C_BOARD_DEFAULT)
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL                 // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT             // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                   // 选择发射机构类型

#endif

/*------------------- BOARD -------------------*/
#define BOARD_CAN (2)
#define BOARD_DATA_ID (0)

/*-------------------- IMU --------------------*/
// IMU安装角度参数 (单位：度)
#define IMU_ROLL_ANGLE (0.0f)    // 绕X轴旋转角度
#define IMU_PITCH_ANGLE (0.0f)   // 绕Y轴旋转角度 
#define IMU_YAW_ANGLE (90.0f)    // 绕Z轴旋转角度

/*-------------------- Chassis --------------------*/

// remote controller sensitivity ---------------------
#define RC_TO_VECTOR_SCALE (0.006f)

// physical parameters ---------------------
#define WHEEL_RADIUS (0.15f)           //(m)轮子直径
#define WHEEL_CENTER_DISTANCE (0.245f) //(m)轮子到车的距离（0.22 + 0.21）

// motor parameters ---------------------
// 底盘电流发送参数
#define CHASSIS_CAN (2)
#define CHASSIS_STDID (0x200)
   
// 电机ID ---------------------
#define WHEEL_1_ID (1)
#define WHEEL_2_ID (4)
#define WHEEL_3_ID (3)
#define WHEEL_4_ID (2)

// 电机CAN ---------------------
#define WHEEL_1_CAN (2)
#define WHEEL_2_CAN (2)
#define WHEEL_3_CAN (2)
#define WHEEL_4_CAN (2)

// 电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

// 电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

// 电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

// 电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

// PID parameters ---------------------
// 驱动轮速度环PID参数
#define KP_OMNI_VEL (20.0f)
#define KI_OMNI_VEL (0.3f)
#define KD_OMNI_VEL (0.3f)
#define MAX_IOUT_OMNI_VEL (10000.0f)
#define MAX_OUT_OMNI_VEL (30000.0f)

// 云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.01f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.5f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (3.0f)

// RC parametes ---------------------E
// 遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)     // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)  // 遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (1.0f)    // 最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f) // 最大角速度(rad/s) 仅用于无云台模式


/*-------------------- Gimbal --------------------*/
// 云台电流发送参数
#define GIMBAL_CAN (1)
#define GIMBAL_STDID (0x1FE) //电压控制1-4(0x1FF)5-7(0x2FF)

// gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)201

// remote controller sensitivity ---------------------
#define RC_TO_VECTOR_SCALE (0.006f)
#define REMOTE_CONTROLLER_SENSITIVITY_YAW (1000.0f)
#define REMOTE_CONTROLLER_SENSITIVITY_PITCH (-300.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (0.015f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-0.015f)
// motor parameters ---------------------
// 电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

// 电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

// 旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

// 减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

// 电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

// physical parameters ---------------------
#define GIMBAL_YAW_LIMIT (0) // 云台yaw轴是否有限位 1 - 有限位，0 - 无限位

#define GIMBAL_UPPER_LIMIT_PITCH (1.5f) // 云台上限pitch电机的角度
#define GIMBAL_LOWER_LIMIT_PITCH (-1.6f) // 云台下限pitch电机的角度
#define GIMBAL_LOWER_LIMIT_YAW (-M_PI) // 云台下限yaw电机的角度
#define GIMBAL_UPPER_LIMIT_YAW (M_PI) // 云台上限yaw电机的角度

// 电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (-0.6741f) // 云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (-1.0754f)   // 云台初始化正对齐的时候使用的yaw轴正中心量

// PID parameters ---------------------
// YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (20.0f)
#define KI_GIMBAL_YAW_ANGLE (3.0f)
#define KD_GIMBAL_YAW_ANGLE (2.5197f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (27.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
// VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (500.0f)
#define KI_GIMBAL_YAW_VELOCITY (12.8047f)
#define KD_GIMBAL_YAW_VELOCITY (1.0557f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (15000.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (1300.0f)

// PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (5.5f)
#define KI_GIMBAL_PITCH_ANGLE (0.07f)
#define KD_GIMBAL_PITCH_ANGLE (0.3f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (9.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (50.0f)
// VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (300.0f)
#define KI_GIMBAL_PITCH_VELOCITY (0.3f)
#define KD_GIMBAL_PITCH_VELOCITY (0.5f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (2000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (15000.0f)

/*-------------------- Shoot --------------------*/
// physical parameters ---------------------
#define FRIC_RADIUS 0.03f // (m)摩擦轮半径
#define BULLET_NUM 5      // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1         // 定义枪管个数
#define TRIGGER_REDUCTION_RATIO 1.0f  // 定义电机到拨弹盘的齿轮减速比
#define all_Transmission_ratio dianji_Transmission_ratio*TRIGGER_REDUCTION_RATIO //电机到拨弹盘的总传动比
#define error1 (all_Transmission_ratio-SHOOT.trigger_error.all_Transmission_ratio_z)/SHOOT.trigger_error.all_Transmission_ratio_z*ECD_RANGE//电机转一圈产生的误差
#define dianji_Transmission_ratio  19.0f //电机的传动比

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1  // 射击发射开关通道数据


/*MOTOR paramters --------------------*/

// 电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DM_4310)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

// 电机ID
#define TRIGGER_MOTOR_ID 4
#define FRIC_MOTOR_R_ID 3
#define FRIC_MOTOR_L_ID 2
#define FRIC_MOTOR_U_ID 1

// 电机can口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 1
#define FRIC_MOTOR_L_CAN 1
#define FRIC_MOTOR_U_CAN 1

// 旋转方向
#define TRIGGER_MOTOR_DIRECTION (1)
#define FRIC_MOTOR_R_DIRECTION  (-1)
#define FRIC_MOTOR_L_DIRECTION  (1)
#define FRIC_MOTOR_U_DIRECTION  (-1)

// 电机std_id
#define STD_ID 0x200
// 单环拨弹速度
#define TRIGGER_SPEED (300.0f)
// 摩擦轮速度
#define FRIC_R_SPEED (330.0f)//660
#define FRIC_L_SPEED (330.0f)
#define FRIC_U_SPEED (330.0f)
#define FRIC_SPEED_LIMIT (150.0f)//600

/*ECD parameters------------*/
// 电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

// 电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

/*BLOCK&REVERSE parameters------------*/

// 初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 5.0f
#define BLOCK_TIME 1000
#define REVERSE_TIME 1250
#define REVERSE_SPEED (-20.0f)

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f) //  (none)

/*PID parameters ---------------------*/

// 拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (200.0f)//100
#define TRIGGER_SPEED_PID_KI (0.0f)
#define TRIGGER_SPEED_PID_KD (0.0f)//0.1

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (7000.0f)

// 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (6.0f)
#define TRIGGER_ANGEL_PID_KI (0.0f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (10000.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (7000.0f)

// 摩擦轮电机PID
#define FRIC_SPEED_PID_KP (66.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (0.0f)

#define FRIC_PID_MAX_OUT (15000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

// 当未连接裁判系统设置负值方便调试
#define SHOOT_HEAT_REMAIN_VALUE -80 // 89

#endif /* INCLUDED_ROBOT_PARAM_H */
