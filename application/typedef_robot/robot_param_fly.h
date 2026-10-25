/**
 * @file       robot_param_fly.h
 * @brief      这里是无人机机器人参数配置文件，包括物理参数、PID参数等
 */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

//快速选择对应的板子
#define BOARD_CURRENT C_BOARD_DEFAULT
#define BOARD_OTHER C_BOARD_DEFAULT
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT             // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                   // 选择发射机构类型

/*------------------- BOARD -------------------*/
#define BOARD_CAN (2)
#define BOARD_DATA_ID (0)

/*-------------------- IMU --------------------*/
// IMU安装角度参数 (单位：度) yaw -> pitvh -> roll
#define IMU_ROLL_ANGLE (0.0f)    // 绕X轴旋转角度
#define IMU_PITCH_ANGLE (180.0f)   // 绕Y轴旋转角度  180
#define IMU_YAW_ANGLE (90.0f)    // 绕Z轴旋转角度 -90

/*-------------------- Gimbal --------------------*/
// 云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF) //电压控制1-4(0x1FF)5-7(0x2FF)

// gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)201

// remote controller sensitivity ---------------------
#define RC_TO_VECTOR_SCALE (0.006f)
#define REMOTE_CONTROLLER_SENSITIVITY_PITCH (-10000.0f)
#define REMOTE_CONTROLLER_SENSITIVITY_YAW   (-10000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (0.005f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-0.005f)
// motor parameters ---------------------
// 电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_ID  ((uint8_t)1)

// 电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE    ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE  ((MotorType_e)DJI_M6020)

// 旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (-1)
#define GIMBAL_DIRECT_PITCH_DIRECTION  (1)

// 减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO  (1)

// 电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE  (0)

// physical parameters ---------------------
#define GIMBAL_YAW_LIMIT (1) // 云台yaw轴是否有限位 1 - 有限位，0 - 无限位

#define GIMBAL_UPPER_LIMIT_PITCH (-0.6f) // 云台上限pitch电机的角度
#define GIMBAL_LOWER_LIMIT_PITCH (-1.4f) // 云台下限pitch电机的角度
#define GIMBAL_LOWER_LIMIT_YAW (-M_PI_4) // 云台下限yaw电机的角度
#define GIMBAL_UPPER_LIMIT_YAW (M_PI_4) // 云台上限yaw电机的角度

// 电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID  (-1.0592f) // 云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (-0.1181f)   // 云台初始化正对齐的时候使用的yaw轴正中心量

// PID parameters ---------------------
// YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (15.0f)
#define KI_GIMBAL_YAW_ANGLE (0.0f)
#define KD_GIMBAL_YAW_ANGLE (0.001f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (27.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
// VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (2000.0f)
#define KI_GIMBAL_YAW_VELOCITY (0.0000f)
#define KD_GIMBAL_YAW_VELOCITY (0.0000f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (10000.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (1300.0f)

// PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (20.5f)
#define KI_GIMBAL_PITCH_ANGLE (0.07f)
#define KD_GIMBAL_PITCH_ANGLE (0.3f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (9.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (50.0f)
// VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (1000.0f)
#define KI_GIMBAL_PITCH_VELOCITY (0.0f)
#define KD_GIMBAL_PITCH_VELOCITY (0.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (20000.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (2000.0f)

/*-------------------- Shoot --------------------*/
// physical parameters ---------------------
#define FRIC_RADIUS 0.03f // (m)摩擦轮半径
#define ECD_RANGE 8192      // 电机反馈码盘值范围
#define BULLET_NUM 8      // 定义拨弹盘容纳弹丸个数
#define TRIGGER_REDUCTION_RATIO 58/42   // 定义电机到拨弹盘的齿轮减速比
#define dianji_Transmission_ratio  36.0f //电机的传动比
#define all_Transmission_ratio dianji_Transmission_ratio*TRIGGER_REDUCTION_RATIO //电机到拨弹盘的总传动比
#define error1 (all_Transmission_ratio-all_Transmission_ratio_z)/all_Transmission_ratio_z*ECD_RANGE//电机转一圈产生的误差
#define GUN_NUM 1         // 定义枪管个数

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1  // 射击发射开关通道数据

/*MOTOR paramters --------------------*/

// 电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

// 电机ID
#define TRIGGER_MOTOR_ID 4
#define FRIC_MOTOR_R_ID 2
#define FRIC_MOTOR_L_ID 1
#define FRIC_MOTOR_U_ID 3

// 电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2
#define FRIC_MOTOR_U_CAN 1

// 电机std_id
#define STD_ID 0x200
// 单环拨弹速度
#define TRIGGER_SPEED (300.0f)
// 摩擦轮速度
#define FRIC_R_SPEED (330.0f)//660
#define FRIC_L_SPEED (-330.0f)
#define FRIC_U_SPEED (-330.0f)
#define FRIC_SPEED_LIMIT (150.0f)//600

/*ECD parameters------------*/
// 电机反馈码盘值范围
#define HALF_ECD_RANGE 4096

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
#define TRIGGER_SPEED_PID_KP (100.0f)//100
#define TRIGGER_SPEED_PID_KI (0.0f)
#define TRIGGER_SPEED_PID_KD (0.0f)//0.1

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

// 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (25.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (30.0f)

// 摩擦轮电机PID
#define FRIC_SPEED_PID_KP (666.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (0.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

// 当未连接裁判系统设置负值方便调试
#define SHOOT_HEAT_REMAIN_VALUE -80 // 89

#endif /* INCLUDED_ROBOT_PARAM_H */
