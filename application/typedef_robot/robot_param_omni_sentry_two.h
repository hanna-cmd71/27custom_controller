/**
 * @file       robot_param_omni_sentry_two.h
 * @brief      这里是全向轮双yaw哨兵机器人参数配置文件，包括物理参数、PID参数等
 */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

//快速选择对应的板子
// #define BOARD_CURRENT C_BOARD_DEFAULT
// #define BOARD_CURRENT C_BOARD_DOWN
#define BOARD_CURRENT C_BOARD_UP

// 快速设置双板的配置
#if(BOARD_CURRENT == C_BOARD_DOWN)
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL                 // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                         // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                           // 选择发射机构类型
#define BOARD_OTHER C_BOARD_UP                          // 选择其他板子类型

#elif(BOARD_CURRENT == C_BOARD_UP)
#define CHASSIS_TYPE CHASSIS_NONE                       // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_DOUBLE_YAW_PITCH             // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                   // 选择发射机构类型
#define BOARD_OTHER C_BOARD_DOWN                        // 选择其他板子类型

#elif(BOARD_CURRENT == C_BOARD_DEFAULT)
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL                 // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_DOUBLE_YAW_PITCH             // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                   // 选择发射机构类型

#endif

/*------------------- BOARD -------------------*/
#define BOARD_CAN (1)
#define BOARD_DATA_ID (0)

/*-------------------- IMU --------------------*/
// IMU安装角度参数 (单位：度) yaw -> pitvh -> roll
#define IMU_ROLL_ANGLE (0.0f)     // 绕X轴旋转角度
#define IMU_PITCH_ANGLE (0.0f)    // 绕Y轴旋转角度 
#define IMU_YAW_ANGLE (0.0f)      // 绕Z轴旋转角度

/*-------------------- Chassis --------------------*/

// remote controller sensitivity ---------------------
#define RC_TO_VECTOR_SCALE (0.006f)

// physical parameters ---------------------
#define WHEEL_RADIUS (0.15f)           //(m)轮子直径
#define WHEEL_CENTER_DISTANCE (0.245f) //(m)轮子到车的距离（0.22 + 0.21）
#define yaw_mid_date -2.2f

// motor parameters ---------------------
// 底盘电流发送参数
#define CHASSIS_CAN (1)
#define CHASSIS_STDID (0x200)

// 电机ID ---------------------
#define WHEEL_1_ID (3)
#define WHEEL_2_ID (4)
#define WHEEL_3_ID (2)
#define WHEEL_4_ID (1)

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
#define KP_OMNI_VEL (150.0f)
#define KI_OMNI_VEL (0.0f)
#define KD_OMNI_VEL (0.3f)
#define MAX_IOUT_OMNI_VEL (0.0f)
#define MAX_OUT_OMNI_VEL (7000.0f)

// 云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.01f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.5f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (3.0f)

// RC parametes ---------------------
// 遥控器相关参数 
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   // 遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (1.0f)     // 最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (5.0f)  // 最大角速度(rad/s) 仅用于无云台模式

/*-------------------- Gimbal --------------------*/
// 云台电流发送参数
#define GIMBAL_YAW_BA_CAN (1)
#define GIMBAL_YAW_UP_CAN (2)
#define GIMBAL_PITCH_CAN  (2)
#define GIMBAL_YAW_CAN GIMBAL_YAW_BA_CAN
#define GIMBAL_STDID (0x1FF)            //电压控制1-4(0x1FF)5-7(0x2FF)

// gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)201

// remote controller sensitivity ---------------------
#define RC_TO_VECTOR_SCALE (0.006f)
#define REMOTE_CONTROLLER_SENSITIVITY_YAW (500.0f)   // 云台遥控器灵敏度
#define REMOTE_CONTROLLER_SENSITIVITY_PITCH (300.0f) // 云台遥控器灵敏度
#define REMOTE_CONTROLLER_MAX_DEADLINE (0.05f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-0.05f)
// motor parameters ---------------------
// 电机id
#define GIMBAL_DIRECT_YAW_ID    ((uint8_t)1)
#define GIMBAL_DIRECT_YAW_BA_ID ((uint8_t)1)
#define GIMBAL_DIRECT_YAW_UP_ID ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_ID  ((uint8_t)3)

// 电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE    ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_YAW_BA_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_YAW_UP_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE  ((MotorType_e)DJI_M6020)

// 旋转方向
#define GIMBAL_DIRECT_YAW_BA_DIRECTION (1)
#define GIMBAL_DIRECT_YAW_UP_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION  (-1)
#define yaw_up_zf -1

// 减速比
#define GIMBAL_DIRECT_YAW_BA_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_YAW_UP_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO  (1)

// 电机运行模式
#define GIMBAL_DIRECT_YAW_BA_MODE (0)
#define GIMBAL_DIRECT_YAW_UP_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE  (0)

// physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH  (0.72f)
#define GIMBAL_LOWER_LIMIT_PITCH  (-0.27f)
#define GIMBAL_LOWER_LIMIT_YAW_BA (-M_PI)
#define GIMBAL_UPPER_LIMIT_YAW_BA (M_PI)
#define GIMBAL_LOWER_LIMIT_YAW_UP (1.2f)
#define GIMBAL_UPPER_LIMIT_YAW_UP (2.59f)
#define GIMBAL_LOWER_LIMIT_YAW    (-M_PI)     // (GIMBAL_LOWER_LIMIT_YAW_BA + GIMBAL_LOWER_LIMIT_YAW_UP) 
#define GIMBAL_UPPER_LIMIT_YAW    (M_PI)      // (GIMBAL_UPPER_LIMIT_YAW_BA + GIMBAL_UPPER_LIMIT_YAW_UP)

// 电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID  (0.0f)       // 云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_BA_MID (-2.7626f)   // 云台初始化正对齐的时候使用的yaw轴正中心量
#define GIMBAL_DIRECT_YAW_UP_MID (-0.3f)      // 云台初始化正对齐的时候使用的yaw轴正中心量
#define yaw_up_mid 1.8                          //yaw电机运行时的中值

// PID parameters ---------------------
// YAW BA ANGLE
#define KP_GIMBAL_YAW_BA_ANGLE (200.0f)
#define KI_GIMBAL_YAW_BA_ANGLE (0.05f)
#define KD_GIMBAL_YAW_BA_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_BA_ANGLE  (40.0f)
#define MAX_IOUT_GIMBAL_YAW_BA_ANGLE (0.1f)
// VELOCITY:角速度
#define KP_GIMBAL_YAW_BA_VELOCITY (150.0f)
#define KI_GIMBAL_YAW_BA_VELOCITY (0.0f)
#define KD_GIMBAL_YAW_BA_VELOCITY (0.0f)
#define MAX_OUT_GIMBAL_YAW_BA_VELOCITY (10000.0f)
#define MAX_IOUT_GIMBAL_YAW_BA_VELOCITY (0.0f)

// YAW UP ANGLE
#define KP_GIMBAL_YAW_UP_ANGLE (500.0f)
#define KI_GIMBAL_YAW_UP_ANGLE (1.0f)
#define KD_GIMBAL_YAW_UP_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_UP_ANGLE  (45.0f)
#define MAX_IOUT_GIMBAL_YAW_UP_ANGLE (0.0f)
// VELOCITY:角速度
#define KP_GIMBAL_YAW_UP_VELOCITY (150.0f)
#define KI_GIMBAL_YAW_UP_VELOCITY (0.0f)
#define KD_GIMBAL_YAW_UP_VELOCITY (0.0f)
#define MAX_OUT_GIMBAL_YAW_UP_VELOCITY  (10000.0f)
#define MAX_IOUT_GIMBAL_YAW_UP_VELOCITY (0.0f)

// PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (50.0f)
#define KI_GIMBAL_PITCH_ANGLE (0.0f)
#define KD_GIMBAL_PITCH_ANGLE (0.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE  (4.0f)
// VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (5000.0f)
#define KI_GIMBAL_PITCH_VELOCITY (25.0f)
#define KD_GIMBAL_PITCH_VELOCITY (0.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (2500.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY  (9000.0f)

/*-------------------- Shoot --------------------*/
// physical parameters ---------------------
#define FRIC_RADIUS 0.03f                    // (m)摩擦轮半径
#define ECD_RANGE 8191                       // 电机反馈码盘值范围
#define BULLET_NUM 8                         // 定义拨弹盘容纳弹丸个数
#define TRIGGER_REDUCTION_RATIO 1        // 定义电机到拨弹盘的齿轮减速比
#define dianji_Transmission_ratio  36.0f     //电机的传动比
#define all_Transmission_ratio dianji_Transmission_ratio*TRIGGER_REDUCTION_RATIO //电机到拨弹盘的总传动比
#define error1 (all_Transmission_ratio-SHOOT.trigger_error.all_Transmission_ratio_z)/SHOOT.trigger_error.all_Transmission_ratio_z*ECD_RANGE//电机转一圈产生的误差

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1                 // 射击发射开关通道数据

/*MOTOR paramters --------------------*/

// 电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE    ((MotorType_e)DJI_M3508)

// 电机ID
#define TRIGGER_MOTOR_ID 8
#define FRIC_MOTOR_R_ID  1
#define FRIC_MOTOR_L_ID  2
#define FRIC_MOTOR_U_ID  9

// 电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN  2
#define FRIC_MOTOR_L_CAN  2
#define FRIC_MOTOR_U_CAN  2

// 旋转方向
#define TRIGGER_MOTOR_DIRECTION (1)
#define FRIC_MOTOR_R_DIRECTION  (1)
#define FRIC_MOTOR_L_DIRECTION  (-1)
#define FRIC_MOTOR_U_DIRECTION  (1)

// 电机std_id
#define STD_ID 0x200
// 单环拨弹速度
#define TRIGGER_SPEED (-300.0f)
// 摩擦轮速度
#define FRIC_R_SPEED (-550.0f)    //660
#define FRIC_L_SPEED (-550.0f)
#define FRIC_U_SPEED (0.0f)
#define FRIC_SPEED_LIMIT (150.0f) //600

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
#define REVERSE_SPEED (300.0f)

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f) //  (none)

/*PID parameters ---------------------*/

// 拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (100.0f)//100
#define TRIGGER_SPEED_PID_KI (0.1f)
#define TRIGGER_SPEED_PID_KD (0.0f)//0.1

#define TRIGGER_SPEED_PID_MAX_OUT (4000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (400.0f)

// 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (500.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (15.0f)

// 摩擦轮电机PID
#define FRIC_SPEED_PID_KP (100.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (0.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

// 当未连接裁判系统设置负值方便调试
#define SHOOT_HEAT_REMAIN_VALUE -80 // 89

#endif /* INCLUDED_ROBOT_PARAM_H */
