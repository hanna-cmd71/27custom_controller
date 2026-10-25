/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)
#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_communication.h"
#include "math.h"
#include "usb_debug.h"
#include "supervisory_computer_cmd.h"
#include "user_lib.h"
#include "arm_math.h"
#include "referee.h"
#include "detect_task.h"


typedef enum 
{
    LOAD_STOP,      // 停止拨盘
    LAOD_BULLET,    // 单发模式
    LOAD_BURSTFIRE,  // 连发模式,对速度闭环
    LOAD_BLOCK       // 堵转，模式
} LoadMode_e;

typedef enum 
{
    FRIC_NOT_READY,      // 未准备发射
    FRIC_READY,          // 准备发射
} FricState_e;

typedef struct feedback
{
  fp32 trigger_angel_fdb;// 拨弹盘输出轴位置
  fp32 trigger_speed_fdb;// 拨弹盘输出轴速度
  fp32 fric_speed_fdb_L;   // 摩擦轮输出轴速度
  fp32 fric_speed_fdb_R;
  fp32 fric_speed_fdb_U;
} Fdb;

typedef struct reference
{
  fp32 trigger_angel_ref;// 拨弹盘位置期望
  fp32 trigger_speed_ref;// 拨弹盘速度期望
  fp32 fric_speed_ref_L;   // 摩擦轮速度期望
  fp32 fric_speed_ref_R;
  fp32 fric_speed_ref_U;
} Ref;

typedef struct trigger_error
{
  fp32 delta;
  int date;
  fp32 all_error;               // 总误差
  int all_Transmission_ratio_z; // 电机到拨弹盘的总传动比的整数
  int COUNT;                    // 拨弹盘转半圈所需的电机圈数的整数
  int COUNT_error;              // 误差
} error;

typedef struct
{

  const RC_ctrl_t * rc;  // 射击使用的遥控器指针
  LoadMode_e mode;       // 射击模式
  FricState_e state;     // 摩擦轮状态

  Motor_s fric_motor[3];  // 摩擦轮电机
  Motor_s trigger_motor;  // 拨弹盘电机
  error trigger_error;

    //pid
  pid_type_def trigger_angel_pid;
  pid_type_def trigger_speed_pid;
  pid_type_def fric_pid[3];

    //block_reverse
  uint16_t reverse_time;
  uint16_t block_time;
  fp32 last_trigger_vel;
  fp32 last_fric_vel;
    
    //feedback
  Fdb FDB;
  
    //reference
  Ref REF;

    //flag
  uint16_t fric_flag; //    摩擦轮状态
  uint16_t move_flag; //    拨弹盘角度状态，用于判断单发射击执行情况
  uint16_t shoot_flag;//    鼠标左键状态，用于判断弹发射击启动

   //ecd
  int16_t last_ecd; //     上一个ecd
  int16_t ecd_count;//     ecd计数

  // heat
  uint16_t heat_limit;
  uint16_t heat;
  uint16_t mr_time;
} Shoot_s;



extern void ShootInit(void);

extern void ShootSetMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void ShootSendCmd(void);

#endif  // SHOOT_FRIC_H
#endif  // SHOOT_TYPE == SHOOT_FRIC

