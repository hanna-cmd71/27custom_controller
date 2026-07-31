/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       controler_myself.c/h
  * @brief      自定义云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef CONTROLER_MYSELF_H
#define CONTROLER_MYSELF_H

#include "robot_param.h"
#include "IMU.h"
#include "gimbal.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "usb_debug.h"
#include "cmsis_os.h"
#include "math.h"
#include "macro_typedef.h"
#include "supervisory_computer_cmd.h"

/**
 * @brief 状态、期望和限制值
 */
#define CONTROLLER_MYSELF_JOINT_NUM 6U

/* SB大头反馈没有速度字段，速度由相邻两次编码器位置估算。 */
#ifndef CONTROLLER_MYSELF_JOINT_KP
#define CONTROLLER_MYSELF_JOINT_KP 120.0f
#endif
#ifndef CONTROLLER_MYSELF_JOINT_KD
#define CONTROLLER_MYSELF_JOINT_KD 8.0f
#endif
#ifndef CONTROLLER_MYSELF_JOINT_TORQUE_MAX
#define CONTROLLER_MYSELF_JOINT_TORQUE_MAX 8000
#endif
#ifndef CONTROLLER_MYSELF_JOINT_TARGET_STEP
#define CONTROLLER_MYSELF_JOINT_TARGET_STEP 0.002f
#endif

typedef struct
{
    float pitch;
    float yaw;
} Values_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def pitch_angle;
    pid_type_def pitch_velocity;

} Gimbal_PID_t;

typedef struct
{
    const RC_ctrl_t * rc;
    BoardMode_e mode, last_mode, mode_before_rc_err;

    /*-------------------- Motors --------------------*/
    Motor_s yaw, pitch;
    /* 六轴SB大头关节控制数据，索引0~5对应扩展ID 0x100~0x600。 */
    float joint_reference[CONTROLLER_MYSELF_JOINT_NUM];
    float joint_feedback[CONTROLLER_MYSELF_JOINT_NUM];
    float joint_velocity[CONTROLLER_MYSELF_JOINT_NUM];
    int32_t joint_torque[CONTROLLER_MYSELF_JOINT_NUM];
    uint8_t joint_direction[CONTROLLER_MYSELF_JOINT_NUM];
    uint32_t joint_last_feedback_time[CONTROLLER_MYSELF_JOINT_NUM];
    uint16_t joint_last_ecd[CONTROLLER_MYSELF_JOINT_NUM];
    bool joint_feedback_valid[CONTROLLER_MYSELF_JOINT_NUM];
    /*-------------------- Values --------------------*/
    Values_t reference;
    Values_t feedback_pos, feedback_vel;
    Values_t upper_limit;
    Values_t lower_limit;

    Gimbal_PID_t pid;

    float pitch_angle_zero_for_imu;
    float yaw_angle_zero_for_imu;

    uint32_t init_start_time, init_timer;

    bool init_continue;
} Gimbal_s;

/*-------------------- Global var --------------------*/

extern Gimbal_s controller_myself;
extern Gimbal_PID_t controller_myself_pid;

/*-------------------- Public --------------------*/

extern void GimbalPublish(void);
extern void GimbalInit(void);
extern void GimbalSetMode(void);
extern void GimbalObserver(void);
extern void GimbalReference(void);
extern void GimbalConsole(void);
extern void GimbalSendCmd(void);

#endif  // CONTROLER_MYSELF_H

/************************ END OF FILE ************************/
