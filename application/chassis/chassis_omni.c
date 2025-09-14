/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.03.03       Harry_Wong        1.重新构建全向轮底盘，完成单底盘控制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h"
#include "detect_task.h"
#include "gimbal.h"
#include "usb_debug.h"
#include "communication.h"

#include <math.h>
#include <string.h>

Chassis_s chassis;
Chassis_PID_t chassis_pid;
static const PidGetVofa_t *pid_get_vofa;
fp32 chassis_wz;
fp32 a1;
fp32 a2;

/*-------------------- Publish --------------------*/

/**
 * @brief          发布底盘状态
 * @param[in]      none
 * @retval         none
 */
void ChassisPublish(void)
{
    Publish(&chassis_pid, CHASSIS_PID_NAME);
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    pid_get_vofa = Subscribe(PID_GET_VOFA_NAME);
    // 获取遥控器指针
    chassis.rc = get_remote_control_point();

    // step2 PID数据清零，设置PID参数
    const static fp32 wheel_vel[3] = {KP_OMNI_VEL, KI_OMNI_VEL, KD_OMNI_VEL};
    for (int i = 0; i < 4; ++i)
    {
        PID_init(&chassis_pid.wheel_velocity[i], PID_POSITION, wheel_vel, MAX_OUT_OMNI_VEL, MAX_IOUT_OMNI_VEL);
    }

    const static fp32 gimbal_follow[3] = {KP_CHASSIS_FOLLOW_GIMBAL, KI_CHASSIS_FOLLOW_GIMBAL, KD_CHASSIS_FOLLOW_GIMBAL};
    PID_init(&chassis_pid.follow, PID_POSITION, gimbal_follow, MAX_OUT_CHASSIS_FOLLOW_GIMBAL, MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    // step3 初始化电机
    MotorInit(&chassis.wheel[0], WHEEL_1_ID, WHEEL_1_CAN, WHEEL_1_MOTOR_TYPE, WHEEL_1_DIRECTION, WHEEL_1_RATIO, WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1], WHEEL_2_ID, WHEEL_2_CAN, WHEEL_2_MOTOR_TYPE, WHEEL_2_DIRECTION, WHEEL_2_RATIO, WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2], WHEEL_3_ID, WHEEL_3_CAN, WHEEL_3_MOTOR_TYPE, WHEEL_3_DIRECTION, WHEEL_3_RATIO, WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3], WHEEL_4_ID, WHEEL_4_CAN, WHEEL_4_MOTOR_TYPE, WHEEL_4_DIRECTION, WHEEL_4_RATIO, WHEEL_4_MODE);

    // step4 初始模式设置
    chassis.mode = CHASSIS_ZERO_FORCE;
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{

    for (int i = 0; i < 4; ++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }
    chassis.yaw_delta = GetGimbalDeltaYawMid();

    switch (__TUNING_MODE)
    {
    case TUNING_CHASSIS_WHEEL:
    {
        for (int i = 0; i < 4; ++i)
        {
            chassis_pid.wheel_velocity[i].Kp = pid_get_vofa->speed_kp;
            chassis_pid.wheel_velocity[i].Ki = pid_get_vofa->speed_ki;
            chassis_pid.wheel_velocity[i].Kd = pid_get_vofa->speed_kd;
            chassis_pid.wheel_velocity[i].max_iout = pid_get_vofa->speed_max_iout;
            chassis_pid.wheel_velocity[i].max_out = pid_get_vofa->speed_max_out;
        }
        break;
    }

    case TUNING_CHASSIS_FOLLOW:
    {
        chassis_pid.follow.Kp = pid_get_vofa->speed_kp;
        chassis_pid.follow.Ki = pid_get_vofa->speed_ki;
        chassis_pid.follow.Kd = pid_get_vofa->speed_kd;

        chassis_pid.follow.max_iout = pid_get_vofa->speed_max_iout;
        chassis_pid.follow.max_out = pid_get_vofa->speed_max_out;
        break;
    }
    default:
        break;
    }
}

/*-------------------- Reference --------------------*/

/**
 * @brief         . 更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{

    if (BOARD_CURRENT == C_BOARD_DEFAULT)
    {
        chassis.reference_rc.vx = GetDt7RcCh(0);
        chassis.reference_rc.vy = GetDt7RcCh(1);
        chassis.reference_rc.chassis_mode = GetDt7RcSw(0);
    }
    else
    {
        GetBoardInfo(&chassis.reference_rc);
    }

    switch (chassis.reference_rc.chassis_mode)
    {
    case CHASSIS_NOMOVE:
    {
        chassis.reference.vx = 0;
        chassis.reference.vy = 0;
        chassis.reference.wz = 0;
        break;
    }
    case CHASSIS_NO_FOLLOW:
    {
        float sin_yaw = sin(chassis.yaw_delta);
        float cos_yaw = cos(chassis.yaw_delta);
        chassis.reference.vx = (chassis.reference_rc.vx * cos_yaw - chassis.reference_rc.vy * sin_yaw) * CHASSIS_RC_MAX_SPEED;
        chassis.reference.vy = (chassis.reference_rc.vx * sin_yaw + chassis.reference_rc.vy * cos_yaw) * CHASSIS_RC_MAX_SPEED;
        a1=cos_yaw;
        a2=sin_yaw;
        // chassis_wz = PID_calc(&chassis_pid.follow, chassis.yaw_delta, 0);
        // // if(chassis_wz<1&&chassis_wz>-1)
        // // {
        // //     chassis_wz=0;
        // // }
        // chassis.reference.wz = 0.1f*chassis_wz;
        break;
    }
    case CHASSIS_FOLLOW_GIMBAL_YAW:
    {
        float sin_yaw = sin(chassis.yaw_delta);
        float cos_yaw = cos(chassis.yaw_delta);
        chassis.reference.vx = (-chassis.reference_rc.vx * cos_yaw - chassis.reference_rc.vy * sin_yaw) * CHASSIS_RC_MAX_SPEED;
        chassis.reference.vy = (-chassis.reference_rc.vx * sin_yaw + chassis.reference_rc.vy * cos_yaw) * CHASSIS_RC_MAX_SPEED;
        chassis.reference.wz = 0; // PID_calc(&chassis_pid.follow, chassis.yaw_delta, 0);
        break;
    }
    case CHASSIS_SPIN:
    {
        float sin_yaw = sin(chassis.yaw_delta);
        float cos_yaw = cos(chassis.yaw_delta);
        chassis.reference.vx = (chassis.reference_rc.vx * cos_yaw - chassis.reference_rc.vy * sin_yaw) * CHASSIS_RC_MAX_SPEED;
        chassis.reference.vy = (chassis.reference_rc.vx * sin_yaw + chassis.reference_rc.vy * cos_yaw) * CHASSIS_RC_MAX_SPEED;
        chassis.reference.wz = 1.0f * CHASSIS_RC_MAX_VELOCITY;
        break;
    }
    }

    // 离线保命
    if (toe_is_error(BOARD_COMMUNICATION_TOE))
    {
        chassis.reference.vx = 0;
        chassis.reference.vy = 0;
        chassis.reference.wz = 0;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    chassis.set[0] = (sqrt(2) * (-chassis.reference.vx - chassis.reference.vy) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[0].reduction_ratio;
    chassis.set[1] = (sqrt(2) * (-chassis.reference.vx + chassis.reference.vy) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[1].reduction_ratio;
    chassis.set[2] = (sqrt(2) * (chassis.reference.vx + chassis.reference.vy) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[2].reduction_ratio;
    chassis.set[3] = (sqrt(2) * (chassis.reference.vx - chassis.reference.vy) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[3].reduction_ratio;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity[i], chassis.feedback[i], chassis.set[i]);
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
    // 添加底盘电机到CAN管理器
    for (int i = 0; i < 4; ++i)
    {
        CanManagerAddMotor(chassis.wheel[i].id, chassis.wheel[i].can, chassis.wheel[i].set.curr);
        //    CanManagerAddMotor(chassis.wheel[i].id, chassis.wheel[i].can, 2000);

    }
}

/*------------------------------ Calibrate Function ------------------------------*/

/**
 * @brief          设置底盘校准值，发送ID为0x700的CAN包,设置3508电机进入快速设置ID模式
 * @param[in]      motor_middle:电机中值 (此处用作ID设置参数，但当前实现为通用快速ID设置)
 * @retval         返回空
 * @note           底盘任务内部调用的函数，用于电机ID快速设置
 */
void ChassisSetCaliData(const fp32 motor_middle[4])
{
    // 添加底盘电机到CAN管理器
    CanCmdDjiMotor(chassis.wheel[0].can, 0x700, 0, 0, 0, 0);// 发送ID为0x700的CAN包，设置3508电机进入快速设置ID模式
    CanCmdDjiMotor(chassis.wheel[1].can, 0x700, 0, 0, 0, 0);
    CanCmdDjiMotor(chassis.wheel[2].can, 0x700, 0, 0, 0, 0);
    CanCmdDjiMotor(chassis.wheel[3].can, 0x700, 0, 0, 0, 0);
}

/**
 * @brief          底盘校准计算，发送快速设置ID命令并等待完成
 * @param[out]     motor_middle:电机中值 (当前实现中用作状态返回)
 * @retval         返回1 代表ID设置命令发送完毕， 返回0 代表正在设置中
 * @note           底盘任务内部调用的函数，用于管理ID设置流程
 */
bool_t ChassisCmdCali(fp32 motor_middle[4])
{
    CanCmdDjiMotor(1, 0x700, 0, 0, 0, 0); // 发送ID为0x700的CAN包，设置3508电机进入快速设置ID模式
    CanCmdDjiMotor(1, 0x700, 0, 0, 0, 0);
    CanCmdDjiMotor(1, 0x700, 0, 0, 0, 0);
    return 1;
}

#endif
