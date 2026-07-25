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
#if (CHASSIS_TYPE == CHASSIS_FOOT_WHEEL)
#include "chassis_foot.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h"
#include "detect_task.h"
#include "gimbal.h"
#include "usb_debug.h"
#include "communication.h"
// #include "gimbal_double_yaw_pitch.h"
#include "IMU.h"//陀螺仪文件

#include <math.h>

Chassis_s chassis;
Chassis_PID_t chassis_pid;
static const PidGetVofa_t *pid_get_vofa;
fp32 chassis_wz;
fp32 chassis_angle_get;
fp32 chassis_angle_set;
pid_type_def angle_pid[2];
fp32 angle_now[50];
int angle_index=0;
fp32 angle_all=0;
fp32 angle_average;
fp32 angle_data;
first_order_filter_type_t vel_get[2];
first_order_filter_type_t chassis_angle_input;
first_order_filter_type_t chassis_angle_output;
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
fp32 chassis_get(int i)
{
    switch (i) 
    {
        case 0:
        {
        return chassis_angle_set;
        }
        case 1:
        {
        return chassis_angle_get;
        }
        case 2:
        {
        return angle_pid[0].out;
        }
        case 3:
        {
        return chassis_pid.wheel_velocity->set;
        }
          case 4:
        {
        return chassis_pid.wheel_velocity->fdb;
        } 
         case 5:
        {
        return chassis_pid.wheel_velocity->out;
        } 
         case 6:
        {
        return   angle_pid[0].Dout;
        } 
        case 7:
        {
        return   chassis_pid.wheel_velocity->Iout;
        }
    }
return 0.0f;  
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
    const static fp32 gimbal_angle[3] = {KP_ANGLE, KI_ANGLE, KD_ANGLE};

    PID_init(&chassis_pid.follow, PID_POSITION, gimbal_follow, MAX_OUT_CHASSIS_FOLLOW_GIMBAL, MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);
    PID_init(&angle_pid[0], PID_POSITION, gimbal_angle, MAX_OUT_ANGLE, MAX_IOUT_ANGLE);//chassis_
    PID_init(&angle_pid[1], PID_POSITION, gimbal_angle, MAX_OUT_ANGLE, MAX_IOUT_ANGLE);//chassis_

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


// angle_data=-0.00f*(-chassis.feedback[0]+chassis.feedback[1]);
angle_data=0.3f*(chassis_angle_set-chassis_angle_get);

    chassis_angle_set=0.005f+angle_data;
    chassis_angle_get=GetImuAngle(AX_ROLL);

        first_order_filter_cali(&chassis_angle_input,chassis_angle_set,1,5);   
    chassis_angle_set=chassis_angle_input.out;
        first_order_filter_cali(&chassis_angle_output,chassis_angle_get,1,5);   
    chassis_angle_get=chassis_angle_output.out;
    
//     angle_all-=angle_now[angle_index]-chassis_angle_get;
//     angle_now[angle_index]=chassis_angle_get;
//     angle_index++;
//     if(angle_index==50)
//     {
//         angle_index=0;
//     }
//    angle_average=angle_all/50.0f; 


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
for (int i = 0; i < 2; ++i)
    {
    first_order_filter_cali(&vel_get[i],chassis.feedback[i],1,5);   
    chassis.feedback[i]=vel_get[i].out;    
    }
    PID_calc(&angle_pid[0], chassis_angle_get, chassis_angle_set);
    PID_calc(&angle_pid[1], chassis_angle_get, chassis_angle_set);
    
    chassis.wheel[0].set.curr = PID_calc(&chassis_pid.wheel_velocity[0], chassis.feedback[0], -(angle_pid[0].out+chassis.reference_rc.vy*50+chassis.reference_rc.vx*50));
    chassis.wheel[1].set.curr = PID_calc(&chassis_pid.wheel_velocity[1], chassis.feedback[1], (angle_pid[1].out+chassis.reference_rc.vy*50-chassis.reference_rc.vx*50));
    
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
    for (int i = 0; i < 2; ++i)
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
