/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_double_yaw_pitch.c/h
  * @brief      双yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "CAN_receive.h"
#include "gimbal_double_yaw_pitch.h"
#include "data_exchange.h"
#include "communication.h"
#include "usb_typdef.h"
#include "usb_task.h"
#include "usart.h"
#include <string.h>
#include <KalmanFilter.h>

#if (GIMBAL_TYPE == GIMBAL_DOUBLE_YAW_PITCH)
 Gimbal_s gimbal_direct;
const PidGetVofa_t *pid_get_vofa;
//测试用变量
first_order_filter_type_t yaw_up_out;
first_order_filter_type_t yaw_up_input;
first_order_filter_type_t yaw_ba_out;
first_order_filter_type_t pitch_out;

first_order_filter_type_t gilt_yaw_ba_input;
first_order_filter_type_t gilt_yaw_ba_vel;
first_order_filter_type_t gilt_yaw_ba_acc;
first_order_filter_type_t gilt_yaw_up_vel;
first_order_filter_type_t gilt_yaw_up_acc;

fp32 yaw_up_ba;
fp32 yaw_ba_pos_err;

fp32 aim_data;
fp32 a2;
fp64 last_time;
fp64 error_time;

fp32 yaw_up_last_vel;
fp32 yaw_up_last_pos;
fp32 yaw_up_vel;
fp32 yaw_up_vel_input;
fp32 yaw_up_pos_err;
fp32 yaw_up_pos;


fp32 yaw_ba_last_vel;
fp32 yaw_ba_last_pos;
fp32 yaw_ba_vel;
fp32 yaw_ba_vel_input;
fp32 yaw_ba_pos_err;
fp32 yaw_ba_pos;
fp32 yaw_ba_up_acc;
fp32 yaw_ba_vel_new;
char txData;
char xz[]="xz";
KalmanFilter kf;
//end
#define ROBO_INIT_TIME 5 // (秒)云台初始化时间
/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

   

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{
    // // 添加电机到CAN管理器
    CanManagerAddMotor(gimbal_direct.pitch.id + 4, 1, gimbal_direct.pitch.set.curr);

}

#endif // GIMBAL_YAW_PITCH
