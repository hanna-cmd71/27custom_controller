/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       controler_myself.c/h
  * @brief      自定义云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成框架搭建。
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "controler_myself.h"
#include "data_exchange.h"
#include "communication.h"
#include "CAN_cmd_SBzdatou.h"
#include "CAN_receive.h"
#include "detect_task.h"

/*-------------------- Global var --------------------*/

Gimbal_s controller_myself;
Gimbal_PID_t controller_myself_pid;
int a1;
int datouctrl;
int sendctrl;
SBzdatouMeasure_s *SBzdatouMeasure;
/*-------------------------------- Internal functions ---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 是否需要继续初始化
 */
bool ControllerMyselfInitJudge(void)
{
    return false;
}

/*------------------------- The end of internal functions --------------------------------------*/

/*-------------------- Public (inline) --------------------*/

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
    return 0.0f;
}

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
    return false;
}

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 (float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
    return 0.0f;
}

/*-------------------- Public --------------------*/

/**
 * @brief          发布数据
 * @param[in]      none
 * @retval         none
 */
void GimbalPublish(void)
{
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void)
{
    for (uint32_t ext_id = 0x100; ext_id <= 0x600; ext_id += 0x100)
    {
        SBzdatou_ecd_get(1, ext_id);
        vTaskDelay(20);
    }
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void)
{
    // SBzdatouMeasure=GetSBzdatouMeasurePoint (1);
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void)
{
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void)
{
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{
    if (sendctrl == 0)
    {
        for (uint32_t ext_id = 0x100; ext_id <= 0x200; ext_id += 0x100)
        {
            SBzdatou_torque_control(1, ext_id, 0x01, datouctrl);
        }
        sendctrl = 1;
    }
    else if (sendctrl == 1)
    {
        for (uint32_t ext_id = 0x300; ext_id <= 0x400; ext_id += 0x100)
        {
            SBzdatou_torque_control(1, ext_id, 0x01, datouctrl);
        }
        sendctrl = 2;
    }
    else
    {
        for (uint32_t ext_id = 0x500; ext_id <= 0x600; ext_id += 0x100)
        {
            SBzdatou_torque_control(1, ext_id, 0x01, datouctrl);
        }
        sendctrl = 0;
    }
    // CanManagerAddMotor(1, 1, 600);
}

/************************ END OF FILE ************************/
