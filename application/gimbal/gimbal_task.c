/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task
  *             完成云台控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024     Penguin          1. done
  *  V1.0.1     Apr-16-2024    Penguin          1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "gimbal_task.h"

#include "attribute_typedef.h"
#include "cmsis_os.h"
#include "gimbal_yaw_pitch_direct.h"
#include "usb_debug.h"
#include "controler_myself.h"
#ifndef GIMBAL_TASK_INIT_TIME
#define GIMBAL_TASK_INIT_TIME 200
#endif  // GIMBAL_TASK_INIT_TIME

#ifndef GIMBAL_CONTROL_TIME
#define GIMBAL_CONTROL_TIME 1
#endif  // GIMBAL_CONTROL_TIME

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

__weak void GimbalPublish(void);
__weak void GimbalInit(void);
__weak void GimbalObserver(void);
__weak void GimbalHandleException(void);
__weak void GimbalSetMode(void);
__weak void GimbalReference(void);
__weak void GimbalConsole(void);
__weak void GimbalSendCmd(void);

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const * pvParameters)
{
    // 云台发布数据
    GimbalPublish();
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    GimbalInit();

    while (1) {
        // 更新状态量
        GimbalObserver();
        // 处理异常
        GimbalHandleException();
        // 设置云台模式
        GimbalSetMode();
        // 更新目标量
        GimbalReference();
        // 计算控制量
        GimbalConsole();
        // 发送控制量
        GimbalSendCmd();
        // 系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void GimbalPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void GimbalSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/*------------------------------ Calibrate Function ------------------------------*/

/**
  * @brief          设置云台校准值，将云台的校准数据设置为传入的校准值
  * @param[in]      yaw_middle:yaw 中值
  * @param[in]      pitch_horizontal:pitch 水平值
  * @param[in]      max_yaw:pitch 最大角度
  * @param[in]      min_yaw:pitch 最小角度
  * @retval         返回空
  * @note           云台任务内部调用的函数
  */
__weak void GimbalSetCaliData(
    const fp32 yaw_middle, const fp32 pitch_horizontal, const fp32 max_pitch, const fp32 min_pitch)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]      yaw_middle:yaw 中值 指针
  * @param[out]      pitch_horizontal:pitch 水平值 指针
  * @param[out]      max_yaw:pitch 最大角度 指针
  * @param[out]      min_yaw:pitch 最小角度 指针
  * @retval         返回空
  * @note           云台任务内部调用的函数
  */
__weak bool_t
GimbalCmdCali(fp32 * yaw_middle, fp32 * pitch_horizontal, fp32 * max_pitch, fp32 * min_pitch)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
    static uint32_t cnt = 0;
    cnt++;
    if (cnt > 1000) {
        cnt = 0;
        *yaw_middle = 0.0f;
        *pitch_horizontal = 0.0f;
        *max_pitch = 0.0f;
        *min_pitch = 0.0f;
        return 1;
    } else {
        return 0;
    }
}

/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_middle:yaw 中值
  * @param[in]      pitch_horizontal:pitch 水平值
  * @param[in]      max_yaw:pitch 最大角度
  * @param[in]      min_yaw:pitch 最小角度
  * @retval         返回空
  * @note           提供给校准任务调用的钩子函数
  */
void set_cali_gimbal_hook(
    const fp32 yaw_middle, const fp32 pitch_horizontal, const fp32 max_pitch, const fp32 min_pitch)
{
    GimbalSetCaliData(yaw_middle, pitch_horizontal, max_pitch, min_pitch);
}

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw_middle:yaw 中值 指针
  * @param[out]     pitch_horizontal:pitch 水平值 指针
  * @param[out]     max_yaw:pitch 最大角度 指针
  * @param[out]     min_yaw:pitch 最小角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @note           提供给校准任务调用的钩子函数
  */
bool_t cmd_cali_gimbal_hook(
    fp32 * yaw_middle, fp32 * pitch_horizontal, fp32 * max_pitch, fp32 * min_pitch)
{
    return GimbalCmdCali(yaw_middle, pitch_horizontal, max_pitch, min_pitch);
}
/*------------------------------ End of File ------------------------------*/
