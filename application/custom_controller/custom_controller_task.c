/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_task.c/h
  * @brief      完成自定义控制器控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. 完成基本框架
  *  V1.0.1     Aug-23-2024     Penguin         1. 将接收和发送模式分开
  *  V1.0.2     Aug-23-2024     Penguin         1. 定义了自定义控制器的统一控制协议，
  *                                                无需再区分发送和接收模式了
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "custom_controller_task.h"

#include "attribute_typedef.h"
#include "cmsis_os.h"
#include "custom_terminal.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t custom_controller_high_water;
#endif

static uint32_t lastTick = 0;

__weak void CustomControllerPublish(void);
__weak void CustomControllerInit(void);
__weak void CustomControllerHandleException(void);
__weak void CustomControllerSetMode(void);
__weak void CustomControllerObserver(void);
__weak void CustomControllerReference(void);
__weak void CustomControllerConsole(void);
__weak void CustomControllerSendCmd(void);
__weak void CustomControllerSendToPC(void);

/**
 * @brief          自定义控制器任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void custom_controller_task(void const * pvParameters)
{
    CustomControllerPublish();
    vTaskDelay(CUSTOM_CONTROLLER_TASK_INIT_TIME);
    // 初始化
    CustomControllerInit();

    while (1) {
        // 更新状态量
        CustomControllerObserver();
        // 处理异常
        CustomControllerHandleException();
        // 设置模式
        CustomControllerSetMode();
        // 设置目标量
        CustomControllerReference();
        // 计算控制量
        CustomControllerConsole();
        // 发送控制量
        CustomControllerSendCmd();

        // 注意：lastTick 必须在这里更新，否则条件恒真，会退化成每拍都发
        if ((xTaskGetTickCount() - lastTick) >= CUSTOM_CONTROLLER_SEND_TIME) {
            lastTick = xTaskGetTickCount();
            CustomControllerSendToPC();
        }

        // 系统延时
        vTaskDelay(CUSTOM_CONTROLLER_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        custom_controller_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void CustomControllerPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

__weak void CustomControllerHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void CustomControllerSendToPC(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容（自定义终端在 custom_terminal.c 里实现）
    */
}
/*------------------------------ End of File ------------------------------*/
