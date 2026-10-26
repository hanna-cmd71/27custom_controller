#include "communication_task.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "communication.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif
// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 100

void communication_task(void const *pvParameters)
{
    SendInit();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1)
    {
        // can发送
        SendRC();

        // 统一发送所有CAN命令
        CanManagerSendAll();

        /* 注：原来这里每轮把 0x0302 帧往 huart1 和 huart6 各发一遍（两套独立 seq、
         * 同一 cmd_id），与 custom_controller_task 里 30Hz 的上行重复。
         * 上行现已收敛为 custom_terminal.c 的 CustomControllerSendToPC 单出口，
         * 本任务只保留收遥控与下行解析。 */

        DataUnpack();

        // 系统延时
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void SendInit()
{
    /*
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void SendRC()
{
    /*
    NOTE : 在其他文件中定义具体内容
   */
}

__weak void CanManagerSendAll()
{
    /*
    NOTE : 在其他文件中定义具体内容
   */
}
