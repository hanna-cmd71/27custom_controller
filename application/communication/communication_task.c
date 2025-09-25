#include "communication_task.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "communication.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif
int a2;
// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void communication_task(void const * pvParameters)
{
    SendInit();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1) {
a2++;
        // can发送
        SendRC();

        // 统一发送所有CAN命令
        CanManagerSendAll();

        //uart1发送
        DataPack("Hello World", 13, 0);
        UartSendTxMessage(&huart1, (uint8_t *)(&BOARD_TX_DATA), sizeof(BOARD_TX_DATA), 100);
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