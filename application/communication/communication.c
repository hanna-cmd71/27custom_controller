/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       communication.h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信的数据包结构如下：
    0x00 header
    0x01 data
    0x03 CRC16
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "communication.h"
#include "data_exchange.h"
#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "CAN_communication.h"
#include "remote_control.h"
#include "detect_task.h"

#define USART_RX_BUF_LENGHT 512
#define USART1_FIFO_BUF_LENGTH 1024

// send data
BoardCommunicateData_s BOARD_TX_DATA;
// 全局CAN发送缓冲区
static CanSendBuffer_t can_buffers[2][2] = {0}; // [CAN总线][标准ID类型]
// can_buffers[0][0] = CAN1 0x200 (ID 1-4)
// can_buffers[0][1] = CAN1 0x1FF (ID 5-8) 
// can_buffers[1][0] = CAN2 0x200 (ID 1-4)
// can_buffers[1][1] = CAN2 0x1FF (ID 5-8)

// receive data
uint8_t BOARD_RX_DATA[DATA_NUM][DATA_LEN + 1];  //第一位存放数据长度信息

// receive data buffer
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t usart1_fifo;
uint8_t usart1_fifo_buf[USART1_FIFO_BUF_LENGTH];
fp32 velocity;
// unpack_data_t referee_unpack_obj;

// 发送初始化
void SendInit(void)
{
    fifo_s_init(&usart1_fifo, usart1_fifo_buf, USART1_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);
}

void SendRC(){

    if(BOARD_CURRENT != C_BOARD_UP || toe_is_error(DBUS_TOE)) return; // 如果当前板子不是上板，或者遥控器断开连接，则不发送遥控器数据
    
    uint8_t data_8[8];
    
    // 获取-1到1的浮点数
    float ch0_float = GetDt7RcCh(0); // 范围: -1.0 ~ 1.0
    float ch1_float = GetDt7RcCh(1); // 范围: -1.0 ~ 1.0
    if (GetDt7Keyboard(KEY_SHIFT))
    {
        velocity=1.0f;
    }
    else
    {
        velocity=0.5f;
    }
    // 使用128作为中值的映射算法
    if(GetDt7Keyboard(KEY_W))
    {
    ch1_float=velocity;
    }
    else if (GetDt7Keyboard(KEY_S))
    {
    ch1_float=-velocity;  
    }
    if(GetDt7Keyboard(KEY_D))
    {
    ch0_float=velocity;
    }
    else if (GetDt7Keyboard(KEY_A))
    {
    ch0_float=-velocity; 
    }
    // 公式: uint8_val = (float_val * 127) + 128
    int16_t temp_ch0 = (int16_t)(ch0_float * 127.0f + 128.0f);
    int16_t temp_ch1 = (int16_t)(ch1_float * 127.0f + 128.0f);
    
    // 限制范围防止溢出
    data_8[0] = (uint8_t)(temp_ch0 < 0 ? 0 : (temp_ch0 > 255 ? 255 : temp_ch0));
    data_8[1] = (uint8_t)(temp_ch1 < 0 ? 0 : (temp_ch1 > 255 ? 255 : temp_ch1));
    
    data_8[2] = 0;
    data_8[3] = 0;

    switch (GetDt7RcSw(0))
    {
    case RC_SW_UP:
        data_8[4] = CHASSIS_SPIN;
        break;
    case RC_SW_MID:
        data_8[4] = CHASSIS_NO_FOLLOW;
        break;
    case RC_SW_DOWN:
        data_8[4] = CHASSIS_NOMOVE;
        break;
    default:
        data_8[4] = CHASSIS_NOMOVE;
        break;
    }

    data_8[5] = 1;
    data_8[6] = 1;
    data_8[7] = 1;

    // 通过CAN总线发送遥控器的四个通道数据到指定板子
    CanSendDataToBoard(BOARD_CAN, BOARD_DATA_ID, BOARD_OTHER, data_8);
}

/**
 * @brief          清空CAN发送缓冲区
 * @param[in]      none
 * @retval         none
 */
void CanManagerClearBuffer(void)
{
    for (int can = 0; can < 2; can++) {
        for (int std_id = 0; std_id < 2; std_id++) {
            can_buffers[can][std_id].need_send = false;
            for (int i = 0; i < 4; i++) {
                can_buffers[can][std_id].data[i] = 0;
            }
        }
    }
}

/**
 * @brief          添加电机控制量到发送缓冲区
 * @param[in]      motor_id: 电机ID (1-8)
 * @param[in]      can_bus: CAN总线 (1或2)  
 * @param[in]      current: 控制电流
 * @retval         bool: 成功返回true，失败返回false
 */
bool_t CanManagerAddMotor(uint8_t motor_id, uint8_t can_bus, int16_t current)
{
    // 参数检查
    if (motor_id < 1 || motor_id > 8 || can_bus < 1 || can_bus > 2) {
        return false;
    }
    
    uint8_t can_index = can_bus - 1;  // 转换为数组索引
    uint8_t std_id_index, data_index;
    
    // 确定标准ID和数据索引
    if (motor_id >= 1 && motor_id <= 4) {
        std_id_index = 0;  // 0x200
        data_index = motor_id - 1;
    } else {
        std_id_index = 1;  // 0x1FF
        data_index = motor_id - 5;
    }
    
    // 检查冲突
    if (can_buffers[can_index][std_id_index].data[data_index] != 0) {
        // 发现冲突，可以选择覆盖或报错
        // 这里选择覆盖，但可以添加日志记录
    }
    
    // 添加到缓冲区
    can_buffers[can_index][std_id_index].data[data_index] = current;
    can_buffers[can_index][std_id_index].need_send = true;
    
    return true;
}

/**
 * @brief          发送所有缓冲区的CAN命令
 * @param[in]      none
 * @retval         none
 */
void CanManagerSendAll(void)
{
    const uint16_t std_ids[2] = {0x200, 0x1FF};
    
    for (int can = 0; can < 2; can++) {
        for (int std_id = 0; std_id < 2; std_id++) {
            if (can_buffers[can][std_id].need_send) {
                CanCmdDjiMotor(can + 1, std_ids[std_id],
                              can_buffers[can][std_id].data[0],
                              can_buffers[can][std_id].data[1], 
                              can_buffers[can][std_id].data[2],
                              can_buffers[can][std_id].data[3]);
            }
        }
    }
    
    // 发送完成后清空缓冲区
    CanManagerClearBuffer();
}

// 4pin Uart口中断处理函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[0], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[1], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        }
    }
}

/**
 * @brief 将数据打包填入发送数据结构体中
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 * @param data_id 数据段id
 */
void DataPack(uint8_t * data, uint8_t data_lenth, uint8_t data_id)
{
    BOARD_TX_DATA.frame_header.sof = 0xA5;        // 数据帧起始字节，固定值为 0xA5
    BOARD_TX_DATA.frame_header.len = data_lenth;  // 数据帧中数据段的长度
    BOARD_TX_DATA.frame_header.id = data_id;
    // 添加帧头 CRC8 校验位
    append_CRC8_check_sum(
        (uint8_t *)(&BOARD_TX_DATA.frame_header), sizeof(BOARD_TX_DATA.frame_header));

    memcpy(BOARD_TX_DATA.data, data, data_lenth);  // 数据段

    // 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&BOARD_TX_DATA), sizeof(BOARD_TX_DATA));
}

void DataUnpack(void)
{
    uint8_t byte = 0;
    uint8_t frame_header[FRAME_HEADER_LEN] = {0};
    uint8_t received[FRAME_HEADER_LEN + DATA_LEN + 2] = {0};

    while (fifo_s_used(&usart1_fifo)) {
        byte = fifo_s_get(&usart1_fifo);
        if (byte == FRAME_HEADER_SOF) {
            frame_header[0] = byte;
            fifo_s_gets(&usart1_fifo, (char *)(frame_header + 1), FRAME_HEADER_LEN - 1);

            uint8_t header_crc_ok = verify_CRC8_check_sum(frame_header, FRAME_HEADER_LEN);
            if (header_crc_ok) {
                memcpy(received, frame_header, FRAME_HEADER_LEN);  //转移帧头信息
                uint8_t data_len = frame_header[FRAME_HEADER_LEN_OFFEST];
                uint8_t data_id = frame_header[FRAME_HEADER_ID_OFFEST];
                // uint8_t data_type = frame_header[FRAME_HEADER_TYPE_OFFEST];

                fifo_s_gets(&usart1_fifo, (char *)(received + FRAME_HEADER_LEN), DATA_LEN + 2);
                uint16_t crc_ok = verify_CRC16_check_sum(received, sizeof(BoardCommunicateData_s));
                if (crc_ok) {
                    BOARD_RX_DATA[data_id][0] = data_len;
                    memcpy(&BOARD_RX_DATA[data_id][1], received + FRAME_HEADER_LEN, data_len);
                }
            }
        }
    }
}


