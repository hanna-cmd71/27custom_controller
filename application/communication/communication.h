#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "struct_typedef.h"
#include "robot_param.h"

#define FRAME_HEADER_SOF 0xA5
#define FRAME_HEADER_LEN 5  // （字节）数据帧头部长度
#define DATA_LEN 20         // 数据段长度
#define DATA_NUM 8          // 数据段数量

#define FRAME_HEADER_LEN_OFFEST 1
#define FRAME_HEADER_ID_OFFEST 2
#define FRAME_HEADER_TYPE_OFFEST 3

typedef struct
{
    struct frame_header
    {
        uint8_t sof;               // 起始字节，固定值为0xA5
        uint16_t data_length;      // 数据帧中data的长度
        uint8_t seq;               // 包序号
        uint8_t crc8;              // 帧头CRC8校验
    } __attribute__((packed)) frame_header;

    uint8_t data[DATA_LEN];
    uint16_t crc;
} __attribute__((packed)) BoardCommunicateData_s;

// CAN发送缓冲区结构
typedef struct {
    int16_t data[4];
    bool_t need_send;
} CanSendBuffer_t;

/* 注：0x0302 自定义控制器上行帧（原 CCControlData_t / Controller_t / BOARD_TX_DATA
 * 与 uart_updata() / DataPack()）已整体迁到 custom_terminal.c，只保留一份实现。
 * 本文件只负责下行解析（DataUnpack）与 CAN 发送缓冲。 */

void SendRC(void);

extern void SendInit(void);

extern void DataUnpack(void);

void CanManagerClearBuffer(void);

bool_t CanManagerAddMotor(uint8_t motor_id, uint8_t can_bus, int16_t current);

void CanManagerSendAll(void);

#endif  // __COMMUNICATION_H
