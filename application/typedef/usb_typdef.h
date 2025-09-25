#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"


#define DEBUG_PACKAGE_NUM 12

// usb通讯版本号
#define PACKET_VERSION                  ((uint8_t)0x33)
// 发送数据包id
#define ROBOT_STATE_DATA_INFO_SEND_ID   ((uint8_t)0x3B)
// 接收数据包id
#define ROBOT_CMD_DATA_RECEIVE_ID       ((uint8_t)0x3A)

// VOFA协议定义
#define VOFA_FRAME_HEADER_1             ((uint8_t)0xFA)
#define VOFA_FRAME_HEADER_2             ((uint8_t)0xFA)
#define VOFA_DATA_FRAME_SIZE            7  // FA FA + ID(1byte) + float(4bytes)
#define PID_VOFA_ANGLE_KP               ((uint8_t)0x20)
#define PID_VOFA_ANGLE_KI               ((uint8_t)0x21)
#define PID_VOFA_ANGLE_KD               ((uint8_t)0x22)
#define PID_VOFA_ANGLE_MAX_OUT          ((uint8_t)0x23)
#define PID_VOFA_ANGLE_MAX_IOUT         ((uint8_t)0x24)
#define PID_VOFA_SPEED_KP               ((uint8_t)0x25)
#define PID_VOFA_SPEED_KI               ((uint8_t)0x26)
#define PID_VOFA_SPEED_KD               ((uint8_t)0x27)
#define PID_VOFA_SPEED_MAX_OUT          ((uint8_t)0x28)
#define PID_VOFA_SPEED_MAX_IOUT         ((uint8_t)0x29)

typedef struct
{
    uint8_t sof; // 数据帧起始字节，为当前协议版本号
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;

/*-------------------- Send --------------------*/

// PID调节数据发送结构体(vofa justfloat协议)
typedef struct
{
    float data[DEBUG_PACKAGE_NUM];  // 浮点数据数组
    unsigned char tail[4];          // 帧尾固定值{0x00, 0x00, 0x80, 0x7f}
} __packed__ SendDataPidTuning_s;

// 机器人信息数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x03
    struct
    {
        float roll;
        float pitch;
        float yaw;

        fp32 encoder_up;
        fp32 encoder_down;

        float super_cap_voltage;
        bool is_super_cap_work;
        bool is_auto_aim; 
        
    } __packed__ data;
    uint16_t crc;

} __packed__ SendDataRobotStateInfo_s;

/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    FrameHeader_t frame_header; // 数据段id = 0x01
    struct
    {
        float pitch;
        float yaw;
        bool fire;

        float vx;
        float vy;
    } __packed__ data;
    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

// VOFA数据接收结构体
typedef struct
{
    uint8_t header1;        // 0xFA
    uint8_t header2;        // 0xFA
    uint8_t data_id;        // 数据ID
    float value;            // 浮点数值
} __packed__ ReceiveDataVofa_s;

#endif // USB_TYPEDEF_H
