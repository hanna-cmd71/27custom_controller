/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>
#include "gimbal_double_yaw_pitch.h"

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "gimbal.h"
#include "gimbal_double_yaw_pitch.h"
#include "gimbal_yaw_pitch_direct.h"
#include "gimbal_virtual.h"
#include "shoot.h"
#include "shoot_fric_trigger.h"
#include "water_shot.h"
#include "chassis_omni.h"
#include "chassis.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1 // ms

#define USB_OFFLINE_THRESHOLD 100 // ms
#define USB_CONNECT_CNT 10

#define SEND_DURATION_RobotStateInfo 10 // ms
#define SEND_DURATION_PidtoVofa 10      // ms

#define USB_RX_DATA_SIZE 256 // byte
#define USB_RECEIVE_LEN 150  // byte
#define HEADER_SIZE 4        // byte

#define CheckDurationAndSend(send_name)                                                \
    do                                                                                 \
    {                                                                                  \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) \
        {                                                                              \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                \
            UsbSend##send_name##Data();                                                \
        }                                                                              \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t *IMU;
static __used const Gimbal_PID_t *GIMBAL_PID;
static __used const Shoot_s *SHOOT;
static __used const Chassis_PID_t *CHASSIS_PID;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
static SendDataRobotStateInfo_s SEND_DATA_ROBOT_STATE_INFO;
static SendDataPidTuning_s SEND_DATA_PID_TUNING;

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataVofa_s RECEIVE_VOFA_DATA;
 PidGetVofa_t RECEIVE_PID_GET_VOFA;
// 发送数据间隔时间
typedef struct
{
    uint32_t RobotStateInfo;
    uint32_t PidtoVofa;
} LastSendTime_t;

static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendRobotStateInfoData(void);
static void UsbSendPidtoVofaData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void ProcessVofaData(ReceiveDataVofa_s *vofa_data);
//0返回pitch的值1返回yaw的值
fp32 send_usb_date(int reference);
/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const *argument)
{
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&RECEIVE_PID_GET_VOFA, PID_GET_VOFA_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10); // 等待USB设备初始化完成
    UsbInit();

    while (1)
    {
        UsbSendData();
        UsbReceiveData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD)
        {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        }
        else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT)
        {
            USB_OFFLINE = false;
        }
        else
        {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = Subscribe(IMU_NAME);               // 获取IMU数据指针
    GIMBAL_PID = Subscribe(GIMBAL_PID_NAME); // 获取云台PID数据指针
    SHOOT = Subscribe(SHOOT_NAME);           // 获取射击数据指针
    CHASSIS_PID = Subscribe(CHASSIS_PID_NAME);

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_GET_VOFA, 0, sizeof(PidGetVofa_t));
    memset(&RECEIVE_VOFA_DATA, 0, sizeof(ReceiveDataVofa_s));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/

    // 1.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = PACKET_VERSION;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_DATA_INFO_SEND_ID;
    append_CRC8_check_sum( // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.roll = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.pitch = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.yaw = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.encoder_down = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.encoder_up = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.is_super_cap_work = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.super_cap_voltage = 0;

    // 2.初始化PID调节数据包
    // 帧尾部分
    uint8_t tail_data[] = {0x00, 0x00, 0x80, 0x7f};
    memcpy(SEND_DATA_PID_TUNING.tail, tail_data, sizeof(tail_data));
    // 数据部分
    for (int i = 0; i < DEBUG_PACKAGE_NUM; i++)
    {
        SEND_DATA_PID_TUNING.data[i] = 0.0f; // 初始化数据数组
    }
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    if (__TUNING)
    {
        // 发送PID调节数据
        CheckDurationAndSend(PidtoVofa);
    }
    else
    {
        // 发送机器人状态信息
        CheckDurationAndSend(RobotStateInfo);
    }
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t *rx_data_start_address = USB_RX_BUF; // 接收数据包时存放于缓存区的起始位置
    // static uint8_t *rx_data_end_address;                // 接收数据包时存放于缓存区的结束位置

    // 计算数据包的结束位置
    // rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len); // Read data into the buffer

    // 如果接收到数据，处理VOFA协议和原有协议
    if (len > 0)
    {
        uint8_t *current_ptr = USB_RX_BUF;    // 当前处理位置指针
        uint8_t *data_end = USB_RX_BUF + len; // 实际数据结束位置

        // 首先处理VOFA数据帧
        uint16_t vofa_processed = 0;
        uint8_t *vofa_search_ptr = current_ptr;

        while (vofa_search_ptr <= data_end - VOFA_DATA_FRAME_SIZE)
        {
            // 查找VOFA帧头 FA FA
            if (vofa_search_ptr[0] == VOFA_FRAME_HEADER_1 && vofa_search_ptr[1] == VOFA_FRAME_HEADER_2)
            {
                // 找到VOFA帧头，复制完整帧数据
                memcpy(&RECEIVE_VOFA_DATA, vofa_search_ptr, sizeof(ReceiveDataVofa_s));

                // 处理VOFA数据
                ProcessVofaData(&RECEIVE_VOFA_DATA);

                // 从缓冲区中移除已处理的VOFA数据
                uint8_t *remaining_start = vofa_search_ptr + VOFA_DATA_FRAME_SIZE;
                uint32_t remaining_len = data_end - remaining_start;

                if (remaining_len > 0)
                {
                    memmove(vofa_search_ptr, remaining_start, remaining_len);
                }

                // 更新数据结束位置
                data_end -= VOFA_DATA_FRAME_SIZE;
                len -= VOFA_DATA_FRAME_SIZE;
                vofa_processed += VOFA_DATA_FRAME_SIZE;

                // 不增加vofa_search_ptr，因为数据已经前移
            }
            else
            {
                vofa_search_ptr++;
            }
        }

        // 处理原有的通信协议数据包
        uint8_t *sof_address = USB_RX_BUF;
        while (sof_address < data_end)
        {
            // 寻找帧头位置
            while (*(sof_address) != PACKET_VERSION && (sof_address < data_end))
            {
                sof_address++;
            }
            // 判断是否超出接收数据范围
            if (sof_address >= data_end)
            {
                break; // 退出循环
            }

            if (*(sof_address) == PACKET_VERSION)
            {
                // 检查是否有足够的数据进行CRC校验
                if (sof_address + HEADER_SIZE > data_end)
                {
                    break; // 数据不完整，退出
                }

                // 检查CRC8校验
                bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
                if (crc8_ok)
                {
                    uint8_t data_len = sof_address[1];
                    uint8_t data_id = sof_address[2];
                    uint16_t total_packet_len = HEADER_SIZE + data_len + 2; // 包括CRC16

                    // 检查是否有完整的数据包
                    if (sof_address + total_packet_len > data_end)
                    {
                        break; // 数据包不完整，退出
                    }

                    // 检查整包CRC16校验
                    bool crc16_ok = verify_CRC16_check_sum(sof_address, total_packet_len);
                    if (crc16_ok)
                    {
                        switch (data_id)
                        {
                        case ROBOT_CMD_DATA_RECEIVE_ID:
                        {
                            memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                            RECEIVE_TIME = HAL_GetTick(); // 更新接收时间
                        }
                        break;
                        default:
                            break;
                        }
                    }
                    sof_address += total_packet_len;
                }
                else
                {
                    sof_address++;
                }
            }
            else
            {
                sof_address++;
            }
        }

        // 更新接收时间
        RECEIVE_TIME = HAL_GetTick();

        // 处理剩余数据
        uint32_t remaining_data_len = data_end - sof_address;
        if (remaining_data_len > 0 && remaining_data_len < USB_RECEIVE_LEN)
        {
            // 将剩余数据移到缓冲区的起始位置
            memmove(USB_RX_BUF, sof_address, remaining_data_len);
            rx_data_start_address = USB_RX_BUF + remaining_data_len;
        }
        else
        {
            // 没有剩余数据或数据异常，重置起始位置
            rx_data_start_address = USB_RX_BUF;
        }
    }

    // 重置len为下次接收准备
    len = USB_RECEIVE_LEN - (rx_data_start_address - USB_RX_BUF);
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.data.roll = IMU->roll;
    SEND_DATA_ROBOT_STATE_INFO.data.pitch = IMU->pitch;
    SEND_DATA_ROBOT_STATE_INFO.data.yaw = IMU->yaw;

    SEND_DATA_ROBOT_STATE_INFO.data.is_super_cap_work = 0;
    SEND_DATA_ROBOT_STATE_INFO.data.super_cap_voltage = 0;
    #if(BOARD_CURRENT==C_BOARD_UP)
    // SEND_DATA_ROBOT_STATE_INFO.data.encoder_up = yaw_angle_get(1);
    // SEND_DATA_ROBOT_STATE_INFO.data.encoder_down = yaw_angle_get(0);
#endif
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}

/**
 * @brief 发送PID调节数据
 * @param duration 发送周期
 */
static void UsbSendPidtoVofaData(void)
{

#if(__TUNING_MODE == TUNING_GIMBAL_PITCH)
        SEND_DATA_PID_TUNING.data[0] = GIMBAL_PID->pitch_angle.set;
        SEND_DATA_PID_TUNING.data[1] = GIMBAL_PID->pitch_angle.fdb;

        SEND_DATA_PID_TUNING.data[2] = GIMBAL_PID->pitch_angle.out;
        SEND_DATA_PID_TUNING.data[3] = GIMBAL_PID->pitch_angle.Pout;
        SEND_DATA_PID_TUNING.data[4] = GIMBAL_PID->pitch_angle.Iout;
        SEND_DATA_PID_TUNING.data[5] = GIMBAL_PID->pitch_angle.Dout;

        SEND_DATA_PID_TUNING.data[6] = GIMBAL_PID->pitch_velocity.set;
        SEND_DATA_PID_TUNING.data[7] = GIMBAL_PID->pitch_velocity.fdb;

        SEND_DATA_PID_TUNING.data[8] = GIMBAL_PID->pitch_velocity.out;
        SEND_DATA_PID_TUNING.data[9] = GIMBAL_PID->pitch_velocity.Pout;
        SEND_DATA_PID_TUNING.data[10] = GIMBAL_PID->pitch_velocity.Iout;
        SEND_DATA_PID_TUNING.data[11] = GIMBAL_PID->pitch_velocity.Dout;
#elif(__TUNING_MODE == TUNING_GIMBAL_YAW)
        SEND_DATA_PID_TUNING.data[0] = GIMBAL_PID->yaw_angle.set;
        SEND_DATA_PID_TUNING.data[1] = GIMBAL_PID->yaw_angle.fdb;

        SEND_DATA_PID_TUNING.data[2] = GIMBAL_PID->yaw_angle.out;
        SEND_DATA_PID_TUNING.data[3] = GIMBAL_PID->yaw_angle.Pout;
        SEND_DATA_PID_TUNING.data[4] = GIMBAL_PID->yaw_angle.Iout;
        SEND_DATA_PID_TUNING.data[5] = GIMBAL_PID->yaw_angle.Dout;

        SEND_DATA_PID_TUNING.data[6] = GIMBAL_PID->yaw_velocity.set;
        SEND_DATA_PID_TUNING.data[7] = GIMBAL_PID->yaw_velocity.fdb;

        SEND_DATA_PID_TUNING.data[8] = GIMBAL_PID->yaw_velocity.out;
        SEND_DATA_PID_TUNING.data[9] = GIMBAL_PID->yaw_velocity.Pout;
        SEND_DATA_PID_TUNING.data[10] = GIMBAL_PID->yaw_velocity.Iout;
        SEND_DATA_PID_TUNING.data[11] = GIMBAL_PID->yaw_velocity.Dout;
#elif(__TUNING_MODE == TUNING_GIMBAL_YAW_BA)
        SEND_DATA_PID_TUNING.data[0] = GIMBAL_PID->yaw_ba_angle.set;
        SEND_DATA_PID_TUNING.data[1] = GIMBAL_PID->yaw_ba_angle.fdb;

        SEND_DATA_PID_TUNING.data[2] = GIMBAL_PID->yaw_ba_angle.out;
        SEND_DATA_PID_TUNING.data[3] = GIMBAL_PID->yaw_ba_angle.Pout;
        SEND_DATA_PID_TUNING.data[4] = GIMBAL_PID->yaw_ba_angle.Iout;
        SEND_DATA_PID_TUNING.data[5] = GIMBAL_PID->yaw_ba_angle.Dout;

        SEND_DATA_PID_TUNING.data[6] = GIMBAL_PID->yaw_ba_velocity.set;
        SEND_DATA_PID_TUNING.data[7] = GIMBAL_PID->yaw_ba_velocity.fdb;

        SEND_DATA_PID_TUNING.data[8] = GIMBAL_PID->yaw_ba_velocity.out;
        SEND_DATA_PID_TUNING.data[9] = GIMBAL_PID->yaw_ba_velocity.Pout;
        SEND_DATA_PID_TUNING.data[10] = GIMBAL_PID->yaw_ba_velocity.Iout;
        SEND_DATA_PID_TUNING.data[11] = GIMBAL_PID->yaw_ba_velocity.Dout;
#elif(__TUNING_MODE == TUNING_GIMBAL_YAW_UP)
        SEND_DATA_PID_TUNING.data[0] = GIMBAL_PID->yaw_up_angle.set;
        SEND_DATA_PID_TUNING.data[1] = GIMBAL_PID->yaw_up_angle.fdb;

        SEND_DATA_PID_TUNING.data[2] = GIMBAL_PID->yaw_up_angle.out;
        SEND_DATA_PID_TUNING.data[3] = GIMBAL_PID->yaw_up_angle.Pout;
        SEND_DATA_PID_TUNING.data[4] = GIMBAL_PID->yaw_up_angle.Iout;
        SEND_DATA_PID_TUNING.data[5] = GIMBAL_PID->yaw_up_angle.Dout;

        SEND_DATA_PID_TUNING.data[6] = GIMBAL_PID->yaw_up_velocity.set;
        SEND_DATA_PID_TUNING.data[7] = GIMBAL_PID->yaw_up_velocity.fdb;

        SEND_DATA_PID_TUNING.data[8] = GIMBAL_PID->yaw_up_velocity.out;
        SEND_DATA_PID_TUNING.data[9] = GIMBAL_PID->yaw_up_velocity.Pout;
        SEND_DATA_PID_TUNING.data[10] = GIMBAL_PID->yaw_up_velocity.Iout;
        SEND_DATA_PID_TUNING.data[11] = GIMBAL_PID->yaw_up_velocity.Dout;
#elif(__TUNING_MODE == TUNING_SHOOT_FIRC)
        SEND_DATA_PID_TUNING.data[0] = SHOOT->fric_pid[0].fdb * FRIC_MOTOR_R_DIRECTION;
        SEND_DATA_PID_TUNING.data[1] = SHOOT->fric_pid[1].fdb * FRIC_MOTOR_L_DIRECTION;
        SEND_DATA_PID_TUNING.data[2] = SHOOT->fric_pid[2].fdb * FRIC_MOTOR_U_DIRECTION;
        
        SEND_DATA_PID_TUNING.data[3] = SHOOT->fric_pid[0].set * FRIC_MOTOR_R_DIRECTION;
        SEND_DATA_PID_TUNING.data[4] = SHOOT->fric_pid[1].set * FRIC_MOTOR_L_DIRECTION;
        SEND_DATA_PID_TUNING.data[5] = SHOOT->fric_pid[2].set * FRIC_MOTOR_U_DIRECTION;

        SEND_DATA_PID_TUNING.data[6] = 0;
        SEND_DATA_PID_TUNING.data[7] = 0;
        SEND_DATA_PID_TUNING.data[8] = 0;
        SEND_DATA_PID_TUNING.data[9] = 0;
        SEND_DATA_PID_TUNING.data[10] = 0;
        SEND_DATA_PID_TUNING.data[11] = 0;
#elif(__TUNING_MODE == TUNING_SHOOT_TRIGGER)
        SEND_DATA_PID_TUNING.data[0] = SHOOT->trigger_angel_pid.set;
        SEND_DATA_PID_TUNING.data[1] = SHOOT->trigger_angel_pid.fdb;

        SEND_DATA_PID_TUNING.data[2] = SHOOT->trigger_angel_pid.out;
        SEND_DATA_PID_TUNING.data[3] = SHOOT->trigger_angel_pid.Pout;
        SEND_DATA_PID_TUNING.data[4] = SHOOT->trigger_angel_pid.Iout;
        SEND_DATA_PID_TUNING.data[5] = SHOOT->trigger_angel_pid.Dout;

        SEND_DATA_PID_TUNING.data[6] = SHOOT->trigger_speed_pid.set;
        SEND_DATA_PID_TUNING.data[7] = SHOOT->trigger_speed_pid.fdb;
        SEND_DATA_PID_TUNING.data[8] = SHOOT->trigger_speed_pid.out;

        SEND_DATA_PID_TUNING.data[9] = SHOOT->trigger_speed_pid.Pout;
        SEND_DATA_PID_TUNING.data[10] = SHOOT->trigger_speed_pid.Iout;
        SEND_DATA_PID_TUNING.data[11] = SHOOT->trigger_speed_pid.Dout;
#elif(__TUNING_MODE == TUNING_CHASSIS_WHEEL)
        SEND_DATA_PID_TUNING.data[0] = 0;
        SEND_DATA_PID_TUNING.data[1] = 0;
        SEND_DATA_PID_TUNING.data[2] = 0;
        SEND_DATA_PID_TUNING.data[3] = 0;
        SEND_DATA_PID_TUNING.data[4] = 0;
        SEND_DATA_PID_TUNING.data[5] = 0;

        SEND_DATA_PID_TUNING.data[6] = CHASSIS_PID->wheel_velocity[0].set;
        SEND_DATA_PID_TUNING.data[7] = CHASSIS_PID->wheel_velocity[0].fdb;

        SEND_DATA_PID_TUNING.data[8] = CHASSIS_PID->wheel_velocity[0].out;
        SEND_DATA_PID_TUNING.data[9] = CHASSIS_PID->wheel_velocity[0].Pout;
        SEND_DATA_PID_TUNING.data[10] = CHASSIS_PID->wheel_velocity[0].Iout;
        SEND_DATA_PID_TUNING.data[11] = CHASSIS_PID->wheel_velocity[0].Dout;
#elif(__TUNING_MODE == TUNING_CHASSIS_FOLLOW)
        SEND_DATA_PID_TUNING.data[0] = 0;
        SEND_DATA_PID_TUNING.data[1] = 0;
        SEND_DATA_PID_TUNING.data[2] = 0;
        SEND_DATA_PID_TUNING.data[3] = 0;
        SEND_DATA_PID_TUNING.data[4] = 0;
        SEND_DATA_PID_TUNING.data[5] = 0;

        SEND_DATA_PID_TUNING.data[6] = CHASSIS_PID->follow.set;
        SEND_DATA_PID_TUNING.data[7] = CHASSIS_PID->follow.fdb;

        SEND_DATA_PID_TUNING.data[8] = CHASSIS_PID->follow.out;
        SEND_DATA_PID_TUNING.data[9] = CHASSIS_PID->follow.Pout;
        SEND_DATA_PID_TUNING.data[10] = CHASSIS_PID->follow.Iout;
        SEND_DATA_PID_TUNING.data[11] = CHASSIS_PID->follow.Dout;
#elif(__TUNING_MODE == 666)     
        SEND_DATA_PID_TUNING.data[0] = yaw_angle_get(0);
        SEND_DATA_PID_TUNING.data[1] = yaw_angle_get(1);
#endif

    USB_Transmit((uint8_t *)&SEND_DATA_PID_TUNING, sizeof(SendDataPidTuning_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

/**
 * @brief      处理VOFA数据
 * @param[in]  vofa_data: VOFA数据结构体指针
 * @retval     None
 */
static void ProcessVofaData(ReceiveDataVofa_s *vofa_data)
{
    switch (vofa_data->data_id)
    {
    case PID_VOFA_ANGLE_KP:
        RECEIVE_PID_GET_VOFA.angle_kp = vofa_data->value;
        break;
    case PID_VOFA_ANGLE_KI:
        RECEIVE_PID_GET_VOFA.angle_ki = vofa_data->value;
        break;
    case PID_VOFA_ANGLE_KD:
        RECEIVE_PID_GET_VOFA.angle_kd = vofa_data->value;
        break;
    case PID_VOFA_ANGLE_MAX_OUT:
        RECEIVE_PID_GET_VOFA.angle_max_out = vofa_data->value;
        break;
    case PID_VOFA_ANGLE_MAX_IOUT:
        RECEIVE_PID_GET_VOFA.angle_max_iout = vofa_data->value;
        break;
    case PID_VOFA_SPEED_KP:
        RECEIVE_PID_GET_VOFA.speed_kp = vofa_data->value;
        break;
    case PID_VOFA_SPEED_KI:
        RECEIVE_PID_GET_VOFA.speed_ki = vofa_data->value;
        break;
    case PID_VOFA_SPEED_KD:
        RECEIVE_PID_GET_VOFA.speed_kd = vofa_data->value;
        break;
    case PID_VOFA_SPEED_MAX_OUT:
        RECEIVE_PID_GET_VOFA.speed_max_out = vofa_data->value;
        break;
    case PID_VOFA_SPEED_MAX_IOUT:
        RECEIVE_PID_GET_VOFA.speed_max_iout = vofa_data->value;
        break;
    default:
        // 未知数据ID，忽略
        break;
    }
}
fp32 send_usb_date(int reference)
{
    if(reference==0)
    {
        return RECEIVE_ROBOT_CMD_DATA.data.pitch;
    }
    else if(reference==1)
    {
        return RECEIVE_ROBOT_CMD_DATA.data.yaw;
    }
    else
    return 0;
}
/*------------------------------ End of File ------------------------------*/
