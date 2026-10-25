#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "struct_typedef.h"

#define SELF_ID 0x01                // self id
#define FIRMWARE_VERSION_X 1        // firmware version x.y.z-release-beta
#define FIRMWARE_VERSION_Y 4        // firmware version x.y.z-release-beta
#define FIRMWARE_VERSION_Z 8        // firmware version x.y.z-release-beta
#define FIRMWARE_VERSION_RELEASE 1  // firmware version x.y.z-release-beta
#define FIRMWARE_VERSION_BETA 1     // firmware version x.y.z-release-beta

#define CALI_FUNC_CMD_ON 1    // 设置校准
#define CALI_FUNC_CMD_INIT 0  // 已经校准过，设置校准值

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_9  // 保存的flash页地址,page 9
#define GYRO_CONST_MAX_TEMP 45.0f            // 最大陀螺仪控制温度
#define CALIED_FLAG 0xAA                     // means it has been calibrated

#define CALIBRATE_END_TIME 20000          // (ms)有20s可以用遥控器进行校准
#define RC_CALI_BUZZER_MIDDLE_TIME 10000  // (ms)当10s的时候,蜂鸣器切成高频声音

#define RC_CALI_BUZZER_CYCLE_TIME 400  // (ms)
#define RC_CALI_BUZZER_PAUSE_TIME 200  // (ms)

#define GYRO_CALIBRATE_TIME 20000  // (ms)gyro calibrate time,陀螺仪校准时间
#define RC_CMD_LONG_TIME 2000  // (ms)遥控器长按时间（达到时间后切换校准状态）

#define CALI_SENSOR_HEAD_LEGHT 1

#define RC_CALI_VALUE_HOLE 600  // 遥控器阈值，遥控器通道的最大值为660。

// Macro functions

#define cali_get_mcu_temperature() get_temprate()  // 获取stm32片内温度，计算imu的控制温度

// clang-format off
#define cali_flash_read(address, buf, len) \
    flash_read((address), (buf), (len))  // flash 读取函数
#define cali_flash_write(address, buf, len) \
    flash_write_single_address((address), (buf), (len))  // flash 写入函数
#define cali_flash_erase(address, page_num) \
    flash_erase_address((address), (page_num))  // flash擦除函数
// clang-format on

#define gyro_cali_disable_control() RC_unable()  // 当imu在校准时候,失能遥控器
#define gyro_cali_enable_control() RC_restart(SBUS_RX_BUF_NUM)  // 当imu校准完成,使能遥控器

#define gyro_cali_fun(cali_scale, cali_offset, time_count) \
    INS_cali_gyro((cali_scale), (cali_offset), (time_count))  // 计算陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset) \
    INS_set_cali_gyro((cali_scale), (cali_offset))  // 设置在INS task内的陀螺仪零漂

//cali device name
typedef enum {
    CALI_HEAD = 0,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_CHASSIS,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

typedef struct
{
    uint8_t name[3];                                    // 设备名称
    uint8_t cali_done;                                  // 0x55 表示已经校准过
    uint8_t flash_len : 7;                              // 缓冲区长度
    uint8_t cali_cmd : 1;                               // 1 表示运行校准钩子函数
    uint32_t * flash_buf;                               // 指向设备校准数据的链接
    bool_t (*cali_hook)(uint32_t * point, bool_t cmd);  // 校准函数的钩子函数
} __attribute__((packed)) cali_sensor_t;

//header device (固件信息)
typedef struct
{
    uint8_t self_id;                    // the "SELF_ID"
    uint16_t firmware_version_x;        // set to the "FIRMWARE_VERSION"
    uint16_t firmware_version_y;        // set to the "FIRMWARE_VERSION"
    uint16_t firmware_version_z;        // set to the "FIRMWARE_VERSION"
    uint16_t firmware_version_release;  // set to the "FIRMWARE_VERSION"
    uint16_t firmware_version_beta;     // set to the "FIRMWARE_VERSION"
    //'temperature'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;  // imu control temperature
} __attribute__((packed)) head_cali_t;

//gimbal device (云台设备)
typedef struct
{
    fp32 yaw_middle;
    fp32 pitch_horizontal;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;

//gyro, accel, mag device (陀螺仪,加速度计,磁力计设备)
typedef struct
{
    fp32 offset[3];  //x,y,z
    fp32 scale[3];   //x,y,z
} imu_cali_t;

//chassis device (底盘设备)
typedef struct
{
    fp32 motor_middle[4];  // 电机中值
} chassis_cali_t;

extern bool_t cali_head_hook(uint32_t * cali, bool_t cmd);
extern bool_t cali_gimbal_hook(uint32_t * cali, bool_t cmd);
extern bool_t cali_gyro_hook(uint32_t * cali, bool_t cmd);
extern bool_t cali_chassis_hook(uint32_t * cali, bool_t cmd);

#endif  // CALIBRATE_TASK_H
