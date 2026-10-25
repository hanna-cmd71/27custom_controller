/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       calibrate_task.c/h
  * @brief      校准任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-27-2024     Penguin         1. done
  @verbatim
  ==============================================================================
  *
  *         使用遥控器进行开始校准
  *             第一步:遥控器的两个开关都打到下
  *             第二步:两个摇杆打成\../,保存两秒.\.代表左摇杆向右下打.（中下）
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *                    或者摇杆打成'\/' 开始云台校准
  *                    或者摇杆打成/''\ 开始底盘校准
  *
  *             数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
  *             例如head_cali有八个字节,但它需要12字节在flash,如果它从0x080A0000开始
  *             0x080A0000-0x080A0007: head_cali数据
  *             0x080A0008: 名字name[0]
  *             0x080A0009: 名字name[1]
  *             0x080A000A: 名字name[2]
  *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0xAA,意味着head_cali已经校准了
  *
  *         添加新设备
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
  *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
  *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
  *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#define CALIBRATE_CONTROL_TIME 1

#include "calibrate_task.h"

#include <stdlib.h>
#include <string.h>

#include "IMU_task.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"
#include "calibrate.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "usb_debug.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_high_water;
#endif

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT                                              \
    (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3 + \
     sizeof(chassis_cali_t) + CALI_LIST_LENGHT * 4)

#define CheckRcCaliValue(sign0, sign1, sign2, sign3)                                   \
    (switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && \
     calibrate_RC->rc.ch[0] sign0 RC_CALI_VALUE_HOLE &&                                \
     calibrate_RC->rc.ch[1] sign1 RC_CALI_VALUE_HOLE &&                                \
     calibrate_RC->rc.ch[2] sign2 RC_CALI_VALUE_HOLE &&                                \
     calibrate_RC->rc.ch[3] sign3 RC_CALI_VALUE_HOLE)

/*------------------------------ Variable Definition ------------------------------*/

static const RC_ctrl_t * calibrate_RC;  //remote control point
static head_cali_t head_cali;           //head cali data
static gimbal_cali_t gimbal_cali;       //gimbal cali data
static imu_cali_t accel_cali;           //accel cali data
static imu_cali_t gyro_cali;            //gyro cali data
static imu_cali_t mag_cali;             //mag cali data
static chassis_cali_t chassis_cali;     //chassis cali data

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG", "CHS"};

//cali data address
static uint32_t * cali_sensor_buf[CALI_LIST_LENGHT] = {
    // clang-format off
    (uint32_t *)&head_cali,
    (uint32_t *)&gimbal_cali,
    (uint32_t *)&gyro_cali,
    (uint32_t *)&accel_cali,
    (uint32_t *)&mag_cali,
    (uint32_t *)&chassis_cali
    // clang-format on
};
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
    // clang-format off
    sizeof(head_cali_t) / 4,
    sizeof(gimbal_cali_t) / 4, 
    sizeof(imu_cali_t) / 4, 
    sizeof(imu_cali_t) / 4,
    sizeof(imu_cali_t) / 4,
    sizeof(chassis_cali_t) / 4
    // clang-format on
};
// clang-format off
void * cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook,
                                          cali_gimbal_hook,
                                          cali_gyro_hook,
                                          NULL,
                                          NULL,
                                          cali_chassis_hook};
// clang-format on

static uint32_t calibrate_systemTick;
static uint32_t deltaTick = 0;
static uint32_t lastTick = 0;

static CaliBuzzerState_e cali_buzzer_state = CALI_BUZZER_OFF;

/*------------------------------ Function Declaration ------------------------------*/

/*******************************************************************************/
/* RC Cmd Function                                                             */
/*******************************************************************************/

/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void);

/*******************************************************************************/
/* Flash Operation Function                                                    */
/*******************************************************************************/

static void cali_data_read(void);
static void cali_data_write(void);

/*------------------------------ Task Definition ------------------------------*/

/**
  * @brief          校准任务
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void calibrate_task(void const * pvParameters)
{
    static uint8_t i = 0;

    Publish(&cali_buzzer_state, CALI_BUZZER_STATE_NAME);

    calibrate_RC = get_remote_control_point();
    // vTaskDelay(10);
    // calibrate_RC = Subscribe("virtual_rc_ctrl");

    while (1) {
        deltaTick = xTaskGetTickCount() - lastTick;
        lastTick = xTaskGetTickCount();

        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++) {
            if (cali_sensor[i].cali_cmd) {
                if (cali_sensor[i].cali_hook != NULL) {
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON)) {
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set CALIED_FLAG
                        cali_sensor[i].cali_done = CALIED_FLAG;
                        cali_sensor[i].cali_cmd = 0;
                        //write
                        cali_data_write();
                        // 用更新后的数据设置设备相关参数
                        cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
                    }
                }
            }
        }

        vTaskDelay(CALIBRATE_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*------------------------------ Function Definition ------------------------------*/

/*******************************************************************************/
/* Hook Function                                                               */
/*******************************************************************************/

/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_head_hook(uint32_t * cali, bool_t cmd)
{
    head_cali_t * local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

    local_cali_t->firmware_version_x = FIRMWARE_VERSION_X;
    local_cali_t->firmware_version_y = FIRMWARE_VERSION_Y;
    local_cali_t->firmware_version_z = FIRMWARE_VERSION_Z;
    local_cali_t->firmware_version_release = FIRMWARE_VERSION_RELEASE;
    local_cali_t->firmware_version_beta = FIRMWARE_VERSION_BETA;

    return 1;
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_gimbal_hook(uint32_t * cali, bool_t cmd)
{
    gimbal_cali_t * local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        set_cali_gimbal_hook(
            local_cali_t->yaw_middle, local_cali_t->pitch_horizontal, local_cali_t->pitch_max_angle,
            local_cali_t->pitch_min_angle);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        return cmd_cali_gimbal_hook(
            &local_cali_t->yaw_middle, &local_cali_t->pitch_horizontal,
            &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle);
    }

    return 0;
}

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_gyro_hook(uint32_t * cali, bool_t cmd)
{
    imu_cali_t * local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        static uint32_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        count_time += deltaTick;
        if (count_time > GYRO_CALIBRATE_TIME) {
            count_time = 0;
            gyro_cali_enable_control();
            return 1;
        } else {
            gyro_cali_disable_control();  //disable the remote control to make robot no move
            return 0;
        }
    }

    return 0;
}

/**
  * @brief          底盘设备校准
  * @param[in][out] cali:指针指向底盘数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_chassis_hook(uint32_t * cali, bool_t cmd)
{
    chassis_cali_t * local_cali_t = (chassis_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        set_cali_chassis_hook(local_cali_t->motor_middle);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        return cmd_cali_chassis_hook(local_cali_t->motor_middle);
    }

    return 0;
}

/*******************************************************************************/
/* RC Cmd Function                                                             */
/*******************************************************************************/

/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void)
{
    typedef enum{
        FLAG_NONE = 0, // 无校准
        FLAG_BEGIN,    // 校准模式
        FLAG_TOGGLE,   // 校准模式变更（进入校准/退出校准）
        FLAG_GIMBAL,   // 云台校准
        FLAG_IMU,      // imu校准
        FLAG_CHASSIS,  // 底盘校准
    }CaliFlag_e;

    static uint32_t rc_cmd_systemTick    = 0;
    static uint16_t buzzer_time          = 0;
    static uint16_t rc_cmd_time          = 0;
    static CaliFlag_e  rc_action_flag    = FLAG_NONE; // rc动作标志
    static CaliFlag_e  cali_state_flag   = FLAG_NONE; //当前执行的校准状态标志

    static uint8_t i;
    //如果已经在校准，就返回
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_sensor[i].cali_cmd) {
            buzzer_time = 0;
            rc_cmd_time = 0;

            return;
        }
    }

    //*********************************************************
    //* 根据rc的动作，选择进入的校准模式
    //*********************************************************
    if (cali_state_flag == FLAG_NONE && rc_action_flag == FLAG_TOGGLE &&
        rc_cmd_time > RC_CMD_LONG_TIME) {
        // 进入校准模式
        rc_cmd_systemTick = xTaskGetTickCount();
        cali_state_flag = FLAG_BEGIN;
        rc_cmd_time = 0;
    } else if (rc_action_flag == FLAG_NONE) {
        // 退出校准模式
        cali_state_flag = FLAG_NONE;
        rc_cmd_time = 0;
        cali_buzzer_state = CALI_BUZZER_OFF;
    } else if (
        cali_state_flag > FLAG_NONE && rc_action_flag == FLAG_GIMBAL &&
        rc_cmd_time > RC_CMD_LONG_TIME) {
        // 切换为云台校准模式
        cali_state_flag = FLAG_GIMBAL;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
        cali_buzzer_state = CALI_BUZZER_GIMBAL;
        return;
    } else if (
        cali_state_flag > FLAG_NONE && rc_action_flag == FLAG_IMU &&
        rc_cmd_time > RC_CMD_LONG_TIME) {
        // 切换为imu校准模式
        cali_state_flag = FLAG_IMU;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        //update control temperature
        head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
            head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
        cali_buzzer_state = CALI_BUZZER_IMU;
        return;
    } else if (
        cali_state_flag > FLAG_NONE && rc_action_flag == FLAG_CHASSIS &&
        rc_cmd_time > RC_CMD_LONG_TIME) {
        // 切换为底盘校准模式
        cali_state_flag = FLAG_CHASSIS;
        rc_cmd_time = 0;
        cali_sensor[CALI_CHASSIS].cali_cmd = 1;
        cali_buzzer_state = CALI_BUZZER_CHASSIS;
        return;
    }

    //*********************************************************
    //* 根据遥控器摇杆的位置，判断rc的动作
    //*********************************************************
    if (CheckRcCaliValue(< -, <-, >, < -) &&
        (cali_state_flag == FLAG_NONE || cali_state_flag == FLAG_BEGIN)) {
        // 两个摇杆打成 \../,保持2s,切换校准模式(进入/退出)
        rc_action_flag = FLAG_TOGGLE;
        rc_cmd_time += deltaTick;
    } else if (CheckRcCaliValue(>, >, <-, >) && cali_state_flag > FLAG_NONE) {
        // 在校准模式中,两个摇杆打成'\/',保持2s,进入云台校准
        rc_cmd_time += deltaTick;
        rc_action_flag = FLAG_GIMBAL;
    } else if (CheckRcCaliValue(>, < -, < -, < -) && cali_state_flag > FLAG_NONE) {
        // 在校准模式中,两个摇杆打成./\.,保持2s,进入陀螺仪校准
        rc_cmd_time += deltaTick;
        rc_action_flag = FLAG_IMU;
    } else if (CheckRcCaliValue(<-, >, >, >) && cali_state_flag > FLAG_NONE) {
        // 在校准模式中,两个摇杆打成/''\,保持2s,进入底盘校准
        rc_cmd_time += deltaTick;
        rc_action_flag = FLAG_CHASSIS;
    } else {
        rc_cmd_time = 0;
    }

    //*********************************************************
    //* 根据校准进行的时间，判断是否需要停止校准，以及是否需要蜂鸣器提示
    //*********************************************************
    calibrate_systemTick = xTaskGetTickCount();
    if (cali_state_flag == FLAG_NONE) {
        // 无需改变状态
        return;
    } else if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME) {
        //超过20s,停止校准
        rc_action_flag = FLAG_NONE;
        return;
    } else if (
        calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME &&
        rc_cmd_systemTick != 0 && cali_state_flag > FLAG_NONE) {
        cali_buzzer_state = CALI_BUZZER_MIDDLE_TIME;
    } else if (
        calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 &&
        cali_state_flag > FLAG_NONE) {
        cali_buzzer_state = CALI_BUZZER_BEGIN;
    }

    //*********************************************************
    //* 蜂鸣器计时循环
    //*********************************************************
    if (cali_state_flag > FLAG_NONE) {
        buzzer_time++;
    }
    if (buzzer_time > RC_CALI_BUZZER_CYCLE_TIME && cali_state_flag > FLAG_NONE) {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && cali_state_flag > FLAG_NONE) {
        cali_buzzer_state = CALI_BUZZER_OFF;
    }
}

/*******************************************************************************/
/* Flash Operation Function                                                    */
/*******************************************************************************/

/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        //read the data in flash,
        cali_flash_read(
            FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);

        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        cali_flash_read(
            FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);

        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }
}

/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        //copy the data of device calibration data
        memcpy(
            (void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf,
            cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
        memcpy(
            (void *)(flash_write_buf + offset), (void *)cali_sensor[i].name,
            CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }

    //erase the page
    cali_flash_erase(FLASH_USER_ADDR, 1);
    //write data
    cali_flash_write(
        FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
  * @brief          校准任务参数初始化
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_sensor[i].cali_done == CALIED_FLAG) {
            if (cali_sensor[i].cali_hook != NULL) {
                //if has been calibrated, set to init
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        } else if (i == CALI_HEAD) {
            //if head has not been calibrated, set to cali
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
int8_t get_control_temperature(void) { return head_cali.temperature; }

/*------------------------------ End of File ------------------------------*/
