#include "CAN_cmd_robostride.h"

#include "bsp_delay.h"
#include "user_lib.h"

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,

};
/**
 * @brief          电机使能运行（MIT指令1）
 * @param[in]      can 发送数据使用的can口
 * @param[in]      id 目标电机CAN_ID
 * @return         none
 */
void Robostride_enable(uint8_t can, uint16_t id)
{
    hcan_t *hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL)
        return;
    CAN_CTRL_DATA.hcan = hcan;
    CAN_CTRL_DATA.tx_header.IDE = CAN_ID_STD;
    CAN_CTRL_DATA.tx_header.StdId = id;
    CAN_CTRL_DATA.tx_header.DLC = 8;
    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFC;
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          MIT模式电机控制指令（MIT指令3）
 * @param[in]      can 发送数据使用的can口
 * @param[in]      id 目标电机CAN_ID
 * @param[in]      pos 目标角度 (-12.57~12.57 rad)
 * @param[in]      vel 目标速度 (-33~33 rad/s)
 * @param[in]      kp 位置刚度 (0~500)
 * @param[in]      kd 阻尼 (0~5)
 * @param[in]      torque 前馈力矩 (-14~14 N.m)
 * @return         none
 */
void Robostride_mit_control(
    uint8_t can, uint16_t id, float pos, float vel, float kp, float kd, float torque)
{
    hcan_t *hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL)
        return;

    uint16_t pos_int = float_to_uint(pos, ROBOSTRIDE_P_MIN, ROBOSTRIDE_P_MAX, 16);
    uint16_t vel_int = float_to_uint(vel, ROBOSTRIDE_V_MIN, ROBOSTRIDE_V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, ROBOSTRIDE_KP_MIN, ROBOSTRIDE_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, ROBOSTRIDE_KD_MIN, ROBOSTRIDE_KD_MAX, 12);
    uint16_t tor_int = float_to_uint(torque, ROBOSTRIDE_T_MIN, ROBOSTRIDE_T_MAX, 12);

    CAN_CTRL_DATA.hcan = hcan;
    CAN_CTRL_DATA.tx_header.IDE = CAN_ID_STD;
    CAN_CTRL_DATA.tx_header.StdId = id;
    CAN_CTRL_DATA.tx_header.DLC = 8;

    // CAN_CTRL_DATA.tx_data[0] = (uint8_t)(pos_int >> 8);
    // CAN_CTRL_DATA.tx_data[1] = (uint8_t)(pos_int);
    // CAN_CTRL_DATA.tx_data[2] = (uint8_t)(vel_int >> 4);
    // CAN_CTRL_DATA.tx_data[3] = (uint8_t)(((vel_int & 0x0F) << 4) | (kp_int >> 8));
    // CAN_CTRL_DATA.tx_data[4] = (uint8_t)(kp_int);
    // CAN_CTRL_DATA.tx_data[5] = (uint8_t)(kd_int >> 4);
    CAN_CTRL_DATA.tx_data[0] = 0;
    CAN_CTRL_DATA.tx_data[1] = 0;
    CAN_CTRL_DATA.tx_data[2] = 0;
    CAN_CTRL_DATA.tx_data[3] = 0;
    CAN_CTRL_DATA.tx_data[4] = 0;
    CAN_CTRL_DATA.tx_data[5] = 0;
    CAN_CTRL_DATA.tx_data[6] = (uint8_t)(((0x00) << 4) | (tor_int >> 8));
    CAN_CTRL_DATA.tx_data[7] = (uint8_t)(tor_int);
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          RobStride电机MIT反馈帧解码（应答指令1）
 * @param[out]     robostride_measure 电机数据缓存
 * @param[in]      rx_data 反馈数据
 * @note           Byte0:电机CAN_ID; Byte1~2:角度; Byte3+Byte4[7-4]:速度;
 *                 Byte4[3-0]+Byte5:力矩; Byte6:模式/故障/预警; Byte6[3-0]+Byte7:温度
 */
void RobostrideFdbData(RobostrideMeasure_s *robostride_measure, uint8_t rx_data[8])
{
    robostride_measure->motor_id = rx_data[0];

    uint16_t pos_int = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    uint16_t vel_int = ((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4);
    uint16_t tor_int = ((uint16_t)(rx_data[4] & 0x0F) << 8) | rx_data[5];
    uint16_t temp_int = ((uint16_t)(rx_data[6] & 0x0F) << 8) | rx_data[7];

    robostride_measure->pos = uint_to_float(pos_int, ROBOSTRIDE_P_MIN, ROBOSTRIDE_P_MAX, 16);
    robostride_measure->vel = uint_to_float(vel_int, ROBOSTRIDE_V_MIN, ROBOSTRIDE_V_MAX, 12);
    robostride_measure->tor = uint_to_float(tor_int, ROBOSTRIDE_T_MIN, ROBOSTRIDE_T_MAX, 12);
    robostride_measure->temp = (float)temp_int / 10.0f;

    robostride_measure->mode_state = (rx_data[6] >> 6) & 0x03;
    robostride_measure->fault = (rx_data[6] >> 5) & 0x01;
    robostride_measure->warning = (rx_data[6] >> 4) & 0x01;

    robostride_measure->last_fdb_time = HAL_GetTick();
}
