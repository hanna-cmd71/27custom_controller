/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_SupCap.c/h
  * @brief      CAN发送函数，通过CAN信号控制超级电容
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-29-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_SupCap.h"

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 2,
};

/**
 * @brief          通过CAN发送超电板控制指令
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      power 超电控制功率
 * @return         none
 */
void CanCmdSupCap(uint8_t can, int16_t power)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = 0x210;

    CAN_CTRL_DATA.tx_data[0] = (power >> 8);
    CAN_CTRL_DATA.tx_data[1] = power;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/************************ END OF FILE ************************/
