#include "CAN_cmd_SBzdatou.h"

#include "bsp_delay.h"

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_EXT,
    .tx_header.RTR = CAN_RTR_DATA,

};
HAL_StatusTypeDef a4;

void SBzdatou_ecd_get(uint8_t can, uint32_t ext_id)
{
    hcan_t *hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL)
        return;
    CAN_CTRL_DATA.tx_header.DLC = 6;
    CAN_CTRL_DATA.hcan = hcan;
    CAN_CTRL_DATA.tx_header.IDE = CAN_ID_EXT;
    CAN_CTRL_DATA.tx_header.ExtId = ext_id;
    CAN_CTRL_DATA.tx_data[0] = 0x11;
    CAN_CTRL_DATA.tx_data[1] = 0x18;
    CAN_CTRL_DATA.tx_data[2] = 0x31;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x0a;
    CAN_CTRL_DATA.tx_data[5] = 0x6b;
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}
/**
 * @brief          通过CAN扩展帧发送控制量
 * @param[in]      can 发送数据使用的can口
 * @param[in]      ext_id 发送数据使用的扩展帧ID
 * @param[in]      direction 方向
 * @param[in]      torque 扭矩控制量
 * @return         none
 */
void SBzdatou_torque_control(
    uint8_t can, uint32_t ext_id, uint8_t direction, int32_t torque)
{
    hcan_t *hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL)
        return;

    if (torque < 0)
    {
        torque = -torque;
        direction = 0x00;
    }
    else
    {
        direction = 0x01;
    }
    CAN_CTRL_DATA.tx_header.DLC = 8;

    CAN_CTRL_DATA.hcan = hcan;
    CAN_CTRL_DATA.tx_header.IDE = CAN_ID_EXT;
    CAN_CTRL_DATA.tx_header.ExtId = ext_id;

    CAN_CTRL_DATA.tx_data[0] = 0xf5;
    CAN_CTRL_DATA.tx_data[1] = direction;
    CAN_CTRL_DATA.tx_data[2] = 0xff;
    CAN_CTRL_DATA.tx_data[3] = 0xff;
    CAN_CTRL_DATA.tx_data[4] = (uint8_t)(torque >> 8);
    CAN_CTRL_DATA.tx_data[5] = (uint8_t)torque;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x6b;
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

void SBzdatouFdbData(SBzdatouMeasure_s *sbzdatou_measure, uint32_t ext_id, uint8_t rx_data[8])
{
    sbzdatou_measure->ext_id = ext_id;

    if (rx_data[0] == 0x31||rx_data[3]==0x6b)
    {
        sbzdatou_measure->ecd =
            ((uint16_t)rx_data[1] << 8) |
            ((uint16_t)rx_data[2]);
        sbzdatou_measure->last_fdb_time = HAL_GetTick();
    }
}
