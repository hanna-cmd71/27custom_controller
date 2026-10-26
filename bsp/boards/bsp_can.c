#include "bsp_can.h"

#include "string.h"

/* 等待空闲发送邮箱的最大轮询次数，等价于原实现的 cnt = 20 */
#define CAN_TX_MAILBOX_RETRY_CNT ((uint8_t)20)

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

hcan_t * can_get_handle(uint8_t can)
{
    if (can == 1U)
    {
        return &hcan1;
    }
    else if (can == 2U)
    {
        return &hcan2;
    }
    return NULL;
}

/**
 * @brief          发送控制数据
 * @param[in]      can_ctrl_data 句柄 + 帧头 + 数据
 * @return         CAN_SEND_OK 成功，CAN_SEND_FAIL 失败
 * @note           与 v1.2 的 fdcanx_send_data 一致：把发送结果返回给调用者，
 *                 不再把 HAL 状态塞进文件级全局变量里（原实现 a3 无人读取）
 */
uint8_t CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
{
    uint32_t send_mail_box = 0;
    uint8_t cnt = CAN_TX_MAILBOX_RETRY_CNT;
    uint32_t free_TxMailbox;

    if (can_ctrl_data == NULL || can_ctrl_data->hcan == NULL)
    {
        return CAN_SEND_FAIL;
    }

    free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
    while (free_TxMailbox < 3U && cnt-- > 0U)
    {
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
    }

    if (HAL_CAN_AddTxMessage(
            can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data,
            &send_mail_box) != HAL_OK)
    {
        return CAN_SEND_FAIL;
    }
    return CAN_SEND_OK;
}

uint8_t can_send_std(hcan_t * hcan, uint16_t std_id, const uint8_t * data, uint8_t len)
{
    CanCtrlData_s ctrl;

    if (hcan == NULL || data == NULL || len == 0U || len > 8U)
    {
        return CAN_SEND_FAIL;
    }

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.hcan = hcan;
    ctrl.tx_header.IDE = CAN_ID_STD;
    ctrl.tx_header.RTR = CAN_RTR_DATA;
    ctrl.tx_header.StdId = std_id;
    ctrl.tx_header.DLC = len;
    memcpy(ctrl.tx_data, data, len);

    return CAN_SendTxMessage(&ctrl);
}

uint8_t can_send_ext(hcan_t * hcan, uint32_t ext_id, const uint8_t * data, uint8_t len)
{
    CanCtrlData_s ctrl;

    if (hcan == NULL || data == NULL || len == 0U || len > 8U)
    {
        return CAN_SEND_FAIL;
    }

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.hcan = hcan;
    ctrl.tx_header.IDE = CAN_ID_EXT;
    ctrl.tx_header.RTR = CAN_RTR_DATA;
    ctrl.tx_header.ExtId = ext_id;
    ctrl.tx_header.DLC = len;
    memcpy(ctrl.tx_data, data, len);

    return CAN_SendTxMessage(&ctrl);
}
