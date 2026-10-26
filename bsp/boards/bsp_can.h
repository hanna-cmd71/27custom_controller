#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define BOARD_DATA_ANY     ((uint16_t)0x500)
#define BOARD_DATA_UINT16  ((uint16_t)0x600)
// clang-format on

typedef CAN_HandleTypeDef hcan_t;

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;

extern hcan_t hcan1;
extern hcan_t hcan2;

/* 发送返回值，参考 v1.2 fdcanx_send_data 的 0-成功 / 1-失败 约定 */
#define CAN_SEND_OK   ((uint8_t)0)
#define CAN_SEND_FAIL ((uint8_t)1)

extern void can_filter_init(void);

/**
 * @brief          发送一帧（帧头由调用者填好）
 * @param[in]      can_ctrl_data 句柄 + 帧头 + 数据
 * @return         CAN_SEND_OK 成功，CAN_SEND_FAIL 句柄为空/邮箱忙/发送失败
 */
extern uint8_t CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data);

/**
 * @brief          按 can 号取 HAL 句柄
 * @param[in]      can 1-CAN1，2-CAN2
 * @return         hcan_t* 句柄，非法 can 号返回 NULL
 */
extern hcan_t * can_get_handle(uint8_t can);

/**
 * @brief          发送标准帧，帧头由本函数组装
 * @param[in]      hcan 句柄
 * @param[in]      std_id 标准帧 ID
 * @param[in]      data 数据字节
 * @param[in]      len 数据长度(1~8)
 * @return         CAN_SEND_OK / CAN_SEND_FAIL
 */
extern uint8_t can_send_std(hcan_t * hcan, uint16_t std_id, const uint8_t * data, uint8_t len);

/**
 * @brief          发送扩展帧，帧头由本函数组装
 * @param[in]      hcan 句柄
 * @param[in]      ext_id 扩展帧 ID
 * @param[in]      data 数据字节
 * @param[in]      len 数据长度(1~8)
 * @return         CAN_SEND_OK / CAN_SEND_FAIL
 */
extern uint8_t can_send_ext(hcan_t * hcan, uint32_t ext_id, const uint8_t * data, uint8_t len);
#endif
