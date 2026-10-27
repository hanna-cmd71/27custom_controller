#include "CAN_cmd_zdatou.h"

/* 反馈缓存：第一维 CAN 口，第二维地址（1~6 用下标 0~5） */
#define ZDATOU_CAN_NUM  2U
static zdatouMeasure_s zdatou_measure[ZDATOU_CAN_NUM][6];

volatile zdatou_debug_t zdatou_dbg;

/**
 * @brief          发送一帧 zdatou 命令
 * @param[in]      can 发送数据使用的can口
 * @param[in]      ext_id 发送数据使用的扩展帧ID
 * @param[in]      cmd 帧字节
 * @param[in]      len 帧长度
 * @return         CAN_SEND_OK 成功 / CAN_SEND_FAIL 失败
 */
static uint8_t zdatou_send(uint8_t can, uint32_t ext_id, const uint8_t * cmd, uint8_t len)
{
    return can_send_ext(can_get_handle(can), ext_id, cmd, len);
}

/**
 * @brief          配置定时上报（手册 5.5.1）
 * @param[in]      can 发送数据使用的can口
 * @param[in]      ext_id 目标电机扩展帧ID
 * @param[in]      param 信息功能码，取 ZDATOU_PARAM_CPOS / ZDATOU_PARAM_RPOS
 * @return         CAN_SEND_OK 成功 / CAN_SEND_FAIL 失败
 * @note           帧：11 18 <功能码> 00 0A 6B，即"每 10 ms 主动返回一次"。
 *                 发一次就一直有效，直到定时时间被改写为 0。
 */
uint8_t zdatou_set_report(uint8_t can, uint32_t ext_id, uint8_t param)
{
    uint8_t cmd[6] = {0};

    cmd[0] = ZDATOU_REPORT_FUNC;        // 定时返回信息功能码
    cmd[1] = ZDATOU_REPORT_AUX;         // 辅助码
    cmd[2] = param;                     // 信息功能码
    cmd[3] = ZDATOU_REPORT_PERIOD_HI;   // 定时时间高 8 位
    cmd[4] = ZDATOU_REPORT_PERIOD_LO;   // 定时时间低 8 位（10 ms）
    cmd[5] = ZDATOU_CHECKSUM;           // 校验字节

    return zdatou_send(can, ext_id, cmd, 6U);
}

/**
 * @brief          电机使能/失能（手册 5.3.2）
 * @param[in]      can 发送数据使用的can口
 * @param[in]      ext_id 目标电机扩展帧ID
 * @param[in]      enable ZDATOU_ENABLE_ON 使能(锁轴) / ZDATOU_ENABLE_OFF 失能(松轴)
 * @return         CAN_SEND_OK 成功 / CAN_SEND_FAIL 失败
 * @note           帧：F3 AB <状态> 00 6B。驱动器上电默认使能，不发失能就锁着；
 *                 只发 0 力矩帧不能让它松开。
 */
uint8_t zdatou_set_enable(uint8_t can, uint32_t ext_id, uint8_t enable)
{
    uint8_t cmd[5] = {0};

    cmd[0] = ZDATOU_ENABLE_FUNC;                                // 使能控制功能码
    cmd[1] = ZDATOU_ENABLE_AUX;                                 // 辅助码
    cmd[2] = (enable == ZDATOU_ENABLE_ON) ? ZDATOU_ENABLE_ON
                                          : ZDATOU_ENABLE_OFF;  // 使能状态
    cmd[3] = ZDATOU_SYNC_IMMEDIATE;                             // 同步标志：立即执行
    cmd[4] = ZDATOU_CHECKSUM;                                   // 校验字节

    return zdatou_send(can, ext_id, cmd, 5U);
}

/**
 * @brief          立即停止（手册 5.5.6）
 * @note           帧：FE 98 6B。比"发 0 力矩"更硬，直接退出当前运动。
 */
uint8_t zdatou_stop(uint8_t can, uint32_t ext_id)
{
    uint8_t cmd[3] = {0};

    cmd[0] = ZDATOU_STOP_FUNC;
    cmd[1] = ZDATOU_STOP_AUX;
    cmd[2] = ZDATOU_CHECKSUM;

    return zdatou_send(can, ext_id, cmd, 3U);
}

/**
 * @brief          解除堵转保护（手册 5.5.5）
 * @note           帧：0E 52 6B。堵转保护触发后必须发这一条才能继续收力矩命令。
 */
uint8_t zdatou_clear_stall(uint8_t can, uint32_t ext_id)
{
    uint8_t cmd[3] = {0};

    cmd[0] = ZDATOU_CLEAR_STALL_FUNC;
    cmd[1] = ZDATOU_CLEAR_STALL_AUX;
    cmd[2] = ZDATOU_CHECKSUM;

    return zdatou_send(can, ext_id, cmd, 3U);
}

/**
 * @brief          设置单圈零点（手册 5.5.9）
 * @param[in]      now 1 = 把当前位置设为零点；0 = 恢复出厂零点
 * @note           帧：93 88 <01/00> 6B。写入驱动器、掉电保持。
 *                 与 robot_param 里的软件零位配合：硬件零点负责粗对齐，软件零位负责微调。
 */
uint8_t zdatou_set_zero(uint8_t can, uint32_t ext_id, uint8_t now)
{
    uint8_t cmd[5] = {0};

    cmd[0] = ZDATOU_SET_ZERO_FUNC;
    cmd[1] = ZDATOU_SET_ZERO_AUX;
    cmd[2] = (now != 0U) ? ZDATOU_SET_ZERO_NOW : ZDATOU_SET_ZERO_RESET;
    cmd[3] = ZDATOU_SYNC_IMMEDIATE;
    cmd[4] = ZDATOU_CHECKSUM;

    return zdatou_send(can, ext_id, cmd, 5U);
}

/**
 * @brief          力矩控制（方向 + 幅值分离）
 * @param[in]      sign 方向，ZDATOU_DIR_POSITIVE / ZDATOU_DIR_NEGATIVE
 * @param[in]      torque 力矩幅值（0 ~ ZDATOU_TORQUE_RAW_MAX），超范围截断
 * @note           帧：F5 <符号> FF FF <电流高> <电流低> 00 6B
 */
uint8_t zdatou_torque_control(uint8_t can, uint32_t ext_id, uint8_t sign, uint16_t torque)
{
    uint8_t cmd[8] = {0};

    if (torque > ZDATOU_TORQUE_RAW_MAX)
    {
        torque = ZDATOU_TORQUE_RAW_MAX;
    }

    cmd[0] = ZDATOU_TORQUE_FUNC;                                    // 力矩模式功能码
    cmd[1] = (sign == ZDATOU_DIR_POSITIVE) ? ZDATOU_DIR_POSITIVE
                                           : ZDATOU_DIR_NEGATIVE;   // 方向
    cmd[2] = ZDATOU_TORQUE_RAMP_HI;                                 // 力矩斜率(高)
    cmd[3] = ZDATOU_TORQUE_RAMP_LO;                                 // 力矩斜率(低)
    cmd[4] = (uint8_t)(torque >> 8);                                // 力矩幅值(高8位)
    cmd[5] = (uint8_t)(torque >> 0);                                // 力矩幅值(低8位)
    cmd[6] = ZDATOU_TORQUE_RESERVED;                                // 同步标志：立即执行
    cmd[7] = ZDATOU_CHECKSUM;                                       // 校验字节

    return zdatou_send(can, ext_id, cmd, 8U);
}

/**
 * @brief          带符号力矩控制
 * @param[in]      torque 有符号力矩原始量 (mA)，符号即方向
 */
uint8_t zdatou_torque_control_i32(uint8_t can, uint32_t ext_id, int32_t torque)
{
    uint8_t sign = ZDATOU_DIR_POSITIVE;
    uint32_t magnitude;

    if (torque < 0)
    {
        /* (x+1) 再取负，避免 torque == INT32_MIN 时取反溢出 */
        sign = ZDATOU_DIR_NEGATIVE;
        magnitude = (uint32_t)(-(torque + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)torque;
    }

    if (magnitude > (uint32_t)ZDATOU_TORQUE_RAW_MAX)
    {
        magnitude = (uint32_t)ZDATOU_TORQUE_RAW_MAX;
    }

    return zdatou_torque_control(can, ext_id, sign, (uint16_t)magnitude);
}

/**
 * @brief          接收一帧本驱动的扩展帧
 * @note           手册：应答数据域 = [信息码][数据...][校验字节]。
 *                 0x31 编码器值：31 <高> <低> 6B，单圈 0~65535 = 0~360°
 *                 0x36 实时位置：36 <符号> <位置 4 字节> 6B，单位 0.1°，可多圈
 *                 其余信息码（02/E2/EE/9F）只记诊断，不改位置。
 */
uint8_t zdatou_on_rx(uint8_t can, uint8_t addr, const uint8_t * rx_data, uint8_t dlc)
{
    zdatouMeasure_s * measure;
    uint8_t bucket;

    /* 地址 0 或 7~255 不是本驱动的电机：这帧可能是别的设备，交回调用方。
     * 但要先把地址记下来：电机 ID 没设成 1~6 时，只有这条诊断能证明它在总线上。 */
    if ((addr == 0U) || (addr > 6U))
    {
        zdatou_dbg.last_unknown_addr = addr;
        zdatou_dbg.unknown_cnt++;
        return 0U;
    }
    bucket = (uint8_t)(addr - 1U);

    /* ---- 总线诊断：按地址分桶，装车时先看 ok_mask ---- */
    zdatou_dbg.rx_cnt++;
    zdatou_dbg.last_addr = addr;
    zdatou_dbg.last_packet = (uint8_t)(ZDATOU_ADDR_TO_EXT_ID(addr) & 0xFFU);
    zdatou_dbg.last_func = rx_data[0];
    zdatou_dbg.addr_cnt[bucket]++;
    zdatou_dbg.addr_func[bucket] = rx_data[0];
    zdatou_dbg.addr_dlc[bucket] = dlc;
    for (uint8_t k = 0U; k < 8U; k++)
    {
        zdatou_dbg.addr_data[bucket][k] = rx_data[k];
    }

    measure = &zdatou_measure[can - 1U][bucket];
    measure->ext_id = ZDATOU_ADDR_TO_EXT_ID(addr);
    zdatou_dbg.addr_extid[bucket] = measure->ext_id;

    switch (rx_data[0])
    {
    case ZDATOU_PARAM_CPOS: /* 31 <高> <低> [6B]，手册 5.5.7 的"5 字节"含地址字节 */
        /* 实测地址 1/6 的回复 DLC=3（31 高 低，无尾字节），有的固件带 6B（DLC=4）。
         * 两种都收，只有在尾字节存在时才校验，避免把正常帧误杀。 */
        if (dlc < 3U)
        {
            return 1U;
        }
        if ((dlc >= 4U) && (rx_data[dlc - 1U] != ZDATOU_CHECKSUM))
        {
            return 1U;
        }
        measure->ecd = ((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[2];
        measure->pos_deg = ZDATOU_ECD_TO_01DEG(measure->ecd);
        break;

    case ZDATOU_PARAM_RPOS: /* 36 <符号> <位置[4]> [6B]，手册 5.5.13 的"8 字节"含地址字节 */
    {
        /* 同上：6 字节（无尾字节）和 7 字节（带 6B）都收 */
        if (dlc < 6U)
        {
            return 1U;
        }
        if ((dlc >= 7U) && (rx_data[dlc - 1U] != ZDATOU_CHECKSUM))
        {
            return 1U;
        }
        int32_t pos = ((int32_t)(uint32_t)rx_data[2] << 24) |
                      ((int32_t)rx_data[3] << 16) |
                      ((int32_t)rx_data[4] << 8) |
                      (int32_t)rx_data[5];
        measure->pos_deg = (rx_data[1] != 0U) ? -pos : pos;
        break;
    }

    default: /* 02/E2/EE/9F 等：只记诊断，不改位置 */
        return 1U;
    }

    measure->kind = rx_data[0];
    measure->last_fdb_time = HAL_GetTick();
    zdatou_dbg.ok_mask |= (uint8_t)(1U << bucket);

    return 1U;
}

/**
 * @brief          取某台电机最近一次反馈
 */
const zdatouMeasure_s * zdatou_get_measure(uint8_t can, uint8_t addr)
{
    if ((addr == 0U) || (addr > 6U) || (can == 0U) || (can > ZDATOU_CAN_NUM))
    {
        return NULL;
    }
    return &zdatou_measure[can - 1U][addr - 1U];
}
