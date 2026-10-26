/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_zdatou.c/h
  * @brief      张大头（ZDT）步进闭环电机 CAN 驱动（SB 步兵自定义大头）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成。
  *  V1.1.0     Oct-25-2026     hanner          1. 按 v1.2 ZDT_X42_V2 驱动风格重写：
  *                                                帧字节具名、方向与幅值分离、发送结果返回。
  *  V1.2.0     Oct-26-2026     hanner          2. 按手册补齐设零/停止/模式/解堵命令，
  *                                                支持 0x31 与 0x36 两种上报，解码收进本驱动。
  *
  @verbatim
  ==============================================================================
    扩展帧，电机由 ExtId 区分：ExtId = (Addr << 8) | Packet，Packet 从 0 开始。
    数据域 = [功能码][命令数据...][校验码]，校验码默认固定 0x6B（也可配成 XOR/CRC8）。
    所以 0x100 = 地址 1 第 0 包，0x600 = 地址 6，与量产电机地址一一对应。
    协议来源：ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_ZDATOU_H
#define CAN_CMD_ZDATOU_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define ZDATOU1_EXT_ID    0x100U
#define ZDATOU2_EXT_ID    0x200U
#define ZDATOU3_EXT_ID    0x300U
#define ZDATOU4_EXT_ID    0x400U
#define ZDATOU5_EXT_ID    0x500U
#define ZDATOU6_EXT_ID    0x600U

/* 地址（扩展 ID 高字节）*/
#define ZDATOU_ADDR_TO_EXT_ID(addr)  ((uint32_t)(addr) << 8)

/* ---------------- 定时返回信息命令（6 字节） ----------------
 * 手册 5.5.1：让电机按固定周期主动返回某个信息，省掉主循环轮询。
 * 帧：11 18 <信息功能码> <定时ms 高> <定时ms 低> 6B
 * 例：让地址 1 每 10 ms 返回一次编码器值 → 11 18 31 00 0A 6B */
#define ZDATOU_REPORT_FUNC      0x11U   // [0] 定时返回信息功能码
#define ZDATOU_REPORT_AUX       0x18U   // [1] 辅助码
#define ZDATOU_PARAM_CPOS       0x31U   // [2] 信息功能码：线性化编码器值（单圈 0~65535 = 0~360°，手册 5.5.7）
#define ZDATOU_PARAM_RPOS       0x36U   // [2] 信息功能码：电机实时位置（带符号 32 位，0.1°，可多圈，手册 5.5.13）
#define ZDATOU_REPORT_PERIOD_HI 0x00U   // [3] 定时时间高 8 位
#define ZDATOU_REPORT_PERIOD_LO 0x0AU   // [4] 定时时间低 8 位 → 10 ms
#define ZDATOU_CHECKSUM         0x6BU   // [5] 校验码（默认固定 0x6B）

/* ---------------- 电机使能控制（5 字节） ----------------
 * 手册 5.3.2：F3 AB <使能状态> <同步标志> 6B
 * 注意：不使能时电机松轴；上电默认是使能状态 */
#define ZDATOU_ENABLE_FUNC      0xF3U   // [0] 功能码
#define ZDATOU_ENABLE_AUX       0xABU   // [1] 辅助码
#define ZDATOU_ENABLE_OFF       0x00U   // [2] 0 = 不使能（松轴）
#define ZDATOU_ENABLE_ON        0x01U   // [2] 1 = 使能（锁轴）
#define ZDATOU_SYNC_IMMEDIATE   0x00U   // [3] 0 = 立即执行（1 = 先缓存，等同步命令）
#define ZDATOU_SYNC_BUFFERED    0x01U   // [3] 1 = 缓存当前命令

/* ---------------- 立即停止（3 字节） ----------------
 * 手册 5.5.6：FE 98 6B。比"发 0 力矩"更硬：直接退出当前运动。 */
#define ZDATOU_STOP_FUNC        0xFEU
#define ZDATOU_STOP_AUX         0x98U

/* ---------------- 解除堵转保护（3 字节） ----------------
 * 手册 5.5.5：0E 52 6B。堵转保护触发后必须发这一条才能继续收力矩命令。 */
#define ZDATOU_CLEAR_STALL_FUNC 0x0EU
#define ZDATOU_CLEAR_STALL_AUX  0x52U

/* ---------------- 设置单圈零点（5 字节） ----------------
 * 手册 5.5.9：93 88 <01 = 把当前位置设为零点 / 00 = 恢复出厂零点> 6B
 * 写入驱动器、掉电保持；软件零位（robot_param 里的 ZERO）用于微调。 */
#define ZDATOU_SET_ZERO_FUNC    0x93U
#define ZDATOU_SET_ZERO_AUX     0x88U
#define ZDATOU_SET_ZERO_NOW     0x01U
#define ZDATOU_SET_ZERO_RESET   0x00U

/* ---------------- 修改控制模式（5 字节） ----------------
 * 手册 5.3.1：46 69 <是否保存 svF> <模式> 6B
 * 模式：0 = 脉冲/位置，1 = 力矩？—— 手册只给出"通用/位置/速度"表，力矩模式由 F5 帧进入。
 * 这里只提供"恢复默认通信/控制配置"的入口，具体 mode 值按现场手册填。 */
#define ZDATOU_MODE_FUNC        0x46U
#define ZDATOU_MODE_AUX         0x69U
#define ZDATOU_MODE_SAVE        0x01U   // [2] 1 = 保存到驱动器
#define ZDATOU_MODE_NOSAVE      0x00U   // [2] 0 = 只本次生效

/* ---------------- 力矩模式控制（8 字节，X 固件） ----------------
 * 手册 5.3.3：F5 <符号> <斜率高> <斜率低> <电流高> <电流低> <同步标志> 6B
 * 电流单位 mA，范围 0x0000~0x1388，即 0~5000 mA（用户手册的相电流满量程）。 */
#define ZDATOU_TORQUE_FUNC       0xF5U   // [0] 力矩模式功能码
#define ZDATOU_TORQUE_RAMP_HI    0xFFU   // [2] 力矩斜率(mA/s)高字节
#define ZDATOU_TORQUE_RAMP_LO    0xFFU   // [3] 力矩斜率(mA/s)低字节（0xFFFF = 最快）
#define ZDATOU_TORQUE_RESERVED   0x00U   // [6] 同步标志：0 = 立即执行

/* 方向字节：[1] 0x01 正转 / 0x00 反转 */
#define ZDATOU_DIR_POSITIVE     0x01U
#define ZDATOU_DIR_NEGATIVE     0x00U

/* 力矩幅值满量程 = 5000 mA（手册 5.3.3 的 0x1388）。 */
#define ZDATOU_TORQUE_CURRENT_MAX   ((uint16_t)5000U)
#define ZDATOU_TORQUE_RAW_MAX       ZDATOU_TORQUE_CURRENT_MAX

/* 0x31 编码器值 → 0.1° 单位（65536 计数 = 3600 个 0.1°） */
#define ZDATOU_ECD_TO_01DEG(ecd)    ((int32_t)(((uint32_t)(ecd) * 3600U) / 65536U))
/* 0.1° → rad */
#define ZDATOU_01DEG_TO_RAD         (0.0017453292519943296f)

typedef struct
{
    uint32_t ext_id;        // 最近一帧的扩展 ID
    uint16_t ecd;           // 0x31 单圈原始值（0x36 上报时保留上一次的值）
    int32_t  pos_deg;     // 统一位置量，0.1°：0x31 由 ecd 换算，0x36 直接来自电机
    uint8_t  kind;          // 最近一帧的信息码：ZDATOU_PARAM_CPOS / ZDATOU_PARAM_RPOS
    uint32_t last_fdb_time; // 最近一帧时间戳 (ms)
} zdatouMeasure_s;

/**
 * @brief 总线诊断：没有 CAN 分析仪时，靠它判断"哪台在回帧、回的什么"
 * @note  下标 0~5 恒对应地址 1~6（与关节号无关），装车后先看 ok_mask。
 */
typedef struct
{
    uint32_t rx_cnt;              // 已被本驱动认领的扩展帧总数
    uint8_t  last_addr;           // 最近一帧的地址
    uint8_t  last_packet;         // 最近一帧的 Packet
    uint8_t  last_func;           // 最近一帧的功能码
    uint32_t addr_cnt[6];         // 每个地址收到的帧数
    uint8_t  addr_func[6];        // 每个地址最近一帧的功能码（0x31/0x36/0x02/0xE2/0xEE/0x9F）
    uint8_t  addr_dlc[6];         // 每个地址最近一帧的 DLC
    uint32_t addr_extid[6];       // 每个地址最近一帧的完整扩展 ID
    uint8_t  addr_data[6][8];     // 每个地址最近一帧的原始数据
    uint8_t  ok_mask;             // bit i = 地址 i+1 最近一帧成功解析成位置
} zdatou_debug_t;

extern volatile zdatou_debug_t zdatou_dbg;

/* 配置定时上报（11 18 <param> 00 0A 6B） */
extern uint8_t zdatou_set_report(uint8_t can, uint32_t ext_id, uint8_t param);

/* 使能 / 失能 */
extern uint8_t zdatou_set_enable(uint8_t can, uint32_t ext_id, uint8_t enable);

/* 立即停止 */
extern uint8_t zdatou_stop(uint8_t can, uint32_t ext_id);

/* 解除堵转保护 */
extern uint8_t zdatou_clear_stall(uint8_t can, uint32_t ext_id);

/* 设置单圈零点：now = 把当前位置设为零点，否则恢复出厂零点 */
extern uint8_t zdatou_set_zero(uint8_t can, uint32_t ext_id, uint8_t now);

/* 力矩控制：方向 + 幅值（幅值 mA，超量程截断） */
extern uint8_t zdatou_torque_control(uint8_t can, uint32_t ext_id, uint8_t sign, uint16_t torque);

/* 力矩控制：有符号原始量，符号即方向 */
extern uint8_t zdatou_torque_control_i32(uint8_t can, uint32_t ext_id, int32_t torque);

/**
 * @brief 接收一帧扩展帧（在 CAN 接收中断里调用）
 * @param can    1 / 2，对应 CAN1 / CAN2
 * @param addr   电机地址（扩展 ID 高字节）
 * @param rx_data 数据域
 * @param dlc    数据长度
 * @return 1 表示"帧已被本驱动认领"，调用方应 return；0 表示不是本驱动的帧。
 * @note  存储与解码都由本驱动负责，调用方不再接触 measure 数组与字节布局；
 *        内部同时维护 zdatou_dbg（按地址分桶的总线诊断）。
 */
extern uint8_t zdatou_on_rx(uint8_t can, uint8_t addr, const uint8_t * rx_data, uint8_t dlc);

/* 取某台电机最近一次反馈（addr 1~6）；CAN 口或地址非法返回 NULL */
extern const zdatouMeasure_s * zdatou_get_measure(uint8_t can, uint8_t addr);

#endif  // CAN_CMD_ZDATOU_H

/************************ END OF FILE ************************/
