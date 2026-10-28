/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_robostride.c/h
  * @brief      RobStride 电机 CAN 驱动（MIT 协议，见 CAN_cmd_robostride.md）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-23-2026     李喆            1. 完成MIT模式控制。
  *  V1.1.0     Oct-25-2026     hanner               1. 按 v1.2 驱动风格重写：补全生命周期
  *                                                指令、发送结果返回、入参限幅。
  *
  @verbatim
  ==============================================================================
    RobStride MIT协议：CAN2.0标准帧，默认波特率1Mbps。
    StdId = 电机CAN_ID（bit10~8为模式，使能/控制/停止时为0）。
    反馈帧（应答指令1）使用主机 CAN_ID，Byte0 携带电机 CAN_ID。
    手册中的指令编号：
      指令1 使能   FF FF FF FF FF FF FF FC
      指令2 停止   FF FF FF FF FF FF FF FD
      指令4 设零   FF FF FF FF FF FF FF FE
      指令5 清错   FF FF FF FF FF FF FF FB
      指令6 设模式 FF FF FF FF FF FF <mode> FC
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_CMD_ROBOSTRIDE_H
#define CAN_CMD_ROBOSTRIDE_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

/* MIT模式各电机CAN_ID（可按实际配置修改） */
#define ROBOSTRIDE1_ID    0x01U
#define ROBOSTRIDE2_ID    0x02U
#define ROBOSTRIDE3_ID    0x03U
#define ROBOSTRIDE4_ID    0x04U
#define ROBOSTRIDE5_ID    0x05U
#define ROBOSTRIDE6_ID    0x06U

#define ROBOSTRIDE_ID_NUM 6U

/* MIT反馈帧标识（应答指令1）使用主机CAN_ID，Byte0携带电机CAN_ID */
#define ROBOSTRIDE_MASTER_ID    0x15

/* 指令6 设置运行模式，mode 取值见手册 */
#define ROBOSTRIDE_MODE_MOTION  0x00U   // 运控模式（MIT）
#define ROBOSTRIDE_MODE_POS     0x01U   // 位置模式
#define ROBOSTRIDE_MODE_SPD     0x02U   // 速度模式

/* 指令尾字节 */
#define ROBOSTRIDE_CMD_TAIL_ENABLE     0xFCU
#define ROBOSTRIDE_CMD_TAIL_STOP       0xFDU
#define ROBOSTRIDE_CMD_TAIL_SET_ZERO   0xFEU
#define ROBOSTRIDE_CMD_TAIL_CLEAR_ERR  0xFBU

/* MIT模式参数范围（详见使用手册第6章） */
#define ROBOSTRIDE_P_MIN    (-12.57f)
#define ROBOSTRIDE_P_MAX    (12.57f)
#define ROBOSTRIDE_V_MIN    (-33.0f)
#define ROBOSTRIDE_V_MAX    (33.0f)
#define ROBOSTRIDE_KP_MIN   (0.0f)
#define ROBOSTRIDE_KP_MAX   (500.0f)
#define ROBOSTRIDE_KD_MIN   (0.0f)
#define ROBOSTRIDE_KD_MAX   (5.0f)
#define ROBOSTRIDE_T_MIN    (-14.0f)
#define ROBOSTRIDE_T_MAX    (14.0f)

typedef struct
{
    uint8_t motor_id;
    float pos;
    float vel;
    float tor;
    float temp;
    uint8_t mode_state;
    uint8_t fault;
    uint8_t warning;
    uint32_t last_fdb_time;
} RobostrideMeasure_s;

/**
 * @brief 总线诊断：装车时先看 rx_cnt 与 last_std_id
 * @note  rx_cnt 不涨而 uid_cnt 在涨 = 电机在回，但回的是"应答指令2（64 位 MCU 唯一标识）"，
 *        说明它只认了停止/改参数类指令，没进使能态；两个都不涨 = 一条帧都没收到。
 *        应答指令1 的帧 ID **是主机 id**（出厂参数 0x200b = 0，不一定是 0x15），
 *        所以不能拿主机 id 当门限，只按"模式类型=0 + Byte0 是否 1~6"区分两种应答。
 */
typedef struct
{
    uint32_t rx_any_cnt;          // 进过 robostride_on_rx 的帧数（门限之前），0 = 回调根本没被调用
    uint32_t rx_cnt;              // 已被本驱动认领的状态应答帧数（应答指令1）
    uint32_t uid_cnt;             // 应答指令2（64 位 UID）帧数，用于区分"没回"和"回了但不是状态帧"
    uint16_t last_std_id;         // 最近一帧标准帧 ID（应答指令1 时等于电机实际的主机 id）
    uint8_t  last_motor_id;       // 最近一帧 Byte0 携带的电机 CAN_ID
    uint8_t  motor_mask;          // bit i = 电机 CAN_ID(i+1) 回过帧
    uint32_t tx_cnt;              // 本驱动成功塞进邮箱的帧数（≈333/s）；不涨 = 连发都没发出去
} robostride_debug_t;

extern volatile robostride_debug_t robostride_dbg;

/* 生命周期指令 */
extern uint8_t robostride_enable(uint8_t can, uint16_t id);

extern uint8_t robostride_stop(uint8_t can, uint16_t id);

extern uint8_t robostride_set_zero(uint8_t can, uint16_t id);

extern uint8_t robostride_clear_fault(uint8_t can, uint16_t id);

extern uint8_t robostride_set_mode(uint8_t can, uint16_t id, uint8_t mode);

/* 运控指令 */
extern uint8_t robostride_mit_control(
    uint8_t can, uint16_t id, float pos, float vel, float kp, float kd, float torque);

extern uint8_t robostride_torque_control(uint8_t can, uint16_t id, float torque);

/* 反馈解码与查询 */
/**
 * @brief 接收一帧可能的应答帧（在 CAN 接收中断里调用）
 * @param can    1 / 2
 * @param std_id 标准帧 ID
 * @param rx_data 数据域
 * @param dlc    数据长度
 * @return 1 表示"这是本驱动的应答帧且已处理"，调用方应 return；
 *         0 表示不认识，交给后续板间通信等分支。
 * @note  存储、解码、诊断全部在驱动内部完成，调用方不接触字节布局。
 */
extern uint8_t robostride_on_rx(uint8_t can, uint16_t std_id, const uint8_t * rx_data, uint8_t dlc);

/* 取某台电机最近一次反馈（id 1~6）；参数非法返回 NULL */
extern const RobostrideMeasure_s * robostride_get_measure(uint8_t can, uint8_t id);

#endif  // CAN_CMD_ROBOSTRIDE_H

/************************ END OF FILE ************************/
