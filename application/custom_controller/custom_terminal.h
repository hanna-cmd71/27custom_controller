/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_terminal.c/h
  * @brief      自定义终端（J0~J5 六轴机械臂，J1 为 RS00，其余为张大头闭环步进）。
  * @note       只依赖 CAN 驱动与通信模块，不依赖云台/遥控器/底盘框架。
  *             机构模型见 terminal_kinematics.h，重力模型见 terminal_gravity.h，
  *             调试快照见 terminal_debug.h。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成框架搭建。
  *  V1.1.0     Oct-26-2026     hanner             1. 模块解耦，删掉云台模板残留；
  *                                                2. 关节参数全部改由 robot_param 提供；
  *                                                3. 反馈统一成机械角 + 零位 + 减速比。
  *  V1.2.0     Oct-26-2026     hanner             1. 解算与重力补偿拆成独立模块；
  *                                                2. 原 custom_controller.c/h 并入本模块。
  @verbatim
  ==============================================================================
    数据流（每 1 ms，由 custom_controller_task 驱动）：
      Observer   读 CAN 驱动缓存 → 换算关节角 → 正解 → 填上行数据
      Reference  由关节角算重力前馈力矩
      Console    标定/测试模式：手动给出单关节力矩
      SendCmd    把力矩换算成各自的协议帧发出去
    上行（每 34 ms）：CustomControllerSendToPC → 0x0302 帧 → huart1 + huart6
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef CUSTOM_TERMINAL_H
#define CUSTOM_TERMINAL_H

#include "robot_param.h"
#include "struct_typedef.h"
#include "stdbool.h"
#include "terminal_debug.h"
#include "terminal_kinematics.h"

/* 关节数与全部机械参数由 robot_param_terminal.h 统一给出 */

/*-------------------- 任务节拍 --------------------
 * 自定义终端是 1 ms 力矩环；未启用时沿用旧调试节拍，避免空转占 CPU。
 */
#define CUSTOM_CONTROLLER_TASK_INIT_TIME 100           // 任务启动后等待时间 (ms)
#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_TERMINAL)
#define CUSTOM_CONTROLLER_CONTROL_TIME 1               // 控制周期 (ms)：力矩环
#else
#define CUSTOM_CONTROLLER_CONTROL_TIME 1000            // 控制周期 (ms)：调试环
#endif
#define CUSTOM_CONTROLLER_SEND_TIME 34                 // 上行周期 (ms)：30 Hz

/* RS00 补发使能的节拍 (ms)。使能命令"发一次就一直有效"，但开机那一帧常常比电机
 * 自检早，丢了就只有断电才能重来；这里在"确实没再回帧"时按这个周期重发一次。
 * 电机正常回帧时不会触发，对运行零影响。 */
#define CUSTOM_TERMINAL_RS_REENABLE_MS 500U

/*-------------------- 上行协议：0x0302 自定义控制器数据帧 --------------------
 * 整帧 39 字节 = 帧头(5) + 命令码(2) + 数据段(30) + 帧尾(2)，
 */
#define CUSTOM_TERMINAL_UPLINK_SOF ((uint8_t)0xA5U)        // 帧起始字节
#define CUSTOM_TERMINAL_UPLINK_CMD_ID ((uint16_t)0x0302U)  // 命令码：自定义控制器数据
#define CUSTOM_TERMINAL_UPLINK_DATA_LEN ((uint16_t)30U)    // 数据段长度（字节）
#define CUSTOM_TERMINAL_UPLINK_HEADER_LEN ((uint8_t)5U)    // 帧头长度（字节）
#define CUSTOM_TERMINAL_UPLINK_PORT_NUM 2U                  // 上行口数量（huart1 + huart6）

/*-------------------- 标定 / 电机测试模式 --------------------
 * 没有控制律时也能对单个关节施加可控力矩，用于标定转向、零位与力标度。
 * 输入全部来自 Keil Watch 里可写的 terminal_dbg.manual_*：
 *   manual_en               写 1 进入手动模式（写 0 按斜坡回零并失能电机）
 *   manual_joint            选中关节 0~5（1 = J1，RS00）
 *   manual_torque_nm        给定力矩 (N·m)：ZDT 按 MA_PER_NM 换算成 mA，RS00 直接使用
 *   manual_ignore_feedback  写 1 时取消"无反馈不出力"保护（临时诊断用，务必先托住）
 * 安全：力矩有硬上限、每 1 ms 限速、解除时显式失能（ZDT 发 F3 AB 00 00 6B）。
 * 标定完成后把 CUSTOM_TERMINAL_TEST_MODE 置 0 再编译。
 */
#define CUSTOM_TERMINAL_TEST_MODE 1
#define CUSTOM_TERMINAL_TEST_NM_RAMP 0.02f  // 每 1 ms 允许的力矩变化量 (N·m)

/*-------------------- 上行数据 --------------------
 * 自定义控制器上行数据段：7 个 float，前 N 个为关节角，其余预留。
 */
#ifndef CC_CONTROL_DATA_T_DEFINED
#define CC_CONTROL_DATA_T_DEFINED
typedef struct
{
    float pos[7];
} CCControlData_t;
#endif

extern CCControlData_t cc_control_data;

/**
 * @brief 自定义终端运行状态
 * @note  只保留真正被读写的量，参数与零位在 robot_param 里。
 */
typedef struct
{
    float joint_feedback[CUSTOM_TERMINAL_JOINT_NUM];       // 关节角 (rad)
    float joint_raw[CUSTOM_TERMINAL_JOINT_NUM];            // 电机侧原始机械角 (rad)
    float joint_velocity[CUSTOM_TERMINAL_JOINT_NUM];       // 关节角速度 (rad/s)
    float joint_torque[CUSTOM_TERMINAL_JOINT_NUM];         // 实际下发力矩 (N·m)
    float joint_reference[CUSTOM_TERMINAL_JOINT_NUM];      // 目标关节角 (rad)，留给后续控制律
    uint32_t joint_last_feedback_time[CUSTOM_TERMINAL_JOINT_NUM];
    uint16_t joint_last_ecd[CUSTOM_TERMINAL_JOINT_NUM];    // 原始编码器值
    bool joint_feedback_valid[CUSTOM_TERMINAL_JOINT_NUM];  // 反馈在超时窗口内为 true
} custom_terminal_t;

extern custom_terminal_t custom_terminal;
extern FK_Result_t custom_terminal_fk;                       // 末端位姿
extern FK_Result_t joint_pose[CUSTOM_TERMINAL_JOINT_NUM];    // 各关节位姿

/*-------------------- 调试快照 --------------------
 * 定义在 terminal_debug.c，类型在 terminal_debug.h（与业务代码分开，方便整体摘除）。
 */

/*-------------------- Public --------------------*/

extern void CustomControllerPublish(void);
extern void CustomControllerInit(void);
extern void CustomControllerSetMode(void);
extern void CustomControllerObserver(void);
extern void CustomControllerReference(void);
extern void CustomControllerConsole(void);
extern void CustomControllerSendCmd(void);

/**
 * @brief          发一帧 0x0302 给上位机（huart1 与 huart6 各一份）
 * @note           由 custom_controller_task 每 CUSTOM_CONTROLLER_SEND_TIME 调用一次。
 */
extern void CustomControllerSendToPC(void);

/**
 * @brief          获取全部关节角度数组指针 (rad)
 * @note           返回 joint_feedback 首地址，指向实时更新的数据，外部只读。
 */
extern const float *custom_terminal_get_joint_feedback(void);

#endif  // CUSTOM_TERMINAL_H

/************************ END OF FILE ************************/
