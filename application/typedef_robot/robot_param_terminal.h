/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param_terminal.h
  * @brief      自定义终端（C 板）的机械臂参数。
  * @note       与 robot_param_omni_mini.h 等"整车参数"文件同级、互相独立：
  *             本文件只描述 C 板上这台六自由度机构，换整车配置时不需要改这里。
  *             由 robot_param.h 统一 include。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     hanner          1. 自定义终端参数统一管理
  @verbatim
  ==============================================================================
    除 CAN 口与关节数外，其余全部是【待实测】的占位值。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef ROBOT_PARAM_TERMINAL_H
#define ROBOT_PARAM_TERMINAL_H

#include "robot_typedef.h"
#include "struct_typedef.h"

/*-------------------- 基本配置 --------------------*/

#define CUSTOM_TERMINAL_CAN              (1)   //can序号
#define CUSTOM_TERMINAL_JOINT_NUM        (6)   // 关节数 J0~J5
#define CUSTOM_TERMINAL_RS_JOINT         (1)   // J1 由 Robostride RS00 驱动

/* ZDT 地址：扩展 ID = (Addr << 8) | Packet，0 表示该关节不是 ZDT。*/
#define CUSTOM_TERMINAL_ZD_ADDR          {1U, 0U, 3U, 4U, 5U, 6U}

/*  RS00 CAN_ID 。 */
#define CUSTOM_TERMINAL_RS_ID            (0x05U)
#define CUSTOM_TERMINAL_RS_MASTER_ID     (0x15U)


/* 零位（电机侧机械角，rad)*/
#define CUSTOM_TERMINAL_JOINT_ZERO       {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}

/* 方向：+1 与机械正方向一致，-1 相反 */
#define CUSTOM_TERMINAL_JOINT_DIR        {1, 1, 1, 1, 1, 1}

/* 减速比：电机端转角 / 关节端转角（直驱填 1） */
#define CUSTOM_TERMINAL_JOINT_GEAR       {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}

/* 力矩标度：ZDT 用电流控制，需 N·m → mA 系数（【待标定】先按 5000 mA / 6.25 N·m = 800）。
 * J1 是 RS00，直接给 N·m，这一项填 0 表示不参与换算。 */
#define CUSTOM_TERMINAL_JOINT_MA_PER_NM  {800.0f, 0.0f, 800.0f, 800.0f, 800.0f, 800.0f}

/*-------------------- 保护限值 --------------------*/

/* ZDT 走 mA（手册相电流满量程 5000），RS00 走 N·m（峰值 14） */
#define CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA (5000)
#define CUSTOM_TERMINAL_RS_TORQUE_MAX_NM (14.0f)

/* 反馈超时：超过这个时间没有新帧就认为该关节离线，力矩强制 0 */
#define CUSTOM_TERMINAL_FEEDBACK_TIMEOUT_MS (20U)

/* 定时上报配置的自愈重发周期：每秒检查一次，对还没上报的地址补发一次 */
#define CUSTOM_TERMINAL_REPORT_RETRY_MS  (1000U)

/* 定时上报的信息码：0x31 = 单圈编码器值(0~65535)，0x36 = 实时位置(0.1°, 带符号、可多圈)*/
#define CUSTOM_TERMINAL_ZD_REPORT_PARAM  (0x31U)

/* -------------------- 连杆参数（重力补偿用） --------------------
 * LINK_MASS[i] = 关节 i 之后那一段连杆的质量 (kg)
 * LINK_COM[i]  = 该段质心在「关节 i 的 DH 坐标系」下的位置 (mm)
 * 注意：不是 URDF 的 Link_i 坐标系。两者差一个常量变换（见 doc/dh_from_urdf.md §4），
 *       所以别把 URDF 的 inertial origin 直接抄进来。
 * 数值来源 = E:\6axis\custom_controller\urdf\custom_controller.urdf 的 CAD 惯性参数，
 *       是「按体积×密度算出来的」，不是称出来的；用之前先按 doc/dh_from_urdf.md §5 复核。
 */
#define CUSTOM_TERMINAL_LINK_MASS        {0.195203f, 0.103890f, 0.136535f, 0.085791f, 0.107672f, 0.026376f}
#define CUSTOM_TERMINAL_LINK_COM         {{-0.4854f, -12.1582f, -60.2141f},  \
                                          {-33.0643f, 0.2736f, -52.7060f},   \
                                          {-0.8378f, 4.6719f, 21.9142f},     \
                                          {-0.4521f, -7.1257f, 0.9729f},     \
                                          {0.5549f, 19.6106f, -52.9501f},    \
                                          {-0.1092f, -0.6219f, -124.9440f}}

/* 重力补偿投入比例：0 = 关闭。
 * 打开的前提是 DIR/GEAR/ZERO 已标完（重力力矩的符号依赖 DIR），
 * 且 LINK_MASS/LINK_COM 已复核；打开时从 0.2 起逐级加到 1.0，每档都要看方向。 */
#define CUSTOM_TERMINAL_GRAVITY_SCALE    (0.0f)

#endif /* ROBOT_PARAM_TERMINAL_H */

/************************ END OF FILE ************************/
