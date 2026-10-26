/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       terminal_gravity.h
  * @brief      自定义终端的重力补偿：由关节角算各关节需抵消的重力力矩。
  * @note       只依赖机构模型（terminal_kinematics），不依赖电机、CAN、任务框架，
  *             因此可以脱机用 PC 端复算、也可以被别的控制律直接调用。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     -               1. 从 custom_terminal.c 拆出重力补偿。
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef TERMINAL_GRAVITY_H
#define TERMINAL_GRAVITY_H

#include "robot_param.h"
#include "struct_typedef.h"

/**
 * @brief 重力补偿：算出各关节需要抵消重力所需的前馈力矩
 * @param[in]  q       关节角数组 (rad)
 * @param[out] tau_nm  各关节重力力矩 (N·m)，正值表示沿关节正方向
 * @note  方法同 v1.2 Gravity_comp.c：沿 DH 链求每根连杆质心位置，
 *        对每个关节 j 取"该关节轴以外所有连杆重力对该轴的力矩"之和。
 *        关节 i 的转轴方向 = 上一坐标系旋转后的 axis[i]，过点 p[i-1]。
 *        质量/质心来自 robot_param_terminal.h 的 CUSTOM_TERMINAL_LINK_*。
 */
void terminal_gravity(const float q[CUSTOM_TERMINAL_JOINT_NUM],
                      float tau_nm[CUSTOM_TERMINAL_JOINT_NUM]);

#endif /* TERMINAL_GRAVITY_H */
