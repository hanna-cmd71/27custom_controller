/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       terminal_kinematics.h
  * @brief      自定义终端（J0~J5 六轴机构）的机构模型与正运动学。
  * @note       本文件只描述"机构"：DH 表、基座安装、坐标变换、正解。
  *             不涉及电机、CAN、力矩、飞行数据，供重力补偿等模块共用。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     -               1. 从 custom_terminal.c 拆出解算部分。
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef TERMINAL_KINEMATICS_H
#define TERMINAL_KINEMATICS_H

#include "robot_param.h"
#include "struct_typedef.h"

/**
 * @brief 单关节机构参数
 * @note  .a/.d 单位 mm，.alpha 单位 rad，.axis 为本关节转轴（本关节坐标系下，不必是单位向量）
 */
typedef struct
{
    float a;        /* 连杆长度 (mm) */
    float d;        /* 连杆偏距 (mm) */
    float alpha;    /* 连杆扭角 (rad) */
    float axis[3];  /* 关节转轴方向 */
} DH_Param_t;

/**
 * @brief 单个关节坐标系的位姿
 * @note  roll/pitch/yaw 为 ZYX 欧拉角，仅用于显示；姿态唯一表示用四元数
 */
typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float qw;
    float qx;
    float qy;
    float qz;
} FK_Result_t;

/* DH 参数表（索引 0~5 对应 J0~J5），同一时刻只应有一个模块修改它 */
extern DH_Param_t arm_dh_param[CUSTOM_TERMINAL_JOINT_NUM];

/* 基座安装变换：机械臂基座相对世界坐标系的姿态，正解从这里起乘 */
extern float base_transform[4][4];

/**
 * @brief 角度循环限幅：把任意角度绕回 [-π, π)
 */
float terminal_wrap_angle(float angle);

/**
 * @brief 4x4 齐次变换矩阵相乘： out = left * right
 * @note  out 不可与 left/right 为同一数组
 */
void terminal_mat4_mul(float left[4][4], float right[4][4], float out[4][4]);

/**
 * @brief 由单个关节的 DH 参数与关节角构造齐次变换矩阵
 * @note  T = R(axis, theta) * Rx(alpha) * Trans(a, 0, d)
 */
void terminal_dh_transform(const DH_Param_t *dh, float theta, float T[4][4]);

/**
 * @brief 逐关节正运动学求解：算出每个关节坐标系相对基座的位姿
 * @param[in]  joints  关节角数组 (rad)
 * @param[out] poses   各关节坐标系位姿，poses[5] 即末端
 */
void FK_SolveAll(const float joints[CUSTOM_TERMINAL_JOINT_NUM],
                 FK_Result_t poses[CUSTOM_TERMINAL_JOINT_NUM]);

#endif /* TERMINAL_KINEMATICS_H */
