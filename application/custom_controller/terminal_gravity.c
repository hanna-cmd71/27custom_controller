/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       terminal_gravity.c
  * @brief      自定义终端的重力补偿实现。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     -               1. 从 custom_terminal.c 拆出重力补偿。
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "terminal_gravity.h"
#include "terminal_kinematics.h"

#include <math.h>
#include <string.h>

/* 重力模型：各连杆质量 (kg) 与质心（本连杆坐标系下，mm） */
static const float ct_link_mass[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_LINK_MASS;
static const float ct_link_com[CUSTOM_TERMINAL_JOINT_NUM][3] = CUSTOM_TERMINAL_LINK_COM;

/*
 * 关节 j 的转轴方向（基座坐标系）：把 axis[j] 用 T0_{j-1} 旋转到基座系再归一化。
 * j=0 时上一个坐标系就是基座，用 base_transform 的旋转部分。
 */
static void LinkAxis(uint8_t j, float T[CUSTOM_TERMINAL_JOINT_NUM][4][4], float z[3])
{
    const DH_Param_t *dh = &arm_dh_param[j];
    const float norm = sqrtf(dh->axis[0] * dh->axis[0] + dh->axis[1] * dh->axis[1] +
                             dh->axis[2] * dh->axis[2]);

    if (norm < 1e-6f)
    {
        z[0] = 0.0f;
        z[1] = 0.0f;
        z[2] = 1.0f;
        return;
    }

    if (j == 0U)
    {
        for (uint8_t k = 0; k < 3; k++)
        {
            z[k] = (base_transform[k][0] * dh->axis[0] + base_transform[k][1] * dh->axis[1] +
                    base_transform[k][2] * dh->axis[2]) / norm;
        }
        return;
    }

    for (uint8_t k = 0; k < 3; k++)
    {
        z[k] = (T[j - 1U][k][0] * dh->axis[0] + T[j - 1U][k][1] * dh->axis[1] +
                T[j - 1U][k][2] * dh->axis[2]) / norm;
    }
}

/**
 * @brief 重力补偿：算出各关节"抵消重力"所需的前馈力矩 (N·m)
 * @note  输出是**要下发给电机的力矩**，符号已经取反：电机力矩 = −重力力矩。
 *        前提是 JOINT_DIR 已经把"正力矩 → 绕 DH z 轴正向转"定下来（见 S2 标定），
 *        所以必须在 DIR/GEAR/ZERO 标完之后才允许把 GRAVITY_SCALE 放到非 0。
 * @note  量纲：坐标 mm、力 N，tau 累加出来是 N·mm，末尾 /1000 变成 N·m。
 */
void terminal_gravity(const float q[CUSTOM_TERMINAL_JOINT_NUM],
                      float tau_nm[CUSTOM_TERMINAL_JOINT_NUM])
{
    float T[CUSTOM_TERMINAL_JOINT_NUM][4][4]; /* T0_i：第 i 个坐标系相对基座 */
    float acc[4][4];
    float link[4][4];
    float tmp[4][4];

    /* 1) 累积正解链，保存每个坐标系 */
    memcpy(acc, base_transform, sizeof(acc));
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        terminal_dh_transform(&arm_dh_param[i], q[i], link);
        terminal_mat4_mul(acc, link, tmp);
        memcpy(acc, tmp, sizeof(acc));
        memcpy(T[i], acc, sizeof(acc));
    }

    /* 2) 各连杆质心（基座坐标系，mm） */
    float pc[CUSTOM_TERMINAL_JOINT_NUM][3];

    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        const float *com = ct_link_com[i];

        for (uint8_t k = 0; k < 3; k++)
        {
            pc[i][k] = T[i][k][0] * com[0] + T[i][k][1] * com[1] + T[i][k][2] * com[2] + T[i][k][3];
        }
    }

    /* 3) 逐关节累加：关节 j 的轴过 p[j]（= T0_{j-1} 原点），方向 z[j] */
    for (uint8_t j = 0; j < CUSTOM_TERMINAL_JOINT_NUM; j++)
    {
        float z[3];

        LinkAxis(j, T, z);

        /* 关节轴上的一点：T0_{j-1} 的原点；j=0 时是基座原点在基座系里的位置，
         * 也就是 base_transform 的平移（J1 轴过 (0.4, 0, 64) mm，不是 (0,0,0)） */
        float p[3] = {base_transform[0][3], base_transform[1][3], base_transform[2][3]};

        if (j > 0U)
        {
            p[0] = T[j - 1U][0][3];
            p[1] = T[j - 1U][1][3];
            p[2] = T[j - 1U][2][3];
        }

        float tau = 0.0f;

        for (uint8_t i = j; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
        {
            /* 该连杆重力 F = (0, 0, -m*g)，单位 N；力臂 r 用 mm，
             * 于是 tau 的量纲是 N·mm，最后 /1000 得到 N·m。
             * （这里原来写的是 -m*GRAVITY*1000，把力换成了 mN，导致结果大 1000 倍） */
            const float fx = 0.0f;
            const float fy = 0.0f;
            const float fz = -ct_link_mass[i] * GRAVITY;

            /* 力臂 r = pc[i] - p[j] */
            const float rx = pc[i][0] - p[0];
            const float ry = pc[i][1] - p[1];
            const float rz = pc[i][2] - p[2];

            /* 力矩 M = r × F，投影到关节轴 */
            const float mx = ry * fz - rz * fy;
            const float my = rz * fx - rx * fz;
            const float mz = rx * fy - ry * fx;

            tau += mx * z[0] + my * z[1] + mz * z[2];
        }

        tau_nm[j] = -tau / 1000.0f; /* N·mm → N·m，并取反成"电机要出的力矩" */
    }
}

/************************ END OF FILE ************************/
