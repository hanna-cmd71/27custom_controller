/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       terminal_kinematics.c
  * @brief      自定义终端的机构模型与正运动学实现。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     -               1. 从 custom_terminal.c 拆出解算部分。
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "terminal_kinematics.h"

#include <math.h>
#include <string.h>

/**
 * @brief 基座安装变换矩阵：机械臂基座相对世界坐标系的姿态
 * @note  正解从这里起乘。默认单位阵；基座歪装时改这一张表即可，其余不用动。
 * @note  这里的数值不是"基座歪装"，而是 DH 起点 frame0 相对 URDF base_link 的平移：
 *        frame0 的原点取在 J1 轴线上（URDF 里 J1 原点 = (0.40233, 0, 64) mm），
 *        所以整条链的 x 偏 0.402 mm、z 偏 64 mm。数值单位是 **mm**，与 .a/.d 一致。
 *        别照抄 URDF 里的米制数（0.064 就是 0.064 mm，差 1000 倍）。见 doc/dh_from_urdf.md。
 */
float base_transform[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.4023300f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 64.0000000f},
    {0.0f, 0.0f, 0.0f, 1.0f},
};

/**
 * @brief 六轴机构 DH 参数表 (索引 0~5 对应 J0~J5)
 * @note  .a/.d 单位 mm，.alpha/.offset 单位 rad，.axis 为本关节转轴（本关节坐标系下）
 * @note  数值来源：E:\6axis\custom_controller\urdf\custom_controller.urdf（SolidWorks 导出）
 *        由 _check\urdf_dh.py 用"Z_i 与 Z_{i+1} 的公垂线"推出来，再与 URDF 正解对拍：
 *        300 组随机位姿下末端位置差 < 0.002 mm。推导过程见 doc/dh_from_urdf.md。
 * @note  alpha 统一 90°、axis 统一 (0,0,1)，转角的信息全部放在 .offset 里：
 *        这样 terminal_dh_transform 就退化成正统的 Rz(θ)Tz(d)Tx(a)Rx(α)。
 * @note  J0 的 offset = -π 是因为 DH 的 x 轴按右手规则 X1 = Z1×Z2 落在 -x 方向，
 *        与 URDF 的零位差半个圈；不是错误。
 */
DH_Param_t arm_dh_param[CUSTOM_TERMINAL_JOINT_NUM] = {
    /* J0 */ {.a = -0.8914f, .d = 47.0001f, .alpha = 1.570796f, .offset = -3.141593f,
              .axis = {0.0f, 0.0f, 1.0f}},
    /* J1 */ {.a = 99.9740f, .d = 0.0000f, .alpha = 0.000000f, .offset = 0.010012f,
              .axis = {0.0f, 0.0f, 1.0f}},
    /* J2 */ {.a = -0.8884f, .d = 14.1315f, .alpha = 1.570796f, .offset = -1.580804f,
              .axis = {0.0f, 0.0f, 1.0f}},
    /* J3 */ {.a = 0.3320f, .d = 95.6620f, .alpha = 1.570796f, .offset = 0.000000f,
              .axis = {0.0f, 0.0f, 1.0f}},
    /* J4 */ {.a = -1.0521f, .d = -5.2923f, .alpha = 1.570796f, .offset = 0.000000f,
              .axis = {0.0f, 0.0f, 1.0f}},
    /* J5 */ {.a = 0.0000f, .d = 0.0000f, .alpha = 0.000000f, .offset = 0.000000f,
              .axis = {0.0f, 0.0f, 1.0f}},
};

/**
 * @brief 角度循环限幅：把任意角度绕回 [-π, π)
 */
float terminal_wrap_angle(float angle)
{
    const float two_pi = 2.0f * (float)M_PI;

    angle = fmodf(angle, two_pi);
    if (angle >= (float)M_PI)
        angle -= two_pi;
    if (angle < -(float)M_PI)
        angle += two_pi;

    return angle;
}

/**
 * @brief 4x4 齐次变换矩阵相乘： out = left * right
 */
void terminal_mat4_mul(float left[4][4], float right[4][4], float out[4][4])
{
    for (uint8_t r = 0; r < 4; r++)
    {
        for (uint8_t c = 0; c < 4; c++)
        {
            out[r][c] = left[r][0] * right[0][c] +
                        left[r][1] * right[1][c] +
                        left[r][2] * right[2][c] +
                        left[r][3] * right[3][c];
        }
    }
}

/**
 * @brief 由单个关节的 DH 参数与关节角构造齐次变换矩阵
 * @note  T = R(axis, theta+offset) * Trans(a,0,d) * Rx(alpha)
 * @note  axis = (0,0,1) 时即 Rz(θ+offset)*Tz(d)*Tx(a)*Rx(α)，与 MDH 的乘积形式一致
 */
void terminal_dh_transform(const DH_Param_t *dh, float theta, float T[4][4])
{
    theta += dh->offset; /* DH 零位偏置：θ = q + offset */
    const float ct = cosf(theta);
    const float st = sinf(theta);
    const float ca = cosf(dh->alpha);
    const float sa = sinf(dh->alpha);

    /* 归一化关节转轴向量 */
    float kx = dh->axis[0], ky = dh->axis[1], kz = dh->axis[2];
    float norm = sqrtf(kx * kx + ky * ky + kz * kz);

    if (norm < 1e-6f)
    {
        kx = 0.0f;
        ky = 0.0f;
        kz = 1.0f;
    }
    else
    {
        kx /= norm;
        ky /= norm;
        kz /= norm;
    }

    /* 关节旋转：罗德里格斯公式，绕单位轴 k 转 theta；axis={0,0,1} 时退化为 Rz */
    const float vt = 1.0f - ct;
    float Rj[3][3];

    Rj[0][0] = ct + kx * kx * vt;
    Rj[0][1] = kx * ky * vt - kz * st;
    Rj[0][2] = kx * kz * vt + ky * st;
    Rj[1][0] = ky * kx * vt + kz * st;
    Rj[1][1] = ct + ky * ky * vt;
    Rj[1][2] = ky * kz * vt - kx * st;
    Rj[2][0] = kz * kx * vt - ky * st;
    Rj[2][1] = kz * ky * vt + kx * st;
    Rj[2][2] = ct + kz * kz * vt;

    /* 固定结构变换 T_fixed = Trans(a,0,d) * Rx(alpha)，最终 T = Rj * T_fixed */
    T[0][0] = Rj[0][0];
    T[0][1] = Rj[0][1] * ca + Rj[0][2] * sa;
    T[0][2] = -Rj[0][1] * sa + Rj[0][2] * ca;
    T[0][3] = Rj[0][0] * dh->a + Rj[0][2] * dh->d;

    T[1][0] = Rj[1][0];
    T[1][1] = Rj[1][1] * ca + Rj[1][2] * sa;
    T[1][2] = -Rj[1][1] * sa + Rj[1][2] * ca;
    T[1][3] = Rj[1][0] * dh->a + Rj[1][2] * dh->d;

    T[2][0] = Rj[2][0];
    T[2][1] = Rj[2][1] * ca + Rj[2][2] * sa;
    T[2][2] = -Rj[2][1] * sa + Rj[2][2] * ca;
    T[2][3] = Rj[2][0] * dh->a + Rj[2][2] * dh->d;

    T[3][0] = 0.0f;
    T[3][1] = 0.0f;
    T[3][2] = 0.0f;
    T[3][3] = 1.0f;
}

/**
 * @brief 从 4x4 齐次变换矩阵提取位姿
 * @note  姿态用最大分量法转四元数（无奇异），ZYX 欧拉角仅用于显示。
 */
static void PoseFromMat(float T[4][4], FK_Result_t *out)
{
    out->x = T[0][3];
    out->y = T[1][3];
    out->z = T[2][3];

    const float r11 = T[0][0];
    const float r12 = T[0][1];
    const float r13 = T[0][2];
    const float r21 = T[1][0];
    const float r22 = T[1][1];
    const float r23 = T[1][2];
    const float r31 = T[2][0];
    const float r32 = T[2][1];
    const float r33 = T[2][2];

    const float trace = r11 + r22 + r33;
    float qw, qx, qy, qz;

    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        qw = 0.25f * s;
        qx = (r32 - r23) / s;
        qy = (r13 - r31) / s;
        qz = (r21 - r12) / s;
    }
    else if (r11 > r22 && r11 > r33)
    {
        float s = sqrtf(1.0f + r11 - r22 - r33) * 2.0f;
        qw = (r32 - r23) / s;
        qx = 0.25f * s;
        qy = (r12 + r21) / s;
        qz = (r13 + r31) / s;
    }
    else if (r22 > r33)
    {
        float s = sqrtf(1.0f + r22 - r11 - r33) * 2.0f;
        qw = (r13 - r31) / s;
        qx = (r12 + r21) / s;
        qy = 0.25f * s;
        qz = (r23 + r32) / s;
    }
    else
    {
        float s = sqrtf(1.0f + r33 - r11 - r22) * 2.0f;
        qw = (r21 - r12) / s;
        qx = (r13 + r31) / s;
        qy = (r23 + r32) / s;
        qz = 0.25f * s;
    }

    float qn = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qn < 1e-9f)
        qn = 1.0f;
    if (qw < 0.0f)
        qn = -qn; /* 翻到 qw>=0 半球，保证同一姿态的四元数唯一 */

    out->qw = qw / qn;
    out->qx = qx / qn;
    out->qy = qy / qn;
    out->qz = qz / qn;

    /* ZYX 欧拉角：r31 = -sin(pitch)，接近 ±1 时进入万向锁 */
    if (r31 <= -1.0f + 1e-6f)
    {
        out->pitch = (float)M_PI * 0.5f;
        out->yaw = 0.0f;
        out->roll = atan2f(r12, r13);
    }
    else if (r31 >= 1.0f - 1e-6f)
    {
        out->pitch = -(float)M_PI * 0.5f;
        out->yaw = 0.0f;
        out->roll = atan2f(-r12, -r13);
    }
    else
    {
        out->pitch = atan2f(-r31, sqrtf(r32 * r32 + r33 * r33));
        out->yaw = atan2f(r21, r11);
        out->roll = atan2f(r32, r33);
    }
}

/*
 * @brief 逐关节正运动学求解：算出每个关节坐标系相对基座的位姿
 */
void FK_SolveAll(const float joints[CUSTOM_TERMINAL_JOINT_NUM],
                 FK_Result_t poses[CUSTOM_TERMINAL_JOINT_NUM])
{
    float acc[4][4];
    float link[4][4];
    float tmp[4][4];

    memcpy(acc, base_transform, sizeof(acc));

    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        terminal_dh_transform(&arm_dh_param[i], joints[i], link);
        terminal_mat4_mul(acc, link, tmp);
        memcpy(acc, tmp, sizeof(acc));
        PoseFromMat(acc, &poses[i]);
    }
}

/************************ END OF FILE ************************/
