/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       controler_myself.c/h
  * @brief      自定义云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     ?            1. 完成框架搭建。
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "controler_myself.h"
#include "data_exchange.h"
#include "communication.h"
#include "CAN_cmd_SBzdatou.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "string.h"

/*-------------------- Global var --------------------*/

Gimbal_s controller_myself;
Gimbal_PID_t controller_myself_pid;
FK_Result_t controller_myself_fk;                    // 正解结果（末端位姿，= joint_pose[5]）
FK_Result_t joint_pose[CONTROLLER_MYSELF_JOINT_NUM]; // 每个关节坐标系相对基座的位姿

/**
 * @brief 基座安装变换矩阵：机械臂基座相对世界坐标系的姿态
 * @note  正解从这里起乘（而非单位阵），用于修正基座实际安装方向。
 *        默认单位阵（基座与世界坐标系对齐）。若基座是歪着安装的，
 *        把此矩阵改成对应的旋转即可，正解链其余部分不用动。
 */
float base_transform[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
};

/**
 * @brief 六轴机械臂DH参数表 (索引0~5 对应关节1~6)
 * @note  【待填写】以下数值需按你的机械臂实际结构测量/查图纸填入：
 *          .a            = 连杆长度   (mm)
 *          .d            = 连杆偏距   (mm)
 *          .alpha        = 连杆扭转角 (rad)
 *          .theta_offset = 关节零位偏置(rad)
 */
DH_Param_t arm_dh_param[CONTROLLER_MYSELF_JOINT_NUM] = {
    /* 关节1 */ {.a = 0.0f, .d = 0.0f, .alpha = 0.0f, .theta_offset = -0.0f, .axis = {0.0f, 0.0f, 1.0f}},
    /* 关节2 */ {.a = 190.0f, .d = 0.0f, .alpha = 0.0f, .theta_offset = 0.0f, .axis = {0.0f, -1.0f, 0.0f}},
    /* 关节3 */ {.a = 87.0f, .d = 0.0f, .alpha = 0.0f, .theta_offset = -0.0f, .axis = {0.0f, 1.0f, 0.0f}},
    /* 关节4 */ {.a = 0.0f, .d = 0.0f, .alpha = 0.0f, .theta_offset = 0.0f, .axis = {1.0f, 0.0f, 0.0f}},
    /* 关节5 */ {.a = 150.5f, .d = 0.0f, .alpha = 0.0f, .theta_offset = -0.0f, .axis = {0.0f, 0.0f, -1.0f}},
    /* 关节6 */ {.a = 0.0f, .d = 0.0f, .alpha = 0.0f, .theta_offset = 0.0f, .axis = {1.0f, 0.0f, 0.0f}},
};

int a1;
int datouctrl[6];
int sendctrl;
SBzdatouMeasure_s *SBzdatouMeasure;
/*-------------------------------- Internal functions ---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 是否需要继续初始化
 */
bool ControllerMyselfInitJudge(void)
{
    return false;
}

/**
 * @brief          角度循环限幅：把任意角度绕回 [-π, π) 区间
 * @param[in]      angle 输入角度 (rad)
 * @retval         float 限幅后的角度 (rad)
 */
static float WrapAngle(float angle)
{
    const float two_pi = 2.0f * (float)M_PI;
    angle = fmodf(angle, two_pi); // 先收敛到 (-2π, 2π)
    if (angle >= (float)M_PI)
        angle -= two_pi;
    if (angle < -(float)M_PI)
        angle += two_pi;
    return angle;
}

/**
 * @brief          4x4 齐次变换矩阵相乘： out = left * right
 * @param[in]      left  左矩阵 [4][4]
 * @param[in]      right 右矩阵 [4][4]
 * @param[out]     out   结果矩阵 [4][4]（不可与 left/right 为同一数组）
 * @retval         none
 */
static void Mat4Mul(float left[4][4], float right[4][4], float out[4][4])
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
 * @brief          由单个关节的DH参数与关节角构造齐次变换矩阵
 * @note           标准DH约定： T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 * @param[in]      dh    该关节的DH参数
 * @param[in]      theta 该关节角度 (rad，已含零位偏置)
 * @param[out]     T     输出的 4x4 齐次变换矩阵
 * @retval         none
 */
static void DhTransform(const DH_Param_t *dh, float theta, float T[4][4])
{
    const float ct = cosf(theta);
    const float st = sinf(theta);
    const float ca = cosf(dh->alpha);
    const float sa = sinf(dh->alpha);

    // 归一化关节转轴向量；若未指定(全0)则默认绕Z轴，保持标准DH行为
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

    // 关节旋转：罗德里格斯公式，绕单位轴 k 转 theta。axis={0,0,1}时退化为Rz(theta)
    const float vt = 1.0f - ct; // versine
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

    // 固定结构变换 T_fixed = Tz(d)*Tx(a)*Rx(alpha)：旋转=Rx(alpha)，平移=(a,0,d)
    // 最终 T = Rj * T_fixed
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
 * @brief          从 4x4 齐次变换矩阵提取位姿(xyz + ZYX欧拉角)
 * @param[in]      T   齐次变换矩阵
 * @param[out]     out 位姿输出
 * @retval         none
 */
static void PoseFromMat(float T[4][4], FK_Result_t *out)
{
    // 末端位置：平移列
    out->x = T[0][3];
    out->y = T[1][3];
    out->z = T[2][3];

    // 旋转矩阵各元素
    const float r11 = T[0][0];
    const float r12 = T[0][1];
    const float r13 = T[0][2];
    const float r21 = T[1][0];
    const float r22 = T[1][1];
    const float r23 = T[1][2];
    const float r31 = T[2][0];
    const float r32 = T[2][1];
    const float r33 = T[2][2];

    // ---- 首选表示：四元数（无奇异点，用于比较/控制）----
    // 用最大分量法从旋转矩阵稳定地提取四元数，避免开根号时被0除。
    const float trace = r11 + r22 + r33;
    float qw, qx, qy, qz;
    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f) * 2.0f; // s = 4*qw
        qw = 0.25f * s;
        qx = (r32 - r23) / s;
        qy = (r13 - r31) / s;
        qz = (r21 - r12) / s;
    }
    else if (r11 > r22 && r11 > r33)
    {
        float s = sqrtf(1.0f + r11 - r22 - r33) * 2.0f; // s = 4*qx
        qw = (r32 - r23) / s;
        qx = 0.25f * s;
        qy = (r12 + r21) / s;
        qz = (r13 + r31) / s;
    }
    else if (r22 > r33)
    {
        float s = sqrtf(1.0f + r22 - r11 - r33) * 2.0f; // s = 4*qy
        qw = (r13 - r31) / s;
        qx = (r12 + r21) / s;
        qy = 0.25f * s;
        qz = (r23 + r32) / s;
    }
    else
    {
        float s = sqrtf(1.0f + r33 - r11 - r22) * 2.0f; // s = 4*qz
        qw = (r21 - r12) / s;
        qx = (r13 + r31) / s;
        qy = (r23 + r32) / s;
        qz = 0.25f * s;
    }
    // 归一化并统一符号(qw>=0)，保证同一姿态的四元数唯一，便于比较
    float qn = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qn < 1e-9f)
        qn = 1.0f;
    if (qw < 0.0f)
        qn = -qn; // 翻到 qw>=0 半球
    out->qw = qw / qn;
    out->qx = qx / qn;
    out->qy = qy / qn;
    out->qz = qz / qn;

    // ---- 仅显示用：ZYX 欧拉角 (yaw→pitch→roll)，会在 pitch=±90° 处跳变 ----
    // 奇异点(万向锁)判定： r31=-sin(pitch)，接近±1时 pitch≈∓90°，yaw 与 roll 耦合。
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

/**
 * @brief          两个姿态之间的旋转误差 (轴角表示，无奇异点)
 * @note           见头文件声明。
 */
float PoseAttitudeError(const FK_Result_t *a, const FK_Result_t *b, float axis_ang[3])
{
    if (a == NULL || b == NULL)
    {
        if (axis_ang != NULL)
        {
            axis_ang[0] = axis_ang[1] = axis_ang[2] = 0.0f;
        }
        return 0.0f;
    }

    // 相对旋转 q_err = q_a^{-1} * q_b  （q_a^{-1} = 共轭，单位四元数）
    const float aw = a->qw, ax = -a->qx, ay = -a->qy, az = -a->qz; // 共轭
    const float bw = b->qw, bx = b->qx, by = b->qy, bz = b->qz;

    float ew = aw * bw - ax * bx - ay * by - az * bz;
    float ex = aw * bx + ax * bw + ay * bz - az * by;
    float ey = aw * by - ax * bz + ay * bw + az * bx;
    float ez = aw * bz + ax * by - ay * bx + az * bw;

    // 取 qw>=0 半球，保证得到最短路径(0~π)
    if (ew < 0.0f)
    {
        ew = -ew;
        ex = -ex;
        ey = -ey;
        ez = -ez;
    }

    // 轴角： angle = 2*acos(ew)， 轴 = (ex,ey,ez)/sin(angle/2)
    if (ew > 1.0f)
        ew = 1.0f;
    const float angle = 2.0f * acosf(ew);
    const float sin_half = sqrtf(ex * ex + ey * ey + ez * ez);

    if (axis_ang != NULL)
    {
        if (sin_half < 1e-6f) // 几乎无旋转，轴任意
        {
            axis_ang[0] = axis_ang[1] = axis_ang[2] = 0.0f;
        }
        else
        {
            axis_ang[0] = ex / sin_half * angle;
            axis_ang[1] = ey / sin_half * angle;
            axis_ang[2] = ez / sin_half * angle;
        }
    }
    return angle;
}

/*------------------------- The end of internal functions --------------------------------------*/

/*-------------------- Public (inline) --------------------*/

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
    return 0.0f;
}

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
    return false;
}

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 (float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
    return 0.0f;
}

/*-------------------- Public --------------------*/

/**
 * @brief          发布数据
 * @param[in]      none
 * @retval         none
 */
void GimbalPublish(void)
{
}

/**
 * @brief          六轴机械臂正运动学求解
 * @note           链式相乘 6 个关节的DH变换矩阵得到末端相对基座的位姿，
 *                 再从 3x3 旋转矩阵反解 ZYX 欧拉角 (yaw-pitch-roll)。
 * @param[in]      joints 6个关节角度 (rad)，对应 controller_myself.joint_feedback
 * @param[out]     result 末端位姿 (xyz + roll/pitch/yaw)
 * @retval         none
 */
void FK_Solve(const float joints[CONTROLLER_MYSELF_JOINT_NUM], FK_Result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    // 累乘矩阵，从基座安装变换起乘
    float acc[4][4];
    float link[4][4];
    float tmp[4][4];
    memcpy(acc, base_transform, sizeof(acc));

    // T_end = T_base * T1 * T2 * ... * T6
    for (uint8_t i = 0; i < CONTROLLER_MYSELF_JOINT_NUM; i++)
    {
        const float theta = joints[i] + arm_dh_param[i].theta_offset;
        DhTransform(&arm_dh_param[i], theta, link);
        Mat4Mul(acc, link, tmp);
        memcpy(acc, tmp, sizeof(acc));
    }

    // 从末端变换矩阵提取位姿
    PoseFromMat(acc, result);
}

/**
 * @brief          逐关节正运动学求解：算出每个关节坐标系相对基座的位姿
 * @note           poses[i] 为关节 i+1 坐标系原点相对基座的位姿；
 *                 poses[5] 即末端位姿，与 FK_Solve 输出一致。
 * @param[in]      joints 6个关节角度 (rad)，对应 controller_myself.joint_feedback
 * @param[out]     poses  长度 CONTROLLER_MYSELF_JOINT_NUM 的位姿数组
 * @retval         none
 */
void FK_SolveAll(const float joints[CONTROLLER_MYSELF_JOINT_NUM],
                 FK_Result_t poses[CONTROLLER_MYSELF_JOINT_NUM])
{
    if (poses == NULL)
    {
        return;
    }

    // 累乘矩阵，从基座安装变换起乘
    float acc[4][4];
    float link[4][4];
    float tmp[4][4];
    memcpy(acc, base_transform, sizeof(acc));

    // 每乘完一个关节，就记录该关节坐标系相对基座的位姿
    for (uint8_t i = 0; i < CONTROLLER_MYSELF_JOINT_NUM; i++)
    {
        const float theta = joints[i] + arm_dh_param[i].theta_offset;
        DhTransform(&arm_dh_param[i], theta, link);
        Mat4Mul(acc, link, tmp);
        memcpy(acc, tmp, sizeof(acc));
        PoseFromMat(acc, &poses[i]);
    }
}

/**
 * @brief          读取单个关节的完整状态快照
 * @param[in]      index 关节索引 (0~5，对应关节1~6)
 * @param[out]     state 该关节的状态输出
 * @retval         bool  索引合法且写入成功返回 true，否则 false
 */
bool GetJointState(uint8_t index, JointState_t *state)
{
    if (state == NULL || index >= CONTROLLER_MYSELF_JOINT_NUM)
    {
        return false;
    }

    state->angle = controller_myself.joint_feedback[index];
    state->velocity = controller_myself.joint_velocity[index];
    state->reference = controller_myself.joint_reference[index];
    state->torque = controller_myself.joint_torque[index];
    state->ecd = controller_myself.joint_last_ecd[index];
    state->last_update_ms = controller_myself.joint_last_feedback_time[index];
    state->valid = controller_myself.joint_feedback_valid[index];
    return true;
}

/**
 * @brief          读取单个关节的当前角度 (rad)
 * @param[in]      index 关节索引 (0~5)
 * @retval         float 关节角度，索引非法返回 0.0f
 */
float GetJointAngle(uint8_t index)
{
    if (index >= CONTROLLER_MYSELF_JOINT_NUM)
    {
        return 0.0f;
    }
    return controller_myself.joint_feedback[index];
}

/**
 * @brief          读取单个关节的当前角速度 (rad/s)
 * @param[in]      index 关节索引 (0~5)
 * @retval         float 关节角速度，索引非法返回 0.0f
 */
float GetJointVelocity(uint8_t index)
{
    if (index >= CONTROLLER_MYSELF_JOINT_NUM)
    {
        return 0.0f;
    }
    return controller_myself.joint_velocity[index];
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void)
{
    static const uint8_t direction[CONTROLLER_MYSELF_JOINT_NUM] = {1, 1, 1, 1, 1, 1};
    memset(&controller_myself, 0, sizeof(controller_myself));
    controller_myself.rc = get_remote_control_point();
    memcpy(controller_myself.joint_direction, direction, sizeof(direction));

    for (uint8_t i = 0; i < CONTROLLER_MYSELF_JOINT_NUM; i++)
    {
        SBzdatou_ecd_get(1, SBZDATOU1_EXT_ID + (uint32_t)i * 0x100U);
        vTaskDelay(20);
    }
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void)
{
    const SBzdatouMeasure_s *measure = GetSBzdatouMeasurePoint(1);
    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < CONTROLLER_MYSELF_JOINT_NUM; i++)
    {
        const uint16_t ecd = measure[i].ecd;
        const uint32_t dt_ms = now - controller_myself.joint_last_feedback_time[i];
        controller_myself.joint_feedback[i] = ((float)ecd / 65535.0f) * 2.0f * (float)M_PI;
        controller_myself.joint_feedback[i] = WrapAngle(controller_myself.joint_feedback[i]);

        if (controller_myself.joint_feedback_valid[i] && dt_ms > 0U)
        {
            float delta = controller_myself.joint_feedback[i] - controller_myself.joint_feedback[i];
            delta = ((float)((int32_t)ecd - (int32_t)controller_myself.joint_last_ecd[i])) *
                    2.0f * (float)M_PI / 65536.0f;
            if (delta > (float)M_PI)
                delta -= 2.0f * (float)M_PI;
            if (delta < -(float)M_PI)
                delta += 2.0f * (float)M_PI;
            controller_myself.joint_velocity[i] = delta / ((float)dt_ms * 0.001f);
        }
        else
        {
            controller_myself.joint_velocity[i] = 0.0f;
        }

        controller_myself.joint_last_ecd[i] = ecd;
        controller_myself.joint_last_feedback_time[i] = measure[i - 1].last_fdb_time;
        controller_myself.joint_feedback_valid[i] = (measure[i - 1].last_fdb_time != 0U);
    }

    // 关节角更新完毕后计算正运动学：每个关节位姿 + 末端位姿
    FK_SolveAll(controller_myself.joint_feedback, joint_pose);
    controller_myself_fk = joint_pose[CONTROLLER_MYSELF_JOINT_NUM - 1];
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void)
{
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void)
{


    
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{
    if (sendctrl == 0)
    {

        SBzdatou_torque_control(1, 0x100, 0x01, datouctrl[0]);
        SBzdatou_torque_control(1, 0x200, 0x01, datouctrl[1]);

        sendctrl = 1;
    }
    else if (sendctrl == 1)
    {

        SBzdatou_torque_control(1, 0x300, 0x01, datouctrl[2]);
        SBzdatou_torque_control(1, 0x400, 0x01, datouctrl[3]);

        sendctrl = 2;
    }
    else
    {

        SBzdatou_torque_control(1, 0x500, 0x01, datouctrl[4]);
        SBzdatou_torque_control(1, 0x600, 0x01, datouctrl[5]);

        sendctrl = 0;
    }
    // CanManagerAddMotor(1, 1, 600);
}

/************************ END OF FILE ************************/
