/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       controler_myself.c/h
  * @brief      自定义云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef CONTROLER_MYSELF_H
#define CONTROLER_MYSELF_H

#include "robot_param.h"
#include "IMU.h"
#include "gimbal.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "usb_debug.h"
#include "cmsis_os.h"
#include "math.h"
#include "macro_typedef.h"
#include "supervisory_computer_cmd.h"

/**
 * @brief 状态、期望和限制值
 */
#define CONTROLLER_MYSELF_JOINT_NUM 6U

/* SB大头反馈没有速度字段，速度由相邻两次编码器位置估算。 */
#ifndef CONTROLLER_MYSELF_JOINT_KP
#define CONTROLLER_MYSELF_JOINT_KP 120.0f
#endif
#ifndef CONTROLLER_MYSELF_JOINT_KD
#define CONTROLLER_MYSELF_JOINT_KD 8.0f
#endif
#ifndef CONTROLLER_MYSELF_JOINT_TORQUE_MAX
#define CONTROLLER_MYSELF_JOINT_TORQUE_MAX 8000
#endif
#ifndef CONTROLLER_MYSELF_JOINT_TARGET_STEP
#define CONTROLLER_MYSELF_JOINT_TARGET_STEP 0.002f
#endif

typedef struct
{
    float pitch;
    float yaw;
} Values_t;

/**
 * @brief 标准DH参数（每个关节一组，由机械结构决定，运行时不变）
 * @note  标准DH约定下单关节变换： T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 */
typedef struct
{
    float a;             // 连杆长度   (mm)  沿 X(i-1) 轴，从 Z(i-1) 到 Z(i) 的距离
    float d;             // 连杆偏距   (mm)  沿 Z(i) 轴，从 X(i-1) 到 X(i) 的距离
    float alpha;         // 连杆扭转角 (rad) 绕 X(i-1) 轴，从 Z(i-1) 转到 Z(i)
    float theta_offset;  // 关节零位偏置(rad) 关节反馈角度的机械零点修正量
    float axis[3];       // 关节转轴单位向量(在本关节坐标系下)。默认{0,0,1}绕Z轴，
                         // 可任意指定，如{1,0,0}绕X、{0,1,0}绕Y。程序内会自动归一化。
} DH_Param_t;

/**
 * @brief 正解输出：末端执行器相对于基座的位姿
 * @note  姿态同时给出两种表示：
 *          - 四元数 (qw,qx,qy,qz)：无奇异点，用于比较/控制，首选。
 *          - ZYX 欧拉角 (roll/pitch/yaw)：仅供人读/显示，会在 pitch=±90°
 *            处发生万向锁跳变，不要拿它做角度比较或闭环。
 */
typedef struct
{
    float x;      // 末端位置 X (mm)
    float y;      // 末端位置 Y (mm)
    float z;      // 末端位置 Z (mm)
    float roll;   // 绕 X 轴姿态角 (rad)  —— 仅显示用
    float pitch;  // 绕 Y 轴姿态角 (rad)  —— 仅显示用
    float yaw;    // 绕 Z 轴姿态角 (rad)  —— 仅显示用
    float qw;     // 姿态四元数 w （标量部）
    float qx;     // 姿态四元数 x
    float qy;     // 姿态四元数 y
    float qz;     // 姿态四元数 z
} FK_Result_t;

/**
 * @brief 单个关节的完整状态快照（供外部随时读取）
 */
typedef struct
{
    float angle;              // 当前关节角度 (rad)
    float velocity;           // 当前关节角速度 (rad/s)
    float reference;          // 目标角度 (rad)
    int32_t torque;           // 当前发送的扭矩控制量
    uint16_t ecd;             // 原始编码器值 (0~65535)
    uint32_t last_update_ms;  // 上次反馈更新时间 (ms)
    bool valid;               // 反馈是否有效
} JointState_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def pitch_angle;
    pid_type_def pitch_velocity;

} Gimbal_PID_t;

typedef struct
{
    const RC_ctrl_t * rc;
    BoardMode_e mode, last_mode, mode_before_rc_err;

    /*-------------------- Motors --------------------*/
    Motor_s yaw, pitch;
    /* 六轴SB大头关节控制数据，索引0~5对应扩展ID 0x100~0x600。 */
    float joint_reference[CONTROLLER_MYSELF_JOINT_NUM];
    float joint_feedback[CONTROLLER_MYSELF_JOINT_NUM];  // 当前关节角 (rad)，已含 DH 零位偏置 theta_offset
    float joint_velocity[CONTROLLER_MYSELF_JOINT_NUM];
    int32_t joint_torque[CONTROLLER_MYSELF_JOINT_NUM];
    uint8_t joint_direction[CONTROLLER_MYSELF_JOINT_NUM];
    uint32_t joint_last_feedback_time[CONTROLLER_MYSELF_JOINT_NUM];
    uint16_t joint_last_ecd[CONTROLLER_MYSELF_JOINT_NUM];
    bool joint_feedback_valid[CONTROLLER_MYSELF_JOINT_NUM];
    /*-------------------- Values --------------------*/
    Values_t reference;
    Values_t feedback_pos, feedback_vel;
    Values_t upper_limit;
    Values_t lower_limit;

    Gimbal_PID_t pid;

    float pitch_angle_zero_for_imu;
    float yaw_angle_zero_for_imu;

    uint32_t init_start_time, init_timer;

    bool init_continue;
} Gimbal_s;

/*-------------------- Global var --------------------*/

extern Gimbal_s controller_myself;
extern Gimbal_PID_t controller_myself_pid;
extern FK_Result_t controller_myself_fk;                       // 末端位姿
extern FK_Result_t joint_pose[CONTROLLER_MYSELF_JOINT_NUM];    // 各关节位姿
extern float base_transform[4][4];                             // 基座安装变换矩阵

/*-------------------- Public --------------------*/

extern void GimbalPublish(void);
extern void GimbalInit(void);
extern void GimbalSetMode(void);
extern void GimbalObserver(void);
extern void GimbalReference(void);
extern void GimbalConsole(void);
extern void GimbalSendCmd(void);

/**
 * @brief          六轴机械臂正运动学求解
 * @param[in]      joints 6个关节角度 (rad)，对应 controller_myself.joint_feedback
 * @param[out]     result 末端位姿 (xyz + roll/pitch/yaw)
 * @retval         none
 */
extern void FK_Solve(const float joints[CONTROLLER_MYSELF_JOINT_NUM], FK_Result_t *result);

/**
 * @brief          逐关节正运动学求解：算出每个关节坐标系相对基座的位姿
 * @note           poses[i] 为关节 i+1 坐标系原点相对基座的位姿；
 *                 poses[5] 即末端位姿，与 FK_Solve 输出一致。
 * @param[in]      joints 6个关节角度 (rad)，对应 controller_myself.joint_feedback
 * @param[out]     poses  长度 CONTROLLER_MYSELF_JOINT_NUM 的位姿数组
 * @retval         none
 */
extern void FK_SolveAll(const float joints[CONTROLLER_MYSELF_JOINT_NUM],
                        FK_Result_t poses[CONTROLLER_MYSELF_JOINT_NUM]);

/**
 * @brief          两个姿态之间的旋转误差 (轴角表示，无奇异点)
 * @note           用四元数求相对旋转 q_err = q_a^{-1} * q_b，再转成轴角。
 *                 返回的模长即绕最短路径的旋转角(rad)，方向为旋转轴。
 *                 用它替代"欧拉角相减"，可避免万向锁处的错误比较。
 * @param[in]      a        参考姿态
 * @param[in]      b        目标姿态
 * @param[out]     axis_ang 长度3的数组：单位旋转轴 × 旋转角(rad)。可传NULL。
 * @retval         float    两姿态间的夹角 (rad, 0~π)
 */
extern float PoseAttitudeError(const FK_Result_t *a, const FK_Result_t *b, float axis_ang[3]);

/**
 * @brief          读取单个关节的完整状态快照
 * @param[in]      index 关节索引 (0~5，对应关节1~6)
 * @param[out]     state 该关节的状态输出
 * @retval         bool  索引合法且写入成功返回 true，否则 false
 */
extern bool GetJointState(uint8_t index, JointState_t *state);

/**
 * @brief          读取单个关节的当前角度 (rad)
 * @param[in]      index 关节索引 (0~5)
 * @retval         float 关节角度，索引非法返回 0.0f
 */
extern float GetJointAngle(uint8_t index);

/**
 * @brief          读取单个关节的当前角速度 (rad/s)
 * @param[in]      index 关节索引 (0~5)
 * @retval         float 关节角速度，索引非法返回 0.0f
 */
extern float GetJointVelocity(uint8_t index);

/**
 * @brief          获取全部关节角度数组指针 (rad)
 * @param[in]      none
 * @retval         const float* joint_feedback 数组首地址，index 0~5 对应关节1~6
 */
extern const float *GetGimbalJointFeedbackPoint(void);

#endif  // CONTROLER_MYSELF_H

/************************ END OF FILE ************************/
