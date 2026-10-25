/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_engineer.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-20-2024     Penguin         1. done
  *  V1.0.1     Jan-14-2025     Penguin         1. 实现机械臂的基本控制
  *
  @verbatim
  ==============================================================================
机械臂相关的一些方向定义
    - 机械臂水平向前时设置J0关节为0位置
    - 机械臂竖直向上时设置J1 J2 J3关节为0位置

    - 定义机械臂水平向前时的J0关节位置为0，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，从机械臂右侧看，逆时针为正方向（注：J1 J2关节的合位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0，从吸盘方向看，逆时针为正方向
    - 定义J4为末端机构右侧（上视，J3向前）电机，J5为末端机构左侧电机

    - 定义虚拟J4关节用来衡量末端机构的pitch, 虚拟J5关节用来衡量末端机构的roll
    - 定义J4正方向为：当J3归中时，从机械臂右侧看，逆时针为正方向
    - 定义J5正方向为：当J3归中时，从机械臂前方看，逆时针为正方向
    - vj4和j4 j5的关系：vj4 = (j4 - j5)/2

机械臂控制
    左拨杆 
          上：跟随模式
          中：调试模式
          下：安全模式
    右拨杆
          上：气泵开
          中：气泵关
          下：气泵关
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_engineer.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include <stdbool.h>

#include "CAN_communication.h"
#include "PWM_cmd_pump.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "custom_controller_connect.h"
#include "detect_task.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "signal_generator.h"
#include "string.h"
#include "usb_debug.h"

/*------------------------------ Macro Definition ------------------------------*/

// clang-format off
#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define CUSTOM_CONTROLLER_DATA_ERROR_OFFSET   ((uint8_t)1 << 1)  // 自定义控制器数据异常偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
// clang-format on

#define MS_TO_S 0.001f  // ms转s

#define ANGLE_PID 0
#define VELOCITY_PID 1

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5

#define DM_DELAY 250  // (us)dm电机发送延时

#define J0_KP_FOLLOW 0
#define J0_KD_FOLLOW 1.5

#define J1_KP_FOLLOW 0
#define J1_KD_FOLLOW 10

#define J2_KP_FOLLOW 0
#define J2_KD_FOLLOW 10

#define INIT_2006_SET_VALUE (-1000)  // 2006电机在进行初始化时的电流设置值
#define INIT_2006_MIN_VEL 1          // 2006电机初始化完成的速度阈值

// 气泵相关
#define PUMP_ON_PWM 30000
#define PUMP_OFF_PWM 0
#define PUMP_PWM_CHANNEL 1

#define JointMotorInit(index)                                                                    \
    MotorInit(                                                                                   \
        &MECHANICAL_ARM.joint_motor[index], JOINT_MOTOR_##index##_ID, JOINT_MOTOR_##index##_CAN, \
        JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION,                             \
        JOINT_MOTOR_##index##_REDUCATION_RATIO, JOINT_MOTOR_##index##_MODE)

#define JointPidInit(index)                                                                \
    {                                                                                      \
        float j##index##_pid_angle[3] = {                                                  \
            KP_JOINT_##index##_ANGLE, KI_JOINT_##index##_ANGLE, KD_JOINT_##index##_ANGLE}; \
        float j##index##_pid_velocity[3] = {                                               \
            KP_JOINT_##index##_VELOCITY, KI_JOINT_##index##_VELOCITY,                      \
            KD_JOINT_##index##_VELOCITY};                                                  \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[0], PID_POSITION, j##index##_pid_angle,         \
            MAX_OUT_JOINT_##index##_ANGLE, MAX_IOUT_JOINT_##index##_ANGLE);                \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[1], PID_POSITION, j##index##_pid_velocity,      \
            MAX_OUT_JOINT_##index##_VELOCITY, MAX_IOUT_JOINT_##index##_VELOCITY);          \
    }

#define JointLowPassFilterInit(index) \
    LowPassFilterInit(&MECHANICAL_ARM.lpf.j[index], J##index##_LPF_ALPHA)

/*------------------------------ Variable Definition ------------------------------*/

MechanicalArm_s MECHANICAL_ARM;
#define MA MECHANICAL_ARM

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/******************************************************************/

void MechanicalArmPublish(void) {}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      MechanicalArmInit                          */
/* auxiliary function: None                                       */
/******************************************************************/

void MechanicalArmInit(void)
{
    MECHANICAL_ARM.rc = get_remote_control_point();

    MECHANICAL_ARM.init_completed = false;
    MECHANICAL_ARM.reach_time = 0;
    // #Motor init ---------------------
    JointMotorInit(0);
    JointMotorInit(1);
    JointMotorInit(2);
    JointMotorInit(3);
    JointMotorInit(4);
    JointMotorInit(5);
    // #PID init ---------------------
    JointPidInit(0);
    JointPidInit(1);
    JointPidInit(2);
    JointPidInit(3);
    JointPidInit(4);
    JointPidInit(5);
    // #LPF init ---------------------
    JointLowPassFilterInit(0);
    JointLowPassFilterInit(1);
    JointLowPassFilterInit(2);
    JointLowPassFilterInit(3);
    JointLowPassFilterInit(4);
    JointLowPassFilterInit(5);
    // #limit init ---------------------
    MECHANICAL_ARM.limit.max.pos[J0] = MAX_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.max.pos[J1] = MAX_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.max.pos[J2] = MAX_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.max.pos[J3] = MAX_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.max.pos[J4] = MAX_JOINT_4_POSITION;
    MECHANICAL_ARM.limit.max.pos[J5] = MAX_JOINT_5_POSITION;

    MECHANICAL_ARM.limit.min.pos[J0] = MIN_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.min.pos[J1] = MIN_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.min.pos[J2] = MIN_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.min.pos[J3] = MIN_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.min.pos[J4] = MIN_JOINT_4_POSITION;
    MECHANICAL_ARM.limit.min.pos[J5] = MIN_JOINT_5_POSITION;
    // #memset ---------------------
    memset(&MECHANICAL_ARM.fdb, 0, sizeof(MECHANICAL_ARM.fdb));
    memset(&MECHANICAL_ARM.ref, 0, sizeof(MECHANICAL_ARM.ref));
    // #ref init ---------------------
    MECHANICAL_ARM.ref.joint[J0].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.limit.max.pos[J1];
    MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.limit.min.pos[J2];
    MECHANICAL_ARM.ref.joint[J3].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J4].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J5].angle = 0.0f;

    // #Initial value setting ---------------------
    MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    MECHANICAL_ARM.error_code = 0;

    MECHANICAL_ARM.cmd.pump_on = false;

    MECHANICAL_ARM.transform.dpos[J0] = J0_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J1] = J1_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J2] = J2_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J3] = J3_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J4] = J4_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J5] = J5_ANGLE_TRANSFORM;

    MECHANICAL_ARM.transform.duration[J0] = 4;
    MECHANICAL_ARM.transform.duration[J1] = 4;
    MECHANICAL_ARM.transform.duration[J2] = 4;
    MECHANICAL_ARM.transform.duration[J3] = 2;
    MECHANICAL_ARM.transform.duration[J4] = 1;
    MECHANICAL_ARM.transform.duration[J5] = 1;
}

/******************************************************************/
/* HandleException                                                */
/******************************************************************/

void MechanicalArmHandleException(void)
{
    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_INIT) {
        // 初始化时如果电机反馈的速度小于阈值，则认为电机初始化完成
        if (fabsf(MECHANICAL_ARM.joint_motor[J4].fdb.vel) < INIT_2006_MIN_VEL &&
            fabsf(MECHANICAL_ARM.joint_motor[J5].fdb.vel) < INIT_2006_MIN_VEL) {
            MECHANICAL_ARM.reach_time += MECHANICAL_ARM.duration;
        } else {
            MECHANICAL_ARM.reach_time = 0;
        }

        if (MECHANICAL_ARM.reach_time > 200 && !MECHANICAL_ARM.init_completed) {
            // 停止时间超过200ms则认为达到限位
            MECHANICAL_ARM.init_completed = true;
            float virtual_j4_pos =
                (MECHANICAL_ARM.fdb.joint[J4].angle - MECHANICAL_ARM.fdb.joint[J5].angle) / 2;
            float virtual_j5_pos =
                MECHANICAL_ARM.fdb.joint[J4].angle + MECHANICAL_ARM.fdb.joint[J5].angle;
            // 设置虚拟关节J4关节的位置限制
            MECHANICAL_ARM.limit.max.vj4_pos = virtual_j4_pos - 0.15f + M_PI;
            MECHANICAL_ARM.limit.min.vj4_pos = virtual_j4_pos + 0.15f;
        }
    }

    if (fabsf(MECHANICAL_ARM.joint_motor[J1].fdb.tor) > 10 ||
        fabsf(MECHANICAL_ARM.joint_motor[J2].fdb.tor) > 10) {
        MECHANICAL_ARM.error_code |= JOINT_TORQUE_MORE_OFFSET;
    }
}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSetMode                      */
/* auxiliary function:  None                                      */
/******************************************************************/

void MechanicalArmSetMode(void)
{
    if (toe_is_error(DBUS_TOE)) {  // 安全，保命！！！！！！
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
        return;
    }

    if (MECHANICAL_ARM.error_code) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
        return;
    }

    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_CALIBRATE) {
        return;
    }

    if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_FOLLOW;
    } else if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_down(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    }

    // 非安全模式下，初始化未完成时，进入初始化模式
    if ((MECHANICAL_ARM.mode != MECHANICAL_ARM_SAFE) && (!MECHANICAL_ARM.init_completed)) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_INIT;
    }
}

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmObserver                     */
/* auxiliary function:  UpdateMotorStatus                         */
/*                      JointStateObserve                         */
/******************************************************************/

static void UpdateMotorStatus(void);
static void JointStateObserve(void);

void MechanicalArmObserver(void)
{
    MA.duration = xTaskGetTickCount() - MA.last_time;
    MA.last_time = xTaskGetTickCount();

    MA.custom_controller_ready = GetRefereeState();

    UpdateMotorStatus();
    JointStateObserve();
}

/**
 * @brief  更新电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    uint8_t i;
    for (i = 0; i < 6; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
    }
}

/**
 * @brief  关节状态观测
 * @param  none
 */
static void JointStateObserve(void)
{
    static float last_angle[6], angle_fdb[6] = {0, 0, 0, 0, 0, 0};
    float vel, dpos;
#define dangle MECHANICAL_ARM.transform.dpos

    /*-----处理J0 J1 J2关节的反馈信息（J4 J5作为差速机构要特殊处理）*/
    uint8_t i;
    for (i = 0; i < 3; i++) {
        MA.fdb.joint[i].angle = theta_transform(
                                    MA.joint_motor[i].fdb.pos, dangle[i],
                                    MA.joint_motor[i].direction, MA.transform.duration[i]) /
                                MA.joint_motor[i].reduction_ratio;
        MA.fdb.joint[i].velocity = MA.joint_motor[i].fdb.vel / MA.joint_motor[i].reduction_ratio *
                                   MA.joint_motor[i].direction;
        MA.fdb.joint[i].torque = MA.joint_motor[i].fdb.tor * MA.joint_motor[i].reduction_ratio *
                                 MA.joint_motor[i].direction;
    }
    /*-----处理J3关节的反馈信息(涉及到多圈计数问题)*/
    angle_fdb[J3] = theta_transform(
                        MA.joint_motor[J3].fdb.pos, dangle[J3], MA.joint_motor[J3].direction,
                        MA.transform.duration[J3]) /
                    MA.joint_motor[J3].reduction_ratio;

    dpos = angle_fdb[J3] - last_angle[J3];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J3].round += (dpos) < 0 ? 1 : -1;
    }

    last_angle[J3] = angle_fdb[J3];

    MA.fdb.joint[J3].angle = angle_fdb[J3] + M_PI * 2 * MA.fdb.joint[J3].round;

    vel = MA.joint_motor[J3].fdb.vel / MA.joint_motor[J3].reduction_ratio *
          MA.joint_motor[J3].direction;

    MA.fdb.joint[J3].velocity = LowPassFilterCalc(&MA.lpf.j[J3], vel);

    /*-----处理J4 J5 关节的反馈信息*/
    // 处理速度反馈
    vel = MA.joint_motor[J4].fdb.vel / MA.joint_motor[J4].reduction_ratio *
          MA.joint_motor[J4].direction;
    MA.fdb.joint[J4].velocity = LowPassFilterCalc(&MA.lpf.j[J4], vel);

    vel = MA.joint_motor[J5].fdb.vel / MA.joint_motor[J5].reduction_ratio *
          MA.joint_motor[J5].direction;
    MA.fdb.joint[J5].velocity = LowPassFilterCalc(&MA.lpf.j[J5], vel);

    // 处理位置反馈
    // J4
    angle_fdb[J4] = theta_transform(
        MA.joint_motor[J4].fdb.pos, dangle[J4], MA.joint_motor[J4].direction,
        MA.transform.duration[J4]);

    dpos = angle_fdb[J4] - last_angle[J4];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J4].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J4] = angle_fdb[J4];
    MA.fdb.joint[J4].angle =
        (angle_fdb[J4] + M_PI * 2 * MA.fdb.joint[J4].round) / MA.joint_motor[J4].reduction_ratio;

    // J5
    angle_fdb[J5] = theta_transform(
        MA.joint_motor[J5].fdb.pos, dangle[J5], MA.joint_motor[J5].direction,
        MA.transform.duration[J5]);

    dpos = angle_fdb[J5] - last_angle[J5];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J5].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J5] = angle_fdb[J5];
    MA.fdb.joint[J5].angle =
        (angle_fdb[J5] + M_PI * 2 * MA.fdb.joint[J5].round) / MA.joint_motor[J5].reduction_ratio;

#undef dangle
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmReference                    */
/******************************************************************/

void MechanicalArmReference(void)
{
    uint8_t i;
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_CUSTOM: {
            // MECHANICAL_ARM.ref.joint[J0].angle = MECHANICAL_ARM.rc->rc.ch[4] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.rc->rc.ch[0] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.rc->rc.ch[1] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J3].angle = 0;
            // MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.rc->rc.ch[2] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J5].angle = MECHANICAL_ARM.rc->rc.ch[3] * RC_TO_ONE * M_PI;
        } break;
        case MECHANICAL_ARM_DEBUG: {
            if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
                // j0
                MA.ref.joint[J0].angle += GetDt7RcCh(DT7_CH_ROLLER) * 0.002f;
                MA.ref.joint[J0].angle = fp32_constrain(
                    MA.ref.joint[J0].angle, MA.limit.min.pos[J0], MA.limit.max.pos[J0]);

                // j1
                MA.ref.joint[J1].angle += GetDt7RcCh(DT7_CH_RH) * 0.002f;
                MA.ref.joint[J1].angle = fp32_constrain(
                    MA.ref.joint[J1].angle, MA.limit.min.pos[J1], MA.limit.max.pos[J1]);

                // j2
                MA.ref.joint[J2].angle += GetDt7RcCh(DT7_CH_RV) * 0.002f;
                MA.ref.joint[J2].angle = fp32_constrain(
                    MA.ref.joint[J2].angle, MA.limit.min.pos[J2], MA.limit.max.pos[J2]);

                // j3
                MA.ref.joint[J3].angle += GetDt7RcCh(DT7_CH_LH) * 0.002f;
                MA.ref.joint[J3].angle = fp32_constrain(
                    MA.ref.joint[J3].angle, MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

                MA.ref.joint[J4].angle = 0;
                MA.ref.joint[J5].angle = 0;
            } else if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
                // j4
                MA.ref.joint[J4].angle += GetDt7RcCh(DT7_CH_RV) * 0.002f;
                // MA.ref.joint[J4].angle = GenerateSinWave(1, 0, 3);
                // MA.ref.joint[J4].angle =
                //     fp32_constrain(MA.ref.joint[J4].angle, MA.limit.min.pos[J4], MA.limit.max.pos[J4]);

                // j5
                MA.ref.joint[J5].angle += GetDt7RcCh(DT7_CH_LV) * 0.002f;
                // MA.ref.joint[J5].angle = GenerateSinWave(1, 0, 3);
                // MA.ref.joint[J5].angle =
                //     fp32_constrain(MA.ref.joint[J5].angle, MA.limit.min.pos[J5], MA.limit.max.pos[J5]);
            }
            float virtual_j4_pos = (MA.ref.joint[J4].angle - MA.ref.joint[J5].angle) / 2;
            float delta = 0;
            if (virtual_j4_pos > MA.limit.max.vj4_pos) {
                delta = virtual_j4_pos - MA.limit.max.vj4_pos;
            } else if (virtual_j4_pos < MA.limit.min.vj4_pos) {
                delta = virtual_j4_pos - MA.limit.min.vj4_pos;
            }
            MA.ref.joint[J4].angle -= delta;
            MA.ref.joint[J5].angle += delta;
        } break;
        case MECHANICAL_ARM_FOLLOW: {
            if (MA.custom_controller_ready) {
                float pos[6] = {0};
                pos[J0] = GetCustomControllerPos(J0);
                pos[J1] = GetCustomControllerPos(J1);
                pos[J2] = GetCustomControllerPos(J2);
                pos[J3] = GetCustomControllerPos(J3);
                pos[J4] = GetCustomControllerPos(J4);
                pos[J5] = GetCustomControllerPos(J5);

                if (fabsf(pos[J0] - MA.ref.joint[J0].angle) > 1.5f ||
                    fabsf(pos[J1] - MA.ref.joint[J1].angle) > 1.5f ||
                    fabsf(pos[J2] - MA.ref.joint[J2].angle) > 1.5f) {  // 位置突变
                    MECHANICAL_ARM.error_code |= CUSTOM_CONTROLLER_DATA_ERROR_OFFSET;
                    return;
                }

                MA.ref.joint[J0].angle =
                    fp32_constrain(pos[J0], MA.limit.min.pos[J0], MA.limit.max.pos[J0]);
                MA.ref.joint[J1].angle =
                    fp32_constrain(pos[J1], MA.limit.min.pos[J1], MA.limit.max.pos[J1]);
                MA.ref.joint[J2].angle =
                    fp32_constrain(pos[J2], MA.limit.min.pos[J2], MA.limit.max.pos[J2]);
                MA.ref.joint[J3].angle =
                    fp32_constrain(pos[J3], MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

                // 自定义控制器传过来的虚拟J4 J5位置以0为中心
                float vj4_pos = (pos[J4] - pos[J5]) / 2;
                float vj5_pos = (pos[J4] + pos[J5]) / 2;

                // 机械臂J4关节以((MA.limit.max.vj4_pos + MA.limit.min.vj4_pos) / 2)为中心
                // 机械臂J5关节以((MA.limit.max.vj5_pos + MA.limit.min.vj5_pos) / 2)为中心
                float vj4_pos_mid = (MA.limit.max.vj4_pos + MA.limit.min.vj4_pos) / 2;
                float vj5_pos_mid = (MA.limit.max.vj5_pos + MA.limit.min.vj5_pos) / 2;

                vj4_pos += vj4_pos_mid;
                vj5_pos += vj5_pos_mid;

                vj4_pos = fp32_constrain(vj4_pos, MA.limit.min.vj4_pos, MA.limit.max.vj4_pos);
                // vj5_pos = fp32_constrain(vj5_pos, MA.limit.min.vj5_pos, MA.limit.max.vj5_pos);

                MA.ref.joint[J4].angle = vj4_pos + vj5_pos;
                MA.ref.joint[J5].angle = -(vj4_pos - vj5_pos);
            }
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            for (i = 0; i < 6; i++) {
                MECHANICAL_ARM.ref.joint[i].velocity = 0;
            }
        }
    }
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmConsole                      */
/******************************************************************/

void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_CUSTOM: {
            // // 优先处理dm电机部分
            // // 位置
            // MECHANICAL_ARM.joint_motor[J0].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J0].angle, -J0_ANGLE_TRANSFORM, 1, 1);
            // MECHANICAL_ARM.joint_motor[J1].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J1].angle, -J1_ANGLE_TRANSFORM, 1, 1);
            // MECHANICAL_ARM.joint_motor[J2].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J2].angle, -J2_ANGLE_TRANSFORM, 1, 1);
            // // 速度
            // MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            // MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            // MECHANICAL_ARM.joint_motor[J3].set.vel = 0;

            // // 然后再处理dji电机部分，涉及到pid计算
            // // J0
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j0[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J0].angle,
            //     MECHANICAL_ARM.ref.joint[J0].angle);
            // MECHANICAL_ARM.joint_motor[J0].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j0[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J0].velocity,
            //     MECHANICAL_ARM.pid.j0[ANGLE_PID].out);
            // // J4
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j4[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J4].angle,
            //     MECHANICAL_ARM.ref.joint[J4].angle);
            // MECHANICAL_ARM.joint_motor[J4].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j4[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J4].velocity,
            //     MECHANICAL_ARM.pid.j4[ANGLE_PID].out);
            // // J5
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j5[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J5].angle,
            //     MECHANICAL_ARM.ref.joint[J5].angle);
            // MECHANICAL_ARM.joint_motor[J5].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j5[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J5].velocity,
            //     MECHANICAL_ARM.pid.j5[ANGLE_PID].out);
        } break;
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_DEBUG: {
            /*机械臂J0 J1 J2基本不会出现过圈问题，不考虑过圈时的最优旋转方向问题。*/

            // J0
            MA.joint_motor[J0].set.vel =
                PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle) *
                MA.joint_motor[J0].direction * MA.joint_motor[J0].reduction_ratio;
            MA.joint_motor[J0].set.tor = 0;

            // J1
            MA.joint_motor[J1].set.vel =
                PID_calc(&MA.pid.j1[0], MA.fdb.joint[J1].angle, MA.ref.joint[J1].angle) *
                MA.joint_motor[J1].direction * MA.joint_motor[J1].reduction_ratio;
            MA.joint_motor[J1].set.tor = 0;

            // J2
            MA.joint_motor[J2].set.vel =
                PID_calc(&MA.pid.j2[0], MA.fdb.joint[J2].angle, MA.ref.joint[J2].angle) *
                MA.joint_motor[J2].direction * MA.joint_motor[J2].reduction_ratio;
            MA.joint_motor[J2].set.tor = 0;

            /*机械臂J3 J4 J5需要考虑过圈的处理（在Observer时已经记录圈数获得多圈反馈了）*/
            // J3
            MA.joint_motor[J3].set.vel =
                PID_calc(&MA.pid.j3[0], MA.fdb.joint[J3].angle, MA.ref.joint[J3].angle) *
                MA.joint_motor[J3].direction * MA.joint_motor[J3].reduction_ratio;
            MA.joint_motor[J3].set.value =
                PID_calc(&MA.pid.j3[1], MA.fdb.joint[J3].velocity, MA.joint_motor[J3].set.vel);

            // J4
            MA.joint_motor[J4].set.vel =
                PID_calc(&MA.pid.j4[0], MA.fdb.joint[J4].angle, MA.ref.joint[J4].angle) *
                MA.joint_motor[J4].direction * MA.joint_motor[J4].reduction_ratio;
            MA.joint_motor[J4].set.value =
                PID_calc(&MA.pid.j4[1], MA.fdb.joint[J4].velocity, MA.joint_motor[J4].set.vel);
            // MA.joint_motor[J4].set.value = GenerateSinWave(1000, 0, 3);

            // J5
            MA.joint_motor[J5].set.vel =
                PID_calc(&MA.pid.j5[0], MA.fdb.joint[J5].angle, MA.ref.joint[J5].angle) *
                MA.joint_motor[J5].direction * MA.joint_motor[J5].reduction_ratio;
            MA.joint_motor[J5].set.value =
                PID_calc(&MA.pid.j5[1], MA.fdb.joint[J5].velocity, MA.joint_motor[J5].set.vel);
            // MA.joint_motor[J5].set.value = GenerateSinWave(2000, 0, 3);

            // 气泵
            if (switch_is_up(GetDt7RcSw(PUMP_CHANNEL))) {
                MA.cmd.pump_on = true;
            } else {
                MA.cmd.pump_on = false;
            }
        } break;
        case MECHANICAL_ARM_INIT: {
            MA.joint_motor[J4].set.value = INIT_2006_SET_VALUE;
            MA.joint_motor[J5].set.value = -INIT_2006_SET_VALUE;
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            MECHANICAL_ARM.joint_motor[J0].set.value = 0;
            MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J3].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J4].set.value = 0;
            MECHANICAL_ARM.joint_motor[J5].set.value = 0;
        }
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSendCmd                      */
/* auxiliary function:  ArmSendCmdSafe                            */
/*                      ArmSendCmdFollow                          */
/******************************************************************/

void ArmSendCmdSafe(void);
void ArmSendCmdDebug(void);
void ArmSendCmdInit(void);

void MechanicalArmSendCmd(void)
{
    uint8_t cnt;
    for (uint8_t i = 0; i < 3; i++) {
        if (cnt % 2 == 0) {
            delay_us(DM_DELAY);
        }
        if (MECHANICAL_ARM.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
            DmEnable(&MECHANICAL_ARM.joint_motor[i]);
            cnt++;
        }
    }

    delay_us(DM_DELAY);

    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_DEBUG: {
            ArmSendCmdDebug();
        } break;
        case MECHANICAL_ARM_INIT: {
            ArmSendCmdInit();
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_CUSTOM:
        case MECHANICAL_ARM_SAFE:
        default: {
            ArmSendCmdSafe();
        }
    }
}

void ArmSendCmdSafe(void)
{
    // 电机控制
    DmMitStop(&MECHANICAL_ARM.joint_motor[J0]);
    delay_us(DM_DELAY);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J1]);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J2]);
    delay_us(DM_DELAY);
    CanCmdDjiMotor(ARM_DJI_CAN, 0x1FF, 0, 0, 0, 0);  // J3 J4 J5

    // 气泵控制
    PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
}

void ArmSendCmdDebug(void)
{
    // 电机控制
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J0], J0_KD_FOLLOW);
    delay_us(DM_DELAY);
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J1], J1_KD_FOLLOW);
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J2], J2_KD_FOLLOW);
    delay_us(DM_DELAY);
    // clang-format off
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF, 
        MA.joint_motor[J3].set.value, 
        0, 
        MA.joint_motor[J4].set.value,
        MA.joint_motor[J5].set.value);  // J3 JN J4 J5
    // clang-format on

    // 气泵控制
    if (MECHANICAL_ARM.cmd.pump_on) {
        PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_ON_PWM);
    } else {
        PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
    }
}

void ArmSendCmdInit(void)
{
    // 电机控制
    // clang-format off
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF, 
        0, 
        0, 
        MA.joint_motor[J4].set.value,
        MA.joint_motor[J5].set.value);  // J3 JN J4 J5
    // clang-format on
    // 气泵控制
    PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
}

#endif

#undef MA
/*------------------------------ End of File ------------------------------*/
