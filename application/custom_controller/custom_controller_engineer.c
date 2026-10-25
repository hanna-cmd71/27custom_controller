/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_engineer.c/h
  * @brief      工程机械臂配套自定义控制器功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-22-2024     Penguin         1. done
  *  V1.0.1     Jan-14-2025     Penguin         1. 能获取关节的位置
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "custom_controller_engineer.h"

#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_ENGINEER)

#include "CAN_communication.h"
#include "cmsis_os.h"
#include "custom_controller.h"
#include "math.h"
#include "string.h"
#include "usb_debug.h"
#include "user_lib.h"

/*------------------------------ Macro Definition ------------------------------*/

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5

#define JointMotorInit(index)                                                                      \
    MotorInit(                                                                                     \
        &CUSTOM_CONTROLLER.joint_motor[index], JOINT_MOTOR_##index##_ID,                           \
        JOINT_MOTOR_##index##_CAN, JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION, 1, \
        JOINT_MOTOR_##index##_MODE)

#define JointPidInit(index)                                                             \
    {                                                                                   \
        float j##index##_pid_velocity[3] = {                                            \
            KP_JOINT_##index##_VELOCITY, KI_JOINT_##index##_VELOCITY,                   \
            KD_JOINT_##index##_VELOCITY};                                               \
        PID_init(                                                                       \
            &CUSTOM_CONTROLLER.pid.joint[index], PID_POSITION, j##index##_pid_velocity, \
            MAX_OUT_JOINT_##index##_VELOCITY, MAX_IOUT_JOINT_##index##_VELOCITY);       \
    }
#define JointLpfInit(index) \
    LowPassFilterInit(&CUSTOM_CONTROLLER.lpf.joint[index], J##index##_LPF_ALPHA)

/*------------------------------ Variable Definition ------------------------------*/

CustomController_s CUSTOM_CONTROLLER;

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerPublish                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerPublish(void) {}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerInit                       */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerInit(void)
{
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
    JointLpfInit(0);
    JointLpfInit(1);
    JointLpfInit(2);
    JointLpfInit(3);
    JointLpfInit(4);
    JointLpfInit(5);
    // #limit init ---------------------
    CUSTOM_CONTROLLER.limit.max.pos[J0] = MAX_JOINT_0_POSITION;
    CUSTOM_CONTROLLER.limit.max.pos[J1] = MAX_JOINT_1_POSITION;
    CUSTOM_CONTROLLER.limit.max.pos[J2] = MAX_JOINT_2_POSITION;
    CUSTOM_CONTROLLER.limit.max.pos[J3] = MAX_JOINT_3_POSITION;
    CUSTOM_CONTROLLER.limit.max.pos[J4] = MAX_JOINT_4_POSITION;
    CUSTOM_CONTROLLER.limit.max.pos[J5] = MAX_JOINT_5_POSITION;
    CUSTOM_CONTROLLER.limit.max.vj4_pos = MAX_VIRTUAL_JOINT_4_POSITION;
    CUSTOM_CONTROLLER.limit.max.vj5_pos = MAX_VIRTUAL_JOINT_5_POSITION;

    CUSTOM_CONTROLLER.limit.min.pos[J0] = MIN_JOINT_0_POSITION;
    CUSTOM_CONTROLLER.limit.min.pos[J1] = MIN_JOINT_1_POSITION;
    CUSTOM_CONTROLLER.limit.min.pos[J2] = MIN_JOINT_2_POSITION;
    CUSTOM_CONTROLLER.limit.min.pos[J3] = MIN_JOINT_3_POSITION;
    CUSTOM_CONTROLLER.limit.min.pos[J4] = MIN_JOINT_4_POSITION;
    CUSTOM_CONTROLLER.limit.min.pos[J5] = MIN_JOINT_5_POSITION;
    CUSTOM_CONTROLLER.limit.min.vj4_pos = MIN_VIRTUAL_JOINT_4_POSITION;
    CUSTOM_CONTROLLER.limit.min.vj5_pos = MIN_VIRTUAL_JOINT_5_POSITION;

    // #Initial value setting ---------------------
    memset(&CUSTOM_CONTROLLER.fdb, 0, sizeof(CUSTOM_CONTROLLER.fdb));  // 反馈量置零
    memset(&CUSTOM_CONTROLLER.ref, 0, sizeof(CUSTOM_CONTROLLER.ref));  // 目标量置零
    CUSTOM_CONTROLLER.mode = CUSTOM_CONTROLLER_DRAGGING;
    CUSTOM_CONTROLLER.error_code = 0;
    CUSTOM_CONTROLLER.transform.pos[J0] = J0_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J1] = J1_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J2] = J2_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J3] = J3_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J4] = J4_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J5] = J5_ANGLE_TRANSFORM;
}

/******************************************************************/
/* HandleException                                                */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerHandleException            */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerHandleException(void) {}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerSetMode                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerSetMode(void) { CUSTOM_CONTROLLER.mode = CUSTOM_CONTROLLER_DRAGGING; }

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerObserver                   */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerObserver(void)
{
    static float last_pos[6], pos_fdb[6] = {0, 0, 0, 0, 0, 0};
    float dpos;
    uint8_t i;
    // 更新电机测量数据
    for (i = 0; i < JOINT_NUM; i++) {
        GetMotorMeasure(&CUSTOM_CONTROLLER.joint_motor[i]);
    }
    // 获取观测值
    for (i = 0; i < JOINT_NUM; i++) {
        CUSTOM_CONTROLLER.fdb.joint[i].vel = CUSTOM_CONTROLLER.joint_motor[i].fdb.vel;

        // 处理多圈计数问题
        pos_fdb[i] = theta_transform(
            CUSTOM_CONTROLLER.joint_motor[i].fdb.pos, CUSTOM_CONTROLLER.transform.pos[i],
            CUSTOM_CONTROLLER.joint_motor[i].direction, 1);

        dpos = pos_fdb[i] - last_pos[i];
        if (fabs(dpos) > M_PI) {
            CUSTOM_CONTROLLER.fdb.joint[i].round += (dpos) < 0 ? 1 : -1;
        }
        last_pos[i] = pos_fdb[i];
        CUSTOM_CONTROLLER.fdb.joint[i].pos =
            (pos_fdb[i] + M_PI * 2 * CUSTOM_CONTROLLER.fdb.joint[i].round) /
            CUSTOM_CONTROLLER.joint_motor[i].reduction_ratio;
    }

    // 更新机械臂控制数据
    // clang-format off
    cc_control_data.pos[J0] = fp32_constrain(
        CUSTOM_CONTROLLER.fdb.joint[J0].pos, 
        CUSTOM_CONTROLLER.limit.min.pos[J0],
        CUSTOM_CONTROLLER.limit.max.pos[J0]);
    cc_control_data.pos[J1] = fp32_constrain(
        CUSTOM_CONTROLLER.fdb.joint[J1].pos, 
        CUSTOM_CONTROLLER.limit.min.pos[J1],
        CUSTOM_CONTROLLER.limit.max.pos[J1]);
    cc_control_data.pos[J2] = fp32_constrain(
        CUSTOM_CONTROLLER.fdb.joint[J2].pos,
        CUSTOM_CONTROLLER.limit.min.pos[J2],
        CUSTOM_CONTROLLER.limit.max.pos[J2]);
    cc_control_data.pos[J3] = fp32_constrain(
        CUSTOM_CONTROLLER.fdb.joint[J3].pos, 
        CUSTOM_CONTROLLER.limit.min.pos[J3],
        CUSTOM_CONTROLLER.limit.max.pos[J3]);
    
    float vj4_pos = fp32_constrain(CUSTOM_CONTROLLER.fdb.joint[J4].pos, CUSTOM_CONTROLLER.limit.min.vj4_pos, CUSTOM_CONTROLLER.limit.max.vj4_pos);
    float vj5_pos = fp32_constrain(CUSTOM_CONTROLLER.fdb.joint[J5].pos, CUSTOM_CONTROLLER.limit.min.vj5_pos, CUSTOM_CONTROLLER.limit.max.vj5_pos);
    
    cc_control_data.pos[J4] =   vj4_pos + vj5_pos;
    cc_control_data.pos[J5] = -(vj4_pos - vj5_pos);
    // clang-format on
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerReference                  */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerReference(void) {}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerConsole                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerConsole(void)
{
    uint8_t i;
    // 计算控制量
    for (i = 0; i < JOINT_NUM; i++) {
        // CUSTOM_CONTROLLER.joint_motor[i].set.value = PID_calc(
        //     &CUSTOM_CONTROLLER.pid.joint[i], CUSTOM_CONTROLLER.fdb.joint[i].vel,
        //     CUSTOM_CONTROLLER.ref.joint[i].vel);
        CUSTOM_CONTROLLER.joint_motor[i].set.value = 0;
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerSendCmd                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerSendCmd(void)
{
    // clang-format off
    CanCmdDjiMotor(
        1, DJI_6020_MODE_VOLTAGE_1, 
        CUSTOM_CONTROLLER.joint_motor[0].set.value,
        CUSTOM_CONTROLLER.joint_motor[1].set.value, 
        CUSTOM_CONTROLLER.joint_motor[2].set.value, 0);
    CanCmdDjiMotor(
        2, DJI_3508_MODE_CURRENT_1, 
        CUSTOM_CONTROLLER.joint_motor[3].set.value,
        CUSTOM_CONTROLLER.joint_motor[4].set.value, 
        CUSTOM_CONTROLLER.joint_motor[5].set.value, 0);
    // clang-format on
}

#endif  // CUSTOM_CONTROLLER_TYPE
/*------------------------------ End of File ------------------------------*/
