/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "CAN_receive.h"
#include "gimbal_yaw_pitch_direct.h"
#include "data_exchange.h"
#include "communication.h"

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
Gimbal_PID_t gimbal_direct_pid;
static const PidGetVofa_t *pid_get_vofa;
#define ROBO_INIT_TIME 10 // (秒)云台初始化时间
/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/
/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */
void Angle_solution(void)
{
    float motor_feedback = gimbal_direct.pitch.fdb.pos;    // 当前电机位置
    float imu_feedback = gimbal_direct.feedback_pos.pitch; // 当前IMU角度
    float motor_mid = GIMBAL_DIRECT_PITCH_MID;             // 电机中值位置
    float imu_mid = 0.0;                                   // IMU中值(设为0)

    float motor_delta = GIMBAL_DIRECT_PITCH_DIRECTION * (motor_feedback - motor_mid);
    float imu_delta = imu_feedback - imu_mid;

    // 当电机在中值时，IMU的弧度
    gimbal_direct.pitch_angle_zero_for_imu = imu_delta - motor_delta;

    motor_feedback = gimbal_direct.yaw.fdb.pos;    // 当前电机位置
    imu_feedback = gimbal_direct.feedback_pos.yaw; // 当前IMU角度
    motor_mid = GIMBAL_DIRECT_YAW_MID;             // 电机中值位置
    imu_mid = 0.0;                                 // IMU中值(设为0)

    motor_delta = GIMBAL_DIRECT_YAW_DIRECTION * (motor_feedback - motor_mid);
    imu_delta = imu_feedback - imu_mid;

    // 当电机在中值时，IMU的弧度
    gimbal_direct.yaw_angle_zero_for_imu = imu_delta - motor_delta;
}

/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准,false为继续初始化，true为结束初始化
 * @param[in]      none
 * @retval         bool 是否需要继续初始化
 */
bool Gimbal_direct_init_judge(void)
{
    // 判断条件1: 位置精度达到要求 (±0.003弧度)
    bool position_reached =
        (fabs(gimbal_direct.reference.yaw - gimbal_direct.yaw.fdb.pos) < 0.003f) &&
        (fabs(gimbal_direct.reference.pitch - gimbal_direct.pitch.fdb.pos) < 0.003f);

    // 判断条件2: 超时保护 (10秒)
    bool timeout_reached = (gimbal_direct.init_timer >= ROBO_INIT_TIME);

    return position_reached || timeout_reached;
}

/*-------------------------The end of internal functions--------------------------------------*/

/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
    return loop_fp32_constrain(gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID, -M_PI, M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
    return gimbal_direct.init_continue;
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
    if (axis == AX_PITCH)
    {
        return loop_fp32_constrain(gimbal_direct.pitch.direction * (gimbal_direct.pitch.fdb.pos - GIMBAL_DIRECT_PITCH_MID), -M_PI, M_PI);
    }
    else if (axis == AX_YAW)
    {
        return loop_fp32_constrain(gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID), -M_PI, M_PI);
    }
    else
    {
        return 0.0;
    }
}

/*-------------------- Public --------------------*/

/**
 * @brief          发布数据
 * @param[in]      none
 * @retval         none
 */
void GimbalPublish(void)
{
    // 发布PID调试数据
    Publish(&gimbal_direct_pid, GIMBAL_PID_NAME);
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void)
{
    // 获取PID参数指针
    pid_get_vofa = Subscribe(PID_GET_VOFA_NAME);

    // step1 获取所有所需变量指针
    gimbal_direct.rc = get_remote_control_point();
    // step2 置零所有值
    gimbal_direct.reference.pitch = 0;
    gimbal_direct.reference.yaw = 0;

    gimbal_direct.feedback_pos.pitch = 0;
    gimbal_direct.feedback_pos.yaw = 0;

    gimbal_direct.feedback_vel.pitch = 0;
    gimbal_direct.feedback_vel.yaw = 0;

    gimbal_direct.upper_limit.pitch = GIMBAL_UPPER_LIMIT_PITCH;

    gimbal_direct.lower_limit.pitch = GIMBAL_LOWER_LIMIT_PITCH;
    // step3 PID数据清零，设置PID参数
    const static fp32 gimbal_yaw_angle[3] = {KP_GIMBAL_YAW_ANGLE, KI_GIMBAL_YAW_ANGLE, KD_GIMBAL_YAW_ANGLE};
    const static fp32 gimbal_yaw_velocity[3] = {KP_GIMBAL_YAW_VELOCITY, KI_GIMBAL_YAW_VELOCITY, KD_GIMBAL_YAW_VELOCITY};

    const static fp32 gimbal_pitch_angle[3] = {KP_GIMBAL_PITCH_ANGLE, KI_GIMBAL_PITCH_ANGLE, KD_GIMBAL_PITCH_ANGLE};
    const static fp32 gimbal_pitch_velocity[3] = {KP_GIMBAL_PITCH_VELOCITY, KI_GIMBAL_PITCH_VELOCITY, KD_GIMBAL_PITCH_VELOCITY};

    PID_init(&gimbal_direct_pid.yaw_angle, PID_POSITION, gimbal_yaw_angle, MAX_OUT_GIMBAL_YAW_ANGLE, MAX_IOUT_GIMBAL_YAW_ANGLE);
    PID_init(&gimbal_direct_pid.yaw_velocity, PID_POSITION, gimbal_yaw_velocity, MAX_OUT_GIMBAL_YAW_VELOCITY, MAX_IOUT_GIMBAL_YAW_VELOCITY);

    PID_init(&gimbal_direct_pid.pitch_angle, PID_POSITION, gimbal_pitch_angle, MAX_OUT_GIMBAL_PITCH_ANGLE, MAX_IOUT_GIMBAL_PITCH_ANGLE);
    PID_init(&gimbal_direct_pid.pitch_velocity, PID_POSITION, gimbal_pitch_velocity, MAX_OUT_GIMBAL_PITCH_VELOCITY, MAX_IOUT_GIMBAL_PITCH_VELOCITY);

    // step4 初始化电机
    MotorInit(&gimbal_direct.yaw, GIMBAL_DIRECT_YAW_ID, GIMBAL_CAN, GIMBAL_DIRECT_YAW_MOTOR_TYPE, GIMBAL_DIRECT_YAW_DIRECTION, GIMBAL_DIRECT_YAW_REDUCTION_RATIO, GIMBAL_DIRECT_YAW_MODE);
    MotorInit(&gimbal_direct.pitch, GIMBAL_DIRECT_PITCH_ID, GIMBAL_CAN, GIMBAL_DIRECT_PITCH_MOTOR_TYPE, GIMBAL_DIRECT_PITCH_DIRECTION, GIMBAL_DIRECT_PITCH_REDUCTION_RATIO, GIMBAL_DIRECT_PITCH_MODE);

    // step5 初始化云台初始化校准相关变量
    gimbal_direct.init_start_time = 0;
    gimbal_direct.init_timer = 0;
    gimbal_direct.init_continue = false;

    // step6 模式设置初始化
    gimbal_direct.mode = GIMBAL_ZERO_FORCE;
    gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
    gimbal_direct.mode_before_rc_err = GIMBAL_ZERO_FORCE;
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
    // 优先级1: 遥控器断联 (最高优先级)
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_direct.mode = GIMBAL_DBUS_ERR;
    }

    // 优先级2: 从断联恢复
    else if (gimbal_direct.last_mode == GIMBAL_DBUS_ERR)
    {
        gimbal_direct.mode = gimbal_direct.mode_before_rc_err;
    }

    // 优先级3: 安全模式 (下档位)
    else if ((switch_is_down(gimbal_direct.rc->rc.s[0]))) // 安全档优先级最高
    {
        gimbal_direct.mode = GIMBAL_ZERO_FORCE;
        gimbal_direct.init_continue = false;
    }

    // 优先级4: 初始化校准流程
    else if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == ROBO_INIT)
    {

        gimbal_direct.mode = ROBO_INIT;

        gimbal_direct.init_continue = Gimbal_direct_init_judge();
        if (gimbal_direct.init_continue == true) // 判断是否需要跳出循环
        {
            gimbal_direct.mode = GIMBAL_GAP; // 跳转到过渡模式
        }
    }

    // 优先级5: 手动控制 (中档位)
    else if (switch_is_mid(gimbal_direct.rc->rc.s[0]))
    {
        gimbal_direct.mode = GIMBAL_ABSOLUTE_ANGLE;
    }

    // 优先级6: 自瞄模式 (上档位)
    else if (switch_is_up(gimbal_direct.rc->rc.s[0]))
    {
        gimbal_direct.mode = GIMBAL_AUTO_AIM;
    }
}
/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void)
{
    // 电机相关数据更新
    GetMotorMeasure(&gimbal_direct.yaw);
    GetMotorMeasure(&gimbal_direct.pitch);

    // IMU相关数据更新
    gimbal_direct.feedback_pos.pitch = GetImuAngle(AX_PITCH);
    gimbal_direct.feedback_pos.yaw = GetImuAngle(AX_YAW);

    gimbal_direct.feedback_vel.pitch = GetImuVelocity(AX_PITCH);
    gimbal_direct.feedback_vel.yaw = GetImuVelocity(AX_YAW);

    // 坐标系映射更新 (关键!)
    Angle_solution();

    // 初始化计时器管理
    if (gimbal_direct.mode == ROBO_INIT)
    {
        if (gimbal_direct.last_mode != ROBO_INIT)
        {
            gimbal_direct.init_start_time = xTaskGetTickCount(); // 设置初始化开始时间
        }
        // 设置初始化进行时间
        gimbal_direct.init_timer = xTaskGetTickCount() - gimbal_direct.init_start_time;
    }
    else
    {
        gimbal_direct.init_timer = 0;
    }

    // 断线恢复模式记录
    if (gimbal_direct.mode == GIMBAL_DBUS_ERR && gimbal_direct.last_mode != GIMBAL_DBUS_ERR)
    {
        gimbal_direct.mode_before_rc_err = gimbal_direct.last_mode;
    }

    gimbal_direct.last_mode = gimbal_direct.mode; // 上一运行模式更新

    switch (__TUNING_MODE)
    {
    case TUNING_GIMBAL_PITCH:
        gimbal_direct_pid.pitch_angle.Kp = pid_get_vofa->angle_kp;
        gimbal_direct_pid.pitch_angle.Ki = pid_get_vofa->angle_ki;
        gimbal_direct_pid.pitch_angle.Kd = pid_get_vofa->angle_kd;

        gimbal_direct_pid.pitch_angle.max_iout = pid_get_vofa->angle_max_iout;
        gimbal_direct_pid.pitch_angle.max_out = pid_get_vofa->angle_max_out;

        gimbal_direct_pid.pitch_velocity.Kp = pid_get_vofa->speed_kp;
        gimbal_direct_pid.pitch_velocity.Ki = pid_get_vofa->speed_ki;
        gimbal_direct_pid.pitch_velocity.Kd = pid_get_vofa->speed_kd;

        gimbal_direct_pid.pitch_velocity.max_iout = pid_get_vofa->speed_max_iout;
        gimbal_direct_pid.pitch_velocity.max_out = pid_get_vofa->speed_max_out;
        break;

    case TUNING_GIMBAL_YAW:
        gimbal_direct_pid.yaw_angle.Kp = pid_get_vofa->angle_kp;
        gimbal_direct_pid.yaw_angle.Ki = pid_get_vofa->angle_ki;
        gimbal_direct_pid.yaw_angle.Kd = pid_get_vofa->angle_kd;

        gimbal_direct_pid.yaw_angle.max_iout = pid_get_vofa->angle_max_iout;
        gimbal_direct_pid.yaw_angle.max_out = pid_get_vofa->angle_max_out;

        gimbal_direct_pid.yaw_velocity.Kp = pid_get_vofa->speed_kp;
        gimbal_direct_pid.yaw_velocity.Ki = pid_get_vofa->speed_ki;
        gimbal_direct_pid.yaw_velocity.Kd = pid_get_vofa->speed_kd;

        gimbal_direct_pid.yaw_velocity.max_iout = pid_get_vofa->speed_max_iout;
        gimbal_direct_pid.yaw_velocity.max_out = pid_get_vofa->speed_max_out;
        break;
    default:
        break;
    }
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void)
{
    if (gimbal_direct.mode == ROBO_INIT)
    {
        // 计算让电机回到中值对应的IMU目标角度
        gimbal_direct.reference.pitch = loop_fp32_constrain(
            gimbal_direct.pitch.direction * (GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos) + gimbal_direct.feedback_pos.pitch, -M_PI, M_PI);
        gimbal_direct.reference.yaw = loop_fp32_constrain(
            gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos) + gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
    }

    // 过渡模式保持当前位置不动
    else if (gimbal_direct.mode == GIMBAL_GAP)
    {
        gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
    }

    else if (gimbal_direct.mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        // 模式切换时: 保持当前位置
        if (gimbal_direct.last_mode != GIMBAL_ABSOLUTE_ANGLE)
        {
            gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
            gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
        }

        else
        {
            // PITCH轴控制 (摇杆CH3)
            gimbal_direct.reference.pitch = fp32_constrain(
                gimbal_direct.reference.pitch +
                    fp32_deadline(GetDt7RcCh(3),
                                  REMOTE_CONTROLLER_MIN_DEADLINE,
                                  REMOTE_CONTROLLER_MAX_DEADLINE) /
                        REMOTE_CONTROLLER_SENSITIVITY_PITCH,
                GIMBAL_LOWER_LIMIT_PITCH - GIMBAL_DIRECT_PITCH_MID,
                GIMBAL_UPPER_LIMIT_PITCH - GIMBAL_DIRECT_PITCH_MID);

            // YAW轴控制 (摇杆CH2)
            if (GIMBAL_YAW_LIMIT == 0)
            {
                gimbal_direct.reference.yaw = loop_fp32_constrain(
                    gimbal_direct.reference.yaw -
                        fp32_deadline(GetDt7RcCh(2),
                                      REMOTE_CONTROLLER_MIN_DEADLINE,
                                      REMOTE_CONTROLLER_MAX_DEADLINE) /
                            REMOTE_CONTROLLER_SENSITIVITY_YAW,
                    -M_PI, M_PI);
            }
            else
            {
                gimbal_direct.reference.yaw = fp32_constrain(
                    gimbal_direct.reference.yaw -
                        fp32_deadline(GetDt7RcCh(2),
                                      REMOTE_CONTROLLER_MIN_DEADLINE,
                                      REMOTE_CONTROLLER_MAX_DEADLINE) /
                            REMOTE_CONTROLLER_SENSITIVITY_YAW,
                GIMBAL_LOWER_LIMIT_YAW - GIMBAL_DIRECT_YAW_MID ,
                GIMBAL_UPPER_LIMIT_YAW - GIMBAL_DIRECT_YAW_MID );
            }
        }
    }

    // 保持当前位置
    else if (gimbal_direct.mode == GIMBAL_AUTO_AIM)
    {
        gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
        // gimbal_direct.reference.pitch = fp32_constrain(Gimbal_direct_ecd_to_imu(AX_PITCH, GetScCmdGimbalAngle(AX_PITCH)), GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.pitch_angle_zero_for_imu, GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.pitch_angle_zero_for_imu);
        // gimbal_direct.reference.yaw = loop_fp32_constrain(Gimbal_direct_ecd_to_imu(AX_YAW, GetScCmdGimbalAngle(AX_YAW)), -M_PI, M_PI);
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void)
{
    if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR)
    {
        // 安全模式: 输出零电流
        gimbal_direct.pitch.set.curr = 0;
        gimbal_direct.yaw.set.curr = 0;
    }
    else if (gimbal_direct.mode == GIMBAL_ABSOLUTE_ANGLE ||
             gimbal_direct.mode == GIMBAL_GAP ||
             gimbal_direct.mode == GIMBAL_AUTO_AIM ||
             gimbal_direct.mode == ROBO_INIT)
    {
        // PITCH轴双环控制
        gimbal_direct.pitch.set.vel = PID_calc(
            &gimbal_direct_pid.pitch_angle,
            gimbal_direct.feedback_pos.pitch, // 当前IMU角度
            gimbal_direct.reference.pitch);   // 目标IMU角度

        gimbal_direct.pitch.set.curr = gimbal_direct.pitch.direction *
                                       PID_calc(&gimbal_direct_pid.pitch_velocity,
                                                gimbal_direct.feedback_vel.pitch, // 当前IMU角速度
                                                gimbal_direct.pitch.set.vel);     // 目标角速度

        // YAW轴双环控制
        if (GIMBAL_YAW_LIMIT == 0)
        {
            // 如果yaw轴没有限位
            fp32 delta_yaw = loop_fp32_constrain(
                gimbal_direct.reference.yaw - gimbal_direct.feedback_pos.yaw,
                -M_PI, M_PI); // 处理±180°跨越

            gimbal_direct.yaw.set.vel = PID_calc(
                &gimbal_direct_pid.yaw_angle, 0, delta_yaw);
        }
        else
        {
            gimbal_direct.yaw.set.vel = PID_calc(
                &gimbal_direct_pid.yaw_angle,
                gimbal_direct.feedback_pos.yaw,
                gimbal_direct.reference.yaw);
        }

        gimbal_direct.yaw.set.curr = gimbal_direct.yaw.direction *
                                     PID_calc(&gimbal_direct_pid.yaw_velocity,
                                              gimbal_direct.feedback_vel.yaw,
                                              gimbal_direct.yaw.set.vel);
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{
    // 添加电机到CAN管理器
    CanManagerAddMotor(gimbal_direct.pitch.id + 4, GIMBAL_CAN, gimbal_direct.pitch.set.curr);
    CanManagerAddMotor(gimbal_direct.yaw.id + 4, GIMBAL_CAN, gimbal_direct.yaw.set.curr);
}

#endif // GIMBAL_YAW_PITCH
