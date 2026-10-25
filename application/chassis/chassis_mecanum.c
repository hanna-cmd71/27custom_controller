/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_mecanum.c/h
  * @brief      麦轮轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-9-2024      Tina_Lin         1. done
  *  V1.0.1     Dec-9-2024      Tina_Lin         1. 完成基本控制
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/


#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#include "chassis_mecanum.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"

Motor_s __Motor;
Chassis_s CHASSIS;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    /*-------------------- 初始化PID --------------------*/
    //速度环pid
    float chassis_speed_pid[3] = {KP_CHASSIS_WHEEL_SPEED, KI_CHASSIS_WHEEL_SPEED, KD_CHASSIS_WHEEL_SPEED};
    //yaw轴跟踪pid
    float chassis_yaw_pid[3] = {KP_CHASSIS_GIMBAL_FOLLOW_ANGLE, KI_CHASSIS_GIMBAL_FOLLOW_ANGLE, KD_CHASSIS_GIMBAL_FOLLOW_ANGLE};;
    
    //获取底盘电机数据指针，初始化PID 
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        //底盘不跟随云台下的pid初始化
        PID_init(&CHASSIS.motor_speed_pid[i], PID_POSITION, chassis_speed_pid, MAX_OUT_CHASSIS_WHEEL_SPEED,MAX_IOUT_CHASSIS_WHEEL_SPEED);
    }
    //底盘跟随云台pid初始化
        PID_init(&CHASSIS.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, 
        MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE,MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE);
    
    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.wheel_motor[0], 1, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[1], 2, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[2], 3, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[3], 4, 1, DJI_M3508, 1, 1, 1);
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
// void ChassisSetMode(void)
// {
//     if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
//         CHASSIS.mode = CHASSIS_SPIN;
//     } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL]) && (GetGimbalInitJudgeReturn())) {
//         CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
//     } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL]) || !(GetGimbalInitJudgeReturn())) {
//         CHASSIS.mode = ROBO_ZERO_FORCE;
//     }
// }


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) {
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void) {

#if (CONTROL_TYPE == SINGLE_CONTROL)
    chassis.reference.vx = GetDt7RcCh(0) * RC_TO_VECTOR_SCALE;
    chassis.reference.vy = GetDt7RcCh(1) * RC_TO_VECTOR_SCALE;
    chassis.reference.chassis_mode = GetDt7RcSw(0);

#elif (CONTROL_TYPE == DOUBLE_CONTROL)
    Get_board_communication_information(&chassis.reference);
#endif

    fp32 rc_x, rc_y;
    fp32 vx_channel, vy_channel;
    rc_deadband_limit(CHASSIS.rc->rc.ch[3], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[2], rc_y, CHASSIS_RC_DEADLINE);
    //rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);

    vx_channel = rc_x * MAX_SPEED_VECTOR_VX;
    vy_channel = rc_y * MAX_SPEED_VECTOR_VY;

    CHASSIS.dyaw = GetGimbalDeltaYawMid();

    //给定摇杆值(无死区)
    // CHASSIS.vx_rc_set = CHASSIS_VX_RC_SEN * CHASSIS.rc->rc.ch[3];
    // CHASSIS.vy_rc_set = CHASSIS_VY_RC_SEN * CHASSIS.rc->rc.ch[2];

    CHASSIS.vx_rc_set = vx_channel;
    CHASSIS.vy_rc_set = vy_channel;

    uint8_t i;
    //具体模式设定
    switch (CHASSIS.mode) {
        case ROBO_ZERO_FORCE: { 
            CHASSIS.wz_set = CHASSIA_STOP_SPEED;
            CHASSIS.vx_set = CHASSIA_STOP_SPEED;
            CHASSIS.vy_set = CHASSIA_STOP_SPEED;
            break;
        }
        case ROBO_CHASSIS_FOLLOW_GIMBAL_YAW:{//云台跟随模式

            //GimbalSpeedVectorToChassisSpeedVector();
            fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	        // 控制vx vy
	        sin_yaw = sinf(CHASSIS.dyaw);
	        cos_yaw = cosf(CHASSIS.dyaw);
            CHASSIS.vy_set = cos_yaw * CHASSIS.vy_rc_set - sin_yaw * CHASSIS.vx_rc_set;
	        CHASSIS.vx_set = sin_yaw * CHASSIS.vy_rc_set + cos_yaw * CHASSIS.vx_rc_set;

            CHASSIS.ref.speed_vector.wz = CHASSIS.dyaw; 
            CHASSIS.wz_set = PID_calc(&CHASSIS.chassis_angle_pid, -CHASSIS.dyaw, 0);//反转dyaw角度
            break;
        }

        
        case ROBO_AUTO_WITH_SPIN:{//小陀螺模式

            //GimbalSpeedVectorToChassisSpeedVector();
            fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	        // 控制vx vy
	        sin_yaw = sinf(CHASSIS.dyaw);
	        cos_yaw = cosf(CHASSIS.dyaw);
            CHASSIS.vy_set = cos_yaw * CHASSIS.vy_rc_set - sin_yaw * CHASSIS.vx_rc_set;
	        CHASSIS.vx_set = sin_yaw * CHASSIS.vy_rc_set + cos_yaw * CHASSIS.vx_rc_set;
            
			CHASSIS.wz_set = NORMAL_MAX_CHASSIS_SPEED_WX;
            break;
        }
        case ROBO_AUTO:
            break;

        }
        default:
            break;
    }

    
    //GimbalSpeedVectorToChassisSpeedVector();
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	// 控制vx vy
	sin_yaw = sinf(CHASSIS.dyaw);
	cos_yaw = cosf(CHASSIS.dyaw);
    CHASSIS.vy_set = cos_yaw * CHASSIS.vy_rc_set - sin_yaw * CHASSIS.vx_rc_set;
	CHASSIS.vx_set = sin_yaw * CHASSIS.vy_rc_set + cos_yaw * CHASSIS.vx_rc_set;

    //键盘控制
    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_W)
    {
        CHASSIS.vx_rc_set = NORMAL_MAX_CHASSIS_SPEED_X;
    }
    else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_S)
    {
        CHASSIS.vx_rc_set = -NORMAL_MAX_CHASSIS_SPEED_X;
    }

    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_A)
    {
        CHASSIS.vy_rc_set = NORMAL_MAX_CHASSIS_SPEED_Y;
    }
    else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_D)
    {
        CHASSIS.vy_rc_set = -NORMAL_MAX_CHASSIS_SPEED_Y;
    }
    /*int spinflag = 0; //标定小陀螺状态
    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_R)//按R键启用小陀螺
    {
        spinflag = 1;
    }
    else if(CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_F)//按F键关闭小陀螺
    {
        spinflag = 0;
    }
    if (spinflag){
        CHASSIS.wz_set = NORMAL_MAX_CHASSIS_SPEED_WX;
    }*/
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    uint8_t i;

    // 判断是否出错，若出错则将电流全部置零
    if (toe_is_error(DBUS_TOE))
    {
        for (i = 0; i < 4; i++)
        {
            CHASSIS.wheel_motor[i].set.curr = CHASSIA_CURR_ZERO; 
        }
        return;
    }
    
    //麦轮解算
    CHASSIS.wheel_motor[0].set.vel = -CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[1].set.vel =  CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[2].set.vel =  CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[3].set.vel = -CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set; 

    //pid速度计算            
    for (i = 0; i < 4; i++)
    {
        CHASSIS.wheel_motor[i].set.curr =
        PID_calc(&CHASSIS.motor_speed_pid[i], CHASSIS.wheel_motor[i].fdb.vel, CHASSIS.wheel_motor[i].set.vel);
    }

}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void)
{

    CanCmdDjiMotor(2, 0x200, 
    CHASSIS.wheel_motor[0].set.curr, CHASSIS.wheel_motor[1].set.curr,
    CHASSIS.wheel_motor[2].set.curr, CHASSIS.wheel_motor[3].set.curr);

}

#endif //CHASSIS_OMNI_WHEEL
