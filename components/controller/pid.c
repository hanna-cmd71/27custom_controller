/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"

#include "main.h"

#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }




/**
 * @brief 初始化低通滤波器
 * @param filter 滤波器指针
 * @param alpha 滤波系数 (0-1)
 *          0.01: 强滤波，慢响应
 *          0.1:  中等滤波
 *          0.3:  弱滤波，快响应
 * @param initial_value 初始值
 */
void lowpass_init(LowPassFilter* filter, float alpha, float initial_value) {
    // 确保alpha在有效范围内
    if (alpha <= 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    filter->alpha = alpha;
    filter->output = initial_value;
    filter->initialized = 1;
}

/**
 * @brief 更新低通滤波器
 * @param filter 滤波器指针
 * @param input 新输入值
 * @return 滤波后的输出值
 */
float lowpass_update(LowPassFilter* filter, float input) {
    if (!filter->initialized) {
        // 如果未初始化，使用输入值作为初始输出
        filter->output = input;
        filter->initialized = 1;
        return input;
    }
    
    // 一阶低通滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    
    return filter->output;
}

/**
 * @brief 重置滤波器（清除历史状态）
 * @param filter 滤波器指针
 * @param reset_value 重置后的值（可选）
 */
void lowpass_reset(LowPassFilter* filter, float reset_value) {
    filter->output = reset_value;
    filter->initialized = 1;
}

/**
 * @brief 获取当前滤波输出（不更新）
 * @param filter 滤波器指针
 * @return 当前滤波输出值
 */
float lowpass_get_output(LowPassFilter* filter) {
    return filter->output;
}

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def * pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL) {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->N = 0.0f;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out =
        0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def * pid, fp32 ref, fp32 set)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        // 对 Dbuf[0] 进行低通滤波
        pid->Dbuf[0] = pid->N * pid->Dbuf[1] + (1.0f - pid->N) * (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def * pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/* ============== 以下为测试中的全新PID算法 ============== */

/**
  * @brief          测试内容
  */
void SinglePidParamUpdate(Pid_t * pid, fp32 param[4])
{
    pid->Kp = param[0];
    pid->Ki = param[1];
    pid->Kd = param[2];
    pid->N = param[3];
}

/**
  * @brief          测试内容
  */
void SinglePidInit(Pid_t * pid, fp32 param[4], fp32 max[2])
{
    SinglePidParamUpdate(pid, param);
    pid->max_out = max[0];
    pid->max_iout = max[1];

    pid->ref = 0;
    pid->fdb = 0;

    pid->out = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
}

/**
  * @brief          测试内容
  */
void SinglePidCalc(Pid_t * pid, fp32 error, fp32 dt)
{
    pid->Pout = pid->Kp * error;
    pid->Iout += pid->Ki * error * dt;
    pid->Dout = pid->Kd * (pid->N * pid->Pout - pid->Dout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;

    if (pid->out > pid->max_out) {
        pid->out = pid->max_out;
    } else if (pid->out < -pid->max_out) {
        pid->out = -pid->max_out;
    }
    if (pid->Iout > pid->max_iout) {
        pid->Iout = pid->max_iout;
    } else if (pid->Iout < -pid->max_iout) {
        pid->Iout = -pid->max_iout;
    }
}
/*------------------------------ End of File ------------------------------*/
