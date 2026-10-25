/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       calibrate_task.c/h
  * @brief      校准任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-27-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

extern void cali_param_init(void);

extern int8_t get_control_temperature(void);

extern void calibrate_task(void const *pvParameters);

#endif // CALIBRATE_TASK_H
