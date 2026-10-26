/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       terminal_debug.c
  * @brief      自定义终端调试快照的定义（声明见 terminal_debug.h）。
  * @note       刻意不放在 #if (CUSTOM_CONTROLLER_TYPE == ...) 里：
  *             CAN 驱动等模块会直接写这个结构，任何变体下都应有符号。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2026     -               1. 从 custom_terminal.c 拆出。
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "terminal_debug.h"

volatile terminal_debug_t terminal_dbg;

/************************ END OF FILE ************************/
