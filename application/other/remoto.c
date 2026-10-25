// #include "remoto.h"
// #include "main.h"
// #include "bsp_usart.h"
// #include "robot_param.h"
// #include "bsp_usart.h"
// extern UART_HandleTypeDef huart1;

// uint8_t aba1;

// void remoto_taks()
// {
//     aba1 = 200;

//     while (1)
//     {
//         HAL_UART_Transmit_IT(&huart1, &aba1, sizeof(aba1));
//         HAL_Delay(10);
//     }
// }