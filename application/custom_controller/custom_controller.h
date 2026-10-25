
#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include "struct_typedef.h"

#define CUSTOM_CONTROLLER_TASK_INIT_TIME 100
#define CUSTOM_CONTROLLER_CONTROL_TIME 1000
#define CUSTOM_CONTROLLER_SEND_TIME 34  //30Hz = (1000/30)ms 向上取整

typedef struct
{
    void (*Blank)(void);
} CustomControllerApi_t;

typedef struct
{
    float pos[7];
} CCControlData_t;

extern CustomControllerApi_t custom_controller;
extern CCControlData_t cc_control_data;

#endif /* CUSTOM_CONTROLLER_H */
/*------------------------------ End of File ------------------------------*/
