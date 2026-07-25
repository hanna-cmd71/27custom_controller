#include "custom_controller.h"

#include <stdlib.h>

CustomControllerApi_t custom_controller = {
    .Blank = NULL,
};

CCControlData_t cc_control_data = {
    .pos = {0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7},
};
/*------------------------------ End of File ------------------------------*/
