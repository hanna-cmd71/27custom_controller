#include "water_shot.h"
#include "data_exchange.h"
#include "communication.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#if (SHOOT_TYPE == SHOOT_FRIC_WATER)

int aaaa1;
static Shoot_s SHOOT = {
    .mode = LOAD_STOP,
    .state = FRIC_NOT_READY,
    .fric_flag = 0,
    .move_flag = 0,
    .ecd_count = 0,
    .shoot_flag = 0,
    .heat = 0,
    .heat_limit = 0,
};
void ShootPublish(void)
{
  Publish(&SHOOT, SHOOT_NAME);
}

void ShootInit(void)
{
    SHOOT.rc = get_remote_control_point();

}
void ShootSetMode(void)
{
  /*键鼠遥控器控制方式初版----------------------------*/
  if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) // 下档防止误触
  {
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
  }
  else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
{
    SHOOT.state = FRIC_READY;
    SHOOT.mode = LAOD_BULLET;
}
  else if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) // 上档连发
{
    SHOOT.state = FRIC_READY;
    SHOOT.mode = LOAD_BURSTFIRE;
}
  if ((switch_is_down(SHOOT.rc->rc.s[0])))
  {
     SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
  }
    if (toe_is_error(DBUS_TOE))
  {
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
  }
}
void ShootReference(void)
{
  switch (SHOOT.mode)
  {
    case LOAD_STOP:
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
      aaaa1=0;
    break;
    case LAOD_BULLET:
    if(aaaa1==0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
      vTaskDelay(50);
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
      aaaa1=1;
    }
    if(SHOOT.rc->mouse.press_l==1)
    {
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
      vTaskDelay(50);
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
            vTaskDelay(120);

    }
    else
    {
       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
    }
    break;
    case LOAD_BURSTFIRE:
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
    break;


  }
}


















#endif