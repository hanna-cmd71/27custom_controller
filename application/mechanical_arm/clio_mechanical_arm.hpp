#include "clio_mechanical_arm.hpp"

#ifndef CLIO_MECHANICAL_ARM_HPP
#define CLIO_MECHANICAL_ARM_HPP

using namespace std;

class MOTOR{

public: 

uint8_t gear_ratio;
uint8_t can_id;
float expect_pos;
float real_vel;
float real_pos;
struct limit{
float max_vel;
float max_acc;
float max_pos;
float min_pos;
};

struct pid_index{

    float kp;
    float ki;
    float kd;
    float error;
    float max_out;
    float max_iout;
    float out;
};

MOTOR(uint8_t gear_ratio,uint8_t can_id,float max_vel,float max_acc,float max_pos,float min_pos,uint16_t kp,uint16_t ki,uint16_t kd)
{
gear_ratio=gear_ratio;
can_id=can_id;
max_vel=max_vel;
max_acc=max_acc;
max_pos=max_pos;
min_pos=min_pos;
kp=kp;
ki=ki;
kd=kd;
}

virtual void data_update(){};

virtual void caculate_and_output(){};

}

class MOTOR_DAMIAO:public MOTOR
{

virtual void data_update() override
{

};

virtual void caculate_and_output() override
{
}
}


class MOTOR_LINGKONG:public MOTOR
{
virtual void data_update() override
{

};

virtual void caculate_and_output() override
{
}


}
























#endif













