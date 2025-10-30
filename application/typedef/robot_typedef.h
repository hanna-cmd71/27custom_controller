#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// clang-format off
// 可用底盘硬件类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘

// 可用云台硬件类型
#define GIMBAL_NONE                0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT    1  // yaw-pitch电机直连云台
#define GIMBAL_DOUBLE_YAW_PITCH    2  // 双yaw-pitch电机直连云台

// 可用的发射机构硬件类型
#define SHOOT_NONE               0  // 无发射机构
#define SHOOT_FRIC_TRIGGER       1  // 摩擦轮+拨弹盘发射机构
#define SHOOT_FRIC_WATER       2  // 摩擦轮+拨弹盘发射机构

// 可用机械臂硬件类型
#define MECHANICAL_ARM_NONE              0  // 无机械臂
#define MECHANICAL_ARM_ENGINEER_ARM      1  // 工程机械臂

// 可用自定义控制器硬件类型
#define CUSTOM_CONTROLLER_NONE         0  // 无自定义控制器
#define CUSTOM_CONTROLLER_ENGINEER     1  // 工程用的自定义控制器

// 可用调参模式
#define TUNING_NONE             0
#define TUNING_CHASSIS          1
#define TUNING_GIMBAL           2
#define TUNING_SHOOT            3
#define TUNING_GIMBAL_PITCH     4
#define TUNING_GIMBAL_YAW       5
#define TUNING_GIMBAL_YAW_BA    6
#define TUNING_GIMBAL_YAW_UP    7
#define TUNING_SHOOT_FIRC       8
#define TUNING_SHOOT_TRIGGER    9
#define TUNING_CHASSIS_WHEEL    10
#define TUNING_CHASSIS_FOLLOW   11
#define aaa666   666


//大yaw的运动模式
#define deadline  0    //死区模式
#define follow1    1    //跟随模式
// 校准数据来源
#define CALI_FROM_FLASH 1
#define CALI_FROM_USB   2
#define CALI_FROM_CODE  3

// 自定义控制器类型
#define CC_RECEIVER 0  // 接收器
#define CC_SENDER   1  // 发送器

// 遥控器类型
#define RC_DT7      0  // DT7遥控器

// C板id
#define C_BOARD_DEFAULT                  0  // C板默认id
#define C_BOARD_UP                       1  // C板上
#define C_BOARD_DOWN                     2  // C板下
#define C_BOARD_OMMI_INFANTRY_CHASSIS    3  // 全向轮步兵底盘C板
#define C_BOARD_OMMI_INFANTRY_GIMBAL     4  // 全向轮步兵云台C板
#define C_BOARD_OMNI_SENTINEIL_CHASSIS   5  // 全向轮哨兵底盘C板
#define C_BOARD_OMNI_SENTINEIL_GIMBAL    6  // 全向轮哨兵云台C板
#define C_BOARD_MECANUM_HERO_CHASSIS     7  // 麦克纳姆轮英雄底盘C板
#define C_BOARD_MECANUM_HERO_GIMBAL      8  // 麦克纳姆轮英雄云台C板
#define C_BOARD_ENGINEER_CHASSIS         9  // (单板)工程C板

// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_M6020,
    CYBERGEAR_MOTOR,
    DM_8009,
    DM_4310,
    DM_4340,
    MF_9025,
} MotorType_e;
// clang-format on

// typedef enum {
//     ROBO_ZERO_FORCE = 2,                    //机器人整体无力                           云台：无力        底盘：无力
//     ROBO_CHASSIS_FOLLOW_GIMBAL_YAW = 3,     //机器人进入底盘跟随云台,云台绝对角度控制    云台：绝对角度    底盘：跟随云台
//     ROBO_AUTO = 1,                          //机器人自动模式没有陀螺                   云台：绝对角度    底盘：不跟随
//     ROBO_AUTO_WITH_SPIN,                    //机器人自动模式有陀螺                   云台：绝对角度    底盘：陀螺
//     ROBO_INIT_OLD,                              //机器人初始化
//     ROBO_GIMBAL_IMU,                        //云台绝对角度控制
//     ROBO_GIMBAL_GAP,                        //跳出矫正进入IMU/AUTO_AIM模式之前的存储数据模式
//     ROBO_DBUS_ERR                           //DBUS错误
// } RoboMode_e;

typedef enum {
    ROBO_INIT                   = 0,      // 机器人初始化

    GIMBAL_ZERO_FORCE           = 2,      // 云台无力
    GIMBAL_ABSOLUTE_ANGLE       = 1,      // 云台绝对角度控制
    GIMBAL_AUTO_AIM             = 3,      // 云台自动瞄准控制

    GIMBAL_RELATIVE_ANGLE       = 4,      // 云台相对角度控制
    GIMBAL_GAP                  = 5,      // 云台跳出矫正进入IMU
    GIMBAL_DBUS_ERR             = 6,      // 云台DBUS错误


    CHASSIS_ZERO_FORCE          = 12,      // 底盘无力
    CHASSIS_NO_FOLLOW           = 11,      // 底盘不跟随云台
    CHASSIS_FOLLOW_GIMBAL_YAW   = 13,      // 底盘跟随云台
    
    CHASSIS_NOMOVE              = 14,      // 底盘不动
    CHASSIS_SPIN                = 15,      // 底盘陀螺
} BoardMode_e;

#endif /* ROBOT_TYPEDEF_H */
/*------------------------------ End of File ------------------------------*/
