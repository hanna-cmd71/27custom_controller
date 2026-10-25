# StandardRobotPro

![language](https://img.shields.io/badge/language-C-blue.svg)

## 简介

> C板电控

本项目的计划是基于 北极熊StandardRobot++ 的基础上改造的通用型机器人代码框架。

## 模块支持

详细信息请参考 [StandardRobotPro 框架](./doc/framework.md)

### 主要模块

- **Robot cmd**\
  机器人控制模块：
  - [x] 大疆电机
  - [x] 达妙电机
  - [x] 瓴控电机
  - [x] 小米电机
  - [x] 超级电容控制板
  - [x] 板间通信(CAN)
  - [x] PWM 控制舵机
  - [x] PWM 控制气泵
- **Chassis**\
  底盘模块：
  - [x] 麦轮底盘
  - [x] 全向轮底盘
  - [ ] 舵轮底盘
  - [ ] 平衡底盘
- **Gimbal**\
  云台模块：
  - [x] yaw-pitch 直连云台
  - [ ] yaw直连-pitch丝杆 云台
  - [ ] yaw-pitch 丝杆云台
- **Shoot**\
  发射机构模块：
  - [x] 摩擦轮+拨弹盘
  - [ ] 摩擦轮+弹鼓+推杆
- **Mechanical arm**\
  机械臂模块：
  - [ ] 六自由度直驱机械臂（末端差速）
- **Custom Controller**\
  自定义控制器模块：
  - [ ] 六自由度直驱机械臂配套自定义控制器（末端差速）
- **Communication**\
  通信模块：
  - [x] 板间通信(4pin uart2)
  - [x] 上下位机通信
- **Music**\
  音乐模块：
  - [x] 开机声
  - [ ] 报错音
  - [ ] 裁判系统接入提示音
  - [x] 《卡农》
  - [x] 《天空之城》
  - [x] 《恭喜发财》
  - [x] 《好运来》
  - [x] 《See You Again》
  - [x] 《Unity》
  - [x] 《你》

## 开发工具

使用VSCode作为IDE。

> 编写代码时参考[API](./doc/API.md)

## 开发日志

[API.md](./doc/API.md) 为开发过程中所使用的模块接口标准，若在编写代码时发现代码与标准冲突，基于本标准修改代码。

## 附录

本文的详细补充内容写在 [附录](./doc/appendix.md) 中。

