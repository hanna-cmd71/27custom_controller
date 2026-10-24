# StandardRobotPro — 自定义控制器 C 板固件

![language](https://img.shields.io/badge/language-C-blue.svg)

## 简介

> C 板电控

本工程基于 [北极熊 StandardRobot++](https://github.com/Penguin-SZ/StandardRobotPro) 改造，**现在只服务于自定义控制器（Custom Controller）**：
读取操作者 6 个关节电机的位置，解算后通过 UART / USB 发给主控或上位机，同时保留裁判系统、遥控器、IMU 以及完整电机与 CAN 驱动。

底盘、云台、发射机构、机械臂、校准、音乐等模块已经从工程与 Keil 工程中删除，代码分层与编程约定不变，详见 [框架说明](./doc/framework.md) 与
[本工程架构说明](./doc/architecture.md)。

## 模块支持

- **Custom Controller**\
  自定义控制器模块（本工程主体）：
  - [x] 六自由度直驱机械臂配套自定义控制器（末端差速）
- **Robot cmd**\
  机器人控制模块（驱动层，全部保留）：
  - [x] 大疆电机（M2006 / M3508 / M6020）
  - [x] 达妙电机
  - [x] 瓴控电机
  - [x] 小米电机
  - [x] 超级电容控制板
  - [x] 板间通信(CAN)
  - [x] PWM 控制舵机
  - [x] PWM 控制气泵
  - [x] 步进电机（SBzdatou / Robostride）
- **Communication**\
  通信模块：
  - [x] 与主控通信(UART1，自定义控制器协议)
  - [x] 板间通信(CAN，可选)
  - [x] 上下位机通信(USB CDC + VOFA)
- **Referee**\
  裁判系统：
  - [x] 串口解析、热量与血量等数据
- **Other**\
  其他：
  - [x] 遥控器（DT7 / SBUS）
  - [x] IMU（BMI088 + IST8310，AHRS 解算）
  - [x] 离线检测、发布订阅数据总线、LED / OLED / 电压检测

## 开发工具

使用 VSCode 作为 IDE，编译使用 Keil MDK（`MDK-ARM/StandardRobotPro.uvprojx`）。

> 编写代码时参考 [API](./doc/API.md)（旧版框架接口标准，部分模块已删除）

不装 Keil 也可以做语法自检：

```powershell
# 需要 D:\Tools\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe
.\_check\syn.ps1
```

它会按 `uvprojx` 里的宏与头文件路径，对 `application/`、`bsp/`、`components/` 下所有 `.c` 做 `-fsyntax-only` 检查。

## 开发日志

[API.md](./doc/API.md) 为开发过程中所使用的模块接口标准，若在编写代码时发现代码与标准冲突，基于本标准修改代码。

- V1.1.0 Oct-24-2026 工程精简：只保留自定义控制器相关模块

## 附录

本文的详细补充内容写在 [附录](./doc/appendix.md) 中。
