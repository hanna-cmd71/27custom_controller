# 本工程架构说明（自定义终端）

> 版本 V1.2.0（2026-10-26）。本文只描述**当前工程**；框架通用约定见 [framework.md](./framework.md)，标定与上车流程见 [calibration_and_validation.md](./calibration_and_validation.md)。

## 1. 定位

C 板固件，只做一件事：**把自定义终端（操作者手上一台六自由度机械臂，J1 为 RS00、其余为 ZDT 闭环步进）的关节角度读出来发给上位机，并能对单个关节做力矩测试与重力补偿**。

- 保留：裁判系统、IMU、自定义终端模块、电机与 CAN 驱动、USB 上位机通信、板间通信、离线检测/数据总线。
- 已删除：底盘、云台、发射机构、机械臂、校准、音乐，以及旧的 `custom_controller_engineer` / `custom_controller_connect` 实现。

```
application/
├── custom_controller/   ← 本工程主体（见 §2、§3）
│   ├── custom_terminal.c/.h        自定义终端：6 关节读回换算、力矩测试、上行（含类型、`cc_control_data`、周期宏）
│   ├── terminal_kinematics.c/.h    机构模型：DH 表 + 基座变换 + 正解（`FK_SolveAll`）
│   ├── terminal_gravity.c/.h       重力补偿：`terminal_gravity(q, tau_nm)`
│   ├── terminal_debug.c/.h         Watch 观察口：`terminal_dbg`
│   └── custom_controller_task.c    任务层：7 步循环 + `__weak` 钩子
├── communication/       板间 CAN 打包 + 上位机下行解包 + 通信任务
├── robot_cmd/           电机抽象与驱动（DJI/达妙/瓴控/小米/超电/zdatou/Robostride）
├── referee/             裁判系统串口解析
├── IMU/                 陀螺仪驱动 + AHRS 解算 + 加热
├── assist/              detect_task（离线检测）、data_exchange（发布订阅总线）
├── other/               遥控器、LED、OLED、电压检测、test_task 占位
├── typedef/             类型与宏
├── typedef_robot/       robot_param.h（入口）+ robot_param_omni_mini.h（整车参数）+ robot_param_terminal.h（本板机械臂参数）
└── gimbal/              云台框架（本工程 `GIMBAL_TYPE = GIMBAL_NONE`，只编译 `gimbal_virtual.c`）
```

## 2. 自定义终端的结构

框架手法不变：**任务层只写流程，实现层放硬件**，任务层的钩子都是 `__weak`，实现层用同名强符号覆盖。

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 任务层 | `custom_controller_task.c` | `Publish → Init`，之后每 1 ms 循环 `Observer → HandleException → SetMode → Reference → Console → SendCmd`；每 `CUSTOM_CONTROLLER_SEND_TIME`(34 ms) 调一次 `CustomControllerSendToPC()` |
| 实现层 | `custom_terminal.c` | 6 关节读回与换算、力矩测试、上行组帧 |
| 模型层 | `terminal_kinematics.c` / `terminal_gravity.c` | 纯数学：DH 表与正解、重力前馈；不碰电机/CAN/任务 |
| 根基层 | `custom_terminal.h` | `custom_terminal_t`（关节数组）、`CCControlData_t`/`cc_control_data`、周期宏与上行协议宏 |
| 观察口 | `terminal_debug.h` / `.c` | `terminal_debug_t`（Watch 用，可整体摘除） |

变体开关：`robot_param_omni_mini.h` 的 `CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_TERMINAL`；机械臂自身参数全在 `robot_param_terminal.h`（与整车参数文件互相独立）。
`custom_terminal.c` 整体包在 `#if` 内，`#else` 只提供全 0 的 `custom_terminal_get_joint_feedback()`，供 `communication.c` 无条件引用。

### 2.1 依赖边界（重要）

`custom_terminal.*` **只依赖**：`robot_param.h`、`struct_typedef.h`、`terminal_debug.h`、`terminal_kinematics.h`、`terminal_gravity.h`、两个电机驱动（`CAN_cmd_zdatou.h`、`CAN_cmd_robostride.h`）、`CRC8_CRC16.h`、`usart.h`、`cmsis_os.h`、`math.h`、`string.h`。

`terminal_kinematics.*` / `terminal_gravity.*` 只依赖 `robot_param.h`（参数）+ `math.h`/`string.h`（数学）。**它们不认识电机、CAN、任务、调试口**，因此可以单独拷到 PC 上复算，也可以被以后的控制律直接复用。

**不依赖**遥控器（`remote_control.h`）、离线检测（`detect_task.h`）、云台（`gimbal.h`）、任何 `Gimbal_s`/`RC_ctrl_t` 类型。这是刻意的：自定义终端是独立模块，除了"通信"以外不与其他模块耦合。

## 3. 任务与调度（`Src/freertos.c`）

| 任务 | 入口 | 优先级 | 栈 | 周期 | 说明 |
| --- | --- | --- | --- | --- | --- |
| imuTask | `IMU_task.c` | Realtime | 1024 | DRDY 中断驱动 | 姿态解算与加热 |
| communication_task | `communication_task.c` | Normal | 256 | 100 ms | `SendRC` → `CanManagerSendAll` → `DataUnpack`（**上行已不在这里**） |
| usb_task | `usb_task.c` | Normal | 128 | 1 ms | USB CDC（VOFA 调参 / 状态包） |
| customControllerTask | `custom_controller_task.c` | High | 512 | **1 ms**（TERMINAL 变体） | 自定义终端主循环 |
| DETECT | `detect_task.c` | Normal | 256 | 10 ms | 外设在线/离线超时检测 |
| REFEREE | `referee_usart_task.c` | Normal | 128 | 10 ms | 裁判系统解帧 |
| BATTERY_VOLTAGE | `voltage_task.c` | Normal | 128 | 100 ms | 电池电压/电量 |
| led / OLED | `led_flow_task.c` / `oled_task.c` | Normal / Low | 256 | 1 ms / 10 ms | 指示灯与错误码 |
| test | `test_task.c` | Normal | 128 | — | 占位任务 |

## 4. 数据流

### 4.1 输入：电机反馈（`CAN_receive.c` 只做分发）

```
CAN1 RX 中断 → CAN_receive.c
   ├─ 标准帧 → robostride_on_rx(can, std_id, data, dlc)   // 只认 (std_id & 0xFF) == ROBOSTRIDE_MASTER_ID
   └─ 扩展帧 → zdatou_on_rx(can, ExtId >> 8, data, dlc)   // 地址 1~6 归本驱动，其余交给别的分支
                     │
                     ├─ 驱动内部缓存 zdatou_measure[2][6] / robostride_measure[2][6]
                     └─ 驱动内部诊断 zdatou_dbg / robostride_dbg（计数、功能码、DLC、原始字节、ok_mask）
```

**反馈不是轮询出来的**：开机时给 5 台 ZDT 发一次"定时上报"配置（`11 18 0x31 00 0A 6B`，即每 10 ms 上报编码器值），之后电机主动发；RS00 则是"使能后才会应答"。`CustomControllerObserver` 每秒检查 `zdatou_dbg.ok_mask`，对还没上报的地址补发一次配置（开机丢配置不会让电机永远哑着）。

### 4.2 主体：`CustomControllerObserver()`（每 1 ms）

```
读 zdatou_get_measure()/robostride_get_measure()
  → raw_mech = DIR[i] * raw
  → joint_fb[i] = wrap(raw_mech - ZERO[i]) / GEAR[i]
  → joint_vel[i] = wrap(raw_mech - raw_prev) / GEAR / dt
  → valid[i] = (有帧 && now - stamp <= 20 ms)
  → FK_SolveAll(joint_fb, joint_pose)
  → cc_control_data.pos[0..5]（供上行；pos[6] 仍是占位）
  → terminal_dbg.* 镜像（Watch 观察口）
```

零位标定：把 `terminal_dbg.zero_capture` 写 1，下一拍把 6 个 `joint_raw[]` 抄进 `zero_readback[]`，把结果填回参数表的 `CUSTOM_TERMINAL_JOINT_ZERO`。

### 4.3 输出：CAN 下发（`CustomControllerSendCmd()`，每 1 ms）

3 拍轮转，每拍 2 帧：拍 0 = J0(ZDT) + J1(RS00)，拍 1 = J2/J3，拍 2 = J4/J5。

- ZDT 走 `zdatou_torque_control_i32()`（`F5 dir FF FF 电流 00 6B`，电流 = N·m × `MA_PER_NM`，满量程 5000 mA）
- RS00 走 `robostride_mit_control()`（pos/vel/kp/kd 填 0，只有前馈力矩）
- 力矩来源只有 `CustomControllerConsole()`（手动测试 / 重力补偿），都带硬上限与变化率限制

### 4.4 上行：`CustomControllerSendToPC()`（30 Hz，DMA）

```
cc_control_data.pos[0..5] → 39 字节帧（sof 0xA5 + len 30 + seq + CRC8 + cmd_id 0x0302 + 30B + CRC16）
   → HAL_UART_Transmit_DMA(huart1, ...)   // 4pin，上位机
   → HAL_UART_Transmit_DMA(huart6, ...)   // 3pin，裁判系统口复用
```

每口一份缓冲、一个 seq；`gState` 不是 READY 时跳过并计数，连续 10 拍（340 ms）卡住就 `HAL_UART_AbortTransmit()` 自愈。
下行（上位机 → 板）仍走 `communication.c` 的 `DataUnpack()`，每 100 ms 一次。

### 4.5 其他流

- **裁判系统**：USART6 DMA 双缓冲 → `referee_usart_task.c` 状态机 → 全局结构体。`USART6_IRQHandler` 里先搬数据、再调 `HAL_UART_IRQHandler()`（DMA 发送要靠它清 TC）。
- **板间通信**：`communication.c` 的 `SendRC()`，有门禁 `BOARD_CURRENT != C_BOARD_UP` 时不发。
- **离线检测**：`detect_task.c` 按报文间隔判掉线；自定义终端自己的超时判定在 `CustomControllerObserver` 里（20 ms）。
- **遥控器**：`remote_control.c` 仍在工程里（其它模块在用），但 `custom_terminal` 不读它。

## 5. 已知问题 / 待办

1. **DH 表不是真 DH**：`arm_dh_param[]` 的 `alpha` 与 `d` 仍全为 0，`DhTransform` 是 `Rj·T_fixed`，第三列不是旋转轴。FK 只能给相对位姿；**接重力补偿前必须换成真 DH**。
2. **软限位缺失**：只有力矩上限，没有关节角限位。
3. **上行阻塞 vs 力矩环**：DMA 已解决阻塞问题，但 30 Hz 组帧仍在 1 ms 任务里跑；若以后帧变长要考虑独立任务。
4. **`cc_control_data.pos[6]` 恒为占位 0xA7**（只有 6 个关节）。
5. `test_task.c` 是空循环占位任务，可删（同时删 `freertos.c` 里的 `osThreadDef(test, ...)`）。

## 6. 如何扩展

- **加一个关节**：`robot_param_terminal.h` 加一项（`ZD_ADDR`/`DIR`/`GEAR`/`ZERO`/`MA_PER_NM`）→ `CUSTOM_TERMINAL_JOINT_NUM` 加 1 → `custom_terminal.c` 的关节循环自动覆盖，只需在 `CustomControllerSendCmd` 的轮转里多分一拍。
- **换一种电机**：`motor.h` 的 `MotorType_e` → `robot_cmd/` 加驱动（照 `CAN_cmd_zdatou.c` 的"自持缓存 + 诊断 + `xxx_on_rx`"写法）→ `CAN_receive.c` 分发处加一行 → `GetMotorMeasure()` 加 case。
- **改通信协议**：`custom_terminal.h` 的 `CUSTOM_TERMINAL_UPLINK_*`（SOF/cmd_id/长度）与主控侧同步。
- **构建**：`D:\RM\Keil\Core\UV4\UV4.exe -b MDK-ARM\StandardRobotPro.uvprojx -j0 -o build.log`（AC5）。
  走 EIDE 时源清单在 `MDK-ARM/.eide/eide.yml`，**新增/删除源文件必须同步这个清单**，否则会静默不编译并报 `L6218E`。
