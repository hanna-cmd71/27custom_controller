# 本工程架构说明（自定义控制器专用）

> 版本 V1.1.0（2026-10-24 精简后）。本文只描述**当前工程**；框架通用约定见 [framework.md](./framework.md)。

## 1. 定位

C 板固件，只做一件事：**把自定义控制器（操作者手上一台六自由度直驱机构）的关节角度读出来，发给主控/上位机**。

因此：

- 保留：遥控器、裁判系统、IMU、自定义控制器模块、电机与 CAN 驱动、USB 上位机通信、板间通信、离线检测/数据总线。
- 已删除：底盘、云台、发射机构、机械臂、校准、音乐、以及配套的配置参数文件。

```
application/
├── custom_controller/   ← 本工程主体（分三层，见 §2）
├── communication/       板间 CAN 打包 + USB 上位机 + 通信任务
├── robot_cmd/           电机抽象与各类电机/电控驱动（DJI/达妙/瓴控/小米/超电/SBzdatou/Robostride）
├── referee/             裁判系统串口解析（无任务框架，纯函数 + 全局结构体）
├── IMU/                 陀螺仪驱动 + AHRS 解算 + 加热
├── assist/              detect_task（离线检测）、data_exchange（发布订阅总线）
├── other/               遥控器、LED、OLED、电压检测、test_task 占位
├── typedef/             类型与宏（attribute/macro/robot/struct/custom/usb_typdef）
└── typedef_robot/       robot_param.h（参数入口）+ robot_param_engineer_controller.h（本机器人）
```

## 2. 自定义控制器模块的三层结构

框架约定（见 framework.md）：**任务层**只写流程，**实现层**放具体硬件，**根基层**放类型与全局变量。

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 任务层 | `custom_controller_task.c` | 建 7 步循环：Observer → HandleException → SetMode → Reference → Console → SendCmd，外加 1 次/循环的 `SendDataToPC()` |
| 实现层 | `custom_controller_engineer.c` | 6 个关节电机的初始化、观测（角度解算/多圈计数）、限幅、发 CAN |
| 根基层 | `custom_controller.h` / `custom_controller_engineer.h` | `CustomController_s`（电机、fdb、ref、transform、limit、pid、lpf）与 `cc_control_data` |

任务层里的 8 个步骤函数都是 `__weak` 空函数，实现层用同名强符号覆盖——这是整个框架的解耦手法：
**换一种硬件，只换实现层的文件，任务层一行不改**。

## 3. 任务与调度（`Src/freertos.c`）

| 任务 | 入口 | 优先级 | 栈 | 周期 | 说明 |
| --- | --- | --- | --- | --- | --- |
| imuTask | `IMU_task.c` | Realtime | 1024 | DRDY 中断驱动 | 等加速度计 DRDY 中断，解算姿态并加热，无固定延时 |
| communication_task | `communication_task.c` | Normal | 256 | 50 ms | 遥控器→板间帧、CAN 下发、CAN 帧拆包 |
| usb_task | `usb_task.c` | Normal | 128 | 1 ms | USB CDC 收发（VOFA 调参 / 状态包） |
| customControllerTask | `custom_controller_task.c` | High | 512 | **1000 tick** | 自定义控制器主循环（见 §5 已知问题） |
| DETECT | `detect_task.c` | Normal | 256 | 10 ms | 各外设在线/离线超时检测 |
| REFEREE | `referee_usart_task.c` | Normal | 128 | 10 ms | 裁判系统串口解帧 |
| BATTERY_VOLTAGE | `voltage_task.c` | Normal | 128 | 100 ms | 电池电压/电量 |
| led / OLED | `led_flow_task.c` / `oled_task.c` | Normal / Low | 256 | 1 ms / 10 ms | 指示灯与错误码显示 |
| test | `test_task.c` | Normal | 128 | — | 占位任务（调 USB 用） |

## 4. 数据流

### 4.1 输入：操作者的动作

```
DT7 遥控器 --USART3 DMA双缓冲+IDLE--> remote_control.c --(sbus_to_rc)--> RC_ctrl_t rc_ctrl
                                                              │
                                                    detect_hook(DBUS_TOE) → 离线检测
```

遥控器在本工程里只作**调试/使能开关**用（CC 模块本身不读摇杆）。

### 4.2 主体：关节角度 → 主控

```
6×电机(CAN1) --CAN RX 中断--> CAN_receive.c --> DjiMotorMeasure_t[] 
                                                   │ GetMotorMeasure(&joint_motor[i])
                                                   ▼
                        custom_controller_engineer.c : CustomControllerObserver()
                          theta_transform(ecd角, 安装角, 方向) → 多圈计数 round → /减速比
                                                   ▼
                                    cc_control_data.pos[0..5]（float[7]，带限幅）
                                                   ▼
        ┌──────────────────────────────┴──────────────────────────────┐
        ▼                                                             ▼
custom_controller_connect.c : SendDataToPC()                  usb_task.c : VOFA/状态包
  UART1, 帧头0xA5 + cmd_id 0x0302, 30 字节数据                    USB CDC
```

- `theta_transform()`：处理「电机零点≠机构零点」和「方向取反」。
- 多圈计数：相邻两次角度差超过 π 就 ±1 圈，避免 8192→0 的跳变。
- J4/J5 是**末端差速**：两个电机解算成一对虚拟关节，再组合成 `pos[4]=vj4+vj5`、`pos[5]=-(vj4-vj5)`。

### 4.3 输出：CAN 下发

关节电机只需要「保持零力/可拖拽」，所以 `CustomControllerSendCmd()` 里发的是 `set.value`（当前恒 0）：

```
CanCmdDjiMotor(1, DJI_6020_MODE_VOLTAGE_1, J0, J1, J2, 0);   // 0x1FF
CanCmdDjiMotor(2, DJI_3508_MODE_CURRENT_1, J3, J4, J5, 0);   // 0x200
```

`Robot cmd` 层是统一抽象：`Motor_s` 里放 `id/can/type/direction/reduction_ratio/mode`，
`GetMotorMeasure()` 按 `type` 去 `CAN_receive.c` 的对应数组取数据，所以加一种新电机 = 在 `CAN_receive.c` 加一个解析函数 + 在 `GetMotorMeasure()` 加一个 `case`。

### 4.4 其他流

- **裁判系统**：USART6 DMA 双缓冲 → `referee_usart_task.c` 的字节状态机（SOF/长度/序号/CRC8/CRC16）→ `referee_data_solve()` → 全局结构体（`get_heat_auto()` 等取值接口）。
- **板间通信**：`communication.c` 的 `SendRC()` 把遥控器 3 个通道 + 模式编码成 8 字节，用 `CanSendDataToBoard()` 发给另一块板。
  注意它有门禁：`if (BOARD_CURRENT != C_BOARD_UP) return;`，而本工程 `BOARD_CURRENT = C_BOARD_DEFAULT`，
  所以这段**当前不会发送**，保留是为了将来再把本板当「上板」用时直接可用。
- **CAN 发送缓冲**：`CanManagerAddMotor()` 按 std_id 把电流塞进 4 个槽，`CanManagerSendAll()` 统一发 `0x200/0x1FF`（每帧 4 个电机）。
- **离线检测**：`detect_task.c` 用「两次报文间隔 > offline 阈值 → 掉线」判定，`toe_is_error()` 供各模块查询。

## 5. 已知问题（本次精简发现，未擅自改行为）

1. **CC 任务实际只有 1 Hz**：`CUSTOM_CONTROLLER_CONTROL_TIME = 1000`（tick），且 `lastTick` 从未更新，
   于是 `xTaskGetTickCount() - lastTick > 34` 恒成立，`SendDataToPC()` 每 1 s 才发一次。
   若要与主控做 30 Hz 的角度流，需要把 `CONTROL_TIME` 改小（例如 1~2 tick）并周期性更新 `lastTick`。
2. **J3/J4/J5 的 CAN 口与配置不一致**：`robot_param_engineer_controller.h` 里三个电机都写 `JOINT_MOTOR_n_CAN 1`，
   但 `CustomControllerSendCmd()` 发到 **CAN2**。需按实际接线确认改哪一边。
3. **Console 恒输出 0**：`PID_calc()` 被注释，`set.value = 0`。这是「只读不驱、可手拖」的刻意做法，
   但速度环 PID 参数也全是 0，将来若要助力/回零需要补参数。
4. **VOFA 调参不可用**：`__TUNING_MODE = TUNING_F00T(12)`，`usb_task.c` 在该模式下只发 `0xffffffff` 占位。
   要在线调 CC 关节 PID，需在 `robot_typedef.h` 增加 `TUNING_CUSTOM_CONTROLLER_*` 并在 `usb_task.c` 增加分支。
5. `test_task.c` 是空循环占位任务，可删（同时删 `freertos.c` 里的 `osThreadDef(test, ...)`）。

## 6. 如何扩展

- **加一个关节**：`JOINT_NUM`（`custom_controller_engineer.h`）→ 参数文件里加 `JOINT_MOTOR_n_{ID,CAN,TYPE,DIRECTION,MODE,ANGLE_TRANSFORM}` 与 PID/LPF/限幅 → `CustomControllerInit/Observer/SendCmd` 里照抄一段。
- **换一种电机**：`motor.h` 的 `MotorType_e` → `CAN_receive.c` 的解析 + `GetMotorMeasure()` 的 case → `robot_cmd/` 加发送函数。
- **改通信协议**：`custom_controller_connect.h` 的 `DATA_LENGTH` / `cmd_id` / 字段顺序，与主控侧同步。
- **语法自检**：`.\_check\syn.ps1`（gcc `-fsyntax-only`，按 uvprojx 的宏和 include 路径跑全部 `.c`）。
