# 自定义终端（6-DoF 主手）技术拆解

> 版本：对应代码 V1.2.0（2026-10-26）。本文只拆 `CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_TERMINAL` 这一条分支，
> 与 [architecture.md](./architecture.md)（架构快照）、[calibration_and_validation.md](./calibration_and_validation.md)（上车流程）互为补充：
> 那两篇讲"是什么/怎么做"，这篇讲"**为什么这样接、数据怎么流、控制怎么闭环**"。

---

## 0. 一页速览

**一句话定位**：C 板固件把操作者手上那台六轴机械臂的关节角读出来，30 Hz 发给上位机；同时能对单个关节施加受控力矩，用于标定转向/零位/力标度，并在标定完成后叠加重力前馈。

| 维度 | 结论 | 出处 |
| --- | --- | --- |
| 板卡 | STM32F407IGH6，168 MHz，单精度 FPU；`BOARD_CURRENT = C_BOARD_DOWN` | `Keil.ioc`、`Src/main.c`、`robot_param_omni_mini.h:12` |
| 角色 | 主手（操作端），不是从手（执行端） | `custom_terminal.h:3` |
| 执行器 | J0/J2/J3/J4/J5 = 张大头 X42S 闭环步进（电流环力矩）；J1 = RobStride RS00（MIT 前馈力矩） | `robot_param_terminal.h:30-37` |
| 总线 | CAN1 @ 1 Mbps（42 MHz ÷ 3 ÷ 14 TQ）；扩展帧（ZDT）与标准帧（RS00）混跑 | `Src/can.c:35-39` |
| 实时性 | 力矩环 1 ms；单关节实际刷新 3 ms；上行 30 Hz（34 ms） | `custom_terminal.h:44-49` |
| 上行 | 39 B 定长帧，`0xA5 / 0x0302 / CRC8+CRC16`，huart1 + huart6 双口 DMA | `custom_terminal.c:442-501` |
| 下行 | huart1 115200 8N1，`5 + 20 + 2` 定长帧，100 ms 轮询解包 | `communication.c:277` |
| 代码量 | `custom_controller/` 10 个文件（5 `.c` + 5 `.h`），约 1.4 k 行 | — |
| 控制律 | 只有"手动力矩 + 重力前馈"，**无位置环、无阻抗环、无软限位** | `custom_terminal.c:308-381` |

四条链路各一句话：

- **数据链**：CAN 中断收 → 驱动缓存 → 1 ms 观测换算 → 30 Hz 组帧 DMA 发；下行反着走，100 ms 解一次。
- **项目架构**：任务层（流程）／实现层（硬件）／模型层（纯数学）／根基层（类型与参数）／观察口（Watch）五层，靠 `__weak` 强符号覆盖粘合。
- **控制链**：`Observer → HandleException → SetMode → Reference → Console → SendCmd` 六步，力矩只有两个来源，且互斥。
- **技术栈**：STM32 HAL + FreeRTOS(CMSIS-OS v1) + Keil AC5/EIDE；算法侧是 DH 正解 + 旋量式重力矩投影。

---

## 1. 技术栈

### 1.1 硬件栈

| 层 | 选型 | 关键参数 | 为什么是它 |
| --- | --- | --- | --- |
| 主控 | STM32F407IGH6（UFBGA176） | 168 MHz、192+128 KB RAM、单精度 FPU、2×CAN、6×USART、USB OTG | RoboMaster C 板标准件；FPU 决定 1 ms 内能跑完两遍 6 轴正解 |
| 关节 1 | RobStride RS00 | MIT 协议，位置 ±12.57 rad、速度 ±33 rad/s、力矩 ±14 N·m，CAN 标准帧 | 大扭矩、直驱，适合基座回转（J1）；自带位置/速度/力矩三环 |
| 关节 2~6 | 张大头 X42S 闭环步进 ×5 | 力矩模式 `F5`，相电流 0~5000 mA，0.1° 位置分辨率，扩展帧 | 便宜、薄、力矩够；闭环不失步；一条 CAN 挂 5 台不冲突 |
| 总线 | CAN1，1 Mbps | 扩展帧 ID = `(Addr << 8) | Packet`；标准帧 ID = 电机 CAN_ID | 1 Mbps 是 6 轴 1 kHz 轮转的下限；ZDT 出厂 500 k 需改 |
| 上位机口 | USART1（4pin），115200 8N1 | TX 走 DMA，RX 走 DMA 双缓冲 + FIFO | 4pin 是 C 板标准调试口，接 PC/裁判系统串口模块 |
| 备用口 | USART6（3pin），115200 8N1 | 与裁判系统**共用**物理口 | 现场没带 4pin 线时的退路（见 §6 P1-4） |
| 调试 | USB CDC + Keil Watch | `terminal_dbg` / `zdatou_dbg` / `robostride_dbg` | 没有 CAN 分析仪也能定位到"哪台电机没回帧" |

### 1.2 软件栈

| 层 | 组件 | 说明 |
| --- | --- | --- |
| 底层驱动 | STM32F4 HAL | CAN、UART、DMA、GPIO；`HAL_UART_IRQHandler` 被手工补进 `USART1/6_IRQHandler` 以释放 DMA 发送的 `gState` |
| 板级封装 | `bsp/boards/bsp_can.c`、`bsp_uart`、`bsp_usart` | `can_send_std` / `can_send_ext` 直接 `HAL_CAN_AddTxMessage`，**不排队** |
| OS | FreeRTOS + CMSIS-OS v1 | `osThreadDef/osThreadCreate`，优先级 Realtime/High/Normal/Low |
| 应用框架 | 北极熊 StandardRobotPro | 三层八步（见 §2.5） |
| 构建 | Keil MDK AC5（`UV4.exe -b`）或 EIDE（`MDK-ARM/.eide/eide.yml`） | 新增源文件必须同步 `eide.yml`，否则静默不编译 → `L6218E` |
| 校验 | `CRC8_CRC16.c` | 上行 `append_CRC8_check_sum`（帧头 5 B）+ `append_CRC16_check_sum`（整帧） |

### 1.3 算法栈

| 算法 | 实现 | 复杂度 | 状态 |
| --- | --- | --- | --- |
| 角度归一化 | `terminal_wrap_angle()`，`fmodf` + 折返到 `[-π, π)` | O(1) | 可用 |
| 正运动学 | `terminal_dh_transform()`（罗德里格斯绕任意轴）+ 4×4 齐次矩阵连乘 + 四元数提取 | 6 次变换 + 6 次矩阵乘 / 周期 | 可用但**不是真 DH**（见 §6 P0-2） |
| 重力补偿 | `terminal_gravity()`：连杆质心 → 基座系 → 对关节轴做 `(r × F)·z` 投影累加 | 6 次变换 + 6 次矩阵乘 + 21 次叉乘 | 已实现，默认 `gravity_en = 0` |
| 力矩整形 | `clamp_torque()` 硬限幅 + `step_toward()` 每拍限速 | O(1) | 可用 |
| 控制律 | — | — | **空缺**（只有前馈） |

### 1.4 选型的代价（不是免费的）

| 选择 | 换来的 | 付出的代价 |
| --- | --- | --- |
| 5 台 ZDT 挂一条 CAN，靠"定时上报"而不是轮询 | 主循环零查询开销，反馈 10 ms 自动到 | 开机配置帧丢一次这台就永久哑；必须靠 1 s 自愈补发（`custom_terminal.c:155-170`） |
| 1 ms 力矩环 + 3 拍轮转 | 单帧总线负载低（每 ms 只 2 帧） | 单关节实际 3 ms 才更新一次，等效 333 Hz，不是 1 kHz |
| 双口同时上行 | 插哪个口都能看到数据 | 两倍总线占用；USART6 与裁判系统抢口 |
| 全部参数外置到 `robot_param_terminal.h` | 换臂只改一个头文件 | 参数与代码强耦合于编译期，改零位必须重编译（Watch 只能临时覆盖） |
| 手动力矩只走 Keil Watch | 无需遥控器、无需上位机协议 | 标定必须连着调试器，现场无法脱机操作 |

---

## 2. 项目架构

### 2.1 目录与职责

```
application/
├── custom_controller/            ← 本工程主体
│   ├── custom_controller_task.c   任务层：8 步循环 + __weak 钩子（139 行）
│   ├── custom_terminal.c/.h       实现层：读回换算、力矩整形、上行组帧（525 + 135 行）
│   ├── terminal_kinematics.c/.h   模型层：DH 表、基座变换、正解（247 + 79 行）
│   ├── terminal_gravity.c/.h      模型层：重力前馈（131 + 30 行）
│   └── terminal_debug.c/.h        观察口：terminal_dbg（18 + 65 行）
├── communication/               板间 CAN 打包 + 上位机下行解包 + 通信任务 + USB CDC
├── robot_cmd/                   电机抽象 + 各厂商驱动（DJI/达妙/瓴控/小米/超电/ZDT/RS）
├── referee/                     裁判系统串口状态机
├── IMU/                         陀螺仪 + AHRS + 加热
├── assist/                      detect_task（掉线检测）、data_exchange（发布订阅）
├── other/                       遥控器、LED、OLED、电压、test_task 占位
├── typedef/                     通用类型与宏
├── typedef_robot/               robot_param.h（入口）→ omni_mini（整车）+ terminal（本板）
└── gimbal/                      云台框架（本工程 GIMBAL_NONE，只编译 gimbal_virtual.c）
```

### 2.2 五层结构与依赖方向

```
任务层   custom_controller_task.c
         Publish → Init → [6 步循环] → SendToPC
         只写流程，全部钩子是 __weak
                    │ 强符号覆盖（同名函数）
                    ▼
实现层   custom_terminal.c
         唯一碰硬件的地方：读驱动缓存 / 换算 / 限幅 / 组帧 / 发 CAN
                    │
        ┌───────────┴────────────┐
        ▼                        ▼
模型层   terminal_kinematics      根基层   custom_terminal.h
         terminal_gravity                  custom_terminal_t
         只依赖 robot_param                 CCControlData_t
         与 math.h                         周期/协议宏

旁挂：terminal_debug.*（观察口）—— 实现层写、Watch 读，可整体摘除
```

依赖是**严格单向**的：模型层不认识电机、CAN、任务、调试口，所以 `terminal_kinematics.c` + `terminal_gravity.c` 可以原样拷到 PC 上做数值复算，也可以被以后的位置环/阻抗环直接复用。这是本模块最重要的架构决定。

### 2.3 依赖边界（为什么这很重要）

`custom_terminal.c` 的 `#include` 只有 10 个：`robot_param.h`、`struct_typedef.h`、`terminal_debug.h`、`terminal_kinematics.h`、`terminal_gravity.h`、`CAN_cmd_zdatou.h`、`CAN_cmd_robostride.h`、`CRC8_CRC16.h`、`usart.h`、`cmsis_os.h` + `math.h`/`string.h`。

**明确不依赖**：`remote_control.h`、`detect_task.h`、`gimbal.h`、任何 `Gimbal_s` / `RC_ctrl_t`。这意味着：

- 删掉遥控器不会编译失败（V1.1.0 就是这么解耦的）；
- 自定义终端不是"云台的一个变体"，而是与底盘/云台/发射并列的独立模块；
- 唯一的外部耦合是"上行"，而它被收在 `CustomControllerSendToPC()` 一个函数里。

### 2.4 编译期变体与门禁

| 宏 | 位置 | 当前值 | 作用 |
| --- | --- | --- | --- |
| `BOARD_CURRENT` | `robot_param_omni_mini.h:12` | `C_BOARD_DOWN` | 决定 `CHASSIS/GIMBAL/SHOOT` 全为 NONE；同时是 `SendRC()` 的门禁 |
| `CUSTOM_CONTROLLER_TYPE` | `robot_param_omni_mini.h:47` | `CUSTOM_CONTROLLER_TERMINAL` | 选实现体；`NONE` 时 `custom_terminal.c` 整体不编译 |
| `CUSTOM_TERMINAL_TEST_MODE` | `custom_terminal.h:71` | `1` | 1 = 编入手动力矩通道；标定完成后置 0 |
| `CUSTOM_TERMINAL_JOINT_NUM` | `robot_param_terminal.h:29` | `6` | 驱动所有关节循环的上界 |
| `CUSTOM_TERMINAL_RS_JOINT` | `robot_param_terminal.h:30` | `1` | 哪个关节走 RS00 分支 |

任务创建处同样有门禁（`Src/freertos.c:218-221`）：

```c
#if (CUSTOM_CONTROLLER_TYPE != CUSTOM_CONTROLLER_NONE)
  osThreadDef(customControllerTask, custom_controller_task, osPriorityHigh, 0, 512);
  customControllerTaskHandle = osThreadCreate(osThread(customControllerTask), NULL);
#endif
```

### 2.5 与母框架的对应关系

| 框架概念 | 本模块落点 |
| --- | --- |
| 上层 task | `custom_controller_task.c`（含 `__weak` 空实现） |
| 中层 执行层 | `custom_terminal.c` |
| 下层 根基层 | `custom_terminal.h` + `robot_param_terminal.h` |
| 八步中的 Publish/Init/…/SendCmd | 全部保留签名，`HandleException` 与 `SetMode` 目前是空实现 |
| 硬件选择 | 不用"多底盘类型"那套，改用"关节表 + 地址表"数据驱动（加关节不改循环） |

### 2.6 资源预算

| 项 | 值 | 说明 |
| --- | --- | --- |
| 任务栈 | 512 words = 2 KB | `osThreadDef(..., 512)` |
| 优先级 | `osPriorityHigh` | 低于 IMU（Realtime），高于 communication/detect |
| 周期 | 1 ms | `CUSTOM_CONTROLLER_CONTROL_TIME` |
| CPU 占用（估算） | 3%~8% | 每周期两遍 6 轴正解（Observer 一次 + Reference 一次），168 MHz + FPU 下粗估 30~80 µs |
| 静态 RAM | 上行帧 2×39 B + 关节状态约 200 B + 诊断结构约 100 B | 可忽略 |

---

## 3. 数据链

### 3.1 全景

```
① 反馈链（10 ms 主动上报 / 应答式）

   ZDT ×5  ──扩展帧──┐
                     ├──▶ CAN1 RX FIFO0 中断 (CAN_receive.c:336)
   RS00 ×1 ──标准帧──┘
                                   │
                                   ├──▶ zdatou_on_rx()     → zdatou_measure[2][6] + zdatou_dbg
                                   └──▶ robostride_on_rx() → robostride_measure[2][6] + robostride_dbg
                                   │
                                   │ 每 1 ms 读缓存
                                   ▼
                   CustomControllerObserver()
                   raw → dir → zero → gear → wrap → FK
                                   │
② 上行链（34 ms）                   ▼
                   cc_control_data.pos[0..5]
                                   │ 组 39 B 帧
                                   ▼
                   huart1 TX-DMA  +  huart6 TX-DMA

③ 下行链（100 ms 轮询）

   PC ──115200──▶ USART1 RX-DMA 双缓冲 ──▶ FIFO(1024) ──▶ DataUnpack()
                                                          └──▶ BOARD_RX_DATA[id][]
```

### 3.2 反馈链：为什么"不是轮询出来的"

ZDT 驱动器支持"定时返回信息"（手册 5.5.1）。开机时对 5 台各发一次：

```
扩展帧 ID = (Addr << 8) | 0x00
数据域    = 11 18 31 00 0A 6B
            │  │  │  └──┬─┘
            │  │  │     └── 定时 0x000A = 10 ms
            │  │  └── 信息码 0x31 = 线性化编码器（0~65535 对应 0~360°）
            │  └── 辅助码 0x18
            └── 功能码 0x11（定时返回信息）
```

发一次就一直有效。之后电机每 10 ms 主动上报，主循环**零查询开销**。

RS00 不同：它必须**先使能**才应答，所以 `CustomControllerInit()` 里先 `robostride_enable()`，之后每收一帧控制指令回一帧状态（应答式）。

**自愈机制**（`custom_terminal.c:155-170`）：每秒检查一次 `zdatou_dbg.ok_mask`，对还没上报的地址补发一次配置帧，`break` 保证每秒最多补一台。因为 CAN 配的是 `AutoRetransmission = DISABLE`（`Src/can.c:42`），开机那一下总线仲裁失败/驱动器上电慢就会静默丢帧——没有这个自愈，那台电机永远哑着。

**中断侧只做分发，不做业务**（`CAN_receive.c:336-351`）：

```c
HAL_CAN_RxFifo0MsgPendingCallback(hcan):
    HAL_CAN_GetRxMessage(...)
    if IDE == STD → DecodeStdIdData()   // DJI / DM / LK / 超电 / RS00 / 板间
    if IDE == EXT → DecodeExtIdData()   // ZDT / 小米
```

两个驱动都返回 `1 = 这帧我认领了`，调用方立刻 `return`，所以新增电机类型只是加一个分支。

**解码在驱动内部完成**，上层拿到的已经是物理量：

| 上报码 | 帧格式 | 驱动产出 |
| --- | --- | --- |
| `0x31` | `31 <高> <低> 6B`，单圈 0~65535 | `ecd` + `pos_01deg = ecd×3600/65536` |
| `0x36` | `36 <符号> <位置 4B> 6B`，0.1° 可多圈 | `pos_01deg` 直接来自电机 |
| 其它（`02/E2/EE/9F`） | — | 只记诊断，不改位置 |

统一成 `pos_01deg`（0.1°）是这层的关键设计：上层只认一个单位，换上报码不用改业务代码。

### 3.3 观测链：1 ms 内的换算顺序

```
raw  = (ZDT) pos_01deg × 0.0017453  → rad       |  (RS) pos 直接是 rad
raw_mech = DIR[i] × raw                        ← 方向（+1 同向 / -1 反向）
angle    = (raw_mech − ZERO[i]) / GEAR[i]      ← 扣零位、除减速比
joint_fb = wrap(angle)  → [-π, π)              ← 归一化
joint_vel= wrap(raw_mech − raw_prev) / GEAR / dt
valid    = (收到过帧) && (now − stamp ≤ 20 ms)
FK_SolveAll(joint_fb, joint_pose)              ← 每关节位姿 + 末端位姿
cc_control_data.pos[0..5] = joint_fb           ← 供上行
terminal_dbg.* = 镜像                          ← 供 Watch
```

顺序不能换：**方向必须在扣零位之前**，因为零位也是按"电机侧机械角"存的（`zero_readback` 抄的就是 `joint_raw`，已乘方向）。

### 3.4 上行链：39 字节怎么排

```
偏移  长度  字段            值
 0     1    sof             0xA5
 1     2    data_length     30（小端）
 3     1    seq             0~255 循环，双口共用一个 seq
 4     1    crc8            对 [0..4] 共 5 字节
 5     2    cmd_id          0x0302（小端）
 7    30    data            7×float = 28 B，前 6 个是关节角(rad)，[6] 保留 = 0，末 2 B 补 0
37     2    frame_tail      CRC16，对整帧 39 B
```

- 结构体必须 `__packed`，否则 `uint16_t` 会对齐到 2 字节边界，帧长变 40 B、CRC 全错。
- `cc_control_data.pos` 是 `float[7]`（28 B），`memcpy` 进 30 B 的 data 段后，剩余 2 B 恒 0。上位机解析时**跳过 `pos[6]`**。
- 带宽：39 B × 30 Hz = 1170 B/s，占 115200 8N1（11520 B/s）的 **10.2%**；两个口各占各的，互不影响。
- 发送自愈（`custom_terminal.c:476-486`）：`gState != READY` 时跳过并计数，连续 10 拍（340 ms）就 `HAL_UART_AbortTransmit()` 强拆。这条是必要的——DMA 发送的 `gState` 只能靠 TC 中断释放，而 TC 中断依赖 `USART1_IRQHandler` 里补的那句 `HAL_UART_IRQHandler(&huart1)`。

### 3.5 下行链：为什么是 100 ms

`communication_task` 每 100 ms 跑一次 `DataUnpack()`，从 FIFO 里逐字节找 `0xA5`：

```
帧头 5 B：sof(1) + len(1) + id(1) + type(1) + crc8(1)
数据 20 B：固定长度（DATA_LEN，与 len 字段无关）
帧尾 2 B：CRC16
```

解出来的载荷按 `id` 存进 `BOARD_RX_DATA[id][0..20]`（第 0 字节存长度），由各模块自行取用。**本工程的自定义终端不消费任何下行指令**——`custom_terminal` 与 `communication` 已完全解耦（`communication.c:31` 明确不 include `custom_terminal.h`）。

代价：下行延迟最坏 100 ms；且 100 ms × 11520 B/s = 1152 B > FIFO 1024 B，**PC 持续满速发就会溢出**（见 §6 P2-6）。

### 3.6 端到端延迟预算

| 段 | 延迟 | 来源 |
| --- | --- | --- |
| 电机采样 → 帧发出 | ≤ 10 ms | ZDT 上报周期 |
| CAN 传输（8 B @1 Mbps） | ~130 µs | 帧本身 |
| 中断 → 驱动缓存 | < 10 µs | 中断上下文 |
| 缓存 → `joint_fb` 更新 | ≤ 1 ms | 任务周期 |
| `joint_fb` → 上行帧 | ≤ 34 ms | 上行周期 |
| 串口发送 39 B @115200 | 3.4 ms | 39×10/115200 |
| **合计（关节角到 PC）** | **≤ 45 ms** | 主手用足够 |
| 下行（PC → 板） | ≤ 100 ms + 传输 | 只用于配置，可接受 |

---

## 4. 控制链路

### 4.1 八步逐函数

| 步 | 函数 | 周期 | 读 | 写 | 当前行为 |
| --- | --- | --- | --- | --- | --- |
| Publish | `CustomControllerPublish` | 一次 | — | — | 空 |
| Init | `CustomControllerInit` | 一次 | 参数表 | CAN | 5 台 ZDT 失能 → RS00 使能 → 5 台配定时上报；`ct_send_phase=0` |
| 1 | `CustomControllerObserver` | 1 ms | 驱动缓存、`HAL_GetTick` | `custom_terminal.*`、`joint_pose`、`cc_control_data`、`terminal_dbg` | 换算 + 正解 + 自愈补发 + 零位捕获 |
| 2 | `CustomControllerHandleException` | 1 ms | — | — | **空实现**（无过温/堵转/故障位处理） |
| 3 | `CustomControllerSetMode` | 1 ms | — | — | 空（当前只有"手动测试"与"重力补偿"两条隐含通路） |
| 4 | `CustomControllerReference` | 1 ms | `joint_fb` | `terminal_dbg.tau_g` | 只算重力前馈，**不写 `joint_reference`** |
| 5 | `CustomControllerConsole` | 1 ms | `terminal_dbg`、`tau_g` | `joint_torque[]`、`armed` | 手动模式 / 重力模式二选一，限幅 + 限速 |
| 6 | `SendCmd` | 1 ms | `joint_torque[]` | CAN | 3 拍轮转发 2 帧 |
| 7 | `SendToPC` | 34 ms | `cc_control_data` | UART DMA | 组 39 B 帧，双口各发一份 |

### 4.2 两条力矩通路

```
输入（全部来自 Keil Watch 里可写的 terminal_dbg）
  manual_en / manual_joint / manual_torque_nm / manual_ignore_feedback
  gravity_en / gravity_scale
              │
              ▼
CustomControllerConsole()
  ├─ 手动通路：valid 门禁 → clamp → step_toward → joint_torque[6]
  └─ 重力通路：scale × tau_g → clamp → step_toward → joint_torque[6]
              │
              ▼
CustomControllerSendCmd()   3 拍轮转
  ├─ ZDT ：N·m × MA_PER_NM → int32 → F5 帧
  └─ RS00：N·m → float_to_uint(12 bit) → MIT 帧
```

**互斥关系**：`armed`（手动模式武装）为真时重力补偿不叠加（`custom_terminal.c:368`）。反过来，重力模式下手动通道被 `enable = false` 清空。这是刻意设计——两路力矩叠加会立刻超出限幅且无法归因。

**手动模式的使能状态机**（`custom_terminal.c:308-365`）：

```
[进入出力]  manual_en=1 且 valid（或 ignore_fb=1）
            · 换关节：先失能旧关节，再使能新关节
            · 同关节：不发重复使能命令（ct_enabled_joint 记录）
[退出出力]  manual_en=0 或 valid=0
            · 目标力矩置 0 → 按 0.02 N·m/ms 斜坡回零
            · 显式失能（ZDT: F3 AB 00 00 6B / RS00: FF…FD）
```

"使能命令只在状态切换时发一次"很关键：1 ms 周期里每拍刷 5 条使能帧会把总线占满，也会让驱动器反复响应。

### 4.3 单位换算链（全工程最容易错的地方）

**角度**（电机 → 关节）：

```
ZDT ：0.1° --(×0.0017453293)--> rad
RS00：rad ─────────────────────┘
       └─ (×DIR) ─ (−ZERO) ─ (÷GEAR) ─ wrap ─▶ [-π, π)
```

**力矩**（关节 → 电机）：

```
ZDT:  N·m --×MA_PER_NM--> mA --clamp(±5000)--> int32 --拆符号+幅值--> F5 <dir> FF FF <curH> <curL> 00 6B
RS00: N·m --clamp(±14)--> float --float_to_uint(12bit, ±14)--> 拼进 8 B MIT 帧
```

两个细节：

1. ZDT 的 `MA_PER_NM = 800` 是"5000 mA ÷ 6.25 N·m"反推的，**不是实测值**，必须按 [calibration_and_validation.md](./calibration_and_validation.md) 的 S5 用砝码标定。
2. RS00 的 MIT 帧里 `pos/vel/kp/kd` 全填 0，只有前馈力矩非零。因为 `kp = 0`，位置项不参与运算；但如果哪天有人打开 `kp`，`pos = 0` 会被编码成 `P_MIN = -12.57 rad`，电机会直接冲过去——驱动里为此专门写了注释（`CAN_cmd_robostride.c:131-135`）。

### 4.4 发送时序：3 拍轮转

```
t=0 ms   phase 0: J0(ZDT addr1) + J1(RS00)      ─┐
t=1 ms   phase 1: J2(addr3) + J3(addr4)          │ 每台电机 3 ms 收到一帧
t=2 ms   phase 2: J4(addr5) + J5(addr6)          │ → 等效 333 Hz
t=3 ms   phase 0: ...                            ─┘
```

为什么不是每拍发 6 帧：6 帧 8 B 扩展帧在 1 Mbps 上约 6×130 µs = 780 µs，加上 RS00 的标准帧已经接近 1 ms 的 90%，没有任何余量给反馈帧。3 拍轮转把负载压到 1/3，代价是力矩刷新率降到 333 Hz——对主手（人手带宽 < 10 Hz）完全够。

ZDT 的 `F5` 是电流指令，驱动器会**保持**上次电流直到下一条命令，所以 3 ms 间隔不会掉力。

### 4.5 五道安全闸

| # | 闸 | 位置 | 阈值 | 触发后果 |
| --- | --- | --- | --- | --- |
| 1 | 反馈超时 | `Observer` | 20 ms 无新帧 → `valid = false` | 手动模式不出力 |
| 2 | 硬限幅 | `Console` | ZDT 5000 mA、RS00 14 N·m | 力矩截断 |
| 3 | 变化率限制 | `Console` | 0.02 N·m/ms（= 20 N·m/s） | 斜坡逼近，不阶跃 |
| 4 | 单关节互锁 | `Console` | 只有被选中关节出力 | 其余恒 0 |
| 5 | 上电失能 | `Init` | 显式 `F3 AB 00 00 6B` | 上电不带力 |

**缺口**：无软限位、无急停、无过温/堵转处理。`HandleException` 是空钩子，`robostride` 的 `fault/warning` 位和 ZDT 的错误码都没人读。

### 4.6 控制律现状

- `joint_reference[]` 定义了但**无人写入**，是给以后的位置环预留的。
- 没有 PID、没有阻抗、没有逆解。
- 所以当前的"控制"只有：**前馈力矩 + 斜坡 + 限幅**。这不是位置控制，是"能托住/能施加已知力矩"的开环力矩控制。

---

## 5. 参数总表

全部集中在 `application/typedef_robot/robot_param_terminal.h`（代码里不再出现机械数字）：

| 宏 | 当前值 | 含义 | 是否已标定 |
| --- | --- | --- | --- |
| `CUSTOM_TERMINAL_CAN` | 1 | 全部关节走 CAN1 | ✅ |
| `CUSTOM_TERMINAL_JOINT_NUM` | 6 | 关节数 | ✅ |
| `CUSTOM_TERMINAL_RS_JOINT` | 1 | J1 是 RS00 | ✅ |
| `CUSTOM_TERMINAL_ZD_ADDR` | `{1,0,3,4,5,6}` | 各关节 ZDT 地址，0 = 非 ZDT | ⚠ 地址 2 空置 |
| `CUSTOM_TERMINAL_RS_ID` | `0x05` | RS00 的 CAN_ID | ⚠ 待确认 |
| `CUSTOM_TERMINAL_RS_MASTER_ID` | `0x15` | 应答帧主机 ID | ⚠ 待确认 |
| `CUSTOM_TERMINAL_JOINT_ZERO` | 全 0 | 零位（电机侧 rad） | ❌ 占位 |
| `CUSTOM_TERMINAL_JOINT_DIR` | 全 +1 | 方向 | ❌ 占位 |
| `CUSTOM_TERMINAL_JOINT_GEAR` | 全 1.0 | 减速比 | ❌ 占位 |
| `CUSTOM_TERMINAL_JOINT_MA_PER_NM` | `{800,0,800,800,800,800}` | N·m→mA | ❌ 反推值 |
| `CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA` | 5000 | ZDT 电流上限 | ✅ 手册 |
| `CUSTOM_TERMINAL_RS_TORQUE_MAX_NM` | 14.0 | RS00 力矩上限 | ✅ 手册 |
| `CUSTOM_TERMINAL_FEEDBACK_TIMEOUT_MS` | 20 | 反馈超时 | ✅ |
| `CUSTOM_TERMINAL_REPORT_RETRY_MS` | 1000 | 上报配置自愈周期 | ✅ |
| `CUSTOM_TERMINAL_ZD_REPORT_PARAM` | `0x31` | 上报码（单圈） | ✅ |
| `CUSTOM_TERMINAL_LINK_MASS` | 6 个质量 | 连杆质量 | ❌ 估算 |
| `CUSTOM_TERMINAL_LINK_COM` | 6 个质心 | 连杆质心 (mm) | ❌ 估算 |
| `CUSTOM_TERMINAL_GRAVITY_SCALE` | 0.0 | 重力投用比例 | ✅ 默认关 |

机构参数在 `terminal_kinematics.c`：`arm_dh_param[6]`（`a` = 190/87/150.5 mm，`alpha`/`d` 全 0）+ `base_transform`（单位阵）。

---

## 6. 现状评估

### 6.1 已经闭环的

1. **反馈链路**：定时上报 + 自愈补发，`fdb_cnt` 能证明帧在刷新。
2. **上行链路**：39 B 定长帧 + 双口 DMA + 卡死自愈，30 Hz 稳定。
3. **单关节力矩**：使能状态机、限幅、限速、无反馈保护、换关节自动失能。
4. **零位标定入口**：`zero_capture` → `zero_readback` → 回填参数表。
5. **重力补偿算法**：已实现并挂在 `Reference`，默认关闭，可分级投入。

### 6.2 代码级问题（按严重度）

**P0-1 重力通道绕过"无反馈不出力"保护**
`CustomControllerConsole` 的重力分支（`custom_terminal.c:368-380`）没有检查 `joint_feedback_valid`，而 `SendCmd`（`custom_terminal.c:392-398`）里那个 `tau = valid ? torque : 0` 只写进了 `terminal_dbg.torque_cmd[]`，真正下发的仍是 `custom_terminal.joint_torque[i]`。结果：**某关节反馈掉线后，重力补偿仍按陈旧角度持续出力**，而 Watch 上显示的是 0。修法：把 valid 门禁提到 `Console` 的公共出口，或让 `SendCmd` 用 `tau` 而不是 `joint_torque`。

**P0-2 DH 表不是真 DH**
`arm_dh_param[]` 的 `alpha` 与 `d` 全 0，`terminal_dh_transform()` 实际算的是 `Rj · T_fixed`，第三列不是旋转轴。后果：`terminal_gravity()` 里 `LinkAxis()` 求出的关节轴方向是错的，**重力补偿方向可能反向**（反向加速下坠）。接重力前必须换成真 DH 或改用旋量（PoE）建模。

**P0-3 无软限位 / 无急停**
只有力矩上限。做联动前必须加关节角限位；急停目前只能靠断电。

**P1-1 速度计算的时间基准错了**
`dt_ms = now − joint_last_feedback_time[i]`，而 `joint_last_feedback_time[i]` 在**同一拍就被更新成 `now`**（`custom_terminal.c:202/216`），所以 `dt_ms` 恒等于 1。ZDT 是 10 ms 才来一帧，于是速度表现为"9 拍为 0、第 10 拍跳 10 倍"。当前没人用 `joint_velocity`，但一旦接速度环就会炸。修法：改用 `stamp − last_stamp` 做分母，或在驱动里算。

**P1-2 `wrap` 在关节侧，压缩了量程**
`joint_fb = wrap((raw − zero)/GEAR)`。若 `GEAR > 1`，关节侧行程被压缩到 `±π/GEAR`——减速比 10 的关节只有 ±18° 就回绕。正确做法是先在电机侧累加多圈再除减速比（或直接用 `0x36` 的多圈位置）。

**P1-3 CAN 关闭了自动重传与总线恢复**
`Src/can.c:42-44`：`AutoRetransmission = DISABLE`、`AutoBusOff = DISABLE`。仲裁失败即丢帧（靠 1 s 自愈兜底），一旦进入 bus-off 不会自恢复。主手场景可接受，但要在文档里写明"上电顺序：先电机后板子"。

**P1-4 USART6 与裁判系统共用**
`CustomControllerSendToPC` 每 34 ms 往 huart6 发 39 B，而 `referee_usart_task.c` 在同一个口上做 DMA 双缓冲接收。两者不能同时接设备，属于**互斥使用**，代码里没有互斥保护。

**P2-1 变体关闭时函数重复定义**
`custom_terminal.c` 的 `#else` 分支（第 508 行）定义了 `custom_terminal_get_joint_feedback()`，`#endif` 之后（第 520 行）又定义了一次。`CUSTOM_CONTROLLER_TERMINAL` 打开时只编到一个，但一旦切到 `NONE` 就会重复定义报错。

**P2-2 正解每周期算两遍**
`Observer` 调一次 `FK_SolveAll`，`Reference` 里的 `terminal_gravity` 又重算一遍整条链。可以把 `T[i]` 缓存下来复用，省一半浮点。

**P2-3 `HandleException` 空实现**
`robostride_dbg` 里的 `fault/warning`、ZDT 的 `0xE2/0xEE` 错误码都没人消费。掉线只影响出力，不影响报警。

**P2-4 文档与代码不一致**
`architecture.md` 说 `cc_control_data.pos[6]` 恒为 `0xA7`，代码里是 `memset(0)` 后只拷 28 B，实际为 `0`。以代码为准。

**P2-5 下行 `data_id` 未做边界检查**
`DataUnpack()` 里 `BOARD_RX_DATA[data_id][0]`，`data_id` 来自帧头字节，范围 0~255，而数组只有 8 项（`communication.h:10`）。PC 发一个 `id = 100` 就越界写。

**P2-6 下行 FIFO 容量小于一个轮询周期**
FIFO 1024 B，而 100 ms 内 115200 可进 1152 B。PC 持续满速发（例如高频下发关节目标）时每轮丢约 128 B，表现为"偶发丢帧"。修法：轮询降到 50 ms，或把 FIFO 提到 2 KB，或改用空闲中断触发解包。

### 6.3 演进路线（建议顺序）

| 阶段 | 内容 | 前置 |
| --- | --- | --- |
| S1 | 修 P0-1（valid 门禁统一到出口）+ P1-1（速度时间基准） | 无 |
| S2 | 实测填 `DIR/GEAR/ZERO/MA_PER_NM` 四张表 | 支架 + 砝码 |
| S3 | 换真 DH（或 PoE），验证 `tau_g` 方向与量级 | S2 完成 |
| S4 | 加软限位 + 过温/堵转处理（`HandleException` 落地） | S3 |
| S5 | 位置环/阻抗环：把 `joint_reference` 真正用起来 | S4 |
| S6 | 若上位机要做力反馈/主从跟随：上行扩到 12 通道（力矩 + 温度） | S5 |

---

## 7. 改动指引

| 需求 | 改动点 | 注意 |
| --- | --- | --- |
| 加一个关节 | `robot_param_terminal.h` 加一项（`ZD_ADDR/DIR/GEAR/ZERO/MA_PER_NM`）→ `JOINT_NUM` +1 → `SendCmd` 轮转多分一拍 | 关节循环自动覆盖，但轮转相位是硬编码的 |
| 换电机 | `motor.h` 的 `MotorType_e` → `robot_cmd/` 照 `CAN_cmd_zdatou.c` 的"自持缓存 + 诊断 + `xxx_on_rx`"写 → `CAN_receive.c` 分发加一行 → `GetMotorMeasure()` 加 case | 驱动必须返回"是否认领"，否则会抢别人的帧 |
| 改上行协议 | `custom_terminal.h` 的 `CUSTOM_TERMINAL_UPLINK_*`（SOF/cmd_id/长度） | 与上位机同步；`__packed` 不能丢 |
| 加控制律 | 在 `Console` 里替换手动分支，输入用 `joint_reference` + `joint_fb`，输出写 `joint_torque` | 保留限幅与限速；`SendCmd` 不用改 |
| 关掉测试模式 | `CUSTOM_TERMINAL_TEST_MODE = 0` | 手动通道整体不编译，`ct_torque_nm` 相关静态变量一并消失 |
| 新增源文件 | 同步 `MDK-ARM/.eide/eide.yml` | 否则静默不编译，报 `L6218E` |
| 构建 | `UV4.exe -b MDK-ARM/StandardRobotPro.uvprojx -j0 -o build.log`（AC5） | — |

---

## 附：关键代码地图

| 关注点 | 文件:行 |
| --- | --- |
| 任务循环 | `custom_controller_task.c:47-82` |
| 上电时序 | `custom_terminal.c:103-135` |
| 角度换算 + 正解 + 上行填充 | `custom_terminal.c:147-252` |
| 重力前馈调用 | `custom_terminal.c:257-267` |
| 手动力矩状态机 | `custom_terminal.c:308-365` |
| 重力投用 | `custom_terminal.c:368-380` |
| 3 拍轮转发帧 | `custom_terminal.c:390-438` |
| 上行组帧 + 双口 DMA | `custom_terminal.c:442-501` |
| DH 表 | `terminal_kinematics.c:32-39` |
| 单关节变换 | `terminal_kinematics.c:78-136` |
| 正解链 | `terminal_kinematics.c:229-245` |
| 重力矩投影 | `terminal_gravity.c:57-129` |
| ZDT 解码 | `CAN_cmd_zdatou.c:180-243` |
| RS00 MIT 打包 | `CAN_cmd_robostride.c:137-158` |
| CAN 中断分发 | `CAN_receive.c:336-351` |
| 下行解包 | `communication.c:277-309` |
| CAN 发送（阻塞等邮箱） | `bsp/boards/bsp_can.c:56-91` |
| 任务创建 | `Src/freertos.c:218-221` |
