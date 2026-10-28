# 自定义控制器 / 自定义终端 通信详解

> 依据：RoboMaster 2026 通信协议 V2.0.0 第 1.4 节（图传链路）、1.5 节（非链路）、第 2 章（自定义客户端协议）
> 对照工程：`D:\RM\new_remoto\new_remoto`（C 板下板 · 六轴主臂 = 自定义控制器）
> 编写时间：2026-10-28

---

## 目录

1. [三个容易被混用的名字](#1-三个容易被混用的名字)
2. [链路与带宽预算](#2-链路与带宽预算)
3. [0x0302 帧的逐字节构造](#3-0x0302-帧的逐字节构造)
4. [30 字节数据段怎么设计](#4-30-字节数据段怎么设计)
5. [下行 0x0309：本项目缺的那一半](#5-下行-0x0309本项目缺的那一半)
6. [0x0310 / 0x0311：给 PC 客户端](#6-0x0310--0x0311给-pc-客户端)
7. [0x0306：模拟选手端键鼠](#7-0x0306模拟选手端键鼠)
8. [本项目的实时性链路](#8-本项目的实时性链路)
9. [代码对照](#9-代码对照)
10. [隐患与改进清单](#10-隐患与改进清单)

---

## 1. 三个容易被混用的名字

| 名词 | 协议里的位置 | 是什么 | 本项目 |
|---|---|---|---|
| **自定义控制器** | 1.4 节 / 0x0302、0x0309、0x0306 | 队伍自制的操作端（摇杆盒、主臂、手柄…），一根串口接到**选手端** | 本板：六轴主臂就是自定义控制器 |
| **自定义客户端** | 第 2 章 + 0x0310 / 0x0311 | 跑在**电脑**上的程序，经选手端与机器人通信 | 未实现 |
| **自定义终端** | 项目内部叫法，协议里没有 | 六轴机械臂（J0~J5）这套 C 板固件的代号 | `custom_terminal.c` |

关键认知：**协议只规定了"信封"（cmd_id + 30 字节），信封里装什么完全由队伍自己定**。
所以 0x0302 不是"遥控器协议"，而是"你和你自己的机器人之间的一根 30 字节管道"。

---

## 2. 链路与带宽预算

```
操作手 ── 自定义控制器 ──[115200, 0x0302 上行 ≤30Hz]── 选手端
                                                        │
                                             图传（无线，延迟 ~130ms）
                                                        │
                                                    机器人主控
                                                        │
        自定义控制器 ◄──[115200, 0x0309 下行 ≤10Hz]── 选手端
```

| 方向 | cmd_id | 频率上限 | 数据段 | 折算带宽 |
|---|---|---|---|---|
| 控制器 → 机器人 | 0x0302 | 30 Hz | 30 B | 900 B/s |
| 机器人 → 控制器 | 0x0309 | 10 Hz | 30 B | 300 B/s |
| 控制器 → 选手端（模拟键鼠） | 0x0306 | 30 Hz | 8 B | 240 B/s |

115200 8N1 的理论吞吐是 **11520 B/s**，所以 0x0302 只占约 **8%**，带宽不是瓶颈。

**真正的瓶颈是延迟**：裁判系统链路延迟约 130 ms（恶劣 200 ms）。30 Hz 下发意味着 **端到端约 4 帧在途**。由此得到两条设计铁律：

1. **不要在"自定义控制器"里做需要低延迟的闭环**——你的反馈是 130 ms 前的。
2. **本地闭环 + 远端参考**：机械臂的力矩环、位置环在本地（本项目就是 1 ms 环），上行只发"目标/参考量"，机器人侧自己插值跟随。

---

## 3. 0x0302 帧的逐字节构造

### 3.1 通用帧（与裁判系统同构）

```
偏移  0     1-2        3     4      5-6      7 … 36      37-38
     A5  len=0x001E  seq  CRC8   02 03    data[30]     CRC16
```

- `data_length = 30`（**只算 data，不含 cmd_id，不含帧尾**）
- `cmd_id = 0x0302`，小端 → `02 03`
- 整帧 = 30 + 9 = **39 字节**
- CRC8：只算前 4 字节，写在偏移 4
- CRC16：算前 37 字节（帧头 + cmd_id + data），写在 37/38，**低字节在前**

### 3.2 本项目的实际代码

`application/custom_controller/custom_terminal.c:472`：

```c
typedef __packed struct {
    uint8_t  sof;                    // 0xA5
    uint16_t data_length;            // 30
    uint8_t  seq;
    uint8_t  crc8;
    uint16_t cmd_id;                 // 0x0302
    uint8_t  data[30];
    uint16_t frame_tail;             // CRC16
} custom_terminal_uplink_frame_t;    // 39 B
```

`custom_terminal.c:497` 的发送流程（逐行对照）：

```c
const uint8_t seq = custom_terminal_uplink_seq++;      // 两路口共用同一个 seq（:499）
for (port = 0; port < 2; port++) {
    if (uart->gState != READY) { busy_cnt++; if (busy_cnt >= 10) AbortTransmit(); continue; }
    frame->sof         = 0xA5;
    frame->data_length = 30;
    frame->seq         = seq;
    frame->crc8        = 0;
    frame->cmd_id      = 0x0302;
    append_CRC8_check_sum(frame, 5);                   // 算前 4 字节，写 offset 4  ✅
    memset(frame->data, 0, 30);
    memcpy(frame->data, cc_control_data.pos, sizeof(cc_control_data.pos));  // 28 B
    append_CRC16_check_sum(frame, 39);                 // 算前 37，写 37/38        ✅
    HAL_UART_Transmit_DMA(uart, frame, 39);
}
```

这一段的 CRC 覆盖范围、小端、`data_length` 语义**都是对的**，可以直接作为模板复用。

### 3.3 六个必须记住的细节

1. `data_length` **不包含** `cmd_id`，也不包含帧尾。
2. `cmd_id` 虽然不在 `data_length` 里，但**要参与 CRC16**。
3. 结构体必须 `__packed`。`uint16_t` 在偏移 1 处是非对齐访问，Cortex-M4 允许，但换到 M0/M7 要小心。
4. `float` 用 IEEE-754 直接 `memcpy`——**两端必须同架构同字节序**（都是小端 ARM 才行）。跨 x86 上位机也没问题（x86 也是小端）。
5. 发送前把 `crc8` 字节清零再算，否则旧值会留在结构体里（本项目每帧都重算，无影响；但如果改成增量更新就会踩坑）。
6. `HAL_UART_Transmit_DMA` 期间**不能修改这块 buffer**。本项目为每个口各准备一份 `custom_terminal_uplink_frame[2]`，所以安全；只准备一份就会 DMA 读到半新半旧的数据。

---

## 4. 30 字节数据段怎么设计

### 4.1 本项目现状

```c
typedef struct { float pos[7]; } CCControlData_t;   // 28 B
```

- `pos[0..5]` = J0~J5 关节角（rad）
- `pos[6]` 保留、恒 0
- 数据段 30 B，实际用 28 B，尾 2 B 补零

**优点**：极简、float 精度高（24 位尾数）、上下位机共用同一个结构体定义就不会错。
**缺点**：没有版本号、没有状态位、没有时间戳、没有关节数协商。一旦将来改成 8 个关节或加夹爪，旧固件会静默解析错。

### 4.2 建议的 v2 布局（正好 30 B）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | `uint8_t` | `version` | 0x02；对端不认就丢弃 |
| 1 | `uint8_t` | `flags` | bit0 标定中、bit1 急停、bit2 夹爪有效、bit3 重力补偿开 |
| 2-3 | `uint16_t` | `tick_ms` | 低 16 位毫秒计数，用于对端算丢包/延迟 |
| 4-27 | `float[6]` | `joint_rad[6]` | J0~J5，单位 rad |
| 28-29 | `int16_t` | `gripper` | 0~1000 归一化；`flags.bit2 = 0` 时忽略 |

```c
typedef __packed struct {
    uint8_t  version;        // 0x02
    uint8_t  flags;
    uint16_t tick_ms;
    float    joint_rad[6];
    int16_t  gripper;
} cc_uplink_v2_t;            // 30 B 整
```

要点：

- **`version` 放第一个字节**，对端先判版本再解析，以后扩容不会崩。
- **`tick_ms` 是排查丢包最快的手段**：对端记录两次 `tick_ms` 的差，和本地时钟比，立刻知道掉了多少帧。
- **急停位放在数据里**比"停发"安全：停发和丢包在接收端无法区分，急停位可以区分。
- 关节角若带宽紧张可换成 `int16_t` 定点（rad × 10000，范围 ±3.27 rad，分辨率 0.0001 rad），6 个关节只要 12 B，省下的字节可以放电流/温度。

### 4.3 下行 0x0309 建议布局（同样 30 B）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | `uint8_t` | `version` |
| 1 | `uint8_t` | `flags`（bit0 机器人使能、bit1 限位、bit2 碰撞） |
| 2-3 | `uint16_t` | `tick_ms` |
| 4-7 | `float` | `end_effector_x` |
| 8-11 | `float` | `end_effector_y` |
| 12-15 | `float` | `end_effector_z` |
| 16-27 | `float[3]` | 末端力 `fx, fy, fz`（力反馈，可选） |
| 28-29 | `int16_t` | `gripper_fb` |

下行只有 10 Hz，够刷状态、够做"虚拟弹簧"的力反馈，但**不够做硬的力觉透明**（10 Hz 的力反馈会让手感发粘），所以力反馈一般只在"接触检测/提示"级别使用。

---

## 5. 下行 0x0309：本项目缺的那一半

### 5.1 现状

全工程**没有 0x0309 的解析**。`huart1` 上确实有一路接收，但那是私有板间帧：

```c
// communication.h:16
typedef struct {
    struct { uint8_t sof; uint16_t data_length; uint8_t seq; uint8_t crc8; } frame_header;  // 5 B
    uint8_t  data[20];
    uint16_t crc;
} BoardCommunicateData_s;   // 27 B
```

用 `data_id`（`FRAME_HEADER_ID_OFFEST = 2`）把 8 路数据分流到 `BOARD_RX_DATA[8][21]`（`communication.c:47`、`:281-312`）。这跟 0x0302/0x0309 是**完全不同的一套**，不能互通。

### 5.2 后果

- 操作手只能"看见"自己主臂的关节角（本地反馈），**看不见机器人那侧的状态**。
- 无法做双边控制（力反馈遥操作）、无法做从臂跟随误差补偿、无法在从臂限位时给操作手阻力。
- 机器人侧如果发 0x0309，本板会当垃圾丢掉——**不会报错，静默丢弃**，这是最难查的一类问题。

### 5.3 怎么加（改动很小）

`referee_usart_task.c` 的帧状态机本身就是通用的（只认 `A5 + len + seq + crc8 + cmd_id + data + crc16`），只要：

1. 把 `huart6` 的接收 FIFO 同时喂给两个解析器，或者在一个解析器里按 `cmd_id` 分流；
2. `protocol.h` 的 cmd_id 枚举加 `ROBOT_TO_CUSTOM_CONTROLLER = 0x0309`；
3. 加一个 `custom_controller_rx.c`，把 30 字节按 4.3 的结构体解开；
4. **注意 `REF_PROTOCOL_FRAME_MAX_SIZE = 128` 这个门限**（`referee_usart_task.c:124` 限制 `data_len < 119`）——0x0309 只有 30 B 没事，但 0x0310 的 300 B 会被直接丢掉。

---

## 6. 0x0310 / 0x0311：给 PC 客户端

这两个是 2026 新增的，**不是给自定义控制器的**，是给"自定义客户端"（电脑程序）的：

| cmd_id | 长度 | 方向 | 频率上限 |
|---|---|---|---|
| 0x0310 | 300 B | 机器人 → 客户端 | 50 Hz |
| 0x0311 | 30 B | 客户端 → 机器人 | 75 Hz |

它们走的是串口（经选手端转），**而协议第 2 章的"自定义客户端协议"走的是 MQTT + protobuf**（选手端上的 `127.0.0.1:3333`，topic 如 `KeyboardMouseControl`、`CustomControl`，QoS ExactlyOnce，附录三给了 C# MQTTnet 示例）。

两套是同一功能的两种形态：
- **MQTT/protobuf**：PC 直连选手端本机服务，结构化、好扩展，适合 PC 客户端。
- **0x0310/0x0311**：走串口，适合用 MCU 做客户端，或者需要在图传链路上透传 300 B 大包。

如果只是想让操作手用电脑看机械臂状态，**优先用 MQTT**，不用改固件；只有当客户端是另一块单片机时才需要 0x0310/0x0311。

---

## 7. 0x0306：模拟选手端键鼠

8 字节，**非链路**（控制器直接插选手端/电脑，不过裁判系统）：

```c
typedef __packed struct {
    uint16_t key_value;
    uint16_t x_position : 12;   // 绝对像素 1920×1080
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right: 4;
    uint16_t reserved;
} custom_client_data_t;
```

用途：让自定义控制器**冒充选手端的键鼠**去操作机器人（比如用主臂上的按键触发机器人上的小地图标记/自动瞄准）。注意坐标是**绝对像素**不是增量，且 0x0304（旧的键鼠命令码）在 2026 已被删除。

---

## 8. 本项目的实时性链路

```
        1 ms 控制环（custom_controller_task）
        ├─ Observer   读 CAN 缓存 → 关节角 → 正解 → 填 pos[0..5]
        ├─ Reference  重力前馈 tau_g
        ├─ Console    标定/测试模式：单关节力矩
        └─ SendCmd    3 拍轮发（每拍 2 帧），单台电机 3 ms 一帧

        34 ms（≈29.4 Hz）
        └─ SendToPC   → 0x0302 39 B → huart1 + huart6
```

**延迟预算**（从电机转到操作手屏幕）：

| 环节 | 耗时 |
|---|---|
| ZDT 定时上报周期 | 10 ms（`CUSTOM_TERMINAL_REPORT_RETRY_MS` 是重发配置，上报本身 10 ms） |
| CAN 传输 + 中断 | < 1 ms |
| 打包等待（34 ms 周期，平均等一半） | ~17 ms |
| UART 39 B @115200 | ~3.4 ms |
| 裁判/图传链路 | ~130 ms |
| **合计** | **≈160 ms** |

这解释了为什么**不能**把上行数据当成实时反馈用：它是一条 160 ms 的"回看通道"，只能用来显示和记录，不能进控制环。

---

## 9. 代码对照

| 功能 | 位置 |
|---|---|
| 上行帧结构 | `custom_terminal.c:472-481` |
| 双口列表 | `custom_terminal.c:484-485`（`{&huart1, &huart6}`） |
| 发帧函数 | `custom_terminal.c:497` `CustomControllerSendToPC()` |
| 共用一个 seq | `custom_terminal.c:499` |
| BUSY 自愈 | `custom_terminal.c:506-516`（连续 10 拍 Abort） |
| CRC8 / CRC16 | `custom_terminal.c:523` / `:527` |
| 帧长上限宏 | `custom_terminal.c:490`（`BUSY_ABORT_CNT 10`） |
| 上行状态量 | `custom_terminal.c:26` `cc_control_data` |
| Observer（填 pos） | `custom_terminal.c:148-281`，填数据在 `:266-269` |
| 力矩下发（3 拍轮发） | `custom_terminal.c:419-468` |
| 反馈门控 | `custom_terminal.c:423-428`（无反馈的关节力矩强制 0） |
| 标定/测试模式 | `custom_terminal.c:337-410`（`CUSTOM_TERMINAL_TEST_MODE 1`） |
| 任务节拍 | `custom_terminal.h:45`（1 ms）、`:49`（34 ms） |
| 任务主循环 | `custom_controller_task.c:54-80`，发帧触发 `:69-72` |
| 私有下行帧 | `communication.h:16`、`communication.c:281-312` |

---

## 10. 隐患与改进清单

### P0（现在就影响功能）

1. **没有 0x0309 下行** → 操作手看不到机器人状态，力反馈/从臂限位提示都做不了。见第 5 节。
2. **payload 无版本号** → 将来改结构体，旧固件会静默解析成乱值（不会报错）。加 `version` 字节是最便宜的保险。
3. **`memcpy(frame->data, cc_control_data.pos, sizeof(pos))` 没有编译期断言**（`custom_terminal.c:526`）。哪天 `pos` 从 7 个 float 改成 8 个（32 B），就会直接溢出 30 B 的 `data[]`。建议加：

```c
_Static_assert(sizeof(cc_control_data.pos) <= CUSTOM_TERMINAL_UPLINK_DATA_LEN,
               "uplink payload overflow");
```

4. **`ct_ma_per_nm[i]` 为 0 时会除零**（RS 关节 i=1 填 0）。`CustomControllerConsole:403` 和 `SendCmd` 里 5 处 `ZD_TORQUE_MAX_MA / ct_ma_per_nm[i]` 都是这样，目前靠 `i != 1` 的路径分支躲开，一旦参数表填错（比如把 0 填到 J2）就是 `inf` → `(int32_t)inf` 是未定义行为。建议在 `CustomControllerInit` 里做一次参数校验，非法就锁死不出力。

### P1（协议合规）

5. **`REF_PROTOCOL_FRAME_MAX_SIZE = 128`** 决定了 `data_len < 119` 才收（`referee_usart_task.c:124`），0x0310（300 B）永远收不到。要做自定义客户端必须改成 ≥ 309。
6. **0x0306 未实现**：若操作手想用主臂上的按键操作选手端 UI，需要补这 8 字节。
7. **两路口共用同一 seq**（`custom_terminal.c:504`）：好处是两路口径一致，坏处是某个口 BUSY 跳过时，该口对端会看到 seq 不连续，误判为丢包。如果对端用 seq 统计丢包率，建议**每口独立 seq**。

### P2（健壮性）

8. BUSY 自愈阈值 10 拍 × 34 ms = **340 ms 才恢复**。可以缩到 3 拍（≈100 ms），因为正常的 DMA 发送 39 B 只要 3.4 ms，连等 3 拍还没好基本就是状态机卡死。
9. 没有"上行节流"逻辑：`SendToPC` 无条件下发，如果 `huart6` 被裁判接收占用（同一 UART 全双工，OK）但上层任务来不及，DMA 会排队——目前靠 `gState != READY` 直接丢弃，属于合理策略，但**丢弃没有计数**，建议加一个 `drop_cnt` 便于现场判断。

---

## 附：一句话总结

**0x0302 是"你和你自己机器人之间的 30 字节管子"，协议只管信封不管内容。**
本项目把 6 个关节角（7 个 float，28 B）塞进去、30 Hz 发、CRC 全对，是合规的；
缺的是**反方向那 10 Hz 的 30 字节**（0x0309），它才是让操作手"感觉到"机器人的唯一通道。
