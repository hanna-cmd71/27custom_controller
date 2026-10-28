# 项目代码评审 —— `restore-full` 工作区（custom_terminal 重构）

> 评审对象：`D:\RM\new_remoto\new_remoto`，HEAD `74d4bcb` + 未提交工作区
> （相对 HEAD：34 files / +1788 / −2027；其中 20 个已跟踪文件在工作区被改、4 个改名 + 6 个删除已暂存、14 个未跟踪）
> 构建证据：`_check/uv4-build.log`（2026-10-26 10:54，全量）`0 Error(s), 13 Warning(s)`，
> Code=90720 RO-data=7260 RW-data=1360 ZI-data=62232；`_check/uv4-build2.log`（11:01，增量）`0/0`。
> 编译器：ARMCC V5.06u7；FreeRTOS + CMSIS-RTOS v1；target `StandardRobotPro`。
>
> **注意**：文中行号对应提交 `d73f81b` 的代码。之后 `custom_terminal.c` 删掉了变体守卫、
> 力矩门控位置也改了，行号已整体变化；已修项、误报、推迟项和硬件实测结论见 **§7 实施记录**。

---

## 0. 结论摘要

这次改动把「工程机器人自定义控制器」（`custom_controller*.c` + `controler_myself.c`，约 1000 行）
整体替换成「6 自由度自定义终端」（`custom_terminal.c` + `terminal_debug/kinematics/gravity.c`）。
**方向是对的**：分层比原来干净（驱动命令 → 关节量换算 → 正解 → 控制量 → 上行），
并且 `doc/calibration_and_validation.md` 先行给出了完整标定流程，`_check/` 里有可复现的 Keil 命令行编译日志。
代码本身也能编过。

但它现在只是**「能上电、能读角、能对单关节打力矩」的标定骨架**，还不是一个能跑的控制系统：
控制律空缺、力矩来源只有调试变量、重力补偿量纲和符号都错、安全门没接到实际发送路径上、参数全是占位值。
更关键的是**它有 6 个「一碰就炸」的问题**：
换机器人配置编译失败 / 断线不断力 / 重力补偿一开就满量程 / 提交出不可编译树 /
总线上来一帧就能越界写 RAM（CAN 驱动层，历史遗留）/
上位机一个合法帧就能写穿全局区（协议解析层，历史遗留）。

| 级别 | 数量 | 概述 |
|---|---|---|
| P0 阻断 | 6 | 重复定义、安全门失效、重力补偿 1000×+反号、工作区与 index 不一致、两处越界写（CAN_CyberGear / 上位机解析） |
| P1 正确性/安全 | 10 | NULL 解引用、断线不停力、速度量无效、配置分层反了、编码损坏、串口争用、ZDT 抢占 CyberGear 帧、上板误开终端 |
| P2 结构/可维护 | 9 | 1 kHz 全量正解、相位轮转硬编码、协议帧复制、文档与实现不符、通信缓冲无临界区 |
| P3 细节 | 8 | 编译告警、IDE 壁纸进库、.gitignore、混合编码、返回值未判等 |

**改完 P0 之前，不要把这个固件放到真机上通电带力矩运行。**

---

## 1. P0 —— 必须先改

### P0-1 `custom_terminal_get_joint_feedback` 被定义两次（换机器人配置就编译不过）

`application/custom_controller/custom_terminal.c`

- 第 488-493 行，在 `#else`（非 TERMINAL 分支，`:484` 开始、`:495` 结束）里定义了一份空实现；
- 第 500-503 行，在 `#endif` 之后又定义了一份真实现（文件共 505 行）。

```
484: #else /* 未启用自定义终端：给通信模块一个安全的空实现 */
488: const float *custom_terminal_get_joint_feedback(void)
489: {   static const float joint_feedback_zero[CUSTOM_TERMINAL_JOINT_NUM] = {0.0f};
492:     return joint_feedback_zero; }
495: #endif
500: const float *custom_terminal_get_joint_feedback(void)
501: {   return custom_terminal.joint_feedback; }
```

现在编得过纯属巧合：`robot_param.h:30` 只 include 了 `robot_param_omni_mini.h`，而它第 47 行把
`CUSTOM_CONTROLLER_TYPE` 设成 `CUSTOM_CONTROLLER_TERMINAL`，`#else` 分支根本不参与编译。
一旦切到 `robot_param_omni_sentry.h`（第 37 行 `NONE`）或 `robot_param_engineer_controller.h`（第 15 行 `NONE`），
两份定义同时出现 → **L6200E: Symbol multiply defined**。

这正好把「多机器人框架」这个卖点打掉了：框架的任何其它配置都编不过。
顺带说明：`robot_param_engineer_controller.h:15` 的注释写着「这里暂时置 NONE」——
**它现在置 NONE 就编不过**，这个注释是误导性的。

**修**（终端专用，最小改动，推荐）：删掉 484-495 这段 `#else` 桩（连 `#endif` 一起），
只保留 500-503 的真实现 —— 构建结果完全不变，地雷当场塌掉，还少 12 行。

⚠️ 注意别删错方向：**500-503 是 TERMINAL 构建真正在用的那份**（`#if` 成立时 `#else` 整段不参与编译），
删它会立刻 L6218E 未定义符号（`custom_controller_task.c` 的 `__weak` 桩只覆盖 9 个 `CustomController*`，
不含这个函数）。本报告早期版本给的「删 500-503」是错的，已更正。

若以后仍想让其它配置能编：把 500-503 上移到 `#if` 分支末尾（483 行之后），`#else` 桩保留 ——
这才真正表达了「按配置给不同实现」的意图。

### P0-2 「无反馈不出力」是假的，只写进了调试结构体

`custom_terminal.c:378-384`

```c
for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++) {
    float tau = custom_terminal.joint_feedback_valid[i] ? custom_terminal.joint_torque[i] : 0.0f;
    terminal_dbg.torque_cmd[i] = tau;          // ← 只存到调试变量
}
switch (ct_send_phase) {                        // ← 真正发出去的是未门控的值
case 0: zdatou_torque_control_i32(..., clamp_torque(custom_terminal.joint_torque[0], ...));
```

第 386-423 行每一帧用的都是 `custom_terminal.joint_torque[i]`，**完全没有看 `joint_feedback_valid`**。
也就是说：电机一旦断线/超时，`joint_feedback_valid[i]` 变 0，但固件仍然按最后一拍的力矩继续发。
`terminal_dbg.torque_cmd[]` 在 Watch 里显示的是 0，看起来像保护生效了 —— 这比没有保护更危险，因为排查时会误判。

**修**：把门控提到 switch 之前算出 `float tau[N]`，发送路径一律使用 `tau[]`；同时把
`joint_feedback_valid == 0` 接到「逐关节力矩归零 + 该关节失能」（见 P1-2）。

### P0-3 重力补偿量纲差 1000 倍，而且符号大概率是反的

`terminal_gravity.c:112` 与 `:127`，`custom_terminal.c:363`

```c
112: const float fz = -ct_link_mass[i] * GRAVITY * 1000.0f;   // GRAVITY=9.8 (m/s²)，struct_typedef.h:23
127: tau_nm[j] = tau / 1000.0f; /* N·mm → N·m */
```

单位对一遍：`fz` 的数值 = `m·g·1000`，即**以 mN 为单位**（1 N = 1000 mN）；
`r` 以 mm 为单位；于是 `tau` 的数值量纲是 **mm·mN = 1e-6 N·m**。
代码只除了一次 1000，得到的是「N·mm 的数值」而不是 N·m。

**数值验证**：1 kg 质心、力臂 100 mm
- `fz = 1 × 9.8 × 1000 = 9800`（mN = 9.8 N）
- `tau = 100 × 9800 = 980000`（mm·mN）
- 真值 = 980000 × 1e-6 = **0.98 N·m**
- 代码返回 `980000 / 1000 = 980`，**大了 1000 倍**

叠加 `custom_terminal.c:363` 的 `clamp_torque(gravity_scale * tau_g[i], limit)`（ZD 关节 5000mA/800 = 6.25 N·m，RS 关节 14 N·m），
结果就是：**只要 `gravity_en=1` 且 scale 非 0，每个关节立刻顶到力矩上限**。
现在之所以没事，是因为 `robot_param_terminal.h:81` 的 `CUSTOM_TERMINAL_GRAVITY_SCALE` 是 `0.0f`、`gravity_en` 默认 0。

符号：`tau_g` 算的是「重力作用在关节轴上的力矩」，补偿要用 `−tau_g`，而第 363 行用的是 `+`。
`doc/calibration_and_validation.md:129` 自己也写了「否则重力补偿会反向加速下坠」—— 说明这条已经怀疑过，
但代码里没有改。

**修**：`fz` 去掉 `* 1000.0f`（让 `fz` 用 N，`tau` 用 N·mm，`/1000` 正好得到 N·m），
或保留 mm 但改成 `/ 1.0e6f`；第 363 行改成 `-gravity_scale * tau_g[i]`。
改完先按 `calibration_and_validation.md` 的 G0-G4 分档（0.2/0.5/0.8/1.0）验证方向。

### P0-4 工作区和 index 不一致：现在 `git commit` 会得到一个不可编译的版本

```
$ git diff --cached --stat | tail -1
 10 files changed, 531 deletions(-)          ← 暂存区只有「删除 + 改名」，内容 0 改动
$ git status --porcelain
D  application/custom_controller/custom_controller{,_connect,_engineer}.c/.h   ← 已暂存删除
RM application/gimbal/controler_myself.c -> application/custom_controller/custom_terminal.c
?? application/custom_controller/terminal_{debug,gravity,kinematics}.c/.h     ← 新文件未暂存
 M MDK-ARM/StandardRobotPro.uvprojx                                          ← 工程文件未暂存
```

如果只 commit 暂存区：旧模块被删了、`custom_terminal.c` 里还是 `controler_myself.c` 的旧内容、
`terminal_*.c` 根本不存在、Keil 工程里也还指着旧文件名 —— 全仓编不过，而且新写的东西一行都没进去。
这是这类「先删后建」重构最容易踩的坑。

**修**：`git add -A`（确认 `.gitignore` 之后再 add，见 P3-2），`git status` 复核，
再 `git diff --cached --stat` 确认是「34 files / +1788 / −2027」这个量级，然后编译一次再提交。

### P0-5 CyberGear 解码用报文里的 `motor_id` 直接索引 6 元数组（越界写）

`application/robot_cmd/CAN_receive.c:312-326`

```c
if (((RxCanInfo_s *)(&rx_header->ExtId))->communication_type == 2)
{
    motor_id = ((RxCanInfoType_2_s *)(&rx_header->ExtId))->motor_id;   /* 8 位：0..255 */
}
if (CAN == &hcan1)
{
    memcpy(&CAN1_CYBERGEAR_MEASURE[motor_id].ext_id, &rx_header->ExtId, 4);
    memcpy(CAN1_CYBERGEAR_MEASURE[motor_id].rx_data, rx_data, 8);
}
```

`motor_id` 是 `uint32_t motor_id : 8`（`motor.h:110`），取值 0..255；
而数组是 `static CybergearMeasure_s CAN1_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1]`，
`CYBERGEAR_NUM = 5` → **只有 6 个元素**（`CAN_receive.c:43-44`），元素 16 字节（`motor.h:122-128`）。
`motor_id = 200` 时写的是 `base + 3200`，直接踩进后面的 `CAN2_CYBERGEAR_MEASURE` / `CAN*_DM_MEASURE` /
`CAN*_LK_MEASURE` / `SUP_CAP_MEASURE`。

触发条件低得可怕：**总线上任何一帧 29 位扩展帧，`communication_type == 2` 且 `motor_id > 5`**。
`can_filter_init()` 的掩码是 `0x0000`（全收），没有硬件过滤兜底；CAN 也没有认证。
这不是「配错参数才炸」，是「总线上有第三个设备/一个干扰帧就炸」。

这条是**本次改动之前就存在的历史缺陷**（`git diff` 里这两行上下文被改到了，逻辑未动），
但既然这次是「整体重构」，就一并修掉 —— 一行 `if (motor_id > CYBERGEAR_NUM) return;` 的事。

**修**：`communication_type == 2` 判断之后立刻 `if (motor_id < 1U || motor_id > CYBERGEAR_NUM) { return; }`；
顺手把 `can_filter_init` 从「全收」改成按 ID 段过滤（见 §2b 的说明）。

### P0-6 上位机解析把帧里的 `data_id`/`data_len` 当数组下标和长度用（越界写）

`application/communication/communication.c:295-304`

```c
uint8_t data_id  = frame_header[2];        /* 0..255 */
uint8_t data_len = frame_header[1];        /* 0..255 */
BOARD_RX_DATA[data_id][0] = data_len;
memcpy(&BOARD_RX_DATA[data_id][1], received + 5, data_len);
```

`BOARD_RX_DATA` 是 `communication.c:47` 定义的 **8 行 × 21 字节**（`DATA_NUM`/`DATA_LEN` 在 `communication.h:9-10`）：
`data_id = 200` 直接写到数组外（`200 × 21 = 4200` 字节之后），
`data_len = 200` 则把 200 字节塞进 20 字节的行尾。
而且这不是「配错参数」才触发的 —— 只要上位机（或任何能往 huart1 发数据的东西）
发一个 **27 字节、CRC8 与 CRC16 都正确的合法帧**就能写穿全局区，CRC 是标准算法，谁都能算。

和 P0-5 一样，这是**本次改动之前就有的缺陷**（`doc/custom_terminal_breakdown.md:492` 已经记过但没修），
而且它比 P0-5 更容易被触发（不需要接错设备，只要上位机软件有个 bug/被改过）。

**修**：写之前校验 `if (data_id >= DATA_NUM || data_len > DATA_LEN) { return; }`；
顺带把 `fifo_s_gets()` 的返回值也判一下（见 §4 第 7 条）。

---

## 2. P1 —— 正确性与安全

### P1-1 `zdatou_get_measure` / `robostride_get_measure` 返回 NULL 没检查

- `CAN_cmd_zdatou.c:248-254`：`addr==0 || addr>6 || can==0 || can>ZDATOU_CAN_NUM` → `return NULL;`
- `CAN_cmd_robostride.c`：`id<1 || id>ROBOSTRIDE_ID_NUM` → `return NULL;`
- 使用处 `custom_terminal.c:170-186`：

```c
raw = rs->pos;              // 无 NULL 判断
stamp = rs->last_fdb_time;
raw = (float)zd->pos_deg * ZDATOU_01DEG_TO_RAD;   // 无 NULL 判断
```

现在不炸，靠的是两条**没有写下来也没有断言**的隐式约定：`CUSTOM_TERMINAL_RS_JOINT == 1`，
且 `ZD_ADDR` 里唯一的 `0` 正好落在 index 1（见 `robot_param_terminal.h` 的 `ZD_ADDR {1,0,3,4,5,6}`）。
哪天把 RS 关节挪到别的下标、或者填错一个地址，就是 1 kHz 中断里的 HardFault（NULL 解引用），
现象是随机死机，很难往参数表上想。

**修**：取到指针后判空，失败按「该关节无反馈」处理并置 `joint_feedback_valid[i]=0`；
更好的是在 `CustomControllerInit` 里一次性校验参数表（RS 下标 ≠ 0 地址、地址 1..6 不重复、`MA_PER_NM` 非 0），
失败就进 `FAULT` 态不发任何力矩。

### P1-2 反馈超时没有「停止出力」的动作

`joint_feedback_valid[i]` 由 `custom_terminal.c:210-211` 算出（基于反馈帧时间戳，20 ms 超时，OK），
但它的全部消费点是：
1. `Console:308-310` —— 决定能不能「使能」；
2. `SendCmd:378-384` —— 只写 debug（见 P0-2）；
3. `Console:356-368` 重力分支 —— **完全不看**。

所以断电/拔线/驱动器掉线时：手动模式因为 `enable=false` 会以 0.02 N·m/ms 斜坡回零（50 ms，还算合理），
但只要 `armed==0`、`gravity_en==1`，重力分支就会接管并**继续对着一台没有反馈的电机加力矩**。
真机上表现为「机械臂掉线后自己飞出去」。

**修**：加一个明确的安全状态机（`INIT → IDLE → ARMED → FAULT`），
`FAULT` 由「任一关节反馈超时」触发，动作固定为：全部 `joint_torque=0` + 全部失能 + 需要人重新 `ARM`。
不要在多个分支里各判一次。

### P1-3 `joint_velocity` 不是速度

`custom_terminal.c:194-206`、`:210`

```c
const uint32_t dt_ms = now - custom_terminal.joint_last_feedback_time[i];
...
custom_terminal.joint_last_feedback_time[i] = now;     // ← 每拍都刷新
```

两个问题：
1. `joint_last_feedback_time[i]` 每拍都被写成 `now`，所以 `dt_ms` **恒等于任务周期**（1~2 tick），
   与「这一拍到底有没有新帧」无关 —— 后面 `dt_ms <= 20ms` 的判断永远成立，等于没有超时保护。
2. `delta = wrap(raw_mech - joint_raw[i])`，`joint_raw[i]` 也是每拍刷新；没新帧时 delta=0 → 速度=0；
   有新帧时 delta 是「这一拍量化跳变」。ZDT 单圈 0.01° 分辨率（`ZD_REPORT_PARAM 0x31`）→
   一跳 0.01° 除以 1 ms 就是 **0.175 rad/s 的假速度尖峰**。

所以这个量是「0 与尖峰交替」的脉冲串，任何基于它的 D 项/阻抗控制都会抖。
现在没人用它（控制律还没写），但它是留给下一阶段的接口，会直接把下一阶段带沟里。

**修**：优先用驱动器自己的速度（RS00 有 `vel`；ZDT 切 `0x36` 实时上报里有速度字段），
或者固定 20~50 ms 窗口、只用「帧序号变化」时的累计角度差算平均速度，并做一阶低通。

### P1-4 参数分层反了：终端参数挂在「全向轮哨兵」参数文件上

- `robot_param.h:30` include 的是 `robot_param_omni_mini.h`（文件头写的是「全向轮哨兵机器人参数配置文件」），
- 而终端开关写在这个文件第 47 行：`#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_TERMINAL`，
- 终端自己的参数表 `robot_param_terminal.h` 在 `robot_param.h:35` 被**无条件** include（所有机器人配置都会拉进来）。

后果：一条 C 板 + 6 自由度机械臂的配置，被塞进「哨兵」参数文件里；
其它机器人以后想用终端，得再改一遍 `robot_param.h`；而 `robot_param_terminal.h` 里的
`CUSTOM_TERMINAL_*` 宏在完全用不到的配置里也一直参与编译。

**修**：新建 `robot_param_terminal_arm.h`（或把 omni_mini 里终端相关段整段搬走），
`robot_param.h` 只 include 选中的那一个；`robot_param_terminal.h` 改到 `#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_TERMINAL)` 里 include。

顺带记一个**改对了**的地方：`robot_param_omni_mini.h` 里 `GIMBAL_TYPE CHASSIS_NONE` → `GIMBAL_NONE`，
原来是把底盘枚举填进云台枚举，虽然两者都是 0（`robot_typedef.h:6,13`）不影响行为，但确实是笔误，改得对。

### P1-5 本次改动把 33 个中文字符改坏了

`robot_param.h`（5 处）和 `robot_param_omni_mini.h`（28 处）在本次改动后被重存成了 UTF-8，
但注释里出现了 U+FFFD 替换字符，HEAD 版本是干净的（`git show HEAD:…` 里 0 处）。
例如：

```
- * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
+ * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数�?
-#define __MUSIC_ON 0               // 开启音乐
+#define __MUSIC_ON 0               // 开启音�?
-#define GIMBAL_YAW_LIMIT (0) // ... 1 - 有限位，0 - 无限位
+#define GIMBAL_YAW_LIMIT (0) // ... 1 - 有限位，0 - 无限�?
```

全是注释，不影响功能，但这是「用错编码的工具改文件」的典型后遗症（本仓 GBK / UTF-8 混排，
`robot_param_terminal.h` 至今还是 GBK，`read` 工具直接读不出来）。
**修**：把那两个文件 `git checkout --` 回来，只重做真正要改的那两行
（`GIMBAL_TYPE`、`CUSTOM_CONTROLLER_TYPE`），或者整仓统一转 UTF-8 后再落一次改动。

### P1-6 `huart6` 上的三方争用 + HAL gState 与非 HAL DMA 混用

上行 0x0302 同时走 `huart1` 和 `huart6`（`custom_terminal.c:436-438`），而 `huart6` 是**裁判系统串口**：

| 使用者 | 位置 | 方式 |
|---|---|---|
| 裁判系统接收 | `referee_usart_task.c:186-223` | 手写双缓冲 DMA + `HAL_UART_IRQHandler(&huart6)`（本次新加在 222 行） |
| 终端上行 | `custom_terminal.c:480` | `HAL_UART_Transmit_DMA`（靠上面那句 IRQHandler 清 TC、放 gState） |
| 双 yaw/pitch 云台变体 | `gimbal_double_yaw_pitch.c:301/594/694` | `HAL_UART_Receive_IT` / `HAL_UART_Transmit` |
| bsp 裸 DMA | `bsp_usart.c:174` `usart6_tx_dma_enable()` | 直接操作 `DMA2_Stream6`，**绕过 HAL 的 gState** |

`referee_usart_task.c:217-222` 那句 `HAL_UART_IRQHandler` 加得有道理（注释解释了
`__HAL_UART_CLEAR_PEFLAG` 是「读 SR 再读 DR」，HAL 先跑会把 IDLE 标志一起吃掉），
但它是**唯一**让 `HAL_UART_Transmit_DMA` 能把 gState 放回 READY 的路径 —— 一旦有人把这句删了，
上行会以 34 ms 周期全部 `BUSY_ABORT`（`custom_terminal.c` 的 `CUSTOM_TERMINAL_UPLINK_BUSY_ABORT_CNT 10` 兜底会把 DMA 掐掉），
表现为「0x0302 偶尔发一帧」。这种跨模块的隐式依赖应该写成注释+断言，而不是只靠一句注释活着。

另外 `gimbal_double_yaw_pitch.c` 若参与编译，会和上行抢同一个 handle。

**取证结论（已核 HAL 源码，这条是「必须保留」而非「应删掉」）**：本次一共新增了两句
`HAL_UART_IRQHandler` —— `communication.c:274`（huart1）与 `referee_usart_task.c:222`（huart6）。
查 `stm32f4xx_hal_uart.c:1998/2019/2131` 可确认：`HAL_UART_Transmit_DMA` 之后，
只有 TC 中断进 `UART_EndTransmit_IT` 才能把 `gState` 从 `BUSY_TX` 放回 `READY`；
不加这句，第二帧起 `gState != READY`，`custom_terminal.c:457-466` 的
`BUSY_ABORT` 计数到 10 就会把 DMA 掐掉，表现为「30 Hz 只发出第一帧」。
同时这两句**不会**干扰接收：HAL 的错误分支只在 `EIE/PEIE/RXNEIE` 置位时进入，
而 huart1/huart6 都没开这三位，接收是 `bsp_usart.c` 的手写双缓冲 DMA（`hdmarx->State` 从未置 `BUSY`）；
把调用放在 IDLE 分支之后也是对的，否则 HAL 先清 IDLE 就取不到长度。

所以这条的真正问题不是「代码错」，而是「**没有注释说明它是承重墙**」——
将来谁把它当冗余删掉，就会得到一个「单帧固件」。

**修**：① `bsp_usart.c` 的裸 DMA 版本和 HAL 版本不要混用，二选一；
② 上行只留一条链路（裁判系统口通常就够了），或明确 `huart1` 只做调试；
③ 在 `bsp_usart.c` 顶部写清楚「huart6 的 TX 归 HAL 管、RX 归 referee 手写 DMA 管」。

### P1-7 0x0302 帧格式本身是对的（记录一下结论，避免重复怀疑）

`custom_terminal.c:428-434` 的 `custom_terminal_uplink_frame_t` 与
`application/referee/protocol.h:52-58` 的 `frame_header_struct_t` 一致：
`SOF(1) + data_length(2) + seq(1) + CRC8(1) + cmd_id(2) + data(30) + CRC16(2)` = 39 字节，
且 `protocol.h:55` 注释明确 `data_length` 是「data 的长度」→ `30` 是对的（不是 32）。
唯一可改进的是**这个结构体是复制品**，`protocol.h` 里已经有同构定义（见 P2-3）。

### P1-8 `SendCmd` 的下标区间是硬编码的，换一个关节排布就会往扩展帧 ID 0 发力矩

`custom_terminal.c:386-423`：phase 0 固定发 `joint[0]`（ZDT）和 `joint[1]`（RS00），
phase 1 发 `[2,4)`，phase 2 发 `[4,6)`，且**没有任何 `ct_zd_addr[i] != 0` 的判断**。
`ZDATOU_ADDR_TO_EXT_ID(addr) = addr << 8`（`CAN_cmd_zdatou.h:40`），所以 `addr==0` → 扩展帧 ID 0。
今天能跑只因为「唯一的 0 恰好是 index 1，而 index 1 只走 robostride」。

同一段里还有：`clamp_torque(x, (float)ZD_TORQUE_MAX_MA / ct_ma_per_nm[i])`，
而 `CUSTOM_TERMINAL_JOINT_MA_PER_NM` 在 index 1 是 `0`。若 RS 关节不在 index 1，这里就是除零 → `inf`，
`clamp_torque` 拿到 `limit=inf` 直接返回原值，再乘 `0` → 静默输出 0 力矩（不报错、不动）。

**修**：循环里加 `if (ct_zd_addr[i] == 0U) continue;`，并把「哪些关节走 ZDT、哪个走 RS」做成
驱动类型数组（`enum {JT_ZDT, JT_RS00}`），而不是靠下标区间 + RS 宏。

### P1-9 `zdatou_on_rx()` 是个「通吃」判断，把 CyberGear 的反馈帧全抢走了（本次重构引入的回归）

`CAN_receive.c:306-310` 在 CyberGear 分支**之前**调用了 ZDT 解码，只要它返回 1 就 `return`：

```c
if (zdatou_on_rx((CAN == &hcan1) ? 1U : 2U, (uint8_t)(rx_header->ExtId >> 8),
                 rx_data, (uint8_t)rx_header->DLC))
{
    return;                      /* ← CyberGear 的帧走不到下面的解码 */
}
```

而 `CAN_cmd_zdatou.c:186-189, 234-236` 的准入条件只有一条：地址在 1..6 之间就认，其余一律
`default: return 1U;`（注释写「只记诊断」）。

CyberGear 的反馈帧把 `motor_id` 放在 ExtId 的 8..15 位（`RxCanInfoType_2_s`，`motor.h:107-120`），
正常配置下正好是 1..5。于是**每一帧 CyberGear 反馈都会被 ZDT 当作自己的帧吃掉**：
`zdatou_measure[can-1][id-1]` 被写进 CyberGear 的字节，`last_fdb_time` 被刷新（看起来「在线且新鲜」），
上层从 `rx_data[0]`（CyberGear 的位置高位）当 0.01° 角度解出来 —— 得到的是一个「有效但完全错误」的角度。

HEAD 版本只精确匹配 `SBZDATOU1..6_RX_EXT_ID` 这几个 ID，所以这是**改成通配后新增的回归**。

目前不发作，是因为没有任何 `robot_param_*.h` 配置了 `CYBERGEAR_MOTOR` 类型的电机 ——
但这两个驱动存在的意义就是「以后接 ZDT/CyberGear」，一接上就撞。

**修**（两条都要）：
1. `zdatou_on_rx` 加信封校验：报文功能码必须属于已知集合、最后一字节必须等于 `ZDATOU_CHECKSUM`(0x6B)、
   DLC 达到该功能码的最小长度（见 §2b-M6）；
2. 把 `zdatou_on_rx` 的调用**移到** CyberGear / `communication_type == 2` 分支之后，
   让「专用解码器」优先于「通配解码器」。

### P1-10 `CUSTOM_CONTROLLER_TYPE` 写在了 `BOARD_CURRENT` 块外面，上下两板都会启用终端

`application/typedef_robot/robot_param_omni_mini.h`

- 第 34 行就已经 `#endif` 结束了 `#if (BOARD_CURRENT == C_BOARD_DOWN)` 分支，
- 而第 47 行的 `#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_TERMINAL` 在它**外面**。

所以 `C_BOARD_UP`（上板，无机械臂）也会把 `CUSTOM_CONTROLLER_TYPE` 编成 TERMINAL，后果是三件事：

1. `Src/freertos.c:218-221` 会一并创建 `custom_controller_task`；
2. 这个任务 30 Hz 往 `huart6`（**裁判系统口**）发 0x0302 上行帧；
3. 它还会在 CAN1 上发 6 轴 ZDT/RS 力矩帧 —— 而上板 CAN1 上挂的是 `TRIGGER` / `FRIC_L` / `FRIC_U`
   （拨弹、摩擦轮），也就是说**上板会按机械臂的关节地址去给摩擦轮电机发帧**。

现在整机只有 C 板（下板）在跑，所以没炸；但只要有人把上板也烧一遍同源固件，
现象会非常难查（摩擦轮乱转 + 裁判系统口被塞了自定义帧）。

**修**：把第 46-47 行移进 `#if (BOARD_CURRENT == C_BOARD_DOWN)` 分支，上板显式给 `NONE`。

---

## 2b. CAN 驱动层专项（`CAN_cmd_zdatou` / `CAN_cmd_robostride` / `CAN_receive` / `bsp_can`）

这一层的**发送侧是干净的**（帧长/DLC/字节序/MIT 位打包全部逐位核对过，见 §5），
问题集中在**接收侧的「认领」逻辑**——也就是「这一帧到底是谁的」。

编号与上文连续，不再重复 P0-5 / P1-9。

### M1 两个 `*_on_rx` 都不校验 `can` 参数，`can == 0` 会往数组前面写 2 KB

`CAN_cmd_zdatou.c:205` `measure = &zdatou_measure[can - 1U][bucket];`
`CAN_cmd_robostride.c:231` `robostride_fdb_data(&robostride_measure[can - 1U][motor_id - 1U], rx_data);`

`can` 是 `uint8_t`，文档说取 1/2（`CAN_cmd_zdatou.h:164`），`can==0` 时 `can-1U` 回绕成 `0xFF` →
写成 `base + 0xFF * sizeof(row)`。同一文件里的 `zdatou_get_measure()`（`:250`）**是**校验的，
所以这是「入口严格、内部松懈」的不一致。现在两个调用点都传 `(CAN == &hcan1) ? 1U : 2U`，够不到；
但只要有人从新总线代码/测试工程调一次，就是 ISR 里的野写，静默无日志。

**修**：两个 `*_on_rx` 开头照 `*_get_measure` 的写法加上界判断。

### M2 `GetDTFdbData()` 拿到了 NULL 也不看，而且只填了 `pos` 和 `offline`

`CAN_receive.c:593` `GetDTFdbData(p_motor, zdatou_get_measure(...));`
`CAN_receive.c:505-513` 里 `DT_measure` 直接解引用。

比 P1-1 更糟的一点：`GetDTFdbData()` 只写 `fdb.pos` 和 `fdb.offline`，
`vel/curr/temp/state/ecd` **保持上一次的值**（对比 `GetLkFdbData` 482-485、`GetDmFdbData` 457-461 都写全了）。
后果是：ZDT 关节放进速度环 → `vel` 永远读到 0（或开机时的 0）而 `offline == false`，
看起来一切正常。0x36 帧本身不带速度，所以必须**显式**清零并写注释，而不是「不写」。

**修**：判空后 `offline = true; return;`；`vel` 显式赋 0 并注明「0x36 无速度字段」。

### M3 ZDT 的位置约定从 `[-π, π)` 改成了 `[0, 2π)`，与另外三种驱动不一致，`@note` 还是旧的

`CAN_receive.c:503` 注释仍写「映射到 [-PI, PI)」，`:511` 实际是 `pos_deg * ZDATOU_01DEG_TO_RAD` → `[0, 2π)`。
HEAD 用的是 `uint_to_float(DT_measure->ecd, -M_PI, M_PI, 16)`；LK 至今还是这么算（`:482`），
CyberGear 用 ±4π（`:280`）。

这条对自定义终端**不一定立刻发作**：`custom_terminal.c:189-190` 里 `ct_zero` 是标定值，
在同一个 `[0,2π)` 约定下标出来的零位自洽。但两点必须记：
① 同一台机器人上 ZDT 关节和 LK 关节的「0 rad」差了 180°，混用两种电机的机器人会出错；
② 任何按 `[-π,π)` 写的上层代码（wrap、限位、IK）跨零时会看到 2π 跳变。
**修**：要么回到 `uint_to_float(ecd, -M_PI, M_PI, 16)`，要么把 `@note` 和所有上层约定一起改成 `[0,2π)` 并写清楚。

### M4 0x36 的「带符号 32 位位置」被当成「符号+数值」解，符号可能被翻了两次，且 `<<24` 是 UB

`CAN_cmd_zdatou.c:220-231`

```c
int32_t pos = ((int32_t)(uint32_t)rx_data[2] << 24) | ...;   /* 226 */
measure->pos_deg = (rx_data[1] != 0U) ? -pos : pos;          /* 230 */
```

驱动自己的头文件说这个字段**已经是带符号数、直接用**：
`CAN_cmd_zdatou.h:49`「电机实时位置（带符号 32 位…」、`:116`「0x36 直接来自电机」。
而 `.c` 的注释（`:177`、`:220`）说是「符号 + 数值」。两者矛盾，而代码按后者写：
如果电机发的是补码，`rx_data[1]` 就是高位字节（负数为 `0xFF`），第 230 行又取反一次 →
**负的位置被读成正的**（180° 量级的静默翻转）。
另外 `(int32_t)(...) << 24` 溢出到符号位在 C99 是实现定义/UB，恰好是 `-O2` 下最容易被优化的那种。

**修**：先用 `uint32_t mag = (rx_data[2]<<24)|...` 拼无符号，再一次性转换；
到底要不要符号位，**去手册 5.5.13 确认**，然后只改一处（要么删翻转、要么改头文件描述）。

### M5 `robostride_on_rx` 只比较 StdId 低 8 位，`0x515`/`0x615`/`0x715` 全被当成反馈

`CAN_cmd_robostride.c:215` `if ((std_id & 0xFFU) != ROBOSTRIDE_MASTER_ID) return 0U;`

注释说掩码是因为「bit10~8 是模式类型」—— 那只对主机→电机方向成立；
这里解的是**电机回帧**，`CAN_cmd_robostride.h:46` 明确回帧的模式位是 0。
于是 `BOARD_DATA_ID = 1`（扩展/标准帧 `0x115` 之类）或板间通信帧 `0x515` 会**先**被这里吃掉，
当成 MIT 反馈解出一个假的电机 id/bucket，而真正的板间解码在 `CAN_receive.c:258-265` 永远轮不到。

**修**：`if (std_id != ROBOSTRIDE_MASTER_ID) return 0U;`（确实需要模式位就 `& 0x700U` 后要求为 0）。

### M6 接收帧完全不校验：`0x6B` 校验字节从没查过，DLC 门槛还比真实帧短

`CAN_cmd_zdatou.c:209-240`：`0x31` 只要求 `dlc >= 3`（真实帧 4 字节，含 `0x6B`），
`0x36` 只要求 `dlc >= 6`（真实帧 7 字节）。`ZDATOU_CHECKSUM`(0x6B) 发送时每帧都带，接收时**从不检查**。

ZDT 的固定校验字节是白送完整性检查，不用等于「线缆噪声翻转一位 → 一个错的角度被当成真值」，
而且 `last_fdb_time` 会跟着刷新，`offline` 一直 false —— 错误数据永远「新鲜」。

**修**：`dlc >= 4` / `dlc >= 7`，并且 `rx_data[dlc-1] == ZDATOU_CHECKSUM` 才提交 `ecd`/`pos_deg`；
不匹配时只记 `err_cnt`、不动 `measure`。

### M7 测量数组不是 `volatile`，任务侧读到的是「两代混合」的记录

`static zdatouMeasure_s zdatou_measure[ZDATOU_CAN_NUM][6];`（`CAN_cmd_zdatou.c:5-7`）
`static RobostrideMeasure_s robostride_measure[...][...];`（`CAN_cmd_robostride.c:7-9`）

调试结构体是 `volatile`，真正给控制用的测量数组反而不是。
M4 上 32 位对齐的单个 store/load 是原子的，所以**不会读到撕裂的半字**，
但会读到「`pos_deg` 是第 N 帧、`last_fdb_time` 是第 N+1 帧」这种跨代记录；
`-O2` 下编译器还可以把整个结构缓存在循环外。`GetDTFdbData()` 恰好就是这种两次读。

**修**：数组加 `volatile`，或每 bucket 加一个 `seq` 计数（写前写后各一次，读侧比对重试）。
这一条是最低优先级，但**改一行**的事。

### L1-L9（细节，可攒着一起清）

| # | 位置 | 问题 |
|---|---|---|
| L1 | `CAN_cmd_zdatou.c:195,206-207` | `last_packet`/`addr_extid` 永远显示 packet 0（函数没拿到完整 ExtId），Watch 里会「确认」一个错的 packet |
| L2 | `CAN_cmd_zdatou.c:196-203,209`、`CAN_cmd_robostride.c:223` | 在 DLC 检查之前就读 `rx_data[0]`、固定拷 8 字节 → 短帧读到未初始化栈内容 |
| L3 | `CAN_cmd_zdatou.c:240` | `ok_mask` 只置不清，与头文件「最近一帧」的描述不符；而 `custom_terminal.c:155` 正是用它决定要不要重发上报配置 → 电机后来坏了永远不会被重新配置 |
| L4 | `CAN_cmd_robostride.c:12` vs `:180` | 原型 `static`、定义非 `static`（C99 下能链，但属链接性矛盾） |
| L5 | `bsp_can.c:51-75,77-95,97-115` | `CAN_SendTxMessage()` 不检查 `DLC <= 8`；`can_send_std/ext` 不检查 ID 范围（超范围被硬件截断 → 发给**别的设备**且不报错） |
| L6 | `CAN_cmd_robostride.c:38-49` | 钳位函数抓不住 NaN（NaN 与任何数比较都为假）；`(int)NaN` 在 M4 上得 0 → 解成 `P_MIN = -12.57 rad`，即「NaN ⇒ 全速砸向负限位」。改成 `if (!(x >= x_min)) x = x_min;` |
| L7 | `bsp_can.c` 全文件 | 没有 bus-off 检测/恢复（无 `HAL_CAN_ErrorCallback`、无 ESR 检查），也不看 `CAN_SEND_OK/FAIL`。掉总线后所有运动指令静默丢弃，电机继续执行最后一条力矩 |
| L8 | `CAN_receive.c:362-373` | `i < 12` 判 11 元数组（历史遗留，`id == 11` 越界一个元素） |
| L9 | `CAN_receive.c:549,553` | CyberGear 访问器用 `p_motor->id` 直接索引 6 元数组，`id == 6` 越界（历史遗留） |

### 设计层

- **两个驱动是近乎克隆的**：同样 `[can][id]` 静态测量矩阵、同样 `volatile *_debug_t`、
  同样 `*_send → can_get_handle → can_send_*`、同样 `*_on_rx` 骨架。
  但两者的「认领契约」严格程度不同（ZDT 认 1..6 全体、RS 按掩码认），
  P1-9 和 M5 就是这么来的。抽一个 `motor_bus_slot_t` + 显式匹配器能把这一类问题一次性消掉。
- **配置在驱动和参数表里各存了一份**：`ROBOSTRIDE_MASTER_ID 0x15`（`CAN_cmd_robostride.h:46`）
  与 `CUSTOM_TERMINAL_RS_MASTER_ID (0x15U)`（`robot_param_terminal.h:36`）重复；
  ZDT 地址表同样重复（`ZD_ADDR {1,0,3,4,5,6}` vs 驱动里硬编码 1..6）。
  换个主控 ID 就得改驱动源码。
- **没有 `_Static_assert`**：`[6]` 与地址范围、`ROBOSTRIDE_ID_NUM` 与数组维度之间没有任何编译期校验。
  加两条 `_Static_assert` 就能把「枚举改了数组忘改」变成编译错误 —— 这正是 L8/L9 那一类问题的解。
- **API 形状不齐**：`zdatou_*` 有 `_torque_control` 和 `_torque_control_i32` 两个版本，`robostride_*` 只有一个；
  所有发送函数都返回 `uint8_t` 而**没有任何调用者看返回值**；
  `GetDTFdbData()` 是公开函数，而 `GetLkFdbData`/`GetDmFdbData` 是 `static`
  —— 这种不对称正是「能被传入 NULL」的根源。

### 这一层做对了的地方

- **发送侧逐位核对通过**：每个 builder 用精确长度的 `uint8_t cmd[N]` 并把同一个 `N` 传下去，
  所以 DLC 恒等于载荷；多字节字段一律 MSB 在前；`memset(&ctrl, 0, sizeof ctrl)` 覆盖 `tx_data` 的尾部。
  MIT 打包按 16+12+12+12+12=64 位逐位验证：pos[0..1]、vel[2]/[3]hi、kp[3]lo/[4]、kd[5]/[6]hi、t[6]lo/[7]，
  无重叠、无超 31 位移位。
- **数值上有个经典坑踩对了**：`zdatou_torque_control_i32` 对 `INT32_MIN` 的处理用了
  `(uint32_t)(-(torque + 1)) + 1U`，**没有**那个常见的 `-INT32_MIN` 溢出。
- **本次 diff 顺手修了两个真 bug**：`MF_9025` case 缺 `break` 会漏进 ZDT 分支；
  `CAN1_LK_MEASURE[id-1]` 在 `LK_NUM == 4` 的数组上用 `id = 5/6` 索引（`:585`、`:591`）。
- **去掉了旧的硬编码暂存**：`&CAN1_ROBOSTRIDE_MEASURE[5]` 这个「不管哪个电机都先写第 5 槽」的
  中间缓冲被删掉，RS 反馈现在落到正确的 bucket。
- **ISR 纪律好**：接收中断里只有 ≤8 字节拷贝 + 一个 switch，没有 HAL 延时、没有锁、没有打印；
  `rx_header`/`rx_data` 是每次调用的栈变量；发送路径原来那个「所有发送者共写一个全局 `a3`」
  的共享状态也删掉了。

---

## 3. P2 —— 结构与可维护性

1. **1 kHz 里算了两次正解。** `Observer:226` 调 `FK_SolveAll`（6 个关节位姿 + 每个位姿的四元数/欧拉角），
   `Reference:249` 又调 `terminal_gravity`，内部再重建一遍 6 连杆 DH 链（`terminal_gravity.c:57-129`）。
   合计约 12 次 DH 变换/ms，而实际上行只需要关节角（30 Hz）、重力补偿只需要质心位置。
   建议：DH 链**每拍算一次**、结果共享给 FK 与重力；姿态解算降到 30 Hz。
   另外 `terminal_gravity` 局部数组 `T[6][4][4]`(384B) + `pc[6][3]` + 若干 4×4 临时矩阵约 600 B，
   跑在 `osThreadDef(customControllerTask, ..., osPriorityHigh, 0, 512)`（2 KB 栈，`Src/freertos.c:218-220`）里，
   `custom_controller_high_water` 有现成的水位可看（`custom_controller_task.c:78`），上机务必量一下。

2. **「每 1 ms 发 2 帧、3 拍转一圈」是魔法数**，`ct_send_phase` 的注释写死了 6 关节/2 帧。
   关节数一变（`CUSTOM_TERMINAL_JOINT_NUM`）逻辑就静默错位。改成
   `for (k = 0; k < FRAMES_PER_TICK; k++) { i = (ct_cursor + k) % N; send(i); } ct_cursor = (ct_cursor + FRAMES_PER_TICK) % N;`。
   顺带算一下总线负载：2 帧/ms = 2000 帧/s，扩展帧 8 字节约 128 bit → **约 26% @1 Mbps**，
   加上反馈帧（5 台 × 100 Hz）约 32%。能用，但**没有给总线错误帧/重传留余量**，值得记一笔。

3. **协议帧结构体复制了一份**（`custom_terminal.c:428-434` vs `referee/protocol.h:52-58`）。
   `referee.h` 里已经有 `frame_header_struct_t` 和 CRC 工具（`append_CRC16_check_sum` 就是复用的），
   帧头不该再抄一遍。

4. **`CustomControllerInit` 是「一次性动作 + memset」**，没有状态机：
   `memset` 掉整个 `custom_terminal`（含 `joint_last_feedback_time`），然后发失能 + 设上报。
   `ct_enabled_joint` 用 `0xFF` 当「没使能任何关节」的哨兵值（`Console:313-326`），
   这属于「用魔法值表达状态」。结合 P1-2 一起重构成显式状态机最划算。

5. **文档与实现不一致**：
   - `terminal_kinematics.h:67` 写 `T = R(axis,θ)·Rx(α)·Trans(a,0,d)`，
     而 `terminal_kinematics.c` 的实际乘法次序是 `Rj · Trans · Rx`。当前 `alpha` 全 0 所以等价，
     一旦有人填真实 `alpha` 就会错。
   - `arm_dh_param` 的 `alpha`、`d` 全 0（`terminal_kinematics.c:32-39`），
     这是个**近似模型**：只能求「末端大概在哪」，不能拿来做 IK。
     `doc/calibration_and_validation.md:16,199` 已经点明了，但代码里没有
     `#warning`/注释阻止后来者直接调它做逆解。建议在 `FK_SolveAll` 上方写死一句「此表仅用于姿态观测，不可用于 IK」。

6. **`GRAVITY` 定义在 `application/typedef/struct_typedef.h:23`** 这种通用头里，
   单位只靠行尾注释。`terminal_gravity.c:109` 的注释还得自己声明「本文件按 mm/s² 用」——
   这就是 P0-3 那个 1000 倍错误的温床。建议改成 `#define GRAVITY_ACCEL_MPS2 9.8f`，并在所有涉及长度的模块里统一 mm/m。

7. **`CAN_SendTxMessage` 等的是「3 个邮箱全空」**（`bsp_can.c:62-66`：`while (free_TxMailbox < 3U && cnt-- > 0U)`）。
   发一帧只需要 1 个邮箱，这个条件会让发送不必要地失败（或空转 ~20 次，`CAN_TX_MAILBOX_RETRY_CNT`）。
   而且 `CAN_SEND_OK=0 / CAN_SEND_FAIL=1`（`bsp_can.h:24-25`）与「0 成功」的直觉相反，
   `zdatou_send` 的返回值在 `custom_terminal.c` 里也没人看 —— 丢帧完全静默。

8. **`BoardCommunicateData_s` 的字段布局和真正的解析器不一致**（`communication.h:16-28`）。
   结构体里 byte1-2 是 `uint16 data_length`、byte3 是 `seq`；
   而 `communication.c:295-304` 的 `DataUnpack` 把 byte1 当 `len`、byte2 当 `id`、byte3 当 `type`。
   目前这个结构体只被 `sizeof` 用到（算出来 27 恰好等于真实帧长），所以还没露馅；
   一旦有人拿它去打包一帧，就是**静默错帧**。
   建议按解析器重新定义（`uint8_t sof, len, id, type, crc8;`），或者干脆删掉、CRC 长度直接写 `FRAME_HEADER_LEN + DATA_LEN + 2`。

9. **跨任务共享的 `can_buffers` 没有临界区**（`communication.c:40` 定义，`:158-235` 读写）。
   1 ms 的电机任务往里写、100 ms 的 `communication_task` 读完之后 `CanManagerClearBuffer()` 清零。
   一次读-清之间写进来的命令会被吞掉（下个周期会重发，所以影响有限），
   但这是「偶发丢一帧控制指令」这种最难复现的 bug 的典型来源。
   建议用 `taskENTER_CRITICAL()` 把「读 + 清」包成一个原子块，或改成双缓冲。

---

## 4. P3 —— 细节

1. **13 条编译告警**（`_check/uv4-build.log`）：12 条是 `#1-D: last line of file ends without a newline`
   （`referee.h:624`、`referee.c:428`、`water_shot.c:113`、`npid.h:28`、`matrix.h:82`、`rls.h/c`、
   `matrix.c`、`chassis_power_control.c:43`），1 条 `#1295-D: Deprecated declaration remoto_taks`
   （`application/other/remoto.h(1)` 的 `extern void remoto_taks();` 没写参数表）。
   都不影响功能，但「0 warning」是能让下次真正的告警被看见的最低成本，建议顺手清掉。

2. **`.vscode/settings.json` 里塞了一张 base64 壁纸**（本次改动 +15 KB 左右的一行巨长字符串）。
   编辑器个人化配置不该进仓库，更不该进一次「重构」的 diff。

3. **`.gitignore` / 缺 `.gitattributes`**：
   - `*(*)*`（写着 "# Duplicate files"）会匹配任何路径里带括号的文件/目录，含义不明且容易误伤；
   - 未忽略 `.workbuddy-ai/`（AI 工具目录，现在整个目录 untracked）和 `_check/`
     （本地构建日志/脚本，如 `_check/set-bg.ps1`），下次 `git add -A` 会一起进去。
     应显式 `git add` 白名单，或补 ignore；
   - 仓库没有 `.gitattributes`，于是 `git diff` 一直在喊
     `warning: in the working copy of '…', LF will be replaced by CRLF the next time Git touches it`
     （本次改动几乎所有 `.c/.h` 都触发）。这意味着换行的规范化没有定论：
     只要有人用不同工具保存一次，就会出现整文件级别的假 diff，把真实改动埋掉。
     建议加 `.gitattributes`（`* text=auto`，或对 `.c/.h` 明确 `eol=crlf`）。

4. **编码不统一**：`robot_param_terminal.h` 是 GBK（`read` 工具直接报 `invalid UTF-8`），
   `custom_terminal.c` / `terminal_*.c` 是 UTF-8，`bsp_led.*` / `oledfont.h` 也是 GBK。
   跨编码改同一文件必然产生 P1-5 那种损坏。建议整仓一次性转 UTF-8 + 加 `.gitattributes`。

5. **`terminal_wrap_angle` 把关节角折到 ±π**（`custom_terminal.c:192`）。
   ZDT 单圈本来就 ±π 无妨，但 RS00 原生给 ±4π 多圈（见 `doc/review_zdatou_vs_robostride.md`），
   在这一行被折掉了。有减速比的关节失去多圈信息后，「关节转过 1 圈」和「没转」在固件里无法区分。
   现在 `JOINT_GEAR` 全 1 所以看不出来，接上减速器就是坑。

6. **`ct_ma_per_nm` 的标定依据不明**：`robot_param_terminal.h` 注释写「5000 mA / 6.25 N·m」，
   即 ZDT 相电流满量程 5000 mA 对应 6.25 N·m。这个 6.25 是从手册推的还是实测的？
   `doc/calibration_and_validation.md` 的 S5 也在问同一件事。**没有实测之前，所有力矩数值都只是「按比例」的**，
   建议在注释里标 `TODO: 待实测`，避免下一阶段把 mA 当 N·m 用。

7. **`fifo_s_gets()` 的返回值没判**（`communication.c:299`，`fifo_s_gets(..., DATA_LEN + 2)`）。
   噪声里出现一个 `0xA5` 就会让解析器进入「等 22 字节」的状态，把后面的真帧头一起吞掉。
   配合 P0-6 一起改：按实际返回长度决定是否继续解析。

8. **`FRAME_HEADER_LEN` 一个宏担了两个含义**（`communication.c:8`）：
   既当「帧头长度」又当「CRC8 覆盖长度」，只在 5 字节头（sof+2 字节 length+seq+crc8）时成立。
   一旦把帧头改成 4 字节，CRC8 覆盖范围会跟着错。建议拆成 `HEADER_LEN` / `CRC8_LEN`。

---

## 5. 做得好的地方（保持）

- 分层清楚：`CAN_cmd_zdatou.c` / `CAN_cmd_robostride.c` 只管协议，`custom_terminal.c` 只管关节量换算与控制量，
  `terminal_kinematics.c` 只做纯数学（可主机侧编译测试），`terminal_debug.c` 只放观测变量。
  比原来的 `controler_myself.c`（690 行，把 CAN 报文、单位换算、正解、调试全揉在一起）好维护得多。
- `CustomControllerInit` 上电**显式发失能**（`custom_terminal.c:104-111`，注释引用手册 5.3.2），
  而不是只发 0 力矩 —— 这是踩过坑才会写的代码。
- 上报配置的**自愈重发**（`custom_terminal.c:148-163`）：每秒挑一台还没上报的补发一次，注释解释了为什么
  （驱动器上电慢/仲裁失败会永久丢配置）。这个思路很好。
- `terminal_dbg` 放在所有变体守卫之外定义（`terminal_debug.c`），保证任何配置下符号都存在，
  不会因为「换配置 Watch 就看不到变量」浪费时间。
- `_check/` 里有可复现的 Keil 命令行构建日志，`doc/calibration_and_validation.md` 有 S0-S5 + G0-G4 分步标定，
  文档先行的习惯很好。
- CAN 发送侧逐位核对通过：帧长恒等于 DLC、多字节 MSB 在前、MIT 打包 16+12+12+12+12 无重叠，
  `zdatou_torque_control_i32` 对 `INT32_MIN` 的处理避开了经典溢出坑（详见 §2b 末尾）。
- 本次 diff 顺手修掉了两个真 bug：`MF_9025` 分支缺 `break`、`CAN1_LK_MEASURE` 在 4 元数组上用 5/6 索引。

---

## 6. 建议的修复顺序

**第 1 轮（一小时，全部是 P0）**
0. `CAN_receive.c:312` 后面加 `motor_id` 上界判断（P0-5，一行）；
0b. `communication.c:295` 前加 `data_id/data_len` 上界判断（P0-6，一行）；
1. 删 `custom_terminal.c:484-495` 的 `#else` 空实现桩（保留 500-503 的真实现）；（P0-1）
2. `SendCmd` 用门控后的 `tau[]` 发送；（P0-2）
3. `terminal_gravity.c` 单位修掉 + `Console:363` 加负号；（P0-3）
4. `git add -A` + 编译 + 复核 diff；（P0-4）

**第 2 轮（半天，安全语义）**
5. `CustomControllerInit` 里做参数表校验 + NULL 判断；（P1-1）
6. 加 `INIT/IDLE/ARMED/FAULT` 状态机，反馈超时 → 全零力矩 + 全失能；（P1-2）
7. 修 `joint_velocity`（或先标 `TODO: 未实现` 免得被误用）；（P1-3）
8. `zdatou_on_rx` 加信封校验 + 把调用移到 CyberGear 分支之后；（P1-9）
9. `robot_param_omni_mini.h` 的 `CUSTOM_CONTROLLER_TYPE` 移进 `C_BOARD_DOWN` 分支；（P1-10）

**第 3 轮（重构前）**
10. 参数分层：终端单独一个 `robot_param_*.h`；（P1-4）
11. 编码统一（`git checkout --` 修回被破坏的两个文件，其余单开一次全仓转码）；（P1-5）
12. `SendCmd` 改成驱动类型表 + 跳过 `addr==0`；（P1-8）
13. `*_on_rx` 的 `can` 校验（M1）、`GetDTFdbData` 判空 + `vel` 显式清零（M2）、
    RS 回帧改为精确匹配 StdId（M5）、ZDT 校验字节 + DLC 门槛（M6）；
14. `can_buffers` 临界区、`fifo_s_gets` 返回值、`BoardCommunicateData_s` 布局（P2-8/9、P3-7）；
15. 清 13 条告警、`.vscode` 壁纸、`.gitignore`/`.gitattributes`。（P3-1~3）

**第 4 轮（功能）**
16. 位置约定统一（M3）与 0x36 符号问题（M4）—— 这两条要在标定 S2/S3 之前定下来，否则零位白标；
17. 正解/重力共用一次 DH 链，姿态降到 30 Hz；（P2-1）
18. 按 `calibration_and_validation.md` 做 S0-S5 标定 → 再谈控制律（阻抗/重力前馈）。

**验证基线**：改完每轮都跑一次
`UV4.exe -b MDK-ARM\StandardRobotPro.uvprojx -j0 -o _check\uv4-build.log -t StandardRobotPro`，
目标是 `0 Error(s), 0 Warning(s)`。

---

## 7. 实施记录（2026-10-26 / 27）

### 7.1 已提交

| 提交 | 内容 |
| --- | --- |
| `d73f81b` 重构清理环境 | S1 基线。把"只暂存了删除和改名"的索引补成完整快照（P0-4 解决）；`.gitignore` 增加 `.workbuddy-ai/`、`_check/` |
| `05d6ff1` fix(P0-1) | 取消 `CUSTOM_CONTROLLER_TYPE` 变体守卫，删除重复的 `custom_terminal_get_joint_feedback` 定义 |
| `64c51dd` fix(P0-6) | `DataUnpack()` 校验 `data_id`/`data_len` 边界，避免写穿 `BOARD_RX_DATA` |

**P0-1 的修法要更正**：本文 §1 原先写"删 500-503"，那是错的——`#else` 分支在 TERMINAL 配置下根本不编译，
真正在用的是 `#endif` 之后那一份，删掉它会 L6218E（`custom_controller_task.c` 的 `__weak` 桩里没有这个函数）。
正确做法是删 `#else` 的桩、保留实现，并把整个变体守卫一起去掉（本工程只有一种配置）。

### 7.2 结论修订

- **M4 是误报**。`0x36` 的"位置"字段是「符号 + 4 字节无符号幅值」（`_check\zdt_manual.txt:3019-3020`），
  代码先拼幅值、再按符号取负，是对的，不存在"符号应用两次"。
- **P0-3（重力 `1000×` 偏大 + 符号）推迟到标定之后**。当前 `CUSTOM_TERMINAL_GRAVITY_SCALE = 0.0f`，
  重力支路不参与控制；`LINK_MASS`/`LINK_COM` 全是"待实测"占位值，现在改也验证不了。
- **P0-5 / P1-9 降级为护栏**：本工程 CAN1 上只有 5 台 ZDT + 1 台 RS00，没有 CyberGear；
  但越界写是确定的 UB，几行判断而已，保留修复。
- **P1-10（上板也会编进 TERMINAL 分支）不在范围内**：只烧下板，上板不刷。
- **P1-4（机器人参数分层反了）取消**：单一配置，改为一行注释说明即可。

### 7.3 硬件实测（下板 + 6 台张大头 X 固件 + RS00）

- 关节 ↔ 地址：`joint0→地址1`、`joint1→RS00`、`joint2→地址3`、`joint3→地址4`、
  `joint4→地址5`、`joint5→地址6`（`ct_zd_addr = {1,0,3,4,5,6}`，`RS_JOINT 1`、`RS_ID 5`）。
- 六台全部 **X 固件**、`CAN_Baud = 1M`、`Checksum = 0x6B`。
  本机总线速率：`Src\can.c:35-39`（`Prescaler 3`、`BS1 10TQ`、`BS2 3TQ`，APB1 42 MHz）→ 42 M/(3×14) = **1 Mbps**。
- **`0x31` 的回复实测 DLC = 3**（`31 高 低`，没有 `6B` 尾字节）。手册 5.5.7 表里的"5 字节"是含地址字节的，
  CAN 数据段只有 3 字节；有的固件带尾字节（DLC = 4）。解析必须两种都收，否则会把正常帧全部误杀。
- 自愈循环去掉 `break` 的修复得证：`zdatou_dbg.ok_mask` 从 `0x21` 变成 `0x29`（地址 1/4/6 全通）。
- 地址 3 那台曾经一个字节都不发（`addr_cnt[2]` 恒 0，连命令确认帧都没有），
  最终定位为**硬件问题**，已解决；现在六台电机全部在线。

### 7.4 标定前提（写进结论，避免以后踩坑）

- **力矩模式只有 X 固件有**（`_check\zdt_manual.txt:1280-1281`）。整套控制律是力矩型的，
  所以臂上每一台都必须是 X 固件；换电机时要先确认这一条。
- `0x33`/`0x36` 的位置单位随固件不同（Emm：`0-65535` = 一圈；X：值/10 即 0.1°），
  而 **`0x31` 两种固件口径一致**（0-65535 = 0-360° 单圈绝对）。因此只要还用 `0x31`，混装固件也不影响读数；
  但**若为多圈改看 `0x36`，就必须逐台知道固件**，否则 Emm 电机的读数会被放大成几十弧度。
