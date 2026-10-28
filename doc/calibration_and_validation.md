# 自定义终端（6 自由度机械臂）标定与实机验证流程

> 适用对象：`application/custom_controller/custom_terminal.c` + `CAN_cmd_zdatou.c` + `CAN_cmd_robostride.c`
> 前提：`robot_param_omni_mini.h` 里 `CUSTOM_CONTROLLER_TYPE = CUSTOM_CONTROLLER_TERMINAL`、`GIMBAL_TYPE = GIMBAL_NONE`
> 本文按"先读通、再动一个关节、最后才联动"的顺序组织，每一级都有判据和失败处置，**不要跳级**。

---

## 0. 现状：这 6 条决定了标定顺序

| # | 事实 | 对流程的影响 |
|---|---|---|
| 1 | 力矩入口已有：`terminal_dbg.manual_en/manual_joint/manual_torque_nm` 驱动单关节（见 §7） | 直接做 S2~S5，无需再补代码 |
| 2 | 反馈靠"定时上报"：开机给 5 台发一次 `11 18 0x31 00 0A 6B`，电机每 10 ms 主动发；每秒对没上报的地址自愈补发 | **S1 仍然第一个做**：先确认 `zdatou_dbg.ok_mask` 与 `fdb_cnt` 真的在动 |
| 3 | 角度 = `wrap(dir·raw - ZERO) / GEAR`，`ZERO`/`DIR`/`GEAR` 全在 `robot_param_terminal.h`，默认值都是占位 | S2/S3/S4 每个关节都要实测填表 |
| 4 | DH 表已按 URDF 推出来并落盘（`a/d/alpha/offset` 全部有值，`base_transform` 也补了），推导与复核见 `doc/dh_from_urdf.md`。**这是经典（distal）DH，不是 Craig 的 MDH**：`terminal_dh_transform` 的乘积是 `Rz·Tz·Tx·Rx`，表按它推（推导对拍 1e-15，落盘的四舍五入值 3.87e-3 mm）；MDH 是另一套乘积 + 另一张表，混用会错到米级，见 `dh_from_urdf.md` §1/§9 | FK 现在是真 6 轴正解；末端还要补 86.19 mm 的法兰偏置（§`dh_from_urdf.md` 3.3）。**`ZERO` 必须按 URDF 零位标**（大臂/小臂水平、肘部折满） |
| 5 | 上行 30 Hz 走 huart1(4pin)+huart6(3pin)（DMA），`pos[0..5]` = 关节角(rad)，`pos[6]` = 0xA7 占位 | 上位机解析要跳过 `pos[6]` |
| 6 | CAN1/CAN2 = 1 Mbps；ZDT 用扩展帧 `(Addr<<8)\|Packet`；RS00 用标准帧、电机 id = `CUSTOM_TERMINAL_RS_ID`；诊断在 `zdatou_dbg`/`robostride_dbg` | 没 CAN 分析仪也能靠 §8 的表定位 |

---

## 1. 工具

- USB-CAN 分析仪（1 Mbps，能看扩展帧与标准帧）
- 串口工具（115200 8N1）接 4pin huart1
- 数字角度尺 / 倾角仪（0.1°）、卷尺 + 卡尺
- 砝码或弹簧秤（力矩标定用）
- **支架或吊带**（S5 上力矩前必须先把机械臂托住）

---

## 2. 分级验证

### S0 静态链路自检（不使能电机、不接动力）

| 步骤 | 操作 | 判据 |
|---|---|---|
| S0.1 | 只给控制板上电，PC 接 4pin 口 | 能收到 39 字节帧，`sof=0xA5`、`cmd_id=0x0302`、`data_length=30` |
| S0.2 | 解析 CRC8/CRC16 | 全部校验通过，错帧率 0 |
| S0.3 | 看 `seq` 与帧间隔 | `seq` 连续递增，间隔 34 ms ± 1 ms（30 Hz） |
| S0.4 | 看 `pos[0..5]` | 静止时只在末位抖动（< 1e-3） |

**失败处置**
- 完全没帧 → `custom_controller_task()` 里那个 `tick - lastTick >= 34` 的条件，或 `CUSTOM_CONTROLLER_TYPE` 没生效（`freertos.c` 里没建任务）。
- 有帧但 CRC 错 → `custom_terminal_uplink_frame_t` 的 `__packed` 少了，或字段顺序错。
- `seq` 成片跳号 → `gState` 长期不是 READY，即 TC 中断没回来 → 查 `USART1_IRQHandler` 有没有真正进到 `HAL_UART_IRQHandler`。

---

### S1 反馈持续性（**关键前置，先做这个**）

**操作**：手转任意一个关节约 30°，盯上位机的 `pos[i]`；保持手不动，再观察 2 s。

| 现象 | 结论 | 处置 |
|---|---|---|
| `pos[i]` 随手变化 | 电机主动上报，读链路可用 | 继续 S2 |
| `pos[i]` 一动不动 | 只有开机那 6 次单次读取，**角度是冻结的** | 必须加周期轮询（见 §5-1），否则后面全部无意义 |
| 变化但跳变/回绕 | 多圈被 ±π wrap 吃掉 | 记录在案，S3 处理 |

**为什么必须第一个做**：`CustomControllerObserver` 里的速度是用 `ecd` 差分算的。若 `ecd` 冻结，速度恒 0、FK 恒不动，后面所有标定都建立在假数据上。

---

### S2 关节 ↔ 电机映射与符号（`JOINT_DIR`）

**不需要上力矩**：失能状态下手转就行（比给力矩安全得多）。

**操作**：逐个关节，手转"该关节的 + 方向"约 30°，看 `terminal_dbg.joint_fb[i]` 是变大还是变小。
"+ 方向"= 绕 base 系下该关节转轴、按右手定则。零位姿态下手转时末端（法兰）会往下表方向走，
这就是最快判断"我转的是不是 + 方向"的办法：

| 关节 | 数组下标 | 驱动 | `+q` 时末端往哪走（URDF 零位） |
|---|---|---|---|
| J1 | 0 | zdatou `0x100` | 向 +y 平移（俯视逆时针） |
| J2 | 1 | **robostride `0x15`** | 向下 |
| J3 | 2 | zdatou `0x300` | 向下（小臂往下折） |
| J4 | 3 | zdatou `0x400` | 绕大臂自身轴自转（末端只走 5 mm，看臂上的标记） |
| J5 | 4 | zdatou `0x500` | 向上 |
| J6 | 5 | zdatou `0x600` | 绕法兰轴自转（末端不动，看法兰盘标记） |

**判据**：按上表方向转，`joint_fb[i]` 应**增大**。

**失败处置**：`CUSTOM_TERMINAL_JOINT_DIR[i]` 取 `-1`（`robot_param_terminal.h:43`）。
`DIR` 必须与 URDF 的关节正方向一致，因为 `arm_dh_param[].offset` 是按 URDF 正方向推的：
填反了正解会把机构镜像，重力补偿会**反向加速**。

---

### S3 减速比（`JOINT_GEAR`）

代码里 `angle = (dir·raw − ZERO) / GEAR`，`raw` 是电机侧机械角（rad），所以

```
GEAR = 电机转过的角度 / 关节转过的角度      （直驱 = 1）
```

**前置（关键）**：把 `CUSTOM_TERMINAL_ZD_REPORT_PARAM`（`robot_param_terminal.h:65`）从
`0x31` 改成 `0x36`，重烧。

- `0x31` = 单圈编码器值（0~360° 回绕）。只要 `GEAR > 1`，关节转过 `2π/GEAR` 就回绕一次，
  `joint_fb` 会跳 360°，**根本没法标**。
- `0x36` = 实时位置（带符号 32 位、0.1°、可多圈），driver 已支持（`zdatou_dbg.addr_func[i]` 会变 `0x36`）。

**操作**：在关节输出端贴标记，手转一个整数圈（量程不够就 180°），角度尺量 Δq，读 `joint_raw[i]` 的变化 Δraw：

```
GEAR[i] = Δraw / Δq
```

`joint_raw` 已经乘过 `DIR`，所以 **DIR 必须先定**。

**判据**：填完 GEAR 后手转 10°，`joint_fb[i]` 变化 10°（±0.5°）；转 90° 得 1.571 ± 0.005。

**常见值**：直驱 1；带行星/谐波减速器就是减速比本身（查电机铭牌，别猜）。

---

### S4 零位（`JOINT_ZERO`）

**操作**：把六个关节一起摆到 **URDF 零位**（见本节末尾），然后走 §9 的 `zero_capture` 流程，
把 `zero_readback[i]` 填进 `CUSTOM_TERMINAL_JOINT_ZERO`（`robot_param_terminal.h:40`）。

**计算**：`angle = (dir·raw − ZERO) / GEAR`，要让零位处 `angle = 0`，则

```
JOINT_ZERO[i] = joint_raw[i]（零位处读到的、已乘过 DIR 的原始角）
```

**判据**：
- 零位处 `joint_fb[i] = 0.000 ± 0.002 rad`
- 转到 J3 = +90° 处 `joint_fb[2] = 1.571 ± 0.005 rad`
- 六个关节全部满足

**注意**：写进 `arm_dh_param[]` 的 offset 必须是 `M_PI`、`-M_PI_2`、`-0.62f` 这种**有物理含义的值**，不要留 `-3.14` 这种截断。`-3.14` 与 `-M_PI` 差 0.0016 rad ≈ 0.09°，在 300 mm 臂展上是 0.5 mm 的末端误差，会污染后面的 FK 判据。

**零位摆在哪**：`arm_dh_param[].offset` 是把"关节零位"钉在 **URDF/CAD 装配零位**上的（见
`doc/dh_from_urdf.md` §6），所以这里必须把臂摆成那个姿态——**整条臂水平躺平、肘部折满**
（大臂沿 -x 水平约 100 mm，小臂沿 +x 折回，所有质心高度 z ≈ 112 mm）。摆错姿态等于给六个
`offset` 各加一个未知量，正解和重力补偿会一起系统性偏。

---

### S5 力矩通道标定（**危险，必须先托住机械臂**）

前置：补一个力矩写入点（见 §5-2）。当前 `datouctrl[]` 无人写入，发出去的全是 0。

| 步骤 | 操作 | 判据 |
|---|---|---|
| S5.1 | 支架托住，只留 1 个关节悬空 | — |
| S5.2 | 发 +500 / +1000 / +2000 三个幅值 | 转向与定义一致；幅值越大保持力越大 |
| S5.3 | 末端挂已知砝码，逐级加 raw 直到能静止保持 | 记录 `raw / N·m` 比例 |
| S5.4 | 连续给 2000 跑 30 s | 电机温度上升 < 10 ℃，无异响 |

**得到的两张表**（后面写进参数头文件）：
- `raw/N·m`（每关节一个，v1.2 用的是 `TAU_MAP_PARAM1..6`）
- 每关节力矩上限（v1.2 是 2500/2500/2000/2000/1000，J6 最弱）

**符号验证**：`zdatou_torque_control_i32()` 里正 raw 对应 `ZDATOU_DIR_POSITIVE`。S2 得到的关节方向符号必须与它一致，否则重力补偿会**反向加速**下坠。

**注意**：`zdatou_torque_control` 只发低 16 位、且内部截断到 `ZDATOU_TORQUE_RAW_MAX = 8000`（这个 8000 是从被注释的 `a1[3] = datouctrl[2]/8000` 反推的，**未经手册确认**）。S5.3 顺带验证：发 8000 与发 9000 现象是否相同（相同则确认截断点）。

---

### S6 FK 验证与 DH 标定

| 步骤 | 操作 | 判据 |
|---|---|---|
| S6.1 | DH 已由 URDF 推出（不再手量）：按 `doc/dh_from_urdf.md` §8 抽查 2~3 个长度（J1→J2 竖直 47.00 mm、J2/J3 轴间距 99.97 mm） | 误差 < 0.5 mm，否则按实测改 `.a/.d` |
| S6.2 | 取 5 个不同姿态：角度尺读 6 个关节角 | 与上位机 `pos[]` 偏差 < 0.5° |
| S6.3 | 同姿态下卷尺量末端坐标，对比 DH 正解（零位姿态的期望值见 `doc/dh_from_urdf.md` §6 的表） | 位置误差 < 5 mm |

**失败处置**
- 误差随姿态系统性变化（有固定偏移）→ 改 `a` / `d`
- 误差与某些关节角正相关 → 该关节零位（S4）没标好
- 误差符号在跨越某关节 180° 时翻转 → 多圈/wrap 问题（S3）

---

### S7 联动前的安全验证（缺一不可）

| 项 | 现状 | 要求 |
|---|---|---|
| 反馈无效时停力矩 | 已实现：`CustomControllerConsole` 里 `valid[joint]` 为假就不出力 | 保持 |
| 通信超时 | 已实现：`now - stamp > CUSTOM_TERMINAL_FEEDBACK_TIMEOUT_MS`（20 ms）即 `valid = false` | 保持 |
| 力矩变化率 | 已实现：`step_toward`，ZDT 走 `MA_TEST_NM_RAMP`，RS 走 `CUSTOM_TERMINAL_TEST_NM_RAMP` | v1.2 是 5~10 N·m/s，可按需改 |
| 急停 | **暂无**（遥控输入已按需求删除） | 需要时接物理断电或后续加按钮；软件侧把 `manual_en` 写 0 可立刻回零失能 |
| 初始上电 | 力矩 0，且 ZDT 显式失能 | 保持；禁止上电即发力矩 |

---

## 3. 标定结果表（填完这张表，参数才算定）

| 关节 | 驱动 | CAN 地址 | `DIR` | `GEAR` | `ZERO`(rad) | `MA_PER_NM` | 力矩上限 | 软限位 min/max |
|---|---|---|---|---|---|---|---|---|
| J0 | zdatou | 1 | | | | 800 | 6.25 N·m | |
| J1 | robostride | 电机 id 0x05 | | | | — | 14 N·m | |
| J2 | zdatou | 3 | | | | 800 | 6.25 N·m | |
| J3 | zdatou | 4 | | | | 800 | 6.25 N·m | |
| J4 | zdatou | 5 | | | | 800 | 6.25 N·m | |
| J5 | zdatou | 6 | | | | 800 | 6.25 N·m | |

**落盘位置**：全部写进 `application/typedef_robot/robot_param_terminal.h`（`CUSTOM_TERMINAL_JOINT_DIR/GEAR/ZERO/MA_PER_NM`），`custom_terminal.c` 里不再出现机械数字。

**命名提醒**：本表的 `J0~J5` 是**数组下标**（= 上位机 `pos[0..5]`、Watch 里 `joint_fb[0..5]`、
`manual_joint` 写 0~5）；`J0` 对应 URDF 的 `Joint1`、`J1` 对应 `Joint2`。`ZERO` 是"关节角 = 0"
那一刻的电机侧机械角，而"关节角 = 0"的物理姿态由 `arm_dh_param[].offset` 钉在 URDF 零位上
（见 §S4 与 `doc/dh_from_urdf.md` §6）——两者都在往"零位"上使劲，别只标其中一个。

---

## 4. 安全红线

1. **任何一次上力矩，机械臂必须先托住。** 现在没有重力补偿，一旦使能就是自由落体。
2. **一次只动一个关节**，且该关节附近不要有手。
3. 第一次试力矩用 **5% 满量程**，确认方向对了再往上加。
4. 手边必须有**物理断电**手段（电池开关），不要只依赖软件急停。
5. 出现"往一个方向持续加速" → 立即断电，这是方向符号反了（S5 符号验证没做）。

---

## 5. 已完成 / 仍待补的代码

已经做完（本轮）：

1. **反馈不再冻结**——改成"定时上报 + 自愈补发"：开机给 5 台 ZDT 发一次 `11 18 0x31 00 0A 6B`，电机每 10 ms 主动上报；`CustomControllerObserver` 每秒检查 `zdatou_dbg.ok_mask`，对没上报的地址补发配置。
2. **力矩入口**——`terminal_dbg.manual_en/manual_joint/manual_torque_nm` 驱动单关节，带限速、硬上限、无反馈保护、换关节自动失能（§7）。
3. **安全三件套**——反馈超时（20 ms）、反馈无效不出力、力矩变化率限制，都在 `CustomControllerConsole` 里。
4. **零点标定入口**——`terminal_dbg.zero_capture` / `zero_readback[6]`（§9）。
5. **重力补偿**——`terminal_gravity()` 已实现并挂在 `CustomControllerReference` 里，`terminal_dbg.tau_g[]` 可看，默认 `gravity_en = 0`（§10）。

仍待补：

1. ~~**真 DH 表**——`alpha` 与 `d` 现在仍是 0，`DhTransform` 是 `Rj·T_fixed`。接重力补偿前必须换成真 DH，否则 `z[j]`（旋转轴）算错，补偿方向就是错的。~~
   **已完成**：`arm_dh_param[]` 已换成 URDF 推出的 6 行（`a/d/alpha/offset` 全有值），`base_transform`
   补了 frame0 的平移，`DH_Param_t` 加了 `.offset`；质量/质心也换到了 DH 系。推导、对拍数字、
   零位姿态见 `doc/dh_from_urdf.md`。
2. **软限位**——现在没有关节限位，只有力矩上限。做联动前要加。
3. **急停**——遥控输入已删，目前只能靠断电。

---

## 6. 数据从哪看（没有上位机时的退路）

- **上位机侧**：`pos[0..5]` 就是关节角（rad），够做 S1~S4。
- **Keil Watch 侧**：这是现在主要的观察手段，都在 `terminal_debug.h` 里声明为 volatile 全局（实现在 `terminal_debug.c`），能直接求值：
  - `terminal_dbg`：`joint_fb[6]`/`joint_raw[6]`/`joint_ecd[6]`/`torque_cmd[6]`/`tau_g[6]`/`fdb_cnt`/`armed`/`test_joint`
  - `zdatou_dbg`：`rx_cnt`/`ok_mask`/`addr_cnt[6]`/`addr_func[6]`/`addr_dlc[6]`/`addr_extid[6]`/`addr_data[6][8]`
  - `robostride_dbg`：`rx_cnt`/`last_std_id`/`last_motor_id`/`motor_mask`
- **帧级细节**（谁发的、ID 对不对、DLC 多少）：靠上面两个驱动自带的诊断，1 Mbps；有 CAN 分析仪更直观。
- 若 Watch 里出现 `cannot evaluate`：先 Rebuild + 重新下载 + 重启调试会话（符号在会话开始时加载），必要时用绝对地址表达式读，地址用 `fromelf --text -s <axf> | findstr terminal_dbg` 查。

---

## 7. 力矩测试模式（当前代码已实现）

用途：在还没有控制律的情况下，对**单个关节**施加可控的小力矩，完成 §2 的 S2（转向）、S3（减速比）、S4（零位）、S5（力矩标度）。

只有一种输入源：**手动模式**（Keil Watch 窗口写 `terminal_dbg` 的四个字段）。原来那套 DT7 遥控输入已按需求删除，`custom_terminal` 不再引用 `remote_control.h`。

### 7.1 手动模式

Watch 窗口加 `terminal_dbg`（或直接用绝对地址表达式，见 §6），改这四个字段：

| 字段 | 写什么 | 含义 |
| --- | --- | --- |
| `manual_en` | `1` | 进入手动模式。写回 `0` 立刻按斜坡回零并显式失能 |
| `manual_joint` | `0`~`5` | 要测的关节；`1` 是 RS00（robostride） |
| `manual_torque_nm` | 如 `1.0f` | 目标力矩，单位统一 **N·m**。ZDT 关节内部乘 `MA_PER_NM` 换成 mA，上限 `5000/MA_PER_NM`（默认 800 → 6.25 N·m）；J1 直接下发 N·m，上限 14 |
| `manual_ignore_feedback` | `0`/`1` | 只在选中电机确实不回帧、需要单独确认方向时临时写 `1`（无反馈保护，必须先托住） |

保护逻辑（都在 `CustomControllerConsole` 里）：选中关节 `joint_feedback_valid == 0` 时不出力（除非上一行那个开关）；力矩按 `CUSTOM_TERMINAL_TEST_NM_RAMP` 限速逼近；换关节/退出时先失能旧关节；`armed` 时其余关节恒 0。

### 7.2 使能/失能（这是"上电带力"的正解）

手册 5.3.2：驱动器**上电默认是使能态**，会锁轴闭环保持；只发 0 力矩帧是松不开的，必须显式发 `F3 AB 00 00 6B`。代码里：

- `CustomControllerInit` 上电先给 5 台 ZDT 发**失能**（`zdatou_set_enable(..., ZDATOU_ENABLE_OFF)`），再给 RS00 发使能（它必须先使能才回反馈帧），最后给 5 台配置定时上报。
- 手动模式只在**选中的那台**发使能（`F3 AB 01 00 6B`），换关节先失能旧的，退出全部失能。
- `zdatou_stop`（`FE 98 6B`）与 `zdatou_clear_stall`（`0E 52 6B`）已经在驱动里备好，需要时直接调。

### 7.3 参数在哪

| 位置 | 参数 |
| --- | --- |
| `application/typedef_robot/robot_param_terminal.h` | `CAN`、`JOINT_NUM`、`RS_JOINT`、`ZD_ADDR`、`RS_ID`、`RS_MASTER_ID`、`JOINT_ZERO`、`JOINT_DIR`、`JOINT_GEAR`、`JOINT_MA_PER_NM`、`ZD_TORQUE_MAX_MA`、`RS_TORQUE_MAX_NM`、`FEEDBACK_TIMEOUT_MS`、`ZD_REPORT_PARAM`、`LINK_MASS`、`LINK_COM`、`GRAVITY_SCALE` |
| `application/custom_controller/custom_terminal.h` | `CUSTOM_TERMINAL_TEST_MODE`（标定完置 0）、`CUSTOM_TERMINAL_TEST_NM_RAMP` |
| `application/custom_controller/terminal_kinematics.h` | DH 表 `arm_dh_param[6]` 与基座变换 `base_transform[4][4]`（改成实测 DH 后正解才准） |
| `application/custom_controller/terminal_gravity.h` | 重力补偿入口（质量/质心在 `robot_param_terminal.h`） |
| `application/custom_controller/terminal_debug.h` | `terminal_debug_t`（Watch 观察口；实现放 `terminal_debug.c`，可整体摘除） |

`custom_terminal.c` 里不再出现任何机械数字。

### 7.4 每个标定项怎么做

- **S2 转向**：选关节 → 给正力矩 → 看 `joint_fb[i]` 增大还是减小。方向反了改 `CUSTOM_TERMINAL_JOINT_DIR[i]`。
- **S3 减速比**：手转输出端一整圈，记 `joint_ecd`（ZDT）或 `joint_fb`，除以 2π 得减速比，填 `JOINT_GEAR[i]`。
- **S4 零位**：见 §9。
- **S5 力矩标度**：托住负载，`manual_torque_nm` 依次给 `0.2 / 0.5 / 1.0`，看方向与线性。ZDT 走电流环，`MA_PER_NM` 是待标定量；确认 `5000 mA` 是否真的到满量程（若 4000 与 5000 一样大，说明驱动器自己饱和了）。

### 7.5 安全动作顺序

1. 先做 S1（反馈持续性），确认 `fdb_cnt` 在涨、`joint_fb[i]` 真在变；不变就不要继续。
2. 机械臂**先托住或用支架固定**，再写 `manual_en = 1`。
3. 第一次只给最低档（`manual_torque_nm` 写 0.1）。
4. 任何时候把 `manual_en` 写回 0，输出都会在斜坡时间内回零并失能；异常直接关机。

---

## 8. 总线检查表（上车五分钟）

上电后按顺序看，任一项不过就别往下走：

| 检查 | 看什么 | 期望 | 不过怎么办 |
| --- | --- | --- | --- |
| 任务在跑 | `terminal_dbg.fdb_cnt` | 持续增大 | 查 `CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_TERMINAL`、`freertos.c` 是否建了任务 |
| ZDT 有帧 | `zdatou_dbg.rx_cnt` | 持续增大 | 接线/CAN 收发器/波特率（电机默认 500 k，本工程 1 M） |
| 五台都在 | `zdatou_dbg.ok_mask` | `0b011111`（地址 1,3,4,5,6 都解析成功） | 缺哪位看 `zdatou_dbg.addr_func[addr-1]` |
| 上报码对 | `zdatou_dbg.addr_func[i]` | `0x31`（或改 `ZD_REPORT_PARAM` 后的 `0x36`） | `0x02`=命令收到但没上报；`0xE2`=参数错；`0xEE`=格式错；`0x00`=不是 ZDT 应答（DLC 也不会是 4/6） |
| 帧长对 | `zdatou_dbg.addr_dlc[i]` | `4`（0x31）或 `6`（0x36） | 8 字节说明是别的设备的扩展帧 |
| RS00 有帧 | `robostride_dbg.rx_cnt` | 持续增大 | 必须**先使能**才应答；不涨则查电机 `CAN_MASTER` 是否等于 `RS_MASTER_ID` |
| RS00 ID 对 | `robostride_dbg.last_std_id` | `0x15`（或 bit10~8 非 0 的 `0xX15`） | 不等于 → 改电机参数或改 `CUSTOM_TERMINAL_RS_MASTER_ID` |
| 关节角在动 | `terminal_dbg.joint_fb[0..5]` | 手转对应关节时有变化 | 不动 = 该关节 `JOINT_GEAR`/地址/反馈码有问题 |
| 上行在发 | `huart1.gState` / `huart6.gState` | 绝大部分 `32`(READY)，每 34 ms 闪一次 `33`(BUSY_TX) | 一直是 33 说明 DMA 卡住，看 `CustomControllerSendToPC` 的 Abort 自愈计数 |

> 自愈机制：`CustomControllerObserver` 每 `CUSTOM_TERMINAL_REPORT_RETRY_MS`（1 s）检查一次 `zdatou_dbg.ok_mask`，对还没上报的地址补发一次"定时上报"配置。所以开机那一下丢配置不会让电机永远哑着，最多晚 1 s。

---

## 9. 零位标定与设置

默认走**软件零位**：电机原始角不动机内任何参数，只在应用层扣偏移。

步骤（对一个关节重复）：

1. 手动模式把关节摆到机械零位，`manual_en` 写回 0，等它停稳。
2. Watch 里把 `terminal_dbg.zero_capture` 写 `1`。
3. `CustomControllerObserver` 下一拍会把 6 个关节的 `joint_raw[]` 抄进 `terminal_dbg.zero_readback[0..5]`，并把 `zero_capture` 自动清 0。
4. 读 `zero_readback[i]`，填进 `robot_param_terminal.h` 的 `CUSTOM_TERMINAL_JOINT_ZERO[i]`（`joint_raw` 已经乘过 `JOINT_DIR`）。
5. 重新编译烧录，确认该关节在机械零位时 `joint_fb[i] ≈ 0`（±0.002 rad）。

注意：

- `joint_raw` 是**未扣零位、已乘方向**的电机侧机械角（rad），所以填的是"零位处的原始角"，不是偏移量。
- 零位精度：0.1° 在 300 mm 臂展上约 0.5 mm；用 `0x36`（0.1° 分辨率、可多圈）比 `0x31`（单圈 0.0055°分辨率但过零回绕）更适合多圈关节。
- 可选**硬件零位**（`zdatou_set_zero(can, ext_id, ZDATOU_SET_ZERO_NOW)` = `93 88 01 6B`，RS00 用指令 4）：好处是断电不丢、多圈计数从零开始；坏处是要改电机内部状态。建议先用软件零位跑通，等机械定型再考虑硬件零位。

---

## 10. 重力补偿分级投入

代码：`terminal_gravity()` 在 `CustomControllerReference` 里每 1 ms 用当前 `joint_feedback` 算一次，结果放在 `terminal_dbg.tau_g[0..5]`（N·m）。

分级：

| 级别 | 操作 | 判据 |
| --- | --- | --- |
| G0 | `terminal_dbg.gravity_en = 0` | 基线，确认手动模式正常 |
| G1 | 关节摆几个已知角度，只看 `tau_g[]` 的值与符号（不使能） | 肩关节水平时力矩最大、竖直时接近 0；符号应与"托住它需要的方向"一致 |
| G2 | `gravity_en = 1`、`gravity_scale = 0.2`，手托着慢慢松 | 臂"变轻"但不自己动 |
| G3 | `gravity_scale` 0.5 → 0.8 → 1.0 逐级 | 每级停 10 s，观察是否自激/漂移 |
| G4 | 全补偿下给关节小扰动 | 无持续振荡；若振荡先降 `gravity_scale`，再查 `LINK_MASS`/`LINK_COM` |

前提（缺一不可）：

1. §9 零位已标定——`tau_g` 直接吃 `joint_feedback`，零位错则补偿方向错；而且零位必须摆成 URDF 零位（§S4）。
2. `JOINT_GEAR` 已标定——减速比错，重力矩会按比例错。
3. DH 表的 `a/d/alpha/offset` 是**当前这把臂**的——已完成，由 URDF 推出（`doc/dh_from_urdf.md`），只剩按 §8 抽查两三个长度确认 CAD 与实物一致。
4. `LINK_MASS`/`LINK_COM` 已按 URDF 的 CAD 惯性参数填入；称一下总重（应约 0.655 kg），差 20% 以上要按实测改，否则只是"看起来有补偿"。
5. `tau_g` 输出**已经取反**（`terminal_gravity.c` 里 `tau_nm = -tau/1000`），即"电机要出的力矩"；如果 G2 一开就往低处跑，先查 `JOINT_DIR` 的符号，别急着改质量。

`terminal_dbg.gravity_en` 与手动模式互斥：`armed` 为真时重力补偿不叠加（手动模式优先）。