# zdatou 与 robostride 对比评审

> 范围：`application/robot_cmd/CAN_cmd_zdatou.c/h`、`CAN_cmd_robostride.c/h`、`CAN_receive.c`、`bsp/boards/bsp_can.c`、
> `controler_myself.c` 中的实际用法。
> 基准：`CAN_cmd_robostride.md`（厂家手册，随仓库提交）与 v1.2 的驱动分层写法。

## 0. 结论速览

| 维度 | zdatou（大头 SB） | robostride（RS00） |
|---|---|---|
| 帧格式 | 扩展帧，ID 0x100×k（0x100~0x600） | 标准帧，ID = 电机 id（1~6），反馈 StdId = 主机 id 0x15 |
| 协议文档 | **无**，全是裸字节 magic number | 1864 行手册提交在仓库里 |
| 使能/停止 | 无使能概念，上电直接吃力矩 | 指令1 使能（8×FF+FC）；**从未发停止指令2、清错指令5** |
| 控制量 | 方向字节 + 16 位力矩幅值 | MIT 五参数帧，实际只发 tor |
| 位置反馈 | 16 位单圈 ecd → ±π | 16 位 ±12.57 rad ≈ ±4π（±2 圈） |
| 速度反馈 | 无（软件差分） | 有，12 位 ±33 rad/s |
| 力矩反馈 | 无 | 有，12 位 ±14 N·m |
| 温度/故障 | 无 | 有 temp/fault/warning/mode_state，**无人使用** |
| 越界保护 | 无，低 16 位静默截断 | 有 12 位映射，但 `float_to_uint` 不 clamp → 越界回绕 |
| 多电机并存 | ID 天然区分，6 路独立 | 反馈共用同一 StdId，**只能存 1 路**（硬编码 `[5]`） |
| 接入 `Motor_s` 抽象 | 有枚举 + 适配器（但调错函数） | **完全没有** |

一句话：robostride 侧协议规范、反馈信息丰富但只用了 20%；zdatou 侧能用但完全不可维护，且两者在单位、量程、多圈语义上都不统一，这会在做重力补偿/阻抗时立刻咬人。

## 1. 驱动层差异

### 1.1 zdatou：三帧裸字节

- `CAN_cmd_zdatou.c:12-32` `zdatou_ecd_get()`：发 `11 18 31 00 0a 6b`（DLC=6）。字段含义无注释、无文档。
  `0x0a` 从数值上看极可能表示"10 ms 上报周期"，但**没有任何证据**；若它只是一次性读位置，则位置永远停在初值。
  全项目仅在 `controler_myself.c:523` 初始化时各发一次，之后没有任何轮询或超时检测（详见 controler_myself 评审 A3）。
- `CAN_cmd_zdatou.c:41-76` `zdatou_torque_control()`：
  - `direction` 形参**被函数体覆盖**（`52-60` 行按 torque 符号重写），调用方传什么都无效。
  - `torque` 是 `int32_t`，但只取了低 16 位（`71-72` 行 `(uint8_t)(torque>>8)`、`(uint8_t)torque`），
    超过 ±32767 静默回绕，没有任何限幅。注释掉的 `controler_myself.c:638` 里 `datouctrl[2]/8000` 暗示满量程约 8000，即实际可用范围比 int16 窄得多。
- `CAN_cmd_zdatou.c:78-89` `zdatouFdbData()`：
  - `if (rx_data[0] == 0x31 || rx_data[3] == 0x6b)` **应为 `&&`**。现在任何尾字节为 0x6b 的回复帧都会把 `rx_data[1..2]` 当成 ecd 写进去。
  - 只有 `ext_id/ecd/last_fdb_time` 三个字段，没有电机 id、温度、故障、力矩。
- 命名三种风格混用：`zdatou_ecd_get` / `zdatou_torque_control`（snake）、`zdatouFdbData`（camel）、`GetzdatouMeasurePoint`（无下划线）。建议统一为 `zdatou_xxx_yyy`。

### 1.2 robostride：规范但只用了前馈力矩

- `CAN_cmd_robostride.c:17-39` 使能帧 `8×FF+FC`，与手册"指令1"逐字节一致 ✅。
  但全项目只在 `controler_myself.c:519` 调用一次；手册里的指令2（停止）、指令5（清错）、指令6（设模式）从未使用，
  掉电、堵转保护后没有重新使能路径，也没有检查 `mode_state` 是否真的进了 Motor 模式(2)。
- `CAN_cmd_robostride.c:52-89` MIT 运控帧：
  - pos/vel/kp/kd 四个量**算完之后全部丢弃**（`63-67` 行算出 `pos_int/vel_int/kp_int/kd_int`，`80-85` 行直接写 0），实际只有 `tx_data[6..7]` 的力矩生效 → 等价于"纯前馈力矩"。
  - 逻辑上成立（`t_ref = Kp(p_set-p_act) + Kd(v_set-v_act) + t_ff`，Kp=0 时位置项消失），但**极脆弱**：pos=0 映射到 P_MIN = **-12.57 rad**，一旦有人打开 kp，电机会立刻冲向 -12.57 rad。建议显式把目标位置写成当前反馈值，kp/kd 走形参。
  - `float_to_uint`（`components/algorithm/user_lib.c:198-204`）**没有 clamp**，而厂家样例（手册 `4.4.2`）是带 clamp 的。力矩越界时 `(x-min)` 变负 → 负值赋给 `uint16_t` 回绕 → 发出的字节完全是垃圾。当前调用方传 0 所以没爆，接重力补偿后必爆。
- `CAN_cmd_robostride.c:98-117` 反馈解码：与手册"应答指令1"的 bit 布局逐位吻合 ✅
  （pos 16 位、vel `Byte3<<4 | Byte4>>4`、tor `Byte4&0x0F<<8 | Byte5`、temp `Byte6&0x0F<<8 | Byte7`、mode[7:6]、fault[5]、warning[4]）。
- 反馈 StdId 用 `ROBOSTRIDE_MASTER_ID 0x15`（`CAN_cmd_robostride.h:36`），但代码/文档没说这个主机 id 是怎么配到电机里的（手册"指令9：修改主机canid"）。若现场电机主机 id 不是 0x15，则一条反馈都收不到，而且没有任何诊断手段。

### 1.3 发送层（两者共用）

`bsp/boards/bsp_can.c:33-46` `CAN_SendTxMessage()`：忙等 20 次邮箱、结果塞进全局 `a3`、**无返回值**。
上层无法感知丢帧；`CAN_cmd_zdatou.c:10` 还留着一个同源的 `HAL_StatusTypeDef a4;` 垃圾变量。
`CAN_CTRL_DATA` 是各自文件内的 static，`IDE` 每次调用都重设，暂不会串帧，但 `StdId`/`ExtId` 从不清零，属于隐患。

## 2. 使用层差异（六轴臂里怎么用）

### 2.1 关节分配

| 关节 | 电机 | 命令通道 | 位置来源 |
|---|---|---|---|
| J1 | zdatou 0x100 | `zdatou_torque_control(1,0x100,...)` | `CAN1_ZDATOU_MEASURE[0].ecd` |
| J2 | robostride id 5 | `Robostride_mit_control(1,0x5,0,0,0,0,τ)` | `CAN1_ROBOSTRIDE_MEASURE[5].pos` |
| J3~J6 | zdatou 0x300/0x400/0x500/0x600 | 同上 | `CAN1_ZDATOU_MEASURE[2..5].ecd` |

- **0x200 没有电机**，但 `controler_myself.c:523` 的初始化循环仍然给 0x200 发了读位置帧（`GimbalSendCmd` 里 0x200 的力矩发送是被注释掉的 `:667`）。等于每上电多发一帧无人应答的请求。
- 6 个关节在第 1 个 1 ms 周期里只发了 2 条，全部发完需要 3 个周期（`GimbalSendCmd` 的 `sendctrl` 轮转），即每个电机 3 ms 一次、相位还错开 —— 对 1 kHz 的阻抗/重力补偿来说这是硬伤。

### 2.2 反馈语义不对等（最关键）

- `controler_myself.c:556` 对 zdatou：`ecd/65535*2π - offset` 再 `WrapAngle` → **单圈 ±π**，多圈信息在 `WrapAngle` 里被彻底丢掉。
- `controler_myself.c:582` 对 robostride：`WrapAngle(robostride->pos - offset)` —— robostride 本来给的是 ±4π（±2 圈）的**多圈**角度，被同一个 `WrapAngle` 折回 ±π，唯一的多圈优势也没了。
- 速度：J1/J3~J6 由 16 位 ecd 差分（`controler_myself.c:563`），1 ms 周期下量化噪声约 0.096 rad/s 量级的跳变；J2 直接用 robostride 的 `vel`（真实测速）。**同一个阻抗参数在两类关节上表现必然不同**，必须先做滤波/归一化，否则调参调不出来。
- 力矩单位：zdatou 侧交的是原始计数（满量程约 8000），robostride 侧交的是 N·m（±14）。将来加 `tau_gravity` 时必须先统一到关节侧 N·m，并就有一张"每关节 kt / 满量程 / 减速比 / 方向"表。

### 2.3 抽象层接入

- `robot_typedef.h:74-84` `MotorType_e` 里有 `zdatou`（第 83 行），**没有 robostride**。
- `CAN_receive.c:597-617` `GetMotorMeasure()` 的 `case zdatou:` 是复制 `MF_9025` 分支改出来的，但**改漏了函数**：
  `611/615` 行仍调用 `GetLkFdbData(p_motor, &CAN1_LK_MEASURE[p_motor->id - 1])`，应为 `GetDTFdbData(p_motor, &CAN1_ZDATOU_MEASURE[p_motor->id - 1])`。
  于是通过 `Motor_s` 拿到的 zdatou 反馈其实是**领控电机的数据**（pos/vel/tor/temp 全错）。
- 更糟：`LK_NUM` 只有 4（`motor.h:184`），而 zdatou id 是 1~6 → id=5/6 时 `[id-1]` 越界读到相邻全局变量。
- `case MF_9025:`（`597`）末尾**缺 break**，会贯穿到 `case zdatou:` 再调一次 `GetLkFdbData`。
- `GetDTFdbData()`（`CAN_receive.c:535-547`）写好了但**全项目无人调用**，是死代码。
- robostride 只有裸数组 + `GetRobostrideMeasurePoint()`，而后者返回 `&...[5]`（硬编码），`CAN_receive.c:258-268` 也把索引硬编码成 `[5]` —— 那行 `uint8_t i = rx_data[0]; if (i>=1 && i<=6)` 的检查形同虚设，**总线上挂第 2 个 robostride 就会被覆盖**。

## 3. 建议（按优先级）

**P0 — 功能性 bug（不改会算错/读越界）**
1. `CAN_receive.c:611/615` 改调 `GetDTFdbData(... &CAN1_ZDATOU_MEASURE[p_motor->id-1])`，并补 `case MF_9025:` 的 `break`。
2. `CAN_cmd_zdatou.c:82` 的 `||` 改 `&&`。
3. zdatou 的 `id-1` 索引与 `LK_NUM` 无关后，把 `MotorType_e` 的 id 语义与数组下标约定写进注释。

**P1 — 让两者真正可替换**
4. 把 robostride 纳入 `MotorType_e` + `GetMotorMeasure` 适配器；`CAN_receive.c:263/267` 用 `rx_data[0]` 索引（`[i-1]`），删掉 `[5]` 硬编码；`GetRobostrideMeasurePoint(can, id)` 加 id 参数。
5. `GimbalSendCmd` 改成每周期发全部 6 路（或用 `CanManager` 统一调度），别再用 `sendctrl` 轮转。
6. zdatou 驱动加超时/离线判定（目前只有 robostride 侧由上层算 `last_fdb_time`）。

**P2 — 单位与量纲统一**
7. 建"每关节参数表"：`direction`、`reduction_ratio`、`zero_offset`、`torque_full_scale`（zdatou 原始计数 → N·m）、`kt`。放进 `robot_param_*.h`，不要散落在 DH 表的 `theta_offset` 和 `axis` 里。
8. robostride 的 `pos` 保留多圈（去掉 `WrapAngle`，用 `atan2` 差分做连续性检查），zdatou 若要单圈就在驱动层显式声明"单圈"。

**P3 — 可维护性**
9. 给 `CAN_cmd_zdatou.c` 补帧格式注释（每个字节是什么、0x6b 是什么、0x0a 是什么），或至少链到厂家文档；它现在和 robostride 的文档化程度差了一个数量级。
10. 补 robostride 的停止/清错/重新使能路径，并把 `fault/warning/temp/mode_state` 接进 `GimbalHandleException`。
11. `float_to_uint` 的 clamp 放在驱动层本地实现（不要改 `user_lib.c`，会影响其他电机）；zdatou 侧加 16 位饱和。
12. 删 `CAN_cmd_zdatou.c:10` 的 `a4`、`bsp_can.c:2` 的 `a3`；`CAN_SendTxMessage` 返回值化，让上层能统计丢帧。
13. 命名统一：`zdatou_fdb_data` / `zdatou_get_measure_point`，与 `zdatou_ecd_get` 对齐；`MotorType_e` 里的 `zdatou` 改 `ZDATOU_MOTOR` 与其余大写风格一致。

## 4. 一句话

robostride 是"协议对、信息全、但我们只用了一条前馈力矩"；zdatou 是"能用、但每一帧都是猜的"。
两者现在混在同一个六轴臂里，**量纲、多圈语义、反馈质量、抽象层支持**四处不一致，所以在补重力补偿/阻抗之前，先按 P0→P2 把接口对齐，否则上层算法写对了也会被底层数据骗。
