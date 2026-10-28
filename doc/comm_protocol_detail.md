# RoboMaster 2026 通信协议详解（含本项目实现对照）

> 依据：`doc/RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）.pdf`、
> `doc/RoboMaster裁判系统相机图传模块VT03&VT13使用说明书.pdf`
> 对照工程：`D:\RM\new_remoto\new_remoto`（C 板下板 / 六轴自定义控制器终端）
> 编写时间：2026-10-28

---

## 目录

1. [物理层与链路拓扑](#1-物理层与链路拓扑)
2. [帧格式](#2-帧格式)
3. [两个 CRC 到底校验哪些字节](#3-两个-crc-到底校验哪些字节)
4. [命令码总表](#4-命令码总表)
5. [逐条命令详解](#5-逐条命令详解)
6. [VT03/VT13 遥控帧（21 字节）](#6-vt03vt13-遥控帧21-字节)
7. [本项目实现对照](#7-本项目实现对照)
8. [已知问题清单](#8-已知问题清单)
9. [实测自检清单](#9-实测自检清单)

---

## 1. 物理层与链路拓扑

### 1.1 三条链路

裁判系统不是一个口，而是**三条彼此独立的链路**：

```
                         ┌──────────────┐
       常规链路            │  裁判系统服务器 │
       (115200)          └──────┬───────┘
                                │ 以太网/无线
                         ┌──────▼───────┐
                         │  电源管理模块  │──User 串口(115200)──┐
                         └──────────────┘                     │
                                                              ▼
  ┌──────────┐   图传链路(921600)   ┌────────────┐       ┌──────────┐
  │  选手端   │◄──────────────────►│ 图传发送端  │       │ 机器人主控 │
  └────┬─────┘                     └────────────┘       └────▲─────┘
       │ 115200                                              │
       ▼                                                     │
  ┌──────────────┐      图传链路(921600)                      │
  │ 自定义控制器  │◄──────────────────────────────────────────┘
  └──────────────┘
   ┌──────────┐   雷达无线链路(电磁波)   ┌──────────┐
   │ 信号发射源 │──────────────────────►│   雷达    │
   └──────────┘                        └──────────┘
```

### 1.2 串口参数表（协议表 1-1）

| 物理接口 | 波特率 | 说明 |
|---|---|---|
| 电源管理模块 ↔ 机器人 | **115200** | 常规链路，除雷达外的所有比赛数据都从这里进出 |
| 自定义控制器 ↔ 选手端 | **115200** | 图传链路（0x0302 / 0x0309 / 0x0310 / 0x0311） |
| 图传发送端 ↔ 机器人 | **921600** | 图传链路；**2026 起从 115200 提到 921600** |

帧格式（协议表 1-2）：

```
┌──────────────┬────────┬──────────┬────────────┐
│ frame_header │ cmd_id │   data   │  frame_tail │
│    5 byte    │ 2 byte │  n byte  │   2 byte    │
└──────────────┴────────┴──────────┴────────────┘
```

### 1.3 时序特性

- 正常：延迟约 **130 ms**，丢包率 < 1%
- 赛场网络恶劣：延迟约 **200 ms**，丢包率约 3%
- 结论：**任何依赖裁判数据的控制环都必须自己能兜底**（丢帧时保持上一帧、超时降级）

---

## 2. 帧格式

### 2.1 字节图（总长 = 5 + 2 + n + 2 = n + 9）

| 偏移 | 0 | 1-2 | 3 | 4 | 5-6 | 7 … 7+n-1 | 7+n, 8+n |
|---|---|---|---|---|---|---|---|
| 内容 | SOF `0xA5` | data_length (u16 LE) | seq | CRC8 | cmd_id (u16 LE) | data | CRC16 (u16 LE) |
| 大小 | 1 | 2 | 1 | 1 | 2 | n | 2 |

协议表 1-4 的原文定义：

| 域 | 偏移 | 大小 | 说明 |
|---|---|---|---|
| SOF | 0 | 1 | 固定 `0xA5` |
| data_length | 1 | 2 | **数据帧中 data 的长度**（不含帧头、不含 cmd_id、不含 CRC16） |
| seq | 3 | 1 | 包序号 |
| CRC8 | 4 | 1 | 帧头 CRC8 校验 |

### 2.2 三个容易搞错的点

1. **`data_length` 只算 `data`**。整帧长度 = `data_length + 9`。很多新手把 `data_length` 写成整帧长度，导致对端按 `+9` 一加就跑偏。
2. **`cmd_id` 是数据的一部分，但不算在 `data_length` 里**。它是帧头和 data 之间的独立字段，也要参与 CRC16。
3. **字节序是小端**。`data_length`、`cmd_id`、CRC16、以及所有 `uint16_t`/`float`/`uint64_t` 字段全部低字节在前。结构体声明必须 `__packed`，否则编译器插入的 padding 会让长度对不上（ARM 上 `uint64_t` 尤其要注意 8 字节对齐填充）。

### 2.3 一帧的真实例子（0x0001 比赛状态，11 字节数据）

```
A5 0B 00 01 CRC8 | 01 00 | <11 字节比赛状态> | CRC16_L CRC16_H
└─ 帧头 5B ─────┘ └cmd┘   └──── data ────┘   └─ 尾 2B ─┘
                         整帧 20 字节 = 11 + 9
```

---

## 3. 两个 CRC 到底校验哪些字节

这是最容易写错的地方：**两个 CRC 的覆盖范围不同**。

| 校验 | 算法 | 覆盖范围 | 存放位置 |
|---|---|---|---|
| CRC8 | poly `x^8+x^5+x^4+1`（0x31），init `0xFF`，查表，不反射、不异或 | **只算帧头前 4 字节**（SOF、data_length 低/高、seq） | 帧头第 5 字节（偏移 4） |
| CRC16 | 多项式按反射式 0x8408 建表（即 0x1021 反射），init `0xFFFF`，不异或 | **整帧除最后 2 字节**：帧头 5B + cmd_id 2B + data nB | 帧尾 2 字节，**低字节在前** |

参考实现（协议附录一，与工程 `components/support/CRC8_CRC16.c` 同源）：

```c
/* CRC8：一次一个字节，异或后查表 */
uint8_t get_CRC8_check_sum(uint8_t *msg, uint32_t len, uint8_t crc)
{
    while (len--) crc = CRC8_TAB[crc ^ (*msg++)];
    return crc;
}

/* CRC16：反射式查表，先右移 8 位再异或表值 */
uint16_t get_CRC16_check_sum(uint8_t *msg, uint32_t len, uint16_t crc)
{
    while (len--) crc = (crc >> 8) ^ wCRC_Table[(crc ^ (*msg++)) & 0xFF];
    return crc;
}

/* 校验：低字节在 len-2，高字节在 len-1 */
bool verify(uint8_t *msg, uint32_t len)
{
    uint16_t expect = get_CRC16_check_sum(msg, len - 2, 0xFFFF);
    return (expect & 0xFF) == msg[len - 2] && (expect >> 8) == msg[len - 1];
}
```

**注意**：VT03 图传模块的 21 字节帧用的是 **CRC-16/CCITT-FALSE**（poly 0x1021、不反射、init 0xFFFF），**不能**复用上面这张 0x8408 反射表。两套 CRC 必须分开实现，见第 6 节。

---

## 4. 命令码总表

按协议表 1-5 整理（**长度列有若干处与结构体定义自相矛盾，见本节末尾警告**）。

### 4.1 常规链路 · 服务器 → 机器人

| cmd_id | 长度 | 内容 | 频率 |
|---|---|---|---|
| 0x0001 | 11 | 比赛状态 | 1 Hz 固定 |
| 0x0002 | 1 | 比赛结果 | 比赛结束触发 |
| 0x0003 | 20 | 机器人血量（己方 + 对方前哨/基地 + 伤害差） | 3 Hz 固定 |
| 0x0101 | 4 | 场地事件（占领/增益点位图） | 1 Hz 固定 |
| 0x0104 | 3 | 裁判警告（判罚） | 触发 + 其余 1 Hz |
| 0x0105 | 3 | 飞镖发射数据 | 1 Hz 固定 |
| 0x0204 | 8 | 机器人增益 + 剩余能量 | 3 Hz 固定 |
| 0x0208 | 8 | 允许发弹量 + 金币 + 堡垒储备 | 10 Hz 固定 |
| 0x0209 | 5 | RFID 状态 | 3 Hz 固定 |
| 0x020A | 6 | 飞镖选手端指令 | 3 Hz 固定 |
| 0x020B | 40 | 地面机器人位置（给哨兵） | 1 Hz 固定 |
| 0x020C | 2 | 雷达标记进度（易伤） | 1 Hz 固定 |
| 0x020D | 14 | 哨兵自主决策信息同步 | 1 Hz 固定 |
| 0x020E | 1 | 雷达自主决策信息同步 | 1 Hz 固定 |
| 0x0305 | 48 | 小地图坐标（雷达 → 选手端，12 台 ×2） | ≤5 Hz |

### 4.2 常规链路 · 主控模块 → 机器人

| cmd_id | 长度 | 内容 | 频率 |
|---|---|---|---|
| 0x0201 | 17 | 机器人性能体系（血量/热量/功率/初速上限） | 10 Hz 固定 |
| 0x0202 | 14 | 缓冲能量 + 枪口热量 | 10 Hz 固定 |
| 0x0203 | 12 | 本机位置（x, y, angle） | 1 Hz 固定 |
| 0x0206 | 1 | 伤害状态（装甲板 + 扣血原因） | 伤害发生后 |
| 0x0207 | 7 | 实时射击数据 | 弹丸发射后 |

### 4.3 常规链路 · 机器人 ↔ 机器人 / 选手端

| cmd_id | 长度 | 内容 | 频率 |
|---|---|---|---|
| 0x0301 | ≤118 | 机器人交互数据（绘图/自定义消息/哨兵/雷达指令） | 触发，≤30 Hz |
| 0x0303 | 12 | 选手端小地图点击指令 | 选手端触发，≥0.5 s 间隔 |
| 0x0307 | 105 | 小地图路径（哨兵/半自动 → 选手端） | ≤1 Hz |
| 0x0308 | 34 | 小地图自定义消息（utf-16） | ≤3 Hz |

### 4.4 图传链路

| cmd_id | 长度 | 方向 | 频率上限 |
|---|---|---|---|
| 0x0302 | 30 | 自定义控制器 → 机器人 | 30 Hz |
| 0x0309 | 30 | 机器人 → 自定义控制器 | 10 Hz |
| 0x0310 | 300 | 机器人 → 自定义客户端 | 50 Hz（无重传） |
| 0x0311 | 30 | 自定义客户端 → 机器人 | 75 Hz |

### 4.5 雷达无线链路（电磁波，非串口）

| cmd_id | 长度 | 内容 | 频率 |
|---|---|---|---|
| 0x0A01 | 24 | 对方 6 台位置（cm） | 10 Hz |
| 0x0A02 | 12 | 对方血量 | 10 Hz |
| 0x0A03 | 10 | 对方剩余发弹量 | 10 Hz |
| 0x0A04 | 8 | 对方宏观状态（金币 + 状态位图） | 10 Hz |
| 0x0A05 | 41 | 对方各机增益 + 姿态 + 存活状态 | 10 Hz |
| 0x0A06 | 6 | 对方干扰波密钥 | 10 Hz |

### 4.6 ⚠ 协议文档自身的三处长度矛盾

协议表 1-5 的"数据段长度"列与后面结构体定义不一致，**实现时必须以后面逐条表的结构体为准**，并建议实测抓包复核：

| cmd_id | 表 1-5 写的长度 | 结构体实际长度 | 差异 |
|---|---|---|---|
| 0x0203 | 16 | **12**（float x, y, angle） | 多算 4 |
| 0x0303 | 15 | **12**（4+4+1+1+2） | 多算 3 |
| 0x0307 | 103 | **105**（1+2+2+49+49+2） | 少算 2 |

同源问题：工程里 `referee.h` 的注释长度（0x0303 写 15、0x0307 写 103、0x0305 写 24）也是从旧表抄的，结构体本身又各自不同，见第 8 节。

---

## 5. 逐条命令详解

### 5.1 比赛状态 0x0001（11 B）

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 1 | `game_type:4`（低 4 位）+ `game_progress:4`（高 4 位） |
| 1 | 2 | `stage_remain_time`（当前阶段剩余秒数） |
| 3 | 8 | `SyncTimeStamp`（UNIX 时间，接通 NTP 后有效） |

```c
typedef __packed struct {
    uint8_t  game_type     : 4;   // 1 超级对抗赛 / 2 高校单项赛 / 3 ICRA / 4 联盟赛3V3 / 5 联盟赛步兵对抗
    uint8_t  game_progress : 4;   // 0 未开始 / 1 准备 / 2 15s自检 / 3 5s倒计时 / 4 比赛中 / 5 结算
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;
```

**位域顺序坑**：C 位域从低位开始分配，所以先写 `game_type` 再写 `game_progress`；反过来会让两个字段互换。跨编译器时更稳妥的写法是读一个 `uint8_t` 再手动掩码。

### 5.2 比赛结果 0x0002（1 B）

`winner`：0 平局 / 1 红方胜 / 2 蓝方胜。

### 5.3 机器人血量 0x0003（20 B）

```c
typedef __packed struct {
    uint16_t ally_1_robot_HP;    // 0  己方英雄
    uint16_t ally_2_robot_HP;    // 2  己方工程
    uint16_t ally_3_robot_HP;    // 4  己方 3 号步兵
    uint16_t ally_4_robot_HP;    // 6  己方 4 号步兵
    int16_t  damage_difference;  // 8  己方总伤害 - 对方总伤害（2026 新增）
    uint16_t ally_7_robot_HP;    // 10 己方哨兵
    uint16_t ally_outpost_HP;    // 12
    uint16_t ally_base_HP;       // 14
    uint16_t enemy_outpost_HP;   // 16
    uint16_t enemy_base_HP;      // 18
} game_robot_HP_t;
```

**2026 最大变化**：只发"己方各机器人 + 双方前哨站/基地"，**不再有对方各机器人的血量**；新增 `damage_difference`。想显示 5 号步兵血量只能自己去 0x0301 问队友。未上场/被罚下的机器人血量填 0。

### 5.4 场地事件 0x0101（4 B）

一个 `uint32_t event_data` 位图（占领状态 0/1、飞镖击中时间等）。2026 新增了**己方堡垒增益点（bit25-26）、己方前哨站增益点（bit27-28）、己方基地增益点（bit29）**。逐位定义以协议表 1-9 为准，代码里**千万不要把整个 32 位数当布尔用**（见第 8 节问题 3）。

### 5.5 裁判警告 0x0104（3 B）

`level`、`offending_robot_id`、`count`。只在己方被判罚时才会推送到己方队员。

### 5.6 飞镖 0x0105（3 B）

`dart_remaining_time`（u8）+ `dart_info`（u16 位域：最近击中目标 1-5、累计击中次数 ≤4、当前选定目标）。

### 5.7 机器人性能体系 0x0201（17 B）

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 1 | `robot_id` |
| 1 | 1 | `robot_level` |
| 2 | 2 | `current_HP` |
| 4 | 2 | `maximum_HP` |
| 6 | 2 | `shooter_barrel_cooling_value`（每秒冷却值） |
| 8 | 2 | `shooter_barrel_heat_limit`（热量上限） |
| 10 | 2 | `chassis_power_limit`（底盘功率上限，W） |
| 12 | 4 | **`float bullet_speed_limit`**（射击初速度上限，2026 新增） |
| 16 | 1 | `power_management_gimbal_output:1 / chassis_output:1 / shooter_output:1` |

这个 float 是 2026 才加的，13 B → 17 B。按 13 B 解析会让后面的 0x0202 之类全部读错位。

### 5.8 缓冲能量与热量 0x0202（14 B）

```c
typedef __packed struct {
    uint16_t reserved;   // 0
    uint16_t reserved;   // 2
    float    reserved;   // 4
    uint16_t buffer_energy;            // 8  缓冲能量 J
    uint16_t shooter_17mm_barrel_heat; // 10
    uint16_t shooter_42mm_barrel_heat; // 12
} power_heat_data_t;
```

2026 把原来的 `17mm_1 / 17mm_2` 两个字段合并成一个 `shooter_17mm_barrel_heat`，长度 16 → 14。**这是最容易被旧代码读错的一条**：按 16 B 解析时，offset 12 会读到 `42mm` 的低位，offset 14 直接越界。

### 5.9 本机位置 0x0203（12 B）

`float x, y, angle`（m / m / 度，正北为 0）。表 1-5 写 16 是笔误。

### 5.10 增益 0x0204（8 B）

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 1 | `recovery_buff`（回血 %） |
| 1 | **2** | `cooling_buff`（热量冷却增益，直接值/s，**2026 从 u8 变 u16**） |
| 3 | 1 | `defence_buff`（防御 %） |
| 4 | 1 | `vulnerability_buff`（负防御 %） |
| 5 | 2 | `attack_buff`（攻击 %） |
| 7 | 1 | `remaining_energy`（bit0-6：能量阈值 125/100/50/30/15/5/1%） |

6 B → 8 B。只改长度不改字段顺序的话，`defence` 之后全部错位。

### 5.11 伤害状态 0x0206（1 B）

`armor_id:4`（装甲板编号）+ `HP_deduction_reason:4`（扣血原因）。

### 5.12 实时射击 0x0207（7 B）

`bullet_type`、`shooter_number`、`launching_frequency`（Hz）、`float initial_speed`（m/s）。

### 5.13 允许发弹量 0x0208（8 B）

`17mm`、`42mm`、`remaining_gold_coin`，**新增 `projectile_allowance_fortress`（堡垒储备弹量）**。6 B → 8 B。

### 5.14 RFID 状态 0x0209（5 B）

```c
typedef __packed struct {
    uint32_t rfid_status;    // 0
    uint8_t  rfid_status_2;  // 4  （2026 新增）
} rfid_status_t;
```

4 B → 5 B。2026 的位表**整体重排**：bit0 己方基地、bit1 己方中央高地、bit2 对方中央高地、bit3 己方梯形高地、bit4 对方梯形高地、bit5-8 飞坡、bit9-16 地形跨越、bit17 己方堡垒、bit18 己方前哨站、bit19 补给区、bit20 重叠补给区、bit21-22 装配增益点、bit23 中心增益点、bit24 对方堡垒、bit25 对方前哨站、bit26-31 及 `rfid_status_2` 的隧道类。**沿用旧代码的 `bit19 = 中心增益点` 是错的**。

### 5.15 飞镖选手端指令 0x020A（6 B）

`dart_launch_opening_status`、`reserved`、`target_change_time`、`latest_launch_cmd_time`。

### 5.16 地面机器人位置 0x020B（40 B，只给哨兵）

`float hero_x/y, engineer_x/y, standard_3_x/y, standard_4_x/y, reserved, reserved`（m）。

### 5.17 雷达标记进度 0x020C（2 B）

`uint16_t mark_progress`（2026 从 1 B 变 2 B）：

- bit0-3：对方 1/2/3/4 号的易伤情况
- bit4：对方空中特殊标识
- bit5：对方哨兵易伤
- bit6-9：己方 1-4 号特殊标识
- bit10：己方空中
- bit11：己方哨兵
- bit12：对方空中被激光瞄准
- bit13：对方空中被反制
- bit14：己方空中被瞄准
- bit15：己方空中被反制

### 5.18 哨兵自主决策同步 0x020D（14 B）

```c
typedef __packed struct {
    uint32_t sentry_info;    // 0
    uint16_t sentry_info_2;  // 4
    uint64_t sentry_info_3;  // 6  （2026 新增，姿态剩余时长）
} sentry_info_t;
```

注意 offset 6 处的 `uint64_t` 是**非对齐**的：`__packed` 下编译器会生成逐字节访问，直接成员访问在 Cortex-M4 上可能触发 `UNALIGNED` 用法错误，用 `memcpy` 拷出来再读更安全。

### 5.19 雷达自主决策同步 0x020E（1 B）

`uint8_t radar_info`：bit0-1 双倍易伤机会、bit2 是否正在生效、bit3-4 己方加密等级、bit5 可否改密钥。

### 5.20 机器人交互数据 0x0301（≤118 B）

```
┌───────────────┬───────────┬─────────────┬──────────────────┐
│ data_cmd_id   │ sender_id │ receiver_id │ user_data[x]     │
│ 2 B           │ 2 B       │ 2 B         │ x ≤ 112          │
└───────────────┴───────────┴─────────────┴──────────────────┘
```

即**固定头 6 B + 内容 ≤112 B = 数据段 ≤118 B**，整帧 ≤127 B。发送频率整体 ≤30 Hz，多个子内容共享这个带宽。

| 子内容 ID | 长度 | 功能 |
|---|---|---|
| 0x0200~0x02FF | ≤112 | 机器人之间自定义通信 |
| 0x0100 | 2 | 选手端删除图层 |
| 0x0101 | 15 | 绘制一个图形 |
| 0x0102 | 30 | 绘制两个图形 |
| 0x0103 | 75 | 绘制五个图形 |
| 0x0104 | 105 | 绘制七个图形（6 + 105 = 111 B，一帧刚好装得下） |
| 0x0110 | 45 | 绘制字符图形 |
| 0x0120 | 4 | 哨兵自主决策指令 |
| 0x0121 | **8** | 雷达自主决策指令（**2026 从 1 B 扩到 8 B**） |

`0x0120 sentry_cmd_t{uint32_t sentry_cmd}` 位定义：bit0 确认复活、bit1 兑换立即复活、bit2-12 兑换弹量、bit13-16 远程兑换弹量次数、bit17-20 远程兑换血量次数、bit21-23 姿态（1 进攻 / 2 防御 / 3 移动 / 4 强化进攻 / 5 强化防御 / 6 强化移动，默认 3）、bit24 能量机关激活。

`0x0121 radar_cmd_t`：`{uint8_t radar_cmd; uint8_t password_cmd; uint8_t password_1..6;}`。第一字节触发双倍易伤（**必须单调 +1**，否则视为非法）；第二字节 1 = 更新己方密钥、2 = 验证/破解对方密钥（每次操作后 10 s 内不能再次修改）。

图形结构 `interaction_figure_t`（15 B）：`operate_type:3 / figure_type:3 / layer:4 / color:4 / details_a:9 / details_b:9 / width:10 / start_x:11 / start_y:11 / details_c:10 / details_d:11 / details_e:11`。

### 5.21 小地图三条

**0x0303（12 B，选手端 → 机器人）**：

```c
typedef __packed struct {
    float    target_position_x;  // 0  发目标 ID 时填 0
    float    target_position_y;  // 4  发目标 ID 时填 0
    uint8_t  cmd_keyboard;       // 8  云台手按下的通用键值，无按键为 0
    uint8_t  target_robot_id;    // 9  发坐标时填 0
    uint16_t cmd_source;         // 10 信息来源 ID
} map_command_t;
```

发送规则：云台手间隔 ≥0.5 s；半自动机器人操作手间隔 ≥3 s。服务器收到后会**额外补发 4 次（每 100 ms 一次，共 5 次）**，之后以 1 Hz 持续发最近一包，两套计时相互独立——所以机器人端会收到大量重复包，**必须用 `cmd_keyboard`/`target_robot_id` 的变化或自己去重**，不能每收到一包就重新规划一次。

**0x0305（48 B，雷达 → 选手端）**：`uint16_t[24]`，对方 hero/engineer/infantry3/infantry4/aerial/sentry + 己方同 6 项，单位 cm。x=y=0 视为未发送。

**0x0307（105 B，哨兵/半自动 → 选手端）**：

```c
typedef __packed struct {
    uint8_t  intention;         // 0  1 到目标点攻击 / 2 防守 / 3 移动到目标点
    uint16_t start_position_x;  // 1  dm，小地图左下角为原点
    uint16_t start_position_y;  // 3
    int8_t   delta_x[49];       // 5  相对上一点的增量
    int8_t   delta_y[49];       // 54
    uint16_t sender_id;         // 103
} map_data_t;
```

**0x0308（34 B，机器人 → 选手端）**：`sender_id`、`receiver_id`、`uint8_t user_data[30]`（**utf-16 编码**，30 字节 = 15 个字符，注意大小端）。

### 5.22 图传链路四条

| cmd_id | 结构 | 方向 | 语义 |
|---|---|---|---|
| 0x0302 | `{uint8_t data[30]}` | 自定义控制器 → 机器人 | 纯自定义，无固定字段 |
| 0x0309 | `{uint8_t data[30]}` | 机器人 → 自定义控制器 | 纯自定义 |
| 0x0310 | `{uint8_t data[300]}` | 机器人 → 自定义客户端 | **2026 新增**，无重传 |
| 0x0311 | `{uint8_t data[30]}` | 自定义客户端 → 机器人 | **2026 新增** |

0x0302 和 0x0309 是**对称的一对**：控制器把摇杆/按键/机械臂目标塞进 30 字节发上去，机器人把云台姿态/血量/状态塞回来。字段格式完全由队伍自己约定，所以**上下位机必须共享同一份结构体定义**。

0x0310 的 300 字节意味着整帧 309 字节，**接收缓冲和帧长上限都要按 309 设计**。

### 5.23 模拟键鼠 0x0306（8 B，非链路）

```c
typedef __packed struct {
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right: 4;
    uint16_t reserved;
} custom_client_data_t;
```

坐标是**绝对像素**（1920×1080），不是增量。这个命令码走"非链路"——自定义控制器直接接到选手端/电脑上，不经过裁判系统。

### 5.24 雷达无线链路 0x0A01~0x0A06

| cmd_id | 结构要点 |
|---|---|
| 0x0A01 | 对方 6 台位置，12 × `uint16_t`（cm） |
| 0x0A02 | 对方血量：1/2/3/4 号、保留、7 号 |
| 0x0A03 | 对方剩余弹量：1/3/4 号、6 号空中、7 号 |
| 0x0A04 | 对方金币总数 + 4 字节宏观状态位图 |
| 0x0A05 | 41 B：hero/engineer/3/4/5 号各 7 B 增益（recovery u8、cooling u16、defence u8、vulnerability u8、attack u16）+ offset35 哨兵姿态 + offset36-40 五台机器人的存活状态（0 存活 / 1 战亡 / 2 无敌不虚弱 / 3 无敌且虚弱） |
| 0x0A06 | 对方干扰波密钥，6 B |

这条链路**不是串口**，是从空气里收电磁波后自己解析的，跟 UART 代码无关。

---

## 6. VT03/VT13 遥控帧（21 字节）

### 6.1 帧结构

图传发送端的 UART（GND/RX/TX 三针）在**与接收机对频成功后**，每 **14 ms** 输出一帧 21 字节，**921600 8N1 无流控**。

| bit 区间 | 宽度 | 内容 |
|---|---|---|
| 0-7 | 8 | Header1 = `0xA9` |
| 8-15 | 8 | Header2 = `0x53` |
| 16-26 | 11 | 通道 0（右摇杆横）364/1024/1684 |
| 27-37 | 11 | 通道 1（右摇杆纵） |
| 38-48 | 11 | 通道 2（左摇杆纵） |
| 49-59 | 11 | 通道 3（左摇杆横） |
| 60-61 | 2 | 模式开关：0=C，1=N，2=S |
| 62 | 1 | 暂停键 |
| 63 | 1 | 左自定义键 |
| 64 | 1 | 右自定义键 |
| 65-75 | 11 | 拨轮 |
| 76 | 1 | 扳机 |
| 77-79 | 3 | 保留 |
| 80-95 | 16 | 鼠标 X（`int16_t`，速度量） |
| 96-111 | 16 | 鼠标 Y |
| 112-127 | 16 | 鼠标 Z |
| 128-129 | 2 | 鼠标左键 |
| 130-131 | 2 | 鼠标右键 |
| 132-133 | 2 | 鼠标中键 |
| 134-135 | 2 | 保留 |
| 136-151 | 16 | 键盘位图：W/S/A/D/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B |
| 152-167 | 16 | CRC16 |

21 字节 = 168 bit，正好填满。

### 6.2 CRC-16/CCITT-FALSE

```c
/* poly 0x1021，init 0xFFFF，MSB first，不反射，不异或 */
static uint16_t vt03_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
/* 校验：对前 19 字节算，结果与 buf[19] | buf[20]<<8 比较 */
```

**不要复用裁判系统的 `get_CRC16_check_sum()`**——那是 0x8408 反射表，算出来完全不同。

### 6.3 与 DBUS(18 B) 的差异

| | DBUS / SBUS（DR16） | VT03 21 字节 |
|---|---|---|
| 串口 | 100000，**8E1**（偶校验），反相 | 921600，8N1 |
| 帧头 | `0x0F` 起始 + `0x00` | `0xA9 0x53` |
| 长度 | 18（固定） | 21（固定） |
| 通道 | ch0-ch3，11 bit，364-1684 | 同左，但位置不同 |
| 鼠标/键盘 | 有（4 字节鼠标 + 2 字节键盘） | 有，位宽不同（鼠标 16 位、键盘 16 位） |
| 校验 | 无 | CRC-16/CCITT-FALSE |
| 周期 | 14 ms | 14 ms |

结论：**2026 的 VT03 是另一套物理层和帧格式，不能沿用 `remote_control.c` 的 `sbus_to_rc()`**。

---

## 7. 本项目实现对照

### 7.1 四条链路 ↔ 代码

| 链路 | 文件 | 关键位置 |
|---|---|---|
| 裁判系统接收 | `application/referee/referee_usart_task.c` | `referee_usart_task()`:63、状态机 `referee_unpack_fifo_data()`:88、`USART6_IRQHandler()`:186 |
| 裁判系统解析 | `application/referee/referee.c` | `referee_data_solve()`:135 起逐 cmd_id memcpy |
| 协议常量 | `application/referee/protocol.h` | `HEADER_SOF`、`REF_PROTOCOL_FRAME_MAX_SIZE`、cmd_id 枚举 |
| 结构体 | `application/referee/referee.h` | 全部 `__packed` 结构体 |
| 0x0302 上行 | `application/custom_controller/custom_terminal.c` | 帧定义:477、`CustomControllerSendToPC()`:502 |
| 板间/上位机帧 | `application/communication/communication.c` | `SendRC()`:62、`DataUnpack()` |
| 遥控器 | `application/other/remote_control.c` | `sbus_to_rc()`:217 |
| CRC | `components/support/CRC8_CRC16.c` | `get_CRC16_check_sum()`:166、`verify_CRC16_check_sum()`:194 |

### 7.2 板子角色

`robot_param.h:30` 选了 `robot_param_omni_mini.h`，其中 `:12` 定义 `BOARD_CURRENT = C_BOARD_DOWN`，于是 `:16` 分支把 `CHASSIS/GIMBAL/SHOOT_TYPE` 全设成 `NONE`，`:47` 是 `CUSTOM_CONTROLLER_TYPE = CUSTOM_CONTROLLER_TERMINAL`。所以这台机器是**C 板下板 + 六轴终端**，`robot_param_terminal.h:35` 提供机构参数（CAN 口、关节数、零位、减速比、连杆质量/质心）。

### 7.3 0x0302 上行帧的实际布局（39 B）

```
A5 | 1E 00 | seq | CRC8 | 02 03 | [7 × float 关节角] + 2B 0 | CRC16
```

- `data_length = 30`（`CUSTOM_TERMINAL_UPLINK_DATA_LEN`）
- CRC8 覆盖前 5 字节、CRC16 覆盖前 37 字节，与协议一致 ✅
- 30 Hz（`CUSTOM_CONTROLLER_SEND_TIME = 34` ms）符合频率上限 ✅
- 同时往 `huart1` 和 `huart6` 各发一份，两路独立 seq

---

## 8. 已知问题清单

### P0（会越界/崩）

| # | 位置 | 问题 |
|---|---|---|
| 1 | `referee.c:163` | `case GAME_RESULT_CMD_ID` 缺 `break`，穿透到 0x0003 的 32 B memcpy，越界读 31 字节 |
| 2 | `referee.c:250 / 256 / 271` | 三处用 `sizeof(结构体)` 做 memcpy，不校验实际 `data_length` |
| 3 | `protocol.h:7` | `REF_PROTOCOL_FRAME_MAX_SIZE = 128`，`referee_usart_task.c:124` 因此丢弃 `data_len ≥ 119` 的帧，0x0310（300 B）永远收不到 |

### P1（与 2026 协议不符，解析错位）

| # | 结构体 | 项目 vs 2026 |
|---|---|---|
| 4 | `game_robot_HP_t` | 32 B → 20 B |
| 5 | `robot_status_t` | 13 B → 17 B |
| 6 | `power_heat_data_t` | 16 B → 14 B |
| 7 | `buff_t` | 6 B → 8 B（cooling u8→u16） |
| 8 | `projectile_allowance_t` | 6 B → 8 B |
| 9 | `rfid_status_t` | 4 B → 5 B |
| 10 | `radar_mark_data_t` | u8 → u16 |
| 11 | `sentry_info_t` | 6 B → 14 B |
| 12 | `map_robot_data_t` | 24 B → 48 B |
| 13 | `robot_interaction_data_t` | user_data 122 → 112；子内容 0x0121 1 B → 8 B |
| 14 | `robot_id_t` | `RED_OUTPOST=10 / RED_BASE=11` 与 2026 相反 |
| 15 | `remote_control_t`（0x0304） | 2026 已删除该命令码 |

### P2（功能缺失）

| # | 缺什么 |
|---|---|
| 16 | 0x0310 / 0x0311 自定义客户端链路 |
| 17 | 0x0309 接收（本板目前只发 0x0302，主控回传的数据收不到） |
| 18 | VT03 21 字节遥控帧解析（工程里搜不到 `0xA9`/`0x53`，用的还是 18 B DBUS） |
| 19 | 0x0A01~0x0A06 雷达无线链路 |

### P3（逻辑错误）

| # | 位置 | 问题 |
|---|---|---|
| 20 | `referee.c:411` | `get_is_center_buff_down()` 把 32 位事件字当布尔，任何一位非 0 都误判 |
| 21 | `referee.c:381` | `get_all_robot_hp_mode()` 一半分支读 2026 不存在的 `blue_*` |
| 22 | `referee.h` 注释 | 0x0303 写 15、0x0307 写 103、0x0305 写 24，全部与 2026 表 1-5 的旧值一致但与结构体不符 |

---

## 9. 实测自检清单

1. **抓一帧 0x0001**：`A5 0B 00 xx CRC8 01 00 ...`，确认 `data_length = 11`、整帧 20 字节。
2. **验证 CRC8 覆盖范围**：只算前 4 字节，与 `buf[4]` 比对。
3. **用 `data_length` 反查表 1-5 的矛盾**：抓 0x0203 / 0x0303 / 0x0307，看真实 `data_length` 是 12/12/105 还是 16/15/103——这是唯一可靠的裁决方式。
4. **0x0201 长度自检**：17 B 才对；如果收到 13 B 说明是旧协议或你截断了。
5. **图传链路通断**：自定义控制器每 34 ms 发 0x0302，用串口助手在 huart1 上确认 39 B 一帧、`cmd_id = 0x0302`。
6. **VT03 对频**：未对频时 UART **无任何输出**，先确认图传灯状态再看波形。
7. **VT03 CRC 交叉验证**：抓一帧，用 CCITT-FALSE（0x1021）算前 19 字节，应该等于 `buf[19] | buf[20]<<8`；若用 0x8408 算不出来是对的。

---

## 附：术语与 ID 速查

**机器人 ID**：1 英雄 / 2 工程 / 3-5 步兵 / 6 空中 / 7 哨兵 / 8 飞镖 / 9 雷达 / **10 基地 / 11 前哨站**，蓝方 +100（101-111）。

**选手端 ID**：红方 0x0101-0x0106，蓝方 0x0165-0x016A，服务器 0x8080。

**易伤/无敌**：雷达触发的双倍易伤对对方生效；己方基地/前哨站被攻击会进"无敌且虚弱"状态。
