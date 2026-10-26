# `controler_myself.c/h` 代码评审

> 基准实现：`D:\RM\new_remoto\2026自定义控制器\2026自定义控制器\代码\Custom_Controller - v1.2`（下称 **v1.2**）
> 被评审：`application/gimbal/controler_myself.c`（690 行）/ `controler_myself.h`（233 行）
> 评审日期：2026-10-25

---

## 0. 总体结论

现在的 `controler_myself.c` 是一个**能算正解、但还不能驱动机器人**的骨架：

| 能力 | v1.2 | 现在 |
|---|---|---|
| 工程可编译 | 是 | **否**（未加入 uvprojx，且被 `communication.c` 硬引用） |
| 关节位置回读 | 每周期读 `S_CPOS`（多圈、角度制） | 只在初始化读一次，单圈 16 位 ecd |
| 速度估计 | 差分 + 低通 | 脉冲式（绝大多数周期为 0） |
| 正解 | 有（标准 modified-DH，米） | 有（自定义 axis 模型，毫米） |
| 逆解 | PUMA 解析解，8 组择优 | 无 |
| 重力补偿 | `Gravity_comp.c`，DH + 质心 + 质量 | 无 |
| 关节阻抗 | `joint_impedance.c`（kp/kd/限幅/限速率/减速比/kt） | 无 |
| 力矩输出 | N·m → 电机电流，带 `Limite_tauqe()` 与限速率 | 直接丢裸整数，`datouctrl[]` 全 0 |
| 安全 | `Safewarning.c` + 离线检测 + 力矩爬升 | 无（`joint_feedback_valid` 算了但没用） |

**最该先处理的 4 件事**

1. **工程链接不过**：`controler_myself.c` 不在 `StandardRobotPro.uvprojx` 里，而 `communication.c:273` 无条件调用它里面的 `GetGimbalJointFeedbackPoint()`（`communication.c:31` 直接 include 了它的头文件）→ 必然 L6218E。同理 `CAN_cmd_robostride.c` 也不在工程里，而 `CAN_receive.c:263/267` 在调 `RobostrideFdbData()`。
2. **`controler_myself.c:577-578` 越界读**：`measure[i-1]`，i=0 时读 `CAN1_ZDATOU_MEASURE[-1]`。
3. **位置只读一次**（`GimbalInit` 里 6 次 `zdatou_ecd_get` 之后再不读）→ FK 输出冻结；速度是"来一帧跳一次、其余全 0"的脉冲，任何 kd 阻尼都会被废掉。
4. **没有重力补偿**：6 轴臂的自重全靠关节电机硬扛，这就是 v1.2 里 `only_gravity 1` 的意义。这一项不做，后面调什么参数都是在给重力擦屁股。

---

## 1. 工程接线（P0，阻塞项）

### 1.1 现状

- `MDK-ARM/StandardRobotPro.uvprojx` 只有一个 target，`<FileName>` 列表里**没有** `controler_myself.c`，**也没有** `CAN_cmd_robostride.c`；但 `CAN_cmd_zdatou.c`、`custom_controller_*.c` 在。
- 当前生效参数：`robot_param.h:30` → `robot_param_omni_mini.h`，其中 `BOARD_CURRENT = C_BOARD_DOWN`（第 12 行）→ `CHASSIS_NONE / SHOOT_NONE`，第 18 行甚至写成 `#define GIMBAL_TYPE CHASSIS_NONE`（拿底盘枚举赋给云台，只是碰巧两者 `NONE == 0` 才没炸），第 46/47 行 `MECHANICAL_ARM_NONE / CUSTOM_CONTROLLER_NONE`。
  → **当前提交是一台"什么模块都没开"的空板固件**。
- `controler_myself.c` **全文没有 `#if (GIMBAL_TYPE == ...)` 守卫**（grep 无匹配），它无条件定义 `GimbalInit/GimbalObserver/GimbalSendCmd/GimbalConsole/...` 以及 `GetGimbalDeltaYawMid / CmdGimbalJointState`。这些名字在 `gimbal_task.c` 里是 `__weak`，本身没问题；但 `gimbal_virtual.c` 在 `#if (GIMBAL_TYPE == GIMBAL_NONE)` 下也定义了同名的两个 getter → **无守卫地把 `controler_myself.c` 拖进工程，会和 `gimbal_virtual.c` 撞名**。这才是它被排除在工程外的真正原因。
- `gimbal.h:27-33` 用 `extern inline` 声明 getter，而 `controler_myself.c` 用裸 `inline` 定义（`GetGimbalDeltaYawMid / GetGimbalInitJudgeReturn / CmdGimbalJointState`）。ARMCC 下 `inline` 不保证生成外部符号，这是典型的"换个编译器就 L6218E"的写法，建议统一改成 `static inline`（仅供本文件）或普通函数（对外）。
- `robot_typedef.h:13-14` 只有 `GIMBAL_NONE 0` / `GIMBAL_YAW_PITCH_DIRECT 1`，**没有机械臂这一档**。

### 1.2 建议的接线改法（5 步）

```c
// 1) application/typedef/robot_typedef.h
#define GIMBAL_CONTROLLER_MYSELF  2   // 自定义控制器 6 轴机械臂

// 2) controler_myself.c 整体包一层
#if (GIMBAL_TYPE == GIMBAL_CONTROLLER_MYSELF)
... 原有全部内容 ...
#endif

// 3) controler_myself.h 同理（防止别人 include 到不该有的声明）
#if (GIMBAL_TYPE == GIMBAL_CONTROLLER_MYSELF)

// 4) robot_param_omni_mini.h：修掉第 18 行的枚举错误，并给机械臂板加一档
#define GIMBAL_TYPE CHASSIS_NONE          -->  #define GIMBAL_TYPE GIMBAL_NONE
// 机械臂板（例如 C_BOARD_ARM / 复用 C_BOARD_DOWN）：
#define GIMBAL_TYPE GIMBAL_CONTROLLER_MYSELF
```

5) `communication.c` 的硬引用必须解耦。两种做法，推荐第二种：

```c
// 做法 A（最小改动）：communication.c:31 和 :273 都加守卫
#if (GIMBAL_TYPE == GIMBAL_CONTROLLER_MYSELF)
#include "controler_myself.h"
#endif
...
#if (GIMBAL_TYPE == GIMBAL_CONTROLLER_MYSELF)
    const float *joint_feedback = GetGimbalJointFeedbackPoint();
#else
    static const float joint_feedback_zero[6] = {0};
    const float *joint_feedback = joint_feedback_zero;
#endif

// 做法 B（推荐）：把它提升为框架级 API —— 在 gimbal_task.c 里加
__weak const float *GetGimbalJointFeedbackPoint(void)
{
    static const float zero[6] = {0};
    return zero;
}
// 这样 communication.c 永远可链接，任何 GIMBAL_TYPE 都不会炸。
```

> 补充：`communication.c:57` 的 `CCControlData_t ControlData;` 与 `custom_controller.h:19` 的同名 typedef 重复定义。一旦某个 TU 同时 include 两个头，就是重定义错误。建议删掉 `communication.h:36-39`，统一用 `custom_controller.h` 里的。

---

## 2. `controler_myself.c` 具体缺陷

### 2.1 功能性（会导致数据错误 / 越界）

| # | 位置 | 问题 | 后果 |
|---|---|---|---|
| A1 | `:577-578` | `measure[i-1]`，应为 `measure[i]` | i=0 越界读 `CAN1_ZDATOU_MEASURE[-1]`；i>0 拿到的是**前一个电机**的时间戳 → `valid/时间戳` 全部错位 |
| A2 | `:523` | 位置回读只在 `GimbalInit` 里发 6 次 `zdatou_ecd_get(1, 0x100+i*0x100)`，主循环再也不读 | 若 `11 18 31 00 0a 6b` 是"读一次"，`ecd` 永远停在初始化值 → FK 输出恒定、速度恒 0。**只有在那一帧同时把电机配成 10 ms 周期自动上报时才成立，请在驱动手册/示波器上确认** |
| A3 | `:556` | `WrapAngle(ecd/65535*2π - offset)`，单圈 + `fmodf` 归一化到 `[-π, π)` | 关节 4/5/6（腕）和关节 2 的实际行程超过 ±π，**多圈信息被丢掉**，FK 在过 ±π 时跳变；后续做 IK/阻抗必然崩 |
| A4 | `:556` | 没有除以减速比 | 工程内另一个 6 关节实现 `custom_controller_engineer.c:183-185` 的约定是 `pos = (pos_fdb + 2π*round) / reduction_ratio`。若 SB 大头也是"编码器在电机端"的关节电机，**这里角度整整差一个减速比** |
| A5 | `:562` | `float delta = joint_feedback[i] - joint_feedback[i];` 恒为 0，下一行立刻被覆盖 | 死语句（历史遗留），读代码的人会误以为速度由角度差分 |
| A6 | `:559-566` | 速度用 `ecd` 差分量、时间用 `last_fdb_time` 差分量，且 `dt_ms > 0` 才更新 | 没来新帧的周期 `delta == 0` → 速度输出 0；来帧的那一拍输出一个大尖峰 → **速度是脉冲串**，做 kd 会变成"每 10 ms 踹一脚" |

A2/A6 的修法：

```c
/* 在 GimbalObserver 里主动轮询（若确认电机不会自动上报） */
static uint32_t last_poll_ms;
if ((now - last_poll_ms) >= 2U) {
    last_poll_ms = now;
    for (uint8_t k = 0; k < 6U; k++) {
        zdatou_ecd_get(1, ZDATOU1_EXT_ID + (uint32_t)k * 0x100U);
    }
}

/* 速度：只在"码值真的变了"时更新，并用有符号 16 位回绕差分 */
if (ecd != joint_last_ecd[i]) {
    int32_t d = (int32_t)(uint16_t)(ecd - joint_last_ecd[i]);   /* 无符号相减天然处理回绕 */
    if (d >  32767) d -= 65536;
    if (d < -32768) d += 65536;
    float dt_s = (float)(now - joint_last_move_ms[i]) * 0.001f;
    if (dt_s > 0.0f) {
        float dq = (float)d * (2.0f * PI / 65536.0f) / dt_s;      /* 再 / reduction_ratio */
        joint_velocity[i] += 0.9f * (dq - joint_velocity[i]);     /* 对齐 v1.2 vel_lpf_alpha */
    }
    joint_last_move_ms[i] = now;
}
joint_last_ecd[i] = ecd;
```

### 2.2 运动学建模（这是最难"打补丁"的一层）

现有 `arm_dh_param[6]` 全部 `alpha = 0`、`d = 0`，把机构信息塞进了自造的 `axis[]`（J2 `{0,-1,0}`、J3 `{0,1,0}`、J4/J6 `{1,0,0}`、J5 `{0,0,-1}`），而 `DhTransform()` 实际算的是

```
T = R_axis(θ) · T_fixed(Rx(α), Tx(a)·Tz(d))
```

- 头文件注释写的是标准 DH `T = Rz(θ)·Tz(d)·Tx(a)·Rx(α)`，**代码与注释不一致**：标准 DH 里关节旋转一定绕 **z**，这里绕任意轴。
- 于是 `T` 的第三列（子坐标系 z 轴）**不再等于关节旋转轴**。而 v1.2 的 `gravity_compensation()` 正是用 `z[j] = T0_j 的第三列` 建雅可比 —— **重力补偿无法直接搬到这张表上**。
- 单位是**毫米**（J2 `a=190`、J3 `a=87`、J5 `a=150.5`），v1.2 全程用**米**（`GRAVITY 9.81`）。混用迟早出事，建议现在统一成米。
- `theta_offset` 是截断的魔法数：`-3.14`（不是 `-M_PI`）、`-2.64`、`-0.62`、`0.55`、`0.2`。0.0016 rad ≈ 0.09° 的常值偏差，且**被算进了 `joint_feedback`**，于是 `communication.c:276` 发给上位机的"关节角"其实是 DH 角而不是机械角。v1.2 严格区分原始 `theta` 与 `theta_dh`（`Get_theta()` / `dh_to_original()` 互为逆），建议照抄这个分层。

**建议**：不要在这张表上继续修。直接换回 v1.2 的 modified-DH（米）：

```c
static const float DH_a[6]     = {0.0f, 0.0f, 0.130f, 0.0f,    0.0f,   0.0f};
static const float DH_alpha[6] = {0.0f, -M_PI_2, 0.0f, -M_PI_2, M_PI_2, -M_PI_2};
static const float DH_d[6]     = {0.0f, 0.042f, 0.0f, 0.115f,  0.0f,   0.0f};
/* T_i = Rz(θ+offset)·Tz(d)·Tx(a)·Rx(α)，行主序 4x4 */
```

并把 `DhTransform()` 改成真正的 `Rz·Tz·Tx·Rx`。这样 `Gravity_comp.c` / `kinematics.c` 可以**原样移植**，不用改写数学。

### 2.3 控制输出与安全

| # | 位置 | 问题 |
|---|---|---|
| C1 | `:612-648` | `GimbalConsole()` **整段被注释**，`datouctrl[] / robostride_datouctrl` 从未被写入 → 机械臂收到的永远是 0 力矩。文件里还躺着 36 行死公式（`a1[]/a2[]` 只被它引用） |
| C2 | `:658-688` | `sendctrl` 3 态轮转，1 ms 周期里每个 tick 只发 2 帧 → 每个电机 **3 ms 才更新一次（333 Hz）且间隔不均匀**。v1.2 每个周期把 6 个关节都发一遍 |
| C3 | `:663` 等 | `zdatou_torque_control(..., 0x01, torque)` 的 `direction` 实参是**无效的**——驱动内部 `if (torque<0) { torque=-torque; direction=0x00; }` 会按符号覆写。传 0x01 纯属误导 |
| C4 | 全局 | **没有任何力矩限幅/量纲换算**：`datouctrl[i]` 直接进驱动，驱动只用低 16 位（`torque>>8, torque`），超过 65535 静默回绕。头文件里的 `CONTROLLER_MYSELF_JOINT_TORQUE_MAX 8000` **从未被使用**。v1.2 有 `TAU_MAP_PARAM1..6` 换算 + `Limite_tauqe()` + 限速率 |
| C5 | 全局 | 没有 `GimbalHandleException()` 覆写；`joint_feedback_valid[i]` 算了但从不用来禁止输出；电机掉线后仍会按陈旧角度继续出力。v1.2 有 `Safewarning.c` 与 `HasPrev` 保护 |
| C6 | `:513-526` | `Robostride_enable(1, 0x5)` 之后**立刻**开始查询，没有让能/建立通信的延时；`get_remote_control_point()` 返回值未判空 |
| C7 | 全局 | 使能瞬间直接从 0 跳到目标力矩，没有软启动/爬升 |

C4 的最小修法（先不管物理量纲，至少别回绕）。顺带一个线索：被注释掉的旧代码 `controler_myself.c:638` 写过 `a1[3] = (double)datouctrl[2] / 8000;`，说明**大头电机的力矩原始量满量程就是 8000**，正好和头文件里没用上的 `CONTROLLER_MYSELF_JOINT_TORQUE_MAX 8000` 对上——量纲标定可以从这里入手。

```c
#define CM_TAU_LSB_PER_NM   1000.0f      /* 先按实测标定，和 v1.2 的 TAU_MAP_PARAM 对齐 */

static int32_t cm_tau_to_raw(float tau_nm, int32_t i)
{
    float raw = tau_nm * CM_TAU_LSB_PER_NM;
    if (raw >  (float)CM_TAU_RAW_MAX[i]) raw =  (float)CM_TAU_RAW_MAX[i];
    if (raw < -(float)CM_TAU_RAW_MAX[i]) raw = -(float)CM_TAU_RAW_MAX[i];
    return (int32_t)raw;
}
```

### 2.4 工程卫生

- **死代码/死变量**：`ControllerMyselfInitJudge()`、`a1[6]`、`a2[6]`（只被 `GimbalConsole` 里被注释的旧推导引用，`:612-640`，那套 `sin/atan2f` 公式是早期手工几何解）、`zdatouMeasure`（未初始化指针）、`controller_myself_pid`、`Gimbal_s` 里约 60% 的字段（`mode/last_mode/mode_before_rc_err`、`Motor_s yaw/pitch`、`reference/feedback_pos/feedback_vel/upper_limit/lower_limit`、`pid`、`pitch/yaw_angle_zero_for_imu`、`init_start_time/init_timer/init_continue`）。建议把 `Gimbal_s` 瘦身成 `ArmState_t`，别继承云台模板。
- **命名**：一个 6 轴机械臂却叫 `Gimbal*`，`CONTROLLER_MYSELF_JOINT_KP/KD/TARGET_STEP` 这类可调参数放在 **header** 里，而本项目约定是放 `robot_param_*.h`。
- **头文件**：`controler_myself.h` 一口气 include 了 15 个头（`IMU.h / pid.h / user_lib.h / usb_debug.h / CAN_cmd_dji.h / supervisory_computer_cmd.h / macro_typedef.h` 都不需要）。
- **未使用的 API**：`PoseAttitudeError / FK_Solve / GetJointState / GetJointAngle / GetJointVelocity` 无人调用；`FK_Solve` 与 `FK_SolveAll` 末项重复。
- **性能**：每 1 ms 对 6 个关节重新归一化 `dh->axis`（6 次 `sqrtf`）、`WrapAngle` 用 `fmodf`、外加每关节一次 `Mat4Mul + memcpy`。F407 扛得住，但换算成"预计算 `axis` 归一化常数 + 指针乒乓"是免费的优化。
- `zdatouFdbData()` 的接收判据 `if (rx_data[0]==0x31 || rx_data[3]==0x6b)` 用了 `||`，**任何**带 0x6b 尾的帧都会把 `rx_data[1..2]` 当码值写入 → 建议改 `&&` 并校验长度。
- `CAN_receive.c:329-338`：CAN2 分支的 `static uint8_t i = 0;` 从不更新，`b1` 算了不用 → CAN2 的 6 个大头永远写进 `[0]`。

---

## 3. 与 v1.2 的能力差距 / 复用清单

| v1.2 文件 | 作用 | 现在 | 建议 |
|---|---|---|---|
| `APP/kinematics.c/h` | modified-DH 正解 + PUMA 解析逆解（8 解择优） | 有正解（自定义 axis 模型） | **直接移植**，替换 `DhTransform`；纯数学无依赖 |
| `APP/Gravity_comp.c/h` | DH + 连杆质心/质量 → 重力力矩 | 无 | **直接移植**（需先统一到米 + 标准 DH） |
| `APP/joint_impedance.c/h` | 单关节阻抗（kp/kd/限幅/限速率/减速比/kt/角度归一） | 无 | **直接移植**，`JointImpConfig_t` 逐字段对应 |
| `APP/servo_mapping.c` | 模式机、键位触发、目标 LPF、`calc_send_torque` | 只有空 `GimbalConsole` | 只搬"结构"（状态机 + LPF + 限幅），电机层用 SB/Robostride |
| `APP/Safewarning.c/h` | 安全警告/降级 | 无 | 移植或至少实现离线检测 + 力矩爬升 |
| `Devices/ZDT_X42_V2.c/h` | ZDT 电机多圈绝对位置 `S_CPOS` | SB 只有单圈 ecd | 若要多圈，必须让电机回报多圈位置（或自己累计圈数） |
| `bsp_fdcan.c/h`、`usart_manager.c`、`crc.c/h` | 底层驱动 | 已有等价物（HAL CAN + `CRC8_CRC16.c` + `bsp_usart`） | 不需要 |

**最大的结构性差距不是缺文件，而是缺"层"**：v1.2 是
`原始电机量 → theta(DH 角) → 重力/阻抗(N·m) → 电机电流指令`，
每个转换都有明确单位和边界钳位；现在的 `controler_myself.c` 把这四层混在一个 1 ms 函数里，中间量还是裸整数。

---

## 4. 建议的实施顺序

### P0（半天，先让它能跑起来）
1. 加 `GIMBAL_CONTROLLER_MYSELF` 枚举值；`controler_myself.c/h` 整体加 `#if` 守卫。
2. `robot_param_omni_mini.h:18` 修 `GIMBAL_TYPE CHASSIS_NONE` → `GIMBAL_NONE`，并给机械臂板设 `GIMBAL_CONTROLLER_MYSELF`。
3. uvprojx 加入 `controler_myself.c` + `CAN_cmd_robostride.c`；确认链接通过。
4. `communication.c` 的 `GetGimbalJointFeedbackPoint()` 改成弱定义兜底（1.2 节做法 B）。
5. 修 A1（`measure[i-1]` → `measure[i]`）、删 A5、`zdatouFdbData` 的 `||`→`&&`。
   **验收**：编译链接 0 error；示波器/CAN 分析仪看到 6 个电机都被周期性查询且 ecd 在动。

### P1（1 天，位置层重构）
6. 统一到**米 + 标准 modified-DH**，`DhTransform` 改 `Rz·Tz·Tx·Rx`。
7. `theta` 与 `theta_dh` 分离，`theta_offset` 用 `-M_PI` 等精确常量，放到 `robot_param_*.h`。
8. 多圈处理：优先让电机回报多圈位置；否则在固件里累计圈数（参考 `custom_controller_engineer.c` 的 `round` 做法）。
9. 速度估计按 2.1 节改写 + 一阶低通；`joint_feedback_valid` 真正参与输出门控。
   **验收**：手掰关节，串口打印 `theta` 连续、`dq` 平滑无尖峰、过 ±π 不跳变。

### P2（1–2 天，重力补偿）
10. 移植 `Gravity_comp.c`，填**真实**连杆质量/质心（`link_mass`、`link_com` 必须实测，v1.2 的数值只对应它自己的臂）。
11. 只做重力前馈：`tau_cmd = tau_gravity`，kp = kd = 0。
    **验收**：整臂松手后能大致悬停，`tau_gravity` 各关节在 5% 以内能托住静摩擦。

### P3（2–3 天，阻抗 + 位置模式）
12. 移植 `joint_impedance.c`，逐关节给 `kp/kd/torque_limit/torque_rate_limit/gear_ratio/kt`。
13. `GimbalConsole()` 重写成真正的控制律（别再交空函数）；`GimbalSendCmd` 改为每周期发全部 6 个电机。
14. 力矩单位换算 + 限幅 + 限速率 + 使能软启动（C3/C4/C7）。
    **验收**：单关节阶跃响应无振荡；限幅生效（打满不超流）。

### P4（1–2 天，位置/位姿模式 + 安全）
15. 移植 `kinematics.c`，按键触发 `tool_relative_transform(T_cur, T_rel, T_goal)` + `compute_desired_joint_angles()`；IK 失败回退 `Hold_Current_Pose()`。
16. 目标角 LPF（v1.2 `FILTER_ALPHA 0.2` + `nearest_equiv_local` 解缠）+ `MIT_POS_STEP_MAX` 步长限制。
17. 安全层：离线检测、超限检测、异常时力矩清零并锁定。
    **验收**：末端画圆/直线跟踪误差可测；拔掉任一电机 CAN 线，1 s 内停止出力。

---

## 5. 一句话总结

`controler_myself.c` 的正解数学写得干净（`Mat4Mul` 无别名、四元数半球归一、姿态误差实现都对），但**它现在只是一个"能算出末端在哪"的观测器**：没进工程、位置读一次、速度是脉冲、输出恒 0、无重力补偿。

对照 v1.2，缺的不是某几个函数，而是 **"物理量分层 + 单位约定 + 限幅/限速率 + 安全降级"** 这套骨架。建议按 §4 的 P0→P2 顺序推进：**先把工程接线和位置层做对，再谈重力和阻抗**——否则后面所有参数整定都是在补偿前面这几层的错误。
