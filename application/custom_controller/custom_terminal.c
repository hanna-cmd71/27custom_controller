/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_terminal.c
  * @brief      自定义终端（J0~J5 六轴机械臂）控制实现。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-26-2025     李喆            1. 完成框架搭建。
  *  V1.1.0     Oct-26-2026     hanner             1. 与模块解耦；
  *                                                2. 反馈统一成"机械角 → 零位 → 减速比"；
  *                                                3. 补重力补偿与零位标定入口。
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#include "custom_terminal.h"
#include "terminal_gravity.h"
#include "CAN_cmd_zdatou.h"
#include "CAN_cmd_robostride.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>


/* 上行数据：Observer 每 1 ms 填 pos[0..5]，SendToPC 每 34 ms 打包发出。
 * pos[6] 为预留位，暂无物理意义，保持 0。 */
CCControlData_t cc_control_data = {
    .pos = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

/* 本工程只服务自定义终端这一种配置，已取消 CUSTOM_CONTROLLER_TYPE 变体分支：
 * 本文件无条件编译，其它配置不再有"空实现"这条退路。 */


/* 每个关节的驱动地址：0 = 该关节不是 ZDT 电机（走 robostride） */
static const uint8_t ct_zd_addr[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_ZD_ADDR;
/* 电机正方向定义：+1 = 电机正转时关节角增大 */
static const int8_t ct_dir[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_JOINT_DIR;
/* 减速比：关节角 = 电机侧机械角 / 减速比 */
static const float ct_gear[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_JOINT_GEAR;
/* 零位：电机侧机械角 (rad)，标定后回填 */
static const float ct_zero[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_JOINT_ZERO;
/* ZDT 力矩换算：mA per N·m（手册相电流满量程 5000 mA）；RS00 用 N·m 直接下发 */
static const float ct_ma_per_nm[CUSTOM_TERMINAL_JOINT_NUM] = CUSTOM_TERMINAL_JOINT_MA_PER_NM;

/*-------------------- Global var --------------------*/

custom_terminal_t custom_terminal;
FK_Result_t custom_terminal_fk;                     
FK_Result_t joint_pose[CUSTOM_TERMINAL_JOINT_NUM];  // 每个关节坐标系相对基座的位姿

/* 发帧游标：每 1 ms 只发 2 帧，3 拍转完一圈，单台电机 3 ms 收一帧 */
static uint8_t ct_send_phase;

/*-------------------------------- Internal functions ---------------------------------------*/

/**
 * @brief 把力矩限幅到 ±limit
 */
static float clamp_torque(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/**
 * @brief 限速逼近：每拍最多变化 ±step
 */
static float step_toward(float value, float target, float step)
{
    if (target > (value + step))
    {
        return value + step;
    }
    if (target < (value - step))
    {
        return value - step;
    }

    return target;
}

/*------------------------- The end of internal functions --------------------------------------*/

/*-------------------- Public --------------------*/

/**
 * @brief 发布数据
 */
void CustomControllerPublish(void)
{

}

/**
 * @brief 初始化：上电把所有电机置于松轴状态，并配置 ZDT 定时上报
 */
void CustomControllerInit(void)
{
    memset(&custom_terminal, 0, sizeof(custom_terminal));

    /* 上电先发失能（手册 5.3.2，F3 AB 00 00 6B）。驱动器上电默认使能并锁轴，
     * 只发 0 力矩帧是松不开的，必须显式发失能命令。 */
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        if (ct_zd_addr[i] != 0U)
        {
            zdatou_set_enable(CUSTOM_TERMINAL_CAN,
                              ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[i]), ZDATOU_ENABLE_OFF);
        }
    }

    /* RS00：使能后才会回反馈帧；力矩为 0，所以不会锁死 */
    robostride_enable(CUSTOM_TERMINAL_CAN, CUSTOM_TERMINAL_RS_ID);

    /* 定时上报：每 10 ms 主动返回一次位置，运行期不再轮询 */
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        if (ct_zd_addr[i] != 0U)
        {
            zdatou_set_report(CUSTOM_TERMINAL_CAN,
                              ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[i]),
                              CUSTOM_TERMINAL_ZD_REPORT_PARAM);
        }
    }

    ct_send_phase = 0U;
    terminal_dbg.test_joint = 0U;
    terminal_dbg.gravity_scale = CUSTOM_TERMINAL_GRAVITY_SCALE;
}

/**
 * @brief 设置模式（暂无可切换模式）
 */
void CustomControllerSetMode(void)
{
}

/**
 * @brief 把驱动反馈换算成关节角，并更新正解与上行数据
 */
void CustomControllerObserver(void)
{
    static uint32_t last_stamp[CUSTOM_TERMINAL_JOINT_NUM];  // 上一帧的时间戳，用于统计新帧
    static uint32_t retry_tick;                             // 上报配置自愈计时
    const uint32_t now = HAL_GetTick();

    /* 自愈：定时上报命令是"发一次就一直有效"，万一开机那一下丢了（驱动器上电慢、
     * 总线仲裁失败），这台电机就永远不会再主动上报。每秒挑一台还没在上报的补发一次。 */
    if ((now - retry_tick) >= CUSTOM_TERMINAL_REPORT_RETRY_MS)
    {
        retry_tick = now;
        for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
        {
            const uint8_t addr = ct_zd_addr[i];

            if ((addr != 0U) && ((zdatou_dbg.ok_mask & (uint8_t)(1U << (addr - 1U))) == 0U))
            {
                zdatou_set_report(CUSTOM_TERMINAL_CAN, ZDATOU_ADDR_TO_EXT_ID(addr),
                                  CUSTOM_TERMINAL_ZD_REPORT_PARAM);
                terminal_dbg.read_cnt++;
                break;
            }
        }
    }

    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        float raw = 0.0f;   /* 电机侧机械角 (rad) */
        uint32_t stamp = 0U;

        if (i == CUSTOM_TERMINAL_RS_JOINT)
        {
            const RobostrideMeasure_s *rs =
                robostride_get_measure(CUSTOM_TERMINAL_CAN, CUSTOM_TERMINAL_RS_ID);

            raw = rs->pos;
            stamp = rs->last_fdb_time;
        }
        else
        {
            const zdatouMeasure_s *zd =
                zdatou_get_measure(CUSTOM_TERMINAL_CAN, ct_zd_addr[i]);

            raw = (float)zd->pos_deg * ZDATOU_01DEG_TO_RAD;
            stamp = zd->last_fdb_time;
            custom_terminal.joint_last_ecd[i] = zd->ecd;
        }

        /* 方向 → 零位 → 减速比；joint_raw 保留未扣零位的值，供零位标定回读 */
        const float raw_mech = (float)ct_dir[i] * raw;
        const float angle = (raw_mech - ct_zero[i]) / ct_gear[i];

        custom_terminal.joint_feedback[i] = terminal_wrap_angle(angle);

        /* 速度：相邻两拍差值，按减速比折算到关节侧 */
        const uint32_t dt_ms = now - custom_terminal.joint_last_feedback_time[i];

        if ((stamp != 0U) && (dt_ms > 0U) && (dt_ms <= CUSTOM_TERMINAL_FEEDBACK_TIMEOUT_MS))
        {
            const float delta = terminal_wrap_angle(raw_mech - custom_terminal.joint_raw[i]);

            custom_terminal.joint_velocity[i] = delta / ct_gear[i] / ((float)dt_ms * 0.001f);
        }
        else
        {
            custom_terminal.joint_velocity[i] = 0.0f;
        }

        custom_terminal.joint_raw[i] = raw_mech;
        custom_terminal.joint_last_feedback_time[i] = now;
        custom_terminal.joint_feedback_valid[i] =
            (stamp != 0U) && ((now - stamp) <= CUSTOM_TERMINAL_FEEDBACK_TIMEOUT_MS);

        /* fdb_cnt 在涨 = 反馈帧真的在刷新；不涨则说明电机没上报 */
        if ((stamp != 0U) && (stamp != last_stamp[i]))
        {
            last_stamp[i] = stamp;
            terminal_dbg.fdb_cnt++;
        }

        terminal_dbg.joint_fb[i] = custom_terminal.joint_feedback[i];
        terminal_dbg.joint_raw[i] = raw_mech;
        terminal_dbg.joint_ecd[i] = custom_terminal.joint_last_ecd[i];
    }

    /* 正解：每个关节位姿 + 末端位姿 */
    FK_SolveAll(custom_terminal.joint_feedback, joint_pose);
    custom_terminal_fk = joint_pose[CUSTOM_TERMINAL_JOINT_NUM - 1];

    /* 上行给操作手：关节角走自定义控制器协议 */
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        cc_control_data.pos[i] = custom_terminal.joint_feedback[i];
    }

    /* 零位标定：Watch 里把 zero_capture 写 1，把当前机械角记到 zero_readback，
     * 再把数值抄进 robot_param 的 CUSTOM_TERMINAL_JOINT_ZERO 即可。 */
    if (terminal_dbg.zero_capture != 0U)
    {
        for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
        {
            terminal_dbg.zero_readback[i] = custom_terminal.joint_raw[i];
        }
        terminal_dbg.zero_capture = 0U;
    }
}

/**
 * @brief 更新目标量：目前只算重力补偿前馈，留给后续控制律使用
 */
void CustomControllerReference(void)
{
    static float tau_g[CUSTOM_TERMINAL_JOINT_NUM];

    terminal_gravity(custom_terminal.joint_feedback, tau_g);

    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        terminal_dbg.tau_g[i] = tau_g[i];
    }
}

/*-------------------- Console --------------------*/

#if CUSTOM_TERMINAL_TEST_MODE

/* 当前已被本模式使能的关节（0xFF = 没有） */
static uint8_t ct_enabled_joint = 0xFFU;
/* 当前实际下发的力矩 (N·m) */
static float ct_torque_nm[CUSTOM_TERMINAL_JOINT_NUM];

/**
 * @brief 使能/失能单个关节（只发一次，避免 1 ms 刷屏）
 */
static void custom_terminal_enable_joint(uint8_t joint, bool enable)
{
    if (joint == CUSTOM_TERMINAL_RS_JOINT)
    {
        if (enable)
        {
            robostride_enable(CUSTOM_TERMINAL_CAN, CUSTOM_TERMINAL_RS_ID);
        }
        else
        {
            robostride_stop(CUSTOM_TERMINAL_CAN, CUSTOM_TERMINAL_RS_ID);
        }
    }
    else
    {
        zdatou_set_enable(CUSTOM_TERMINAL_CAN, ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[joint]),
                          enable ? ZDATOU_ENABLE_ON : ZDATOU_ENABLE_OFF);
    }
}

#endif

/**
 * @brief 计算控制量
 * @note  当前是"标定/电机测试模式"：只对单个关节给力矩，不接任何控制律。
 *        正式力矩环上线时把这一段整体换掉。
 */
void CustomControllerConsole(void)
{
#if CUSTOM_TERMINAL_TEST_MODE
    uint8_t joint = terminal_dbg.manual_joint;
    bool enable;

    if (joint >= CUSTOM_TERMINAL_JOINT_NUM)
    {
        joint = 0U;
    }

    /* 选中关节没有有效反馈时不出力，避免对着一台没回帧的电机盲推 */
    enable = (terminal_dbg.manual_en != 0U) &&
             ((terminal_dbg.manual_ignore_feedback != 0U) ||
              custom_terminal.joint_feedback_valid[joint]);

    /* 使能状态只在这里整体对齐，避免每拍重复发命令 */
    if (enable && (ct_enabled_joint != joint))
    {
        if (ct_enabled_joint != 0xFFU)
        {
            custom_terminal_enable_joint(ct_enabled_joint, false);
        }
        custom_terminal_enable_joint(joint, true);
        ct_enabled_joint = joint;
    }
    else if (!enable && (ct_enabled_joint != 0xFFU))
    {
        custom_terminal_enable_joint(ct_enabled_joint, false);
        ct_enabled_joint = 0xFFU;
    }

    /* 目标力矩：使能时取手动值，否则回零 */
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        ct_torque_nm[i] = 0.0f;
    }

    if (enable)
    {
        float limit = (joint == CUSTOM_TERMINAL_RS_JOINT) ? CUSTOM_TERMINAL_RS_TORQUE_MAX_NM
                                                         : (CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA /
                                                            ct_ma_per_nm[joint]);

        ct_torque_nm[joint] = clamp_torque(terminal_dbg.manual_torque_nm, limit);
    }

    /* 限速逼近：松手或解除后按同样速率回零 */
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        custom_terminal.joint_torque[i] =
            step_toward(custom_terminal.joint_torque[i], ct_torque_nm[i],
                        CUSTOM_TERMINAL_TEST_NM_RAMP);
    }

    terminal_dbg.armed = enable ? 1U : 0U;
    terminal_dbg.test_joint = joint;
#endif

    /* 重力补偿：默认关闭，测试模式未武装时才生效（避免与手动力矩叠加） */
    if ((terminal_dbg.gravity_en != 0U) && (terminal_dbg.armed == 0U))
    {
        for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
        {
            const float limit = (i == CUSTOM_TERMINAL_RS_JOINT)
                                    ? CUSTOM_TERMINAL_RS_TORQUE_MAX_NM
                                    : (CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA / ct_ma_per_nm[i]);
            const float target = clamp_torque(terminal_dbg.gravity_scale * terminal_dbg.tau_g[i], limit);

            custom_terminal.joint_torque[i] =
                step_toward(custom_terminal.joint_torque[i], target, CUSTOM_TERMINAL_TEST_NM_RAMP);
        }
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief 发送控制量
 * @note  每 1 ms 只发 2 帧，3 拍转完一圈，单台电机 3 ms 收到一帧。
 *        ZDT 把 N·m 按 MA_PER_NM 换成 mA；RS00 直接发 N·m。
 */
void CustomControllerSendCmd(void)
{
    for (uint8_t i = 0; i < CUSTOM_TERMINAL_JOINT_NUM; i++)
    {
        float tau = custom_terminal.joint_feedback_valid[i] ? custom_terminal.joint_torque[i]
                                                            : 0.0f;

        terminal_dbg.torque_cmd[i] = tau;
    }

    switch (ct_send_phase)
    {
    case 0:
        zdatou_torque_control_i32(
            CUSTOM_TERMINAL_CAN, ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[0]),
            (int32_t)(clamp_torque(custom_terminal.joint_torque[0],
                                   (float)CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA / ct_ma_per_nm[0]) *
                      ct_ma_per_nm[0]));
        robostride_mit_control(CUSTOM_TERMINAL_CAN, CUSTOM_TERMINAL_RS_ID, 0.0f, 0.0f, 0.0f, 0.0f,
                               clamp_torque(custom_terminal.joint_torque[1],
                                            CUSTOM_TERMINAL_RS_TORQUE_MAX_NM));
        ct_send_phase = 1U;
        break;

    case 1:
        for (uint8_t i = 2U; i < 4U; i++)
        {
            zdatou_torque_control_i32(
                CUSTOM_TERMINAL_CAN, ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[i]),
                (int32_t)(clamp_torque(custom_terminal.joint_torque[i],
                                       (float)CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA / ct_ma_per_nm[i]) *
                          ct_ma_per_nm[i]));
        }
        ct_send_phase = 2U;
        break;

    default:
        for (uint8_t i = 4U; i < 6U; i++)
        {
            zdatou_torque_control_i32(
                CUSTOM_TERMINAL_CAN, ZDATOU_ADDR_TO_EXT_ID(ct_zd_addr[i]),
                (int32_t)(clamp_torque(custom_terminal.joint_torque[i],
                                       (float)CUSTOM_TERMINAL_ZD_TORQUE_MAX_MA / ct_ma_per_nm[i]) *
                          ct_ma_per_nm[i]));
        }
        ct_send_phase = 0U;
        break;
    }
}

/*-------------------- Upload to PC --------------------*/

typedef __packed struct
{
    uint8_t sof;                                          // 帧起始 0xA5
    uint16_t data_length;                                 // 数据段长度
    uint8_t seq;                                          // 帧序号
    uint8_t crc8;                                         // 帧头校验
    uint16_t cmd_id;                                      // 命令码 0x0302
    uint8_t data[CUSTOM_TERMINAL_UPLINK_DATA_LEN];        // 数据段
    uint16_t frame_tail;                                  // 整帧校验
} custom_terminal_uplink_frame_t;

static custom_terminal_uplink_frame_t custom_terminal_uplink_frame[CUSTOM_TERMINAL_UPLINK_PORT_NUM];
static UART_HandleTypeDef *const custom_terminal_uplink_uart[CUSTOM_TERMINAL_UPLINK_PORT_NUM] = {
    &huart1, &huart6};
static uint8_t custom_terminal_uplink_seq;
static uint8_t custom_terminal_uplink_busy_cnt[CUSTOM_TERMINAL_UPLINK_PORT_NUM];

/* 连续这么多拍都发不出去就强制中止，避免一次异常把上行永久卡死 */
#define CUSTOM_TERMINAL_UPLINK_BUSY_ABORT_CNT 10U

/**
 * @brief 发一帧 0x0302 给上位机（huart1 与 huart6 各一份）
 * @note  用 DMA 发送；两个串口的 HAL 状态机由补过 HAL_UART_IRQHandler 的中断驱动。
 *        若某口长时间 BUSY（例如被别的代码改过状态），10 拍后强制 Abort 自愈。
 */
void CustomControllerSendToPC(void)
{
    const uint8_t seq = custom_terminal_uplink_seq++;

    for (uint8_t port = 0; port < CUSTOM_TERMINAL_UPLINK_PORT_NUM; port++)
    {
        UART_HandleTypeDef *uart = custom_terminal_uplink_uart[port];
        custom_terminal_uplink_frame_t *frame = &custom_terminal_uplink_frame[port];

        if (uart->gState != HAL_UART_STATE_READY)
        {
            custom_terminal_uplink_busy_cnt[port]++;
            if (custom_terminal_uplink_busy_cnt[port] >= CUSTOM_TERMINAL_UPLINK_BUSY_ABORT_CNT)
            {
                (void)HAL_UART_AbortTransmit(uart);
                custom_terminal_uplink_busy_cnt[port] = 0U;
            }
            continue;
        }
        custom_terminal_uplink_busy_cnt[port] = 0U;

        frame->sof = CUSTOM_TERMINAL_UPLINK_SOF;
        frame->data_length = CUSTOM_TERMINAL_UPLINK_DATA_LEN;
        frame->seq = seq;
        frame->crc8 = 0U;
        frame->cmd_id = CUSTOM_TERMINAL_UPLINK_CMD_ID;
        append_CRC8_check_sum((uint8_t *)frame, CUSTOM_TERMINAL_UPLINK_HEADER_LEN);

        memset(frame->data, 0, sizeof(frame->data));
        memcpy(frame->data, cc_control_data.pos, sizeof(cc_control_data.pos));
        append_CRC16_check_sum((uint8_t *)frame, sizeof(custom_terminal_uplink_frame_t));

        (void)HAL_UART_Transmit_DMA(uart, (uint8_t *)frame, (uint16_t)sizeof(*frame));
    }
}

/**
 * @brief 获取全部关节角度数组指针 (rad)
 */
const float *custom_terminal_get_joint_feedback(void)
{
    return custom_terminal.joint_feedback;
}

/************************ END OF FILE ************************/
