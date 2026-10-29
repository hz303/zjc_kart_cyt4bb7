/*********************************************************************************************************************
 * 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS）实现
 * 通道分配、标定值、急停、接管状态机、与 WiFi 的授权关系见 remote_ctrl.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "remote_ctrl.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"
#include "ins_odom.h"           /* ins_m_per_rev：rps <-> m/s */
#include "route_pb.h"           /* pb_l：与回放共用同一个有效轴距 */
#include "proto.h"              /* 驱动令牌 / 急停的唯一入口 */
#include "motor_link.h"         /* motor_link_now_ms()：中位学习的毫秒时基 */


/* ---- 对外数据 ---- */
volatile uint16  RC_Raw_Ch1     = 0;
volatile uint16  RC_Raw_Ch2     = 0;
volatile uint16  RC_Raw_Ch3     = 0;
volatile uint16  RC_Raw_Ch4     = 0;
volatile uint16  RC_Raw_Ch5     = 0;
volatile uint16  RC_Raw_Ch6     = 0;
volatile float   RC_Steer_Deg   = 0.0f;
volatile float   RC_Steer_Norm  = 0.0f;
volatile uint8   RC_Ch3_Pct     = 100u;
volatile uint8   RC_Stick_Bad   = 0u;       /* bit0 CH1 / bit1 CH2 / bit2 CH3 的 raw 低于有效下限 */
volatile uint8   RC_Hdg_Mode    = 0u;
volatile uint16  RC_Thr_Mid     = RC_THR_MID;       /* 学习到的油门中位 */
volatile uint16  RC_Steer_Mid   = RC_STEER_MID;     /* 学习到的方向盘中位 */
volatile uint16  RC_Hdg_Thr     = RC_SW_VAL_MID;   /* CH3 阈值：自适应，见 rc_ch3_track() */
volatile uint8   RC_Hdg_Cal     = 0u;              /* 1 = 已标定（见过开关两端） */
volatile float   RC_Hdg_Step    = 1.0f;
volatile float   RC_Hdg_Off     = 0.0f;
volatile float   RC_Cmd_Yaw     = 0.0f;
volatile float   RC_Throttle    = 0.0f;
volatile float   RC_Scale       = RC_SCALE_MAX;
volatile uint8   RC_Estop       = 0;
volatile uint8   RC_Arm         = 0;
volatile uint8   RC_Online      = 0;
volatile uint8   RC_Engage      = RC_ENGAGE_OFF;
volatile uint32  RC_Frames      = 0;
volatile float   RC_Target_Rps  = 0.0f;

volatile float   rc_arm_en      = 1.0f;                     /* 1 = 允许遥控接管 */
volatile float   rc_thr_max     = RC_THR_MAX_DEFAULT;       /* 满杆目标 rps     */

/* ---- 内部工具 ---- */

/* 原始值 -> 归一化(-1.0 ~ +1.0)
 * 中位两侧量程不对称，左右各自按半幅归一化，两端都能吃满 ±1.0 */
static float rc_norm (uint16 raw, int mid, int lo, int hi)
{
    float t;

    if ((int)raw >= mid)
    {
        if (hi <= mid) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(hi - mid);
    }
    else
    {
        if (mid <= lo) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(mid - lo);
    }

    if (t >  1.0f) { t =  1.0f; }
    if (t < -1.0f) { t = -1.0f; }

    return t;
}

/* 两态开关：以中位 992 为界；active_high = 1 时高档为 1 */
static uint8 rc_pick2 (uint16 v, uint8 active_high)
{
    uint8 hi = ((int)v > RC_SW_VAL_MID) ? 1u : 0u;

    return (0 != active_high) ? hi : (uint8)(1u - hi);
}

static float rc_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 中位学习：把扳机/方向盘的静止位学出来，而非固定 992 */
static uint8  s_mid_locked = 0u;        /* 0 = 仍在开机捕获阶段 */
static uint16 s_mid_vf     = 0u;        /* 自遥控信号首次有效起的帧数 */
static uint32 s_mid_ms     = 0u;        /* 上次慢跟踪时刻 ms */

static void rc_mid_step (volatile uint16 *mid, uint16 raw)
{
    int d = (int)raw - (int)(*mid);

    /* 偏离超过跟踪带 = 手在推杆，不挪动 */
    if ((d > RC_MID_TRACK_BAND) || (d < -RC_MID_TRACK_BAND)) { return; }

    if (d >= RC_MID_TRACK_STEP)        { *mid = (uint16)(*mid + RC_MID_TRACK_STEP); }
    else if (d <= -RC_MID_TRACK_STEP)  { *mid = (uint16)(*mid - RC_MID_TRACK_STEP); }
}

static void rc_mid_learn (void)
{
    uint32 now = motor_link_now_ms();
    int    d1, d2;

    /* 捕获窗口锚在信号首次有效：先上电、后开遥控器时也能捕获 */
    if (0u == (RC_Stick_Bad & 0x03u))
    {
        if (s_mid_vf < 60000u) { s_mid_vf ++; }
    }

    if (0u == s_mid_locked)
    {
        /* ① 开机捕获：第 3 ~ 20 帧取一次静止位，偏差在 CAP_BAND 内才认 */
        if ((s_mid_vf >= 3u) && (s_mid_vf <= 20u) && (0u == (RC_Stick_Bad & 0x03u)))
        {
            d1 = (int)RC_Raw_Ch1 - RC_STEER_MID;
            d2 = (int)RC_Raw_Ch2 - RC_THR_MID;

            if ((d1 <= RC_MID_CAP_BAND) && (d1 >= -RC_MID_CAP_BAND))
            {
                RC_Steer_Mid = RC_Raw_Ch1;
            }
            if ((d2 <= RC_MID_CAP_BAND) && (d2 >= -RC_MID_CAP_BAND))
            {
                RC_Thr_Mid = RC_Raw_Ch2;
            }

            s_mid_locked = 1u;
            s_mid_ms     = now;

            printf("[RC] mid captured: steer=%u thr=%u (nominal 992/992)\r\n",
                   (unsigned)RC_Steer_Mid, (unsigned)RC_Thr_Mid);
        }

        return;   /* 未锁定前不做慢跟踪 */
    }


    /* ② 慢跟踪：1 计数 / 100 ms，仅在看着像中位时挪动
     * 遥控处于 ON 时跳过：司机稳定按住小油门会被当成中位漂移学掉 */
    if ((RC_ENGAGE_ON != RC_Engage) && ((uint32)(now - s_mid_ms) >= RC_MID_TRACK_MS))
    {
        s_mid_ms = now;
        rc_mid_step(&RC_Steer_Mid, RC_Raw_Ch1);
        rc_mid_step(&RC_Thr_Mid,   RC_Raw_Ch2);
    }
}

/* CH3 二值开关 -> RC_Hdg_Mode：自适应阈值 + 滞环
 * 记 min/max；见过高位（>1200）且跨度 ≥400 才标定，阈值取中点
 * 未标定（通道未接）时保持舵机角模式；低 = 航向，高 = 舵机角 */
static uint16 s_c3_min = 0x0FFFu;
static uint16 s_c3_max = 0u;
static uint8  s_c3_cal = 0u;
static uint8  s_c3_hi  = 0u;            /* 0 = 低档（航向） / 1 = 高档（舵机角） */

static void rc_ch3_track (void)
{
    uint16 thr;
    uint8  mode;

    if (RC_Raw_Ch3 < s_c3_min) { s_c3_min = RC_Raw_Ch3; }
    if (RC_Raw_Ch3 > s_c3_max) { s_c3_max = RC_Raw_Ch3; }

    if ((0u == s_c3_cal) &&
        ((uint32)(s_c3_max - s_c3_min) >= 400u) &&
        (s_c3_max >= 1200u))
    {
        s_c3_cal = 1u;
        printf("[RC] CH3 calibrated: min=%u max=%u\r\n",
               (unsigned)s_c3_min, (unsigned)s_c3_max);
    }

    thr = (0u != s_c3_cal) ? (uint16)(((uint32)s_c3_min + (uint32)s_c3_max) / 2u)
                           : (uint16)RC_SW_VAL_MID;

    RC_Hdg_Thr = thr;
    RC_Hdg_Cal = s_c3_cal;

    /* 滞环：低->高 要 > thr+60；高->低 要 < thr-60 */
    if (0u == s_c3_hi)
    {
        if (RC_Raw_Ch3 > (uint16)(thr + RC_HDG_HYST)) { s_c3_hi = 1u; }
    }
    else
    {
        if (((uint32)RC_Raw_Ch3 + RC_HDG_HYST) < (uint32)thr) { s_c3_hi = 0u; }
    }

    /* 未标定且通道无信号：不知道开关位置，保持舵机角模式 */
    if ((0u == s_c3_cal) && (0u != (RC_Stick_Bad & 0x04u)))
    {
        mode = 0u;
    }
    else
    {
        mode = (0u != s_c3_hi) ? 0u : 1u;
    }

    if (mode != RC_Hdg_Mode)
    {
        RC_Hdg_Mode = mode;
        printf("[RC] CH3 -> %s (raw=%u thr=%u span=%u)\r\n",
               (0u != mode) ? "HEADING" : "SERVO",
               (unsigned)RC_Raw_Ch3, (unsigned)thr,
               (unsigned)(s_c3_max - s_c3_min));
    }

    /* 标定后不再把 CH3 的 0 报成掉线，避免淹没真实掉线 */
    if (0u != s_c3_cal)
    {
        RC_Stick_Bad = (uint8)(RC_Stick_Bad & (uint8)~0x04u);
    }
}

/* 按步长量化，使同一摇杆位置输出相同目标角 */
static float rc_quantize (float v, float step)
{
    if (step < 1.0e-4f) { return v; }
    return ((float)(int)((v / step) + ((v >= 0.0f) ? 0.5f : -0.5f))) * step;
}

/* 转向指令：RC_Hdg_Mode = 1 时 CH1 给目标航向（基准 + 偏移），0 时给舵机转角
 * 基准在进模式时取一次，之后每次松手（CH1 回中）重取 */
static float s_base_yaw = 0.0f;
static uint8 s_ht_on    = 0u;
static float s_prev_n   = 0.0f;

static void rc_leave_heading (void)
{
    if (0u != s_ht_on)
    {
        s_ht_on = 0u;
        steer_ctrl_set_head_enable(0u);                            /* 先关外环 */
        steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());  /* 目标对齐实测角，避免甩头 */
        RC_Hdg_Off = 0.0f;
        printf("[RC] heading mode OFF: CH1 -> steer angle\r\n");
    }
}

static void rc_apply_steer (void)
{
    if (0u != RC_Hdg_Mode)
    {
        if (0u == s_ht_on)
        {
            s_ht_on    = 1u;
            s_base_yaw = steer_yaw_deg;                 /* 基准 = 进模式时的航向 */
            s_prev_n   = 0.0f;
            printf("[RC] heading mode ON: CH3=%u%% step=%u.%u deg, CH1 = +/-%u deg\r\n",
                   (unsigned)RC_Ch3_Pct,
                   (unsigned)((int)(RC_Hdg_Step * 10.0f) / 10),
                   (unsigned)((int)(RC_Hdg_Step * 10.0f) % 10),
                   (unsigned)(int)RC_HDG_MAX_DEG);
        }

        /* 松手那一拍：把当前航向记为新基准（推一次 = 一个台阶） */
        if ((0.0f == RC_Steer_Norm) && (0.0f != s_prev_n))
        {
            s_base_yaw = steer_yaw_deg;
        }

        /* 符号：CH1 右推 = RC_Steer_Norm > 0 = 右转（δ > 0），而 yaw 正方向为左转，故取负 */
        RC_Hdg_Off = rc_quantize(-RC_Steer_Norm * RC_HDG_MAX_DEG, RC_Hdg_Step);

        if (RC_Hdg_Off >  RC_HDG_MAX_DEG) { RC_Hdg_Off =  RC_HDG_MAX_DEG; }
        if (RC_Hdg_Off < -RC_HDG_MAX_DEG) { RC_Hdg_Off = -RC_HDG_MAX_DEG; }

        RC_Cmd_Yaw = s_base_yaw + RC_Hdg_Off;

        /* 增益每拍写一次：route_pb 会按 2L/ld 改写它，不写会被带跑 */
        steer_ctrl_set_head_kp(RC_HDG_KP);
        steer_ctrl_set_head_enable(1u);
        steer_ctrl_set_target_yaw(RC_Cmd_Yaw);

        s_prev_n = RC_Steer_Norm;
    }
    else
    {
        rc_leave_heading();
        steer.target_delta = RC_Steer_Deg;
    }
}

/* 油门归一化值 -> 后轮目标 rps（乘 rc_thr_max 与 RC_Scale，按 spd_target_max 夹紧） */
static float rc_throttle_to_target (void)
{
    float t;
    float lim;

    if (rc_fabs(RC_Throttle) <= 0.0001f) { return 0.0f; }

    t   = RC_Throttle * rc_thr_max * RC_Scale;
    lim = spd_target_max;

    if (lim > 0.0f)
    {
        if (t >  lim) { t =  lim; }
        if (t < -lim) { t = -lim; }
    }

    return t;
}

/* ---- 接口实现 ---- */

/* 初始化：清状态并启动接收机（UART_4 = SCB2，RX = P14.0，SBUS 100 kbps，帧间隔计时 TC_TIME2_CH2）
 * 必须在 5 ms PIT 中断使能前调用 */
void remote_ctrl_init (void)
{
    /* 先解除接管，避免初始化期间的残留状态放开电机 */
    remote_ctrl_release();

    RC_Raw_Ch1 = 0;
    RC_Raw_Ch2 = 0;
    RC_Raw_Ch4 = 0;
    RC_Raw_Ch5 = 0;
    RC_Raw_Ch6 = 0;
    RC_Scale   = RC_SCALE_MAX;
    RC_Estop   = 0;
    RC_Arm     = 0;
    RC_Online  = 0;
    RC_Frames  = 0;

    uart_receiver_init();

    /* 清一次接收机快照，避免上电首圈读到旧值 */
    uart_receiver.state      = 0;
    uart_receiver.finsh_flag = 0;
    {
        uint8 i;
        for (i = 0; i < UART_RECEVIER_CHANNEL_NUM; i ++)
        {
            uart_receiver.channel[i] = 0;
        }
    }
}

/* 从 uart_receiver 快照换算一次；本模块不碰驱动，只读快照 */
void remote_ctrl_update (void)
{
    int thr;

    RC_Raw_Ch1 = uart_receiver.channel[RC_CH_STEER];
    RC_Raw_Ch2 = uart_receiver.channel[RC_CH_THROTTLE];
    RC_Raw_Ch3 = uart_receiver.channel[RC_CH_STEER_TRAVEL];
    RC_Raw_Ch4 = uart_receiver.channel[RC_CH_THR_TRAVEL];
    RC_Raw_Ch5 = uart_receiver.channel[RC_CH_ESTOP];
    RC_Raw_Ch6 = uart_receiver.channel[RC_CH_ARM];
    RC_Online  = uart_receiver.state;

    /* 收帧计数：不涨说明接收机没发 / 帧格式不符 / 接线错 */
    if (0 != uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;
        RC_Frames ++;
    }

    /* 两个二值开关，极性由 RC_*_ACTIVE_HIGH 决定 */
    RC_Estop = rc_pick2(RC_Raw_Ch5, RC_ESTOP_ACTIVE_HIGH);
    RC_Arm   = rc_pick2(RC_Raw_Ch6, RC_ARM_ACTIVE_HIGH);

    /* 方向盘：先过中位死区，再按实际生效限位归一化
     * 用 steer_ctrl_eff_limit() 而非 steer.limit_deg，保证不超过 ±27
     * 通道无信号（raw < RC_CH_VALID_MIN）按回中处理，并记入 RC_Stick_Bad */
    /* 先学中位，须在下面所有减中位之前 */
    rc_mid_learn();

    RC_Stick_Bad = 0u;
    if (RC_Raw_Ch1 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x01u; }
    if (RC_Raw_Ch2 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x02u; }
    if (RC_Raw_Ch3 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x04u; }

    {
        int d = (int)RC_Raw_Ch1 - (int)RC_Steer_Mid;    /* 减去学到的中位，不是常量 992 */

        if ((0u != (RC_Stick_Bad & 0x01u)) ||
            ((d <= RC_STEER_DEADBAND) && (d >= -RC_STEER_DEADBAND)))
        {
            /* 死区内 = 回中，不吃零点漂移 */
            RC_Steer_Deg  = 0.0f;
            RC_Steer_Norm = 0.0f;
        }
        else
        {
            /* 量程随中位平移，满杆仍为 ±1.0 */
            RC_Steer_Norm = (float)RC_STEER_DIR_SIGN
                          * rc_norm(RC_Raw_Ch1,
                                    (int)RC_Steer_Mid,
                                    (int)RC_Steer_Mid - (RC_STEER_MID - RC_STEER_MIN),
                                    (int)RC_Steer_Mid + (RC_STEER_MAX - RC_STEER_MID));
            RC_Steer_Deg  = RC_Steer_Norm * steer_ctrl_eff_limit();
        }
    }

    /* 油门：死区 ±RC_THR_DEADBAND 压掉静止抖动 */
    thr = (int)RC_Raw_Ch2 - (int)RC_Thr_Mid;     /* 同上：学到的中位 */
    if ((0u != (RC_Stick_Bad & 0x02u)) ||
        ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND)))
    {
        RC_Throttle = 0.0f;         /* 无信号按回中，不变成满倒车 */
    }
    else
    {
        RC_Throttle = rc_norm(RC_Raw_Ch2,
                              (int)RC_Thr_Mid,
                              (int)RC_Thr_Mid - (RC_THR_MID - RC_THR_MIN),
                              (int)RC_Thr_Mid + (RC_THR_MAX - RC_THR_MID));
    }

    RC_Scale = RC_SCALE_MAX;    /* 恒 1.0：行程在发射机侧缩放 */

    /* CH3 二值开关 -> 模式：低 = CH1 给目标航向 / 高 = CH1 给舵机角
     * 阈值走 rc_ch3_track() 的自适应 + 滞环；须在 RC_Stick_Bad 算完之后调用 */
    rc_ch3_track();
    RC_Ch3_Pct  = (0u != RC_Hdg_Mode) ? 0u : 100u;
    RC_Hdg_Step = RC_HDG_STEP;
}

/* 接管状态机，优先级从高到低：
 * 1) 接收机失控（ST=0，含从未收到帧）-> FAILSAFE（锁存）
 * 2) 急停生效或未回位 -> ESTOP（锁存）
 * 3) 使能开关收回 / 上位机禁用遥控 -> OFF
 * 4) 合闸（CH6 + 在线 + 授权）-> STANDBY，油门回中才升 ON
 * FAILSAFE 与 ESTOP 均锁存，须先把开关收回才能恢复 */
static void rc_state_machine (void)
{
    uint8 armed;

    /* 1) 接收机报告失控（从未收到帧时 state 恒 0）*/
    if (0 == RC_Online)
    {
        RC_Engage = RC_ENGAGE_FAILSAFE;
        return;
    }

    /* 2) 急停：生效即锁存；只有 CH5 回到不生效位置才放行 */
    if (0 != RC_Estop)
    {
        RC_Engage = RC_ENGAGE_ESTOP;
        return;
    }

    /* 急停已回位但状态还停在 ESTOP -> 降回 OFF，等重新合闸 */
    if (RC_ENGAGE_ESTOP == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    armed = ((rc_arm_en > 0.5f) && (0 != RC_Arm)) ? 1u : 0u;

    /* 3) 使能开关收回 / 上位机禁用遥控：无条件放开，并清 FAILSAFE 锁存 */
    if (0 == armed)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    /* 4) 合闸中但仍锁存在 FAILSAFE -> 保持挂起，等 3) 放开一次 */
    if (RC_ENGAGE_FAILSAFE == RC_Engage)
    {
        return;
    }

    /* 5) 刚合闸：先进 STANDBY（双环使能、目标 0），确认方向盘回中 */
    if (RC_ENGAGE_OFF == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_STANDBY;
        return;
    }

    /* 6) STANDBY -> ON：须油门在中位，防止合闸瞬间窜车 */
    if ((RC_ENGAGE_STANDBY == RC_Engage) && (rc_fabs(RC_Throttle) <= RC_ARM_THR_NEUTRAL))
    {
        RC_Engage = RC_ENGAGE_ON;
    }
    /* ON 保持 */
}

/* 跑状态机、写目标、推急停与本地接管档位
 * 授权：ON 时 spd_safety_set_local(1)，退出 / 失控 / 急停时 set_local(0)
 * 目标量（spd_target_l/r、steer.target_delta）只在遥控接管时写：
 *   ON 每拍写；进 STANDBY 那一拍与退出那一拍写 0 一次；其余时刻不写
 * 这三个量上位机也按地址直接写入，无条件压 0 会覆盖上位机下发且不报错 */
void remote_ctrl_apply (void)
{
    static uint8 prev_engage = 0xFFu;       /* 上一拍状态(0xFF = 本函数还没跑过) */
    uint8        held;                      /* 上一拍是不是遥控持有控制权 */

    rc_state_machine();

    /* 急停是全局的：不分谁在开车，先硬停两个环；放在 switch 之前，本拍必定生效 */
    proto_estop(PROTO_SRC_RC, RC_Estop);


    held = ((RC_ENGAGE_ON == prev_engage) || (RC_ENGAGE_STANDBY == prev_engage)) ? 1u : 0u;

    switch (RC_Engage)
    {
        case RC_ENGAGE_ON:
            /* 驱动令牌：遥控 ON 即抢占，每圈 acquire 一次即续租 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 遥控接管：双环使能 + 遥控给目标 */
            spd_safety_set_local(1);
            steer_en_cmd = 1.0f;
            RC_Target_Rps = rc_throttle_to_target();

            /* CH3 ≤ 50% 时 CH1 给目标航向（±20°），否则给舵机角 */
            rc_apply_steer();

            /* 电子差速：κ = tan(实测δ)/L，v_l = v(1-κW/2)、v_r = v(1+κW/2)
             * spd_target_* 单位 rps，spd_diff_alloc 用 m/s，进出各换算一次 m_per_rev
             * 用实测舵角而非目标角，避免后轮先于舵机到位差速 */
            {
                float mpr   = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
                float v_mps = RC_Target_Rps * mpr;
                float vl    = v_mps;
                float vr    = v_mps;

                spd_diff_alloc(v_mps, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

                spd_target_l = vl / mpr;
                spd_target_r = vr / mpr;
            }
            break;

        case RC_ENGAGE_STANDBY:
            /* 合闸即占住令牌，避免油门回中期间上位机写入目标 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 待命：双环使能（方向盘主动回中），目标仅在入口那一拍压 0
             * CH6 已合闸即视为遥控接管，上位机的写入会被 NAK(busy)；交还控制权需收回 CH6 */

            RC_Target_Rps = 0.0f;
            steer_en_cmd = 1.0f;
            rc_leave_heading();                 /* 待命 = 方向盘回中，不保持航向 */
            if (RC_ENGAGE_STANDBY != prev_engage)
            {
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;

        default:
            /* OFF / ESTOP / FAILSAFE：
             * 1) 放掉本地档位，控制权交还上位机（急停那一路已被硬钉住）
             * 2) 从遥控持有退出时，本拍把目标压 0 并失能转向
             * 3) 之后不再写这三个量，否则上位机的滑条失效 */
            spd_safety_set_local(0);
            (void)proto_drv_release(PROTO_SRC_RC);      /* 退出即释放驱动权，上位机/菜单可接管 */
            RC_Target_Rps = 0.0f;
            if (0 != held)
            {
                rc_leave_heading();
                steer_en_cmd = 0.0f;
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;
    }

    prev_engage = RC_Engage;
}

/* 立刻解除接管（上电 / 停机路径调用）；唯一能放开急停的入口 */
void remote_ctrl_release (void)
{
    RC_Engage     = RC_ENGAGE_OFF;
    RC_Target_Rps = 0.0f;

    proto_estop(PROTO_SRC_RC, 0u);      /* 唯一能清急停的地方：上电 / 停机路径 */
    proto_drv_release(PROTO_SRC_RC);    /* 上电初态 = 没有任何设备持有驱动权 */
    spd_safety_set_local(0);

    steer_en_cmd = 0.0f;
    steer.target_delta = 0.0f;
    spd_target_l = 0.0f;
    spd_target_r = 0.0f;
}

uint8 remote_ctrl_estop (void)
{
    return RC_Estop;
}
