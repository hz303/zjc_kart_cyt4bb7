/* 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS）实现
 * 通道分配、标定值、急停、接管状态机、与 WiFi 的授权关系见 remote_ctrl.h */

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
#include "motor_link.h"         /* motor_link_now_ms()：中位学习要一个毫秒时基 */


/* ---- 对外数据 ---- */
volatile uint16  RC_Raw_Ch1     = 0;
volatile uint16  RC_Raw_Ch2     = 0;
volatile uint16  RC_Raw_Ch3     = 0;
volatile uint16  RC_Raw_Ch4     = 0;
volatile uint16  RC_Raw_Ch5     = 0;
volatile uint16  RC_Raw_Ch6     = 0;
volatile float   RC_Steer_Deg   = 0.0f;
volatile float   RC_Steer_Norm  = 0.0f;

volatile uint8   RC_Stick_Bad   = 0u;       /* bit0 CH1 / bit1 CH2 的 raw 低于有效下限 */
volatile uint16  RC_Thr_Mid     = RC_THR_MID;       /* 学习到的油门中位（规则见 RC_MID_*）*/
volatile uint16  RC_Steer_Mid   = RC_STEER_MID;     /* 学习到的方向盘中位 */

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

/* 原始值 -> 归一化(-1.0 ~ +1.0)。中位两侧量程不对称（方向盘 776:824），两侧分别归一化，两端都能吃满 ±1.0 */
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

/* 两态开关：以中位 992 为界，activeHigh = 1 时高档为 1 */
static uint8 rc_pick2 (uint16 v, uint8 active_high)
{
    uint8 hi = ((int)v > RC_SW_VAL_MID) ? 1u : 0u;

    return (0 != active_high) ? hi : (uint8)(1u - hi);
}

static float rc_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 中位学习：把方向盘/扳机的静止位学出来，不恒等 992；规则见 RC_MID_* 四个宏 */
static uint8  s_mid_locked = 0u;        /* 0 = 还在开机捕获阶段 */
static uint16 s_mid_vf     = 0u;        /* 自遥控信号首次有效起的帧数（捕获窗口锚点）*/
static uint32 s_mid_ms     = 0u;        /* 上次慢跟踪的时刻 ms */

static void rc_mid_step (volatile uint16 *mid, uint16 raw)
{
    int d = (int)raw - (int)(*mid);

    /* 手在推/扳着时一个计数都不许动，否则推杆会把中位学跑 */
    if ((d > RC_MID_TRACK_BAND) || (d < -RC_MID_TRACK_BAND)) { return; }

    if (d >= RC_MID_TRACK_STEP)        { *mid = (uint16)(*mid + RC_MID_TRACK_STEP); }
    else if (d <= -RC_MID_TRACK_STEP)  { *mid = (uint16)(*mid - RC_MID_TRACK_STEP); }
}

static void rc_mid_learn (void)
{
    uint32 now = motor_link_now_ms();
    int    d1, d2;

    /* 捕获窗口锚在信号首次有效：先上电、后开遥控器时早期帧没有信号 */
    if (0u == (RC_Stick_Bad & 0x03u))
    {
        if (s_mid_vf < 60000u) { s_mid_vf ++; }
    }

    if (0u == s_mid_locked)
    {
        /* ① 开机捕获：第 3~20 帧之间取一次静止位；偏离标称中位超过 ±RC_MID_CAP_BAND 不认，保持标称值 */
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


    /* ② 慢跟踪：1 计数 / 100 ms，只在跟踪带内挪。必须排除 ON 段：ON 时按住小油门会被当成中位漂移学掉 */
    if ((RC_ENGAGE_ON != RC_Engage) && ((uint32)(now - s_mid_ms) >= RC_MID_TRACK_MS))
    {
        s_mid_ms = now;
        rc_mid_step(&RC_Steer_Mid, RC_Raw_Ch1);
        rc_mid_step(&RC_Thr_Mid,   RC_Raw_Ch2);
    }
}

/* 转向指令只有一种模式：CH1 直接写 steer.target_delta，此处不得改写目标角 */
static void rc_apply_steer (void)
{
    steer.target_delta = RC_Steer_Deg;
}

/* 油门归一化值 -> 后轮目标 rps：乘 rc_thr_max 与 RC_Scale，再按 spd_target_max 夹紧 */
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

/* 初始化：清状态 + 起接收机（UART_4 = SCB2 / P14.0, SBUS 100 kbps, 引脚取自 zf_device_uart_receiver.h）。
   必须在 5 ms PIT 起来之前调用，接收机会注册帧间隔计数器 */
void remote_ctrl_init (void)
{
    /* 先解除接管，避免初始化期间的残留状态把电机放开 */
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

    /* 顺便清一次接收机快照，避免上电第一圈读到栈上的旧值 */
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

/* 从接收机快照换算一次（只读 uart_receiver，不碰驱动） */
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

    /* 收帧计数：RC_Frames 不增长说明接收机没在发 / 帧格式对不上 / 线接错 */
    if (0 != uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;
        RC_Frames ++;
    }

    /* 两个二值开关（极性由 RC_*_ACTIVE_HIGH 定） */
    RC_Estop = rc_pick2(RC_Raw_Ch5, RC_ESTOP_ACTIVE_HIGH);
    RC_Arm   = rc_pick2(RC_Raw_Ch6, RC_ARM_ACTIVE_HIGH);

    /* 先把中位学出来，必须在下面所有减中位之前 */
    rc_mid_learn();

    /* 方向盘：先过中位死区（±RC_STEER_DEADBAND），再归一化到 -1~+1 乘 steer_ctrl_eff_limit()，
       该限位保证参数怎么改都越不过 ±27；通道无信号时按下限处理为回中，并记入 RC_Stick_Bad */
    RC_Stick_Bad = 0u;
    if (RC_Raw_Ch1 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x01u; }
    if (RC_Raw_Ch2 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x02u; }

    {
        int d = (int)RC_Raw_Ch1 - (int)RC_Steer_Mid;    /* 减的是学到的中位 */

        if ((0u != (RC_Stick_Bad & 0x01u)) ||
            ((d <= RC_STEER_DEADBAND) && (d >= -RC_STEER_DEADBAND)))
        {
            /* 死区内按方向盘回中处理，不跟随电位器零点漂移 */
            RC_Steer_Deg  = 0.0f;
            RC_Steer_Norm = 0.0f;
        }
        else
        {
            /* 量程随中位平移（span 不变），满杆仍为 ±1.0，0 点对到真实静止位 */
            RC_Steer_Norm = (float)RC_STEER_DIR_SIGN
                          * rc_norm(RC_Raw_Ch1,
                                    (int)RC_Steer_Mid,
                                    (int)RC_Steer_Mid - (RC_STEER_MID - RC_STEER_MIN),
                                    (int)RC_Steer_Mid + (RC_STEER_MAX - RC_STEER_MID));
            RC_Steer_Deg  = RC_Steer_Norm * steer_ctrl_eff_limit();
        }
    }

    /* 油门：同样 ±RC_THR_DEADBAND 死区，把静止抖动压成 0 */
    thr = (int)RC_Raw_Ch2 - (int)RC_Thr_Mid;     /* 减学到的中位 */
    if ((0u != (RC_Stick_Bad & 0x02u)) ||
        ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND)))
    {
        RC_Throttle = 0.0f;         /* 通道无信号按油门回中，不许变成满倒车 */
    }
    else
    {
        RC_Throttle = rc_norm(RC_Raw_Ch2,
                              (int)RC_Thr_Mid,
                              (int)RC_Thr_Mid - (RC_THR_MID - RC_THR_MIN),
                              (int)RC_Thr_Mid + (RC_THR_MAX - RC_THR_MID));
    }

    RC_Scale = RC_SCALE_MAX;    /* 恒 1.0：行程在发射机侧缩放, 代码再乘即为平方 */
}

/* ---- 接管状态机，优先级从高到低 ----
 * 1) 接收机失控（ST=0, 含从未收到帧） -> FAILSAFE（锁存）
 * 2) 急停生效 / 急停未回位            -> ESTOP（锁存）
 * 3) 使能开关收回 / 上位机禁遥控      -> OFF
 * 4) 合闸（CH6 + 在线 + 授权）        -> STANDBY, 油门回中才升 ON
 * FAILSAFE 与 ESTOP 锁存，需先把开关收回再重新合闸 */
static void rc_state_machine (void)
{
    uint8 armed;

    /* 1) 接收机报告失控（或从来没收到过帧 -> state 恒 0）*/
    if (0 == RC_Online)
    {
        RC_Engage = RC_ENGAGE_FAILSAFE;
        return;
    }

    /* 2) 急停：生效即锁存；锁存后只有 CH5 回到不生效位置才放行 */
    if (0 != RC_Estop)
    {
        RC_Engage = RC_ENGAGE_ESTOP;
        return;
    }

    /* 急停开关已回位但状态还停在 ESTOP -> 降回 OFF，等重新合闸 */
    if (RC_ENGAGE_ESTOP == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    armed = ((rc_arm_en > 0.5f) && (0 != RC_Arm)) ? 1u : 0u;

    /* 3) 使能开关收回 / 上位机禁了遥控：无条件放开，并清 FAILSAFE 锁存 */
    if (0 == armed)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    /* 4) 合闸中，但还处于 FAILSAFE 锁存 -> 保持挂起，等上面的 3) 放开一次 */
    if (RC_ENGAGE_FAILSAFE == RC_Engage)
    {
        return;
    }

    /* 5) 刚合闸：先进 STANDBY（双环使能、目标 0），让人看清楚方向盘回中 */
    if (RC_ENGAGE_OFF == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_STANDBY;
        return;
    }

    /* 6) STANDBY -> ON：必须油门在中位（防止合闸瞬间全油门窜出去） */
    if ((RC_ENGAGE_STANDBY == RC_Engage) && (rc_fabs(RC_Throttle) <= RC_ARM_THR_NEUTRAL))
    {
        RC_Engage = RC_ENGAGE_ON;
    }
    /* ON 保持 */
}

/* ---- 跑状态机 + 写目标 + 推急停/本地接管档位 ----
 * 授权：ON 置本地授权（替掉上位机在场三道门），退出/失控/急停恢复五道门；心跳门与限幅门始终生效
 * 目标量只在遥控真在开车时写：ON 每拍写，进 STANDBY 与退出那一拍写 0，其余时刻不写
 * 这三个量上位机也按地址写（target_l / target_r / steer_tgt），每拍压 0 会废掉上位机滑条 */
void remote_ctrl_apply (void)
{
    static uint8 prev_engage = 0xFFu;       /* 上一拍状态(0xFF = 本函数还没跑过) */
    uint8        held;                      /* 上一拍是否由遥控持有控制权 */

    rc_state_machine();

    /* 急停是全局的：先硬停两个环，再走 proto_estop()，它同时剥夺驱动令牌。
       放在 switch 之前，ESTOP 后状态机降回 OFF 时本拍仍先推住急停 */
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

            /* CH1 -> 舵机角，CH3 不参与转向 */
            rc_apply_steer();

            /* 电子差速（与回放同一条律）：κ = tan(实测δ)/L，v_l = v(1−κW/2)、v_r = v(1+κW/2)
               单位：spd_target_* 是 rps，spd_diff_alloc 吃 m/s，进来乘 m_per_rev、出去除回去
               用实测舵角而非目标角，否则后轮会在舵机到位前提前差速 */
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
            /* 合闸即占住驱动令牌，STANDBY 期间上位机写入会被 NAK(busy) */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 待命：双环使能（方向盘主动回中），目标只在入口那一拍压 0 */

            RC_Target_Rps = 0.0f;
            steer_en_cmd = 1.0f;
            if (RC_ENGAGE_STANDBY != prev_engage)
            {
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;

        default:
            /* OFF / ESTOP / FAILSAFE：放掉本地档位并释放驱动权；从持有控制权退出那一拍把三个目标量压 0，
               其余时刻不碰这三个量，否则上位机滑条全废 */
            spd_safety_set_local(0);
            (void)proto_drv_release(PROTO_SRC_RC);      /* 退出即释放驱动权，上位机/菜单可接管 */
            RC_Target_Rps = 0.0f;
            if (0 != held)
            {
                steer_en_cmd = 0.0f;
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;
    }

    prev_engage = RC_Engage;
}

/* 立刻解除接管（上电 / 停机路径调用）。唯一能放开急停的地方，正常运行时只有 CH5 回位能放 */
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
