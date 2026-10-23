/* 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS）实现 通道分配、标定值、急停、接管状态机、与 WiFi 的授权关系，全部见 remote_ctrl.h */

#include "zf_common_headfile.h"
#include "remote_ctrl.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"
#include "ins_odom.h"           /* ins_m_per_rev：rps <-> m/s */
#include "route_pb.h"           /* pb_l：与回放共用同一个有效轴距 */
#include "proto.h"
#include "motor_link.h"         /* motor_link_now_ms()：中位学习要一个毫秒时基 */

/* 对外数据 */
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
volatile uint16  RC_Thr_Mid     = RC_THR_MID;       /* 学习到的油门中位（见 remote_ctrl.h 的 RC_MID_* ） */
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
volatile float   rc_thr_max     = RC_THR_MAX_DEFAULT;       /* 满杆目标 rps */

/* 内部工具 */

/* 原始值 -> 归一化(-1.0 ~ +1.0) 中位两侧量程不对称（方向盘 776:824），所以两侧分别归一化， */
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

/* 两态开关：以中位(992)为界。activeHigh = 1 时"高档 = 1" */
static uint8 rc_pick2 (uint16 v, uint8 active_high)
{
    uint8 hi = ((int)v > RC_SW_VAL_MID) ? 1u : 0u;

    return (0 != active_high) ? hi : (uint8)(1u - hi);
}

static float rc_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 中位学习：把"扳机/方向盘的静止位"学出来，而不是永远假设 992 ⇒ 归一化油门 -0.03 ⇒ target -0.06 rps，车自己慢慢倒。 */
static uint8  s_mid_locked = 0u;        /* 0 = 还在开机捕获阶段 */
static uint32 s_mid_ms     = 0u;

static void rc_mid_step (volatile uint16 *mid, uint16 raw)
{
    int d = (int)raw - (int)(*mid);

    /* 明显不是中位（手在推/扳着）⇒ 一个计数都不许动，否则推杆会把中位学跑 */
    if ((d > RC_MID_TRACK_BAND) || (d < -RC_MID_TRACK_BAND)) { return; }

    if (d >= RC_MID_TRACK_STEP)        { *mid = (uint16)(*mid + RC_MID_TRACK_STEP); }
    else if (d <= -RC_MID_TRACK_STEP)  { *mid = (uint16)(*mid - RC_MID_TRACK_STEP); }
}

static void rc_mid_learn (void)
{
    uint32 now = motor_link_now_ms();
    int    d1, d2;

    if (0u == s_mid_locked)
    {
        /* ① 开机捕获：收到 3 帧以后、20 帧以前，取一次"看起来是静止位"的值。 */
        if ((RC_Frames >= 3u) && (RC_Frames <= 20u) && (0u == (RC_Stick_Bad & 0x03u)))
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
        return;
    }

    /* ② 慢跟踪：1 计数 / 100 ms，且只在"看着像中位"时挪。
     * 100 ms 挪 1 个 → 5 s 就被当成"中位漂移"学掉 ⇒ 车越走越慢。这不是理论风险，
     * 是"学习式中位"这类实现的经典坑。
     * OFF/STANDBY 时本来要求油门回中（进 ON 也要油门先回中），所以那两段学中位是安全的。
     */
    if ((RC_ENGAGE_ON != RC_Engage) && ((uint32)(now - s_mid_ms) >= RC_MID_TRACK_MS))
    {
        s_mid_ms = now;
        rc_mid_step(&RC_Steer_Mid, RC_Raw_Ch1);
        rc_mid_step(&RC_Thr_Mid,   RC_Raw_Ch2);
    }
}

/* CH3 二值开关：自适应阈值 + 滞环  ->  RC_Hdg_Mode
 * 旧写法是 `(raw > 992) ? 舵机 : 航向`，外加"raw<120 判成掉线 ⇒ 强制舵机"。
 * 这三个假设里任何一个不成立，症状都**一模一样**：模式永远出不来。
 * ① 低档 = 0     -> 先被 RC_CH_VALID_MIN 判成"没信号"，强制舵机（就是现在的现象）
 * ② 低档 = 992   -> 阈值正好压在低档上，`>992` 不成立 ⇒ 高档反而判成"低"？两边都错
 * ③ 量程不是 192~1792（有的接收机 1000~2000）-> 992 这个数根本不在量程里
 * ⇒ 不再相信写死的端点：自己记 min/max，
 * · 见过 >1200（说明开关真能到高位）且跨度 ≥400 ⇒ 标定，阈值 = (min+max)/2；
 * · 通道真没接（恒 0 / 恒 992，跨度不够）      ⇒ **不标定**，保持舵机角模式（最保守）。
 * 阈值两侧各留 RC_HDG_HYST 的滞环：开关在临界点抖动时模式不会来回翻
 * （翻一次就会把"松手基准航向"重取一次，车会莫名其妙偏一下）。
 */
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

    /* 没标定 + 通道没信号 = 不知道开关在哪 -> 保持舵机角模式（原行为最安全） */
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

    /* 再置 bit2 只会让屏幕一直挂着 `!`，把"真的掉线"这个信号淹掉。 */
    if (0u != s_c3_cal)
    {
        RC_Stick_Bad = (uint8)(RC_Stick_Bad & (uint8)~0x04u);
    }
}

/* 油门行程缩放：**恒 1.0，不从通道读**
 * 接收机收到的 CH2 已经是缩过的 ⇒ 代码里再乘一次就是**平方**（0.5 的行程变成 0.25）。
 * 那个三档开关（CH4）**空置**，不承担任何功能。
 * 想调快慢：拧发射机那个旋钮，或改参数表 rc_thr_max(28)。
 * 想临时限速：用路表/回放里的 pb_vmax 或菜单 7.Speed。
 */

/* 按步长量化（0.2 / 0.5 / 1.0 deg）
 * 量化是为了让"同一个摇杆位置"每次都给出**完全一样**的目标角，
 * 否则手一抖 20.003 与 20.000 会来回跳，看起来像外环在抖。
 */
static float rc_quantize (float v, float step)
{
    if (step < 1.0e-4f) { return v; }
    return ((float)(int)((v / step) + ((v >= 0.0f) ? 0.5f : -0.5f))) * step;
}

/* 转向指令：两种模式二选一（见 remote_ctrl.h 三·五）
 * RC_Hdg_Mode = 1 ⇒ CH1 给**目标航向**（基准 + 偏移）
 * RC_Hdg_Mode = 0 ⇒ CH1 给**舵机转角**（原行为）
 * 基准 s_base_yaw：进模式取一次；之后**每次松手（CH1 回中）再取一次**
 * ⇒ 推一下走一段、松手停在新的航向上，可以连做台阶。
 */
static float s_base_yaw = 0.0f;
static uint8 s_ht_on    = 0u;
static float s_prev_n   = 0.0f;

static void rc_leave_heading (void)
{
    if (0u != s_ht_on)
    {
        s_ht_on = 0u;
        steer_ctrl_set_head_enable(0u);                            /* 先关外环 */
        steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());  /* 目标对齐实测，别甩头 */
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

        /* 松手那一拍：把当前航向记成新基准（推一次 = 一个台阶） */
        if ((0.0f == RC_Steer_Norm) && (0.0f != s_prev_n))
        {
            s_base_yaw = steer_yaw_deg;
        }

        /* 而本工程 `yaw` 正 = **左转**（x += -sin(yaw)*ds）。 ⇒ 右推必须让目标航向**变小**，所以取负号。写正号 = "推右往左拐"。 */
        RC_Hdg_Off = rc_quantize(-RC_Steer_Norm * RC_HDG_MAX_DEG, RC_Hdg_Step);

        if (RC_Hdg_Off >  RC_HDG_MAX_DEG) { RC_Hdg_Off =  RC_HDG_MAX_DEG; }
        if (RC_Hdg_Off < -RC_HDG_MAX_DEG) { RC_Hdg_Off = -RC_HDG_MAX_DEG; }

        RC_Cmd_Yaw = s_base_yaw + RC_Hdg_Off;

        /* 增益每拍写一次：route_pb 会按 2L/ld 改它，不写就会被带跑 */
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

/* 油门归一化值 -> 后轮目标 rps（带死区 + 速度缩放 + 上限夹紧） */
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

/* 接口实现 */

/* 初始化：清状态 + 起接收机
 * 接收机：UART_4 = SCB2，RX = P14.0，SBUS 100 kbps，帧间隔计时用 TC_TIME2_CH2
 * （以上引脚/通道都在 zf_device_uart_receiver.h 里，逐飞库自带，不在这里重定义）
 */
void remote_ctrl_init (void)
{
    /* 先整流解除，避免初始化期间的残留状态把电机放开 */
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

/* 从接收机快照换算一次（本模块不主动碰驱动，只读 uart_receiver） */
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

    /* 收帧计数：FRAMES 一直不涨 => 接收机没在发 / 帧格式对不上 / 线接错 */
    if (0 != uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;
        RC_Frames ++;
    }

    /* 两个二值开关（极性由 RC_*_ACTIVE_HIGH 定） */
    RC_Estop = rc_pick2(RC_Raw_Ch5, RC_ESTOP_ACTIVE_HIGH);
    RC_Arm   = rc_pick2(RC_Raw_Ch6, RC_ARM_ACTIVE_HIGH);

    /* 方向盘：**先过中位死区**（±RC_STEER_DEADBAND），再归一化到 -1~+1 乘【实际生效限位】。
     * 已经保证"参数怎么改都越不过 ±27"（见 steer_abs.h / steer_ctrl.h）。
     */
    /* 接收机取不到某通道时 `channel[]` 是 0，而 0 在量程 192~1792 之外：
     * 归一化会算出 -1.24、夹到 -1 ⇒ 满舵（油门同理 ⇒ 满倒车，更危险）。
     * 现场症状就是题述的"CH1 为 0 时一给油，方向盘莫名其妙打一个角"——
     * 其实是"方向盘被判成左满舵"，只是 OFF/STANDBY 不写转向，一合闸才看得见。
     * ⇒ raw 低于有效下限时按"摇杆回中"处理，并记进 RC_Stick_Bad（屏幕那一行看得见）。
     */
    rc_mid_learn();

    RC_Stick_Bad = 0u;
    if (RC_Raw_Ch1 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x01u; }
    if (RC_Raw_Ch2 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x02u; }
    if (RC_Raw_Ch3 < RC_CH_VALID_MIN) { RC_Stick_Bad |= 0x04u; }

    {
        int d = (int)RC_Raw_Ch1 - (int)RC_Steer_Mid;

        if ((0u != (RC_Stick_Bad & 0x01u)) ||
            ((d <= RC_STEER_DEADBAND) && (d >= -RC_STEER_DEADBAND)))
        {
            /* 死区内 = 方向盘回中：不跟着手上/电位器的零点漂移走。 （没有这一段时，静止时方向盘会被 ±1~2 个计数的抖动顶到零点几度） */
            RC_Steer_Deg  = 0.0f;
            RC_Steer_Norm = 0.0f;
        }
        else
        {
            /* 量程跟着中位一起平移（span 不变）⇒ 满杆仍然是 ±1.0，只是 0 点对到了真实静止位 */
            RC_Steer_Norm = (float)RC_STEER_DIR_SIGN
                          * rc_norm(RC_Raw_Ch1,
                                    (int)RC_Steer_Mid,
                                    (int)RC_Steer_Mid - (RC_STEER_MID - RC_STEER_MIN),
                                    (int)RC_Steer_Mid + (RC_STEER_MAX - RC_STEER_MID));
            RC_Steer_Deg  = RC_Steer_Norm * steer_ctrl_eff_limit();
        }
    }

    thr = (int)RC_Raw_Ch2 - (int)RC_Thr_Mid;
    if ((0u != (RC_Stick_Bad & 0x02u)) ||
        ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND)))
    {
        RC_Throttle = 0.0f;         /* 通道没信号时按"油门回中"，不许变成满倒车 */
    }
    else
    {
        RC_Throttle = rc_norm(RC_Raw_Ch2,
                              (int)RC_Thr_Mid,
                              (int)RC_Thr_Mid - (RC_THR_MID - RC_THR_MIN),
                              (int)RC_Thr_Mid + (RC_THR_MAX - RC_THR_MID));
    }

    RC_Scale = RC_SCALE_MAX;    /* 恒 1.0，见上面那段：行程在发射机侧缩 */

    /* CH3 **二值**开关 -> 模式：低 = CH1 给目标航向 / 高 = CH1 给舵机角。 */
    rc_ch3_track();
    RC_Ch3_Pct  = (0u != RC_Hdg_Mode) ? 0u : 100u;
    RC_Hdg_Step = RC_HDG_STEP;
}

/* 接管状态机
 * 优先级（从高到低）：
 * 1) 接收机失控（ST=0，含"从来没收到过帧"）  -> FAILSAFE（锁存）
 * 2) 急停生效 / 急停未回位                  -> ESTOP（锁存）
 * 3) 使能开关收回 / 上位机禁了遥控            -> OFF
 * 4) 合闸（CH6 合闸 + 在线 + 授权）           -> OFF 先 STANDBY；油门回中才升 ON
 * 失控/急停过就一定是哪里不对，不该让它自动恢复。
 */
static void rc_state_machine (void)
{
    uint8 armed;

    /* 1) 接收机报告失控（或从来没收到过帧 -> state 恒 0） */
    if (0 == RC_Online)
    {
        RC_Engage = RC_ENGAGE_FAILSAFE;
        return;
    }

    /* 2) 急停：生效就锁存；已经锁存时只有"CH5 回到不生效位置"才放行 */
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

/* 跑状态机 + 写目标 + 推急停/本地接管档位
 * · 遥控 ON          -> spd_safety_set_local(1)：把"上位机在场"那三道门换成"本地授权"
 * · 退出 / 失控 / 急停 -> spd_safety_set_local(0)：完整五道门原样恢复
 * => 两条路合起来 = 「WiFi 在场 OR 遥控在场」，两个都不在才真的不动。
 * [!] 心跳门与限幅门**不参与**替换，永远生效。
 * · ON                 每拍写 —— 遥控就是当前驾驶员（全工程唯一一处每拍写目标的地方）；
 * · 进 STANDBY 那一拍、以及从 ON/STANDBY **退出**那一拍 —— 写 0 一次
 * （"停住 + 方向盘回中"的语义，管住"摇杆还推着但接收机突然没了"）；
 * · 其余任何时刻 —— **一个字都不许写**。
 * [!][!] 这条是硬约束：这三个量**上位机也在写**（参数 target_l / target_r / steer_tgt
 * 就是按地址直接下发到这三个变量的）。遥控模块每 5ms 无条件压 0，等于把上位机
 * 那三个滑条全废掉，而且**不报任何错**：
 * 根因：接收机没插时 RC_Online 恒 0 => 状态机恒为 FAILSAFE => 旧代码的
 * ESTOP/FAILSAFE/OFF 三个分支每拍 spd_target_l = 0，刚写进去的值下一拍就被清掉。
 * 语义上也要分清：**"遥控没在开车" != "车必须停"**。车要停有急停门（硬钉输出）
 * 和占空比限幅门，不该靠"霸占目标量"来实现。
 */
void remote_ctrl_apply (void)
{
    static uint8 prev_engage = 0xFFu;       /* 上一拍状态(0xFF = 本函数还没跑过) */
    uint8        held;                      /* 上一拍是不是"遥控持有控制权" */

    rc_state_machine();

    /* 急停是**全局**的：不管此刻谁在开车，先把两个环硬停掉。 放在 switch 之前 -> 即使 ESTOP 之后状态机立刻降回 OFF，本拍也仍然先把急停推住。 */
    proto_estop(PROTO_SRC_RC, RC_Estop);

    held = ((RC_ENGAGE_ON == prev_engage) || (RC_ENGAGE_STANDBY == prev_engage)) ? 1u : 0u;

    switch (RC_Engage)
    {
        case RC_ENGAGE_ON:
            /* 每圈 acquire 一次就是续租；抢占成功时 proto.c 会给上位机发一条 EVENT。 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 遥控接管：双环使能 + 遥控给目标 */
            spd_safety_set_local(1);
            steer_en_cmd = 1.0f;
            RC_Target_Rps = rc_throttle_to_target();

            /* 见 rc_apply_steer() 与 remote_ctrl.h 三·五。 */
            rc_apply_steer();

            /* κ = tan(实测δ)/L，v_l = v(1−κW/2)、v_r = v(1+κW/2)
             * ⇒ 进来乘 `m_per_rev`、出去除回去。漏一边就是 1.8 倍误差（cpr 不是 1）。
             * STANDBY/OFF 的"不写"分支原样保留（见本文件前面的长注释）。
             */
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
            /* 免得上位机在等油门回中的这段时间把目标写进来（两个人抢方向盘）。 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 待命：双环使能（方向盘会主动回中），目标**只在入口那一拍**压 0。
             * 驱动令牌归遥控，上位机的 P_CMD 写入会被 NAK(busy)。要交还控制权，
             * 把 CH6 收回（-> OFF，本函数会释放令牌）即可。
             */

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
             * 1) 本地档位放掉 —— 控制权交还上位机（急停那一路已经被硬钉住）；
             * 2) 若是从"遥控持有控制权"退出来的，在**这一拍**把目标压 0
             * （摇杆还推着但接收机没了，必须停下来），同时转向失能（方向盘自由）；
             * 3) 之后**不再碰**这三个量 —— 否则上位机的滑条就全废了。
             */
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

/* 立刻解除接管（上电 / 停机路径调用） 上电路径先 release 一次是刻意的：保证上电初态是"全部放开"。 */
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
