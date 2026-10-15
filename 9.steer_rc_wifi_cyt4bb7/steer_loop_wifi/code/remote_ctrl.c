/*********************************************************************************************************************
 * 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS）实现
 * 通道分配、标定值、急停、接管状态机、与 WiFi 的授权关系，全部见 remote_ctrl.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "remote_ctrl.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"

/*===================================================================================================================
 *  对外数据
 *=================================================================================================================*/
volatile uint16  RC_Raw_Ch1     = 0;
volatile uint16  RC_Raw_Ch2     = 0;
volatile uint16  RC_Raw_Ch4     = 0;
volatile uint16  RC_Raw_Ch5     = 0;
volatile uint16  RC_Raw_Ch6     = 0;
volatile float   RC_Steer_Deg   = 0.0f;
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

/*===================================================================================================================
 *  内部工具
 *=================================================================================================================*/

/*----------------------------------------------------------
 * 原始值 -> 归一化(-1.0 ~ +1.0)
 * 中位两侧量程不对称（方向盘 776:824），所以两侧分别归一化，
 * 这样两端都能吃满 ±1.0，不会有一边提前截断。**照抄队友的算法**。
 *----------------------------------------------------------*/
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

/*----------------------------------------------------------
 * CH4 油门行程旋钮 -> 速度缩放
 *   旋钮低 -> RC_SCALE_MIN，中 -> 两者中点，高 -> RC_SCALE_MAX。
 *   用 rc_norm 拿到 -1~+1 再线性映射到 [MIN, MAX]，两端对称。
 *   ⚠ CH3（方向行程）**用户明确说先不做**，所以这里不读它；
 *     但遥测/打印里预留了位置，将来要做时在这里加一个 rc_steer_travel() 即可。
 *----------------------------------------------------------*/
static float rc_thr_travel_scale (void)
{
    float n = rc_norm(RC_Raw_Ch4, RC_SW_VAL_MID, RC_SW_VAL_LOW, RC_SW_VAL_HIGH);
    float s = RC_SCALE_MIN + ((n + 1.0f) * 0.5f) * (RC_SCALE_MAX - RC_SCALE_MIN);

    if (s < RC_SCALE_MIN) { s = RC_SCALE_MIN; }
    if (s > RC_SCALE_MAX) { s = RC_SCALE_MAX; }

    return s;
}

/*----------------------------------------------------------
 * 油门归一化值 -> 后轮目标 rps（带死区 + 速度缩放 + 上限夹紧）
 *   队友是 ±1.0 线性映射到固定常数；本工程多了"CH4 缩放"与 spd_target_max 夹紧。
 *----------------------------------------------------------*/
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

/*===================================================================================================================
 *  接口实现
 *=================================================================================================================*/

/*----------------------------------------------------------
 * 初始化：清状态 + 起接收机
 * 接收机：UART_4 = SCB2，RX = P14.0，SBUS 100 kbps，帧间隔计时用 TC_TIME2_CH2
 *        （以上引脚/通道都在 zf_device_uart_receiver.h 里，逐飞库自带，不在这里重定义）
 * ⚠ 必须在 5ms PIT 起来之前调用（接收机会注册帧间隔计数器）
 *----------------------------------------------------------*/
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

/*----------------------------------------------------------
 * 从接收机快照换算一次（本模块不主动碰驱动，只读 uart_receiver）
 *----------------------------------------------------------*/
void remote_ctrl_update (void)
{
    int thr;

    RC_Raw_Ch1 = uart_receiver.channel[RC_CH_STEER];
    RC_Raw_Ch2 = uart_receiver.channel[RC_CH_THROTTLE];
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
       ⚠ 用 steer_ctrl_eff_limit() 而不是 steer.limit_deg：那是**全系统唯一出口**，
         已经保证"参数怎么改都越不过 ±27"（见 steer_abs.h / steer_ctrl.h）。 */
    {
        int d = (int)RC_Raw_Ch1 - RC_STEER_MID;

        if ((d <= RC_STEER_DEADBAND) && (d >= -RC_STEER_DEADBAND))
        {
            /* 死区内 = 方向盘回中：不跟着手上/电位器的零点漂移走。
               （没有这一段时，静止时方向盘会被 ±1~2 个计数的抖动顶到零点几度） */
            RC_Steer_Deg = 0.0f;
        }
        else
        {
            RC_Steer_Deg = (float)RC_STEER_DIR_SIGN
                         * rc_norm(RC_Raw_Ch1, RC_STEER_MID, RC_STEER_MIN, RC_STEER_MAX)
                         * steer_ctrl_eff_limit();
        }
    }

    /* 油门：同样 ±RC_THR_DEADBAND 死区，把静止抖动压成 0（队友实测中位在 988/990/992 之间跳） */
    thr = (int)RC_Raw_Ch2 - RC_THR_MID;
    if ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND))
    {
        RC_Throttle = 0.0f;
    }
    else
    {
        RC_Throttle = rc_norm(RC_Raw_Ch2, RC_THR_MID, RC_THR_MIN, RC_THR_MAX);
    }

    /* CH4 旋钮 -> 速度缩放 */
    RC_Scale = rc_thr_travel_scale();
}

/*----------------------------------------------------------
 * 接管状态机
 *   优先级（从高到低）：
 *     1) 接收机失控（ST=0，含"从来没收到过帧"）  -> FAILSAFE（锁存）
 *     2) 急停生效 / 急停未回位                  -> ESTOP（锁存）
 *     3) 使能开关收回 / 上位机禁了遥控            -> OFF
 *     4) 合闸（CH6 合闸 + 在线 + 授权）           -> OFF 先 STANDBY；油门回中才升 ON
 *   ⚠ FAILSAFE 与 ESTOP 都是**锁存**的，且都要求"先把开关收回去"才有机会恢复 ——
 *     失控/急停过就一定是哪里不对，不该让它自动恢复。
 *----------------------------------------------------------*/
static void rc_state_machine (void)
{
    uint8 armed;

    /* 1) 接收机报告失控（或从来没收到过帧 -> state 恒 0）*/
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

/*----------------------------------------------------------
 * 跑状态机 + 写目标 + 推急停/本地接管档位
 *
 * ★★ 授权关系（见 remote_ctrl.h 的长注释）：
 *     · 遥控 ON          -> spd_safety_set_local(1)：把"上位机在场"那三道门换成"本地授权"
 *     · 退出 / 失控 / 急停 -> spd_safety_set_local(0)：完整五道门原样恢复
 *   => 两条路合起来 = 「WiFi 在场 OR 遥控在场」，两个都不在才真的不动。
 *   [!] 心跳门与限幅门**不参与**替换，永远生效。
 *
 * ★★ 目标量（spd_target_l / spd_target_r / steer.target_delta）**只在"遥控真的在开车"时写**：
 *     · ON                 每拍写 —— 遥控就是当前驾驶员（全工程唯一一处每拍写目标的地方）；
 *     · 进 STANDBY 那一拍、以及从 ON/STANDBY **退出**那一拍 —— 写 0 一次
 *       （"停住 + 方向盘回中"的语义，管住"摇杆还推着但接收机突然没了"）；
 *     · 其余任何时刻 —— **一个字都不许写**。
 *
 *   [!][!] 这条是硬约束：这三个量**上位机也在写**（参数 target_l / target_r / steer_tgt
 *        就是按地址直接下发到这三个变量的）。遥控模块每 5ms 无条件压 0，等于把上位机
 *        那三个滑条全废掉，而且**不报任何错**：
 *          2026-10-13 现场症状 —— 上位机把"左轮转速"拖到 2.85，车端遥测 target_l 恒 0。
 *          根因：接收机没插时 RC_Online 恒 0 => 状态机恒为 FAILSAFE => 旧代码的
 *          ESTOP/FAILSAFE/OFF 三个分支每拍 spd_target_l = 0，刚写进去的值下一拍就被清掉。
 *        语义上也要分清：**"遥控没在开车" != "车必须停"**。车要停有急停门（硬钉输出）
 *        和占空比限幅门，不该靠"霸占目标量"来实现。
 *----------------------------------------------------------*/
void remote_ctrl_apply (void)
{
    static uint8 prev_engage = 0xFFu;       /* 上一拍状态(0xFF = 本函数还没跑过) */
    uint8        held;                      /* 上一拍是不是"遥控持有控制权" */

    rc_state_machine();

    /* 急停是**全局**的：不管此刻谁在开车，先把两个环硬停掉。
       放在 switch 之前 -> 即使 ESTOP 之后状态机立刻降回 OFF，本拍也仍然先把急停推住。 */
    spd_safety_set_estop(RC_Estop);
    steer_ctrl_set_estop(RC_Estop);

    held = ((RC_ENGAGE_ON == prev_engage) || (RC_ENGAGE_STANDBY == prev_engage)) ? 1u : 0u;

    switch (RC_Engage)
    {
        case RC_ENGAGE_ON:
            /* 遥控接管：双环使能 + 遥控给目标 */
            spd_safety_set_local(1);
            steer_en_cmd = 1.0f;
            RC_Target_Rps = rc_throttle_to_target();
            steer.target_delta = RC_Steer_Deg;
            spd_target_l = RC_Target_Rps;
            spd_target_r = RC_Target_Rps;
            break;

        case RC_ENGAGE_STANDBY:
            /* 待命：双环使能（方向盘会主动回中），目标**只在入口那一拍**压 0，之后放手。
               放手是给上位机让路 —— 遥控还没给油门前，上位机想开车就让它开。 */
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
            /* OFF / ESTOP / FAILSAFE：
               1) 本地档位放掉 —— 控制权交还上位机（急停那一路已经被硬钉住）；
               2) 若是从"遥控持有控制权"退出来的，在**这一拍**把目标压 0
                  （摇杆还推着但接收机没了，必须停下来），同时转向失能（方向盘自由）；
               3) 之后**不再碰**这三个量 —— 否则上位机的滑条就全废了。 */
            spd_safety_set_local(0);
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

/*----------------------------------------------------------
 * 立刻解除接管（上电 / 停机路径调用）
 * ⚠ 会把**急停也放开** —— 这是唯一能放开急停的地方（正常运行时只有 CH5 回位能放）。
 *   上电路径先 release 一次是刻意的：保证上电初态是"全部放开"。
 *----------------------------------------------------------*/
void remote_ctrl_release (void)
{
    RC_Engage     = RC_ENGAGE_OFF;
    RC_Target_Rps = 0.0f;

    spd_safety_set_estop(0);
    steer_ctrl_set_estop(0);
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
