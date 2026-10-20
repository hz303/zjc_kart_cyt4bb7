/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 保护与门控实现
 * 设计说明见 spd_safety.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spd_safety.h"
#include "motor_link.h"
#include "motor_drv.h"

volatile float  spd_duty_limit_pct = SPD_DUTY_DEFAULT;
volatile float  spd_lost_ms        = 600.0f;
volatile float  spd_auto_stop      = 1.0f;

static volatile uint32  s_flags      = 0;       /* 当前保护状态位图            */
static volatile uint32  s_trip_msg   = 0;       /* 新发生的事件(被主循环取走)  */
static volatile uint32  s_last_hb_ms = 0;       /* 上次主循环心跳时刻          */
static volatile uint8   s_hb_seen    = 0;       /* 是否已经收到过心跳          */
static uint32           s_cfg_bad    = 0;       /* 配置自检结果                */
static volatile uint8   s_local      = 0;       /* ★ 本地接管档位(跳过上位机三道门) */
static volatile uint8   s_estop      = 0;       /* ★ 遥控急停(全局硬停机, 替不掉)   */

#define SPD_HB_TIMEOUT_MS   (200u)              /* 主循环心跳超时 */

/*-------------------------------------------------------------------------------------------------------------------
 *  配置合法性自检(运行期做, 不能用 #if 里的浮点常量)
 *     bit0 遥测通道数超协议上限
 *     bit1 发送周期过短(会把主循环拖垮)
 *     bit2 接收周期过短
 *     bit3 PWM 频率过低(会听见啸叫)
 *     bit4 PWM 频率过高(超过听频上限没必要)
 *-----------------------------------------------------------------------------------------------------------------*/
static uint32 spd_safety_cfg_check (void)
{
    uint32 bad = 0;

    if (MOTOR_CH_COUNT > 48)                { bad |= 0x01u; }   /* 原上限 32; 2026-10-13 加遥控通道后放到 48。
                                                                 依据: 遥测帧总长 = 11 + 10 + 4*CH_COUNT, 34 -> 157 B,
                                                                 发送缓冲 motor_txbuf[256] 与 WiFi 模块单帧(~200 B)都还够用。 */
    if (MOTOR_TX_PERIOD_MS < 5)             { bad |= 0x02u; }
    if (MOTOR_RX_PERIOD_MS < 1)             { bad |= 0x04u; }
    if (MOTOR_PWM_FREQ_HZ < 1000)           { bad |= 0x08u; }
    if (MOTOR_PWM_FREQ_HZ > 20000)          { bad |= 0x10u; }

    return bad;
}

void spd_safety_init (void)
{
    s_flags      = 0;
    s_trip_msg   = 0;
    s_last_hb_ms = 0;
    s_hb_seen    = 0;
    s_local      = 0;                       /* 上电默认【不】接管: 一律走完整五道门 */
    s_cfg_bad    = spd_safety_cfg_check();

    if (0 != s_cfg_bad)
    {
        s_flags |= SAFE_BIT_CFG_BAD;
    }
}

/* ★ 本地接管档位。由主循环每圈按"当前有没有本地驱动源"推一次(见 remote_ctrl.c)。
   ⚠ 这里只换 1)/2)/3) 三道"上位机门"; 心跳与限幅不受它影响。 */
void spd_safety_set_local (uint8 on)
{
    s_local = on ? 1 : 0;
}

uint8 spd_safety_local (void)
{
    return s_local;
}

/* ★★ 遥控急停：全局硬停机。见 spd_safety.h / remote_ctrl.h。
   ⚠ 与 s_local 的关系是**单向**的：estop=1 时无论 local 是什么都停机；
     local 只能替换"上位机三道门"，永远换不掉急停。 */
void spd_safety_set_estop (uint8 on)
{
    s_estop = on ? 1 : 0;
}

uint8 spd_safety_estop (void)
{
    return s_estop;
}

void spd_safety_heartbeat (void)
{
    s_last_hb_ms = motor_link_now_ms();
    s_hb_seen    = 1;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  5ms 保护判定。放在测速/控制之前, 保证控制器用的是本拍的结论。
 *-----------------------------------------------------------------------------------------------------------------*/
void spd_safety_tick (void)
{
    uint32 now;
    uint32 f = 0;
    uint16 en;

    now = motor_link_now_ms();

    /* 0) ★ 遥控急停：**最高优先级**，与 local / 上位机门都无关，直接亮位停机 */
    if (0 != s_estop)
    {
        f |= SAFE_BIT_ESTOP;
    }

    if (0 != s_local)
    {
        /* ★ 本地接管: 跳过 1)/2)/3) 三道"上位机门", 只亮信息位。
           4)/5) 照旧在下面判 —— 那是"MCU 自己还活着"的门, 不许绕。 */
        f |= SAFE_BIT_LOCAL;
    }
    else
    {
        /* 1) 上位机说过话吗 */
        if (0 == motor_link_ever_rx())
        {
            f |= SAFE_BIT_NEVER_RX;
        }

        /* 2) 使能位: bit1 电机 且 bit2 总使能 */
        en = motor_link_enable_bits();
        if ((0 == (en & 0x0002u)) || (0 == (en & 0x0004u)))
        {
            f |= SAFE_BIT_NOT_EN;
        }

        /* 3) 失联自停 */
        if (spd_auto_stop > 0.5f)
        {
            if (0 == motor_link_online())
            {
                f |= SAFE_BIT_LOST;
            }
        }
    }

    /* 4) 主循环心跳。没收到过心跳也算不通过(说明主循环还没起来) */
    if (0 == s_hb_seen)
    {
        f |= SAFE_BIT_NO_HB;
    }
    else if ((uint32)(now - s_last_hb_ms) > SPD_HB_TIMEOUT_MS)
    {
        f |= SAFE_BIT_NO_HB;
    }

    /* 5) 配置 */
    if (0 != s_cfg_bad)
    {
        f |= SAFE_BIT_CFG_BAD;
    }

    /* 新发生的"致命门控"记一次事件, 供主循环打印(只报一次, 不刷屏) */
    {
        static uint32 s_prev = 0;
        uint32 fatal_mask = SAFE_BIT_LOST | SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD | SAFE_BIT_ESTOP;
        uint32 newly = (f & fatal_mask) & (~s_prev);

        if (0 != newly)
        {
            s_trip_msg |= newly;
        }
        s_prev = f;
    }

    s_flags = f;
}

float spd_safety_limit_u (void)
{
    float p = spd_duty_limit_pct;

    if (p < 0.0f)               { p = 0.0f; }
    if (p > SPD_DUTY_HARD_MAX)  { p = SPD_DUTY_HARD_MAX; }

    return p * 0.01f;
}

uint8 spd_safety_allow (uint8 ch)
{
    /* 任何一道门不过 -> 该路不出力。两路共用同一套门控(同一辆车, 不该一路让一路不让)。
       ⚠ SAFE_BIT_ESTOP 在列表里，而且**不在下面那个"本地接管可豁免"的集合中** ——
         急停永远有效，这正是设计意图。 */
    uint32 block = SAFE_BIT_NEVER_RX | SAFE_BIT_NOT_EN | SAFE_BIT_LOST |
                   SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD | SAFE_BIT_ESTOP;

    (void)ch;

    if (0 != s_local)
    {
        /* 本地接管: 1)/2)/3) 三道"上位机门"不参与判定; 4)/5) 与急停照旧 */
        block &= ~(SAFE_BIT_NEVER_RX | SAFE_BIT_NOT_EN | SAFE_BIT_LOST);
    }

    return (0 != (s_flags & block)) ? 0 : 1;
}

uint32 spd_safety_flags (void)
{
    return s_flags;
}

uint32 spd_safety_trip_take (void)
{
    uint32 t = s_trip_msg;

    s_trip_msg = 0;
    return t;
}

uint8 spd_safety_cfg_bad (void)
{
    return (uint8)s_cfg_bad;
}

void spd_safety_note_clip (uint8 ch)
{
    (void)ch;

    /* 注意: 这是**瞬时位**, 由 spd_safety_tick() 每拍重建 s_flags 时清掉,
       所以"这一拍被限幅夹了"只反映本拍。想统计频率应该在上位机看 duty_limit 与 u 的关系。 */
    s_flags |= SAFE_BIT_OVER_LIM;
}
