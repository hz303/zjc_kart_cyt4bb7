/**********************************************************************************************************************
 * 双电机速度环(WiFi) - 保护与门控实现: 5ms 判一次状态, 门控与位定义见 spd_safety.h
 *********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spd_safety.h"
#include "motor_link.h"
#include "motor_drv.h"

volatile float  spd_duty_limit_pct = SPD_DUTY_DEFAULT;
volatile float  spd_lost_ms        = 600.0f;
volatile float  spd_auto_stop      = 1.0f;

static volatile uint32  s_flags      = 0;
static volatile uint32  s_trip_msg   = 0;       /* 记录新发生的致命门控        */
static volatile uint32  s_last_hb_ms = 0;       /* 上次主循环心跳时刻          */
static volatile uint8   s_hb_seen    = 0;
static uint32           s_cfg_bad    = 0;
static volatile uint8   s_local_mask = 0;   /* 本地接管位图: 0x01 遥控 / 0x02 回放 PWR */
static volatile uint8   s_estop      = 0;       /* 遥控急停(全局硬停机, 替不掉)    */

#define SPD_HB_TIMEOUT_MS   (200u)              /* 主循环心跳超时 */

/* 配置合法性自检(运行期做, 不能用 #if 里的浮点常量)
   bit0 通道数超上限  bit1 发送周期过短  bit2 接收周期过短  bit3 PWM 过低  bit4 PWM 过高 */
static uint32 spd_safety_cfg_check (void)
{
    uint32 bad = 0;

    if (MOTOR_CH_COUNT > 48)                { bad |= 0x01u; }   /* 遥测帧长 = 11 + 10 + 4*CH_COUNT, 48 通道 = 213 B, 仍在 motor_txbuf[256] 内 */
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
    s_local_mask = 0;                       /* 上电默认不接管, 一律走完整五道门 */
    s_cfg_bad    = spd_safety_cfg_check();

    if (0 != s_cfg_bad)
    {
        s_flags |= SAFE_BIT_CFG_BAD;
    }
}

/* 本地接管按来源点灯/灭灯, 谁都不能灭别人的灯; 只换 1)/2)/3) 三道门 */
void spd_safety_set_local_src (uint8 src, uint8 on)
{
    if (0u != on) { s_local_mask = (uint8)(s_local_mask | src); }
    else          { s_local_mask = (uint8)(s_local_mask & (uint8)(~src)); }
}

/* 兼容旧调用, 只知道 on/off 的调用方记成遥控那一位 */
void spd_safety_set_local (uint8 on)
{
    spd_safety_set_local_src(SPD_LOCAL_SRC_RC, on);
}

uint8 spd_safety_local (void)
{
    return (0u != s_local_mask) ? 1u : 0u;
}

/* 遥控急停: 全局硬停机; estop=1 时无论 local 是什么都停机, local 换不掉它 */
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

/* 5ms 保护判定: 放在测速/控制之前, 保证控制器用的是本拍结论 */
void spd_safety_tick (void)
{
    uint32 now;
    uint32 f = 0;
    uint16 en;

    now = motor_link_now_ms();

    /* 0) 遥控急停: 最高优先级, 与 local / 上位机门都无关, 直接亮位停机 */
    if (0 != s_estop)
    {
        f |= SAFE_BIT_ESTOP;
    }

    if (0 != s_local_mask)
    {
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
        /* 任何一道门不过 -> 该路不出力; 两路共用同一套门控。ESTOP 不在本地接管可豁免的集合内 */
    uint32 block = SAFE_BIT_NEVER_RX | SAFE_BIT_NOT_EN | SAFE_BIT_LOST |
                   SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD | SAFE_BIT_ESTOP;

    (void)ch;

    if (0 != s_local_mask)
    {
        /* 本地接管: 1)/2)/3) 三道上位机门不参与判定; 4)/5) 与急停照旧 */
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

        /* 瞬时位: spd_safety_tick() 每拍重建 s_flags 时清掉, 只反映本拍 */
    s_flags |= SAFE_BIT_OVER_LIM;
}
