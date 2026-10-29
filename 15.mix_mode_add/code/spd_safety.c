/* 双电机速度环 - 保护与门控实现; 门控规则见 spd_safety.h */

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
static volatile uint8   s_local_mask = 0;       /* 本地接管来源位图             */
static volatile uint8   s_estop      = 0;       /* 遥控急停, 全局硬停机         */

#define SPD_HB_TIMEOUT_MS   (200u)              /* 主循环心跳超时 ms */

/* 配置自检(运行期做): bit0 通道数超限, bit1 发送周期过短, bit2 接收周期过短,
   bit3 PWM 频率过低, bit4 PWM 频率过高 */
static uint32 spd_safety_cfg_check (void)
{
    uint32 bad = 0;

    if (MOTOR_CH_COUNT > 48)                { bad |= 0x01u; }
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
    s_local_mask = 0;                       /* 上电默认不接管, 走完整五道门 */
    s_cfg_bad    = spd_safety_cfg_check();

    if (0 != s_cfg_bad)
    {
        s_flags |= SAFE_BIT_CFG_BAD;
    }
}

/* 本地接管来源位图: 各来源独立置位/清零, 互不影响; 只跳过上位机三道门 */
void spd_safety_set_local_src (uint8 src, uint8 on)
{
    if (0u != on) { s_local_mask = (uint8)(s_local_mask | src); }
    else          { s_local_mask = (uint8)(s_local_mask & (uint8)(~src)); }
}

void spd_safety_set_local (uint8 on)
{
    spd_safety_set_local_src(SPD_LOCAL_SRC_RC, on);
}

uint8 spd_safety_local (void)
{
    return (0u != s_local_mask) ? 1u : 0u;
}

/* 急停优先于本地接管: local 只能换掉上位机三道门, 换不掉急停 */
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

/* 5 ms 保护判定: 在测速与控制之前执行, 保证控制器用的是本拍结论 */
void spd_safety_tick (void)
{
    uint32 now;
    uint32 f = 0;
    uint16 en;

    now = motor_link_now_ms();

    /* 遥控急停: 最高优先级, 与 local 及上位机门无关, 直接亮位停机 */
    if (0 != s_estop)
    {
        f |= SAFE_BIT_ESTOP;
    }

    if (0 != s_local_mask)
    {
        /* 本地接管: 跳过上位机三道门只亮信息位; 心跳与限幅照旧判定 */
        f |= SAFE_BIT_LOCAL;
    }
    else
    {
        if (0 == motor_link_ever_rx())
        {
            f |= SAFE_BIT_NEVER_RX;
        }

        /* 使能位: bit1 电机 且 bit2 总使能 */
        en = motor_link_enable_bits();
        if ((0 == (en & 0x0002u)) || (0 == (en & 0x0004u)))
        {
            f |= SAFE_BIT_NOT_EN;
        }

        if (spd_auto_stop > 0.5f)
        {
            if (0 == motor_link_online())
            {
                f |= SAFE_BIT_LOST;
            }
        }
    }

    /* 主循环心跳: 未收到过也算不通过(主循环还没起来) */
    if (0 == s_hb_seen)
    {
        f |= SAFE_BIT_NO_HB;
    }
    else if ((uint32)(now - s_last_hb_ms) > SPD_HB_TIMEOUT_MS)
    {
        f |= SAFE_BIT_NO_HB;
    }

    if (0 != s_cfg_bad)
    {
        f |= SAFE_BIT_CFG_BAD;
    }

    /* 新发生的致命门控记一次事件, 供主循环打印, 不重复报 */
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
    /* 任一道门不过则不出力; 本地接管只豁免上位机三道门, 急停不可豁免 */
    uint32 block = SAFE_BIT_NEVER_RX | SAFE_BIT_NOT_EN | SAFE_BIT_LOST |
                   SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD | SAFE_BIT_ESTOP;

    (void)ch;

    if (0 != s_local_mask)
    {
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

    /* 瞬时位: spd_safety_tick() 每拍重建 s_flags 时清掉, 只反映本拍是否被夹 */
    s_flags |= SAFE_BIT_OVER_LIM;
}
