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

    if (MOTOR_CH_COUNT > 32)                { bad |= 0x01u; }
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
    s_cfg_bad    = spd_safety_cfg_check();

    if (0 != s_cfg_bad)
    {
        s_flags |= SAFE_BIT_CFG_BAD;
    }
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
        uint32 fatal_mask = SAFE_BIT_LOST | SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD;
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
    (void)ch;

    /* 任何一道门不过 -> 该路不出力。两路共用同一套门控(同一辆车, 不该一路让一路不让) */
    if (0 != (s_flags & (SAFE_BIT_NEVER_RX | SAFE_BIT_NOT_EN | SAFE_BIT_LOST | SAFE_BIT_NO_HB | SAFE_BIT_CFG_BAD)))
    {
        return 0;
    }

    return 1;
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
