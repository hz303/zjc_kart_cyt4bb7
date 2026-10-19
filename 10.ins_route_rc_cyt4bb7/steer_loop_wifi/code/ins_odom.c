/*********************************************************************************************************************
 * C 车惯导 + 路表 - 航位推算实现
 * 坐标约定、Polaris 缺陷修正、比例尺说明 见 ins_odom.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include "ins_odom.h"
#include "ins_imu.h"
#include "spd_enc.h"

#define DEG2RAD_F   (0.017453292519943295f)

/*===================================================================================================================
 *  ★★ 临界区：**绝不要用逐飞的 `interrupt_global_disable()` / `interrupt_global_enable(pm)` 配对**
 *
 *  原因（见 libraries/zf_common/zf_common_interrupt.c）—— 那一对不是"保存/恢复 PRIMASK"，
 *  而是一个自增自减的计数器，且 `enable` 只在参数为 0 时才真的开中断：
 *
 *      void interrupt_global_enable (uint32 primask)
 *      {
 *          if(primask == 0) { __enable_irq(); }    // ★ 只有传 0 才开中断
 *          interrupt_global_flag ++;
 *      }
 *      uint32 interrupt_global_disable (void)
 *      {
 *          __disable_irq();
 *          if(interrupt_global_flag > 0) interrupt_global_flag --;
 *          return interrupt_global_flag;           // ★ 返回计数器，不是 PRIMASK
 *      }
 *
 *  ⇒ `clock_init()` 里调过一次 `interrupt_global_enable(0)`，`main()` 里又调一次，
 *    计数器已经 ≥ 2。此后写 `pm = disable(); ...; enable(pm);`，`pm` 拿到的是 1（非 0）
 *    ⇒ **`__enable_irq()` 不会执行 ⇒ 中断被永久关死，且零报错**。
 *    症状：主循环照跑，但 5ms 中断再不进入 ⇒ 速度环/编码器/IMU 全停、`motor_tick_ms` 不涨、
 *    WiFi 一帧不发（`motor_link_task()` 连 if 体都不进）。
 *
 *  ⇒ 正确做法就是下面两行：直接存取 PRIMASK（CMSIS 标准做法，可任意嵌套）。
 *=================================================================================================================*/
static uint32 ins_crit_enter (void) { uint32 pm = __get_PRIMASK(); __disable_irq(); return pm; }
static void   ins_crit_exit  (uint32 pm) { __set_PRIMASK(pm); }

volatile float  ins_m_per_rev   = INS_MPM_DEFAULT;  /* ★ 工程 10：2026-10-15 实测定稿 0.8193（2.815 m / 6383.5 cnt，cpr 1858），见 ins_odom.h */
volatile float  ins_odom_enable = 1.0f;
volatile float  ins_slip_pct    = 30.0f;

/* ★ 工程 10 新增：位姿归零命令（参数表 ID 37）。0->1 边沿触发一次，
   由 main 的慢速决策层消费（ins_odom_reset() + ins_imu_zero_yaw()）—— 与工程 8 同一套写法。
   ⚠ 归零必须"位姿 + 航向"一起归：只归一个，后面所有相对量（路表、回放）就整体错位。 */
volatile float  ins_zero_cmd    = 0.0f;

static volatile float s_x        = 0.0f;
static volatile float s_y        = 0.0f;
static volatile float s_yaw      = 0.0f;
static volatile float s_dist_abs = 0.0f;
static volatile float s_v_mps    = 0.0f;
static volatile uint8 s_flags    = 0;

/* 静止计数（连续多少拍两轮都没计数） */
static uint16   s_still_cnt = 0;
#define ODOM_STILL_TICKS    (10)                /* 10 拍 = 50ms 没动才算静止 */

void ins_odom_init (void)
{
    s_x = 0.0f;
    s_y = 0.0f;
    s_yaw = 0.0f;
    s_dist_abs = 0.0f;
    s_flags = ODOM_ST_ACTIVE;
    s_still_cnt = 0;

    /* 把编码器累计计数归零，让 pos 与我们的积分基准对齐 */
    spd_enc_reset_pos(&enc_l);
    spd_enc_reset_pos(&enc_r);
}

void ins_odom_reset (void)
{
    s_x = 0.0f;
    s_y = 0.0f;
    s_dist_abs = 0.0f;
    s_flags &= (uint8)~ODOM_ST_SLIP;
    s_flags |=  ODOM_ST_ACTIVE;

    /* 位置清零时同步把编码器累计计数也清了，避免下次更新时把
       "清零前就攒下的计数" 一次性吃进来（Polaris 在 set_active 里专门处理过这个）。 */
    spd_enc_reset_pos(&enc_l);
    spd_enc_reset_pos(&enc_r);
}

void ins_odom_set_origin (float x, float y, float yaw_deg)
{
    uint32 primask = ins_crit_enter();

    s_x   = x;
    s_y   = y;
    s_yaw = yaw_deg;

    ins_crit_exit(primask);

    spd_enc_reset_pos(&enc_l);
    spd_enc_reset_pos(&enc_r);
}

/*-------------------------------------------------------------------------------------------------------------------
 *  5ms 每拍
 *  必须在 spd_ctrl_tick() 之后调用 —— 那里面的 spd_enc_update() 才是本拍的 Δcnt。
 *-----------------------------------------------------------------------------------------------------------------*/
void ins_odom_update (void)
{
    float dl_m, dr_m, ds;
    float yaw_deg;
    int16 dl, dr;
    uint8 flags;

    /* --- 航向（用累积角；-180~180 的那个在跨边界会跳） --- */
    yaw_deg = ins_imu_get_yaw();

    /* --- 里程：左右各自换算成米，再取车体中心（左右 cpr 可以不同） --- */
    dl = enc_l.delta;
    dr = enc_r.delta;

    dl_m = ((float)dl / ((enc_l.cpr > 1.0f) ? enc_l.cpr : 1.0f)) * ins_m_per_rev;
    dr_m = ((float)dr / ((enc_r.cpr > 1.0f) ? enc_r.cpr : 1.0f)) * ins_m_per_rev;

    ds = (dl_m + dr_m) * 0.5f;

    flags = 0;

    /* --- 静止判据（只用于打状态位，不冻结积分：编码器里程本来就不像纯惯导那样漂）--- */
    if ((0 == dl) && (0 == dr))
    {
        if (s_still_cnt < 0xFFFFu) { s_still_cnt ++; }
    }
    else
    {
        s_still_cnt = 0;
    }
    if (s_still_cnt >= ODOM_STILL_TICKS) { flags |= ODOM_ST_STILL; }

    /* --- 打滑判据：左右计数差很大、但航向几乎没变 ⇒ 大概率一侧打滑。**只报警** --- */
    {
        int32 al = (dl < 0) ? -(int32)dl : (int32)dl;
        int32 ar = (dr < 0) ? -(int32)dr : (int32)dr;
        int32 mx = (al > ar) ? al : ar;

        if (mx > 5)                                     /* 计数太小不算，避免噪声 */
        {
            float diff_pct = (float)((al > ar) ? (al - ar) : (ar - al)) * 100.0f / (float)mx;
            float dyaw     = fabsf(yaw_deg - s_yaw);

            if (dyaw > 180.0f) { dyaw = 360.0f - dyaw; }

            if ((diff_pct > ins_slip_pct) && (dyaw < 1.0f))
            {
                flags |= ODOM_ST_SLIP;
            }
        }
    }

    /* --- 积分（带符号的 ds —— 这是相对 Polaris 的关键修正） --- */
    if (ins_odom_enable > 0.5f)
    {
        float rad = yaw_deg * DEG2RAD_F;

        s_x += -sinf(rad) * ds;
        s_y +=  cosf(rad) * ds;

        s_dist_abs += fabsf(ds);                        /* 总路程另算，与位置积分分开 */
    }

    s_yaw   = yaw_deg;
    s_v_mps = ds / 0.005f;                          /* 本拍车速（带符号） */
    s_flags = (uint8)(flags | ODOM_ST_ACTIVE);
}

/*-------------------------------------------------------------------------------------------------------------------
 *  查询
 *-----------------------------------------------------------------------------------------------------------------*/
float ins_odom_get_x    (void) { return s_x; }
float ins_odom_get_y    (void) { return s_y; }
float ins_odom_get_yaw  (void) { return s_yaw; }
float ins_odom_get_dist (void) { return s_dist_abs; }
float ins_odom_get_v_mps(void) { return s_v_mps; }
uint8 ins_odom_flags    (void) { return s_flags; }

float ins_odom_get_km (void)
{
    /* 诊断用：当前生效的"米/计数"（左右取平均；两侧 cpr 相同时就是它本身） */
    float cpr_avg = (enc_l.cpr + enc_r.cpr) * 0.5f;

    if (cpr_avg < 1.0f) { cpr_avg = 1.0f; }
    return ins_m_per_rev / cpr_avg;
}

void ins_odom_snapshot (ins_odom_snap_t *out)
{
    uint32 primask;

    if (NULL == out) { return; }

    /* ★ 见 ins_odom.h 缺陷 2：x/y/yaw/dist 必须一次性复制，
       否则会被 5ms 中断插到半路，拿到"旧 x + 新 yaw"的撕裂值。 */
    primask = ins_crit_enter();
    out->x        = s_x;
    out->y        = s_y;
    out->yaw      = s_yaw;
    out->dist_abs = s_dist_abs;
    out->v_mps    = s_v_mps;
    out->flags    = s_flags;
    ins_crit_exit(primask);
}
