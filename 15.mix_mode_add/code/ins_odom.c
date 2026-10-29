/*********************************************************************************************************************
 * 航位推算实现: IMU 航向 × 编码器里程 → 米制位姿
 * 坐标约定与比例尺说明见 ins_odom.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include "ins_odom.h"
#include "ins_imu.h"
#include "spd_enc.h"

#define DEG2RAD_F   (0.017453292519943295f)

/* 临界区: 直接存取 PRIMASK, 可任意嵌套; 逐飞 interrupt_global_* 为计数器语义, 不能配对使用 */
static uint32 ins_crit_enter (void) { uint32 pm = __get_PRIMASK(); __disable_irq(); return pm; }
static void   ins_crit_exit  (uint32 pm) { __set_PRIMASK(pm); }

volatile float  ins_m_per_rev   = INS_MPM_DEFAULT;  /* 米/转, 比例尺参数, 见 ins_odom.h */
volatile float  ins_odom_enable = 1.0f;
volatile float  ins_slip_pct    = 30.0f;

/* 位姿归零命令, 0->1 边沿触发一次, 由主循环消费(ins_odom_reset() + ins_imu_zero_yaw())
   位姿与航向必须一起归零, 否则后续相对量整体错位 */
volatile float  ins_zero_cmd    = 0.0f;

static volatile float s_x        = 0.0f;
static volatile float s_y        = 0.0f;
static volatile float s_yaw      = 0.0f;
static volatile float s_dist_abs = 0.0f;
static volatile float s_v_mps    = 0.0f;
static volatile uint8 s_flags    = 0;

/* 连续多少拍两轮都没计数 */
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

    /* 编码器累计计数归零, 与积分基准对齐 */
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

    /* 位置清零时同步清编码器累计计数, 避免下次更新一次性吃进清零前的计数 */
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

/* ---- 5 ms 每拍 ----
 * 必须在 spd_ctrl_tick() 之后调用, 其中 spd_enc_update() 给出本拍的 Δcnt */
void ins_odom_update (void)
{
    float dl_m, dr_m, ds;
    float yaw_deg;
    int16 dl, dr;
    uint8 flags;

    /* --- 航向: 用累积角, -180~180 的原始角跨边界会跳 --- */
    yaw_deg = ins_imu_get_yaw();

    /* --- 里程: 左右各自换算成米, 取车体中心; 左右 cpr 可不同 --- */
    dl = enc_l.delta;
    dr = enc_r.delta;

    dl_m = ((float)dl / ((enc_l.cpr > 1.0f) ? enc_l.cpr : 1.0f)) * ins_m_per_rev;
    dr_m = ((float)dr / ((enc_r.cpr > 1.0f) ? enc_r.cpr : 1.0f)) * ins_m_per_rev;

    ds = (dl_m + dr_m) * 0.5f;

    flags = 0;

    /* --- 静止判据: 只置状态位, 不冻结积分 --- */
    if ((0 == dl) && (0 == dr))
    {
        if (s_still_cnt < 0xFFFFu) { s_still_cnt ++; }
    }
    else
    {
        s_still_cnt = 0;
    }
    if (s_still_cnt >= ODOM_STILL_TICKS) { flags |= ODOM_ST_STILL; }

    /* --- 打滑判据: 左右计数差大且航向几乎没变, 只报警 --- */
    {
        int32 al = (dl < 0) ? -(int32)dl : (int32)dl;
        int32 ar = (dr < 0) ? -(int32)dr : (int32)dr;
        int32 mx = (al > ar) ? al : ar;

        if (mx > 5)                                     /* 计数过小不计, 避免噪声 */
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

    /* --- 积分: ds 带符号, 前进为正、后退为负 --- */
    if (ins_odom_enable > 0.5f)
    {
        float rad = yaw_deg * DEG2RAD_F;

        s_x += -sinf(rad) * ds;
        s_y +=  cosf(rad) * ds;

        s_dist_abs += fabsf(ds);                        /* 总路程, 与位置积分分开 */
    }

    s_yaw   = yaw_deg;
    s_v_mps = ds / 0.005f;                          /* 本拍车速, 带符号 */
    s_flags = (uint8)(flags | ODOM_ST_ACTIVE);
}

/* ---- 查询 ---- */
float ins_odom_get_x    (void) { return s_x; }
float ins_odom_get_y    (void) { return s_y; }
float ins_odom_get_yaw  (void) { return s_yaw; }
float ins_odom_get_dist (void) { return s_dist_abs; }
float ins_odom_get_v_mps(void) { return s_v_mps; }
uint8 ins_odom_flags    (void) { return s_flags; }

float ins_odom_get_km (void)
{
    /* 当前生效的米/计数, 左右取平均, 诊断用 */
    float cpr_avg = (enc_l.cpr + enc_r.cpr) * 0.5f;

    if (cpr_avg < 1.0f) { cpr_avg = 1.0f; }
    return ins_m_per_rev / cpr_avg;
}

void ins_odom_snapshot (ins_odom_snap_t *out)
{
    uint32 primask;

    if (NULL == out) { return; }

    /* x/y/yaw/dist 必须一次性复制, 否则会被 5 ms 中断插到半路 */
    primask = ins_crit_enter();
    out->x        = s_x;
    out->y        = s_y;
    out->yaw      = s_yaw;
    out->dist_abs = s_dist_abs;
    out->v_mps    = s_v_mps;
    out->flags    = s_flags;
    ins_crit_exit(primask);
}
