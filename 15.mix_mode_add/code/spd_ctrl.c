/* 双电机速度环 - 速度环控制器实现; 控制律与参数含义见 spd_ctrl.h */

#include "zf_common_headfile.h"
#include "spd_ctrl.h"
#include "spd_enc.h"
#include "spd_safety.h"
#include "motor_drv.h"

#include <math.h>

spd_gains_t     spd_g =
{
    SPD_K_V_DEFAULT,
    SPD_K_Z_DEFAULT,
    SPD_INV_K_DEFAULT,
    SPD_U0_DEFAULT
};

volatile float  spd_target_l     = 0.0f;
volatile float  spd_target_r     = 0.0f;
volatile float  spd_target_max   = SPD_TARGET_MAX_DEFAULT;
volatile float  spd_slew_rps_s   = SPD_SLEW_DEFAULT;

/* 开环 duty 模式(说明见 spd_ctrl.h); s_ol_last 记录上一拍开关值, 用于检测边沿并清积分 */
volatile float  spd_ol_en        = 0.0f;
volatile float  spd_ol_duty_l    = 0.0f;
volatile float  spd_ol_duty_r    = 0.0f;
static   uint8  s_ol_last        = 0;

spd_ctrl_st_t   ctl_l = {0};
spd_ctrl_st_t   ctl_r = {0};

void spd_ctrl_init (void)
{
    spd_ctrl_st_t cleared = {0};

    ctl_l = cleared;
    ctl_r = cleared;
    s_ol_last = 0;
    motor_drv_stop_all();
}

void spd_ctrl_reset_int (void)
{
    ctl_l.z = 0.0f;
    ctl_r.z = 0.0f;
}

/* 目标限幅: 只夹本拍控制目标, 不改命令源的全局值; 上限取 spd_target_max */
static float limit_target (float r)
{
    float m = spd_target_max;

    if (m < 0.0f) { m = 0.0f; }
    if (r >  m)   { r =  m; }
    if (r < -m)   { r = -m; }

    return r;
}

/* 单路闭环: 前馈 + PI, 条件积分抗饱和; r 目标 rps, en 门控, ulim 输出上限(归一化) */
static void spd_step_one (uint8 ch, spd_enc_t *e, spd_ctrl_st_t *st, float r, uint8 en, float ulim)
{
    float v;
    float err;
    float u_ff;
    float z_try;
    float u_raw;
    float u_clamped;

    if (0 == en)
    {
        /* 门控不过 -> 输出严格 0, 积分与斜坡状态一并清掉 */
        st->e      = 0.0f;
        st->z      = 0.0f;
        st->r_app  = 0.0f;
        st->r_cmd_prev = 0.0f;
        st->u_ff   = 0.0f;
        st->u_cmd  = 0.0f;
        st->u      = 0.0f;
        st->u_max  = ulim;
        st->sat    = 0;
        st->enabled = 0;
        motor_drv_set(ch, 0.0f);
        return;
    }

    r = limit_target(r);

    /* 目标斜坡 + 大跳变清积分(参数含义见 spd_ctrl.h), 遥测 target 为下发值而非 r_app */
    {
        float r_in = r;
        float lim  = spd_slew_rps_s * SPD_T_S;
        float d    = r_in - st->r_cmd_prev;

        if ((d > SPD_INT_CLR_STEP) || (d < -SPD_INT_CLR_STEP))
        {
            st->z = 0.0f;
        }
        st->r_cmd_prev = r_in;

        if (lim > 0.0f)
        {
            if      (r > st->r_app + lim) { r = st->r_app + lim; }
            else if (r < st->r_app - lim) { r = st->r_app - lim; }
        }
        st->r_app = r;
    }

    v = e->rps;

    /* 零目标静止锁: |r| < SPD_ZERO_RPS 且 |v| < SPD_ZERO_V_RPS 时输出 0, 防编码器假脉冲自转 */
    if ((r > -SPD_ZERO_RPS) && (r < SPD_ZERO_RPS) && (fabsf(v) < SPD_ZERO_V_RPS))
    {
        st->e       = 0.0f;
        st->z       = 0.0f;
        st->u_ff    = 0.0f;
        st->u_cmd   = 0.0f;
        st->u       = 0.0f;
        st->u_max   = ulim;
        st->sat     = 0;
        st->enabled = 0;
        motor_drv_set(ch, 0.0f);
        return;
    }

    err = r - v;

    /* 前馈: 线性项 + 摩擦截距(带符号); 目标接近 0 时不加截距, 免输出在 +-u0 之间抖 */
    u_ff = r * spd_g.inv_K;
    if      (r >  SPD_FF_DEADBAND) { u_ff += spd_g.u0; }
    else if (r < -SPD_FF_DEADBAND) { u_ff -= spd_g.u0; }

    /* 积分试探: 先累积并限幅在 +-SPD_Z_MAX */
    z_try = st->z + err * SPD_T_S;
    if (z_try >  SPD_Z_MAX) { z_try =  SPD_Z_MAX; }
    if (z_try < -SPD_Z_MAX) { z_try = -SPD_Z_MAX; }

    u_raw   = u_ff + spd_g.k_v * err + spd_g.k_z * z_try;
    st->sat = 0;

    /* 条件积分抗饱和: 输出饱和且误差仍朝饱和方向 -> 冻结积分, 用旧 z 重算 */
    if (((u_raw > ulim) && (err > 0.0f)) || ((u_raw < -ulim) && (err < 0.0f)))
    {
        u_raw   = u_ff + spd_g.k_v * err + spd_g.k_z * st->z;
        st->sat = 1;
    }
    else
    {
        st->z = z_try;
    }

    st->u_cmd = u_raw;

    u_clamped = u_raw;
    if (u_clamped >  ulim) { u_clamped =  ulim; }
    if (u_clamped < -ulim) { u_clamped = -ulim; }

    if (u_clamped != u_raw)
    {
        st->sat = 1;
        spd_safety_note_clip(ch);
    }

    st->e       = err;
    st->u_ff    = u_ff;
    st->u       = u_clamped;
    st->u_max   = ulim;
    st->enabled = 1;
    st->steps++;

    motor_drv_set(ch, u_clamped);
}

/* 单路开环输出: 直接用上位机给的 duty(%) 当输出, 绕过速度环; 门控不过则输出 0, 仍受 ulim 与 +-1.0 限幅 */
static void spd_step_openloop (uint8 ch, spd_ctrl_st_t *st, float duty_pct, uint8 en, float ulim)
{
    float u;
    float u_requested;

    if (0 == en)
    {
        st->e       = 0.0f;
        st->z       = 0.0f;
        st->r_app   = 0.0f;
        st->r_cmd_prev = 0.0f;
        st->u_ff    = 0.0f;
        st->u_cmd   = 0.0f;
        st->u       = 0.0f;
        st->u_max   = ulim;
        st->sat     = 0;
        st->enabled = 0;
        motor_drv_set(ch, 0.0f);
        return;
    }

    u_requested = duty_pct * 0.01f;             /* % -> 归一化 */
    u = u_requested;

    if (u >  ulim) { u =  ulim; }
    if (u < -ulim) { u = -ulim; }
    if (u >  1.0f) { u =  1.0f; }
    if (u < -1.0f) { u = -1.0f; }

    st->e       = 0.0f;
    st->z       = 0.0f;
    st->r_app   = 0.0f;
    st->r_cmd_prev = 0.0f;
    st->u_ff    = 0.0f;
    st->u_cmd   = u_requested;
    st->u       = u;
    st->u_max   = ulim;
    st->sat     = (u != u_requested) ? 1u : 0u;
    st->enabled = 1;
    st->steps++;

    if (st->sat)
    {
        spd_safety_note_clip(ch);
    }
    motor_drv_set(ch, u);
}

/* 5 ms 主节拍: 保护判定 -> 测速 -> 控制 -> 写输出; 保护判定在前, 保证用本拍结论 */
volatile uint8 spd_coast_req = 0u;      /* 自由轮请求, 1 = 两后轮输出钉 0 */

void  spd_ctrl_set_coast (uint8 on) { spd_coast_req = on ? 1u : 0u; }
uint8 spd_ctrl_coast     (void)     { return spd_coast_req; }

void spd_ctrl_tick (void)
{
    uint8 en;
    uint8 ol;
    float ulim;

    spd_safety_tick();

    ulim = spd_safety_limit_u();
    en   = spd_safety_allow(0);                 /* 两路共用同一套门控 */

    spd_enc_update(&enc_l);
    spd_enc_update(&enc_r);

    /* 自由轮: 输出钉 0, 测速与里程照跑; 须在 spd_enc_update() 之后执行 */
    if (0u != spd_coast_req)
    {
        ctl_l.z = 0.0f;             ctl_r.z = 0.0f;
        ctl_l.r_app = 0.0f;         ctl_r.r_app = 0.0f;
        ctl_l.r_cmd_prev = 0.0f;    ctl_r.r_cmd_prev = 0.0f;
        ctl_l.e = 0.0f;             ctl_r.e = 0.0f;
        ctl_l.u_ff = 0.0f;          ctl_r.u_ff = 0.0f;
        ctl_l.u_cmd = 0.0f;         ctl_r.u_cmd = 0.0f;
        ctl_l.u = 0.0f;             ctl_r.u = 0.0f;
        ctl_l.sat = 0u;             ctl_r.sat = 0u;
        ctl_l.enabled = 0u;         ctl_r.enabled = 0u;
        ctl_l.u_max = ulim;         ctl_r.u_max = ulim;
        motor_drv_set(0, 0.0f);
        motor_drv_set(1, 0.0f);
        return;
    }

    ol = (spd_ol_en > 0.5f) ? 1u : 0u;

    /* 模式切换时闭环斜坡从 0 重启, 丢弃旧目标; 不是运动中无扰切换 */
    if (ol != s_ol_last)
    {
        s_ol_last = ol;
        ctl_l.z = 0.0f;
        ctl_r.z = 0.0f;
        ctl_l.r_app = 0.0f;
        ctl_r.r_app = 0.0f;
        ctl_l.r_cmd_prev = 0.0f;
        ctl_r.r_cmd_prev = 0.0f;
    }

    if (0 != ol)
    {
        spd_step_openloop(0, &ctl_l, spd_ol_duty_l, en, ulim);
        spd_step_openloop(1, &ctl_r, spd_ol_duty_r, en, ulim);
    }
    else
    {
        spd_step_one(0, &enc_l, &ctl_l, spd_target_l, en, ulim);
        spd_step_one(1, &enc_r, &ctl_r, spd_target_r, en, ulim);
    }
}

/* 电子差速(说明见 spd_ctrl.h); 纯函数式分配, 不改任何状态机 */
volatile float spd_diff_en  = 1.0f;
volatile float spd_diff_w   = SPD_DIFF_W_DEFAULT;
volatile float spd_diff_max = SPD_DIFF_MAX_DEFAULT;

static float s_diff_dm = 0.0f;          /* 实测舵角一阶低通状态, 度 */

void spd_diff_alloc (float v_mps, float L_m, float delta_meas_deg, float *vl, float *vr)
{
    float dm, k, r;

    if ((NULL == vl) || (NULL == vr)) { return; }

    *vl = v_mps;
    *vr = v_mps;

    if (spd_diff_en < 0.5f)
    {
        s_diff_dm = delta_meas_deg;
        return;
    }

    /* 静止与倒车低速直通: 前进 0.02 m/s 以上才分, 倒车须超过 SPD_DIFF_REV_V; 低通跟随当前角防首拍跳变 */
    if (((v_mps >= 0.0f) && (v_mps < 0.02f)) ||
        ((v_mps <  0.0f) && (v_mps > -SPD_DIFF_REV_V)))
    {
        s_diff_dm = delta_meas_deg;
        return;
    }

    if (L_m < 0.30f) { return; }                    /* 轴距最小值, m; 未配对则不差速 */

    /* 一阶低通, alpha = 0.25; 抑制舵角量化噪声经 tan() 放大 */
    dm = 0.25f * delta_meas_deg + 0.75f * s_diff_dm;
    s_diff_dm = dm;

    k = tanf(dm * 0.0174532925f) / L_m;             /* 曲率, 1/m; 正 = 右转 */
    r = k * spd_diff_w * 0.5f;                      /* 单侧相对偏差 */

    if (r >  spd_diff_max) { r =  spd_diff_max; }
    if (r < -spd_diff_max) { r = -spd_diff_max; }

    /* 右转(k > 0)时左轮在外侧加快, 右轮在内侧减慢 */
    *vl = v_mps * (1.0f + r);
    *vr = v_mps * (1.0f - r);
}
