/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 速度环控制器实现
 * 控制律与参数出处见 spd_ctrl.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spd_ctrl.h"
#include "spd_enc.h"
#include "spd_safety.h"
#include "motor_drv.h"

#include <math.h>

/*===================================================================================================================
 *  全局参数与状态
 *=================================================================================================================*/
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

/* ★ 开环 duty 模式（说明见 spd_ctrl.h）:
   外面还留了一份"上一次的开关值"，用来检测 0->1 / 1->0 的边沿去清积分。 */
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

/*-------------------------------------------------------------------------------------------------------------------
 *  目标限幅: 上位机的滑条量程(+-8 rps)比这台电机的能力(满占空比约 1.7 rps)大得多,
 *  所以必须有第二道"目标上限"。仅限制本拍控制目标, 不改写命令源的全局值。
 *-----------------------------------------------------------------------------------------------------------------*/
static float limit_target (float r)
{
    float m = spd_target_max;

    if (m < 0.0f) { m = 0.0f; }
    if (r >  m)   { r =  m; }
    if (r < -m)   { r = -m; }

    return r;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  一路控制
 *-----------------------------------------------------------------------------------------------------------------*/
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
        /* 门控不过 -> 输出严格 0, 同时把积分清掉,
           否则"被保护停住"期间误差一直累积, 一放开门就是一次猛冲。 */
        st->e      = 0.0f;
        st->z      = 0.0f;
        st->r_app  = 0.0f;      /* 斜坡状态也归零 -> 下次使能是从 0 平滑升上去的 */
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

    /* ---- 目标斜坡 + 大跳变清积分 (见 spd_ctrl.h 的 SPD_SLEW_DEFAULT 注解) ----
       注意: 遥测里 target 通道给的是"上位机下发的目标", 不是这里的 r_app,
       所以波形上看不到斜坡本身, 但能从 u_l / v_l 的形状看出来(不再是硬折线)。 */
    {
        float r_in = r;
        float lim  = spd_slew_rps_s * SPD_T_S;      /* 每拍最多变多少 rps */
        /* Compare consecutive limited commands, not the remaining ramp distance. */
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

    err = r - v;

    /* 前馈: 线性项 + 摩擦截距(带符号)。目标接近 0 时不加截距,
       否则目标 0 时输出会一直在 +-u0 之间抖。 */
    u_ff = r * spd_g.inv_K;
    if      (r >  SPD_FF_DEADBAND) { u_ff += spd_g.u0; }
    else if (r < -SPD_FF_DEADBAND) { u_ff -= spd_g.u0; }

    /* 积分试探 */
    z_try = st->z + err * SPD_T_S;
    if (z_try >  SPD_Z_MAX) { z_try =  SPD_Z_MAX; }
    if (z_try < -SPD_Z_MAX) { z_try = -SPD_Z_MAX; }

    u_raw   = u_ff + spd_g.k_v * err + spd_g.k_z * z_try;
    st->sat = 0;

    /* 条件积分抗饱和: 饱和且误差还在往饱和方向推 -> 用旧 z 重算(冻结积分) */
    if (((u_raw > ulim) && (err > 0.0f)) || ((u_raw < -ulim) && (err < 0.0f)))
    {
        u_raw   = u_ff + spd_g.k_v * err + spd_g.k_z * st->z;
        st->sat = 1;
    }
    else
    {
        st->z = z_try;
    }

    st->u_cmd = u_raw;                          /* 限幅前的请求值(遥测用) */

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

/*-------------------------------------------------------------------------------------------------------------------
 *  一路**开环**输出: 直接用上位机给的 duty(%) 当输出, 绕过整个速度环
 *
 *  与闭环的两处相同、一处不同（说明见 spd_ctrl.h）：
 *    相同：i) 门控不过(en=0) -> 输出严格 0; ii) 输出仍受占空比限幅 ulim 约束
 *    不同：不算前馈/积分/误差, 遥测里的 e / z / u_ff 一律记 0（免得被误读成闭环数据）
 *-----------------------------------------------------------------------------------------------------------------*/
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

    /* 限幅: 先夹"本拍允许的上限"(= duty_limit), 再夹物理 1.0。
       ⚠ 开环也**不许绕限幅** —— 想给更大的占空比就先把「限幅|占空比上限」拖上去。 */
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

/*-------------------------------------------------------------------------------------------------------------------
 *  5ms 主节拍
 *
 *  顺序不能变:
 *    保护判定 -> 测速 -> 控制 -> 写输出
 *  保护判定必须在控制之前, 这样控制器用的一定是本拍的结论(而不是上一拍的)。
 *
 *  ★ 2026-10-13: 加了开环分支。测速**照跑**（开环阶跃就是要看 v 怎么响应）。
 *-----------------------------------------------------------------------------------------------------------------*/
/* 自由轮标志（见 spd_ctrl.h 的 spd_ctrl_set_coast） */
volatile uint8 spd_coast_req = 0u;

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

    /* ★★ 自由轮（人推车）：输出钉 0，**测速/里程照跑**。
       ⚠ 顺序：必须放在 spd_enc_update() 之后 —— 里程要靠这两个 delta。
       ⚠ 绝对不能"把目标写 0"来代替本分支：闭环看到 v>0 会反向下力刹车，人会推不动。 */
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

    /* Restart the closed-loop ramp from zero on either mode transition.
       This discards stale references; it is not a bumpless moving handoff. */
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

/*===================================================================================================================
 *  ★★ 电子差速（说明见 spd_ctrl.h）
 *     纯函数式分配：不碰任何状态机，调用方（回放/遥控/跑班）各自决定什么时候用。
 *=================================================================================================================*/
volatile float spd_diff_en  = 1.0f;
volatile float spd_diff_w   = SPD_DIFF_W_DEFAULT;
volatile float spd_diff_max = SPD_DIFF_MAX_DEFAULT;

static float s_diff_dm = 0.0f;          /* 实测舵角一阶低通状态（度）*/

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

    /* 倒车/静止：不做分配，但**让低通跟上**当前角，免得下次前进第一拍跳变 */
    if (v_mps < 0.02f)
    {
        s_diff_dm = delta_meas_deg;
        return;
    }

    if (L_m < 0.30f) { return; }                    /* 轴距没配好：宁可不差速 */

    /* 一阶低通（α=0.25）：实测舵角带编码器量化噪声，直接进 tan() 会让差速抖 */
    dm = 0.25f * delta_meas_deg + 0.75f * s_diff_dm;
    s_diff_dm = dm;

    k = tanf(dm * 0.0174532925f) / L_m;             /* 1/m，正 = 右转 */
    r = k * spd_diff_w * 0.5f;                      /* 相对偏差 */

    if (r >  spd_diff_max) { r =  spd_diff_max; }
    if (r < -spd_diff_max) { r = -spd_diff_max; }

    /* 右转（k>0）：左轮在外侧 ⇒ 快；右轮在内侧 ⇒ 慢。
       ⚠ 符号来自 `steer_delta > 0 ⇒ 右转`（见 steer_ctrl / 轮径反查文档）。 */
    *vl = v_mps * (1.0f + r);
    *vr = v_mps * (1.0f - r);
}
