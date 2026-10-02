/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 速度环控制器实现
 * 控制律与参数出处见 spd_ctrl.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "spd_ctrl.h"
#include "spd_enc.h"
#include "spd_safety.h"
#include "motor_drv.h"

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

spd_ctrl_st_t   ctl_l = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f};
spd_ctrl_st_t   ctl_r = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f};

void spd_ctrl_init (void)
{
    spd_ctrl_reset_int();
    motor_drv_stop_all();
}

void spd_ctrl_reset_int (void)
{
    ctl_l.z = 0.0f;
    ctl_r.z = 0.0f;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  目标限幅: 上位机的滑条量程(+-8 rps)比这台电机的能力(满占空比约 1.7 rps)大得多,
 *  所以必须有第二道"目标上限"。夹完写回全局, 上位机看到的就是真正生效的目标。
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
        float d    = r_in - st->r_app;

        if ((d > SPD_INT_CLR_STEP) || (d < -SPD_INT_CLR_STEP))
        {
            st->z = 0.0f;
        }

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
 *  5ms 主节拍
 *
 *  顺序不能变:
 *    保护判定 -> 测速 -> 控制 -> 写输出
 *  保护判定必须在控制之前, 这样控制器用的一定是本拍的结论(而不是上一拍的)。
 *-----------------------------------------------------------------------------------------------------------------*/
void spd_ctrl_tick (void)
{
    uint8 en;
    float ulim;

    spd_safety_tick();

    ulim = spd_safety_limit_u();
    en   = spd_safety_allow(0);                 /* 两路共用同一套门控 */

    spd_enc_update(&enc_l);
    spd_enc_update(&enc_r);

    spd_step_one(0, &enc_l, &ctl_l, spd_target_l, en, ulim);
    spd_step_one(1, &enc_r, &ctl_r, spd_target_r, en, ulim);
}
