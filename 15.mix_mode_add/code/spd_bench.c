/* 速度环 12 段跑班序列器实现; 排班与安全门见 spd_bench.h */

#include "zf_common_headfile.h"
#include "spd_bench.h"
#include "bench_log.h"
#include "spd_ctrl.h"
#include "spd_enc.h"
#include "spd_safety.h"
#include "remote_ctrl.h"

/* ---- 一、排班表: 每行 { 段号 1~12, 占空比 %, 持续 ms }, 改这里即改跑班内容 ---- */
typedef struct
{
    uint8  seg;
    float  duty;
    uint16 ms;
} sb_step_t;

static const sb_step_t SB_STEPS[] =
{
    /* 段1~4: 锁死带至突破上沿, 每档 2~3 s */
    { 1u,  6.0f, 2000u }, { 1u,  8.0f, 2000u }, { 1u, 10.0f, 2000u },
    { 2u, 11.0f, 3000u }, { 2u, 12.0f, 3000u }, { 2u, 13.0f, 3000u },
    { 3u, 13.5f, 3000u }, { 3u, 14.0f, 3000u }, { 3u, 14.5f, 3000u },
    { 4u, 15.0f, 3000u }, { 4u, 15.5f, 3000u }, { 4u, 16.0f, 3000u },
    /* 段5~9: 直线段 18 % ~ 35 % */
    { 5u, 18.0f, 3000u },
    { 6u, 21.0f, 3000u },
    { 7u, 25.0f, 3000u },
    { 8u, 30.0f, 3000u },
    { 9u, 35.0f, 3000u },
    /* 段10 重复性; 段11/12 阶跃, 前置电平保持 1500 ms */
    {10u, 25.0f, 3000u }, {10u, 25.0f, 3000u },
    {11u, 20.0f, 1500u }, {11u, 30.0f, 3000u },
    {12u, 30.0f, 1500u }, {12u, 20.0f, 3000u },
};

#define SB_NSTEPS   ((uint8)(sizeof(SB_STEPS) / sizeof(SB_STEPS[0])))

#define SB_TICK_MS      (5u)
#define SB_GAP_MS       (2000u)     /* 小步之间停机时长 ms, duty = 0 */

static uint8  s_state  = SB_IDLE;
static uint8  s_refuse = 0u;
static uint8  s_abort  = 0u;
static uint8  s_step   = 0u;        /* 当前小步下标 */
static uint8  s_instep = 0u;        /* 1 = 在小步里, 0 = 段间停机 */
static uint16 s_t      = 0u;        /* 当前相位已过 ms */
static float  s_duty   = 0.0f;
static uint8  s_seg    = 0u;

void spd_bench_init(void)
{
    s_state  = SB_IDLE;
    s_refuse = 0u;
    s_abort  = 0u;
    s_step   = 0u;
    s_instep = 0u;
    s_t      = 0u;
    s_duty   = 0.0f;
    s_seg    = 0u;

    spd_ol_en     = 0.0f;
    spd_ol_duty_l = 0.0f;
    spd_ol_duty_r = 0.0f;
}

uint8 spd_bench_start(void)
{
    s_refuse = 0u;

    if ((SB_RUN == s_state) || (SB_WAITLOG == s_state)) { s_refuse = SB_REF_BUSY;        return s_refuse; }
    if (0u == RC_Online)                                { s_refuse = SB_REF_NO_RC;       return s_refuse; }
    if (0u != RC_Estop)                                 { s_refuse = SB_REF_ESTOP;       return s_refuse; }
    if (RC_ENGAGE_STANDBY != RC_Engage)                 { s_refuse = SB_REF_NOT_STANDBY; return s_refuse; }
    if (0u == spd_safety_allow(0))                      { s_refuse = SB_REF_NOT_STANDBY; return s_refuse; }

    /* 目标归零、积分清零后再进开环 */
    spd_target_l = 0.0f;
    spd_target_r = 0.0f;
    spd_ctrl_reset_int();

    spd_ol_duty_l = 0.0f;
    spd_ol_duty_r = 0.0f;
    spd_ol_en     = 1.0f;                   /* 进开环: 直接给占空比, 绕过速度环 */

    bench_log_begin();

    s_step   = 0u;
    s_instep = 1u;
    s_t      = 0u;
    s_seg    = SB_STEPS[0].seg;
    s_duty   = SB_STEPS[0].duty;
    s_abort  = 0u;
    s_state  = SB_RUN;

    printf("[##] BENCH    start: %u steps, %u%% first\r\n", (unsigned)SB_NSTEPS, (unsigned)s_duty);

    return 0u;
}

void spd_bench_abort(uint8 code)
{
    if ((SB_RUN != s_state) && (SB_WAITLOG != s_state)) { return; }

    if (0u == s_abort) { s_abort = code; }

    spd_ol_duty_l = 0.0f;
    spd_ol_duty_r = 0.0f;
    spd_ol_en     = 0.0f;                   /* 回闭环, 目标为 0 */

    bench_log_end(0u);
    s_state = SB_WAITLOG;

    printf("[##] BENCH    abort code=%u\r\n", (unsigned)code);
}

void spd_bench_tick(void)
{
    float rps_l;
    float rps_r;

    if (SB_RUN != s_state) { return; }

    /* 安全复核: 5 ms 一次, 任一不满足即撤开环并中止 */
    if (0u != RC_Estop)                    { spd_bench_abort(SB_AB_ESTOP);       return; }
    if (0u == RC_Online)                   { spd_bench_abort(SB_AB_NO_RC);       return; }
    if (RC_ENGAGE_STANDBY != RC_Engage)    { spd_bench_abort(SB_AB_NOT_STANDBY); return; }
    if (0u == spd_safety_allow(0))         { spd_bench_abort(SB_AB_SAFETY);      return; }

    s_t += SB_TICK_MS;
    bench_log_tick_clock();

    rps_l = enc_l.rps;
    rps_r = enc_r.rps;

    if (0u != s_instep)
    {
        spd_ol_duty_l = s_duty;
        spd_ol_duty_r = s_duty;

        /* 记录实际施加的占空比(ctl_?.u 为限幅后的值), 便于在曲线上看出限幅 */
        bench_log_tick(s_t, s_seg, (ctl_l.u + ctl_r.u) * 50.0f, rps_l, rps_r);

        if (s_t >= SB_STEPS[s_step].ms)
        {
            /* 本小步结束: 回 0, 进入段间停机 */
            spd_ol_duty_l = 0.0f;
            spd_ol_duty_r = 0.0f;
            s_duty   = 0.0f;
            s_instep = 0u;
            s_seg    = 0u;
            s_t      = 0u;
        }
    }
    else
    {
        spd_ol_duty_l = 0.0f;
        spd_ol_duty_r = 0.0f;

        /* 停机期间 seg = 0, 按 100 ms 稀疏记录 */
        bench_log_tick(BL_BURST_MS, 0u, (ctl_l.u + ctl_r.u) * 50.0f, rps_l, rps_r);

        if (s_t >= SB_GAP_MS)
        {
            if ((uint8)(s_step + 1u) < SB_NSTEPS)
            {
                s_step++;
                s_instep = 1u;
                s_seg    = SB_STEPS[s_step].seg;
                s_duty   = SB_STEPS[s_step].duty;
                s_t      = 0u;
            }
            else
            {
                /* 全部跑完 */
                spd_ol_duty_l = 0.0f;
                spd_ol_duty_r = 0.0f;
                spd_ol_en     = 0.0f;

                bench_log_end(1u);
                s_state = SB_WAITLOG;

                printf("[##] BENCH    done: %u rec\r\n", (unsigned)bench_log_count());
                return;
            }
        }
    }
}

void spd_bench_poll(void)
{
    if (SB_WAITLOG != s_state) { return; }

    switch (bench_log_state())
    {
        case BL_DONE: s_state = (0u != s_abort) ? SB_ABORT : SB_DONE; break;
        case BL_ERR:  s_state = SB_ABORT;                             break;
        default:      break;
    }
}

uint8 spd_bench_state     (void) { return s_state;  }
uint8 spd_bench_refuse    (void) { return s_refuse; }
uint8 spd_bench_abort_code(void) { return s_abort;  }
uint8 spd_bench_seg       (void) { return s_seg;    }
uint8 spd_bench_step      (void) { return s_step;   }
uint8 spd_bench_nsteps    (void) { return SB_NSTEPS; }
