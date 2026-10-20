/*********************************************************************************************************************
 * 工程 11 · 航向测试实现（设计说明见 head_test.h）
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>

#include "ins_odom.h"
#include "remote_ctrl.h"
#include "spd_ctrl.h"
#include "steer_ctrl.h"
#include "route_pb.h"
#include "motor_link.h"          /* motor_link_online()：链路在线判据 */
#include "spd_safety.h"          /* spd_safety_estop()：急停必须中止测试 */
#include "proto.h"               /* ★ 驱动令牌：ht_mode 是谁写的，谁就是驾驶员 */
#include "head_test.h"

/*===================================================================================================================
 *  参数（ID 69~75）
 *=================================================================================================================*/
volatile float ht_mode  = 0.0f;
volatile float ht_rps   = 1.20f;
volatile float ht_yaw   = 0.0f;
volatile float ht_dwell = 3.00f;
volatile float ht_step  = 5.00f;
volatile float ht_n     = 8.00f;
volatile float ht_k     = 2.00f;

/*===================================================================================================================
 *  状态
 *=================================================================================================================*/
static uint8   s_on    = 0u;
static uint8   s_head  = 0u;            /* 航向外环是否由本模块打开 */
static uint8   s_mode  = 0u;
static uint16  s_seg   = 0u;            /* 1..n，0 = 没在跑 */
static float   s_seg_view = HT_SEG_OFF; /* 遥测用：跑完保留 254 */
static uint32  s_ticks = 0u;            /* 段内 5 ms 计数 */
static float   s_yaw0  = 0.0f;          /* 起步航向（所有目标都相对它）*/
static float   s_tar   = 0.0f;          /* 本拍目标航向（绝对，deg）*/
static float   s_err   = 0.0f;
static uint8   s_rise  = 0u;            /* 本段是否已经进过 ±1° */
static uint32  s_rise_ms = 0u;
static float   s_dist0 = 0.0f;          /* 段起点里程 */
static uint32  s_print = 0u;            /* 模式①的 1 s 打印节拍 */

static float ht_wrap180 (float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

/*===================================================================================================================
 *  起 / 停
 *=================================================================================================================*/
static void ht_stop (const char *why)
{
    if (0u != s_head)
    {
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
    }
    /* ★ 先对齐转角，再清速度：反过来做，退出那一刻方向盘会甩一下 */
    steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());
    spd_target_l = 0.0f;
    spd_target_r = 0.0f;

    printf("[HT] 停止（%s）seg=%u\r\n", why, (unsigned)s_seg);

    s_on   = 0u;
    s_seg  = 0u;
    s_head = 0u;
    s_ticks = 0u;
    if (s_mode == HT_MODE_STEP) { s_seg_view = HT_SEG_DONE; }
    else                        { s_seg_view = HT_SEG_OFF;  }
}

static float ht_seg_target (uint16 seg)
{
    float tgt = 0.0f;

    if (seg >= 2u)
    {
        float mag = (float)(seg - 1u) * ht_step;     /* seg2: 1×step, seg3: 2×step … */

        tgt = ((0u == (seg & 1u)) ? mag : -mag);     /* 偶数段为正、奇数段为负 */
    }
    return tgt;
}

static void ht_begin (uint8 mode)
{
    ins_odom_snap_t o;

    ins_odom_snapshot(&o);

    s_mode  = mode;
    s_on    = 1u;
    s_seg   = 1u;
    s_ticks = 0u;
    s_rise  = 0u;
    s_rise_ms = 0u;
    s_print = 0u;
    s_yaw0  = o.yaw;
    s_dist0 = o.dist_abs;
    s_err   = 0.0f;

    s_seg_view = 1.0f;

    /* 航向环：增益走 ht_k；打开的上升沿由 steer_ctrl 自己把 target_yaw 初始化成当前航向 */
    steer_ctrl_set_head_kp(ht_k);
    if (0u == s_head) { steer_ctrl_set_head_enable(1u); s_head = 1u; }

    printf("[HT] 开始 mode=%u rps=%u.%02u k=%u.%02u n=%u step=%u.%01u dwell=%u.%02u\r\n",
           (unsigned)mode,
           (unsigned)(ht_rps), (unsigned)((ht_rps - (float)(unsigned)ht_rps) * 100.0f),
           (unsigned)(ht_k),   (unsigned)((ht_k - (float)(unsigned)ht_k) * 100.0f),
           (unsigned)ht_n,
           (unsigned)ht_step,  (unsigned)((ht_step - (float)(unsigned)ht_step) * 10.0f),
           (unsigned)ht_dwell, (unsigned)((ht_dwell - (float)(unsigned)ht_dwell) * 100.0f));
}

/*===================================================================================================================
 *  5 ms 节拍
 *=================================================================================================================*/
void head_test_tick (void)
{
    uint8 mode = (uint8)(ht_mode + 0.5f);
    ins_odom_snap_t o;

    if (mode > (uint8)HT_MODE_STEP) { mode = (uint8)HT_MODE_OFF; }

    /* ---- 0) 关 ---- */
    if ((uint8)HT_MODE_OFF == mode)
    {
        if (0u != s_on) { ht_stop("ht_mode=0"); }
        return;
    }

    /* ---- 1) 起（等链路在线再起，避免"一按就报安全门"）---- */
    if (0u == s_on)
    {
        if (pb_en >= 0.5f)        { return; }   /* 路表回放优先，互斥 */
        if (!motor_link_online()) { return; }
        if (ins_odom_enable < 0.5f) { return; }

        /* ★ 驱动权必须在上位机手里：ht_mode 是 P_CMD，写它的那个来源就是令牌持有者。
           没有令牌就不起 —— 否则会出现"遥控在开车、航向测试同时往 spd_target_* 写 rps"。 */
        if (proto_drv_owner() != PROTO_SRC_SPI) { return; }

        ht_begin(mode);
    }

    /* ---- 2) 安全门：链路 / 遥控接管 / 急停 / 驱动权 ---- */
    if ((!motor_link_online()) ||
        (RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_FAILSAFE == RC_Engage) ||
        (RC_ENGAGE_ESTOP == RC_Engage) ||
        (proto_drv_owner() != PROTO_SRC_SPI) ||
        (0u != spd_safety_estop()))
    {
        ht_stop("安全门");
        ht_mode = 0.0f;
        return;
    }

    ins_odom_snapshot(&o);
    s_ticks ++;

    /* ---- 3) 目标航向 ---- */
    if ((uint8)HT_MODE_FIXED == s_mode)
    {
        s_tar = s_yaw0 + ht_yaw;
        s_seg_view = 1.0f;
    }
    else
    {
        s_tar = s_yaw0 + ht_seg_target(s_seg);
        s_seg_view = (float)s_seg;
    }
    s_err = ht_wrap180(s_tar - o.yaw);

    /* ---- 4) 下发：航向外环 + 恒定后轮 rps ---- */
    steer_ctrl_set_head_kp(ht_k);
    steer_ctrl_set_target_yaw(s_tar);

    spd_target_l = ht_rps;
    spd_target_r = ht_rps;

    /* ---- 5) 上升时间（进 ±1° 的那一刻，记一次）---- */
    if ((0u == s_rise) && (fabsf(s_err) < 1.0f))
    {
        s_rise    = 1u;
        s_rise_ms = s_ticks * 5u;
    }

    /* ---- 6) 段推进 / 打印 ---- */
    if ((uint8)HT_MODE_FIXED == s_mode)
    {
        /* 固定航向：不结束，每秒打一行，方便串口看趋势 */
        s_print ++;
        if (s_print >= 200u)                    /* 200 × 5 ms = 1 s */
        {
            s_print = 0u;
            printf("[HT] tgt=%d.%01d yaw=%d.%01d err=%d.%01d u=%d\r\n",
                   (int)(s_tar),      (int)(fabsf(s_tar) * 10.0f) % 10,
                   (int)(o.yaw),      (int)(fabsf(o.yaw) * 10.0f) % 10,
                   (int)(s_err),      (int)(fabsf(s_err) * 10.0f) % 10,
                   (int)steer_ctrl_get_output());
        }
    }
    else
    {
        uint32 dwell_ticks = (uint32)((ht_dwell > 0.1f ? ht_dwell : 0.1f) * 200.0f);  /* 5 ms/拍 */

        if (s_ticks >= dwell_ticks)
        {
            float dist = o.dist_abs - s_dist0;

            printf("[HT] seg=%u/%u tgt=%d.%01d rise=%ums dist=%ucm err=%d.%01d\r\n",
                   (unsigned)s_seg, (unsigned)ht_n,
                   (int)(s_tar), (int)(fabsf(s_tar) * 10.0f) % 10,
                   (unsigned)s_rise_ms, (unsigned)(dist * 100.0f),
                   (int)(s_err), (int)(fabsf(s_err) * 10.0f) % 10);

            s_dist0 = o.dist_abs;
            s_ticks = 0u;
            s_rise  = 0u;
            s_rise_ms = 0u;

            if ((float)s_seg >= (ht_n > 0.5f ? ht_n : 1.0f))
            {
                ht_stop("序列跑完");
                ht_mode = 0.0f;
            }
            else
            {
                s_seg ++;
            }
        }
    }
}

/*===================================================================================================================
 *  读出（遥测用）
 *=================================================================================================================*/
void  head_test_init   (void) { s_on = 0u; s_seg = 0u; s_seg_view = HT_SEG_OFF; s_head = 0u; }
uint8 head_test_active (void) { return s_on; }
float head_test_tar_yaw(void) { return s_tar; }
float head_test_seg    (void) { return s_seg_view; }
float head_test_err    (void) { return s_err; }
