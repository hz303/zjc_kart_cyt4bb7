/*********************************************************************************************************************
 * 路表回放 Pure Pursuit 外环（结构见 route_pb.h）
 * 分段：一、工具  二、速度剖面  三、最近点（单调）  四、起/停  五、主循环  六、读出接口
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>

#include "ins_odom.h"
#include "ins_imu.h"        /* ins_imu_get_yaw()：绝对航向 deg */
#include "ins_route.h"
#include "remote_ctrl.h"
#include "spd_ctrl.h"
#include "spd_safety.h"         /* spd_safety_estop()：急停必须中止回放 */
#include "steer_ctrl.h"
#include "route_pb.h"
#ifdef HOST_SIM
#include <stdlib.h>
#endif
#include "head_test.h"
#include "proto.h"              /* 驱动令牌：回放是启动它的来源的子模式 */
#include "motor_link.h"         /* motor_link_online()：手机/上位机链路是否在线（PWR 放行判据） */
#include "route_app.h"          /* rt_revplay：ID77 倒序回放开关 */

#define PB_D2R      (0.01745329252f)
#define PB_R2D      (57.29577951f)

/* ---- 参数（ID 43~55, 64~68）---- */
volatile float pb_en      = 0.0f;
volatile float pb_mode    = 1.0f;       /* 默认只给转向 */
volatile float pb_vmax    = 1.50f;      /* m/s  */
volatile float pb_alat    = 2.00f;      /* m/s^2 */
volatile float pb_abrk    = 0.80f;      /* m/s^2 */
volatile float pb_l       = PB_L_DEFAULT;
volatile float pb_ld_a    = 0.35f;
volatile float pb_ld_b    = 0.20f;
volatile float pb_ld_min  = 0.30f;
volatile float pb_ld_max  = 1.50f;
volatile float pb_ff_en   = 0.0f;
volatile float pb_ff_gain = 1.00f;

volatile float pb_vmin    = 0.20f;              /* 64 最低速度，防爬行 */
volatile float pb_kwin    = PB_KWIN_DEFAULT;    /* 65 曲率中心差分半窗（点）*/
volatile float pb_smode   = 0.0f;               /* 66 0 = 航向外环 / 1 = 直接给 δ */
volatile float pb_kh      = 0.0f;               /* 67 航向环增益；0 = 自动 2L/ld */
volatile float pb_fin     = 0.15f;              /* 68 终点判定半径 m */

/* ---- 状态 ---- */
static uint8   s_st    = PB_ST_IDLE;
static uint8   s_arm   = 0u;            /* pb_en 的上一拍，用来抓 0→1 上升沿 */
static uint8   s_done_armed = 0u;       /* 1 = 已走过完成阈值点，才允许判到终点 */
static uint16  s_idx   = 0u;            /* 最近点（单调，不后退）*/
static uint16  s_dbg_best = 0u;      /* 遥测 41：进度点索引 */
static uint16  s_dbg_fi   = 0u;      /* 遥测 42：前视点索引 */
static uint16  s_aim   = 0u;            /* 前视点 */
static uint16  s_n     = 0u;            /* 开跑时快照的点数 */
static float   s_len   = 0.0f;          /* 开跑时快照的表长 m，跑飞判据用 */
static float   s_d0    = 0.0f;          /* 开跑时的累计里程(m) */
static float   s_elat  = 0.0f;
static float   s_dtrue = 0.0f;   /* 到表折线的真实距离 m */
static uint16  s_offcnt = 0u;    /* 连续脱表拍数 */
static float   s_ld    = 0.0f;
static float   s_tv    = 0.0f;
static float   s_ytgt  = 0.0f;
static uint8   s_head  = 0u;            /* 1 = 航向外环正处于打开状态 */
static uint8   s_rev_tab     = 0u;   /* 1 = 表是倒车录的（REV 点过半）*/
static uint8   s_rev_ord     = 0u;   /* 1 = 本次把点序倒过来了 */
static uint8   s_drv_rev     = 0u;   /* 1 = 本次倒车开（ID85）*/
static uint8   s_rev_eff     = 0u;   /* 1 = 实际行进方向反 */
static float   s_tab_off     = 0.0f; /* 表 yaw 到行进切向要加的 0/180 deg */
static uint8   s_ctrl_eff    = PB_CTRL_PP;  /* 本趟控制律（ID87 自动或手动） */
/* 信念锚点常量补偿 deg：让信念航向与真实车头同向 */
static float   s_anchor_off  = 0.0f;
static uint8   s_nose_flip   = 0u;
/* 1 = 倒车，内环 P 用 1/3 增益 */
static uint8   s_gain_rev   = 0u;
static float   s_kp_fwd     = 0.0f;
/* 世界系航向零位 deg（起跑首拍测出，之后扣掉） */
static float   s_yaw_off    = 0.0f;
static uint8   s_yaw_ok     = 0u;   /* 1 = 航向零位已标定 */
/* 本趟遍历顺序副本，不写录制侧的路表数组 */
static ins_wp_t s_play[INS_ROUTE_MAX_POINTS];
static ins_route_head_t s_play_head;
static float s_arc[INS_ROUTE_MAX_POINTS];
static uint8 s_dir[INS_ROUTE_MAX_POINTS]; /* 每段出向的实际行驶方向 1 = 倒车 */
static uint8 s_mixed = 0u;
static uint8 s_mileage = 0u;
static uint16 s_shift_idx = 0u;
static uint16 s_leg_start = 0u;
static uint8 s_shifting = 0u, s_stopped = 0u;
static uint32 s_shift_t0 = 0u, s_stopped_t0 = 0u;
static uint8 s_depart = 0u;
static uint32 s_depart_t0 = 0u;
static float s_leg_d0 = 0.0f;
static uint8   s_drv   = PROTO_SRC_NONE;/* 启动本轮回放的驱动令牌持有者（菜单/上位机），
                                           抢占检测用它，不用当前 owner 续租 */
static float   s_vlim[INS_ROUTE_MAX_POINTS];    /* 每点限速 m/s（2048 × 4 B = 8 KB）*/
static float   s_kap[INS_ROUTE_MAX_POINTS];     /* 每点曲率 1/m（供速率约束）*/

/* 起点基准：起跑那一刻的 odom 快照，之后位姿一律按相对它的增量算
   px = p[0] + R(-s_o0yaw)·(o − o0)；表头 origin_x/y/yaw 只用于诊断打印 */
static float   s_o0x = 0.0f, s_o0y = 0.0f, s_o0yaw = 0.0f;
/* 1 = PWR 期间本模块已拿本地授权；不拿则 SAFE_BIT_NOT_EN 挡着，电机不出力 */
static uint8   s_local = 0u;
/* 1 = 当前进度点在倒车段（录制速度 < -PB_REV_EPS，带滞回） */
static uint8   s_rev = 0u;
/* 急停连续计数（去抖用，见 poll 里的急停门） */
static uint16  s_estop_ticks = 0u;
/* 收车回中连续计数（去抖用，见 pb_release_tick） */
static uint16  s_rel_ticks = 0u;
/* 1 = 本拍目标转角由路线层独占（9b / 倒车段），0 = 交给航向环（9a） */
static uint8   s_own_target = 0u;

/* 停车斜坡：0 = 未在斜坡中；1 = 把命中那一拍的左右目标按 PB_STOP_MS 线性降到 0，
   单位同 spd_target_*（rps） */
static uint8   s_ramp    = 0u;
static uint32  s_ramp_t0 = 0u;    /* 斜坡起始时刻 ms（motor_link_now_ms） */
static float   s_ramp_l  = 0.0f;
static float   s_ramp_r  = 0.0f;

/* 跟丢计时：满舵 + elat 不小 + 最近点索引冻结 ⇒ 车在绕着前视点画圆 */
static uint32  s_lost_t0  = 0u;   /* 计时起点 ms；0 = 没在计时 */
static uint16  s_lost_idx = 0u;   /* 计时起点的最近点索引 */
static float   s_tanref  = 0.0f;  /* 本拍参考切向 deg */
static float   s_err_w  = 0.0f;   /* 航向误差 deg（无 PP 通道）*/
static float   s_kref   = 0.0f;   /* LQR 曲率参考 1/m */
static float   s_alpha   = 0.0f;  /* 上一拍前视偏角 rad */
static float   s_lqr_kappa = 0.0f;   /* 上一拍下发的曲率指令 1/m（正 = 右转）*/
static float   s_lpf_y      = 0.0f;  /* 输出低通状态 */
static uint8   s_dir_chk    = 1u;    /* 1 = 起跑首拍方向自检尚未执行 */

/* ---- 一、工具 ---- */
static float pb_kappa_at (const ins_wp_t *p, uint16 n, uint32 i, float ds);

static float pb_wrap180 (float a)
{
    /* 归一化到 ±180 deg：NaN 返回 0，带环绕次数上限，病态输入夹到 ±180 */
    uint8 guard = 0u;

    if (!(a == a))                                     { return 0.0f; }   /* NaN */
    while ((a >  180.0f) && (guard < 64u)) { a -= 360.0f; guard ++; }
    guard = 0u;
    while ((a < -180.0f) && (guard < 64u)) { a += 360.0f; guard ++; }
    if (a >  180.0f) { a =  180.0f; }                /* 超过 64 圈 => 数值已病态，直接夹住 */
    if (a < -180.0f) { a = -180.0f; }
    return a;
}

static float pb_clampf (float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float pb_dist (const ins_wp_t *a, const ins_wp_t *b)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    return sqrtf(dx * dx + dy * dy);
}

/* ---- 二、速度剖面（曲率限速 + 反向刹车 + 最低速度地板 + 末点收油）----
 * 窗口点数走参数 pb_kwin；末段由末点锚 PB_END_CREEP + 反向刹车生成减速曲线 */
/* 点到折线的距离 m，用 i-1..i+1 的弦近似 */
static float pb_seg_dist (float px, float py, const ins_wp_t *p, uint16 n, uint16 i)
{
    uint16 i0 = i;
    uint16 i1 = i;
    float ax, ay, bx, by, vx, vy, wx, wy, dd, t;

    if (n < 2u) { return 0.0f; }
    if (i0 > 0u)               { i0 = (uint16)(i - 1u); }
    if (i1 < (uint16)(n - 1u)) { i1 = (uint16)(i + 1u); }
    ax = p[i0].x; ay = p[i0].y;
    bx = p[i1].x; by = p[i1].y;
    vx = bx - ax; vy = by - ay;
    wx = px - ax; wy = py - ay;
    dd = (vx * vx) + (vy * vy);
    t  = (dd > 1.0e-9f) ? ((wx * vx + wy * vy) / dd) : 0.0f;
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }
    wx = wx - (t * vx);
    wy = wy - (t * vy);
    return sqrtf((wx * wx) + (wy * wy));
}

static void pb_build_profile (uint16 n)
{
    const ins_wp_t *p = s_play;
    float alat = (pb_alat > 0.05f) ? pb_alat : 0.05f;
    float abrk = (pb_abrk > 0.05f) ? pb_abrk : 0.05f;
    float vmax = (pb_vmax > 0.05f) ? pb_vmax : 0.05f;
    float vmin = (pb_vmin > 0.00f) ? pb_vmin : 0.00f;
    int   win  = (int)pb_kwin;
    uint16 i;

    if (win < 1)  { win = 1; }
    if (win > 20) { win = 20; }
    if (vmin > vmax) { vmin = vmax; }

    /* ---- ① 曲率限速（倒车段不参与）----
       κ = |Δyaw| / 弧长，v = sqrt(alat/κ)；弧长逐段累加弦长，不用两端点直线距离。
       arc ≈ 0：原地打方向 ⇒ vmin（最急），静止重复采样 ⇒ vmax（无信息）。
       录制速度 < 0 的点是倒车段，直接取用录制速度并作为反向传播的屏障。 */
    for (i = 0u; i < n; i ++)
    {
        float  rec = p[i].v;
        uint16 i0, i1, k;
        float  arc = 0.0f;
        float  dyaw_abs;
        float  v;

        s_kap[i] = 0.0f;
        if (rec < -(float)PB_REV_EPS)
        {
            s_vlim[i] = rec;
            continue;
        }

        i0 = ((int)i - win > 0) ? (uint16)((int)i - win) : 0u;
        i1 = ((int)i + win < (int)n - 1) ? (uint16)((int)i + win) : (uint16)(n - 1u);
        if (s_mixed)
        {
            i0 = i;
            i1 = i;
            while (i0 > 0u && i - i0 < (uint16)win && s_dir[i0 - 1u] == s_dir[i]) { i0 --; }
            while (i1 + 1u < n && i1 - i < (uint16)win && s_dir[i1 + 1u] == s_dir[i]) { i1 ++; }
        }

        for (k = i0; k < i1; k ++) { arc += pb_dist(&p[k], &p[k + 1u]); }

        dyaw_abs = fabsf(pb_wrap180(p[i1].yaw - p[i0].yaw) * PB_D2R);

        if (arc < 0.02f)
        {
            /* 0.017 rad ≈ 1°：转过角度 ⇒ 原地打方向（最急）；没转 ⇒ 静止段 */
            v = (dyaw_abs > 0.017f) ? vmin : vmax;
        }
        else
        {
            float kap = dyaw_abs / arc;

            s_kap[i] = kap;                         /* 供 ①c 速率约束用 */
            v = (kap < 1e-3f) ? vmax : sqrtf(alat / kap);
        }

        if (v > vmax) { v = vmax; }
        if (v < vmin) { v = vmin; }
        s_vlim[i] = v;
    }

    /* ---- ①c 曲率变化率限速（转向速率约束）----
       δ = atan(L·κ) ⇒ |dδ/dt| = L·|κ'|·v / (1 + (Lκ)²) ≤ PB_SLEW_RATE_DEG_S。
       直道与定圆的 κ'≈0 不受限，只压边跑边打舵的 S 段。 */
    {
        const float rate = (float)PB_SLEW_RATE_DEG_S * PB_D2R;      /* rad/s */
        const ins_route_head_t *hp2 = &s_play_head;
        float ds_pt = ((0 != hp2) && (hp2->total_dist > 0.05f) && (n > 1u))
                    ? (hp2->total_dist / (float)(n - 1u)) : 0.1f;   /* 点距 m */

        for (i = 1u; i + 1u < n; i ++)
        {
            float dk;
            float vcap;

            float span = s_arc[i + 1u] - s_arc[i - 1u];
            if (span < 0.001f) { span = 2.0f * ds_pt; }
            dk = (s_kap[i + 1u] - s_kap[i - 1u]) / span;
            if (fabsf(dk) < 1.0e-3f) { continue; }

            {
                float lk = pb_l * s_kap[i];
                vcap = (rate * (1.0f + lk * lk)) / ((pb_l * fabsf(dk)) + 1.0e-6f);
            }

            if (s_vlim[i] > vcap) { s_vlim[i] = vcap; }
            if (s_vlim[i] < vmin) { s_vlim[i] = vmin; }
        }
    }

    /* ---- ①b 换向点前爬行 ----
       换向前那一点留速度，保证进度点能越过换向点（钉死 0 会永远换不过去）。 */
    for (i = 1u; i < n; i ++)
    {
        if ((s_vlim[i] < 0.0f) && (s_vlim[i - 1u] > 0.0f)) { s_vlim[i - 1u] = (float)PB_REV_CREEP; }
    }

    /* ---- ①c 末点锚爬行速度 ----
       只改最后一个前进点的速度，减速曲线由 ② 反向刹车生成（连续且不超过 pb_abrk）。
       creep 夹在 vmin~vmax，且只在前进段（倒车段是原样的负速）。 */
    {
        float creep = (float)PB_END_CREEP;

        if (creep < vmin) { creep = vmin; }
        if (creep > vmax) { creep = vmax; }

        for (i = (uint16)(n - 1u); i > 0u; i --)
        {
            if (s_vlim[i] >= 0.0f)
            {
                if (s_vlim[i] > creep) { s_vlim[i] = creep; }
                break;
            }
        }
    }

    /* ---- ② 反向传刹车：v[i-1] ≤ sqrt(v[i]² + 2·abrk·ds) ----
       换向点两侧不互传，否则 lim 会算成负数把前进段压死。 */
    for (i = (uint16)(n - 1u); i > 0u; i --)
    {
        float ds;
        float lim;

        if ((s_vlim[i] < 0.0f) || (s_vlim[i - 1u] < 0.0f)) { continue; }

        ds  = pb_dist(&p[i - 1u], &p[i]);
        lim = sqrtf(s_vlim[i] * s_vlim[i] + 2.0f * abrk * ds);
        if (s_vlim[i - 1u] > lim) { s_vlim[i - 1u] = lim; }
    }

    /* ---- ③ 地板抬回 vmin，只作用于前进段 ---- */
    for (i = 0u; i < n; i ++)
    {
        if ((s_vlim[i] >= 0.0f) && (s_vlim[i] < vmin)) { s_vlim[i] = vmin; }
    }

    if (s_mixed)
    {
        /* 混合表：各段减速到换向点，不再抬回 vmin */
        for (i = 1u; i < n; i ++)
        {
            uint16 k;
            float next = 0.0f;
            if (s_dir[i] == s_dir[i - 1u]) { continue; }
            for (k = i; k > 0u; k --)
            {
                float lim = sqrtf(next * next + 2.0f * abrk * pb_dist(&p[k - 1u], &p[k]));
                if (s_vlim[k - 1u] > lim) { s_vlim[k - 1u] = lim; }
                next = s_vlim[k - 1u];
                if (k > 1u && s_dir[k - 2u] != s_dir[k - 1u]) { break; }
            }
        }
    }

}

/* ---- 三、最近点（单调前进，不后退）----
 * 只在 [s_idx, s_idx + PB_NEAR_WIN] 内前向搜索：全局搜索会在邻近两条边上跳边 */
static uint16 pb_find_nearest (float px, float py, const ins_wp_t *p)
{
    uint16 i, best = s_idx;
    uint16 end = (uint16)(s_idx + PB_NEAR_WIN);
    float  best_d;

    if (end >= s_n) { end = (uint16)(s_n - 1u); }

    {
        float ex = px - p[s_idx].x;
        float ey = py - p[s_idx].y;
        best_d = ex * ex + ey * ey;
    }
    for (i = (uint16)(s_idx + 1u); i <= end; i ++)
    {
        float ex = px - p[i].x;
        float ey = py - p[i].y;
        float d  = ex * ex + ey * ey;

        if (d < best_d) { best_d = d; best = i; }
    }

    return best;
}

/* 倒车回放按里程查点：重叠分支的 x/y 相同但路线位置不同，几何不能用来推进度 */
static uint16 pb_find_mileage (float travel_m)
{
    uint16 best = s_idx;

    if (travel_m < 0.0f) { travel_m = 0.0f; }
    while (((uint16)(best + 1u) < s_n) &&
           (s_arc[best + 1u] <= (travel_m + 0.001f)))
    {
        best ++;
    }
    return best;
}

/* 沿弧长找前视点；混合表不越过换向点，到表尾封顶 */
static uint16 pb_find_lookahead (uint16 nearest, const ins_wp_t *p, float ld)
{
    uint16 i;
    float  acc = 0.0f;

    for (i = nearest; (uint16)(i + 1u) < s_n; i ++)
    {
        if (s_mixed && i >= s_shift_idx) { return i; }
        acc += pb_dist(&p[i], &p[i + 1u]);
        if (acc >= ld) { return (uint16)(i + 1u); }
    }
    return (uint16)(s_n - 1u);
}

/* ---- 四、起 / 停 ---- */
/* 最近一次没跑 / 结束的原因，屏幕与串口都用它 */
static char s_why[24] = "";

/* 收车原因码（遥测第 45 通道 pb_why）：
     1 head test busy   2 pb_en=0        3 RC ON       4 ESTOP
     5 odom off         6 done: finish   7 end idx     8 runaway 2x
     9 lost: orbit     10 drv preempted 11 ht busy    15 其它
   0 = 本次上电还没停过。 */
static uint8 s_why_code = 0u;

static void pb_set_why (const char *w)
{
    if (0 == w) { s_why[0] = '\0'; s_why_code = 0u; return; }
    snprintf(s_why, sizeof(s_why), "%s", w);

    if      (0 != strstr(w, "head test busy")) { s_why_code = 1u; }
    else if (0 != strstr(w, "pb_en=0"))        { s_why_code = 2u; }
    else if (0 != strstr(w, "RC ON"))          { s_why_code = 3u; }
    else if (0 != strstr(w, "ESTOP"))          { s_why_code = 4u; }
    else if (0 != strstr(w, "odom off"))       { s_why_code = 5u; }
    else if (0 != strstr(w, "done: finish"))   { s_why_code = 6u; }
    else if (0 != strstr(w, "end idx"))        { s_why_code = 7u; }
    else if (0 != strstr(w, "runaway"))        { s_why_code = 8u; }
    else if (0 != strstr(w, "lost: orbit"))    { s_why_code = 9u; }
    else if (0 != strstr(w, "drv preempted"))  { s_why_code = 10u; }
    else if (0 != strstr(w, "ht busy"))        { s_why_code = 11u; }
    else                                       { s_why_code = 15u; }
}

const char * route_pb_reason (void) { return s_why; }

/* 回中防抖：连续 PB_REL_TICKS 拍都处于已收车才写 0，滤掉单拍的 finish 毛刺 */
static void pb_release_tick (void)
{
    if (s_rel_ticks < (uint16)PB_REL_TICKS) { s_rel_ticks ++; }
    if (s_rel_ticks >= (uint16)PB_REL_TICKS) { steer_ctrl_set_target_delta(0.0f); }
}

static void pb_release_steer (void)
{
    /* 交还控制：先关航向外环，再交还转角目标 */
    if (0u != s_head)
    {
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
        s_own_target = 1u;
    }
    /* 退出写 0 回中，不对齐实测角：对齐等于把满舵保持住，人一推车就原地绕圈 */
    pb_release_tick();
}

static void pb_finish (uint8 st, const char *why)
{
    /* 资源释放无条件做，不挂在 if (PB_ST_RUN == s_st) 里，
       否则第二次起跑会带着上一趟的令牌、授权、斜坡与点序残留 */
    if (0u != s_gain_rev) { steer_ctrl_set_angle_pid(s_kp_fwd); }
    s_gain_rev = 0u;

    if (PROTO_SRC_NONE != s_drv)
    {
        proto_drv_release(s_drv);
        s_drv = PROTO_SRC_NONE;
    }

    s_ramp = 0u;                    /* 清停车斜坡，不带入下一次起跑 */
    spd_ctrl_set_coast(0u);         /* 退出时务必撤自由轮，否则遥控/跑班也推不动 */

    /* 交还本地授权，只清本模块那一位 */
    if (0u != s_local)
    {
        s_local = 0u;
        spd_safety_set_local_src(SPD_LOCAL_SRC_PB, 0u);
    }
    if (pb_mode >= 1.5f)
    {
        spd_target_l = 0.0f;
        spd_target_r = 0.0f;
    }

    if (PB_ST_RUN == s_st)
    {
        pb_release_steer();         /* 只有真的跑过才回中（PUSH 时别乱动方向盘） */
        printf("[PB] 结束（%s）state=%u idx=%u/%u\r\n",
               why, (unsigned)st, (unsigned)s_idx, (unsigned)s_n);
        pb_set_why((0 != why) ? why : "stop");
    }
    s_st = st;
    s_shifting = s_stopped = s_depart = 0u;
    s_own_target = 0u;
    s_yaw_ok = 0u;
    s_lpf_y = 0.0f;
}

static uint8 pb_start (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    uint16 n;

    n = ins_route_get_count();
    /* 录制中不许起跑，与 route_app.c 的回放中不许录制配成一对锁 */
    if (0u != ins_route_is_running()) { pb_set_why("recording"); return 0u; }
    if (n < (uint16)PB_MIN_POINTS)  { printf("[PB] 表太短（%u 点）\r\n", (unsigned)n); pb_set_why("no table"); return 0u; }
    if (ins_odom_enable < 0.5f)     { printf("[PB] 航位推算没开\r\n");                 pb_set_why("odom off"); return 0u; }

    /* ---- 倒序回放（ID77）----
       把点序倒过来，车正着开回起点；yaw 列与 steer 列都不用改。
       方向配对由 ID76/ID77 两个开关决定，不自洽时拒绝起跑 */
    {
        uint16 nrev  = ins_route_rev_count();
        uint8  isrev = ((uint16)(nrev * 2u) >= n) ? 1u : 0u;      /* 过半 ⇒ 倒车录的表 */
        uint8  want  = (rt_revplay >= 0.5f) ? 1u : 0u;
        const ins_wp_t *raw = ins_route_get_points();
        uint16 i;
        uint16 moving_fwd = 0u, moving_rev = 0u;
        uint8 mode = (rt_route_mode >= 0.0f && rt_route_mode < (float)RT_MODE_COUNT) ?
                     (uint8)(rt_route_mode + 0.5f) : 0xFFu;

        if (mode != 0xFFu)
        {
            want = (mode == RT_MODE_ZF || mode == RT_MODE_FZ || mode == RT_MODE_MIX_E) ? 1u : 0u;
        }

        for (i = 0u; i < n; i ++)
        {
            if (fabsf(raw[i].v) <= PB_RECORD_DIR_EPS) { continue; }
            if ((raw[i].flags & INS_WP_FL_REV) || (raw[i].v < -PB_RECORD_DIR_EPS)) { moving_rev ++; }
            else { moving_fwd ++; }
        }
        s_mixed = ((moving_fwd != 0u) && (moving_rev != 0u)) ? 1u : 0u;
        if (s_mixed && mode < RT_MODE_MIX_S)
        { pb_set_why("select mix_s/mix_e"); return 0u; }
        if (moving_rev != 0u) { isrev = 1u; }
        else if (moving_fwd != 0u) { isrev = 0u; }

        s_rev_tab = isrev;
        s_rev_ord = want;
        s_tab_off = ((s_rev_tab ^ s_rev_ord) != 0u) ? 180.0f : 0.0f;
        /* 1 = 表 yaw 与车头差 180° */
        /* 行驶方向 = RecDir(ID76) XOR PlayDir(ID77)：
           RecDir 0 = 表正着录 / 1 = 倒车录；PlayDir 0 = 顺表开 / 1 = 逆表开 */
        {
            uint8 rec = (rt_revrec  >= 0.5f) ? 1u : 0u;
            uint8 ply = want;

            s_drv_rev = (uint8)(isrev ^ ply);
            pb_drev   = (float)s_drv_rev;
            if (mode == 0xFFu && rec != s_rev_tab)
            {
                printf("[PB] !! 录制方向开关=%u 与表不符（表内 %u/%u 点标 REV）=> 可能用错表\r\n",
                       (unsigned)rec, (unsigned)nrev, (unsigned)n);
            }
        }

        s_nose_flip  = (uint8)((s_rev_tab ^ s_rev_ord) & 1u);
        /* 锚点转角恒为 0：车头 = 表 yaw 列，行进方向由 s_tab_off 与 s_rev_eff 各自处理 */
        s_anchor_off = 0.0f;

        s_play_head = *ins_route_get_head();
        /* 有符号速度与 REV 标志都描述到达该点的那一段；
           静止点继承最近的有效方向 */
        {
            uint8 rev = isrev;
            for (i = 0u; i < n; i ++)
            {
                if (fabsf(raw[i].v) > PB_RECORD_DIR_EPS)
                { rev = (uint8)((raw[i].flags & INS_WP_FL_REV) || raw[i].v < 0.0f); break; }
            }
            for (i = 0u; i < n; i ++)
            {
                if (fabsf(raw[i].v) > PB_RECORD_DIR_EPS)
                { rev = (uint8)((raw[i].flags & INS_WP_FL_REV) || raw[i].v < 0.0f); }
                s_dir[i] = rev;
            }
        }
        s_arc[0] = 0.0f;
        for (i = 0u; i < n; i ++)
        {
            s_play[i] = raw[(0u != want) ? (uint16)(n - 1u - i) : i];
            s_play[i].v = fabsf(s_play[i].v);
            if (i > 0u) { s_arc[i] = s_arc[i - 1u] + pb_dist(&s_play[i - 1u], &s_play[i]); }
        }
        if (want)
        {
            for (i = 0u; i < n / 2u; i ++)
            { uint8 tmp = s_dir[i]; s_dir[i] = s_dir[n - 1u - i]; s_dir[n - 1u - i] = tmp; }
            for (i = 0u; i < n; i ++) { s_dir[i] ^= 1u; }
        }
        else
        {
            for (i = 0u; i + 1u < n; i ++) { s_dir[i] = s_dir[i + 1u]; }
        }
        s_dir[n - 1u] = s_dir[n - 2u];
        s_drv_rev = s_dir[0];
        s_rev_eff = s_drv_rev;
        s_mileage = (s_mixed || s_drv_rev) ? 1u : 0u;
        s_shift_idx = (uint16)(n - 1u);
        for (i = 1u; i + 1u < n; i ++)
        { if (s_dir[i] != s_dir[i - 1u]) { s_shift_idx = i; break; } }
        s_shifting = s_stopped = s_depart = 0u;
        s_leg_start = 0u;
        pb_drev = (float)s_drv_rev;
        if (!s_mixed && mode < RT_MODE_MIX_S &&
            ((mode == RT_MODE_FZ) ? !isrev : isrev))
        { pb_set_why("route mode mismatch"); return 0u; }
        s_play_head.total_dist = s_arc[n - 1u];
    }

    p = s_play;
    h = &s_play_head;

    ins_odom_snapshot(&o);
    s_yaw_ok = 0u;
    s_yaw_off = 0.0f;
    s_lpf_y = 0.0f;
    s_lqr_kappa = 0.0f;
    s_err_w = 0.0f;
    s_dbg_best = s_dbg_fi = 0u;
    s_own_target = 0u;
    s_kp_fwd = steer.kp;
    s_gain_rev = s_drv_rev;
    steer_ctrl_set_angle_pid(s_drv_rev ? s_kp_fwd / 3.0f : s_kp_fwd);
    /* 起跑视为新控制会话：清掉速度环与转角环的历史记忆 */
    spd_ctrl_reset_int();
    steer_ctrl_set_angle_enable(0u);
    steer_ctrl_set_angle_enable(1u);
    steer_ctrl_set_head_enable(0u);
    s_head = 0u;
        /* 不带上一趟的直接转角指令进第一拍 */
    steer_ctrl_set_target_delta(0.0f);
    steer_ctrl_set_target_yaw(o.yaw);
    s_yaw_off = pb_wrap180(ins_imu_get_yaw() - p[0].yaw);
    s_yaw_ok = 1u;

    /* ---- 起点对齐 ----
       只把起跑那一刻的位姿记成基准 (s_o0*)：当前点 ≡ 表第 0 点、当前朝向 ≡ p[0].yaw，
       之后全用增量换算；表头 origin_x/y/yaw 不参与换算 */
    {
        float c0 = cosf(h->origin_yaw * PB_D2R);
        float s0 = sinf(h->origin_yaw * PB_D2R);
        float ex = ( (o.x - h->origin_x) * c0 + (o.y - h->origin_y) * s0 - p[0].x);
        float ey = (-(o.x - h->origin_x) * s0 + (o.y - h->origin_y) * c0 - p[0].y);
        float d0 = sqrtf(ex * ex + ey * ey);
        /* 期望车头 = 表首朝向 + 180×(倒车) */
        float yaw_exp = pb_wrap180(p[0].yaw + ((pb_drev >= 0.5f) ? 180.0f : 0.0f));
        float dyaw = pb_wrap180(o.yaw - h->origin_yaw - yaw_exp);

        s_o0x   = o.x;
        s_o0y   = o.y;
        s_o0yaw = o.yaw;

        /* 起跑朝向差只告警不拦车：整表随之旋转是自洽的，摆车问题由操作员判断 */
        if (fabsf(dyaw) > (float)PB_ALIGN_YAW_MAX)
        {
            printf("[PB] 注意: 起跑朝向与表首差 %d deg（表首朝向 %d deg，"
                   "odom %s）⇒ 整张表会被转 %d deg\r\n",
                   (int)dyaw, (int)p[0].yaw,
                   ((d0 > (float)PB_ALIGN_MAX_M) ? "换过/被挪过" : "同一上电周期"),
                   (int)dyaw);
        }

        /* 不用 %f：IAR 默认库不一定带浮点格式化 */

    }

    /* 起始进度点 0：与信念锚 p[0] 同源，避免起步点凭空多出航向误差 */
    s_idx  = 0u;
    s_rev  = 0u;
    s_rev_eff = s_drv_rev;
    s_estop_ticks = 0u;
    s_n    = n;
    s_len  = (h->total_dist > 0.5f) ? h->total_dist : 0.0f;
    /* ---- 自动选控制律（ID87 pb_cauto >= 0.5 时生效）----
       正向 LQR；倒车时表短且有余量用 mode0，否则 LQR；混合表一律 LQR。
       tight = 需求舵角超过 PB_AUTO_NEED_DEG 的点占比 */
    {
        float tight = 0.0f;
        if (n > 2u)
        {
            uint16 i;
            uint16 cnt_bad = 0u;
            for (i = 0u; (i + 1u) < n; i ++)
            {
                float dyw = pb_wrap180(p[i + 1u].yaw - p[i].yaw) * PB_D2R;
                float dx  = (float)(p[i + 1u].x - p[i].x);
                float dyy = (float)(p[i + 1u].y - p[i].y);
                float ds  = sqrtf((dx * dx) + (dyy * dyy));
                float dd;
                if (ds < 0.02f) { ds = 0.02f; }
                dd = atanf(pb_l * fabsf(dyw / ds)) * PB_R2D;
                if (dd > (float)PB_AUTO_NEED_DEG) { cnt_bad ++; }
            }
            tight = (float)cnt_bad / (float)(n - 1u);
        }
        if (s_mixed)
        {
            s_ctrl_eff = PB_CTRL_LQR;
        }
        else if (pb_cauto >= 0.5f)
        {
            if (0u == s_drv_rev)
            {
                s_ctrl_eff = PB_CTRL_LQR;
            }
            else if ((s_len < (float)PB_AUTO_LEN_M) && (tight < (float)PB_AUTO_TIGHT))
            {
                s_ctrl_eff = PB_CTRL_NOPP;
            }
            else
            {
                s_ctrl_eff = PB_CTRL_LQR;
            }
            printf("[PB] 控制律自动: %u (len %u cm, 紧点 %u%%)\r\n",
                   (unsigned)s_ctrl_eff, (unsigned)(s_len * 100.0f), (unsigned)(tight * 100.0f));
        }
        else
        {
            s_ctrl_eff = (uint8)(pb_ctrlmode + 0.5f);
            /* 手动档夹到合法范围，越界退回 LQR */
            if (s_ctrl_eff > PB_CTRL_MAX) { s_ctrl_eff = PB_CTRL_LQR; }
        }
    }
    s_d0   = o.dist_abs;
    s_leg_d0 = o.dist_abs;
    s_elat = 0.0f;
    s_dtrue = 0.0f;
    s_offcnt = 0u;
    s_ld   = 0.0f;
    s_tv   = 0.0f;
    s_lost_t0  = 0u;
    s_lost_idx = 0u;
    s_alpha    = 0.0f;
    s_aim  = s_idx;

    /* 起步诊断：起点需求舵角 δ₀、前 2 m 内贴极限的比例（仅打印）*/
    {
        float ds0 = ((h->count > 1u) && (h->total_dist > 0.1f)) ? (h->total_dist / (float)(h->count - 1u)) : 0.10f;
        float lim = (steer.limit_deg > 1.0f) ? steer.limit_deg : 27.0f;
        float k0  = pb_kappa_at(p, n, s_idx, ds0);
        float d0  = atanf(pb_l * k0) * PB_R2D;
        uint16 ii;
        uint16 tot = 0u;
        uint16 hit = 0u;

        for (ii = s_idx; (ii < n) && (ii < (uint16)(s_idx + 20u)); ii ++)
        {
            float kk = pb_kappa_at(p, n, ii, ds0);
            float dd = fabsf(atanf(pb_l * kk) * PB_R2D);
            tot ++;
            if (dd > (0.9f * lim)) { hit ++; }
        }
        if ((fabsf(d0) >= (float)PB_PRE_MIN_DEG) || (hit > (uint16)(tot / 2u)))
        {
            printf("[PB] 起步诊断: 该点需求 %.1f°，前 2 m 内 %u/%u 点贴极限(>%.1f°)\r\n",
                   (double)d0, (unsigned)hit, (unsigned)tot, (double)(0.9f * lim));
        }
    }

    pb_build_profile(n);

    /* 不用 %f：IAR 默认库不一定带浮点格式化；起跑日志每趟只打一次 */
    printf("[PB] 起跑: 槽%u %u点 从#%u 巡航%u 打角%u | 表%s序%s开%s\r\n",
           (unsigned)ins_route_slot_active(), (unsigned)n, (unsigned)s_idx,
           (unsigned)(pb_vmax * 100.0f), (unsigned)ins_route_steer_count(),
           ((0u != s_rev_tab) ? "倒" : "正"),
           ((0u != s_rev_ord) ? "倒" : "正"),
           ((0u != s_drv_rev) ? "倒" : "正"));

    if ((s_ctrl_eff < 0.5f) && (0u == ins_route_steer_count()))
    {
        printf("[PB] !! ID78=0 但本表没有打角点 => 倒车段也退回纯追踪\r\n");
    }
    if (((pb_vmax) < (SPD_DIFF_REV_V * 1.2f)))
    {
        /* 倒车差速有速度门限，低于门限后轴变刚轴，等效轴距变大 ⇒ 倒车回放切内弯 */
        printf("[PB] !! 巡航 %u cm/s 低于倒车差速门限 %u cm/s => 倒车回放会漂\r\n",
               (unsigned)(pb_vmax * 100.0f), (unsigned)(SPD_DIFF_REV_V * 120.0f));
    }
    if ((pb_ff_en >= 0.5f) && (0u == ins_route_steer_count()))
    {
    }

    s_done_armed = 0u;              /* 新起跑：重新武装完成判据 */
    s_ramp       = 0u;              /* 新起跑：清残留的停车斜坡 */

    s_drv = proto_drv_owner();
    if (PROTO_SRC_NONE == s_drv)
    {
        /* 没有令牌也能跑：记 INTERNAL 继续（PUSH 本来就不给后轮力）*/
        s_drv = PROTO_SRC_INTERNAL;
    }

    /* 起跑成功清掉上一次的失败原因 */
    pb_set_why("");
    s_dir_chk = 1u;   /* 新一次起跑重新做方向自检 */

    return 1u;
}

/* ---- 五、主循环 ---- */
void route_pb_init (void)
{
    s_st   = PB_ST_IDLE;
    s_arm  = 0u;
    s_done_armed = 0u;
    s_ramp = 0u;
    s_idx  = 0u;
    s_n    = 0u;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_tv   = 0.0f;
    s_head = 0u;
    s_drv  = PROTO_SRC_NONE;
    s_local = 0u;
    pb_en  = 0.0f;
    /* 所有跨趟状态归零，否则第二趟会带着上一趟的初值 */
    s_yaw_ok    = 0u;
    s_yaw_off   = 0.0f;
    s_lost_t0   = 0u;
    s_lost_idx  = 0u;
    s_offcnt    = 0u;
    s_estop_ticks = 0u;
    s_rel_ticks   = 0u;
    s_own_target  = 0u;
    s_tanref    = 0.0f;
    s_kref      = 0.0f;
    s_alpha     = 0.0f;
    s_err_w     = 0.0f;
    s_lpf_y     = 0.0f;
    s_ramp_t0   = 0u;
    s_ramp_l    = 0.0f;
    s_ramp_r    = 0.0f;
    s_lqr_kappa = 0.0f;
    /* 清 0 ⇒ 下一拍必按正向重设一次转角 PID 增益；s_kp_fwd 是正向标称值，不清 */
    s_gain_rev  = 0u;
    s_mixed = s_mileage = s_shifting = s_stopped = s_depart = 0u;
    s_dir_chk   = 1u;
    /* 不清 pb_ff_en / pb_ff_gain：本函数在 app_param_init() 之后调，清掉会抹掉 Flash 参数 */
}

/* ---- LQR 本体（ID78 = 2）----
 * 弧长域误差模型 e' = e_psi，e_psi' = u − kref，u 为曲率指令 1/m
 * 稳态增益 Ke = sqrt(qe/r)，Kth = sqrt((2·sqrt(qe·r) + qth)/r)，均与车速无关
 * 倒车只翻 δ 的号：K(−v) = K(v)·diag(1, −1)，横向项与曲率前馈不变 */

/* ---- 控制律调优开关（只有回放台会打开，车端默认全关）----
 * HS_KWIN 曲率半窗点数 / HS_KFRAC 前视索引小数插值 / HS_LPF 输出低通 Hz /
 * HS_DB 输出死区 deg / HS_VGATE 生效车速门限 m/s */
static uint16 s_tune_kwin    = 1u;
static uint8  s_tune_kfrac   = 0u;
static float  s_tune_lpf_hz  = 0.0f;
static float  s_tune_db_deg  = 0.0f;
static float  s_tune_vgate   = 0.0f;   /* >0 时只有 v ≥ gate 才用上面这些修法 */
static uint8  s_tune_gate_ok = 1u;
static uint8  s_tune_init    = 0u;

static void pb_tune_init (void)
{
    if (0u != s_tune_init) { return; }
    s_tune_init = 1u;
#ifdef HOST_SIM
    {
        const char *s;

        s = getenv("HS_KWIN");
        if (0 != s) { s_tune_kwin = (uint16)atoi(s); if (s_tune_kwin < 1u) { s_tune_kwin = 1u; } }
        s = getenv("HS_KFRAC");
        if (0 != s) { s_tune_kfrac = (uint8)(0 != atoi(s)); }
        s = getenv("HS_LPF");
        if (0 != s) { s_tune_lpf_hz = (float)atof(s); }
        s = getenv("HS_DB");
        if (0 != s) { s_tune_db_deg = (float)atof(s); }
        s = getenv("HS_VGATE");
        if (0 != s) { s_tune_vgate = (float)atof(s); }
    }
#endif
}

/* 表上某点的曲率 κ = −dψ/ds（正 = 右转），中心差分、端点单边、先解绕再算 */
static float pb_kappa_at (const ins_wp_t *p, uint16 n, uint32 i, float ds)
{
    uint32 i0;
    uint32 i1;
    float  d;

    if ((0 == p) || (n < 3u) || (ds < 0.02f)) { return 0.0f; }
    if (i >= (uint32)n) { i = (uint32)n - 1u; }
    i0 = (i > 0u) ? (i - 1u) : i;
    i1 = ((i + 1u) < (uint32)n) ? (i + 1u) : i;
    if (s_mixed)
    {
        if (i0 < s_leg_start) { i0 = s_leg_start; }
        if (i1 > s_shift_idx) { i1 = s_shift_idx; }
    }
    if (i1 == i0) { return 0.0f; }
    d = pb_wrap180(p[i1].yaw - p[i0].yaw);
    ds = pb_dist(&p[i0], &p[i]) + pb_dist(&p[i], &p[i1]);
    return (ds > 0.001f) ? -(d * PB_D2R) / ds : 0.0f;
}

/* 加窗曲率：w = 1 时与 pb_kappa_at 逐位等价；窗越宽越平滑，但会抹平反弯点 */
static float pb_kappa_win (const ins_wp_t *p, uint16 n, uint32 i, float ds, uint16 w)
{
    uint32 i0;
    uint32 i1;
    float  d;

    if (w <= 1u) { return pb_kappa_at(p, n, i, ds); }
    if ((0 == p) || (n < 3u) || (ds < 0.02f)) { return 0.0f; }
    if (i >= (uint32)n) { i = (uint32)n - 1u; }
    i0 = (i > (uint32)w) ? (i - (uint32)w) : 0u;
    i1 = ((i + (uint32)w) < (uint32)n) ? (i + (uint32)w) : ((uint32)n - 1u);
    if (s_mixed)
    {
        if (i0 < s_leg_start) { i0 = s_leg_start; }
        if (i1 > s_shift_idx) { i1 = s_shift_idx; }
    }
    if (i1 == i0) { return 0.0f; }
    d = pb_wrap180(p[i1].yaw - p[i0].yaw);
    ds = 0.0f;
    { uint32 k; for (k = i0; k < i1; k ++) { ds += pb_dist(&p[k], &p[k + 1u]); } }
    return (ds > 0.001f) ? -(d * PB_D2R) / ds : 0.0f;
}

/* LQR 转向：e 横向误差 m，eyaw_deg 航向误差 deg，kref 曲率参考 1/m。
   u = kref − Ke·e − Kth·e_psi；δ = atan(L·u)，倒车时 δ 取反 */
static float pb_ctrl_lqr (float e, float eyaw_deg, float kref)
{
    float qe, qth, r, ke, kth, gam, u, delta, lim;

    qe  = (ctrl_qe  > 1.0e-4f) ? ctrl_qe  : 1.0e-4f;
    qth = (ctrl_qth > 0.0f)    ? ctrl_qth : 0.0f;
    r   = (ctrl_r   > 1.0e-4f) ? ctrl_r   : 1.0e-4f;

    ke  = sqrtf(qe / r);
    kth = sqrtf(((2.0f * sqrtf(qe * r)) + qth) / r);
    gam = eyaw_deg * PB_D2R;

    u = (kref - (ke * e)) - (kth * gam);

    delta = atanf(pb_l * u) * PB_R2D;
    /* yaw_dot = -v*tan(delta)/L; kappa_travel = -d(yaw)/d|s|. */
    if (0u != s_rev_eff) { delta = -delta; }

    lim = steer_ctrl_eff_limit();
    if (delta >  lim) { delta =  lim; }
    if (delta < -lim) { delta = -lim; }

    s_lqr_kappa = u;
    return delta;
}

/* 到表折线的真实距离 m */
float route_pb_get_offdist (void) { return s_dtrue; }
float route_pb_get_tanref (void) { return s_tanref; }

float route_pb_get_alpha (void)
{
    return s_alpha;
}

float route_pb_get_errw (void)
{
    return s_err_w;
}

float route_pb_get_kref (void)
{
    return s_kref;
}

/* 收车原因码 → 遥测 CH_PB_WHY（见 pb_set_why 的码表） */
float route_pb_get_why_code (void)
{
    return (float)s_why_code;
}

/* 遥测：进度点索引 */
float route_pb_get_best (void)
{
    return (float)s_dbg_best;
}

float route_pb_get_fi   (void)
{
    return (float)s_dbg_fi;
}

/* LQR 上一拍曲率指令 1/m（正 = 右转）*/
float route_pb_get_kappa (void)
{
    return s_lqr_kappa;
}

void route_pb_poll (void)
{

    const ins_wp_t *p;
    ins_odom_snap_t o;
    float  px, py, pyaw;
    float  vabs, ld, elat;
    float  tx, ty, tyaw;
    float  dxr, dyr, pr, bx, by, alpha;
    uint16 j, best;
    uint32 fi;

    /* ---- 1) 安全门 ---- */
    if (head_test_active())
    {
        /* 航向测试优先：它也在写 target_yaw / spd_target_*，回放让位 */
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "head test busy"); }
        else                   { pb_set_why("ht busy"); }   /* 没在跑也要报原因 */
        s_arm = 0u;
        return;
    }
    if (pb_en < 0.5f)
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "pb_en=0"); }
        if (2u == s_arm) { pb_set_why(""); }        /* 用户关掉了：清掉上次的失败原因 */
        s_arm = 0u;
        return;
    }
    /* 安全门：RC ON / ESTOP 一律让位；FAILSAFE 只在 PWR 且链路也不在线时挡
       （PUSH 不给后轮力，不需要活着的遥控）*/
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_ESTOP == RC_Engage) ||
        ((RC_ENGAGE_FAILSAFE == RC_Engage) && (pb_mode >= 1.5f) &&
         (0u == motor_link_online())))
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "RC ON/ESTOP"); }
        /* 每次写原因，字串里带上解除方法 */
        if (RC_ENGAGE_ESTOP == RC_Engage)      { pb_set_why("ESTOP latch"); }
        else if (RC_ENGAGE_ON == RC_Engage)    { pb_set_why("RC ON: CH6->OFF"); }
        /* 只有既没接收机又没链路才走到这里（链路在线时上面的门不会进） */
        else                                   { pb_set_why("no RC & no link"); }
        s_arm = 0u;
        return;
    }
    /* 急停是中止回放而非暂停：否则解除那一拍车会直接窜出去 */
    if (0u != spd_safety_estop())
    {
        /* 去抖：安全层硬停机当拍生效，回放中止可慢 PB_ESTOP_TICKS 拍 */
        if (s_estop_ticks < (uint16)PB_ESTOP_TICKS) { s_estop_ticks ++; }
        if (s_estop_ticks >= (uint16)PB_ESTOP_TICKS)
        {
            if (PB_ST_RUN == s_st) { pb_finish(PB_ST_ABORT, "ESTOP"); pb_en = 0.0f; }
            else                   { pb_set_why("ESTOP latch"); }
            s_arm = 0u;
            return;
        }
    }
    else
    {
        s_estop_ticks = 0u;
    }
    if (ins_odom_enable < 0.5f)
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "odom off"); }
        else                   { pb_set_why("odom off"); }
        s_arm = 0u;
        return;
    }

    /* ---- 1a2) 终点停车斜坡 ----
       安全门之后执行，期间仍占驱动权与本地授权；不写舵机目标，保持最后舵角 */
    if (0u != s_ramp)
    {
        uint32 el = (uint32)(motor_link_now_ms() - s_ramp_t0);
        float  k;

        if (pb_mode < 1.5f)
        {
            /* 中途切 PUSH（人推）：立刻放手，别继续给力 */
            s_ramp = 0u;
            pb_finish(PB_ST_DONE, "done: finish");
            pb_en = 0.0f;
            return;
        }
        if (el >= (uint32)PB_STOP_MS)
        {
            s_ramp = 0u;
            spd_target_l = 0.0f;
            spd_target_r = 0.0f;
            pb_finish(PB_ST_DONE, "done: finish");
            pb_en = 0.0f;
            return;
        }

        k = 1.0f - ((float)el / (float)PB_STOP_MS);
        spd_target_l = s_ramp_l * k;
        spd_target_r = s_ramp_r * k;
        return;
    }

    /* ---- 1b) 只在 pb_en 的 0→1 上升沿启动：用状态位会在 DONE 后原地重跑 ---- */
    if (0u == s_arm)
    {
        s_arm = 1u;
        if (0u == pb_start())
        {
            /* 起跑失败不清 pb_en：保留用户意图，state 置 ABORT，s_arm=2 锁住重试 */
            s_arm = 2u;
            s_st  = PB_ST_ABORT;
            return;
        }
            s_st = PB_ST_RUN;
        pb_set_why("");

        /* 外环目标先落在当前航向，避免发车那一拍用上一次的 target_yaw */
        steer_ctrl_set_target_yaw(s_o0yaw);
    }
    /* 没在跑：继续走收车回中的驻留计数 */
    if (PB_ST_RUN != s_st) { pb_release_tick(); return; }
    /* 在跑：清零驻留计数，目标由下面的控制律重新写 */
    s_rel_ticks = 0u;

    /* ---- 1c) 驱动令牌 ----
       每拍续租；owner 变了（遥控抢占 / 菜单接管）立刻让位收车 */
    if (proto_drv_owner() != s_drv)
    {
        pb_finish(PB_ST_ABORT, "drv preempted");
        pb_en = 0.0f;                       /* 释放命令，否则下一拍会原地重新起跑 */
        s_arm = 0u;
        return;
    }
    proto_drv_renew(s_drv);

    /* ---- 2) 当前位姿（路表系，只用相对量）----
       位置与航向用同一个旋转角（刚体变换）：
       px,py = p[0] + R(θ)·(o − o0)，θ = s_o0yaw − p[0].yaw − s_anchor_off */
    p = s_play;
    ins_odom_snapshot(&o);
    {
        float dx0 = o.x - s_o0x;
        float dy0 = o.y - s_o0y;
        /* θ 含锚点项，否则车头与表反向时位置与航向不是刚体 */
        float th  = (s_o0yaw - p[0].yaw - s_anchor_off) * PB_D2R;   /* α 的相反数（含锚点项）*/
        float c   = cosf(th);
        float sn  = sinf(th);

        px   = p[0].x + ( dx0 * c + dy0 * sn);
        py   = p[0].y + (-dx0 * sn + dy0 * c);
        pyaw = pb_wrap180(p[0].yaw + s_anchor_off + (o.yaw - s_o0yaw));   /* R1 */
    }

    /* ---- 3) 进度点：正向按几何，倒车/混合按本趟里程查表 ---- */
    if (0u != s_mileage)
    {
        float travel = s_mixed ? (s_arc[s_leg_start] + (o.dist_abs - s_leg_d0)) :
                                 (o.dist_abs - s_d0);
        best = pb_find_mileage(travel);
    }
    else
    {
        best = pb_find_nearest(px, py, p);
    }
    if (s_mixed && s_shift_idx < s_n - 1u)
    {
        float travel = s_arc[s_leg_start] + (o.dist_abs - s_leg_d0);
        uint32 now = motor_link_now_ms();
        if (!s_shifting && travel >= s_arc[s_shift_idx] - 0.002f)
        {
            s_shifting = 1u;
            s_stopped = 0u;
            s_shift_t0 = now;
        }
        if (s_shifting)
        {
            /* 换向驻留：保持旧档位与打角，直到实测运动停止；安全门仍然有效 */
            spd_target_l = spd_target_r = 0.0f;
            s_tv = 0.0f;
            if (fabsf(o.v_mps) <= PB_SHIFT_STOP_MPS &&
                fabsf(enc_l.rps * ins_m_per_rev) <= PB_SHIFT_STOP_MPS &&
                fabsf(enc_r.rps * ins_m_per_rev) <= PB_SHIFT_STOP_MPS)
            {
                if (!s_stopped) { s_stopped = 1u; s_stopped_t0 = now; }
            }
            else { s_stopped = 0u; }
            if (s_stopped && (uint32)(now - s_stopped_t0) >= PB_SHIFT_HOLD_MS)
            {
                uint16 i;
                s_idx = best = s_shift_idx;
                /* 停止后重置里程基准，防刹车过冲推进下一小段 */
                s_leg_start = s_shift_idx;
                s_leg_d0 = o.dist_abs;
                s_drv_rev = s_dir[s_shift_idx];
                s_rev_eff = s_drv_rev;
                pb_drev = (float)s_drv_rev;
                s_shift_idx = (uint16)(s_n - 1u);
                for (i = (uint16)(s_leg_start + 1u); i + 1u < s_n; i ++)
                { if (s_dir[i] != s_dir[i - 1u]) { s_shift_idx = i; break; } }
                s_shifting = s_stopped = 0u;
                s_depart = 1u;
                s_depart_t0 = now;
                s_lpf_y = 0.0f;
                s_lost_t0 = 0u;
                spd_ctrl_reset_int();
                pb_set_why("");
            }
            else
            {
                s_idx = s_dbg_best = s_dbg_fi = (uint16)(s_shift_idx - 1u);
                pb_set_why("shift: stop");
                if ((uint32)(now - s_shift_t0) >= PB_SHIFT_TIMEOUT_MS)
                { pb_finish(PB_ST_ABORT, "shift timeout"); pb_en = 0.0f; }
                return;
            }
        }
        else if (best >= s_shift_idx) { best = (uint16)(s_shift_idx - 1u); }
    }
    if (s_mixed)
    {
        s_tab_off = s_drv_rev ? 180.0f : 0.0f;
        s_nose_flip = s_drv_rev;
    }
    s_idx = best;
    s_dbg_best = best;
    s_dbg_fi = best;

    /* 闭合表防误判完成：进度走过 3/4 表长才武装完成判据，
       否则首尾相近的表在起点就判 DONE */
    {
        uint16 need = (uint16)((s_n * 3u) / 4u);

        if (need < 4u)                      { need = 4u; }
        if (need > (uint16)(s_n - 1u))      { need = (uint16)(s_n - 1u); }
        if (best >= need)                   { s_done_armed = 1u; }
    }

    /* ---- 4) 完成 / 中止 ----
       进终点圈 ⇒ DONE；索引到末点但仍远离末点 ⇒ ABORT（脱线/跳点） */
    {
        /* 跑了 2 倍表长还没到末点 ⇒ 判跑飞 */
        if ((s_len > 0.5f) &&
            ((o.dist_abs - s_d0) > ((2.0f * s_len) + 2.0f)) &&
            (s_idx < (uint16)(s_n - 1u)))
        {
            pb_finish(PB_ST_ABORT, "runaway 2x len");
            pb_en = 0.0f;
            return;
        }

        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;
        float ld2 = sqrtf(lx * lx + ly * ly);
        float travel = s_mixed ? (s_arc[s_leg_start] + (o.dist_abs - s_leg_d0)) :
                                 (o.dist_abs - s_d0);
        float remaining = s_play_head.total_dist - travel;
        if (remaining < 0.0f) { remaining = 0.0f; }

        /* 完成判据：里程通道看剩余弧长，几何通道看离末点距离，两者都用 pb_fin */
        if (((0u != s_mileage) && (!s_mixed || s_shift_idx == s_n - 1u) &&
             (remaining < ((pb_fin > 0.02f) ? pb_fin : 0.02f))) ||
            ((0u == s_mileage) && (0u != s_done_armed) &&
             (ld2 < ((pb_fin > 0.02f) ? pb_fin : 0.02f))))
        {
            /* 先走停车斜坡再 finish，避免 spd_target 阶跃；只对 PWR 做，
               目标已≈0 则跳过 */
            if ((pb_mode >= 1.5f) && (0u == s_ramp) &&
                ((fabsf(spd_target_l) + fabsf(spd_target_r)) > 0.03f))
            {
                s_ramp_l  = spd_target_l;
                s_ramp_r  = spd_target_r;
                s_ramp_t0 = motor_link_now_ms();
                s_ramp    = 1u;
                return;
            }

            pb_finish(PB_ST_DONE, "done: finish");
            pb_en = 0.0f;
            return;
        }
        if (best >= (uint16)(s_n - 1u))
        {
            pb_finish(PB_ST_ABORT, "end idx, far off");
            pb_en = 0.0f;
            return;
        }
    }

    /* ---- 5) 横向误差（对着最近点的切线投影；正 = 车在参考线右侧）---- */
    tx   = p[best].x;
    ty   = p[best].y;
    tyaw = p[best].yaw * PB_D2R;
    elat = (px - tx) * cosf(tyaw) + (py - ty) * sinf(tyaw);
    s_elat = elat;
    /* 脱表监测：elat 只是一维横向投影，沿切线跑偏时≈0，真实距离用 s_dtrue */
    s_dtrue = pb_seg_dist(px, py, p, s_n, best);
    /* 脱表不停车，只计数 + 限频告警 */
    if (s_dtrue > (float)PB_OFF_DIST)
    {
        if (s_offcnt < 60000u) { s_offcnt ++; }
        if (s_offcnt == (uint16)PB_OFF_TICKS)
        {
            printf("[PB] !! 已脱表 %.1f m 连续 %u 拍（不停车，仅告警）idx=%u state=%u\r\n",
                   (double)s_dtrue, (unsigned)s_offcnt, (unsigned)s_idx, (unsigned)route_pb_state());
        }
        else if ((s_offcnt > (uint16)PB_OFF_TICKS) && (0u == (s_offcnt % (uint16)(0.5f / 0.02f))))
        {
            printf("[PB] !! 仍在脱表 %.1f m（idx=%u）\r\n",
                   (double)s_dtrue, (unsigned)s_idx);
        }
    }
    else
    {
        if (s_offcnt >= (uint16)PB_OFF_TICKS)
        {
            printf("[PB] 已回到轨道上（脱表 %u 拍后）\r\n", (unsigned)s_offcnt);
        }
        s_offcnt = 0u;
    }

    /* ---- 6) 目标速度取最近点限速：速度跟当前弧段，弯道不提前拉满 ---- */
    {
        float v = s_vlim[best];
        float mpr;

        if (s_mixed && s_depart && best == s_leg_start &&
            (uint16)(best + 1u) < s_n)
        { v = s_vlim[best + 1u]; }

        if (s_mixed && s_shift_idx < s_n - 1u)
        {
            float remain = s_arc[s_shift_idx] - s_arc[s_leg_start] - (o.dist_abs - s_leg_d0);
            float abrk = (pb_abrk > 0.05f) ? pb_abrk : 0.05f;
            float cap = sqrtf(2.0f * abrk * ((remain > 0.0f) ? remain : 0.0f));
            /* 换向前用爬行速度收尾，驻留阶段再请求 0 */
            if (cap < PB_SHIFT_CREEP) { cap = PB_SHIFT_CREEP; }
            if (v > cap) { v = cap; }
        }
        if (s_depart)
        {
            float elapsed = (float)(uint32)(motor_link_now_ms() - s_depart_t0) * 0.001f;
            float cap = PB_SHIFT_CREEP + ((pb_abrk > 0.05f) ? pb_abrk : 0.05f) * elapsed;
            if (v > cap) { v = cap; }
            else { s_depart = 0u; }
        }

        /* 下界不写死 0：倒车段是负速，只夹 ±pb_vmax */
        if (0u != s_drv_rev) { v = -v; }    /* 倒车 = 整条剖面翻号 */
        if (v >  pb_vmax) { v =  pb_vmax; }
        if (v < -pb_vmax) { v = -pb_vmax; }
        s_tv = v;

        if (pb_mode < 1.5f)
        {
            /* 6.1 无动力跟随（人推车）：目标写 0 且置自由轮，闭环不会反向刹车；
               测速/里程照跑，推车时位姿还要用 */
            spd_target_l = 0.0f;
            spd_target_r = 0.0f;
            spd_ctrl_set_coast(1u);
        }
        else
        {
            float vl, vr;

            spd_ctrl_set_coast(0u);         /* 带速度跟随：先撤自由轮，再给目标 */

            /* 拿本地授权：不拿则 SAFE_BIT_NOT_EN 一直挂着，电机不出力 */
            if (0u == s_local)
            {
                /* 按来源点灯：遥控会在自己的分支里每 5 ms 清零 */
                spd_safety_set_local_src(SPD_LOCAL_SRC_PB, 1u);
                s_local = 1u;
            }

            mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;

            /* ---- 6b) 电子差速 ----
               v_l = v(1 − κW/2)，v_r = v(1 + κW/2)，κ = tan(δ_meas)/L。
               用实测舵角；平均速度恒为 v，剖面限速仍成立 */
            spd_diff_alloc(v, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

            spd_target_l = vl / mpr;
            spd_target_r = vr / mpr;
        }
    }

    /* ---- 7) 前视距离 ld = a + b·|v_tgt| ----
       用目标速度而非实测速度：剖面已按几何算好该多快，跟实测会被打滑带偏 */
    vabs = fabsf(s_tv);
    ld   = pb_ld_a + pb_ld_b * vabs;
    ld   = pb_clampf(ld, pb_ld_min, pb_ld_max);
    if (ld < 0.05f) { ld = 0.05f; }

    /* s_alpha 供 LQR / 6c 使用 */
    s_ld = ld;

    /* ---- 6c) 跟丢保护 ----
       舵角贴限位 (0.95×limit) + elat > PB_LOST_MIN_M + 索引 PB_LOST_MS 内未前进
       ⇒ 判绕圈收车 */
    {
        float  dmeas = fabsf(steer_ctrl_get_meas_delta());
        uint32 now   = (uint32)motor_link_now_ms();

        if ((dmeas >= (0.95f * steer.limit_deg)) && (s_elat > (float)PB_LOST_MIN_M))
        {
            if (0u == s_lost_t0)
            {
                s_lost_t0  = now;
                s_lost_idx = s_idx;
            }
            else if (s_idx >= (uint16)(s_lost_idx + 3u))
            {
                s_lost_t0  = now;              /* 还在往前走 ⇒ 重新计时 */
                s_lost_idx = s_idx;
            }
            else if ((uint32)(now - s_lost_t0) > (uint32)PB_LOST_MS)
            {
                printf("[PB] !! 满舵 %u ms 而点序卡在 #%u、elat %u cm ⇒ 判定跟丢\r\n",
                       (unsigned)(now - s_lost_t0), (unsigned)s_idx,
                       (unsigned)(s_elat * 100.0f));

                pb_finish(PB_ST_ABORT, "lost: orbit");
                pb_en      = 0.0f;             /* 释放命令，否则下一拍会原地重新起跑 */
                s_arm      = 0u;
                s_lost_t0  = 0u;
                return;
            }
        }
        else
        {
            s_lost_t0 = 0u;
        }
    }

    /* ---- 8) 倒车段判定 ----
       PP 的航向外环对车尾是正反馈 ⇒ 关外环、只走内环 + 开环录制打角。
       判据用录制速度 p[best].v（剖面在换向前被压到接近 0），带 PB_REV_EPS 滞回 */
    s_rev = s_mixed ? s_drv_rev : 0u;

    /* 实际行进方向 = 逐点符号 或 ID85 的全局选择 */
    s_rev_eff = (uint8)(((0u != s_rev) || (0u != s_drv_rev)) ? 1u : 0u);
    /* 倒车用 1/3 内环增益，否则转角环振 */
    {
        uint8 want_bg = (0u != s_rev_eff) ? 1u : 0u;
        if (0.0f == s_kp_fwd) { s_kp_fwd = (steer.kp > 1.0f) ? steer.kp : 400.0f; }
        if (want_bg != s_gain_rev)
        {
            s_gain_rev = want_bg;
            steer_ctrl_set_angle_pid((0u != want_bg) ? (s_kp_fwd / 3.0f) : s_kp_fwd);
        }
    }

    /* ID78 = 0 无 PP：与倒车段同一条通道 —— 最近点录制打角 + 航向 P 纠偏，无前视点。
       只在倒车/反向时进入，正向落到下面的纯追踪 */
    if ((0u != s_rev_eff) && (s_ctrl_eff < 1.5f))
    {
        float delta = (float)p[best].steer_cdeg / 100.0f;
#ifdef HOST_SIM
        {
            const char *e2 = getenv("HS_NEGREV");
            if ((0 != e2) && (e2[0] == 49)) { delta = -delta; }
        }
#endif
        /* 录制那一拍实际施加的角 deg */

        /* 倒车航向 P 增益 */
        float k_rv  = pb_rvk;   /* 倒车航向纠偏增益（ID86）*/
#ifdef HOST_SIM
        { const char *e3 = getenv("HS_RVK"); if (0 != e3) { k_rv = (float)atof(e3); } }
#endif
        /* 世界系航向误差：ref − (imu − 零位)；用表内 yaw 相减的写法恒≈0 */
        float ref_w = pb_wrap180(p[best].yaw);
        if (0u == s_yaw_ok)
        {   /* 首拍把表的零位与 IMU 零位对齐（pb_start 里 p 还没赋值） */
            float ref0 = pb_wrap180(p[0].yaw);
            s_yaw_off = pb_wrap180(ins_imu_get_yaw() - ref0);
            s_yaw_ok  = 1u;
        }
        float imu_tab = pb_wrap180(ins_imu_get_yaw() - s_yaw_off);   /* 零位对齐后的实测航向 */
        float err_w   = pb_wrap180(ref_w - imu_tab);
        s_err_w = err_w;
        float corr  = -PB_NOPP_REV_SIGN * k_rv * err_w;
        if (corr >  (float)PB_NOPP_REVCORR) { corr =  (float)PB_NOPP_REVCORR; }
        if (corr < -(float)PB_NOPP_REVCORR) { corr = -(float)PB_NOPP_REVCORR; }

        /* 该点没有打角记录（flags bit0=0）时只用航向 P，不当作直行 */
        if (0u == (p[best].flags & (uint16)INS_WP_FL_STEER)) { delta = 0.0f; }

        delta += corr;
        /* 用实际生效限位夹（任何往 target 写角度的代码都要用它）*/
        {
            float lim = steer_ctrl_eff_limit();
            if (delta >  lim) { delta =  lim; }
            if (delta < -lim) { delta = -lim; }
        }

        if (0u != s_head)
        {
            steer_ctrl_set_head_enable(0u);     /* 关掉航向外环 */
            s_head = 0u;
        }
        s_own_target = 1u;                  /* 倒车段也是路线层独占 */
        steer_ctrl_set_target_delta(delta);     /* 只走内环 */

        s_ytgt = pb_wrap180(o.yaw + err_w);     /* 仅遥测 */
        s_aim  = best;

        /* 速度已在第 6 步按 s_vlim[best] 下发 */
        return;
    }
    else if (0u == s_head)
    {
        /* 回到前进段：重新打开外环 */
        steer_ctrl_set_head_enable(1u);
        s_head = 1u;
    }

    /* ---- 9) 沿弧长找前视点 ---- */
    s_aim = pb_find_lookahead(best, p, ld);
    s_dbg_best = best;
    s_dbg_fi   = s_aim;
    j     = s_aim;

    /* ---- 10) 转向：车体系 x 向右 / y 向前，alpha 正 = 目标点在右；
       steer_delta > 0 = 右转，yaw 正 = 左转 ⇒ 目标航向 = 当前航向 − alpha ---- */
    dxr = p[j].x - px;
    dyr = p[j].y - py;
    pr  = pyaw * PB_D2R;
    bx  =  dxr * cosf(pr) + dyr * sinf(pr);
    by  = -dxr * sinf(pr) + dyr * cosf(pr);
    alpha = atan2f(bx, by);                 /* rad，正 = 右 */
    s_alpha = alpha;                        /* 上一拍前视偏角，供遥测 */

    if (s_ctrl_eff >= 1.5f)
    {
        /* ---- 10c) LQR：参考 = 最近点曲率 + 横/航误差反馈，无前视点 ---- */
        const ins_route_head_t *hh = &s_play_head;
        float ds  = ((s_n > 2u) && (0 != hh)) ? (hh->total_dist / (float)(s_n - 1u)) : 0.1f;
        /* 航向误差比的是行进切向与行进朝向：
           T = psi_best + 180×(表倒录 XOR 倒序)，H = psi_meas + 180×V，误差 = T − H */
        float tan_ref = pb_wrap180(p[best].yaw + s_tab_off);
        s_tanref = tan_ref;
        float psi_tr  = pb_wrap180(pyaw + ((0u != s_rev_eff) ? 180.0f : 0.0f));
        float eyaw    = pb_wrap180(tan_ref - psi_tr);      /* deg，正 = 表更靠左 */

        /* 首拍方向自检：切向与车头差 > 120° 说明方向开关与表不自洽，拒绝起跑 */
        if (0u != s_dir_chk)
        {
            s_dir_chk = 0u;
            if (fabsf(eyaw) > 120.0f)
            {
                pb_finish(PB_ST_ABORT, "dir mismatch");
                pb_en = 0.0f;
                return;
            }
        }
        float kref;
        float delta;

        ds   = (ds > 0.02f) ? ds : 0.1f;
        pb_tune_init();
        /* 门控用车速 o.v_mps：vabs 是计划速度，PUSH 时恒为 pb_vmax */
        s_tune_gate_ok = (uint8)((s_tune_vgate <= 0.0f) ||
                                 (fabsf(o.v_mps) >= s_tune_vgate));
        {
            float dprev = vabs * ctrl_tlead;
            float xoff;
            float k2;
            float fr;

#ifdef HOST_SIM
            {   /* HS_PREV=1：预览距离改成 d = 0.13·v² + 0.03 */
                const char *p1 = getenv("HS_PREV");
                if ((0 != p1) && (p1[0] == 49) && (0u != s_tune_gate_ok))
                {
                    dprev = (0.13f * vabs * vabs) + 0.03f;
                }
            }
#endif
            fi = best;
            while ((fi + 1u < s_n) && (!s_mixed || fi < s_shift_idx) &&
                   (s_arc[fi] - s_arc[best] < dprev)) { fi ++; }
            xoff = (float)(fi - best);
            kref = pb_kappa_win(p, s_n, fi, ds,
                                (0u != s_tune_gate_ok) ? s_tune_kwin : 1u);
            if ((0u != s_tune_kfrac) && (0u != s_tune_gate_ok))
            {   /* 取整丢掉的余量按弧长线性补回，曲率随 s 连续 */
                fr   = (xoff + 0.5f) - floorf(xoff + 0.5f);
                k2   = pb_kappa_win(p, s_n, fi + 1u, ds,
                                    (0u != s_tune_gate_ok) ? s_tune_kwin : 1u);
                kref = kref + (fr * (k2 - kref));
            }
        }
        s_dbg_best = best;
        s_dbg_fi   = (uint16)fi;      /* 原始未夹值，越界会显示成大数 */
        s_kref = kref;
        /* 行进切向与表切向差 180° 时，横向误差与曲率符号各翻一次 */
        {
            /* e 在车体系里量，换算到行进系只差一个符号；kref 由 yaw 列差分得到，
               数组倒序后已经是行进系，不翻号 */
            float sg_e = ((0u != s_nose_flip) ? -1.0f : 1.0f);   /* 表 yaw 与车头差 180° 时左右手系相反 */
#ifdef HOST_SIM
            {
                const char *e1 = getenv("HS_SGE");
                if ((0 != e1) && (e1[0] == 49)) { sg_e = ((s_tab_off > 90.0f) ? -1.0f : 1.0f); }
                if ((0 != e1) && (e1[0] == 50)) { sg_e = -sg_e; }
            }
#endif
            delta = pb_ctrl_lqr(sg_e * elat, eyaw, kref);
        }

        /* 归属仲裁：本通道每拍无条件关外环，不依赖 s_head 镜像变量（会失步）*/
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
        s_own_target = 1u;
        s_ytgt = pb_wrap180(o.yaw - (alpha * PB_R2D));    /* 仅遥测 */
        if ((0.0f < s_tune_db_deg) && (0u != s_tune_gate_ok))
        {   /* 死区：|delta| 小于阈值归 0 */
            if ((delta < s_tune_db_deg) && (delta > -s_tune_db_deg)) { delta = 0.0f; }
        }
        if ((0.0f < s_tune_lpf_hz) && (0u != s_tune_gate_ok))
        {   /* 输出一阶低通，按 5 ms 控制拍算系数 */
            float lpf_a = 6.2831853f * s_tune_lpf_hz * 0.005f;

            if (lpf_a > 1.0f) { lpf_a = 1.0f; }
            s_lpf_y += lpf_a * (delta - s_lpf_y);
            delta = s_lpf_y;
        }
        steer_ctrl_set_target_delta(delta);
        return;
    }

    if (pb_smode < 0.5f)
    {
        /* ---- 9a) 航向外环 ----
           δ = k·(yaw_tgt − yaw)，k = 2L/ld 时与纯追踪等价；增益按 ld 现算 ---- */
        float k = (pb_kh > 1.0e-3f) ? pb_kh : (2.0f * pb_l / ld);

        k = pb_clampf(k, 0.3f, 8.0f);
        /* 倒车时车头背着目标点指（+180）*/
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D + ((0u != s_rev_eff) ? 180.0f : 0.0f));

        steer_ctrl_set_head_kp(k);
        /* 归属仲裁：9a 由航向环独占目标转角，每拍声明一次 */
        steer_ctrl_set_head_enable(1u);
        s_head = 1u;
        s_own_target = 0u;
        steer_ctrl_set_target_yaw(s_ytgt);

        /* 航向通道不吃打角前馈：前馈给 δ，入口是 yaw；要用手工表请 pb_smode=1 */
    }
    else
    {
        /* ---- 9b) 直接给 δ：κ = 2·sin(α)/ld，δ = atan(L·κ) ---- */
        float kappa = 2.0f * sinf(alpha) / ld;
        float delta = atanf(pb_l * kappa) * PB_R2D;

        /* 归属仲裁：本通道每拍无条件关外环，不依赖 s_head 镜像变量（会失步）*/
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
        s_own_target = 1u;
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D + ((0u != s_rev_eff) ? 180.0f : 0.0f));

        /* 打角前馈：δ = w·steer[best] + (1−w)·δ_pp；不能再叠一层，
           否则双倍打角（弯里切内、直道来回摆）*/
        if ((pb_ff_en >= 0.5f) && (0u != (p[best].flags & (uint16)INS_WP_FL_STEER)))
        {
            float w  = pb_clampf(pb_ff_gain, 0.0f, 1.0f);
            float st = (float)p[best].steer_cdeg / 100.0f;

            delta = w * st + (1.0f - w) * delta;
        }
        steer_ctrl_set_target_delta(delta);
    }
}

/* ---- 六、读出接口（遥测/调试）---- */
/* 每 5 ms 由中断调用：只按用户意图与谁在开车决定自由轮，
   不能依赖状态机走到 PUSH 分支 */
void route_pb_sync_coast (void)
{
    uint8 push = 0u;

    if ((pb_en > 0.5f) && (pb_mode < 1.5f))
    {
        /* 遥控在开车 / 急停 / 航向测试 ⇒ 不许自由轮 */
        if ((RC_ENGAGE_ON != RC_Engage) && (RC_ENGAGE_ESTOP != RC_Engage) &&
            (0u == head_test_active()))
        {
            push = 1u;
        }
    }
    spd_ctrl_set_coast(push);
}

/* 起跑失败后允许重试一次：s_arm 锁在 2，菜单写 Follow 时调 */
void  route_pb_rearm   (void)
{
    /* 已在跑则先收车，再清重试锁 */
    if (PB_ST_RUN == s_st)
    {
        pb_finish(PB_ST_IDLE, "rearm");
    }
    s_arm = 0u;
    pb_set_why("");
}

void  route_pb_stop    (const char *why) { pb_finish(PB_ST_IDLE, why); }

float route_pb_elat    (void) { return s_elat; }
uint8 route_pb_active  (void) { return (PB_ST_RUN == s_st) ? 1u : 0u; }
uint8 route_pb_state   (void) { return s_st; }

/* 1 = 正在回放：target_delta 归路线层，航向环不许覆写 */
uint8 route_pb_is_running (void) { return (PB_ST_RUN == s_st) ? 1u : 0u; }

/* 1 = 本拍 target_delta 由 route_pb 写（9b / 倒车段），0 = 交给航向环（9a）；
   不能用 route_pb_is_running() 代替 */
uint8 route_pb_owns_target (void) { return (PB_ST_RUN == s_st) ? s_own_target : 0u; }
float route_pb_ld      (void) { return s_ld; }
float route_pb_tv      (void) { return s_tv; }
float route_pb_yaw_tgt (void) { return s_ytgt; }
uint16 route_pb_aim    (void) { return s_aim; }
uint16 route_pb_index  (void) { return s_idx; }
