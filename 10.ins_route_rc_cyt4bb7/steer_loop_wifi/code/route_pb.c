/* 工程 10（卡丁）· 路表回放 Pure Pursuit 实现（设计意图见 route_pb.h） */
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>

#include "ins_odom.h"
#include "ins_route.h"
#include "remote_ctrl.h"
#include "spd_ctrl.h"
#include "steer_ctrl.h"
#include "route_pb.h"

#define PB_D2R      (0.01745329252f)
#define PB_R2D      (57.29577951f)

/* ---- 参数（绑定到参数表 ID 43~55）---- */
volatile float pb_en      = 0.0f;
volatile float pb_mode    = 1.0f;       /* 默认"只给转向"：最安全的验收模式 */
volatile float pb_vmax    = 1.50f;
volatile float pb_alat    = 1.20f;      /* m/s^2 */
volatile float pb_abrk    = 0.80f;      /* m/s^2 */
/* - 几何实测：前轴中心→后轴中心 **0.63 m**（卷尺，纵向垂直量）。
 * - 但本参数要的是"命令 δ → 得到多少 κ"的比值：`L_eff = tan(δ_read)/κ_measured`。
 * 手推（无滑移）两次 ±满舵反推 **0.648**（两次差 0.18%），驱动工况 0.68（内轮打滑抬高里程）。
 * - 与几何差 2.9%，全部落在"δ 读数空间 vs 真实轮角"或"里程刻度"里，**量长度分辨不出来**。
 * ⇒ 填 0.648（与 27° 软限位同源）；填 0.63 会让每拍少转 2.9%，等于把等效前视拉长 2.9%。
 * （那是"高估车的能力 2.9%"，方向危险 —— 规划工具 `cone_map_route.py` 里同理）。
 * R(IMU+里程)=1.305 / R(纯计数+几何W)=1.298 ⇒ L_eff=0.658 ⇒ **定 0.66**。
 * 几何 0.63 与 0.66 差 4.3% ＝「27° 读数 ≈ 25.9° 真实轮角」，数学等价。
 * 加了电子差速之后这一点才会好转，届时可重测一次作验证。
 */
volatile float pb_l       = PB_L_DEFAULT;   /* m（有效轴距；几何实测 0.63，见上面的长注释） */
volatile float pb_ld_a    = 0.35f;
volatile float pb_ld_b    = 0.20f;
volatile float pb_ld_min  = 0.30f;
volatile float pb_ld_max  = 1.50f;
volatile float pb_ff_en   = 0.0f;
volatile float pb_ff_gain = 1.00f;

/* ---- 状态 ---- */
static uint8   s_on      = 0u;
static uint16  s_idx     = 0u;          /* 进度索引（单调，不后退） */
static uint16  s_n       = 0u;          /* 开跑时快照的点数 */
static float   s_elat    = 0.0f;
static float   s_vlim[INS_ROUTE_MAX_POINTS];   /* 每点限速（m/s），2040 × 4 B = 8 KB */

static float pb_wrap180 (float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
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

/* 一、速度剖面（曲率限速 + 反向刹车） */
static void pb_build_profile (uint16 n)
{
    const ins_wp_t *p = ins_route_get_points();
    float alat  = (pb_alat > 0.05f) ? pb_alat : 0.05f;
    float abrk  = (pb_abrk > 0.05f) ? pb_abrk : 0.05f;
    float vmax  = (pb_vmax > 0.05f) ? pb_vmax : 0.05f;
    uint16 i;

    for (i = 0u; i < n; i ++)
    {
        uint16 i0 = (i > 2u) ? (uint16)(i - 2u) : 0u;
        uint16 i1 = (uint16)((i + 2u < n) ? (i + 2u) : (n - 1u));
        float  arc = pb_dist(&p[i0], &p[i1]);
        float  dyaw;
        float  kap;
        float  v;

        /* 曲率用**中心差分**：航向差 / 弧长。 */
        dyaw = pb_wrap180(p[i1].yaw - p[i0].yaw) * PB_D2R;
        kap  = (arc > 1e-3f) ? fabsf(dyaw) / arc : 0.0f;

        v = (kap > 1e-4f) ? sqrtf(alat / kap) : vmax;
        if (v > vmax) { v = vmax; }
        s_vlim[i] = v;
    }

    /* 反向传刹车：进弯前就要减到位（只做前向限制的典型坏味道是"入弯冲出去"） */
    for (i = (uint16)(n - 1u); i > 0u; i --)
    {
        float ds  = pb_dist(&p[i - 1u], &p[i]);
        float lim = sqrtf(s_vlim[i] * s_vlim[i] + 2.0f * abrk * ds);

        if (s_vlim[i - 1u] > lim) { s_vlim[i - 1u] = lim; }
    }
}

/* 二、起停 */
static uint8 pb_start (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    float y0, dx, dy, px, py;
    float best_d = 1e9f;
    uint16 n, i, best = 0u;

    n = ins_route_get_count();
    if (n < (uint16)PB_MIN_POINTS)      { return 0u; }
    if (ins_odom_enable < 0.5f) { return 0u; }

    p = ins_route_get_points();
    h = ins_route_get_head();

    ins_odom_snapshot(&o);

    /* 车当前位姿 → 路表坐标系（录制起点车体系） */
    y0 = h->origin_yaw * PB_D2R;
    dx = o.x - h->origin_x;
    dy = o.y - h->origin_y;
    px =  dx * cosf(y0) + dy * sinf(y0);
    py = -dx * sinf(y0) + dy * cosf(y0);

    /* 起跑做一次**全局**最近点搜索（之后只用单调窗口） */
    for (i = 0u; i < n; i ++)
    {
        float ex = px - p[i].x;
        float ey = py - p[i].y;
        float d  = ex * ex + ey * ey;

        if (d < best_d) { best_d = d; best = i; }
    }

    s_idx = best;
    s_n   = n;
    s_elat = 0.0f;

    pb_build_profile(n);

    /* 刻意不用 %f：IAR 默认库不一定带浮点格式化（主程序里也是同一条纪律） */
    printf("[PB] 起跑: 槽 %u, %u 点, 最近点 #%u (距 %u mm), 巡航 %u cm/s, 打角 %u 点\r\n",
           (unsigned)ins_route_slot_active(), (unsigned)n, (unsigned)best,
           (unsigned)(sqrtf(best_d) * 1000.0f), (unsigned)(pb_vmax * 100.0f),
           (unsigned)ins_route_steer_count());

    if ((pb_ff_en >= 0.5f) && (0u == ins_route_steer_count()))
    {
        printf("[PB] !! pb_ff_en=1 但本表没有打角列（flags.bit0 全 0）⇒ 前馈不起作用\r\n");
    }

    return 1u;
}

static void pb_stop (const char *why)
{
    if (0u != s_on)
    {
        steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());
        if (pb_mode >= 1.5f) { spd_target_l = 0.0f; spd_target_r = 0.0f; }
        printf("[PB] 退出回放（%s）\r\n", why);
    }
    s_on   = 0u;
    s_idx  = 0u;
    s_n    = 0u;
    s_elat = 0.0f;
}

/* 三、主循环 */
void route_pb_init (void)
{
    s_on   = 0u;
    s_idx  = 0u;
    s_n    = 0u;
    s_elat = 0.0f;
    pb_en  = 0.0f;
    /* （main_cm7_0.c 里 486 → 492），清掉就等于把 Flash 里存的参数抹了。 */
}

void route_pb_poll (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    float  y0, dx, dy, px, py, pyaw;
    float  ld, vabs, acc;
    uint16 i, i0, i1, j, best;
    float  best_d;
    float  tx, ty, tyaw, elat;
    float  dxr, dyr, pr, bx, by, alpha, kappa, delta;

    /* ---- 1) 安全门 ---- */
    if (pb_en < 0.5f)
    {
        if (0u != s_on) { pb_stop("pb_en=0"); }
        return;
    }
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_FAILSAFE == RC_Engage) ||
        (RC_ENGAGE_ESTOP == RC_Engage))
    {
        if (0u != s_on) { pb_stop("遥控接管/失效"); }
        return;
    }
    if (ins_odom_enable < 0.5f)
    {
        if (0u != s_on) { pb_stop("航位推算关闭"); }
        return;
    }

    if (0u == s_on)
    {
        if (0u == pb_start()) { pb_en = 0.0f; return; }   /* 起不来就把开关退回 0 */
        s_on = 1u;
    }

    /* ---- 2) 当前位姿（路表系）---- */
    p = ins_route_get_points();
    h = ins_route_get_head();
    ins_odom_snapshot(&o);

    y0 = h->origin_yaw * PB_D2R;
    dx = o.x - h->origin_x;
    dy = o.y - h->origin_y;
    px   =  dx * cosf(y0) + dy * sinf(y0);
    py   = -dx * sinf(y0) + dy * cosf(y0);
    pyaw = o.yaw - h->origin_yaw;

    /* ---- 3) 最近点：只在 [idx-5, idx+40] 里找 ⇒ **进度单调不后退** ---- 两条靠得很近的边上会**跳边**，表现是方向盘突然一把反打。 */
    best   = s_idx;
    best_d = 1e9f;
    i0 = (s_idx > 5u) ? (uint16)(s_idx - 5u) : 0u;
    i1 = (uint16)(s_idx + 40u);
    if (i1 >= s_n) { i1 = (uint16)(s_n - 1u); }

    for (i = i0; i <= i1; i ++)
    {
        float ex = px - p[i].x;
        float ey = py - p[i].y;
        float d  = ex * ex + ey * ey;

        if (d < best_d) { best_d = d; best = i; }
    }
    s_idx = best;

    /* ---- 4) 横向误差（对着该点的切线投影；正 = 车在参考线右侧）---- */
    tx   = p[best].x;
    ty   = p[best].y;
    tyaw = p[best].yaw * PB_D2R;
    elat = (px - tx) * cosf(tyaw) + (py - ty) * sinf(tyaw);
    s_elat = elat;

    /* ---- 5) 前视距离 ----
     * ld = clamp(a + b·|v|, min, max)。用**实测车速**（不是目标）：
     * 打滑/被推着走时前视才不会缩得太短（缩太短 = 追点追出振荡）。
     */
    vabs = fabsf(o.v_mps);
    ld   = pb_ld_a + pb_ld_b * vabs;
    ld   = pb_clampf(ld, pb_ld_min, pb_ld_max);
    if (ld < 0.05f) { ld = 0.05f; }

    /* ---- 6) 沿弧长找前视点 ---- */
    j = best;
    acc = 0.0f;
    while (j + 1u < s_n)
    {
        acc += pb_dist(&p[j], &p[j + 1u]);
        j ++;
        if (acc >= ld) { break; }
    }

    if (j >= (uint16)(s_n - 3u))
    {
        pb_stop("到表尾");
        pb_en = 0.0f;
        return;
    }

    /* ---- 7) 前视点 → 车体系 → 曲率 → 转角 ---- x 向右 / y 向前；正的 α（目标在右侧）=> 右转 => steer_delta > 0。 */
    dxr = p[j].x - px;
    dyr = p[j].y - py;
    pr  = pyaw * PB_D2R;
    bx  =  dxr * cosf(pr) + dyr * sinf(pr);
    by  = -dxr * sinf(pr) + dyr * cosf(pr);

    alpha = atan2f(bx, by);                 /* rad，正 = 右 */
    kappa = 2.0f * sinf(alpha) / ld;        /* 1/m，正 = 右 */
    delta = atanf(pb_l * kappa) * PB_R2D;

    /* δ = w · steer[best] + (1 − w) · δ_pp        w = pb_ff_gain ∈ [0,1]
     * 曲率前馈和横向反馈两部分，再叠一个 steer[] 就是**双倍打角**（表现为弯里切内、
     * 直道来回摆）。融合把两个估计器当成同一个量的两种观测，w 就是"更信人还是更信
     * 航向 P 纠偏（它的实测横向偏 0.5~1 m，正是"没有位置反馈"的代价）。
     */
    if ((pb_ff_en >= 0.5f) && (0u != (p[best].flags & (uint16)INS_WP_FL_STEER)))
    {
        float w  = pb_clampf(pb_ff_gain, 0.0f, 1.0f);
        float st = (float)p[best].steer_cdeg / 100.0f;

        delta = w * st + (1.0f - w) * delta;
    }

    steer_ctrl_set_target_delta(delta);

    /* ---- 8) 速度（mode 2 才给）---- 用**前视点**的限速：它天然带一点预瞄，进弯前就开始减。 */
    if (pb_mode >= 1.5f)
    {
        float vt  = s_vlim[j];
        float mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
        float vl, vr;

        /* 到表尾前 1 m 主动收车（不然靠"到表尾"那一拍突然断，会冲出去）
         * 这个函数每圈都跑，逐点 sqrt 到 2040 点会吃掉主循环几百微秒，
         * 把 spd_safety 的 200 ms 心跳门顶穿。点距本来就近等距，误差 < 2%。
         */
        {
            float left = (float)(s_n - 1u - j) * ins_route_ds;
            if (left < 1.0f) { vt *= (left / 1.0f); }
        }

        /* 转弯时内外轮弧长不同（满舵 ±23%）。不差速 = 内轮被拖着走 ⇒ 编码器里程虚高
         * （实测 +22%）⇒ 位姿/航向整体偏。分配律见 spd_ctrl.h：
         * v_l = v(1 − κW/2)   v_r = v(1 + κW/2)   κ = tan(δ_meas)/L
         */
        spd_diff_alloc(vt, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

        spd_target_l = vl / mpr;
        spd_target_r = vr / mpr;
    }
}

float route_pb_elat   (void) { return s_elat; }
uint8 route_pb_active (void) { return s_on;   }
