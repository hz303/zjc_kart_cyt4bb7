/*********************************************************************************************************************
 * 工程 11（卡丁）· 路表回放 Pure Pursuit 外环（结构见 route_pb.h）
 *
 * 分段与 Polaris `kart_playback.c` 一一对应：
 *   一、工具   二、速度剖面   三、最近点（单调）   四、起/停   五、主循环
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>

#include "ins_odom.h"
#include "ins_route.h"
#include "remote_ctrl.h"
#include "spd_ctrl.h"
#include "spd_safety.h"         /* spd_safety_estop()：急停必须中止回放 */
#include "steer_ctrl.h"
#include "route_pb.h"
#include "head_test.h"
#include "proto.h"              /* ★ 驱动令牌：回放是"启动它的那个来源"的子模式 */

#define PB_D2R      (0.01745329252f)
#define PB_R2D      (57.29577951f)

/*===================================================================================================================
 *  参数（ID 43~55 原有；64~68 工程 11 新增）
 *=================================================================================================================*/
volatile float pb_en      = 0.0f;
volatile float pb_mode    = 1.0f;       /* 默认"只给转向"：最安全的验收模式 */
volatile float pb_vmax    = 1.50f;      /* m/s  */
volatile float pb_alat    = 1.20f;      /* m/s^2 */
volatile float pb_abrk    = 0.80f;      /* m/s^2 */
volatile float pb_l       = PB_L_DEFAULT;
volatile float pb_ld_a    = 0.35f;
volatile float pb_ld_b    = 0.20f;
volatile float pb_ld_min  = 0.30f;
volatile float pb_ld_max  = 1.50f;
volatile float pb_ff_en   = 0.0f;
volatile float pb_ff_gain = 1.00f;

volatile float pb_vmin    = 0.20f;              /* 64 ★ 最低速度，防爬行 */
volatile float pb_kwin    = PB_KWIN_DEFAULT;    /* 65 ★ 曲率中心差分半窗（点）*/
volatile float pb_smode   = 0.0f;               /* 66 ★ 0 = 航向外环（Polaris 通道）*/
volatile float pb_kh      = 0.0f;               /* 67 ★ 航向环增益；0 = 自动 2L/ld */
volatile float pb_fin     = 0.15f;              /* 68 ★ 终点判定半径 m */

/*===================================================================================================================
 *  状态
 *=================================================================================================================*/
static uint8   s_st    = PB_ST_IDLE;
static uint8   s_arm   = 0u;            /* pb_en 的上一拍，用来抓 0→1 上升沿 */
static uint16  s_idx   = 0u;            /* 最近点（单调，不后退）*/
static uint16  s_aim   = 0u;            /* 前视点 */
static uint16  s_n     = 0u;            /* 开跑时快照的点数 */
static float   s_elat  = 0.0f;
static float   s_ld    = 0.0f;
static float   s_tv    = 0.0f;
static float   s_ytgt  = 0.0f;
static uint8   s_head  = 0u;            /* 1 = 航向外环正处于打开状态 */
static uint8   s_drv   = PROTO_SRC_NONE;/* ★ 启动本轮回放的驱动令牌持有者（菜单/上位机）
                                           —— 抢占检测用它，别用"当前 owner"去续租：
                                              遥控抢走后若继续 renew，回放会跟着遥控跑。 */
static float   s_vlim[INS_ROUTE_MAX_POINTS];    /* 每点限速 m/s（2048 × 4 B = 8 KB）*/

/*===================================================================================================================
 *  一、工具
 *=================================================================================================================*/
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

/*===================================================================================================================
 *  二、速度剖面（曲率限速 + 反向刹车 + 最低速度地板 + 末段收油）
 *
 *  与 Polaris `kart_playback_build_profile` 同构，按我们的表改两处：
 *    · 窗口点数走参数 `pb_kwin`（它写死 PLAYBACK_KAPPA_WIN = 8）
 *    · 末尾 PB_END_SLOW_M 内线性收油（它靠 mission 的完成判据 + 主动刹停）
 *=================================================================================================================*/
static void pb_build_profile (uint16 n)
{
    const ins_wp_t *p = ins_route_get_points();
    float alat = (pb_alat > 0.05f) ? pb_alat : 0.05f;
    float abrk = (pb_abrk > 0.05f) ? pb_abrk : 0.05f;
    float vmax = (pb_vmax > 0.05f) ? pb_vmax : 0.05f;
    float vmin = (pb_vmin > 0.00f) ? pb_vmin : 0.00f;
    int   win  = (int)pb_kwin;
    uint16 i;

    if (win < 1)  { win = 1; }
    if (win > 20) { win = 20; }
    if (vmin > vmax) { vmin = vmax; }

    /* ---- ① 曲率限速 ---- */
    for (i = 0u; i < n; i ++)
    {
        uint16 i0 = ((int)i - win > 0) ? (uint16)((int)i - win) : 0u;
        uint16 i1 = ((int)i + win < (int)n - 1) ? (uint16)((int)i + win) : (uint16)(n - 1u);
        float  arc = pb_dist(&p[i0], &p[i1]);
        float  dyaw;
        float  kap;
        float  v;

        /* 曲率 = 航向差 / 弧长，**中心差分**。
           ⚠ 窗口不能太小：航向量化 ÷ 短弧长 = 大曲率噪声 ⇒ 直道上冒"假弯"、
             速度被莫名其妙压下去（Polaris 写 8 点就是这个原因）。 */
        dyaw = pb_wrap180(p[i1].yaw - p[i0].yaw) * PB_D2R;
        kap  = (arc > 1e-3f) ? fabsf(dyaw) / arc : 0.0f;

        v = (kap > 1e-4f) ? sqrtf(alat / kap) : vmax;
        if (v > vmax) { v = vmax; }
        if (v < vmin) { v = vmin; }
        s_vlim[i] = v;
    }

    /* ---- ② 反向传刹车：弯心的低速往前传 ⇒ 入弯前就减到位 ---- */
    for (i = (uint16)(n - 1u); i > 0u; i --)
    {
        float ds  = pb_dist(&p[i - 1u], &p[i]);
        float lim = sqrtf(s_vlim[i] * s_vlim[i] + 2.0f * abrk * ds);

        if (s_vlim[i - 1u] > lim) { s_vlim[i - 1u] = lim; }
    }

    /* ---- ③ 反向传完再抬一次地板（上一步会把直道端点压到 vmin 以下）---- */
    for (i = 0u; i < n; i ++)
    {
        if (s_vlim[i] < vmin) { s_vlim[i] = vmin; }
    }

    /* ---- ④ 末段收油：最后 PB_END_SLOW_M 米线性降到 0 ----
       ⚠ 必须在**差速分配之前**把 v 缩掉，否则两侧同比例缩放、平均速度不降。 */
    {
        const ins_wp_t *q = ins_route_get_points();
        float back = 0.0f;
        uint16 k;

        for (k = (uint16)(n - 1u); k > 0u; k --)
        {
            if (back < PB_END_SLOW_M)
            {
                /* ★ 收到 **vmin**，不是 0。
                   ⚠ 收到 0 会死锁：车停在终点圈外 0.2 m、速度目标恒 0 ⇒ 永远进不了
                     `pb_fin` 判据 ⇒ `pb_en` 一直是 1，车趴在原地不动、也不报错。 */
                float r = 1.0f - (back / PB_END_SLOW_M);

                s_vlim[k] = vmin + (s_vlim[k] - vmin) * r;
            }
            back += pb_dist(&q[k - 1u], &q[k]);
        }
    }
}

/*===================================================================================================================
 *  三、最近点（单调前进，不后退）
 *
 *  ⚠ 与"全局最近点"的关键差别：全局搜索在两条靠得很近的边上会**跳边**，
 *    表现是方向盘突然一把反打。Polaris 用 `[progress, progress+40]`，**只向前** ——
 *    首圈起跑已经做过一次全局定位，之后不该倒退。工程 10 额外允许回看 5 点，
 *    那是为了"倒车/被推回"的场合；本工程不需要，去掉。
 *=================================================================================================================*/
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

/* 沿弧长找前视点（Polaris `kart_playback_find_lookahead` 同构） */
static uint16 pb_find_lookahead (uint16 nearest, const ins_wp_t *p, float ld)
{
    uint16 i;
    float  acc = 0.0f;

    for (i = nearest; (uint16)(i + 1u) < s_n; i ++)
    {
        acc += pb_dist(&p[i], &p[i + 1u]);
        if (acc >= ld) { return (uint16)(i + 1u); }
    }
    return (uint16)(s_n - 1u);
}

/*===================================================================================================================
 *  四、起 / 停
 *=================================================================================================================*/
/* 最近一次"为什么没跑 / 为什么结束"。屏幕要能看见 ——
   否则现场只有"按了 PUSH 没反应"，串口又没接，等于没有任何线索。 */
static char s_why[24] = "";

static void pb_set_why (const char *w)
{
    if (0 == w) { s_why[0] = '\0'; return; }
    snprintf(s_why, sizeof(s_why), "%s", w);
}

const char * route_pb_reason (void) { return s_why; }

static void pb_release_steer (void)
{
    /* 交还控制：航向外环先关，再把转角目标**对齐实测角** ——
       否则退出那一刻内环会把方向盘甩一下（工程 10 踩过）。 */
    if (0u != s_head)
    {
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
    }
    steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());
}

static void pb_finish (uint8 st, const char *why)
{
    if (PB_ST_RUN == s_st)
    {
        /* ★ 交还驱动权：回放结束（跑完/中止/被抢占）就不该继续占着方向盘，
           否则上位机/菜单要等到租约到期才能接管。 */
        if (PROTO_SRC_NONE != s_drv)
        {
            proto_drv_release(s_drv);
            s_drv = PROTO_SRC_NONE;
        }

        pb_release_steer();
        spd_ctrl_set_coast(0u);         /* ★ 退出时务必撤自由轮，否则遥控/跑班也推不动 */
        if (pb_mode >= 1.5f)
        {
            spd_target_l = 0.0f;
            spd_target_r = 0.0f;
        }
        printf("[PB] 结束（%s）state=%u idx=%u/%u\r\n",
               why, (unsigned)st, (unsigned)s_idx, (unsigned)s_n);
        pb_set_why((0 != why) ? why : "stop");
    }
    s_st = st;
}

static uint8 pb_start (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    float y0, dx, dy, px, py;
    float best_d = 1e9f;
    uint16 n, i, best = 0u;

    n = ins_route_get_count();
    if (n < (uint16)PB_MIN_POINTS)  { printf("[PB] 表太短（%u 点）\r\n", (unsigned)n); pb_set_why("no table"); return 0u; }
    if (ins_odom_enable < 0.5f)     { printf("[PB] 航位推算没开\r\n");                 pb_set_why("odom off"); return 0u; }

    p = ins_route_get_points();
    h = ins_route_get_head();

    ins_odom_snapshot(&o);

    /* 车当前位姿 → 路表坐标系（录制起点车体系）*/
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

    s_idx  = best;
    s_n    = n;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_tv   = 0.0f;
    s_aim  = best;

    pb_build_profile(n);

    /* 刻意不用 %f：IAR 默认库不一定带浮点格式化 */
    printf("[PB] 起跑: 槽 %u, %u 点, 最近点 #%u (距 %u mm), 巡航 %u cm/s, 打角 %u 点\r\n",
           (unsigned)ins_route_slot_active(), (unsigned)n, (unsigned)best,
           (unsigned)(sqrtf(best_d) * 1000.0f), (unsigned)(pb_vmax * 100.0f),
           (unsigned)ins_route_steer_count());

    if ((pb_ff_en >= 0.5f) && (0u == ins_route_steer_count()))
    {
        printf("[PB] !! pb_ff_en=1 但本表没有打角列 ⇒ 前馈不起作用\r\n");
    }

    /* ★ 记下"谁授权了这次回放"：pb_en 是 P_CMD，写它的那个来源此刻就是令牌持有者。
       回放全程占着这个令牌（每拍续租），跑完在 pb_finish 里释放。 */
    s_drv = proto_drv_owner();
    if (PROTO_SRC_NONE == s_drv)
    {
        printf("[PB] 没有驱动令牌（pb_en 不是经协议层写的？）\r\n");
        return 0u;
    }

    return 1u;
}

/*===================================================================================================================
 *  五、主循环
 *=================================================================================================================*/
void route_pb_init (void)
{
    s_st   = PB_ST_IDLE;
    s_arm  = 0u;
    s_idx  = 0u;
    s_n    = 0u;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_tv   = 0.0f;
    s_head = 0u;
    s_drv  = PROTO_SRC_NONE;
    pb_en  = 0.0f;
    /* ⚠ 不要在这里清 pb_ff_en / pb_ff_gain：本函数在 app_param_init() **之后**调，
       清掉就等于把 Flash 里存的参数抹了。 */
}

void route_pb_poll (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    float  y0, dx, dy, px, py, pyaw;
    float  vabs, ld, elat;
    float  tx, ty, tyaw;
    float  dxr, dyr, pr, bx, by, alpha;
    uint16 j, best;

    /* ---- 1) 安全门 ---- */
    if (head_test_active())
    {
        /* 航向测试（App 的测试页）优先：它也在写 target_yaw / spd_target_*，
           两边同时跑就是"两个源抢同一个执行器"。让位方必须是回放 —— 测试是人手按下的短时实验。 */
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "航向测试占用"); }
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
    /* NOTE 2026-10-20: FAILSAFE only blocks PWR (pb_mode>=1.5).
       With no receiver plugged, RC_Engage is ALWAYS FAILSAFE, so the old gate
       blocked PUSH as well -> the servo never followed AND the speed loop held
       target=0 (car could not be pushed), with no error shown anywhere.
       PUSH drives no motor (a human pushes), so it must not require a live RC.
       ON = someone is actually driving by RC -> yield; ESTOP = always highest. */
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_ESTOP == RC_Engage) ||
        ((RC_ENGAGE_FAILSAFE == RC_Engage) && (pb_mode >= 1.5f)))
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "RC ON/ESTOP"); }
        if (RC_ENGAGE_ESTOP == RC_Engage) { pb_set_why("RC ESTOP"); }
        else if (RC_ENGAGE_ON == RC_Engage) { pb_set_why("RC ON"); }
        s_arm = 0u;
        return;
    }
    /* ★ 急停（遥控 CH5 或上位机 0x09）：**中止**回放，不是"暂停"。
       否则急停期间本模块还在往 spd_target_* 写速度，解除急停那一拍车会直接窜出去。 */
    if (0u != spd_safety_estop())
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_ABORT, "急停"); pb_en = 0.0f; }
        s_arm = 0u;
        return;
    }
    if (ins_odom_enable < 0.5f)
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "航位推算关闭"); }
        s_arm = 0u;
        return;
    }

    /* ---- 1b) 只在 pb_en 的 **0→1 上升沿**启动 ----
       ⚠ 用状态位判断"要不要起跑"会有一个坑：跑到终点后状态是 DONE、`pb_en` 还是 1，
         下一拍会**原地重新起跑**。所以必须抓沿。 */
    if (0u == s_arm)
    {
        s_arm = 1u;
        if (0u == pb_start())
        {
            /* NOTE 2026-10-20: do NOT clear pb_en here.
               Clearing it erased "the user asked for PUSH", so the free-wheel
               sync released the coast flag and the speed loop locked the car at
               target=0 while the screen showed OFF.
               Keep pb_en=1 (intent), state=ABORT + reason on screen; the car stays
               pushable. s_arm=2 latches the retry until Follow goes OFF then on,
               or the user presses K1 again (menu calls route_pb_rearm()). */
            s_arm = 2u;
            s_st  = PB_ST_ABORT;
            return;
        }
        s_st = PB_ST_RUN;
        pb_set_why("");
    }
    if (PB_ST_RUN != s_st) { return; }

    /* ---- 1c) 驱动令牌（2026-10-20）----
       · 每拍续租：回放是"启动它的那个来源"的子模式（笔记 §7.3 的 REPLAY）；
       · 一旦 owner 变了（遥控 CH6 合闸抢占 / 上位机断链后菜单接管），**立刻让位收车**。
         不做这一步的话，遥控把车抢过去了、回放还在 5ms 里往 spd_target_* 写速度，
         两边打架，而人看不出是谁在下命令。 */
    if (proto_drv_owner() != s_drv)
    {
        pb_finish(PB_ST_ABORT, "驱动权被抢占");
        pb_en = 0.0f;                       /* 释放命令：否则下一拍会"原地重新起跑" */
        s_arm = 0u;
        return;
    }
    proto_drv_renew(s_drv);

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

    /* ---- 3) 最近点（单调）---- */
    best  = pb_find_nearest(px, py, p);
    s_idx = best;

    /* ---- 4) 完成 / 中止（Polaris 的两条判据；工程 10 只有"到表尾"）----
       · 进终点圈 ⇒ COMPLETED（正常收车）
       · 索引到末点、但离末点还远 ⇒ ABORTED（脱线/跳点，**不是**跑完了）
       分开报的理由：现场要能一眼区分"跑完"和"跑飞"，工程 10 里两者都长成
       "到表尾 + pb_en 归 0"，追责时看不出来。 */
    {
        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;
        float ld2 = sqrtf(lx * lx + ly * ly);

        if (ld2 < ((pb_fin > 0.02f) ? pb_fin : 0.02f))
        {
            pb_finish(PB_ST_DONE, "到终点圈");
            pb_en = 0.0f;
            return;
        }
        if (best >= (uint16)(s_n - 1u))
        {
            pb_finish(PB_ST_ABORT, "索引到末点但离终点很远");
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

    /* ---- 6) 目标速度：取**最近点**（Polaris："速度跟当前弧段，弯道不提前拉满"）
       ⚠ 工程 10 用的是前视点的限速 —— 那等于在直道末段就套上弯里的低速，
         长直道会一直偏慢。这里改回最近点。 */
    {
        float v = s_vlim[best];
        float mpr;

        if (v >  pb_vmax) { v =  pb_vmax; }
        if (v <  0.0f)    { v =  0.0f;    }
        s_tv = v;

        if (pb_mode < 1.5f)
        {
            /* ★ 6.1 无动力跟随（人推车）：pwm 不给力。
               为什么每拍都写 0、而不是只在起跑写一次：菜单 6.Follow 可以在跑的中途切模式，
               只在沿上写会让"切过来"这一路仍带着上一次的目标速度 —— 人推车时车自己窜出去。
               ⚠ 用 pb_mode 而不是 pb_en 判断：pb_en=1 但 mode=1 就是"只给舵机"。
               ⚠⚠ 光把目标写 0 **不等于不给力**：闭环看到 v>0、target=0 会当成超速去
                  反向下力刹车，人根本推不动（现场实测）。必须走 `spd_ctrl_set_coast(1)`：
                  输出钉 0（= 自由轮）、PID 不跑，但**测速/里程照跑**（推车时位姿还要用）。 */
            spd_target_l = 0.0f;
            spd_target_r = 0.0f;
            spd_ctrl_set_coast(1u);
        }
        else
        {
            float vl, vr;

            spd_ctrl_set_coast(0u);         /* 带速度跟随：先撤自由轮，再给目标 */
            mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;

            /* ---- 6b) 电子差速（2026-10-17）----
               转弯时内外轮弧长不同（满舵 ±23%）。不差速 = 内轮被拖着走 ⇒ 里程虚高 +22%
               ⇒ 位姿/航向整体偏。分配律见 spd_ctrl.h：
                   v_l = v(1 − κW/2)   v_r = v(1 + κW/2)   κ = tan(δ_meas)/L
               ⚠ 用**实测**舵角（目标有滞后，用目标会让后轮提前差速、车尾先被推出去）。
               ⚠ 平均速度恒等于 v ⇒ ins_odom 的 (dl+dr)/2 不受影响，剖面限速仍然成立。 */
            spd_diff_alloc(v, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

            spd_target_l = vl / mpr;
            spd_target_r = vr / mpr;
        }
    }

    /* ---- 7) 前视距离（Polaris 用**目标**速度，工程 10 用实测速度）----
       用目标速度的理由：剖面已经按几何算好了"这里该多快"，前视跟着它走是确定的；
       跟实测走会被打滑/被推着走带偏（工程 10 的注释里也承认这一点，但选了实测）。
       ⚠ 本工程按 Polaris：目标速度。跑起来若发现"打滑时前视缩得太短"，
         把 pb_ld_b 调小、pb_ld_min 调大即可。 */
    vabs = fabsf(s_tv);
    ld   = pb_ld_a + pb_ld_b * vabs;
    ld   = pb_clampf(ld, pb_ld_min, pb_ld_max);
    if (ld < 0.05f) { ld = 0.05f; }
    s_ld = ld;

    /* ---- 8) 沿弧长找前视点 ---- */
    s_aim = pb_find_lookahead(best, p, ld);
    j     = s_aim;

    /* ---- 9) 转向：两条通道（数学等价，留两条做 A/B）----
       车体系：x 向右 / y 向前；alpha 正 = 目标点在右；
       `steer_delta > 0` = 右转（已实测），`yaw` 正 = 左转。
       ⇒ 前视点在世界系的**目标航向** = 当前航向 − alpha。 */
    dxr = p[j].x - px;
    dyr = p[j].y - py;
    pr  = pyaw * PB_D2R;
    bx  =  dxr * cosf(pr) + dyr * sinf(pr);
    by  = -dxr * sinf(pr) + dyr * cosf(pr);
    alpha = atan2f(bx, by);                 /* rad，正 = 右 */

    if (pb_smode < 0.5f)
    {
        /* ---- 9a) 航向外环（Polaris 通道）----
           δ = k·(yaw_tgt − yaw)，k = 2L/ld 时与 Pure Pursuit 等价（推导见 route_pb.h）。
           ⚠ 增益必须**按 ld 现算**，不能像工程 10 那个 head_kp=15 手填：
             15 与 ld 无关 ⇒ 前视一变就成了另一个环，谁开谁振。 */
        float k = (pb_kh > 1.0e-3f) ? pb_kh : (2.0f * pb_l / ld);

        k = pb_clampf(k, 0.3f, 8.0f);
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D);

        steer_ctrl_set_head_kp(k);
        if (0u == s_head) { steer_ctrl_set_head_enable(1u); s_head = 1u; }
        steer_ctrl_set_target_yaw(s_ytgt);

        /* ⚠ 航向通道**不用**打角前馈（pb_ff_en 在这里无效）。
           理由：前馈给的是 δ，航向通道的入口是 yaw，要融合就得先把 δ 反解成 yaw，
           等于绕一圈回到 9b。想用手工表的打角，把 pb_smode 拨到 1。 */
    }
    else
    {
        /* ---- 9b) 直接给 δ（工程 10 通道）----
           κ = 2·sin(α)/ld，δ = atan(L·κ)。参数少、相位短，但没有"目标航向"可看。 */
        float kappa = 2.0f * sinf(alpha) / ld;
        float delta = atanf(pb_l * kappa) * PB_R2D;

        if (0u != s_head) { steer_ctrl_set_head_enable(0u); s_head = 0u; }
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D);

        /* 打角前馈（手工/人开车方案）：δ = w·steer[best] + (1−w)·δ_pp。
           为什么是融合而不是"前馈 + PP 修正"：PP 的 δ 本身已含曲率前馈和横向反馈，
           再叠一个 steer[] 就是**双倍打角**（弯里切内、直道来回摆）。 */
        if ((pb_ff_en >= 0.5f) && (0u != (p[best].flags & (uint16)INS_WP_FL_STEER)))
        {
            float w  = pb_clampf(pb_ff_gain, 0.0f, 1.0f);
            float st = (float)p[best].steer_cdeg / 100.0f;

            delta = w * st + (1.0f - w) * delta;
        }
        steer_ctrl_set_target_delta(delta);
    }
}

/*===================================================================================================================
 *  六、读出接口（遥测/调试）
 *=================================================================================================================*/
/* 每 5 ms 由中断调用：只按"用户意图 + 谁在开车"决定要不要自由轮。
   见 route_pb.h 的说明 —— 不能依赖状态机走到 PUSH 分支。 */
void route_pb_sync_coast (void)
{
    uint8 push = 0u;

    if ((pb_en > 0.5f) && (pb_mode < 1.5f))
    {
        /* 遥控真的在开车 / 急停 / 航向测试在跑 -> 不许自由轮，否则它们的速度被吃掉 */
        if ((RC_ENGAGE_ON != RC_Engage) && (RC_ENGAGE_ESTOP != RC_Engage) &&
            (0u == head_test_active()))
        {
            push = 1u;
        }
    }
    spd_ctrl_set_coast(push);
}

/* 起跑失败后允许再试一次：s_arm 会锁在 2，必须 OFF->ON 才重来。
   菜单每次把 Follow 写成 PUSH/PWR 时调它 —— 补完表/开航位后按一下就能再试。 */
void  route_pb_rearm   (void)
{
    s_arm = 0u;
    pb_set_why("");
}

void  route_pb_stop    (const char *why) { pb_finish(PB_ST_IDLE, why); }

float route_pb_elat    (void) { return s_elat; }
uint8 route_pb_active  (void) { return (PB_ST_RUN == s_st) ? 1u : 0u; }
uint8 route_pb_state   (void) { return s_st; }
float route_pb_ld      (void) { return s_ld; }
float route_pb_tv      (void) { return s_tv; }
float route_pb_yaw_tgt (void) { return s_ytgt; }
uint16 route_pb_aim    (void) { return s_aim; }
uint16 route_pb_index  (void) { return s_idx; }
