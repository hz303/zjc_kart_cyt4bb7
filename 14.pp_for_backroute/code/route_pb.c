/* 工程 11（卡丁）· 路表回放 Pure Pursuit 外环（结构见 route_pb.h） 一、工具   二、速度剖面   三、最近点（单调）   四、起/停   五、主循环 */
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
#include "proto.h"
#include "motor_link.h"
#include "route_app.h"

#define PB_D2R      (0.01745329252f)
#define PB_R2D      (57.29577951f)

/* 参数（ID 43~55 原有；64~68 工程 11 新增） */
volatile float pb_en      = 0.0f;
volatile float pb_mode    = 1.0f;       /* 默认"只给转向"：最安全的验收模式 */
volatile float pb_vmax    = 1.50f;
volatile float pb_alat    = 2.00f;      /* m/s^2 */
volatile float pb_abrk    = 0.80f;      /* m/s^2 */
volatile float pb_l       = PB_L_DEFAULT;
volatile float pb_ld_a    = 0.35f;
volatile float pb_ld_b    = 0.20f;
volatile float pb_ld_min  = 0.30f;
volatile float pb_ld_max  = 1.50f;
volatile float pb_ff_en   = 0.0f;
volatile float pb_ff_gain = 1.00f;

volatile float pb_vmin    = 0.20f;
volatile float pb_kwin    = PB_KWIN_DEFAULT;
volatile float pb_smode   = 0.0f;
volatile float pb_kh      = 0.0f;
volatile float pb_fin     = 0.15f;

static uint8   s_st    = PB_ST_IDLE;
static uint8   s_arm   = 0u;            /* pb_en 的上一拍，用来抓 0→1 上升沿 */
static uint8   s_done_armed = 0u;       /* 1 = 已走过完成阈值点，才允许判"到终点" */
static uint16  s_idx   = 0u;            /* 最近点（单调，不后退） */
static uint16  s_aim   = 0u;
static uint16  s_n     = 0u;            /* 开跑时快照的点数 */
static float   s_elat  = 0.0f;
static float   s_ld    = 0.0f;
static float   s_tv    = 0.0f;
static float   s_ytgt  = 0.0f;
static uint8   s_head  = 0u;            /* 1 = 航向外环正处于打开状态 */
static uint8   s_rev_flipped = 0u;
static uint8   s_drv   = PROTO_SRC_NONE;/* —— 抢占检测用它，别用"当前 owner"去续租： 遥控抢走后若继续 renew，回放会跟着遥控跑。 */
static float   s_vlim[INS_ROUTE_MAX_POINTS];    /* 每点限速 m/s（2048 × 4 B = 8 KB） */

/* 之后所有位姿都用"相对它"的增量算：px = p[0] + R(-s_o0yaw)·(o − o0)。 只拿来打一行诊断。这样"烧完再录、录完再烧、中间把车挪走"都不影响表。 */
static float   s_o0x = 0.0f, s_o0y = 0.0f, s_o0yaw = 0.0f;
/* 而"改 Follow"这条路根本不会发 SET_ENABLE ⇒ SAFE_BIT_NOT_EN 一直在 ⇒
 * spd_target 写得再对，电机也不出力 —— 现场就是"PWR 电机根本不转"。
 * 遥控 ON 时是遥控自己 set_local(1)，回放这条路上没人做这件事。
 */
static uint8   s_local = 0u;
/* 倒车时不能开航向外环：Pure Pursuit 的负反馈是对车头的，对车尾是正反馈 ⇒ 打圈。 */
static uint8   s_rev = 0u;
/* 急停连续计数（去抖用，见 poll 里的急停门） */
static uint16  s_estop_ticks = 0u;

/* 而是把"命中那一拍的左右目标"快照下来，用 PB_STOP_MS 毫秒线性降到 0。 0 = 没有在斜坡中。单位同 spd_target_*（rps）。 */
static uint8   s_ramp    = 0u;    /* 1 = 正在走停车斜坡 */
static uint32  s_ramp_t0 = 0u;    /* 斜坡起始时刻 ms（motor_link_now_ms） */
static float   s_ramp_l  = 0.0f;
static float   s_ramp_r  = 0.0f;

/* 满舵 + elat 不小 + 最近点索引冻结 ⇒ 车在绕着前视点画圆，不是在跟线。 */
static uint32  s_lost_t0  = 0u;   /* 计时起点 ms；0 = 没在计时 */
static uint16  s_lost_idx = 0u;   /* 计时起点的最近点索引 */
static float   s_alpha   = 0.0f;

/* 一、工具 */
static float pb_wrap180 (float a)
{
    /* （上游任何一个量炸了：未初始化的 target_yaw、除零、NaN 之后的溢出…），
     * `-= 360` 永远退不出来 => 这个函数所在的 1 kHz ISR / 主循环就地卡死 =>
     * 菜单冻结、按键无效、WiFi·SPI 停发（TX 灯灭 / RX 被从机拉常亮）、电机不动。
     * 加次数上限 + NaN 保护：病态输入只会被夹到 +-180，绝不会再卡住。
     */
    uint8 guard = 0u;

    if (!(a == a))                                     { return 0.0f; }
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

/* 二、速度剖面（曲率限速 + 反向刹车 + 最低速度地板 + 末段收油）
 * · 窗口点数走参数 `pb_kwin`（它写死 PLAYBACK_KAPPA_WIN = 8）
 * · 末尾由"末点锚 PB_END_CREEP + 反向刹车"生成减速曲线（它靠 mission 的完成判据刹停）
 */
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

    /* ---- ① 曲率限速（倒车段不参与）----
     * a) 窗口里的"弧长"要**逐段累加弦长**，不能用两端点直线距离：弯道上弦长 < 弧长，
     * 会把 κ 算大、把弯里速度压得过低；
     * b) `arc ≈ 0` 有两种成因、处理相反：原地打方向 ⇒ 最急，给 vmin；
     * 原地没动也没转（静止重复采样）⇒ 曲率无信息，给 vmax。
     * 以前一律当 κ=0 给 vmax，等于"原地满舵再全油门"；
     * c) 录制速度 < 0 的点是**倒车段**：几何限速（横向抓地）对倒车没意义，
     */
    for (i = 0u; i < n; i ++)
    {
        float  rec = p[i].v;
        uint16 i0, i1, k;
        float  arc = 0.0f;
        float  dyaw_abs;
        float  v;

        if (rec < -(float)PB_REV_EPS)
        {
            s_vlim[i] = rec;
            continue;
        }

        i0 = ((int)i - win > 0) ? (uint16)((int)i - win) : 0u;
        i1 = ((int)i + win < (int)n - 1) ? (uint16)((int)i + win) : (uint16)(n - 1u);

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

            v = (kap < 1e-3f) ? vmax : sqrtf(alat / kap);
        }

        if (v > vmax) { v = vmax; }
        if (v < vmin) { v = vmin; }
        s_vlim[i] = v;
    }

    /* 车不可能不经过零速就从前进变倒车。换向点**前**那点留一点速度：慢了冲过头可忽略， 又保证进度点能越过换向点（钉死 0 会卡在那儿，永远换不过去）。 */
    for (i = 1u; i < n; i ++)
    {
        if ((s_vlim[i] < 0.0f) && (s_vlim[i - 1u] > 0.0f)) { s_vlim[i - 1u] = (float)PB_REV_CREEP; }
    }

    /* 只改**最后一个前进点**的速度，剩下的减速曲线由下面 ② 的反向刹车
     * `v[i-1] ≤ sqrt(v[i]² + 2·abrk·ds)` 自动算出来 ——
     * 那正是"以 pb_abrk 刹住所需的最快速度"，所以连续、无台阶、也不会超过 pb_abrk。
     * ⇒ **方向反了**：末点保持全速、末点前 1 m 压到 0.24 rps、1 m 外又跳回 1.7 rps，
     * 在 L−1.0 m 处形成 13.3 m/s² 的减速度尖峰（abrk 的 16 倍）= 现场那"一顿"。
     * 另外 pb_vmin=0.2 m/s 落在电机死区，爬行段会一走一停。
     */
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

    /* ---- ② 反向传刹车：弯心低速往前传 ⇒ 入弯前就减到位 ---- 不跳过的话 lim 会算成负数，把前进段整段压死。 */
    for (i = (uint16)(n - 1u); i > 0u; i --)
    {
        float ds;
        float lim;

        if ((s_vlim[i] < 0.0f) || (s_vlim[i - 1u] < 0.0f)) { continue; }

        ds  = pb_dist(&p[i - 1u], &p[i]);
        lim = sqrtf(s_vlim[i] * s_vlim[i] + 2.0f * abrk * ds);
        if (s_vlim[i - 1u] > lim) { s_vlim[i - 1u] = lim; }
    }

    /* ---- ③ 再抬一次地板（上一步会把直道端点压到 vmin 以下）---- */
    for (i = 0u; i < n; i ++)
    {
        if ((s_vlim[i] >= 0.0f) && (s_vlim[i] < vmin)) { s_vlim[i] = vmin; }
    }

}

/* 三、最近点（单调前进，不后退） 首圈起跑已经做过一次全局定位，之后不该倒退。工程 10 额外允许回看 5 点， 那是为了"倒车/被推回"的场合；本工程不需要，去掉。 */
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

/* 四、起 / 停 */
/* 否则现场只有"按了 PUSH 没反应"，串口又没接，等于没有任何线索。 */
static char s_why[24] = "";

static void pb_set_why (const char *w)
{
    if (0 == w) { s_why[0] = '\0'; return; }
    snprintf(s_why, sizeof(s_why), "%s", w);
}

const char * route_pb_reason (void) { return s_why; }

static void pb_release_steer (void)
{
    /* 交还控制：航向外环先关，再把转角目标**对齐实测角** —— 否则退出那一刻内环会把方向盘甩一下（工程 10 踩过）。 */
    if (0u != s_head)
    {
        steer_ctrl_set_head_enable(0u);
        s_head = 0u;
    }
    /* 原因（现场）：PUSH 下被中止时方向盘正打在 27°，对齐实测角等于**把满舵保持住**，
     * 人一推车就原地绕圈（18:05 那次转了 1.2 圈、yaw -454°）。
     * 回中的代价是"退出那一刻内环走回正"（最多 27° 行程，SLEW 限速下约 0.2 s），
     * 比"锁死在满舵"好得多；RC 接管时遥控本来就会自己写目标，不受影响。
     */
    steer_ctrl_set_target_delta(0.0f);
}

static void pb_finish (uint8 st, const char *why)
{
    /* `if (PB_ST_RUN == s_st)` 里。现场症状：第一次跑完 s_st 已经是 DONE / ABORT，
     * 之后任何一次 pb_finish（关 Follow、急停、被抢占、odom 关掉）都进不了那个 if =>
     * 本地授权、自由轮、停车斜坡、驱动令牌、以及"倒序回放"的点序全留在上一次的状态。
     * 第二次起跑就带着这些残留 => "第一次好好的，第二次开始每次都触发"。
     */
    if (0u != s_rev_flipped)
    {
        (void)ins_route_reverse_points();      /* 连倒两次 = 原序 */
        s_rev_flipped = 0u;
    }

    if (PROTO_SRC_NONE != s_drv)
    {
        proto_drv_release(s_drv);
        s_drv = PROTO_SRC_NONE;
    }

    s_ramp = 0u;                    /* 别把停车斜坡带到下一次起跑 */
    spd_ctrl_set_coast(0u);         /* 退出时务必撤自由轮，否则遥控/跑班也推不动 */

    /* 交还本地授权（只灭自己那一盏，别把遥控刚点的那盏也吹了）。 */
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
}

static uint8 pb_start (void)
{
    const ins_wp_t *p;
    const ins_route_head_t *h;
    ins_odom_snap_t o;
    uint16 n;

    n = ins_route_get_count();
    if (n < (uint16)PB_MIN_POINTS)  { printf("[PB] 表太短（%u 点）\r\n", (unsigned)n); pb_set_why("no table"); return 0u; }
    if (ins_odom_enable < 0.5f)     { printf("[PB] 航位推算没开\r\n");                 pb_set_why("odom off"); return 0u; }

    /* 用途：车**倒着**沿轨迹录一张表之后，把点序倒过来，车**正着**开回起点。
     * 同一点的车身朝向完全相同、需要的前轮转角也完全相同，只有遍历顺序相反。
     * 所以这里按"表里 REV 点过半"判定表的出身，和开关不一致就**不让起跑**，
     * 原因写进 pb_set_why（菜单第 7 项那行会显示）。
     */
    {
        uint16 nrev  = ins_route_rev_count();
        uint8  isrev = ((uint16)(nrev * 2u) >= n) ? 1u : 0u;      /* 过半 ⇒ 倒车录的表 */
        uint8  want  = (rt_revplay >= 0.5f) ? 1u : 0u;

        s_rev_flipped = 0u;

        if (want != isrev)
        {
            printf("[PB] !! 表与回放方向不匹配: RevPlay=%u, 倒车点 %u/%u\r\n",
                   (unsigned)want, (unsigned)nrev, (unsigned)n);
            pb_set_why((0u != want) ? "not rev table" : "need RevPlay");
            return 0u;
        }

        if (0u != want)
        {
            if (0u == ins_route_reverse_points())
            {
                printf("[PB] !! 点序倒置失败（%u 点，正在录？）\r\n", (unsigned)n);
                pb_set_why("rev flip fail");
                return 0u;
            }
            s_rev_flipped = 1u;
            printf("[PB] 倒序回放: %u 点已倒置，从原末点往原起点正车走\r\n", (unsigned)n);
        }
    }

    p = ins_route_get_points();
    h = ins_route_get_head();

    ins_odom_snapshot(&o);

    /* 表的 x/y/yaw 是**相对录制起点**的（车体系），这部分与 odom 无关，永远有效；
     * 会烂掉的是表头那三行 `origin_x/y/yaw` —— 它们是"录制那一刻的 odom"。
     * odom 是**上电以来的积分量**：烧一次录、断一次电就归零；把车抱起来走一段，
     * 轮子不转、odom 也不动。把它当锚点，表就只能在"同一上电周期 + 没人动过车"时用。
     * ⇒ 这里**一个绝对量都不用**：只把"现在的位姿"记成基准 (s_o0*)，
     * 于是当前点 ≡ 表的第 0 点、当前朝向 ≡ p[0].yaw，之后全部用**增量**换算。
     * 语义唯一且可复现：**把车摆到起点、大致朝对方向，再开 Follow。**
     * （录完立刻跑也成立：录制起点本来就是第 0 点，车这时也在那儿。）
     */
    {
        float c0 = cosf(h->origin_yaw * PB_D2R);
        float s0 = sinf(h->origin_yaw * PB_D2R);
        float ex = ( (o.x - h->origin_x) * c0 + (o.y - h->origin_y) * s0 - p[0].x);
        float ey = (-(o.x - h->origin_x) * s0 + (o.y - h->origin_y) * c0 - p[0].y);
        float d0 = sqrtf(ex * ex + ey * ey);
        float dyaw = pb_wrap180(o.yaw - h->origin_yaw - p[0].yaw);

        s_o0x   = o.x;
        s_o0y   = o.y;
        s_o0yaw = o.yaw;

        /* 我先前把这里改成 |dyaw|>60° 就拒绝起跑，理由是"车头偏 90° 会跑一张旋转
         * 90° 的副本"。**这个理由是错的，撤回**：
         * · 表的 x/y/yaw 全是**相对录制起点**的量，起跑锚点又取"车此刻的位姿"，
         * 所以整张表被转 dyaw 是**设计如此、数学上完全自洽**的 —— 车头朝哪边，
         * 表就跟着转到哪边，路径永远在车头前方。dyaw 本身不是误差。
         * 那是**摆车**问题，一刀切拦下来只会误伤：断电、ins_zero、换电池之后
         * 表头 origin 全是脏的，dyaw 会变成纯噪声。
         * · 而且 dyaw 只在"录制之后没 reset 过 odom"时才有物理意义；reset 过就
         * 只能信几何。
         * ⇒ 保留提示，让操作员自己判断；不再拒绝起跑。
         */
        if (fabsf(dyaw) > (float)PB_ALIGN_YAW_MAX)
        {
            printf("[PB] 注意: 起跑朝向与表首差 %d deg（表首朝向 %d deg，"
                   "odom %s）⇒ 整张表会被转 %d deg\r\n",
                   (int)dyaw, (int)p[0].yaw,
                   ((d0 > (float)PB_ALIGN_MAX_M) ? "换过/被挪过" : "同一上电周期"),
                   (int)dyaw);
        }

        printf("[PB] 对齐: START (表头 origin 差 %u cm / %d deg%s)\r\n",
               (unsigned)(d0 * 100.0f), (int)dyaw,
               ((d0 > (float)PB_ALIGN_MAX_M) ? ", odom 换过/车被挪过 ⇒ 已忽略" : ""));

    }

    /* 固定从第 3 点起步：对齐之后车本来就落在 p[0]，全局搜索必然返回 0；而第 0/1 点在
     * 录制时是"静止起步段"（v≈0、位置几乎不动），拿它当进度点会让目标速度贴 0，
     * 点数太少（n<=3）就退回 0，短表靠完成判据收尾。
     * 0.3~0.7 m 里车头还在甩的那几点排到了最前面（221324 末 4 点转 14.9°、
     * 222112 转 10.6°，而正常的 211537 / 221519 只有 1.3° / 1.4°），
     * 「|v| < 0.4 × 本表最大 |v|」去跳，**三张真表一个点都跳不掉**——
     * 收车甩头那几点速度并不低（0.3~0.6 m/s），只是车头角速度快，判据不成立。
     * 判据立不住的改动不该上，撤回；这个坑改由 6b 的前视距离可达性下限兜住。
     */
    s_idx  = (n > 3u) ? 3u : 0u;
    s_rev  = 0u;
    s_estop_ticks = 0u;
    s_n    = n;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_tv   = 0.0f;
    s_lost_t0  = 0u;
    s_lost_idx = 0u;
    s_alpha    = 0.0f;
    s_aim  = s_idx;

    pb_build_profile(n);

    /* 刻意不用 %f：IAR 默认库不一定带浮点格式化 */
    printf("[PB] 起跑: 槽 %u, %u 点, 起步点 #%u, 巡航 %u cm/s, 打角 %u 点\r\n",
           (unsigned)ins_route_slot_active(), (unsigned)n, (unsigned)s_idx,
           (unsigned)(pb_vmax * 100.0f), (unsigned)ins_route_steer_count());

    if ((pb_ff_en >= 0.5f) && (0u == ins_route_steer_count()))
    {
        printf("[PB] !! pb_ff_en=1 但本表没有打角列 ⇒ 前馈不起作用\r\n");
    }

    /* 回放全程占着这个令牌（每拍续租），跑完在 pb_finish 里释放。 */
    s_done_armed = 0u;
    s_ramp       = 0u;

    s_drv = proto_drv_owner();
    if (PROTO_SRC_NONE == s_drv)
    {
        /* ⇒ 屏幕写着 PUSH、车纹丝不动，现场只会当成"按键坏了"。 现在记 INTERNAL 继续跑（PUSH 本来就不给后轮力，风险很小）。 */
        s_drv = PROTO_SRC_INTERNAL;
        printf("[PB] 起跑时没有令牌持有者，按 INTERNAL 记账\r\n");
    }

    /* 否则"need RevPlay" / "start yaw off" 这类原因会一直挂在屏幕上， 车明明在跑、菜单第 7 项却写着上次的错，现场只能瞎猜。 */
    pb_set_why("");

    return 1u;
}

/* 五、主循环 */
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
    s_rev_flipped = 0u;
    s_drv  = PROTO_SRC_NONE;
    s_local = 0u;
    pb_en  = 0.0f;
    /* 清掉就等于把 Flash 里存的参数抹了。 */
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

    /* ---- 1) 安全门 ---- */
    if (head_test_active())
    {
        /* 航向测试（App 的测试页）优先：它也在写 target_yaw / spd_target_*，
         * 两边同时跑就是"两个源抢同一个执行器"。让位方必须是回放 —— 测试是人手按下的短时实验。
         */
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "head test busy"); }
        else                   { pb_set_why("ht busy"); }   /* 没在跑也要说原因，否则屏幕只剩 WAIT */
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
    /* With no receiver plugged, RC_Engage is ALWAYS FAILSAFE, so the old gate
     * blocked PUSH as well -> the servo never followed AND the speed loop held
     * target=0 (car could not be pushed), with no error shown anywhere.
     * PUSH drives no motor (a human pushes), so it must not require a live RC.
     * ON = someone is actually driving by RC -> yield; ESTOP = always highest.
     * 在线时（能按急停）允许 PWR，见下面的判据。
     */
    /* 现场常态是**不插接收机**（遥控不开），靠手机/上位机走 WiFi·SPI 看着车 ——
     * 于是"遥控没开 ⇒ PWR 永远 WAIT"，可 PUSH 又能跑（因为 PUSH 不需要这条），
     * 看上去像"PWR 坏了"。
     * ⇒ 判据改成：FAILSAFE + PWR + **链路也不在** 才挡。
     * `motor_link_online()` 自带超时（spd_lost_ms），手机一断线立刻恢复保护。
     */
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_ESTOP == RC_Engage) ||
        ((RC_ENGAGE_FAILSAFE == RC_Engage) && (pb_mode >= 1.5f) &&
         (0u == motor_link_online())))
    {
        if (PB_ST_RUN == s_st) { pb_finish(PB_ST_IDLE, "RC ON/ESTOP"); }
        /* 现场最常卡的两条：
         * · RC ON          —— CH6 还合着，回放必须让位（这是设计，不是 bug）
         * · RC lost: PWR   —— 接收机没插/没开，`RC_Engage` **恒为 FAILSAFE**，
         * PWR 被安全门挡死（PUSH 不受影响）⇒ 屏幕上必须看得见，
         * 否则就是"切 PWR 电机根本不转"却一点线索都没有。
         */
        if (RC_ENGAGE_ESTOP == RC_Engage)      { pb_set_why("ESTOP latch"); }
        else if (RC_ENGAGE_ON == RC_Engage)    { pb_set_why("RC ON: CH6->OFF"); }
        /* 只有"既没接收机、又没链路"才走到这里（链路在线时上面的门不会进） */
        else                                   { pb_set_why("no RC & no link"); }
        s_arm = 0u;
        return;
    }
    /* 否则急停期间本模块还在往 spd_target_* 写速度，解除急停那一拍车会直接窜出去。 */
    if (0u != spd_safety_estop())
    {
        /* 所以"回放要不要中止"完全可以慢 20 ms 再判。
         * 18:05 那份日志里 CH5 只跳了**一帧**（20 ms）就把整趟废了、pb_en 清 0，
         * 而且退出时方向盘还锁在满舵 —— 一个坏帧的代价太大了。
         */
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

    /* 放在**所有安全门之后**：斜坡期间急停/遥控抢占照样能进来把它打断。 期间还占着驱动权与本地授权（车还在受控地减速），走完再 pb_finish 交还。 */
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

    /* ---- 1b) 只在 pb_en 的 **0→1 上升沿**启动 ---- 下一拍会**原地重新起跑**。所以必须抓沿。 */
    if (0u == s_arm)
    {
        s_arm = 1u;
        if (0u == pb_start())
        {
            /* Clearing it erased "the user asked for PUSH", so the free-wheel
             * sync released the coast flag and the speed loop locked the car at
             * target=0 while the screen showed OFF.
             * Keep pb_en=1 (intent), state=ABORT + reason on screen; the car stays
             * pushable. s_arm=2 latches the retry until Follow goes OFF then on,
             * or the user presses K1 again (menu calls route_pb_rearm()).
             */
            s_arm = 2u;
            s_st  = PB_ST_ABORT;
            return;
        }
            s_st = PB_ST_RUN;
        pb_set_why("");

        /* 回放留下的 target_yaw，方向盘会先甩一下。 */
        steer_ctrl_set_target_yaw(o.yaw);
    }
    if (PB_ST_RUN != s_st) { return; }

    /* · 一旦 owner 变了（遥控 CH6 合闸抢占 / 上位机断链后菜单接管），**立刻让位收车**。
     * 不做这一步的话，遥控把车抢过去了、回放还在 5ms 里往 spd_target_* 写速度，
     * 两边打架，而人看不出是谁在下命令。
     */
    if (proto_drv_owner() != s_drv)
    {
        pb_finish(PB_ST_ABORT, "drv preempted");
        pb_en = 0.0f;                       /* 释放命令：否则下一拍会"原地重新起跑" */
        s_arm = 0u;
        return;
    }
    proto_drv_renew(s_drv);

    /* ---- 2) 当前位姿（路表系，**只用相对量**）----
     * 与 pb_start 同一套基准：把"起跑那一拍的位姿"钉成表的第 0 点，之后全用增量换算。
     * 烧录/断电/抱车之后就会算到几十米外（17:25 那次满舵左就是这么来的）。
     * 旧代码：位置增量只转 −s_o0yaw（等于**假定 p[0].yaw ≡ 0**），
     * 航向却老老实实叠了 p[0].yaw。两者角度不同 ⇒ 不是刚体！
     * `p[0].yaw == 0` 时两者恰好一致（正车录的表永远满足），所以这个 bug 一直没露头。
     * 一旦把倒车录的点序倒过来，p[0] 变成原表末点、p[0].yaw = 87.68° ⇒
     * 车"以为自己朝 87.68°"、却"按 0° 的规矩走路" ⇒ 位置与航向对不上 87.68°，
     * Pure Pursuit 的横向外环变成正反馈，车一路画大弧跑飞
     * （仿真：最大偏离 3.081 m / RMS 1.719 m）。
     * 改成下面这版后同一张表最大偏离 0.024 m / RMS 0.012 m，且
     * p[0].yaw == 0 时与原代码逐字节等价，正车录的表零回归**。
     * 刚体 ⇒ θ = ψ + α，**位置增量也必须转 α**
     * px,py = p[0] + R(α)·(o − o0)
     * 锚点 (o0, ψ0) 映到 (p[0], p[0].yaw) ⇒ α = p[0].yaw − s_o0yaw
     * 下面这两行用的是 R(−θ) 的那套符号，所以 θ 要取 α 的相反数：
     * θ = s_o0yaw − p[0].yaw
     * （`route_pb.h` 的 PB_D2R 是度→弧度的宏。）
     */
    p = ins_route_get_points();
    ins_odom_snapshot(&o);
    {
        float dx0 = o.x - s_o0x;
        float dy0 = o.y - s_o0y;
        float th  = (s_o0yaw - p[0].yaw) * PB_D2R;
        float c   = cosf(th);
        float sn  = sinf(th);

        px   = p[0].x + ( dx0 * c + dy0 * sn);
        py   = p[0].y + (-dx0 * sn + dy0 * c);
        pyaw = pb_wrap180(p[0].yaw + (o.yaw - s_o0yaw));
    }

    /* ---- 3) 最近点（单调）---- */
    best  = pb_find_nearest(px, py, p);
    s_idx = best;

    /* 8 字表首尾只差 74 mm，而完成判据是"离末点 < pb_fin(0.15 m)"
     * ⇒ 车停得准的时候（离起点 <0.15 m），**起跑第一拍就判 DONE**：
     * pb_finish(DONE) + pb_en=0 ⇒ 菜单从 PUSH 一闪 WAIT 又掉回 OFF，
     * 现场就是"始终切不到 PUSH"。21:11 那次能跑起来，只是因为车停在 0.2 m 外。
     * 保护：进度必须走过"完成阈值点"（约为表长 1/4，夹在 4~30 点 = 0.3~2.2 m）之后，
     * 才允许进终点圈判完成。半圈表也成立（阈值随 n 缩小，且不超过 n/2）。
     */
    {
        uint16 need = (uint16)(s_n / 4u);

        if (need < 4u)                      { need = 4u; }
        if (need > 30u)                     { need = 30u; }
        if (need > (uint16)(s_n / 2u))      { need = (uint16)(s_n / 2u); }
        if (best >= need)                   { s_done_armed = 1u; }
    }

    /* · 进终点圈 ⇒ COMPLETED（正常收车）
     * · 索引到末点、但离末点还远 ⇒ ABORTED（脱线/跳点，**不是**跑完了）
     * 分开报的理由：现场要能一眼区分"跑完"和"跑飞"，工程 10 里两者都长成
     * "到表尾 + pb_en 归 0"，追责时看不出来。
     */
    {
        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;
        float ld2 = sqrtf(lx * lx + ly * ly);

        /* 就是 `if(last_dist < FINISH_DIST) complete`），先前我自己加的"必须走过一半"
         * 已经撤掉 —— 那是为了绕开"8 字起点≈终点"，但真正的保护是：
         * ① 起始进度点从第 3 点起（不是 0）；② 距离圈只有 pb_fin(0.15 m)；
         * ③ 进度索引单调，只前进不后退。加"过半"反而会让半圈表永远收不了车。
         */
        if ((0u != s_done_armed) && (ld2 < ((pb_fin > 0.02f) ? pb_fin : 0.02f)))
        {
            /* 会立刻给反向大占空比 ⇒ 电流冲击 + 车头点一下。 只对 PWR 做（PUSH 本来就不给力，coast 已在跑）。 */
            if ((pb_mode >= 1.5f) && (0u == s_ramp) &&
                ((fabsf(spd_target_l) + fabsf(spd_target_r)) > 0.03f))
            {
                s_ramp_l  = spd_target_l;
                s_ramp_r  = spd_target_r;
                s_ramp_t0 = motor_link_now_ms();
                s_ramp    = 1u;
                printf("[PB] 终点停车斜坡: %.2f/%.2f rps -> 0 over %u ms\r\n",
                       s_ramp_l, s_ramp_r, (unsigned)PB_STOP_MS);
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

    /* 长直道会一直偏慢。这里改回最近点。 */
    {
        float v = s_vlim[best];
        float mpr;

        /* 于是"倒车段开环打角"只是把方向盘打过去、车一步都不退。 只是别超过 pb_vmax（反方向同样的速度上限）。 */
        if (v >  pb_vmax) { v =  pb_vmax; }
        if (v < -pb_vmax) { v = -pb_vmax; }
        s_tv = v;

        if (pb_mode < 1.5f)
        {
            /* 只在沿上写会让"切过来"这一路仍带着上一次的目标速度 —— 人推车时车自己窜出去。
             * 反向下力刹车，人根本推不动（现场实测）。必须走 `spd_ctrl_set_coast(1)`：
             * 输出钉 0（= 自由轮）、PID 不跑，但**测速/里程照跑**（推车时位姿还要用）。
             */
            spd_target_l = 0.0f;
            spd_target_r = 0.0f;
            spd_ctrl_set_coast(1u);
        }
        else
        {
            float vl, vr;

            spd_ctrl_set_coast(0u);         /* 带速度跟随：先撤自由轮，再给目标 */

            /* PWR 时就是本模块拿 —— 否则 SAFE_BIT_NOT_EN 一直挂着，电机不出力。 */
            if (0u == s_local)
            {
                /* `set_local(0)`，一把清零的写法会让这里 5 ms 就失效 （PWR 电机不转的真因）。点完顺手打一行"到底出不出力"。 */
                spd_safety_set_local_src(SPD_LOCAL_SRC_PB, 1u);
                s_local = 1u;
                printf("[PB] PWR 授权: local=%u allow=%u flags=0x%04X (allow=0 就是出力被闸死)\r\n",
                       (unsigned)spd_safety_local(), (unsigned)spd_safety_allow(0u),
                       (unsigned)spd_safety_flags());
            }

            mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;

            /* 转弯时内外轮弧长不同（满舵 ±23%）。不差速 = 内轮被拖着走 ⇒ 里程虚高 +22%
             * ⇒ 位姿/航向整体偏。分配律见 spd_ctrl.h：
             * v_l = v(1 − κW/2)   v_r = v(1 + κW/2)   κ = tan(δ_meas)/L
             */
            spd_diff_alloc(v, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

            spd_target_l = vl / mpr;
            spd_target_r = vr / mpr;
        }
    }

    /* 用目标速度的理由：剖面已经按几何算好了"这里该多快"，前视跟着它走是确定的；
     * 跟实测走会被打滑/被推着走带偏（工程 10 的注释里也承认这一点，但选了实测）。
     * 把 pb_ld_b 调小、pb_ld_min 调大即可。
     */
    vabs = fabsf(s_tv);
    ld   = pb_ld_a + pb_ld_b * vabs;
    ld   = pb_clampf(ld, pb_ld_min, pb_ld_max);
    if (ld < 0.05f) { ld = 0.05f; }

    /* κ_cmd = 2·sin(α)/ld，车能给的上限 tan(δ_lim)/L。令两者相等得
     * α_max = asin( ld · tan(δ_lim) / (2L) )
     * 按出厂值（ld 0.5 m、L 0.66 m、27°）算出来只有 **11.3°**：车头一偏 12° 以上，
     * 前视点就落进车的**最小转弯圆内部**，目标物理上追不到 ⇒ 舵角顶死 ⇒ 纯追踪会
     * 稳定地"绕着它转圈"（PP 的已知不可达平衡点，抖动/增益都救不回）。
     * 转满 257°~263° 才偶然脱出；而 221519 起步只偏 +2.7°，跑出 2.6 cm。
     * 解法不是"把 ld 统一调大"——那会在 R≈1.3 m 的弯里系统性切内 ld²/(8R) ≈ 0.3 m。
     * 而是**只在当前目标确实不可达时**把 ld 放大到刚好可达：
     * 要 sin|α| ≤ ld·tan(δ_lim)/(2L)  ⇒  ld ≥ |sin α| · 2L / tan(δ_lim)
     * 偏角小的时候这一项远小于 pb_ld_a/b 的结果，**直道和缓弯完全不受影响**。
     * α 取**上一拍**的值（滞后一拍 ≈ 50 ms），省掉"算 ld 要先知道 α、算 α 要先知道 ld"
     * 的循环依赖；偏角一大，两拍之内就会把 ld 顶到该有的位置。
     * 仍然受 pb_ld_max 约束（出厂 1.5 m ⇒ α_max 35°，对本车够用）。
     */
    {
        float tan_lim = tanf(steer.limit_deg * PB_D2R);

        if (tan_lim > 1.0e-3f)
        {
            float ld_need = (fabsf(sinf(s_alpha)) * 2.0f * pb_l) / tan_lim;

            if (ld < ld_need) { ld = ld_need; }
        }
        if (ld > pb_ld_max) { ld = pb_ld_max; }
    }
    s_ld = ld;

    /* 6b 把 α_max 抬到 45° 之后，理论上不该再出现绕圈；但表首那段甩头、摆车、
     * 打滑仍可能让车锁住。判据只看可观测量，三条同时成立才收车：
     * ① 实测舵角贴着限位（0.95 × steer.limit_deg）
     * ② elat 已经不小（> PB_LOST_MIN_M）
     * ③ **最近点索引 PB_LOST_MS 内没有前进**（打转的特征；正常过急弯时索引一直在走）
     * 满足就 pb_finish(ABORT)，原因写进 pb_set_why，让菜单第 7 项能看见，
     * 而不是闷头转到撞墙（实车那次转了 257°）。
     */
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
                pb_en      = 0.0f;             /* 释放命令：否则下一拍会"原地重新起跑" */
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

    /* Pure Pursuit 的航向外环是"车头"的负反馈；倒车时车尾才是被控端，同一套符号
     * 会变成**正反馈** ⇒ 越跟越偏、原地打圈。所以倒车段：关外环、只开转角内环、
     * 开环回放录制那一拍的打角。
     * 判据用**录制速度**（p[best].v），不是剖面速度：剖面在换向前会被压到接近 0，
     * 拿它判段会在换向点附近来回翻。
     * 带死区滞回（PB_REV_EPS），防边界反复翻使能位。
     */
    {
        float recv = p[best].v;

        if (recv >  (float)PB_REV_EPS)      { s_rev = 0u; }
        else if (recv < -(float)PB_REV_EPS) { s_rev = 1u; }
    }

    if (0u != s_rev)
    {
        float delta = (float)p[best].steer_cdeg / 100.0f;      /* 录制那一拍实际施加的角 */
        float err   = pb_wrap180(p[best].yaw - pyaw);           /* 表系里的航向误差 */
        float corr  = (float)PB_REV_HEAD_KP * err;

        /* 因为它的倒车段有好几米、漂 11°；我们这张表倒车只有 7 点 ≈ 0.7 m，先不开。 真要开，先确认符号：反了就是正反馈（越纠越斜）。 */
        delta += corr;
        if (delta >  27.0f) { delta =  27.0f; }
        if (delta < -27.0f) { delta = -27.0f; }

        if (0u != s_head)
        {
            steer_ctrl_set_head_enable(0u);     /* 关掉前进用航向外环 */
            s_head = 0u;
        }
        steer_ctrl_set_target_delta(delta);     /* 只走内环 */

        s_ytgt = pb_wrap180(o.yaw + err);       /* 仅给遥测/调试看 */
        s_aim  = best;

        return;
    }
    else if (0u == s_head)
    {
        /* 从倒车段回到前进段：把外环重新打开（9a 里也会开，这里只是早点，避免一拍滞后） */
        steer_ctrl_set_head_enable(1u);
        s_head = 1u;
    }

    /* ---- 9) 沿弧长找前视点 ---- */
    s_aim = pb_find_lookahead(best, p, ld);
    j     = s_aim;

    /* ---- 10) 转向：两条通道（数学等价，留两条做 A/B）----
     * 车体系：x 向右 / y 向前；alpha 正 = 目标点在右；
     * `steer_delta > 0` = 右转（已实测），`yaw` 正 = 左转。
     * ⇒ 前视点在世界系的**目标航向** = 当前航向 − alpha。
     */
    dxr = p[j].x - px;
    dyr = p[j].y - py;
    pr  = pyaw * PB_D2R;
    bx  =  dxr * cosf(pr) + dyr * sinf(pr);
    by  = -dxr * sinf(pr) + dyr * cosf(pr);
    alpha = atan2f(bx, by);                 /* rad，正 = 右 */
    s_alpha = alpha;

    if (pb_smode < 0.5f)
    {
        /* 15 与 ld 无关 ⇒ 前视一变就成了另一个环，谁开谁振。 */
        float k = (pb_kh > 1.0e-3f) ? pb_kh : (2.0f * pb_l / ld);

        k = pb_clampf(k, 0.3f, 8.0f);
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D);

        steer_ctrl_set_head_kp(k);
        if (0u == s_head) { steer_ctrl_set_head_enable(1u); s_head = 1u; }
        steer_ctrl_set_target_yaw(s_ytgt);

        /* 等于绕一圈回到 9b。想用手工表的打角，把 pb_smode 拨到 1。 */
    }
    else
    {
        /* ---- 9b) 直接给 δ（工程 10 通道）---- κ = 2·sin(α)/ld，δ = atan(L·κ)。参数少、相位短，但没有"目标航向"可看。 */
        float kappa = 2.0f * sinf(alpha) / ld;
        float delta = atanf(pb_l * kappa) * PB_R2D;

        if (0u != s_head) { steer_ctrl_set_head_enable(0u); s_head = 0u; }
        s_ytgt = pb_wrap180(o.yaw - alpha * PB_R2D);

        /* 打角前馈（手工/人开车方案）：δ = w·steer[best] + (1−w)·δ_pp。 再叠一个 steer[] 就是**双倍打角**（弯里切内、直道来回摆）。 */
        if ((pb_ff_en >= 0.5f) && (0u != (p[best].flags & (uint16)INS_WP_FL_STEER)))
        {
            float w  = pb_clampf(pb_ff_gain, 0.0f, 1.0f);
            float st = (float)p[best].steer_cdeg / 100.0f;

            delta = w * st + (1.0f - w) * delta;
        }
        steer_ctrl_set_target_delta(delta);
    }
}

/* 六、读出接口（遥测/调试） */
/* 每 5 ms 由中断调用：只按"用户意图 + 谁在开车"决定要不要自由轮。 见 route_pb.h 的说明 —— 不能依赖状态机走到 PUSH 分支。 */
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

/* 起跑失败后允许再试一次：s_arm 会锁在 2，必须 OFF->ON 才重来。 菜单每次把 Follow 写成 PUSH/PWR 时调它 —— 补完表/开航位后按一下就能再试。 */
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
