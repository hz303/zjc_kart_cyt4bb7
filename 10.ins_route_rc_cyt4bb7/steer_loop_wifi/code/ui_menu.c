/*********************************************************************************************************************
 * 工程 10（卡丁）· 屏幕菜单实现（见 ui_menu.h）
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "ui_menu.h"
#include "ui_core.h"
#include "zf_device_key.h"

#include "motor_link.h"
#include "spd_ctrl.h"
#include "spd_enc.h"
#include "spd_safety.h"
#include "steer_ctrl.h"
#include "remote_ctrl.h"
#include "ins_odom.h"
#include "ins_imu.h"

#include "spd_bench.h"
#include "bench_log.h"
#include "app_params.h"
#include "ins_route.h"          /* 路表：点数/槽号/点距 */
#include "route_app.h"          /* 路表：录制/选槽命令量 */
#include "route_pb.h"           /* 路表：回放使能 */

/*===================================================================================================================
 *  一、页面模式
 *=================================================================================================================*/
#define UM_LIST         (0u)
#define UM_EDIT         (1u)
#define UM_DASH         (2u)

#define UM_ROW_TITLE    (0u)
#define UM_ROW_ITEM0    (1u)
#define UM_ROW_HINT     (17u)       /* 固定最后一行，与工程 8 一样 */
#define UM_HINT         "K1=OK K2=Bk K3/K4=Move"

/* ★ 列表光标方向符号（2026-10-17）
 *   屏转 180°（`UI_ROT180=1`，见 ui_core.h）后，屏幕上的"往下"= 面板坐标的"往上"。
 *   如果现场发现「按往上那颗键光标反而往下走」，改这里 +1 ⇄ -1 就行 ——
 *   **不要去动接线、也不要改 KEY_LIST 顺序**（那是 K3/K4 物理位置的事，两者别混着改，
 *   否则一次动两处，下次出问题分不清是哪一处）。 */
#define UM_DIR_SIGN     (+1)

/*===================================================================================================================
 *  二、菜单项
 *     MK_ACT = 执行一个动作；MK_PF = 编辑参数表里的某一项（按 ID 索引，**量程取自车端参数表**）
 *=================================================================================================================*/
typedef struct
{
    const char *name;
    uint8       kind;               /* 0 = ACT, 1 = PF */
    void      (*act)(void);
    uint8       pid;               /* PF：参数表 ID */
} um_item_t;

static void act_dash (void);
static void act_bench(void);
static void act_dump (void);
static void act_clr  (void);
static void act_yaw  (void);
static void act_psave(void);
static void act_pload(void);
static void act_rtrec(void);
static void act_rtslot(void);
static void act_mark (void);
static void act_play (void);
static void act_sts  (void);

static const um_item_t UM_ITEMS[] =
{
    /* name          kind    action      pid */
    { "Dashboard",  0u, act_dash,   0u  },
    { "SteerEn",    1u, 0,          23u },   /* 转向|内环使能        */
    { "Bench GO",   0u, act_bench,  0u  },   /* ★ 一键跑班（12 段） */
    { "BenchClr",   0u, act_clr,    0u  },
    { "BenchDump",  0u, act_dump,   0u  },   /* 导出 CSV 到调试串口  */
    { "OlEn",       1u, 0,          29u },   /* 开环模式             */
    { "OlDutyL",    1u, 0,          30u },
    { "OlDutyR",    1u, 0,          31u },
    { "k_v",        1u, 0,          4u  },
    { "k_z",        1u, 0,          5u  },
    { "Umax",       1u, 0,          22u },   /* 转向输出上限         */
    { "Mpm",        1u, 0,          34u },
    { "Yaw Reset",  0u, act_yaw,    0u  },
    { "ParmSve",    0u, act_psave,  0u  },   /* ★ 写 Flash 页 0      */
    { "ParmLd",     0u, act_pload,  0u  },   /* 重新从 Flash 载入    */
    /* ---- 路表（P1/P2）。列表超过一屏会自动滚动，见 um_draw_list() ---- */
    { "RtRec",      0u, act_rtrec,  0u  },   /* 开始/停止录制 -> 槽 0 */
    { "RtSts",      0u, act_sts,    0u  },   /* ★ 路表状态：槽/点数/REC-SAV-ok/上次返回码 */
    { "RtSlot",     0u, act_rtslot, 0u  },   /* 切到下一槽并载入      */
    { "RtMark",     0u, act_mark,   0u  },   /* ★ 打关键点（手工方案） */
    { "Play",       0u, act_play,   0u  },   /* 回放开/关（pb_mode 1 只转向）*/
    { "FF En",      1u, 0,          54u },   /* ★ 回放|打角前馈使能   */
    { "FF w",       1u, 0,          55u },   /* ★ 回放|前馈融合权重   */
};

#define UM_NITEMS   ((uint8)(sizeof(UM_ITEMS) / sizeof(UM_ITEMS[0])))

/* 16 项 + 标题 1 行 + 提示 1 行 ⇒ 18 行；屏幕 320/16 = 20 行 ⇒ 不用滚动。
   ⚠ 加项超过 18 项时必须加滚动，这里先编译期拦住。 */
/* ⚠ 原先是"菜单项必须一屏放下"，一加项就编译失败。现在列表会滚动 ⇒
   只需至少 1 项；窗口大小由 um_draw_list() 里的 span 决定。 */
typedef char um_fit_check[(UM_NITEMS >= 1u) ? 1 : -1];

/*===================================================================================================================
 *  三、参数表查找（**量程/指针都从车端表里取，不在菜单里重复写一遍**）
 *=================================================================================================================*/
static volatile float *um_ptr(uint8 id)
{
    switch (id)
    {
#define UM_CASE_PTR(ID, NM, LB, LOV, HIV, DF, UN, STV, PTR)  case ID: return (volatile float *)(PTR);
        MOTOR_PARAM_TABLE(UM_CASE_PTR)
#undef UM_CASE_PTR
        default: return 0;
    }
}

static uint8 um_range(uint8 id, float *lo, float *hi, float *st)
{
    switch (id)
    {
#define UM_CASE_RNG(ID, NM, LB, LOV, HIV, DF, UN, STV, PTR)  case ID: *lo = (LOV); *hi = (HIV); *st = (STV); return 1u;
        MOTOR_PARAM_TABLE(UM_CASE_RNG)
#undef UM_CASE_RNG
        default: return 0u;
    }
}

/*===================================================================================================================
 *  四、状态
 *=================================================================================================================*/
static uint8  s_mode   = UM_LIST;
static uint8  s_cur    = 0u;
static uint8  s_rep3   = 0u;        /* K3 长按连按计数 */
static uint8  s_rep4   = 0u;        /* K4 长按连按计数 */
static uint8  s_top    = 0u;        /* 列表窗口顶端（菜单比屏幕长时滚动用）*/
static uint32 s_last   = 0u;
static uint32 s_stlast = 0u;
static uint8  s_savlast= 0u;        /* 上一拍的"落盘中"标志（1→0 = 保存结束）*/
static char   s_msg[24] = "";

/* 仪表盘字段表 */
static void dash_fmt(char *buf, uint8 size, void *ctx);
static const uint8 s_dix[10] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u };
static ui_field_t  s_df[10];
static ui_page_t   s_dpage;

/*===================================================================================================================
 *  五、动作
 *=================================================================================================================*/
static void act_dash(void)
{
    s_mode = UM_DASH;
    ui_page_mark_dirty(&s_dpage);
}

static void act_bench(void)
{
    uint8 r = spd_bench_start();

    if (0u == r) { snprintf(s_msg, sizeof(s_msg), "bench started"); }
    else if (SB_REF_NO_RC       == r) { snprintf(s_msg, sizeof(s_msg), "REFUSE: RC offline"); }
    else if (SB_REF_ESTOP       == r) { snprintf(s_msg, sizeof(s_msg), "REFUSE: CH5 estop");   }
    else if (SB_REF_NOT_STANDBY == r) { snprintf(s_msg, sizeof(s_msg), "REFUSE: arm CH6+thr 0"); }
    else if (SB_REF_BUSY        == r) { snprintf(s_msg, sizeof(s_msg), "REFUSE: busy");       }
    else                              { snprintf(s_msg, sizeof(s_msg), "REFUSE: %u", (unsigned)r); }
}

static void act_dump(void)
{
    bench_log_dump();
    snprintf(s_msg, sizeof(s_msg), "dump %u rec @115200", (unsigned)bench_log_stored());
}

static void act_clr(void)
{
    bench_log_clear();
    snprintf(s_msg, sizeof(s_msg), "flash bench cleared");
}

static void act_yaw(void)
{
    ins_odom_reset();
    ins_imu_zero_yaw();
    snprintf(s_msg, sizeof(s_msg), "yaw/pose zeroed");
}

static void act_psave(void)
{
    uint8 e = app_param_save();

    if (0u == e) { snprintf(s_msg, sizeof(s_msg), "saved to flash OK"); }
    else         { snprintf(s_msg, sizeof(s_msg), "SAVE FAIL err=%u", (unsigned)e); }
}

static void act_pload(void)
{
    app_param_init();
    snprintf(s_msg, sizeof(s_msg), "loaded ld=%u err=%u",
             (unsigned)app_param_loaded(), (unsigned)app_param_last_err());
}

/* ---- 路表（P1/P2）----
 *  ⚠ 录制一律写**槽 0**（手推表"的家"）；电子图从 PC 灌到槽 1~3，协议层禁止写 0。
 *  ⚠ 消息里不用浮点：snprintf 走同一套 IAR 库，%f 不一定支持。 */
static void act_rtrec(void)
{
    if (rt_rec_cmd < 0.5f)
    {
        rt_rec_cmd = 1.0f;
        snprintf(s_msg, sizeof(s_msg), "REC start ds=%umm",
                 (unsigned)(ins_route_ds * 1000.0f + 0.5f));
    }
    else
    {
        rt_rec_cmd = 0.0f;
        /* ⚠ 停录只是"请求保存"：真正的擦写由主循环一页一页做（一轮 ≈ 16 页 + 头页 + 索引页
           ≈ 1 s）。**这期间 reset/断电 = 只写了半张表，头页没写 ⇒ 上电载入失败 ⇒ 0 点。**
           所以提示语后面必须让用户看到 SAV→ok（见下方 ui_menu_task 里的自动刷新）。 */
        snprintf(s_msg, sizeof(s_msg), "REC stop n=%u SAV..",
                 (unsigned)route_app_count());
    }
}

/* ---- 路表状态快照（现场没串口时唯一能看的地方）----
 *  rc: 1=写完 2=读回校验失败 3=没东西可写 4=槽越界
 *  ⚠ 按一下是**快照**，要看保存进度就等一秒再按一次。 */
static void act_sts(void)
{
    uint8 rc = ins_route_flash_last_rc();

    snprintf(s_msg, sizeof(s_msg), "S%u %upt %s rc=%u",
             (unsigned)ins_route_slot_active(),
             (unsigned)route_app_count(),
             (0u != ins_route_is_running()) ? "REC" :
             ((0u != ins_route_flash_busy()) ? "SAV" : "ok"),
             (unsigned)rc);
}

static void act_rtslot(void)
{
    uint8 s = route_app_slot_next();

    snprintf(s_msg, sizeof(s_msg), "S%u %upt %umm", (unsigned)s,
             (unsigned)route_app_count(),
             (unsigned)(ins_route_get_head()->total_dist * 1000.0f + 0.5f));
}

static void act_play(void)
{
    if (pb_en < 0.5f)
    {
        if (route_app_count() < (uint16)PB_MIN_POINTS)
        {
            snprintf(s_msg, sizeof(s_msg), "REFUSE: no table");
            return;
        }
        pb_en = 1.0f;
        snprintf(s_msg, sizeof(s_msg), "PLAY on (mode %u)", (unsigned)pb_mode);
    }
    else
    {
        pb_en = 0.0f;
        snprintf(s_msg, sizeof(s_msg), "PLAY off");
    }
}

/* ---- ★ 手工方案：车上打关键点（录制中按一下=标在最后一个已录点上）----
 *  用途：人开车过锥桶时，在每个锥桶/弯心按一下 ⇒ 表里那一点多一个 KEY 标记。
 *  ⚠ 打点不动任何控制，只写 RAM ⇒ 行驶中按也安全。
 *  ⚠ 用了**一个脉冲**（置 1 后立刻清）：route_app_poll 只认 0→1 边沿，
 *    若这里留着 1，下一次录制开始时会被当成"又要打一个点"。 */
static void act_mark(void)
{
    uint8 ok;

    rt_mark_cmd = 1.0f;
    ok = ins_route_mark_key();              /* 直接调一次，屏幕立刻给反馈 */
    rt_mark_cmd = 0.0f;

    if (0u != ok)
    {
        snprintf(s_msg, sizeof(s_msg), "KEY #%u @%upt",
                 (unsigned)ins_route_key_count(), (unsigned)route_app_count());
    }
    else
    {
        snprintf(s_msg, sizeof(s_msg), "KEY: not recording");
    }
}

/*===================================================================================================================
 *  六、绘制
 *=================================================================================================================*/
/* 安全写串：先过 ui_fit_chars 截断（越界 = IPS200 的 zf_assert = while(true) = 整机死机） */
static void um_puts(uint8 row, uint8 col, const char *s)
{
    char t[48];
    uint8 n = ui_fit_chars(ui_col(col), 46u);

    if (0u == n) { return; }
    if (n > 45u) { n = 45u; }
    memcpy(t, s, n);
    t[n] = '\0';
    ui_puts_row(row, col, t);
}

static const char *um_bench_state_str(void)
{
    switch (spd_bench_state())
    {
        case SB_RUN:     return "RUN";
        case SB_WAITLOG: return "WRITE";
        case SB_DONE:    return "DONE";
        case SB_ABORT:   return "ABORT";
        case SB_REFUSED: return "REFUSED";
        default:         return "IDLE";
    }
}

static void um_draw_list(void)
{
    uint8 span = (uint8)(UM_ROW_HINT - UM_ROW_ITEM0);   /* 一屏能放几项 */
    uint8 i;

    ui_clear();

    /* ★ 窗口跟随光标：菜单项比屏幕长时自动滚动（原来写死"全列出来"，
       加到 16 项就顶到编译期断言上了 —— 路表/回放还要加，所以改成窗口）。 */
    if (s_cur < s_top) { s_top = s_cur; }
    if ((uint8)(s_cur - s_top) >= span) { s_top = (uint8)(s_cur - span + 1u); }
    if (s_top > (uint8)(UM_NITEMS - 1u)) { s_top = (uint8)(UM_NITEMS - 1u); }

    um_puts(UM_ROW_TITLE, 0u, "KART BENCH MENU");
    um_puts(UM_ROW_TITLE, 16u, um_bench_state_str());

    for (i = 0u; i < span; i++)
    {
        uint8    k = (uint8)(s_top + i);
        char     line[48];
        char     val[16];
        uint8    row = (uint8)(UM_ROW_ITEM0 + i);

        if (k >= UM_NITEMS) { break; }

        if (UM_ITEMS[k].kind == 1u)
        {
            volatile float *p = um_ptr(UM_ITEMS[k].pid);

            if (0 != p) { ui_ftoa(val, sizeof(val), *p, 3u); } else { val[0] = '?'; val[1] = 0; }
            snprintf(line, sizeof(line), "%-10s %s", UM_ITEMS[k].name, val);
        }
        else
        {
            snprintf(line, sizeof(line), "%s", UM_ITEMS[k].name);
        }

        um_puts(row, 2u, line);
    }

    um_puts(UM_ROW_HINT, 0u, UM_HINT);

    if (s_msg[0] != '\0') { um_puts(UM_ROW_TITLE, 0u, "KART  "); um_puts(UM_ROW_TITLE, 6u, s_msg); }

    /* 光标（按窗口内的相对行号画） */
    um_puts((uint8)(UM_ROW_ITEM0 + (uint8)(s_cur - s_top)), 0u, ">");
}

static void um_draw_edit(void)
{
    const um_item_t *it  = &UM_ITEMS[s_cur];
    volatile float  *p   = um_ptr(it->pid);
    float lo, hi, st;
    char  t[32];
    char  line[48];

    if ((0 == p) || (0u == um_range(it->pid, &lo, &hi, &st))) { s_mode = UM_LIST; return; }

    ui_clear();

    um_puts(0u, 0u,  "EDIT");
    snprintf(line, sizeof(line), "id=%u %s", (unsigned)it->pid, it->name);
    um_puts(1u, 0u, line);

    ui_ftoa(t, sizeof(t), *p, 4u);
    snprintf(line, sizeof(line), "= %s", t);
    um_puts(3u, 0u, line);

    ui_ftoa(t, sizeof(t), st, 4u);
    snprintf(line, sizeof(line), "step %s", t);
    um_puts(4u, 0u, line);

    ui_ftoa(t, sizeof(t), lo, 3u);
    snprintf(line, sizeof(line), "min  %s", t);
    um_puts(5u, 0u, line);

    ui_ftoa(t, sizeof(t), hi, 3u);
    snprintf(line, sizeof(line), "max  %s", t);
    um_puts(6u, 0u, line);

    um_puts(8u,  0u, "K3/K4 = -/+");
    um_puts(9u,  0u, "K1 = OK  K2 = Back");
}

/*===================================================================================================================
 *  七、仪表盘字段
 *=================================================================================================================*/static void dash_fmt(char *buf, uint8 size, void *ctx)
{
    uint8 i = *(const uint8 *)ctx;
    char  a[16];
    char  b[16];

    switch (i)
    {
        case 0u:
            ui_ftoa(a, sizeof(a), enc_l.rps, 2u);
            ui_ftoa(b, sizeof(b), enc_r.rps, 2u);
            snprintf(buf, size, "rps L%s R%s", a, b);
            break;

        case 1u:
            ui_ftoa(a, sizeof(a), enc_l.rps * ins_m_per_rev, 2u);
            ui_ftoa(b, sizeof(b), ins_odom_get_dist(), 2u);
            snprintf(buf, size, "v~%s m/s  d=%s m", a, b);
            break;

        case 2u:
            ui_ftoa(a, sizeof(a), ins_imu_get_yaw(), 1u);
            ui_ftoa(b, sizeof(b), ins_odom_get_x(), 2u);
            snprintf(buf, size, "yaw %s  x %s", a, b);
            break;

        case 3u:
            ui_ftoa(a, sizeof(a), ins_odom_get_y(), 2u);
            ui_ftoa(b, sizeof(b), steer.meas_delta, 2u);
            snprintf(buf, size, "y %s  steer %s", a, b);
            break;

        case 4u:
            ui_ftoa(a, sizeof(a), spd_ol_duty_l, 1u);
            ui_ftoa(b, sizeof(b), spd_ol_duty_r, 1u);
            snprintf(buf, size, "ol%u L%s R%s", (unsigned)(spd_ol_en > 0.5f), a, b);
            break;

        case 5u:
            snprintf(buf, size, "safe %u loc%u est%u",
                     (unsigned)spd_safety_allow(0u),
                     (unsigned)spd_safety_local(),
                     (unsigned)spd_safety_estop());
            break;

        case 6u:
            snprintf(buf, size, "rc c5=%u c6=%u e%u on%u",
                     (unsigned)RC_Raw_Ch5, (unsigned)RC_Raw_Ch6,
                     (unsigned)RC_Engage, (unsigned)RC_Online);
            break;

        case 7u:
            snprintf(buf, size, "bench %s s%u n%u",
                     um_bench_state_str(), (unsigned)spd_bench_seg(),
                     (unsigned)bench_log_count());
            break;

        case 8u:
            snprintf(buf, size, "store%u ld%u e%u",
                     (unsigned)bench_log_stored(),
                     (unsigned)app_param_loaded(),
                     (unsigned)app_param_last_err());
            break;

        default:
            snprintf(buf, size, "imu %u fl%u", (unsigned)ins_imu_link_ok(),
                     (unsigned)ins_odom_flags());
            break;
    }
}

/*===================================================================================================================
 *  八、初始化
 *=================================================================================================================*/
void ui_menu_init(void)
{
    uint8 i;

    key_init(10u);

    for (i = 0u; i < 10u; i++)
    {
        s_df[i].row    = (uint8)(1u + i);
        s_df[i].x      = 0u;
        s_df[i].policy = (i < 9u) ? UI_REALTIME : UI_STATIC;
        s_df[i].format = dash_fmt;
        s_df[i].ctx    = (void *)&s_dix[i];
        s_df[i].cache[0] = '\0';
    }

    s_dpage.fields            = s_df;
    s_dpage.count             = 10u;
    s_dpage.cursor_row        = -1;
    s_dpage.cursor_row_last   = -1;
    s_dpage.cursor_glyph      = 0u;
    s_dpage.cursor_glyph_last = 0u;
    s_dpage.dirty             = 1u;

    s_mode = UM_LIST;
    s_cur  = 0u;
    um_draw_list();
}

/*===================================================================================================================
 *  九、主循环任务
 *=================================================================================================================*/
uint8 ui_menu_in_list(void) { return (UM_LIST == s_mode) ? 1u : 0u; }

void ui_menu_task(void)
{
    uint32 now = motor_link_now_ms();
    uint8  k1s, k2s, k3s, k4s, h3, h4;
    int8   dir = 0;

    if ((uint32)(now - s_last) < 10u) { return; }       /* 10 ms 分频 */
    s_last = now;

    /* ★★ 落盘结束的那一拍自动更新提示行 —— 现场只有屏幕、没有串口时，
       这是唯一能确认"表真的写进 Flash 了"的地方。
       ⚠ 停录只是"请求保存"：主循环一页一页写（≈1 s）。这期间 reset/断电 =
         只写了一半、头页没写 ⇒ 上电 ins_route_flash_load() 判无效 ⇒ 0 点，
         表现得跟"从来没保存过"一模一样。 */
    {
        uint8 b = ins_route_flash_busy();

        if ((0u != s_savlast) && (0u == b))
        {
            uint8 rc = ins_route_flash_last_rc();

            snprintf(s_msg, sizeof(s_msg), "SAVE %s n=%u rc=%u",
                     (1u == rc) ? "ok" : "FAIL",
                     (unsigned)route_app_count(), (unsigned)rc);
            if (UM_LIST == s_mode) { um_draw_list(); }
        }
        s_savlast = b;
    }

    key_scanner();

    k1s = (KEY_SHORT_PRESS == key_get_state(KEY_1)) ? 1u : 0u;
    k2s = (KEY_SHORT_PRESS == key_get_state(KEY_2)) ? 1u : 0u;
    k3s = (KEY_SHORT_PRESS == key_get_state(KEY_3)) ? 1u : 0u;
    k4s = (KEY_SHORT_PRESS == key_get_state(KEY_4)) ? 1u : 0u;
    h3  = (KEY_LONG_PRESS  == key_get_state(KEY_3)) ? 1u : 0u;
    h4  = (KEY_LONG_PRESS  == key_get_state(KEY_4)) ? 1u : 0u;

    if (0u != k1s) { key_clear_state(KEY_1); }
    if (0u != k2s) { key_clear_state(KEY_2); }

    /* K3/K4：短按走一步；按住则每 100 ms（10 拍 × 10 ms）走一步 */
    if (0u != k3s)      { key_clear_state(KEY_3); s_rep3 = 0u; dir = 1; }
    else if (0u != h3)  { s_rep3++; if (s_rep3 >= 10u) { s_rep3 = 0u; dir = 1; } }
    else                { s_rep3 = 0u; }

    if (0u != k4s)      { key_clear_state(KEY_4); s_rep4 = 0u; dir = -1; }
    else if (0u != h4)  { s_rep4++; if (s_rep4 >= 10u) { s_rep4 = 0u; dir = -1; } }
    else                { s_rep4 = 0u; }

    /* ---------- DASH ---------- */
    if (UM_DASH == s_mode)
    {
        if ((0u != k1s) || (0u != k2s)) { s_mode = UM_LIST; um_draw_list(); return; }
        ui_page_render(&s_dpage, 0u);
        return;
    }

    /* ---------- EDIT ---------- */
    if (UM_EDIT == s_mode)
    {
        float lo = 0.0f, hi = 0.0f, st = 0.0f;
        volatile float *p = um_ptr(UM_ITEMS[s_cur].pid);

        if ((0 != p) && (0 != dir) && (0u != um_range(UM_ITEMS[s_cur].pid, &lo, &hi, &st)))
        {
            float v = *p + ((dir > 0) ? st : -st);

            if (v < lo) { v = lo; }
            if (v > hi) { v = hi; }
            *p = v;
        }

        if ((0u != k1s) || (0u != k2s)) { s_mode = UM_LIST; um_draw_list(); return; }

        um_draw_edit();
        return;
    }

    /* ---------- LIST ---------- */
    if (0 != dir)
    {
        /* ⚠ 方向符号：屏转 180°（UI_ROT180=1）后，屏幕上"往下"= 面板坐标的"往上"。
           如果现场发现「按往上那颗键，光标反而往下走」，把 UM_DIR_SIGN 改成 -1 即可
           —— **只改这一处**，不用去动 K3/K4 的接线或 KEY_LIST 顺序。 */
        s_cur = (uint8)((s_cur + UM_NITEMS + ((dir > 0) ? UM_DIR_SIGN : -UM_DIR_SIGN)) % UM_NITEMS);
        um_draw_list();
        return;
    }

    if (0u != k1s)
    {
        const um_item_t *it = &UM_ITEMS[s_cur];

        s_msg[0] = '\0';

        if (1u == it->kind) { s_mode = UM_EDIT; um_draw_edit(); return; }
        if (0 != it->act)   { it->act(); }
        um_draw_list();
        return;
    }

    if (0u != k2s) { s_mode = UM_DASH; ui_page_mark_dirty(&s_dpage); return; }

    /* 跑班进行中：每 200 ms 只刷标题行（1 行，SPI 开销极小）*/
    if ((SB_RUN == spd_bench_state()) || (SB_WAITLOG == spd_bench_state()))
    {
        if ((uint32)(now - s_stlast) >= 200u)
        {
            char line[32];

            s_stlast = now;
            snprintf(line, sizeof(line), "%s seg%u %u/%u",
                     um_bench_state_str(), (unsigned)spd_bench_seg(),
                     (unsigned)(spd_bench_step() + 1u), (unsigned)spd_bench_nsteps());
            um_puts(UM_ROW_TITLE, 0u, line);
        }
    }
}
