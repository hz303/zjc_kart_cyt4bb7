/*********************************************************************************************************************
 * 工程 11（卡丁）· 屏幕菜单（IPS200 竖屏 + 主板 4 键）
 *
 * ============================  2026-10-20 重排：只留 9 项  ============================
 *   现场翻页翻不完，菜单从 23 项收敛到 9 项，每一项都是"跑车那天真会按"的：
 *
 *     1. Dashboard   仪表盘（车速/航向/位姿/安全/遥控，K1 进 K2 出）
 *     2. CtrlMode    控制模式（**预留**：当前只有手动，占位显示）
 *     3. Rec/Save    表录制 / 停录自动落盘（按一下开、再按一下停）
 *     4. Rt ds       采样点距 0.05~0.15 m，步长 **0.01**
 *     5. Rt Status   表状态：槽 / 点数 / REC-SAV-ok / 上次返回码
 *     6. Follow      跟随模式：1=PUSH（只舵机、后轮不给力，人推车）2=PWR（带速度）
 *     7. Speed       跟随基础速度 **0.8~3.0 rps**，步长 0.1（内部写 pb_vmax = rps×mpm）
 *     8. Save Par    参数写 Flash（不按这一下，上电全回源码默认）
 *     9. Reset       参数恢复默认（**只写 RAM**，要留就再按 8）
 *
 *   离开菜单的东西并没有消失 —— 跑班 / 开环标定 / k_v / k_z / Umax / 打角前馈
 *   全部还在**上位机 + 手机 App** 上，那些地方有一屏能看全的表；屏幕只有 4 个键，
 *   不该承担"调 60 个参数"的活。
 *
 * ---- 键位（与工程 8/10 一致）--------------------------------------------------------
 *      K1 = 确认/进编辑       K2 = 返回/退编辑
 *      K3 = 光标下移          K4 = 光标上移（长按每 100 ms 走一步）
 *      ⚠ 屏装反 180° 时若发现"方向反了"，只改 `UM_DIR_SIGN`，别动接线。
 *
 * ---- 页面 ------------------------------------------------------------------------------
 *      LIST  9 项（只有按键那一拍重画 ⇒ 空闲屏幕 SPI 全静默）
 *      EDIT  改一项（K3/K4 按该项自己的步长加减，K1 生效返回）
 *      DASH  仪表盘（ui_core 字段引擎，逐字段比对缓存，变了才画）
 *
 * ---- ★ 为什么 Save Par 这么重要 -------------------------------------------------------
 *  车端 `MOTOR_CMD_SAVE` **只回 ACK、不写 Flash**。没有这一项，每次上电
 *  `k_v/k_z/ins_mpm/转向限位/跟随速度` 全回源码默认值。
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
#include "proto.h"
#include "ins_route.h"          /* 路表：点数/槽号/点距 */
#include "route_app.h"          /* 路表：录制/选槽命令量 */
#include "route_pb.h"           /* 路表：回放使能 / pb_mode / pb_vmax */

/*===================================================================================================================
 *  一、页面模式
 *=================================================================================================================*/
#define UM_LIST         (0u)
#define UM_EDIT         (1u)
#define UM_DASH         (2u)

#define UM_ROW_TITLE    (0u)
#define UM_ROW_ITEM0    (1u)
#define UM_ROW_HINT     (17u)       /* 固定最后一行 */
#define UM_HINT         "K1=OK K2=Bk K3/K4=Move"

/* ★ 列表光标方向符号（屏转 180° 后"往下"= 面板坐标"往上"）。
 *   现场发现方向反了只改这里 +1 ⇄ -1，不要动接线与 KEY_LIST 顺序。 */
#define UM_DIR_SIGN     (+1)

/*===================================================================================================================
 *  二、菜单项
 *      ACT  = 执行动作
 *      PF   = 直接编辑参数表的某一项（量程/步长**取自车端参数表**，不在菜单里重写）
 *      MODE = 跟随模式（1/2，写 pb_mode，显示成 PUSH/PWR）
 *      RPS  = 跟随基础速度（**rps** 口径，内部写 pb_vmax = rps × ins_m_per_rev）
 *=================================================================================================================*/
#define UM_ACT      (0u)
#define UM_PF       (1u)
#define UM_MODE     (2u)
#define UM_RPS      (3u)

typedef struct
{
    const char *name;
    uint8       kind;
    void      (*act)(void);
    uint8       pid;                /* PF：参数表 ID */
} um_item_t;

static void act_dash  (void);
static void act_mode  (void);
static void act_rtrec (void);
static void act_sts   (void);
static void act_psave (void);
static void act_reset (void);

static const um_item_t UM_ITEMS[] =
{
    /* name            kind      action       pid */
    { "1.Dashboard",   UM_ACT,   act_dash,    0u  },
    { "2.CtrlMode",    UM_ACT,   act_mode,    0u  },   /* 预留 */
    { "3.Rec/Save",    UM_ACT,   act_rtrec,   0u  },
    { "4.Rt ds",       UM_PF,    0,           39u },   /* 路表|采样点距（0.05~0.15，步长 0.01）*/
    { "5.Rt Status",   UM_ACT,   act_sts,     0u  },
    { "6.Follow",      UM_MODE,  0,           0u  },   /* pb_mode：1=PUSH 2=PWR */
    { "7.Speed",       UM_RPS,   0,           0u  },   /* 0.8~3.0 rps，步长 0.1 */
    { "8.Save Par",    UM_ACT,   act_psave,   0u  },   /* ★ 写 Flash 页 0 */
    { "9.Reset",       UM_ACT,   act_reset,   0u  },   /* 参数恢复默认（只写 RAM）*/
};

#define UM_NITEMS   ((uint8)(sizeof(UM_ITEMS) / sizeof(UM_ITEMS[0])))

/* ---- 第 7 项的取值范围（**rps 口径**）------------------------------------------------
 *  用户口径是 rps，而 pb_vmax 在车端是 m/s ⇒ 菜单这边换算，参数表不动。
 *  ⚠ 换轮子重标 ins_mpm 后这里自动跟随（系数取自 ins_m_per_rev）。 */
#define UM_RPS_MIN      (0.8f)
#define UM_RPS_MAX      (3.0f)
#define UM_RPS_STEP     (0.1f)

/* 提示行（列表标题右侧那一行）。放在这里是因为 um_write() 也要往它里面写"拒跑原因"。 */
static char   s_msg[24] = "";
/* ★ 编辑页只在"值变了"时重画。
   原来是无条件每 10 ms ui_clear() + 全屏重画 = 100 Hz 刷屏 ⇒ 现场看到"页面一直在闪"，
   而且整屏 SPI 传输还把主循环拖慢（route_pb_poll 也跟着抖）。 */
static uint8  s_edit_dirty = 0u;
static float  s_edit_v     = 0.0f;

/*===================================================================================================================
 *  三、参数表查找（量程/指针都从协议层元数据取）
 *
 *  ★ 2026-10-20：这里原来又展开了一遍 `MOTOR_PARAM_TABLE`（第三份平行表）。
 *    现在只有 `param_table.h` 一份，元数据由 `proto_param_by_id()` 提供 ——
 *    加字段时不会再出现"这个文件漏改了、编不过"（更糟的是顺序错了却编得过）。
 *=================================================================================================================*/
static volatile float *um_ptr(uint8 id)
{
    const proto_param_t *p = proto_param_by_id(id);

    return (NULL == p) ? 0 : p->ptr;
}

static uint8 um_range(uint8 id, float *lo, float *hi, float *st)
{
    const proto_param_t *p = proto_param_by_id(id);

    if (NULL == p) { return 0u; }

    *lo = p->lo;
    *hi = p->hi;
    *st = p->step;
    return 1u;
}

/* ---- 统一取/存：三种 kind 都在这里落地，编辑页与列表页共用 ---------------------------- */
static uint8 um_read(uint8 idx, float *v, float *lo, float *hi, float *st)
{
    const um_item_t *it = &UM_ITEMS[idx];

    switch (it->kind)
    {
        case UM_PF:
        {
            volatile float *p = um_ptr(it->pid);

            if ((0 == p) || (0u == um_range(it->pid, lo, hi, st))) { return 0u; }
            *v = *p;
            return 1u;
        }

        case UM_MODE:                       /* 三态：0=OFF 1=PUSH 2=PWR */
            *v  = (pb_en < 0.5f) ? 0.0f : ((pb_mode > 1.5f) ? 2.0f : 1.0f);
            *lo = 0.0f;
            *hi = 2.0f;
            *st = 1.0f;
            return 1u;

        case UM_RPS:                        /* 显示 rps，存 m/s */
        {
            float mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;

            *v  = pb_vmax / mpr;
            *lo = UM_RPS_MIN;
            *hi = UM_RPS_MAX;
            *st = UM_RPS_STEP;
            return 1u;
        }

        default:
            return 0u;
    }
}

/* ---- 菜单要写的参数 ID（**与 param_table.h 对齐**；ID 是协议，不许插队）---- */
#define UM_ID_STEER_EN  (23u)
#define UM_ID_RT_REC    (38u)
#define UM_ID_PB_EN     (43u)
#define UM_ID_PB_MODE   (44u)
#define UM_ID_PB_VMAX   (45u)

/* ---- 统一写入口：菜单的每一次"改参数"都走协议层（权限 / 钳位 / 驱动令牌）----
 *  ⚠ 以前菜单是**直接写变量**的（`pb_en = 1`、`steer_ctrl_set_angle_enable(1)`）——
 *    于是"谁在开车"这件事在菜单这一路完全不存在：遥控开着车，屏幕照样能把回放打开。
 *    现在一律 um_set()：被拒时把原因写进提示行（现场只有 4 个键，得自己说出来）。 */
static uint8 um_set (uint8 id, float v)
{
    proto_msg_t m;
    float out = 0.0f;
    uint8 e;

    m.src   = PROTO_SRC_MENU;
    m.op    = PROTO_OP_SET;
    m.id    = id;
    m.val   = v;
    m.token = 0u;

    e = proto_exec(&m, &out);

    if (PROTO_OK != e)
    {
        snprintf(s_msg, sizeof(s_msg), "REFUSE %s", proto_err_name(e));
    }
    return e;
}

static void um_write(uint8 idx, float v)
{
    const um_item_t *it = &UM_ITEMS[idx];

    switch (it->kind)
    {
        case UM_PF:
            (void)um_set(it->pid, v);
            break;

        case UM_MODE:
            /* ★ 这一项就是"开/关跟随"的唯一入口：
               0 = 停（pb_en=0）｜1 = 无动力（pb_en=1 + pb_mode=1，人推车）｜2 = 带速度。
               ⚠ 先写 pb_mode 再写 pb_en：反过来的话，pb_en 的 0→1 沿会让回放先按旧模式起跑一拍。
               ⚠ 三项都是 P_CMD：菜单必须先拿到驱动令牌才写得进去（上位机在跑时会回 REFUSE busy）。 */
            if (v < 0.5f)
            {
                (void)um_set(UM_ID_PB_EN, 0.0f);
            }
            else if (route_app_count() < (uint16)PB_MIN_POINTS)
            {
                /* 没有表就别让它"看起来开了又自己关"：现场只会看到 pb_en 弹回 0，
                   以为是按键坏了。这里直接说清楚。 */
                (void)um_set(UM_ID_PB_EN, 0.0f);
                snprintf(s_msg, sizeof(s_msg), "REFUSE: no table");
            }
            else
            {
                /* 一步失败就停：否则会留下"模式写进去了、使能没写进去"的半套状态 */
                if (PROTO_OK == um_set(UM_ID_PB_MODE, (v > 1.5f) ? 2.0f : 1.0f))
                {
                    route_pb_rearm();           /* 允许重试：起跑失败后 s_arm 会锁住 */
                    /* ★ 同时把转向内环打开：屏幕上没有单独的"SteerEn"项了，
                       而"跟随表"天然要求舵机在位控状态（否则方向盘是自由的）。
                       关掉跟随（OFF）时**不**撤销内环 —— 保持住当前位置比松手安全。 */
                    (void)um_set(UM_ID_STEER_EN, 1.0f);
                    (void)um_set(UM_ID_PB_EN, 1.0f);
                }
            }
            break;

        case UM_RPS:
        {
            float mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;

            (void)um_set(UM_ID_PB_VMAX, v * mpr);
            break;
        }

        default:
            break;
    }
}

/*===================================================================================================================
 *  四、状态
 *=================================================================================================================*/
static uint8  s_mode   = UM_LIST;
static uint8  s_cur    = 0u;
static uint8  s_rep3   = 0u;
static uint8  s_rep4   = 0u;
static uint8  s_top    = 0u;
static uint32 s_last   = 0u;
static uint32 s_stlast = 0u;
static uint8  s_savlast= 0u;

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

/* 控制模式：**预留**。当前只有"手动"一种，显示出来占位，K1 不做事。
 * 为什么不删：现场会问"模式在哪"，留一个明确写 RESERVED 的入口，
 * 比让人以为菜单漏了要好。 */
static void act_mode(void)
{
    snprintf(s_msg, sizeof(s_msg), "ctrl: MANUAL (reserved)");
}

/* 表录制：按一下开始（写槽 0），再按一下停止。
 * ⚠ 停录只是"请求保存"：主循环一页一页写（≈1 s）。这期间 reset/断电 =
 *   只写了半张表、头页没写 ⇒ 上电载入失败 ⇒ 0 点。所以提示里给 SAV..，
 *   写完由 ui_menu_task 自动刷成 "SAVE ok n=..."。 */
static void act_rtrec(void)
{
    if (rt_rec_cmd < 0.5f)
    {
        if (PROTO_OK != um_set(UM_ID_RT_REC, 1.0f)) { return; }
        snprintf(s_msg, sizeof(s_msg), "REC start %umm",
                 (unsigned)(ins_route_ds * 1000.0f + 0.5f));
    }
    else
    {
        if (PROTO_OK != um_set(UM_ID_RT_REC, 0.0f)) { return; }
        snprintf(s_msg, sizeof(s_msg), "REC stop n=%u SAV..",
                 (unsigned)route_app_count());
    }
}

/* 表状态快照（现场没串口时唯一能看的地方）
 * rc: 1=写完 2=读回校验失败 3=没东西可写 4=槽越界；按一次是快照，看进度就等一秒再按。 */
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

static void act_psave(void)
{
    proto_msg_t m;
    float e = 0.0f;
    uint8 r;

    m.src   = PROTO_SRC_MENU;
    m.op    = PROTO_OP_SAVE;
    m.id    = 0u;
    m.val   = 0.0f;
    m.token = 0u;

    r = proto_exec(&m, &e);                  /* 真正落盘在 proto -> app_param_save() */

    if (PROTO_OK == r)
    {
        snprintf(s_msg, sizeof(s_msg), "saved to flash OK");
    }
    else if (PROTO_ERR_BUSY == r)
    {
        snprintf(s_msg, sizeof(s_msg), "BUSY: host driving");
    }
    else
    {
        snprintf(s_msg, sizeof(s_msg), "SAVE FAIL err=%u", (unsigned)(int)e);
    }
}

/* 参数恢复默认：**只写 RAM**，要留住必须再按 8.Save Par。
 * 为什么不做成"一步到位连 Flash 一起写"：reset 常常是"先看看默认值合不合适"，
 * 直接覆盖 Flash 就把现场调好的那套抹了 —— 多一步，但可后悔。 */
static void act_reset(void)
{
    proto_msg_t m;
    uint8 r;

    m.src   = PROTO_SRC_MENU;
    m.op    = PROTO_OP_LOAD_DEF;
    m.id    = 0u;
    m.val   = 0.0f;
    m.token = 0u;

    r = proto_exec(&m, NULL);

    if (PROTO_OK == r) { snprintf(s_msg, sizeof(s_msg), "defaults in RAM (press 8)"); }
    else               { snprintf(s_msg, sizeof(s_msg), "REFUSE %s", proto_err_name(r)); }
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

/* 列表右侧那一列的值：按 kind 显示成人看得懂的词，而不是裸数字 */
static void um_value_text(uint8 idx, char *out, uint8 size)
{
    float v = 0.0f, lo = 0.0f, hi = 0.0f, st = 0.0f;
    char  t[16];

    if (0u == um_read(idx, &v, &lo, &hi, &st))
    {
        out[0] = '-';
        out[1] = '\0';
        return;
    }

    switch (UM_ITEMS[idx].kind)
    {
        case UM_MODE:
            snprintf(out, size, "%s", (v < 0.5f) ? "OFF" : ((v > 1.5f) ? "PWR" : "PUSH"));
            break;

        case UM_RPS:
            ui_ftoa(t, sizeof(t), v, 1u);
            snprintf(out, size, "%s rps", t);
            break;

        default:                                /* PF：参数原始值 */
            ui_ftoa(t, sizeof(t), v, 3u);
            snprintf(out, size, "%s", t);
            break;
    }
}

static void um_draw_list(void)
{
    uint8 span = (uint8)(UM_ROW_HINT - UM_ROW_ITEM0);
    uint8 i;

    ui_clear();

    if (s_cur < s_top) { s_top = s_cur; }
    if ((uint8)(s_cur - s_top) >= span) { s_top = (uint8)(s_cur - span + 1u); }
    if (s_top > (uint8)(UM_NITEMS - 1u)) { s_top = (uint8)(UM_NITEMS - 1u); }

    um_puts(UM_ROW_TITLE, 0u, "KART MENU");

    for (i = 0u; i < span; i++)
    {
        uint8    k = (uint8)(s_top + i);
        char     line[48];
        char     val[16];
        uint8    row = (uint8)(UM_ROW_ITEM0 + i);

        if (k >= UM_NITEMS) { break; }

        um_value_text(k, val, sizeof(val));
        snprintf(line, sizeof(line), "%-14s %s", UM_ITEMS[k].name, val);
        um_puts(row, 2u, line);
    }

    um_puts(UM_ROW_HINT, 0u, UM_HINT);

    if (s_msg[0] != '\0') { um_puts(UM_ROW_TITLE, 0u, "KART  "); um_puts(UM_ROW_TITLE, 6u, s_msg); }

    um_puts((uint8)(UM_ROW_ITEM0 + (uint8)(s_cur - s_top)), 0u, ">");
}

static void um_draw_edit(void)
{
    const um_item_t *it  = &UM_ITEMS[s_cur];
    float v = 0.0f, lo = 0.0f, hi = 0.0f, st = 0.0f;
    char  t[32];
    char  line[48];

    if (0u == um_read(s_cur, &v, &lo, &hi, &st)) { s_mode = UM_LIST; return; }

    ui_clear();

    um_puts(0u, 0u,  "EDIT");
    snprintf(line, sizeof(line), "%s", it->name);
    um_puts(1u, 0u, line);

    /* 第 6/7 项显示成人话，不加单位后缀也能看懂 */
    if (UM_MODE == it->kind)
    {
        um_puts(3u, 0u, (v < 0.5f) ? "= OFF  (pb_en=0)"
                     : ((v > 1.5f) ? "= PWR  (steer+speed)" : "= PUSH (steer only)"));
    }
    else if (UM_RPS == it->kind)
    {
        ui_ftoa(t, sizeof(t), v, 2u);
        snprintf(line, sizeof(line), "= %s rps", t);
        um_puts(3u, 0u, line);
    }
    else
    {
        ui_ftoa(t, sizeof(t), v, 4u);
        snprintf(line, sizeof(line), "= %s m", t);
        um_puts(3u, 0u, line);
    }

    if (UM_MODE == it->kind)
    {
        /* ★ 这一页必须一眼看出"为什么不动"（2026-10-20 现场踩过）：
           en = pb_en 实际值；st = 回放状态(0空闲 1跑 2完成 3中止)；
           coast = 自由轮是否生效（1 = 两后轮 0 占空比，推得动）。
           step/min/max 对一个 0~2 的三态开关没有意义，直接让位给诊断。 */
        snprintf(line, sizeof(line), "en=%u st=%u",
                 (unsigned)((pb_en > 0.5f) ? 1u : 0u), (unsigned)route_pb_state());
        um_puts(4u, 0u, line);

        snprintf(line, sizeof(line), "coast=%u", (unsigned)spd_ctrl_coast());
        um_puts(5u, 0u, line);

        snprintf(line, sizeof(line), "%.18s", route_pb_reason());
        um_puts(6u, 0u, line);
    }
    else
    {
        ui_ftoa(t, sizeof(t), st, 3u);
        snprintf(line, sizeof(line), "step %s", t);
        um_puts(4u, 0u, line);

        ui_ftoa(t, sizeof(t), lo, 3u);
        snprintf(line, sizeof(line), "min  %s", t);
        um_puts(5u, 0u, line);

        ui_ftoa(t, sizeof(t), hi, 3u);
        snprintf(line, sizeof(line), "max  %s", t);
        um_puts(6u, 0u, line);
    }

    /* 拒跑原因（REFUSE busy / no table …）也留在编辑页，别让用户"按了没反应" */
    if (s_msg[0] != '\0') { um_puts(7u, 0u, s_msg); }

    um_puts(8u,  0u, "K3/K4 = -/+");
    um_puts(9u,  0u, "K1 = OK  K2 = Back");
}

/*===================================================================================================================
 *  七、仪表盘字段
 *=================================================================================================================*/
static void dash_fmt(char *buf, uint8 size, void *ctx)
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
            /* ★ 第 4 行改成"跟随状态"：现场最常问的就是"现在 PB 跑没跑、第几段" */
            snprintf(buf, size, "pb en%u %s s%u",
                     (unsigned)(pb_en > 0.5f),
                     (1u == route_pb_state()) ? "RUN" :
                     ((2u == route_pb_state()) ? "DONE" :
                      ((3u == route_pb_state()) ? "ABRT" : "idle")),
                     (unsigned)route_pb_index());
            break;

        case 5u:
            /* ★ 2026-10-20 加 drv：当前**驱动令牌**在谁手里（0 无 / 1 遥控 / 2 上位机 / 3 菜单）。
               现场"为什么滑条推不动/为什么菜单打不开跟随"第一眼就看这个数。 */
            snprintf(buf, size, "safe %u loc%u est%u drv%u",
                     (unsigned)spd_safety_allow(0u),
                     (unsigned)spd_safety_local(),
                     (unsigned)spd_safety_estop(),
                     (unsigned)proto_drv_owner());
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
            /* ★ CH3 ≤ 50% 时 CH1 在控航向 —— 这一行必须看得出来，
               否则现场"方向盘不跟手"会以为是内环坏了。
               c3 = CH3 百分比 / s = 当前步长 / o = 当前航向偏移（有符号）。 */
            if (0u != RC_Hdg_Mode)
            {
                int off = (int)(RC_Hdg_Off + ((RC_Hdg_Off >= 0.0f) ? 0.5f : -0.5f));

                snprintf(buf, size, "hdg c3=%u s%u.%u o%s%d",
                         (unsigned)RC_Ch3_Pct,
                         (unsigned)((int)(RC_Hdg_Step * 10.0f) / 10),
                         (unsigned)((int)(RC_Hdg_Step * 10.0f) % 10),
                         (off < 0) ? "-" : "+",
                         (off < 0) ? -off : off);
            }
            else
            {
                snprintf(buf, size, "imu %u fl%u", (unsigned)ins_imu_link_ok(),
                         (unsigned)ins_odom_flags());
            }
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
    s_top  = 0u;
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

    /* 落盘结束那一拍自动更新提示行 —— 现场只有屏幕时，这是唯一能确认"表真写进去了"的地方 */
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
        float v = 0.0f, lo = 0.0f, hi = 0.0f, st = 0.0f;
        uint8 has = um_read(s_cur, &v, &lo, &hi, &st);

        if ((0 != dir) && (0u != has))
        {
            v += ((dir > 0) ? st : -st);

            /* 按步长吸附：避免 0.1 反复加减后变成 1.1999999。
               ⚠ 必须**统一 +0.5 再取整**（四舍五入）。之前按方向取 ±0.5，
                 往小走时 (int)(-0.5) 朝 0 截断 ⇒ 每次少走一格，表现为"往下按不动"。 */
            if (st > 1.0e-6f)
            {
                v = lo + ((float)(int)(((v - lo) / st) + 0.5f)) * st;
            }
            if (v < lo) { v = lo; }
            if (v > hi) { v = hi; }
            um_write(s_cur, v);

            has = um_read(s_cur, &v, &lo, &hi, &st);    /* 以"真正生效的值"为准重读 */
        }

        if ((0u != k1s) || (0u != k2s)) { s_mode = UM_LIST; um_draw_list(); return; }

        /* 只在值真的变了才重画（含外部改动：被拒时 pb_en 会被按回 0） */
        if ((0u != has) && ((v > (s_edit_v + 1.0e-4f)) || (v < (s_edit_v - 1.0e-4f))))
        {
            s_edit_v     = v;
            s_edit_dirty = 1u;
        }
        if (0u != s_edit_dirty)
        {
            s_edit_dirty = 0u;
            um_draw_edit();
        }
        return;
    }

    /* ---------- LIST ---------- */
    if (0 != dir)
    {
        s_cur = (uint8)((s_cur + UM_NITEMS + ((dir > 0) ? UM_DIR_SIGN : -UM_DIR_SIGN)) % UM_NITEMS);
        um_draw_list();
        return;
    }

    if (0u != k1s)
    {
        const um_item_t *it = &UM_ITEMS[s_cur];

        s_msg[0] = '\0';

        if (UM_ACT != it->kind)
        {
            s_mode       = UM_EDIT;
            s_msg[0]     = '\0';
            s_edit_dirty = 1u;
            s_edit_v     = 1.0e30f;         /* 强制第一帧一定画 */
            um_draw_edit();
            return;
        }
        if (0 != it->act)       { it->act(); }
        um_draw_list();
        return;
    }

    if (0u != k2s) { s_mode = UM_DASH; ui_page_mark_dirty(&s_dpage); return; }

    /* 跑班进行中：每 200 ms 只刷标题行（SPI 开销极小）*/
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
