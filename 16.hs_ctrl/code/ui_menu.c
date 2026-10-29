/*********************************************************************************************************************
 * 屏幕菜单：IPS200 竖屏 240x320 + 主板 4 键
 *
 * ---- 键位 ----
 *      K1 = 确认/进编辑     K2 = 返回/退编辑
 *      K3 = 光标下移        K4 = 光标上移（长按每 100 ms 走一步）
 *      屏转 180° 后方向反了只改 UM_DIR_SIGN，不动接线与 KEY_LIST 顺序
 *
 * ---- 页面 ----   LIST 13 项只在那拍重画 / EDIT 按步长加减 K1 生效 / DASH 逐字段比缓存
 *
 *  车端 MOTOR_CMD_SAVE 只回 ACK、不写 Flash；参数须按 11.Save Par 才落 Work Flash 页 0。
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
#include "ins_route.h"
#include "route_app.h"
#include "route_pb.h"

/* ---- 一、页面模式 ---- */
#define UM_LIST         (0u)
#define UM_EDIT         (1u)
#define UM_DASH         (2u)

#define UM_ROW_TITLE    (0u)
#define UM_ROW_ITEM0    (1u)
#define UM_ROW_HINT     (17u)       /* 固定最后一行 */
#define UM_HINT         "K1=OK K2=Bk K3/K4=Move"

/* 列表光标方向符号：屏转 180° 后按下移键光标向上走；方向反了只改这里 ±1 */
#define UM_DIR_SIGN     (+1)

/* ---- 二、菜单项 ----
 *      ACT 执行动作 / PF 编辑参数表某项（量程与步长取自参数表）/ MODE 跟随模式 1=PUSH 2=PWR
 *      RPS 跟随基础速度，rps 口径，内部写 pb_vmax = rps × ins_m_per_rev
 */
#define UM_ACT      (0u)
#define UM_PF       (1u)
#define UM_MODE     (2u)
#define UM_RPS      (3u)
#define UM_SLOT     (4u)    /* 路表槽号 0~4，选完立即载入 RAM */
#define UM_TEL      (5u)    /* 遥测档案 LITE(0) / FULL(2) / TRAIN(3)，写 tel_sel */
#define UM_ROUTE    (7u)
static const char * const UM_ROUTE_NAMES[] = { "zz", "zf", "fz", "mix_s", "mix_e" };

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
    /* 选槽：K3/K4 一按即生效，先校验再选 active 并载入 RAM；空槽 / 跟随中 / 录制中 / 落盘中一律拒绝 */
    { "6.Slot",        UM_SLOT,  0,           0u  },
    { "7.Follow",      UM_MODE,  0,           0u  },   /* pb_mode：1=PUSH 2=PWR */
    { "8.Speed",       UM_RPS,   0,           0u  },   /* 0.8/1.2/2.4/3.0 rps 四档 */
    { "9.TelSel",      UM_TEL,   0,           0u  },   /* LITE 22ch / FULL 45ch / TRAIN 16ch */
    /* 10.RouteDir：路表方向档位，写 ID88，可存 Flash */
    { "10.RouteDir",   UM_ROUTE, 0,           88u },
    { "11.Save Par",   UM_ACT,   act_psave,   0u  },
    { "12.Reset",      UM_ACT,   act_reset,   0u  },
    { "13.Auto",       UM_PF,    0,           87u },
    /* 自动选控制律：0 = 按 2.CtrlMode 手选，1 = 按录制方向与回放方向自动推荐并生效 */
};

#define UM_NITEMS   ((uint8)(sizeof(UM_ITEMS) / sizeof(UM_ITEMS[0])))

/* 8.Speed 档位表：用户口径 rps，车端 pb_vmax 是 m/s，这里乘 ins_m_per_rev 换算 */
static const float UM_PWR_GEAR[] = { 0.8f, 1.2f, 2.4f, 3.6f };
#define UM_PWR_NGEAR    ((uint8)(sizeof(UM_PWR_GEAR) / sizeof(UM_PWR_GEAR[0])))

/* 遥测档案三档：存 tel_sel 原值 0 = 巡线 22ch / 2 = 全量 45ch / 3 = 采集 16ch；
   标定档 1（18ch）不占菜单位，上位机 ID32 直接写 1；顺序即 K4 方向 */
static const float UM_TEL_CHOICE[] = { 0.0f, 2.0f, 3.0f };
static const char * const UM_TEL_NAMES[] = { "LITE", "FULL", "TRAIN" };
static const uint8 UM_TEL_CH[] = { 22u, 45u, 16u };      /* 各档实际通道数（见 motor_link.h）*/
#define UM_TEL_N    ((uint8)(sizeof(UM_TEL_CHOICE) / sizeof(UM_TEL_CHOICE[0])))

/* 在候选值里找最近的序号：外部写入的怪值吸附到档位，二值项与 8.Speed 共用 */
static uint8 um_nearest (float v, const float *arr, uint8 n)
{
    uint8 best = 0u, i;
    float bd = 1.0e9f;

    for (i = 0u; i < n; i ++)
    {
        float d = v - arr[i];

        if (d < 0.0f) { d = -d; }
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

static uint8 um_gear_of (float rps)
{
    uint8 g;
    uint8 best = 0u;
    float bd   = 1.0e9f;

    for (g = 0u; g < UM_PWR_NGEAR; g ++)
    {
        float d = rps - UM_PWR_GEAR[g];

        if (d < 0.0f) { d = -d; }
        if (d < bd) { bd = d; best = g; }
    }
    return best;
}

/* 提示行：列表标题右侧，um_write() 也往这里写拒跑原因 */
static char   s_msg[24] = "";
/* 编辑页只在值变化时重画，避免每 10 ms 全屏刷新 */
static uint8  s_edit_dirty = 0u;
static float  s_edit_v     = 0.0f;

/* ---- 三、参数表查找：量程与指针都取自协议层元数据 proto_param_by_id() ---- */
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
        case UM_ROUTE:
            *v = (rt_route_mode >= 0.0f) ? rt_route_mode :
                 ((rt_revplay < 0.5f) ? (float)RT_MODE_ZZ :
                  ((rt_revrec < 0.5f) ? (float)RT_MODE_ZF : (float)RT_MODE_FZ));
            *lo = 0.0f;
            *hi = (float)(RT_MODE_COUNT - 1u);
            *st = 1.0f;
            return 1u;
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

        case UM_SLOT:                       /* 槽号：读当前 active，写见 um_write */
            *v  = (float)ins_route_slot_active();
            *lo = 0.0f;
            *hi = (float)(INS_RT_SLOT_COUNT - 1u);
            *st = 1.0f;
            return 1u;

        case UM_TEL:                        /* 三档遥测档案：LITE(0) / FULL(2) / TRAIN(3) */
        {
            *v  = UM_TEL_CHOICE[um_nearest(spd_tel_sel, UM_TEL_CHOICE, UM_TEL_N)];
            *lo = UM_TEL_CHOICE[0];
            *hi = UM_TEL_CHOICE[UM_TEL_N - 1u];
            *st = 0.0f;                     /* 记号：不是等差数列，不用 +=st */
            return 1u;
        }

        case UM_RPS:                        /* 显示 rps 并按 3 档吸附，存 m/s */
        {
            float mpr = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
            uint8 g   = um_gear_of(pb_vmax / mpr);

            *v  = UM_PWR_GEAR[g];
            *lo = UM_PWR_GEAR[0];
            *hi = UM_PWR_GEAR[UM_PWR_NGEAR - 1u];
            *st = 0.0f;                     /* 记号：按键处理按档位加减，不用 st */
            return 1u;
        }

        default:
            return 0u;
    }
}

/* ---- 菜单要写的参数 ID：与 param_table.h 对齐，ID 是协议，不许插队 ---- */
#define UM_ID_STEER_EN  (23u)
#define UM_ID_RT_REC    (38u)
#define UM_ID_PB_EN     (43u)
#define UM_ID_PB_MODE   (44u)
#define UM_ID_TEL_SEL   (32u)   /* 标定|遥测档案 0巡1标2全3采，菜单只暴露 0/2/3 */
#define UM_ID_PB_VMAX   (45u)
#define UM_ID_PB_CTRL   (78u)   /* 回放|控制律 0..5 */

/* 令牌持有者 -> 三个字母，屏幕上只放得下这么点 */
static const char * um_src_name (uint8 src)
{
    switch (src)
    {
        case PROTO_SRC_RC:       return "RC";
        case PROTO_SRC_SPI:      return "PC";
        case PROTO_SRC_MENU:     return "MENU";
        case PROTO_SRC_INTERNAL: return "INT";
        default:                 return "-";
    }
}

/* 已使能但回放没跑起来：pb_en=1 且 route_pb 不在 RUN（没表 / odom 关 / 起跑失败），供列表显示 WAIT */
static uint8 um_pb_wait (void)
{
    return ((pb_en > 0.5f) && (0u == route_pb_active())) ? 1u : 0u;
}

/* ---- 统一写入口：菜单改参数一律走协议层，权限 / 钳位 / 驱动令牌都在那里，被拒时写提示行 ---- */
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
        /* 被拒时带上错误码与令牌持有者：RC 遥控即驾驶员 / PC 上位机在写 / MENU 自己上次租期未走完 */
        snprintf(s_msg, sizeof(s_msg), "REFUSE %s by %s",
                 proto_err_name(e), um_src_name(proto_drv_owner()));
    }
    return e;
}

static void um_write(uint8 idx, float v)
{
    const um_item_t *it = &UM_ITEMS[idx];

    switch (it->kind)
    {
        case UM_ROUTE:
            (void)um_set(it->pid, v);
            break;
        case UM_PF:
            (void)um_set(it->pid, v);
            break;

        case UM_MODE:
            /* 跟随开关唯一入口：0 = 停（pb_en=0）/ 1 = 无动力人推车 / 2 = 带速度；
               先写 pb_mode 再写 pb_en，否则 pb_en 的 0->1 沿会让回放按旧模式起跑一拍 */
            if (v < 0.5f)
            {
                (void)um_set(UM_ID_PB_EN, 0.0f);
            }
            else if (route_app_count() < (uint16)PB_MIN_POINTS)
            {
                /* 没有表时把使能按回 0 并说明，避免看起来像按键失灵 */
                (void)um_set(UM_ID_PB_EN, 0.0f);
                snprintf(s_msg, sizeof(s_msg), "REFUSE: no table");
            }
            else
            {
                /* 模式写失败就停，避免只写进一半 */
                if (PROTO_OK == um_set(UM_ID_PB_MODE, (v > 1.5f) ? 2.0f : 1.0f))
                {
                    route_pb_rearm();           /* 允许重试，起跑失败后 s_arm 会锁住 */
                    /* 跟随要求舵机处于位控状态；关闭跟随时不撤销内环，保持当前位置比松手安全 */
                    (void)um_set(UM_ID_STEER_EN, 1.0f);
                    (void)um_set(UM_ID_PB_EN, 1.0f);

                    /* 使能已写入但回放起不来时，贴出 route_pb 的原因 */
                    if (0u != um_pb_wait())
                    {
                        snprintf(s_msg, sizeof(s_msg), "WAIT: %s", route_pb_reason());
                    }
                }
            }
            break;

        case UM_SLOT:
        {
            uint8                 sl   = (uint8)(v + 0.5f);
            uint8                 prev = ins_route_slot_active();
            ins_route_slot_info_t inf;

            /* ---- 三种状态不许换表：跟随在跑 / 正在录制（RAM 表被覆盖）/ 正在落盘 ---- */
            if ((pb_en > 0.5f) || (0u != route_pb_active()))
            {
                snprintf(s_msg, sizeof(s_msg), "REFUSE: follow ON");
                break;
            }
            if (0u != ins_route_is_running())
            {
                snprintf(s_msg, sizeof(s_msg), "REFUSE: recording");
                break;
            }
            if (0u != ins_route_flash_busy())
            {
                snprintf(s_msg, sizeof(s_msg), "REFUSE: flash busy");
                break;
            }
            if (sl >= (uint8)INS_RT_SLOT_COUNT) { break; }
            if (sl == prev)
            {
                snprintf(s_msg, sizeof(s_msg), "S%u already", (unsigned)sl);
                break;
            }

            /* 空槽拒绝，否则 active_slot 会被写坏，切过去只会在 7.Follow 报 no table */
            if ((0u == ins_route_slot_info(sl, &inf)) || (0u == inf.valid))
            {
                snprintf(s_msg, sizeof(s_msg), "S%u EMPTY", (unsigned)sl);
                break;
            }

            if (0u == ins_route_slot_select(sl, 1u))
            {
                /* 索引有表但页读不出（半写 / 掉电）时回滚，不留空槽 */
                snprintf(s_msg, sizeof(s_msg), "S%u LOAD FAIL", (unsigned)sl);
                break;
            }
            snprintf(s_msg, sizeof(s_msg), "S%u loaded %upt",
                     (unsigned)sl, (unsigned)inf.count);
            break;
        }

        case UM_TEL:
            /* 只改发几路、不产生运动量，允许边跑边切（P_RW_LIVE）；帧长变化由上位机按帧长自识别 */
            (void)um_set(UM_ID_TEL_SEL, v);
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

/* ---- 四、状态与仪表盘字段表 ---- */
static uint8  s_mode   = UM_LIST;
static uint8  s_cur    = 0u;
static uint8  s_rep3   = 0u;
static uint8  s_rep4   = 0u;
static uint8  s_top    = 0u;
static uint32 s_last   = 0u;
static uint32 s_stlast = 0u;
static uint8  s_savlast= 0u;

static void dash_fmt(char *buf, uint8 size, void *ctx);
static const uint8 s_dix[10] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u };
static ui_field_t  s_df[10];
static ui_page_t   s_dpage;

/* ---- 五、动作 ---- */
static void act_dash(void)
{
    s_mode = UM_DASH;
    ui_page_mark_dirty(&s_dpage);
}

/* 控制律选择器：每按一次往下走 0->1->2->0，写 ID78（P_RW_LIVE，可存 Flash），排障用 */
static const char *um_ctrl_name(uint8 m)
{
    switch (m)
    {
        case PB_CTRL_NOPP:    return "0:NOPP(Polaris)";
        case PB_CTRL_PP:      return "1:PurePursuit";
        case PB_CTRL_LQR:     return "2:LQR";
        default:              return "?";
    }
}

static void act_mode(void)
{
    uint8 cur = (uint8)(pb_ctrlmode + 0.5f);
    uint8 nxt;

    if (cur > PB_CTRL_MAX) { cur = PB_CTRL_PP; }

    nxt = (uint8)(cur + 1u);
    if (nxt > PB_CTRL_MAX) { nxt = 0u; }

    if (PROTO_OK != um_set(UM_ID_PB_CTRL, (float)nxt)) { return; }   /* 失败时 um_set 已经写好原因 */

    /* 换档只在下一次起跑生效，正在跑时不切控制律 */
    if (pb_en >= 0.5f)
    {
        snprintf(s_msg, sizeof(s_msg), "%s (next run)", um_ctrl_name(nxt));
    }
    else
    {
        snprintf(s_msg, sizeof(s_msg), "%s", um_ctrl_name(nxt));
    }
}

/* 表录制：按一下开始（写槽 0），再按一下停止；停录只是请求保存，主循环逐页写约 1 s，
   期间断电会写半张表，故先给 SAV..，写完由 ui_menu_task 刷成 SAVE ok n=... */
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

/* 表状态快照：槽 / 点数 / REC-SAV-ok / rc；rc 1=写完 2=读回校验失败 3=没东西可写 4=槽越界 */
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

/* 参数恢复默认，只写 RAM；要留住须再按 11.Save Par */
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

/* ---- 六、绘制 ---- */
/* 安全写串：先过 ui_fit_chars 截断，越界会触发 IPS200 的 zf_assert 即 while(true) 整机死机 */
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
        case UM_ROUTE:
            snprintf(out, size, "%s", UM_ROUTE_NAMES[(uint8)(v + 0.5f) % RT_MODE_COUNT]);
            break;
        case UM_MODE:
            if (0u != um_pb_wait()) { snprintf(out, size, "WAIT"); break; }   /* 开了但没跑起来 */
            snprintf(out, size, "%s", (v < 0.5f) ? "OFF" : ((v > 1.5f) ? "PWR" : "PUSH"));
            break;

        case UM_TEL:
            /* 显示档位名（ASCII，屏幕字体无中文），通道数留给编辑页 */
            snprintf(out, size, "%s", UM_TEL_NAMES[um_nearest(v, UM_TEL_CHOICE, UM_TEL_N)]);
            break;

        case UM_RPS:
            ui_ftoa(t, sizeof(t), v, 1u);
            snprintf(out, size, "%s rps", t);
            break;

        case UM_SLOT:
        {
            /* 点数必须显示：选槽看的就是哪一槽有表、表多长 */
            uint8                 sl = (uint8)(v + 0.5f);
            ins_route_slot_info_t inf;

            if (0u != ins_route_flash_busy()) { snprintf(out, size, "S%u SAV", (unsigned)sl); break; }
            if ((0u == ins_route_slot_info(sl, &inf)) || (0u == inf.valid))
            {
                snprintf(out, size, "S%u empty", (unsigned)sl);
                break;
            }
            snprintf(out, size, "S%u %upt", (unsigned)sl, (unsigned)inf.count);
            break;
        }

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

    /* 按 kind 分派第 3 行：跟随、方向、速度、档案、槽号各有专用格式，其余走通用 */
    if (UM_MODE == it->kind)
    {
        if (0u != um_pb_wait())
        {
            /* 已使能但没跑：贴出 route_pb 的原因（no table / odom off 等） */
            snprintf(line, sizeof(line), "= WAIT: %s", route_pb_reason());
            um_puts(3u, 0u, line);
        }
        else
        {
            /* OFF 时也贴出上一次的退出原因（done / ESTOP / RC ON 等），区分跑完了还是被挡了 */
            if (v < 0.5f)
            {
                const char *rr = route_pb_reason();

                if ((NULL != rr) && (0 != rr[0]))
                {
                    snprintf(line, sizeof(line), "= OFF: %.13s", rr);
                    um_puts(3u, 0u, line);
                }
                else
                {
                    um_puts(3u, 0u, "= OFF  (pb_en=0)");
                }
            }
            else
            {
                um_puts(3u, 0u, (v > 1.5f) ? "= PWR  (steer+speed)" : "= PUSH (steer only)");
            }
        }
    }
    else if (UM_ROUTE == it->kind)
    {
        snprintf(line, sizeof(line), "= %s", UM_ROUTE_NAMES[(uint8)(v + 0.5f) % RT_MODE_COUNT]);
        um_puts(3u, 0u, line);
    }
    else if (UM_RPS == it->kind)
    {
        ui_ftoa(t, sizeof(t), v, 2u);
        snprintf(line, sizeof(line), "= %s rps", t);
        um_puts(3u, 0u, line);
    }
    else if (UM_TEL == it->kind)
    {
        uint8 g = um_nearest(v, UM_TEL_CHOICE, UM_TEL_N);

        snprintf(line, sizeof(line), "= %s  %uch", UM_TEL_NAMES[g], (unsigned)UM_TEL_CH[g]);
        um_puts(3u, 0u, line);
    }
    else if (UM_SLOT == it->kind)
    {
        snprintf(line, sizeof(line), "= S%u (now S%u)",
                 (unsigned)(v + 0.5f), (unsigned)ins_route_slot_active());
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
        /* 诊断三行：en = pb_en 实际值；st = 回放状态 0 空闲 1 跑 2 完成 3 中止；
           rc = RC_Engage 0 OFF / 1 STANDBY / 2 ON / 3 FAILSAFE / 4 ESTOP，接收机没插时恒为 3 */
        snprintf(line, sizeof(line), "en=%u st=%u rc=%u",
                 (unsigned)((pb_en > 0.5f) ? 1u : 0u), (unsigned)route_pb_state(),
                 (unsigned)RC_Engage);
        um_puts(4u, 0u, line);

        /* 第 5 行：进度 idx/n 与横向误差（e 的单位 0.01 m）与自由轮标志 c */
        snprintf(line, sizeof(line), "i%u/%u e%d c%u",
                 (unsigned)route_pb_index(), (unsigned)ins_route_get_count(),
                 (int)(route_pb_elat() * 100.0f), (unsigned)spd_ctrl_coast());
        um_puts(5u, 0u, line);

        /* 这里只放 ASCII：屏幕字体没有中文，%.18s 还会从半个汉字处截断成两行 */
        snprintf(line, sizeof(line), "%.18s", route_pb_reason());
        um_puts(6u, 0u, line);
    }
    else if (UM_SLOT == it->kind)
    {
        /* 显示该槽有没有表、多少点、多长、crc，避免到 7.Follow 才报 no table */
        ins_route_slot_info_t inf;
        uint8                 sl = (uint8)(v + 0.5f);

        if ((0u != ins_route_slot_info(sl, &inf)) && (0u != inf.valid))
        {
            snprintf(line, sizeof(line), "n=%u  %u.%02u m",
                     (unsigned)inf.count,
                     (unsigned)(inf.lap_mm / 1000u),
                     (unsigned)((inf.lap_mm % 1000u) / 10u));
            um_puts(4u, 0u, line);
            snprintf(line, sizeof(line), "ds=%u mm", (unsigned)inf.ds_mm);
            um_puts(5u, 0u, line);
            snprintf(line, sizeof(line), "crc %04X", (unsigned)(inf.crc_r & 0xFFFFu));
            um_puts(6u, 0u, line);
        }
        else
        {
            um_puts(4u, 0u, "EMPTY - no table");
            um_puts(5u, 0u, "");
            um_puts(6u, 0u, "");
        }
    }
    else if (UM_TEL == it->kind)
    {
        /* 三档各自发什么直接列出来；只放 ASCII */
        um_puts(4u, 0u, "lite  22ch 5ms");
        um_puts(5u, 0u, "full  45ch 5ms");
        um_puts(6u, 0u, "train 16ch 5ms");
    }
    else if (UM_ROUTE == it->kind)
    {
        um_puts(4u, 0u, "zz / zf / fz");
        um_puts(5u, 0u, "mix_s / mix_e");
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

    /* 拒跑原因（REFUSE busy / no table 等）也留在编辑页 */
    if (s_msg[0] != '\0') { um_puts(7u, 0u, s_msg); }

    if (UM_SLOT == it->kind)
    {
        um_puts(8u,  0u, "K3/K4 = slot   K1 = back");
        um_puts(9u,  0u, "(-/+ applies now)");
    }
    else
    {
        um_puts(8u,  0u, "K3/K4 = -/+");
        um_puts(9u,  0u, "K1 = OK  K2 = Back");
    }
}

/* ---- 七、仪表盘字段 ---- */
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
            /* 跟随状态：使能、回放状态、段索引 */
            snprintf(buf, size, "pb en%u %s s%u",
                     (unsigned)(pb_en > 0.5f),
                     (1u == route_pb_state()) ? "RUN" :
                     ((2u == route_pb_state()) ? "DONE" :
                      ((3u == route_pb_state()) ? "ABRT" : "idle")),
                     (unsigned)route_pb_index());
            break;

        case 5u:
            /* drv = 驱动令牌持有者：0 无 / 1 遥控 / 2 上位机 / 3 菜单 */
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
            /* 通道核对行：CH1 / CH2 / CH3 原始值，末尾 ! = 关键通道 raw 低于有效下限（掉线）；
               CH1 为 0 表示该通道无信号，不能归一成 -1 当满舵；CH3 已空置 */
            if (0u != RC_Online)
            {
                snprintf(buf, size, "RC %u/%u/%u%s",
                         (unsigned)RC_Raw_Ch1, (unsigned)RC_Raw_Ch2,
                         (unsigned)RC_Raw_Ch3,
                         (0u != RC_Stick_Bad) ? "!" : "");
            }
            else
            {
                snprintf(buf, size, "imu %u fl%u", (unsigned)ins_imu_link_ok(),
                         (unsigned)ins_odom_flags());
            }
            break;
    }
}

/* ---- 八、初始化 ---- */
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

/* ---- 九、主循环任务 ---- */
uint8 ui_menu_in_list(void) { return (UM_LIST == s_mode) ? 1u : 0u; }

void ui_menu_task(void)
{
    uint32 now = motor_link_now_ms();
    uint8  k1s, k2s, k3s, k4s, h3, h4;
    int8   dir = 0;

    if ((uint32)(now - s_last) < 10u) { return; }       /* 10 ms 分频 */
    s_last = now;

    /* 落盘结束那一拍自动更新提示行 */
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

        if ((0 != dir) && (0u != has) && (UM_ROUTE == UM_ITEMS[s_cur].kind))
        {
            uint8 m = (uint8)(v + 0.5f);
            m = (uint8)((m + ((dir > 0) ? 1u : RT_MODE_COUNT - 1u)) % RT_MODE_COUNT);
            um_write(s_cur, (float)m);
            has = um_read(s_cur, &v, &lo, &hi, &st);
        }
        else if ((0 != dir) && (0u != has) && (UM_RPS == UM_ITEMS[s_cur].kind))
        {
            /* 3 档循环：K3 慢一档、K4 快一档，首尾相接 */
            uint8 g = um_gear_of(v);

            g = (uint8)((dir > 0) ? ((g + 1u) % UM_PWR_NGEAR)
                                  : ((g + (UM_PWR_NGEAR - 1u)) % UM_PWR_NGEAR));
            um_write(s_cur, UM_PWR_GEAR[g]);
            has = um_read(s_cur, &v, &lo, &hi, &st);
        }
        else if ((0 != dir) && (0u != has) && (UM_TEL == UM_ITEMS[s_cur].kind))
        {
            /* 三档遥测档案一按即写，不用再按 K1，首尾相接，序号取自当前实际值 */
            uint8 g = um_nearest(v, UM_TEL_CHOICE, UM_TEL_N);

            g = (uint8)((dir > 0) ? ((g + 1u) % UM_TEL_N) : ((g + (UM_TEL_N - 1u)) % UM_TEL_N));
            um_write(s_cur, UM_TEL_CHOICE[g]);
            has = um_read(s_cur, &v, &lo, &hi, &st);

            snprintf(s_msg, sizeof(s_msg), "tel %s %uch",
                     UM_TEL_NAMES[g], (unsigned)UM_TEL_CH[g]);
        }
        else if ((0 != dir) && (0u != has))
        {
            v += ((dir > 0) ? st : -st);

            /* 按步长吸附，避免 0.1 反复加减后退化成 1.1999999；统一 +0.5 再取整 */
            if (st > 1.0e-6f)
            {
                v = lo + ((float)(int)(((v - lo) / st) + 0.5f)) * st;
            }
            if (v < lo) { v = lo; }
            if (v > hi) { v = hi; }
            um_write(s_cur, v);

            has = um_read(s_cur, &v, &lo, &hi, &st);    /* 以真正生效的值重读 */
        }

        if ((0u != k1s) || (0u != k2s)) { s_mode = UM_LIST; um_draw_list(); return; }

        /* 值真的变了才重画，含被拒后 pb_en 被按回 0 */
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
