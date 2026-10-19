/*********************************************************************************************************************
 * C 车惯导 + 路表 - 路表录制实现
 * 设计意图、阈值出处、"录满不静默"的修正 见 ins_route.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stddef.h>
#include "ins_route.h"
#include "ins_odom.h"
#include "ins_imu.h"        /* ins_imu_zupt_frz_reset() —— 录表起点清冻结计数器 */
#include "steer_ctrl.h"     /* ★ steer_ctrl_get_target_delta() —— 记录"人打的舵角" */

#define DEG2RAD_F   (0.017453292519943295f)

/* ★★ 转角触发开关：**默认 0 = 只用距离**。
 *    改成 1 会让弯道额外加密 ⇒ 点距不再相等 ⇒ `route_cruise` 的"等间距里程索引"整体错位。
 *    真要开，必须同时把"每点累积里程"存进表里、并把匹配改成二分查找（见 route_cruise.h 的说明）。 */
#define INS_ROUTE_YAW_TRIG_EN   (0)

static ins_wp_t          s_wp[INS_ROUTE_MAX_POINTS];
static ins_route_head_t  s_head;

static uint8   s_running   = 0;
static uint8   s_flags     = 0;
static float   s_dist_acc  = 0.0f;      /* 距上一个已存点的累计距离 */
static float   s_yaw_last  = 0.0f;      /* 上一个已存点的相对 yaw（仅转角触发启用时参与判断）*/
static float   s_dist_last = 0.0f;      /* 上一次采样的 dist_abs    */
static float   s_dist_at_start = 0.0f;  /* 起点处的 dist_abs（用于总里程）*/

/* ★ 手推方案：关键点 / 打角列 的统计（菜单与遥测要显示"这份表带不带打角"）*/
static uint16  s_key_cnt   = 0u;
static uint16  s_steer_cnt = 0u;
static uint8   s_traj_mode = 0u;        /* 1 = 这份是实测轨迹（打 INS_WP_FL_TRAJ）*/

/* ★ 工程 10：点距做成运行时参数（ID 39 `rt_ds` 绑到它）。
   等间距仍然重要 —— Pure Pursuit 沿弧长找前视点时会用它做"整数索引"近似。 */
volatile float ins_route_ds = INS_ROUTE_DS_DEFAULT;

void ins_route_init (void)
{
    /* 点距先夹回合法区间：参数可能被上位机写成 0 或离谱值 */
    if (ins_route_ds < INS_ROUTE_DS_MIN) { ins_route_ds = INS_ROUTE_DS_MIN; }
    if (ins_route_ds > INS_ROUTE_DS_MAX) { ins_route_ds = INS_ROUTE_DS_MAX; }

    s_head.origin_x   = 0.0f;
    s_head.origin_y   = 0.0f;
    s_head.origin_yaw = 0.0f;
    s_head.count      = 0;
    s_head.version    = 1;
    s_head.total_dist = 0.0f;

    s_running  = 0;
    s_flags    = 0;
    s_dist_acc = 0.0f;
    s_yaw_last = 0.0f;
    s_dist_last = 0.0f;
}

void ins_route_clear (void)
{
    s_head.count      = 0;
    s_head.total_dist = 0.0f;
    s_dist_acc  = 0.0f;
    s_yaw_last  = 0.0f;
    s_flags    &= (uint8)~INS_ROUTE_ST_FULL;
    s_key_cnt   = 0u;
    s_steer_cnt = 0u;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  ★ 手推方案：关键点标记 / 打角统计
 *      `ins_route_mark_key()` 标的是**最后一个已录点**（不是下一个点）：
 *      采样阈值 5 cm，人按下去时车又走了几厘米，标"最近那个点"才是他心里的位置。
 *      ⚠ 只在录制中有效；非录制状态调用是空操作（返回 0）。
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 ins_route_mark_key (void)
{
    uint16 last;

    if (0 == s_running) { return 0u; }
    if (s_head.count < 2u) { return 0u; }

    last = (uint16)(s_head.count - 1u);
    if (0u == (s_wp[last].flags & (uint16)INS_WP_FL_KEY))
    {
        s_wp[last].flags |= (uint16)INS_WP_FL_KEY;
        s_key_cnt ++;
    }
    return 1u;
}

uint16 ins_route_key_count   (void) { return s_key_cnt;   }
uint16 ins_route_steer_count (void) { return s_steer_cnt; }

/* 1 = 录制出来的点打"实测轨迹"标记（跑一趟回来存轨迹时用）；0 = 教学表（默认）*/
void ins_route_set_traj (uint8 on) { s_traj_mode = (0u != on) ? 1u : 0u; }

/*-------------------------------------------------------------------------------------------------------------------
 *  把 odom 的"世界"位姿换算到"录制起点车体系"
 *      x' =  cos(a0)*dx + sin(a0)*dy
 *      y' = -sin(a0)*dx + cos(a0)*dy        (a0 = 起点航向；等价于把世界系旋转 -a0)
 *-----------------------------------------------------------------------------------------------------------------*/
static void route_to_local (const ins_odom_snap_t *o, uint16 idx, float v_mps)
{
    float dx = o->x - s_head.origin_x;
    float dy = o->y - s_head.origin_y;
    float a0 = s_head.origin_yaw * DEG2RAD_F;
    float c  = cosf(a0);
    float s  = sinf(a0);
    float d;
    uint16 fl = 0u;

    s_wp[idx].x   =  c * dx + s * dy;
    s_wp[idx].y   = -s * dx + c * dy;
    s_wp[idx].yaw =  o->yaw - s_head.origin_yaw;    /* ★ 相对起点的增量，不是绝对航向 */
    s_wp[idx].v   =  v_mps;

    /* ---- ★ 打角列（v2）：记**当拍实际施加的舵机目标角**，不是实测角 ----------------
       为什么记目标而不是实测：实测角有随动滞后（转向内环 ~几十 ms），把它当"前馈"灌回去
       等于把滞后也复制进回放；而人打方向时手上的输入就是目标角。
       ⚠ 上限按 ±40°（软限位 27° 之外留余量），超出就当无效 —— 免得编码器异常值污染表。 */
    d = steer_ctrl_get_target_delta();
    if ((d > -40.0f) && (d < 40.0f))
    {
        s_wp[idx].steer_cdeg = (int16)(d * 100.0f);
        fl |= (uint16)INS_WP_FL_STEER;
        s_steer_cnt ++;
    }
    else
    {
        s_wp[idx].steer_cdeg = 0;
    }

    if (v_mps < -0.02f)                 { fl |= (uint16)INS_WP_FL_REV; }
    if (0u != s_traj_mode)              { fl |= (uint16)INS_WP_FL_TRAJ;}

    s_wp[idx].flags = fl;
}

void ins_route_start (void)
{
    ins_odom_snap_t o;

    ins_odom_snapshot(&o);

    /* 重新捕获起点：这一步让"录制"与"复现"共用同一个基准（Polaris 的纪律） */
    s_head.origin_x   = o.x;
    s_head.origin_y   = o.y;
    s_head.origin_yaw = o.yaw;
    s_head.count      = 0;
    s_head.total_dist = 0.0f;
    s_head.version    = 1;

    s_dist_acc  = 0.0f;
    s_dist_last = o.dist_abs;
    s_dist_at_start = o.dist_abs;
    s_yaw_last  = 0.0f;
    s_flags     = INS_ROUTE_ST_RUNNING;

    s_key_cnt   = 0u;
    s_steer_cnt = 0u;

    /* 起点一定是第 0 点（否则复现时起点对不上） */
    route_to_local(&o, 0, 0.0f);
    s_head.count = 1;

    /* ★ 冻结计数器归零：这一圈"ZUPT 吃了多少"从这一刻开始算（1 Hz 行 fz=/fzd=） */
    ins_imu_zupt_frz_reset();

    s_running = 1;
}

void ins_route_stop (void)
{
    s_running = 0;
    s_flags  &= (uint8)~INS_ROUTE_ST_RUNNING;
}

void ins_route_poll (void)
{
    ins_odom_snap_t o;
    float ds;
    float dyaw;
    uint8 append;

    if ((!s_running) || (s_head.count >= INS_ROUTE_MAX_POINTS))
    {
        /* ★ 录满不静默：置位并让调用方/遥测看得见（Polaris 在这里什么都不说） */
        if (s_running && (s_head.count >= INS_ROUTE_MAX_POINTS))
        {
            s_flags |= INS_ROUTE_ST_FULL;
            s_running = 0;
            s_flags &= (uint8)~INS_ROUTE_ST_RUNNING;
        }
        return;
    }

    ins_odom_snapshot(&o);

    /* 本拍位移（用 dist_abs 的差值：标量、单调增，与 16 位回绕无关） */
    ds = o.dist_abs - s_dist_last;
    if (ds < 0.0f) { ds = 0.0f; }
    s_dist_last = o.dist_abs;
    s_dist_acc += ds;

    /* 录制期间总里程 = 当前累计里程 − 起点累计里程 */
    s_head.total_dist = o.dist_abs - s_dist_at_start;

    dyaw = (o.yaw - s_head.origin_yaw) - s_yaw_last;
    if (dyaw >  180.0f) { dyaw -= 360.0f; }
    if (dyaw < -180.0f) { dyaw += 360.0f; }

    /* ★★ 采样触发：**只用距离**（默认），转角触发默认关闭（`INS_ROUTE_YAW_TRIG_EN = 0`）。
       为什么默认关：`route_cruise_tick()` 的匹配是 `idx = dist / ds` 的**等间距索引**，
       一旦叠加"转角触发"，弯道会额外加密 ⇒ 点距不再相等 ⇒ 索引整体错位
       （表现为"越跑越偏，但看起来又像在跟"）。原工程 `rt_sample_pending()` 也只按距离（`RT_DS_MM = 10`）。
       若确实要弯道加密，必须把"每点累积里程"也存进表里，然后改成二分查找 —— 见 route_cruise.h。 */
    append = 0;
    if (s_dist_acc >= ins_route_ds) { append = 1; }
    if ((0 != INS_ROUTE_YAW_TRIG_EN) && (fabsf(dyaw) >= INS_ROUTE_YAW_THRESH)) { append = 1; }

    if (append)
    {
        route_to_local(&o, s_head.count, o.v_mps);

        s_head.count ++;

        /* ★★ 扣掉阈值，**不是清零**（2026-10-11 实机实测出来的 bug）：
           本函数每圈只调一次，`ds` = "这一圈走了多少"。若直接 `s_dist_acc = 0`，
           每一点都会把这一圈的**超出量**丢掉 ⇒ 实际点距 = 设定的 ds + 一圈位移（平均 +半圈）。
           实测：**1728 点 / 37.06 m ⇒ 平均点距 21.45 mm（+7.2%）** —— 后果：
             · 表头自称 `ds_mm=20`，与真实间距不符（上位机/工具都会按 20 算）；
             · `route_cruise` 的 `idx = dist / ds` 是**等间距索引** ⇒ 整体错位 7%
               ⇒ 一圈 37 m 差 2.6 m ≈ 123 个点，正是"越跑越偏但看着又像在跟"。
           改成扣阈值后，残余误差只剩"不到一圈的抖动"，点距是真的 20.00 mm。 */
        s_dist_acc -= ins_route_ds;
        if (s_dist_acc < 0.0f) { s_dist_acc = 0.0f; }

        s_yaw_last = o.yaw - s_head.origin_yaw;

        if (s_head.count >= INS_ROUTE_MAX_POINTS)
        {
            s_flags |= INS_ROUTE_ST_FULL;
            s_running = 0;
            s_flags &= (uint8)~INS_ROUTE_ST_RUNNING;
        }
    }
}

uint8  ins_route_is_running (void)      { return s_running; }
uint8  ins_route_flags      (void)      { return s_flags; }
uint16 ins_route_get_count  (void)      { return s_head.count; }
float  ins_route_get_total_dist (void)  { return s_head.total_dist; }

const ins_wp_t         *ins_route_get_points (void) { return s_wp; }
const ins_route_head_t *ins_route_get_head   (void) { return &s_head; }

/*===================================================================================================================
 *  ★★ Flash 持久化
 *
 *  磁盘（Work Flash）布局见 ins_route.h。
 *  这里只强调三件事：
 *    ① **分步写**：`start` 武装，`poll` 每圈写一页 ⇒ 主循环不被上百毫秒的擦写顶住。
 *    ② **CRC 按"存进去的字节"算**：存/取两侧都走同一个 `rt_rec_pack()`
 *       ⇒ 16 B 的 mm/cdeg/mm-cdeg 整数是**字节级可复现**的，不存在浮点舍入导致的校验失败。
 *    ③ `flash_write_page_from_buffer()` 只认全局 `flash_union_buffer`（2048 B），
 *       并且**一次写整页**（len = FLASH_PAGE_LENGTH = 512 个 uint32）。
 *       ⚠ 逐飞的 `flash_buffer_clear()` 只 memset **前 512 字节**，负载大时必须自己 memset。
 *=================================================================================================================*/

#define RT_MAGIC            (0x4B525443u)       /* "CTRK" */
#define RT_VER              (2u)                /* ★ v2 = 16 B/条；v1(12 B) 载入时判无效 */

typedef struct __attribute__((packed))
{
    int32   x_mm;           /* 相对录制起点，mm（向右） */
    int32   y_mm;           /* 相对录制起点，mm（向前） */
    int16   yaw_cdeg;       /* 相对录制起点航向，0.01°（已折到 ±180°）*/
    int16   v_cmps;         /* 车速，cm/s（记录类则解释为舵机 duty）*/
    int16   steer_cdeg;     /* ★ 当拍实际施加的舵机目标角，0.01°（无则 0，看 flags）*/
    uint16  flags;          /* ★ INS_WP_FL_* */
} rt_rec_t;                                     /* = 16 B */

typedef struct __attribute__((packed))
{
    uint32  magic;
    uint16  ver;
    uint16  rec_len;
    uint32  count;
    uint32  ds_mm;          /* 采样点距 mm（= 存盘那一刻的 ins_route_ds × 1000）*/
    uint32  lap_mm;         /* 录制期间总里程 mm */
    int32   origin_x_mm;    /* ★ 起点位姿（复现时"只用相对量"的依据）*/
    int32   origin_y_mm;
    int16   origin_yaw_cdeg;
    uint16  flags;          /* bit0 = 录满 */
    uint16  crc_r;          /* ★ 记录区 CRC16（按"写进去的字节"累进算）*/
    uint16  crc_h;          /* ★ 表头 CRC16，覆盖 [0, 34)（含 crc_r，不含自己）*/
} rt_head_t;                                    /* = 36 B（两个 CRC 各 2 B）*/

_Static_assert(sizeof(rt_head_t) == 36u, "rt_head_t 不是 36 B：CRC 覆盖范围会错");
_Static_assert(offsetof(rt_head_t, crc_h) == 34u, "表头布局变了：CRC 覆盖长度要同步改");
_Static_assert(sizeof(rt_rec_t)  == INS_RT_REC_SIZE, "rt_rec_t 不是 16 B：Flash 布局会整体错位");
_Static_assert(INS_RT_REC_PER_PAGE * INS_RT_REC_SIZE <= 2048u, "一页装不下 128 条");

/* ⚠ 关于 `__attribute__((packed))`：**IAR(ICCARM) 对 GCC 风格属性不保证生效**
 *   （IAR 用的是 `__packed` 关键字或 `#pragma pack`）。
 *   好在本工程这两个结构体**天然对齐**（字段按 4/2 字节顺序声明，
 *   正好没有填充）⇒ “加不加 packed 都是同一套字节布局”，这也是“上位机按固定偏移解析”
 *   能成立的前提。
 *   ⇒ **以后往这两个结构体里插 uint8/int8 字段时必须重新核对**；
 *     或者干脆改成“手工按偏移读写”（见 motor_link.c 的 motor_rt_build_head）。 */

static uint8   s_fl_busy   = 0u;
static uint8   s_fl_rc     = 0u;        /* 0 未做 / 1 成功 / 2 写失败 / 3 表空 */
static uint8   s_fl_loaded = 0u;        /* 当前 RAM 里的表来自 Flash */
static uint16  s_fl_page   = 0u;        /* 下一个要写的记录页下标（槽内相对） */
static uint16  s_fl_pages  = 0u;        /* 需要写的记录页数 */
static uint16  s_fl_crc    = 0u;        /* 累进 CRC（记录部分）*/
static uint16  s_fl_done   = 0u;        /* 已写页数（含表头那一步），用于百分比 */

/* ---- 多槽（2026-10-12）---- */
static uint8   s_fl_slot   = 0u;        /* 当前操作的槽号 */
static uint32  s_fl_base   = 0u;        /* = INS_RT_SLOT_BASE(槽号)，本槽的头页 */
static uint8   s_fl_last_slot = 0u;     /* 最近一次写/读的槽（菜单显示用） */
static uint8   s_active_slot  = 0u;     /* 槽索引里记的"当前选中槽" */

/* ---- 分步写盘：**数据源**与**表头快照**（2026-10-12，为了支持"写巡航记录"）----
 *   原来 poll/表头都直接读全局 `s_head` / `s_wp[]`，只能写路表。
 *   现在把"这次要写什么"在**武装那一刻**快照下来 ⇒ 同一个分步写盘器能写两种东西。 */
static uint8   s_fl_src    = 0u;        /* 0 = s_wp[]（路表）  1 = s_log[]（巡航记录，已是 12 B 打包态）*/
static uint32  s_fl_count  = 0u;        /* 本次要写多少条（快照）*/
static uint32  s_fl_ds_mm  = 0u;        /* 槽头 ds_mm（路表=点距 mm；记录=采样周期 ms）*/
static uint32  s_fl_lap_mm = 0u;        /* 槽头 lap_mm（路表=圈长 mm；记录=总时长 ms）*/
static int32   s_fl_ox_mm  = 0;         /* 槽头 origin（记录恒 0）*/
static int32   s_fl_oy_mm  = 0;
static int16   s_fl_oyaw   = 0;
static uint16  s_fl_flags  = 0u;        /* 槽头 flags（bit1 = 巡航记录）*/
static uint8   s_fl_setact = 1u;        /* ★ 写完是否把该槽设为 active —— 见下面 flash_arm 的说明 */

/* ---- 巡航记录缓冲（见 ins_route.h 的说明）---- */
static rt_rec_t s_log[INS_ROUTE_LOG_MAX];
static uint32   s_log_n      = 0u;
static uint16   s_log_acc_ms = 0u;      /* 距上次采样累计了多少 ms */
static uint8    s_log_full   = 0u;
static uint8    s_log_arm    = 0u;      /* 1 = 有人要求"停巡后落盘"（中断里只置这个）*/
static uint8    s_log_saving = 0u;      /* 1 = 已武装/正在写 */
static uint16   s_log_flags  = (uint16)INS_ROUTE_HDR_FL_LOG;   /* 这份记录的"类型"（见 ins_route.h）*/
static uint16   s_log_per_ms = 20u;     /* 写入槽头的采样周期（巡航记录可调，阶跃测试恒 5）*/
static uint8    s_log_slot   = (uint8)INS_ROUTE_LOG_SLOT;      /* 写进哪一槽（按类型自动定）*/

volatile float  ins_log_ms   = 20.0f;   /* 采样周期 ms（参数 ID 38）*/

/* 槽索引页（page 91）：active_slot + 每槽摘要。单独一页 ⇒ 改它不影响任何槽的数据。
 * ⚠ 页 0 是槽 0 的**头页**，不能拿来放索引（写槽 0 头会把索引抹掉）。 */
#define RT_IDX_MAGIC    (0x58495243u)   /* "CRIX" */
#define RT_IDX_VER      (1u)

typedef struct __attribute__((packed))
{
    uint32  magic;
    uint16  ver;
    uint16  slot_count;
    uint8   active_slot;
    uint8   rsv[3];
    struct
    {
        uint8   valid;
        uint8   slot;
        uint16  tag;
        uint32  count;
        uint32  lap_mm;
        uint32  ds_mm;
        uint32  crc_r;
    } s[INS_RT_SLOT_COUNT];
    uint16  crc;                    /* CRC16-MODBUS 覆盖 [0, offsetof(crc)) */
} rt_idx_t;

/* ⚠ 槽数从 7 改成 4（工程 10）⇒ CRC 覆盖长度跟着变。
   这里**由槽数算**、不写死数字 —— 写死就必然在下一次改槽数时炸在这个断言上。 */
_Static_assert(offsetof(rt_idx_t, crc) == 12u + 20u * INS_RT_SLOT_COUNT,
               "槽索引布局变了：CRC 覆盖长度要同步改");
_Static_assert(sizeof(rt_idx_t) <= 2048u,        "槽索引装不下一页");

static uint16 rt_crc16 (uint16 crc, const uint8 *p, uint32 n)
{
    uint32 i;
    uint8  j;

    for (i = 0; i < n; i ++)
    {
        crc ^= (uint16)p[i];
        for (j = 0; j < 8u; j ++)
        {
            if (0u != (crc & 1u)) { crc = (uint16)((crc >> 1) ^ 0xA001u); }
            else                  { crc = (uint16)(crc >> 1); }
        }
    }
    return crc;
}

/* 相对 yaw 折到 ±180°（表的 yaw 本来就是"相对起点"，一圈内不会超；这里只是兜底） */
static int16 rt_wrap_cdeg (float yaw_deg)
{
    float y = yaw_deg;

    while (y >  180.0f) { y -= 360.0f; }
    while (y < -180.0f) { y += 360.0f; }

    return (int16)(y * 100.0f);
}

static float rt_clampf (float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static void rt_rec_pack (const ins_wp_t *wp, rt_rec_t *r)
{
    r->x_mm       = (int32)(wp->x * 1000.0f);
    r->y_mm       = (int32)(wp->y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(wp->yaw);
    r->v_cmps     = (int16)rt_clampf(wp->v * 100.0f, -32000.0f, 32000.0f);
    r->steer_cdeg = wp->steer_cdeg;
    r->flags      = wp->flags;
}

uint16 ins_route_pack_records (uint16 first, uint8 *dst, uint16 max_n)
{
    uint16 k;

    if (NULL == dst) { return 0u; }

    for (k = 0; k < max_n; k ++)
    {
        uint32 idx = (uint32)first + k;
        rt_rec_t r;

        if (idx >= (uint32)s_head.count) { break; }

        rt_rec_pack(&s_wp[idx], &r);
        memcpy(dst + (uint32)k * INS_RT_REC_SIZE, &r, INS_RT_REC_SIZE);
    }
    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  槽索引页（page 91）
 *      rt_idx_load  —— 读回来；magic/ver/slot_count/CRC 任一不过 ⇒ 返回"全空"的默认值
 *                      （**不报错**：从没存过索引是正常状态，槽 0 的表照样能读）
 *      rt_idx_store —— 整页重写（擦 + 写）。索引只有 154 B，一格页很便宜。
 *-----------------------------------------------------------------------------------------------------------------*/
static void rt_idx_load (rt_idx_t *d)
{
    rt_idx_t r;

    memset(d, 0, sizeof(*d));
    d->magic      = RT_IDX_MAGIC;
    d->ver        = (uint16)RT_IDX_VER;
    d->slot_count = (uint16)INS_RT_SLOT_COUNT;

    flash_read_page_to_buffer(0u, INS_RT_INDEX_PAGE, (uint32)FLASH_PAGE_LENGTH);
    memcpy(&r, (const void *)flash_union_buffer, sizeof(r));

    if ((RT_IDX_MAGIC == r.magic) &&
        ((uint16)RT_IDX_VER == r.ver) &&
        ((uint16)INS_RT_SLOT_COUNT == r.slot_count) &&
        (rt_crc16(0xFFFFu, (const uint8 *)&r, (uint32)offsetof(rt_idx_t, crc)) == r.crc))
    {
        *d = r;
    }
    else
    {
        uint8 k;

        for (k = 0u; k < (uint8)INS_RT_SLOT_COUNT; k ++) { d->s[k].slot = k; }
    }

    if (d->active_slot >= (uint8)INS_RT_SLOT_COUNT) { d->active_slot = 0u; }
}

static void rt_idx_store (const rt_idx_t *src)
{
    rt_idx_t w = *src;

    w.magic      = RT_IDX_MAGIC;
    w.ver        = (uint16)RT_IDX_VER;
    w.slot_count = (uint16)INS_RT_SLOT_COUNT;
    w.crc        = rt_crc16(0xFFFFu, (const uint8 *)&w, (uint32)offsetof(rt_idx_t, crc));

    memset((void *)flash_union_buffer, 0xFF, 2048u);
    memcpy((void *)flash_union_buffer, &w, sizeof(w));

    flash_erase_page(0u, INS_RT_INDEX_PAGE);
    (void)flash_write_page_from_buffer(0u, INS_RT_INDEX_PAGE, (uint32)FLASH_PAGE_LENGTH);
}

/*-------------------------------------------------------------------------------------------------------------------
 *  分步写盘
 *-----------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
 *  武装一次分步写盘（路表 / 巡航记录**共用**）
 *      ⚠ 这里**只快照**"要写什么"，真正的擦写由 `ins_route_flash_poll()` 一页一页做。
 *        把 count/表头字段都在这一刻定下来，同一个写盘器才能写两种来源的东西。
 *
 *  ⚠⚠ `setact`（2026-10-12 补）—— 写完要不要把这一槽设为 active。
 *      原来**无条件**把 active 指到刚写的槽（理由：录完/灌完下一步就是想跑它）。
 *      但"记录"（巡航记录 / 阶跃测试）不是路表 ⇒ 写完把它设成 active，
 *      下次上电 `ins_route_flash_load()` 会**把一份记录当成路表载入**，
 *      而 `route_cruise` 会拿第 4 字段（舵机 duty）当车速用 ⇒ **静默乱跑**。
 *      ⇒ 路表(setact=1) 保持老行为；记录(setact=0) 绝不碰 active。
 *-----------------------------------------------------------------------------------------------------------------*/
static void flash_arm (uint8 slot, uint8 src, uint32 count,
                       uint32 ds_mm, uint32 lap_mm,
                       int32 ox, int32 oy, int16 oyaw, uint16 flags,
                       uint8 setact)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { s_fl_busy = 0u; s_fl_rc = 4u; return; }
    if (0u == count)                      { s_fl_busy = 0u; s_fl_rc = 3u; return; }
    if (count > INS_RT_FLASH_CAPACITY)    { count = INS_RT_FLASH_CAPACITY; }

    flash_init();

    s_fl_slot  = slot;
    s_fl_base  = INS_RT_SLOT_BASE(slot);
    s_fl_pages = (uint16)((count + (INS_RT_REC_PER_PAGE - 1u)) / INS_RT_REC_PER_PAGE);
    s_fl_page  = 0u;
    s_fl_crc   = 0xFFFFu;
    s_fl_done  = 0u;
    s_fl_rc    = 0u;
    s_fl_busy  = 1u;

    s_fl_src   = src;
    s_fl_count = count;
    s_fl_ds_mm = ds_mm;
    s_fl_lap_mm= lap_mm;
    s_fl_ox_mm = ox;
    s_fl_oy_mm = oy;
    s_fl_oyaw  = oyaw;
    s_fl_flags = flags;
    s_fl_setact= (0u != setact) ? 1u : 0u;
}

void ins_route_flash_save_slot (uint8 slot)
{
    flash_arm(slot, 0u, (uint32)s_head.count,
              (uint32)(ins_route_ds * 1000.0f + 0.5f),
              (uint32)(s_head.total_dist * 1000.0f + 0.5f),
              (int32)(s_head.origin_x * 1000.0f),
              (int32)(s_head.origin_y * 1000.0f),
              rt_wrap_cdeg(s_head.origin_yaw),
              (uint16)((0u != (s_flags & INS_ROUTE_ST_FULL)) ? 1u : 0u),
              1u);                                  /* 路表：写完设为 active（老行为）*/
}

/* 把**巡航记录 / 阶跃记录**写进某个槽（数据源 = s_log[]，不是 s_wp[]）
   ⚠ setact = 0：记录不是路表，绝不能抢 active（否则下次上电会把记录当路表载入）。 */
static void flash_arm_log (uint8 slot)
{
    flash_arm(slot, 1u, s_log_n,
              (uint32)s_log_per_ms,                                   /* ds_mm 存周期 ms */
              (uint32)(s_log_n * (uint32)s_log_per_ms),               /* lap_mm 存总时长 ms */
              0, 0, 0,
              (uint16)(s_log_flags | ((0u != s_log_full) ? INS_ROUTE_HDR_FL_FULL : 0u)),
              0u);
}

void ins_route_flash_save_start (void)
{
    ins_route_flash_save_slot(0u);          /* 手推录制的家固定是槽 0 */
}

void ins_route_flash_poll (void)
{
    uint8  *pg = (uint8 *)flash_union_buffer;

    if (0u == s_fl_busy) { return; }

    /* --- 还有记录页要写：一次只写一页 --- */
    if (s_fl_page < s_fl_pages)
    {
        uint16 k;

        memset(pg, 0xFF, 2048u);

        for (k = 0; k < (uint16)INS_RT_REC_PER_PAGE; k ++)
        {
            uint32 idx = (uint32)s_fl_page * INS_RT_REC_PER_PAGE + k;

            if (idx >= s_fl_count) { break; }

            if (0u == s_fl_src)
            {
                rt_rec_t r;

                rt_rec_pack(&s_wp[idx], &r);
                memcpy(pg + (uint32)k * INS_RT_REC_SIZE, &r, INS_RT_REC_SIZE);
            }
            else
            {
                /* 巡航记录已经是 12 B 打包态 ⇒ 直接搬（**逐字节同构**，CRC 算法一致） */
                memcpy(pg + (uint32)k * INS_RT_REC_SIZE, (const void *)&s_log[idx], INS_RT_REC_SIZE);
            }
        }

        /* ★ 只对"真正写进去的那些字节"算 CRC（尾部的 0xFF 填充不参与）*/
        {
            uint32 used = (uint32)s_fl_page * INS_RT_REC_PER_PAGE;
            uint32 left = (s_fl_count > used) ? (s_fl_count - used) : 0u;

            if (left > INS_RT_REC_PER_PAGE) { left = INS_RT_REC_PER_PAGE; }
            s_fl_crc = rt_crc16(s_fl_crc, pg, left * INS_RT_REC_SIZE);
        }

        flash_erase_page(0u, (uint32)(s_fl_base + 1u + (uint32)s_fl_page));
        (void)flash_write_page_from_buffer(0u, (uint32)(s_fl_base + 1u + (uint32)s_fl_page),
                                           (uint32)FLASH_PAGE_LENGTH);

        s_fl_page ++;
        s_fl_done ++;

        /* ★★ 读回校验：**先把"期望值"抄出来**，再读回比较。
           ⚠ 踩过的坑：读回用的是同一个 `flash_union_buffer`（`pg` 就指着它），
             所以"读回后再拿 pg 比"等于**自己跟自己比**，永远相等 —— 校验形同虚设。 */
        if (1u == s_fl_page)
        {
            uint8 exp[16];

            memcpy(exp, pg, sizeof(exp));

            flash_read_page_to_buffer(0u, (uint32)(s_fl_base + 1u), (uint32)FLASH_PAGE_LENGTH);
            if (0 != memcmp(exp, (const void *)flash_union_buffer, sizeof(exp)))
            {
                s_fl_busy = 0u;
                s_fl_rc   = 2u;
                return;
            }
        }
        return;
    }

    /* --- 记录写完，最后写表头页 --- */
    {
        rt_head_t h;

        memset(&h, 0, sizeof(h));
        h.magic     = RT_MAGIC;
        h.ver       = (uint16)RT_VER;
        h.rec_len   = (uint16)INS_RT_REC_SIZE;
        h.count     = s_fl_count;                 /* ★ 用武装时的快照（路表=点数，记录=条数）*/
        h.ds_mm     = s_fl_ds_mm;                 /* ★ 路表=点距mm；记录=采样周期ms */
        h.lap_mm    = s_fl_lap_mm;                /* ★ 路表=圈长mm；记录=总时长ms */
        h.origin_x_mm     = s_fl_ox_mm;
        h.origin_y_mm     = s_fl_oy_mm;
        h.origin_yaw_cdeg = s_fl_oyaw;
        h.flags     = s_fl_flags;                 /* ★ bit1 = 这是一份巡航记录 */

        h.crc_r = s_fl_crc;                          /* 写页时已经累进算好 */
        h.crc_h = rt_crc16(0xFFFFu, (const uint8 *)&h, 34u);   /* 覆盖 [0,34) = magic..crc_r（含 crc_r 本身，正好保护它）*/

        memset((void *)flash_union_buffer, 0xFF, 2048u);
        memcpy((void *)flash_union_buffer, &h, sizeof(h));

        flash_erase_page(0u, s_fl_base);
        (void)flash_write_page_from_buffer(0u, s_fl_base, (uint32)FLASH_PAGE_LENGTH);

        /* ★ 读回校验表头（写失败最常见的原因是掉电/擦除未完）*/
        {
            rt_head_t rb;

            flash_read_page_to_buffer(0u, s_fl_base, (uint32)FLASH_PAGE_LENGTH);
            memcpy(&rb, (const void *)flash_union_buffer, sizeof(rb));
            if (0 != memcmp(&rb, &h, sizeof(h)))
            {
                s_fl_busy = 0u;
                s_fl_rc   = 2u;
                s_fl_done ++;
                return;
            }
        }

        /* ★★ 更新槽索引：写本槽摘要；**只有路表**才把 active 指到本槽。
           ⚠ 老行为是"无条件改 active"（理由：录完/灌完下一步就是想跑它），
             但记录（巡航记录 / 阶跃测试）不是路表 —— 把 active 指过去，
             下次上电就会把一份记录当路表载入（`route_cruise` 拿第 4 字段当车速用）。
           ⇒ 由 `s_fl_setact` 决定；路表 = 1（保持老行为），记录 = 0。 */
        {
            rt_idx_t d;
            uint8    k = s_fl_slot;

            rt_idx_load(&d);
            d.s[k].valid  = 1u;
            d.s[k].slot   = k;
            d.s[k].count  = h.count;
            d.s[k].lap_mm = h.lap_mm;
            d.s[k].ds_mm  = h.ds_mm;
            d.s[k].crc_r  = h.crc_r;
            if (0u != s_fl_setact)
            {
                d.active_slot = k;
            }
            rt_idx_store(&d);
            if (0u != s_fl_setact)
            {
                s_active_slot = k;
            }
        }
    }

    s_fl_last_slot = s_fl_slot;
    s_fl_busy = 0u;
    s_fl_rc   = 1u;
    s_fl_done ++;
    s_fl_loaded = 1u;
}

uint8 ins_route_flash_busy     (void) { return s_fl_busy; }
uint8 ins_route_flash_last_rc  (void) { return s_fl_rc; }
uint8 ins_route_flash_loaded   (void) { return s_fl_loaded; }
uint8 ins_route_flash_progress (void)
{
    uint16 total = (uint16)(s_fl_pages + 1u);

    if (0u == total) { return 0u; }
    if (s_fl_done >= total) { return 100u; }
    return (uint8)((s_fl_done * 100u) / total);
}

/*-------------------------------------------------------------------------------------------------------------------
 *  上电读取
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 ins_route_flash_load_slot (uint8 slot)
{
    rt_head_t h;
    uint32    n;
    uint32    base;

    s_fl_loaded = 0u;

    if (slot >= (uint8)INS_RT_SLOT_COUNT) { return 0u; }
    base = INS_RT_SLOT_BASE(slot);

    flash_init();
    flash_read_page_to_buffer(0u, base, (uint32)FLASH_PAGE_LENGTH);
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if (RT_MAGIC != h.magic)                       { return 0u; }   /* 从没存过 */
    if ((uint16)RT_VER != h.ver)                   { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != h.rec_len)      { return 0u; }
    if (0u == h.count)                             { return 0u; }
    if (h.count > INS_RT_FLASH_CAPACITY)           { return 0u; }
    if (h.count > (uint32)INS_ROUTE_MAX_POINTS)    { return 0u; }
    if (rt_crc16(0xFFFFu, (const uint8 *)&h, 34u) != h.crc_h) { return 0u; }

    n = h.count;

    /* ★ 按页读、逐条 CRC、逐条展开 —— 一次页读处理 128 条，
       比"每条都整页读一遍"快 128 倍（后者是 2048 次 2 KB 读，要好几秒）。 */
    {
        uint16 crc_r = 0xFFFFu;
        uint32 done  = 0u;

        while (done < n)
        {
            uint32 page = done / INS_RT_REC_PER_PAGE;
            uint32 left = n - done;
            uint32 k;

            if (left > INS_RT_REC_PER_PAGE) { left = INS_RT_REC_PER_PAGE; }

            flash_read_page_to_buffer(0u, (uint32)(base + 1u + page),
                                      (uint32)FLASH_PAGE_LENGTH);

            crc_r = rt_crc16(crc_r, (const uint8 *)flash_union_buffer,
                             left * INS_RT_REC_SIZE);

            for (k = 0; k < left; k ++)
            {
                rt_rec_t r;

                memcpy(&r, (const uint8 *)flash_union_buffer + k * INS_RT_REC_SIZE,
                       INS_RT_REC_SIZE);

                s_wp[done + k].x          = (float)r.x_mm / 1000.0f;
                s_wp[done + k].y          = (float)r.y_mm / 1000.0f;
                s_wp[done + k].yaw        = (float)r.yaw_cdeg / 100.0f;
                s_wp[done + k].v          = (float)r.v_cmps / 100.0f;
                s_wp[done + k].steer_cdeg = r.steer_cdeg;
                s_wp[done + k].flags      = r.flags;
            }

            done += left;
        }

        if (crc_r != h.crc_r) { return 0u; }        /* 记录区被写坏 ⇒ 当没有表 */
    }

    s_head.origin_x   = (float)h.origin_x_mm / 1000.0f;
    s_head.origin_y   = (float)h.origin_y_mm / 1000.0f;
    s_head.origin_yaw = (float)h.origin_yaw_cdeg / 100.0f;
    s_head.count      = (uint16)n;
    s_head.version    = 1u;
    s_head.total_dist = (float)h.lap_mm / 1000.0f;

    s_running = 0u;
    s_flags   = 0u;
    if (0u != (h.flags & 1u)) { s_flags |= INS_ROUTE_ST_FULL; }

    /* 统计"这份表带不带打角/关键点"（回放前菜单要能看出来）*/
    {
        uint32 i;

        s_steer_cnt = 0u;
        s_key_cnt   = 0u;
        for (i = 0u; i < n; i ++)
        {
            if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_STEER)) { s_steer_cnt ++; }
            if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_KEY))   { s_key_cnt   ++; }
        }
    }

    s_fl_last_slot = slot;
    s_fl_loaded = 1u;
    return 1u;
}

uint8 ins_route_flash_load (void)
{
    return ins_route_flash_load_slot(s_active_slot);    /* 载入"当前选中槽" */
}


/*===================================================================================================================
 *  ★ 从"内存镜像"载入路表（2026-10-12）
 *
 *  与 `ins_route_flash_load()` 读的是**同一套字节格式**（36 B 头 + 12 B/点 + 两个 CRC16-MODBUS），
 *  区别只是来源：这里是一块**连续内存**，不是 Flash 分页。
 *
 *  两个用途：
 *    ① **内建理论表**：`ins_route_builtin.c`（由 `tools/route_to_carbin.py` 生成）——
 *        "小车能不能转圈"必须先把一张理论表放上车，而 PC→车 的灌表通道还没做
 *        （见 `06-链路缺口清单.md` §2）。编译进去是最短路径，不依赖 WiFi/串口。
 *    ② 以后补"下发通道"时，把收到的整块字节直接交给它 —— **校验逻辑只留一份**。
 *
 *  ⚠ 按**手工偏移**读，不 memcpy 到结构体：省得再赌 `__attribute__((packed))` 在 IAR 下是否生效
 *     （Flash 那条路能 memcpy 是因为 2048 B 页里布局天然对齐；这里不想依赖同一个运气）。
 *=================================================================================================================*/
uint8 ins_route_load_image (const uint8 *img, uint32 len)
{
    uint32 magic, count, ds_mm, lap_mm, need, i;
    uint16 ver, rec_len, crc_r, crc_h, flags;
    int32  ox, oy;
    int16  oyaw;

    if ((NULL == img) || (len < 36u)) { return 0u; }

    magic   = (uint32)img[0]  | ((uint32)img[1]  << 8) | ((uint32)img[2]  << 16) | ((uint32)img[3]  << 24);
    ver     = (uint16)((uint16)img[4]  | ((uint16)img[5]  << 8));
    rec_len = (uint16)((uint16)img[6]  | ((uint16)img[7]  << 8));
    count   = (uint32)img[8]  | ((uint32)img[9]  << 8) | ((uint32)img[10] << 16) | ((uint32)img[11] << 24);
    ds_mm   = (uint32)img[12] | ((uint32)img[13] << 8) | ((uint32)img[14] << 16) | ((uint32)img[15] << 24);
    lap_mm  = (uint32)img[16] | ((uint32)img[17] << 8) | ((uint32)img[18] << 16) | ((uint32)img[19] << 24);
    ox      = (int32)((uint32)img[20] | ((uint32)img[21] << 8) | ((uint32)img[22] << 16) | ((uint32)img[23] << 24));
    oy      = (int32)((uint32)img[24] | ((uint32)img[25] << 8) | ((uint32)img[26] << 16) | ((uint32)img[27] << 24));
    oyaw    = (int16)((uint16)img[28] | ((uint16)img[29] << 8));
    flags   = (uint16)((uint16)img[30] | ((uint16)img[31] << 8));
    crc_r   = (uint16)((uint16)img[32] | ((uint16)img[33] << 8));
    crc_h   = (uint16)((uint16)img[34] | ((uint16)img[35] << 8));

    /* ---- 与 flash_load 完全一致的准入条件 ---- */
    if (RT_MAGIC != magic)                        { return 0u; }
    if ((uint16)RT_VER != ver)                    { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != rec_len)       { return 0u; }
    if (0u == count)                              { return 0u; }
    if (count > (uint32)INS_ROUTE_MAX_POINTS)     { return 0u; }
    if (0u == ds_mm)                              { return 0u; }
    if (rt_crc16(0xFFFFu, img, 34u) != crc_h)     { return 0u; }

    need = 36u + count * (uint32)INS_RT_REC_SIZE;
    if (len < need)                               { return 0u; }
    if (rt_crc16(0xFFFFu, img + 36u, count * (uint32)INS_RT_REC_SIZE) != crc_r) { return 0u; }

    for (i = 0u; i < count; i ++)
    {
        const uint8 *p = img + 36u + i * (uint32)INS_RT_REC_SIZE;
        int32  x_mm     = (int32)((uint32)p[0] | ((uint32)p[1] << 8) | ((uint32)p[2] << 16) | ((uint32)p[3] << 24));
        int32  y_mm     = (int32)((uint32)p[4] | ((uint32)p[5] << 8) | ((uint32)p[6] << 16) | ((uint32)p[7] << 24));
        int16  yaw_cdeg = (int16)((uint16)p[8]  | ((uint16)p[9]  << 8));
        int16  v_cmps   = (int16)((uint16)p[10] | ((uint16)p[11] << 8));
        int16  st_cdeg  = (int16)((uint16)p[12] | ((uint16)p[13] << 8));
        uint16 fl       = (uint16)((uint16)p[14] | ((uint16)p[15] << 8));

        s_wp[i].x          = (float)x_mm / 1000.0f;
        s_wp[i].y          = (float)y_mm / 1000.0f;
        s_wp[i].yaw        = (float)yaw_cdeg / 100.0f;
        s_wp[i].v          = (float)v_cmps / 100.0f;
        s_wp[i].steer_cdeg = st_cdeg;
        s_wp[i].flags      = fl;
    }

    /* 统计"这份表带不带打角/关键点"（菜单与遥测要看）*/
    s_steer_cnt = 0u;
    s_key_cnt   = 0u;
    for (i = 0u; i < count; i ++)
    {
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_STEER)) { s_steer_cnt ++; }
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_KEY))   { s_key_cnt   ++; }
    }

    s_head.origin_x   = (float)ox / 1000.0f;
    s_head.origin_y   = (float)oy / 1000.0f;
    s_head.origin_yaw = (float)oyaw / 100.0f;
    s_head.count      = (uint16)count;
    s_head.version    = 1u;
    s_head.total_dist = (float)lap_mm / 1000.0f;

    s_running = 0u;
    s_flags   = 0u;
    if (0u != (flags & 1u)) { s_flags |= INS_ROUTE_ST_FULL; }

    s_fl_loaded = 0u;               /* ★ 不是从 Flash 来的：上位机会提示"这是内建/下发的表" */

    return 1u;
}

/*===================================================================================================================
 *  ★ 多槽接口（2026-10-12）
 *=================================================================================================================*/
void ins_route_slot_init (void)
{
    rt_idx_t d;

    flash_init();
    rt_idx_load(&d);
    s_active_slot = d.active_slot;
}

uint8 ins_route_slot_active (void)     { return s_active_slot; }
uint8 ins_route_flash_last_slot (void) { return s_fl_last_slot; }

uint8 ins_route_slot_set_active (uint8 slot, uint8 persist)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { return 1u; }

    s_active_slot = slot;

    if (0u != persist)
    {
        rt_idx_t d;

        flash_init();
        rt_idx_load(&d);
        d.active_slot = slot;
        rt_idx_store(&d);
    }
    return 0u;
}

/*---- 槽摘要：索引里有就用索引（它是缓存）；没有就现读该槽头页（老槽 0 靠这条能列出来）----*/
uint8 ins_route_slot_info (uint8 slot, ins_route_slot_info_t *out)
{
    rt_idx_t  d;
    rt_head_t h;

    if ((NULL == out) || (slot >= (uint8)INS_RT_SLOT_COUNT)) { return 0u; }

    memset(out, 0, sizeof(*out));
    out->valid = 0u;
    out->slot  = slot;

    flash_init();

    rt_idx_load(&d);
    if (0u != d.s[slot].valid)
    {
        out->valid  = 1u;
        out->tag    = d.s[slot].tag;
        out->count  = d.s[slot].count;
        out->lap_mm = d.s[slot].lap_mm;
        out->ds_mm  = d.s[slot].ds_mm;
        out->crc_r  = d.s[slot].crc_r;
        return 1u;
    }

    flash_read_page_to_buffer(0u, INS_RT_SLOT_BASE(slot), (uint32)FLASH_PAGE_LENGTH);
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if ((RT_MAGIC != h.magic) || ((uint16)RT_VER != h.ver) ||
        ((uint16)INS_RT_REC_SIZE != h.rec_len) || (0u == h.count) ||
        (h.count > INS_RT_FLASH_CAPACITY) ||
        (rt_crc16(0xFFFFu, (const uint8 *)&h, 34u) != h.crc_h))
    {
        return 0u;
    }

    out->valid  = 1u;
    out->count  = h.count;
    out->lap_mm = h.lap_mm;
    out->ds_mm  = h.ds_mm;
    out->crc_r  = h.crc_r;
    return 1u;
}

/*---- 从内存镜像写进指定槽：先校验+展开进 RAM，再分步落盘。
       返回 0 = 已武装（等 poll 写完）；1 = 槽越界；2 = 镜像不过；3 = 无法武装 ----*/
uint8 ins_route_slot_load_image (uint8 slot, const uint8 *img, uint32 len)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT)     { return 1u; }
    if (0u == ins_route_load_image(img, len)) { return 2u; }

    ins_route_flash_save_slot(slot);
    if (0u == s_fl_busy)                      { return 3u; }
    return 0u;
}

/*===================================================================================================================
 *  ★ 按"镜像偏移"读字节（2026-10-12）—— 串口上传（车 → PC）用
 *
 *  镜像布局： [0,36) = 36 B 头，之后每 12 B 一条记录，**逻辑连续**（物理上分页）。
 *  所以"偏移 → 页"要算一下：
 *      off < 36                     ⇒ 头页（page base）
 *      否则 rec=(off-36)/12、page=rec/170 ⇒ 记录页（base+1+page）
 *  一次调用里**只在页号变化时重读**那一页（`flash_read_page_to_buffer` 是 2 KB 阻塞读，
 *  512 B 的块最多碰 2 页）。⚠ 不做跨调用的页缓存 —— `flash_union_buffer` 是全局共享的，
 *  别的模块（app_params / 分步写）随时会把它盖掉，跨调用缓存就是自找脏读。
 *=================================================================================================================*/
uint16 ins_route_slot_read_bytes (uint8 slot, uint32 off, uint8 *dst, uint16 n)
{
    rt_head_t h;
    uint32    base, img_len, pos;
    uint32    last_page = 0xFFFFFFFFu;
    uint16    done = 0u, copy;

    if ((NULL == dst) || (slot >= (uint8)INS_RT_SLOT_COUNT) || (0u == n)) { return 0u; }

    flash_init();
    base = INS_RT_SLOT_BASE(slot);

    flash_read_page_to_buffer(0u, base, (uint32)FLASH_PAGE_LENGTH);
    last_page = base;
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if ((RT_MAGIC != h.magic) || ((uint16)RT_VER != h.ver) ||
        ((uint16)INS_RT_REC_SIZE != h.rec_len) || (0u == h.count) ||
        (h.count > INS_RT_FLASH_CAPACITY))
    {
        return 0u;
    }

    img_len = 36u + (uint32)h.count * (uint32)INS_RT_REC_SIZE;
    if (off >= img_len)              { return 0u; }
    if ((off + (uint32)n) > img_len) { n = (uint16)(img_len - off); }

    pos = off;

    /* --- 头 36 B（就在刚才那一页里）--- */
    if (pos < 36u)
    {
        uint32 take = 36u - pos;

        if (take > (uint32)n) { take = (uint32)n; }
        memcpy(dst, (const uint8 *)flash_union_buffer + pos, take);
        done = (uint16)take;
        pos += take;
    }

    /* --- 记录区 --- */
    while (done < n)
    {
        uint32 rec      = (pos - 36u) / (uint32)INS_RT_REC_SIZE;
        uint32 in_rec   = (pos - 36u) % (uint32)INS_RT_REC_SIZE;
        uint32 page     = rec / (uint32)INS_RT_REC_PER_PAGE;
        uint32 in_page  = (rec % (uint32)INS_RT_REC_PER_PAGE) * (uint32)INS_RT_REC_SIZE + in_rec;

        /* ★★ 2026-10-12 修：本页还剩多少字节，必须**从 pos 算**，不能从"本条记录的开头"算。
           旧写法 `till_end = 2040 - (rec%170)*12` 漏掉了 `in_rec`，于是每个页边界都会：
             ① 多拷 in_rec 个字节（越过 2040 的记录区，读到页内 0xFF 填充）；
             ② 把下一页**开头 in_rec 个字节**整段跳掉（pos 落到 2040+in_rec）。
           现象极具误导性：**镜像长度完全正确、但记录区 CRC 不过**（每个 512 B 帧自身的
           CRC 都过 ⇒ 上位机看不出丢包），而且**只有跨页的表才中招** ——
           157 点的圆环 1884 B 一页装得下（2040 B）永远不跨页 ⇒ 一直是好的；
           1742 点的手推表跨 11 页 ⇒ 必错。
           （旧代码那句 `2048 - in_page` 的二次限幅非但没救回来，反而比正确的
            `2040 - in_page` 松 8 字节，正好放行了那 8 字节页尾填充。） */
        uint32 till_end = (uint32)INS_RT_REC_PER_PAGE * (uint32)INS_RT_REC_SIZE - in_page;

        if (last_page != (base + 1u + page))
        {
            flash_read_page_to_buffer(0u, base + 1u + page, (uint32)FLASH_PAGE_LENGTH);
            last_page = base + 1u + page;
        }

        copy = (uint16)((uint32)n - (uint32)done);
        if ((uint32)copy > till_end) { copy = (uint16)till_end; }

        memcpy(dst + done, (const uint8 *)flash_union_buffer + in_page, copy);
        done += copy;
        pos  += copy;
    }

    return done;
}

/*===================================================================================================================
 *  ★★ 巡航记录（Cruise 实际轨迹）—— 见 ins_route.h 顶部的说明
 *
 *  一句话：巡迹中**只在 RAM 里攒**（2040 × 12 B = 24 KB），停巡后由主循环分步写进
 *  `INS_ROUTE_LOG_SLOT`。巡迹中绝不碰 Flash（擦一页是毫秒级，会顶穿 5 ms 节拍）。
 *=================================================================================================================*/
void ins_route_log_reset (void)
{
    uint16 period_ms = (uint16)((ins_log_ms < 1.0f) ? 1.0f : (ins_log_ms + 0.5f));

    if (period_ms < 1u)    { period_ms = 1u; }
    if (period_ms > 1000u) { period_ms = 1000u; }

    ins_route_rec_begin((uint16)INS_ROUTE_HDR_FL_LOG, period_ms);
}

/*===================================================================================================================
 *  通用 16 B 记录器（见 ins_route.h）
 *      ⚠ 这三个函数是**巡航记录与阶跃测试共用的**：同一块 `s_log[]`、同一条落槽通道，
 *        靠 `flags` 区分"里面那 16 个字节怎么解释"。
 *=================================================================================================================*/
void ins_route_rec_begin (uint16 flags, uint16 sample_ms)
{
    s_log_flags  = flags;
    s_log_per_ms = (0u == sample_ms) ? 1u : sample_ms;
    /* 记录写哪一槽：阶跃测试进 STEP_SLOT，其余（巡航记录）进 LOG_SLOT。
       ⚠ 两者**故意分开**：不然一次阶跃测试会把上一圈的巡航记录覆盖掉。 */
    s_log_slot   = ((0u != (flags & INS_ROUTE_HDR_FL_STEP)) ? (uint8)INS_ROUTE_STEP_SLOT
                                                           : (uint8)INS_ROUTE_LOG_SLOT);

    s_log_n      = 0u;
    s_log_acc_ms = 0u;
    s_log_full   = 0u;
    s_log_arm    = 0u;
    s_log_saving = 0u;
}

void ins_route_rec_push (const uint8 *rec16)
{
    if (NULL == rec16)     { return; }
    if (0u != s_log_full)  { return; }

    if (s_log_n >= (uint32)INS_ROUTE_LOG_MAX)
    {
        s_log_full = 1u;
        return;
    }

    memcpy((void *)&s_log[s_log_n], (const void *)rec16, INS_RT_REC_SIZE);
    s_log_n ++;

    if (s_log_n >= (uint32)INS_ROUTE_LOG_MAX) { s_log_full = 1u; }
}

uint32 ins_route_rec_count  (void) { return s_log_n; }
uint8  ins_route_rec_saving (void) { return s_log_saving; }

uint16 ins_route_log_count  (void) { return (uint16)((s_log_n > 0xFFFFu) ? 0xFFFFu : s_log_n); }
uint8  ins_route_log_full   (void) { return s_log_full; }
uint8  ins_route_log_saving (void) { return s_log_saving; }

/*-------------------------------------------------------------------------------------------------------------------
 *  5 ms 调用一次（由 route_cruise_tick 在"循迹中"那条路径上调）
 *      ⚠ 本函数**只写 RAM**，绝不动 Flash，也绝不阻塞。
 *-----------------------------------------------------------------------------------------------------------------*/
void ins_route_log_tick (float x, float y, float yaw_deg, float servo_duty)
{
    uint16    period_ms;
    rt_rec_t *r;

    if (0u != s_log_full) { return; }

    if (s_log_n >= (uint32)INS_ROUTE_LOG_MAX)
    {
        s_log_full = 1u;
        return;
    }

    /* 采样周期夹一下：参数被写成 0 会导致"每拍都记"，2040 条 10 秒就满了 */
    period_ms = s_log_per_ms;
    if (0u == period_ms)   { period_ms = 1u; }
    if (period_ms > 1000u) { period_ms = 1000u; }

    s_log_acc_ms = (uint16)(s_log_acc_ms + 5u);          /* 本函数由 5 ms 中断调用 */
    if (s_log_acc_ms < period_ms) { return; }
    s_log_acc_ms = 0u;

    r = &s_log[s_log_n];

    r->x_mm       = (int32)(x * 1000.0f);
    r->y_mm       = (int32)(y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(yaw_deg);                 /* ★ 绝对航向（不是相对起点）*/
    r->v_cmps     = (int16)rt_clampf(servo_duty, -32000.0f, 32000.0f);  /* ★ 这一格是**舵机 PWM duty** */
    r->steer_cdeg = 0;                                     /* v2 新列：记录类不用 */
    r->flags      = 0u;

    s_log_n ++;

    if (s_log_n >= (uint32)INS_ROUTE_LOG_MAX) { s_log_full = 1u; }
}

/*-------------------------------------------------------------------------------------------------------------------
 *  "把记录写进槽"分两步，是刻意的：
 *      `ins_route_log_finish()` 可能由**中断**调用（循迹跑完/中止都在 ISR 里发生），
 *      而武装要 `flash_init()` —— 不在中断里做。所以中断只置一个**请求位**，
 *      由主循环的 `ins_route_log_poll()` 真正武装。
 *-----------------------------------------------------------------------------------------------------------------*/
void ins_route_log_finish (void)
{
    if (s_log_n < 10u) { return; }        /* 太短不值得写，也别覆盖上一次的记录 */
    s_log_arm = 1u;
}

/* 与 ins_route_log_finish 同一件事，给"非巡航"的记录用（阶跃测试）*/
void ins_route_rec_end (void)
{
    ins_route_log_finish();
}

void ins_route_log_poll (void)
{
    if (0u == s_log_arm)    { return; }
    if (0u != s_fl_busy)    { return; }   /* 写盘器忙（可能在写路表）⇒ 下一圈再说 */

    s_log_arm    = 0u;
    s_log_saving = 1u;
    flash_arm_log(s_log_slot);
}

/* 手动立刻写（菜单/上位机想强制落盘时用；**不要从中断调**）*/
void ins_route_log_save_now (void)
{
    if (0u != s_fl_busy) { return; }
    if (s_log_n < 10u)   { return; }

    s_log_arm    = 0u;
    s_log_saving = 1u;
    flash_arm_log(s_log_slot);
}
