/*********************************************************************************************************************
 * C 车惯导 + 路表 - 路表录制实现
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stddef.h>
#include "ins_route.h"
#include "ins_odom.h"
#include "ins_imu.h"        /* ins_imu_zupt_frz_reset()：录表起点清冻结计数器 */
#include "steer_ctrl.h"     /* steer_ctrl_get_target_delta()：取当拍舵机目标角 */
#include "route_app.h"      /* rt_revrec：参数 ID 76 倒车录表开关 */
#include "route_pb.h"        /* route_pb_active()：回放中禁止改写 s_wp[] */

#define DEG2RAD_F   (0.017453292519943295f)

/* 转角触发开关：0 = 只按距离取点，置 1 后点距不等且等间距索引会错位 */
#define INS_ROUTE_YAW_TRIG_EN   (0)

static ins_wp_t          s_wp[INS_ROUTE_MAX_POINTS];
/* 校验装载与异步存盘的暂存区 */
static ins_wp_t          s_stage[INS_ROUTE_MAX_POINTS];
static ins_route_head_t  s_head;
static uint32 s_revision = 0u;
static uint8 s_rec_reverse = 0u;

static uint8   s_running   = 0;
static uint8   s_flags     = 0;
static float   s_dist_acc  = 0.0f;      /* 距上一个已存点的累计距离 */
static float   s_yaw_last  = 0.0f;      /* 上一个已存点的相对 yaw，仅转角触发启用时用 */
static float   s_dist_last = 0.0f;      /* 上一次采样的 dist_abs */
static float   s_dist_at_start = 0.0f;  /* 起点处的 dist_abs，用于总里程 */

/* 关键点 / 打角列计数，供菜单与遥测显示 */
static uint16  s_key_cnt   = 0u;
static uint16  s_steer_cnt = 0u;
static uint8   s_traj_mode = 0u;        /* 1 = 打 INS_WP_FL_TRAJ */

/* 运行时点距，参数 ID 39 rt_ds 绑定 */
volatile float ins_route_ds = INS_ROUTE_DS_DEFAULT;

void ins_route_init (void)
{
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
    if (route_pb_active() || ins_route_flash_busy() || s_running) { return; }
    s_revision ++;
    s_head.count      = 0;
    s_head.total_dist = 0.0f;
    s_dist_acc  = 0.0f;
    s_yaw_last  = 0.0f;
    s_flags    &= (uint8)~INS_ROUTE_ST_FULL;
    s_key_cnt   = 0u;
    s_steer_cnt = 0u;
}

/* 标记最后一个已录点；仅录制中有效，返回 1 = 标上，0 = 未录/点太少 */
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
        s_revision ++;
    }
    return 1u;
}

uint16 ins_route_key_count   (void) { return s_key_cnt;   }
uint16 ins_route_steer_count (void) { return s_steer_cnt; }

/* 1 = 之后录的点打实测轨迹标记；0 = 教学表（默认）*/
void ins_route_set_traj (uint8 on) { s_traj_mode = (0u != on) ? 1u : 0u; }

/* 换算到录制起点车体系（a0 = 起点航向）：x' = c*dx + s*dy，y' = -s*dx + c*dy */
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
    s_wp[idx].yaw =  o->yaw - s_head.origin_yaw;    /* 相对起点的增量，不是绝对航向 */
    /* 倒车录表：v 存 |v|，方向只由 INS_WP_FL_REV 表示 */
    s_wp[idx].v   =  fabsf(v_mps);

    /* 打角列记当拍舵机目标角，非实测角；超 ±40° 视为无效 */
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

    if (v_mps < -0.02f) { s_rec_reverse = 1u; }
    else if (v_mps > 0.02f) { s_rec_reverse = 0u; }
    if (s_rec_reverse)                  { fl |= (uint16)INS_WP_FL_REV; }
    if (0u != s_traj_mode)              { fl |= (uint16)INS_WP_FL_TRAJ;}

    s_wp[idx].flags = fl;
}

void ins_route_start (void)
{
    ins_odom_snap_t o;

    if (route_pb_active() || ins_route_flash_busy() || s_running) { return; }
    ins_odom_snapshot(&o);
    s_rec_reverse = (rt_route_mode >= 0.0f) ?
                    ((rt_route_mode == (float)RT_MODE_FZ) ? 1u : 0u) :
                    ((rt_revrec >= 0.5f) ? 1u : 0u);
    s_revision ++;

    /* 重新捕获起点位姿，录制与回放共用同一基准 */
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

    /* 起点记为第 0 点 */
    route_to_local(&o, 0, 0.0f);
    s_head.count = 1;

    /* ZUPT 冻结计数器归零，1 Hz 行的 fz=/fzd= 从这一刻累计 */
    ins_imu_zupt_frz_reset();

    s_running = 1;
}

void ins_route_stop (void)
{
    /* 收笔补停车点；与上一点距离 < 20 mm 不补，速度记 0 */
    if ((0 != s_running) && (s_head.count > 0u) &&
        (s_head.count < INS_ROUTE_MAX_POINTS))
    {
        ins_odom_snap_t o;
        uint16 k = (uint16)(s_head.count - 1u);
        float  dx;
        float  dy;

        ins_odom_snapshot(&o);
        route_to_local(&o, s_head.count, 0.0f);

        dx = s_wp[s_head.count].x - s_wp[k].x;
        dy = s_wp[s_head.count].y - s_wp[k].y;

        if (sqrtf(dx * dx + dy * dy) >= 0.020f)
        {
            s_head.count ++;
            s_head.total_dist = o.dist_abs - s_dist_at_start;
            s_revision ++;
        }
    }

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
        /* 录满置 FULL 位并停录，遥测可见 */
        if (s_running && (s_head.count >= INS_ROUTE_MAX_POINTS))
        {
            s_flags |= INS_ROUTE_ST_FULL;
            s_running = 0;
            s_flags &= (uint8)~INS_ROUTE_ST_RUNNING;
        }
        return;
    }

    ins_odom_snapshot(&o);

    /* 本拍位移：dist_abs 差值，标量单调增，与 16 位回绕无关 */
    ds = o.dist_abs - s_dist_last;
    if (ds < 0.0f) { ds = 0.0f; }
    s_dist_last = o.dist_abs;
    s_dist_acc += ds;

    /* 总里程 = 当前累计里程 − 起点累计里程 */
    s_head.total_dist = o.dist_abs - s_dist_at_start;

    dyaw = (o.yaw - s_head.origin_yaw) - s_yaw_last;
    if (dyaw >  180.0f) { dyaw -= 360.0f; }
    if (dyaw < -180.0f) { dyaw += 360.0f; }

    /* 采样触发：累计距离 ≥ ins_route_ds 时取点；转角触发默认关，
       开启后点距不等，route_cruise 的等间距索引会错位 */
    append = 0;
    if (s_dist_acc >= ins_route_ds) { append = 1; }
    if ((0 != INS_ROUTE_YAW_TRIG_EN) && (fabsf(dyaw) >= INS_ROUTE_YAW_THRESH)) { append = 1; }

    if (append)
    {
        route_to_local(&o, s_head.count, o.v_mps);

        s_head.count ++;
        s_revision ++;

        /* 取点后累计距离清零，不扣阈值：扣减在主循环被拖慢时会连续写入重复点 */
        s_dist_acc = 0.0f;
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
uint32 ins_route_revision (void) { return s_revision; }

const ins_wp_t         *ins_route_get_points (void) { return s_wp; }
const ins_route_head_t *ins_route_get_head   (void) { return &s_head; }

/* ---- 倒车录表 / 倒序回放 ---- */

/* 统计打了 REV 的点数（O(n)，n ≤ 2048） */
uint16 ins_route_rev_count (void)
{
    uint16 i;
    uint16 c = 0u;

    for (i = 0u; i < s_head.count; i ++)
    {
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_REV)) { c ++; }
    }
    return c;
}

/* 旧 ABI 的拒绝入口：永远返回 0，回放不修改原始点序 */
uint8 ins_route_reverse_points (void)
{
    /* Legacy ABI: playback owns a private traversal snapshot. */
    return 0u;
}

/* ---- Flash 持久化 ----
 *   start 武装，poll 每圈写一页；CRC 按写进去的字节算。
 *   整页写只认 flash_union_buffer（2048 B）；flash_buffer_clear() 只清前 512 B。 */

#define RT_MAGIC            (0x4B525443u)       /* CTRK 标识 */
#define RT_VER              (2u)                /* 2 = 16 B/条；其它版本载入时判无效 */

typedef struct __attribute__((packed))
{
    int32   x_mm;           /* 相对录制起点，mm（向右） */
    int32   y_mm;           /* 相对录制起点，mm（向前） */
    int16   yaw_cdeg;       /* 相对起点航向，0.01°（已折到 ±180°）*/
    int16   v_cmps;         /* 车速 cm/s；记录类则解释为舵机 duty */
    int16   steer_cdeg;     /* 当拍舵机目标角，0.01°（无效时 0，看 flags）*/
    uint16  flags;          /* INS_WP_FL_* */
} rt_rec_t;                                     /* = 16 B */

typedef struct __attribute__((packed))
{
    uint32  magic;
    uint16  ver;
    uint16  rec_len;
    uint32  count;
    uint32  ds_mm;          /* 点距 mm（存盘时的 ins_route_ds × 1000）*/
    uint32  lap_mm;         /* 录制总里程 mm */
    int32   origin_x_mm;    /* 起点位姿 */
    int32   origin_y_mm;
    int16   origin_yaw_cdeg;
    uint16  flags;          /* bit0 = 录满 */
    uint16  crc_r;          /* 记录区 CRC16 */
    uint16  crc_h;          /* 表头 CRC16，覆盖 [0, 34) */
} rt_head_t;                                    /* = 36 B（两个 CRC 各 2 B）*/

_Static_assert(sizeof(rt_head_t) == 36u, "rt_head_t 不是 36 B：CRC 覆盖范围会错");
_Static_assert(offsetof(rt_head_t, crc_h) == 34u, "表头布局变了：CRC 覆盖长度要同步改");
_Static_assert(sizeof(rt_rec_t)  == INS_RT_REC_SIZE, "rt_rec_t 不是 16 B：Flash 布局会整体错位");
_Static_assert(INS_RT_REC_PER_PAGE * INS_RT_REC_SIZE <= 2048u, "一页装不下 128 条");

/* 两个结构体天然对齐；插入 uint8/int8 字段时必须重核偏移 */

static uint8   s_fl_busy   = 0u;
static uint8   s_fl_rc     = 0u;        /* 0 未做 / 1 成功 / 2 写失败 / 3 表空 */
static uint8   s_fl_loaded = 0u;        /* 当前 RAM 里的表来自 Flash */
static uint16  s_fl_page   = 0u;        /* 下一个要写的记录页下标（槽内相对） */
static uint16  s_fl_pages  = 0u;        /* 需要写的记录页数 */
static uint16  s_fl_crc    = 0u;        /* 累进 CRC（记录部分）*/
static uint16  s_fl_done   = 0u;        /* 已写页数（含表头那一步），用于百分比 */

/* ---- 多槽 ---- */
static uint8   s_fl_slot   = 0u;        /* 当前操作的槽号 */
static uint32  s_fl_base   = 0u;        /* 本槽头页 = INS_RT_SLOT_BASE(槽号) */
static uint8   s_fl_last_slot = 0u;     /* 最近一次写/读的槽，菜单显示用 */
static uint8   s_active_slot  = 0u;     /* 槽索引里记的当前选中槽 */

/* 分步写盘的数据源与表头快照，武装时定下要写什么 */
static uint8   s_fl_src    = 0u;        /* 0 = s_wp[]（路表） 1 = s_log[]（记录，已是 16 B 打包态）*/
static uint32  s_fl_count  = 0u;        /* 本次要写多少条（快照）*/
static uint32  s_fl_ds_mm  = 0u;        /* 槽头 ds_mm（路表=点距 mm；记录=采样周期 ms）*/
static uint32  s_fl_lap_mm = 0u;        /* 槽头 lap_mm（路表=圈长 mm；记录=总时长 ms）*/
static int32   s_fl_ox_mm  = 0;         /* 槽头 origin（记录恒 0）*/
static int32   s_fl_oy_mm  = 0;
static int16   s_fl_oyaw   = 0;
static uint16  s_fl_flags  = 0u;        /* 槽头 flags（bit1 = 巡航记录）*/
static uint8   s_fl_setact = 1u;        /* 写完是否把该槽设为 active */

/* ---- 巡航记录缓冲 ---- */
static rt_rec_t s_log[INS_ROUTE_LOG_MAX];
static uint32   s_log_n      = 0u;
static uint16   s_log_acc_ms = 0u;      /* 距上次采样累计了多少 ms */
static uint8    s_log_full   = 0u;
static uint8    s_log_arm    = 0u;      /* 1 = 已请求停巡后落盘（中断里只置这一位）*/
static uint8    s_log_saving = 0u;      /* 1 = 已武装/正在写 */
static uint16   s_log_flags  = (uint16)INS_ROUTE_HDR_FL_LOG;   /* 这份记录的类型 */
static uint16   s_log_per_ms = 20u;     /* 写入槽头的采样周期（巡航记录可调，阶跃测试恒 5）*/
static uint8    s_log_slot   = (uint8)INS_ROUTE_LOG_SLOT;      /* 写进哪一槽（按类型自动定）*/

volatile float  ins_log_ms   = 20.0f;   /* 采样周期 ms（参数 ID 38）*/

/* 槽索引页：active_slot + 每槽摘要，页号见 INS_RT_INDEX_PAGE */
#define RT_IDX_MAGIC    (0x58495243u)   /* CRIX 标识 */
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

/* CRC 覆盖长度由槽数算出，不写死数字 */
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

/* 正负对称四舍五入取整，避免反复存载累积漂移 */
static int32 rt_round_i32 (float v)
{
    return (int32)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* yaw 折到 ±180°（表里存的是相对起点航向，这里只兜底） */
static int16 rt_wrap_cdeg (float yaw_deg)
{
    float y = yaw_deg;

    while (y >  180.0f) { y -= 360.0f; }
    while (y < -180.0f) { y += 360.0f; }

    return (int16)rt_round_i32(y * 100.0f);
}

static float rt_clampf (float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static void rt_rec_pack (const ins_wp_t *wp, rt_rec_t *r)
{
    r->x_mm       = rt_round_i32(wp->x * 1000.0f);
    r->y_mm       = rt_round_i32(wp->y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(wp->yaw);
    r->v_cmps     = (int16)rt_round_i32(rt_clampf(wp->v * 100.0f, -32000.0f, 32000.0f));
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

/* 槽索引页：load 读回（校验不过则为全空默认值），store 整页重写 */
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

/* 武装一次分步写盘（路表 / 记录共用），只快照要写的内容
 *   setact：路表写完设为 active；记录必须为 0，否则上电会把记录当路表载入 */
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
    if (ins_route_flash_busy() || s_running) { return; }
    memcpy(s_stage, s_wp, (size_t)s_head.count * sizeof(s_wp[0]));
    flash_arm(slot, 0u, (uint32)s_head.count,
              (uint32)(ins_route_ds * 1000.0f + 0.5f),
              (uint32)(s_head.total_dist * 1000.0f + 0.5f),
              (int32)(s_head.origin_x * 1000.0f),
              (int32)(s_head.origin_y * 1000.0f),
              rt_wrap_cdeg(s_head.origin_yaw),
              (uint16)((0u != (s_flags & INS_ROUTE_ST_FULL)) ? 1u : 0u),
              1u);                                  /* 路表：写完设为 active */
}

/* 把记录写进槽（数据源 s_log[]，setact = 0 不抢 active） */
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
    ins_route_flash_save_slot(0u);          /* 手推录制固定写槽 0 */
}

void ins_route_flash_poll (void)
{
    uint8  *pg = (uint8 *)flash_union_buffer;

    if (0u == s_fl_busy) { return; }

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

                rt_rec_pack(&s_stage[idx], &r);
                memcpy(pg + (uint32)k * INS_RT_REC_SIZE, &r, INS_RT_REC_SIZE);
            }
            else
            {
                /* 记录已是 16 B 打包态，直接搬 */
                memcpy(pg + (uint32)k * INS_RT_REC_SIZE, (const void *)&s_log[idx], INS_RT_REC_SIZE);
            }
        }

        /* CRC 只算真正写进去的字节，尾部 0xFF 填充不参与 */
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

        /* 读回校验：先把期望值拷出来，读回用的是同一个 flash_union_buffer */
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

    /* 记录写完，最后写表头页 */
    {
        rt_head_t h;

        memset(&h, 0, sizeof(h));
        h.magic     = RT_MAGIC;
        h.ver       = (uint16)RT_VER;
        h.rec_len   = (uint16)INS_RT_REC_SIZE;
        h.count     = s_fl_count;                 /* 武装时的快照：路表=点数，记录=条数 */
        h.ds_mm     = s_fl_ds_mm;                 /* 路表=点距 mm；记录=采样周期 ms */
        h.lap_mm    = s_fl_lap_mm;                /* 路表=圈长 mm；记录=总时长 ms */
        h.origin_x_mm     = s_fl_ox_mm;
        h.origin_y_mm     = s_fl_oy_mm;
        h.origin_yaw_cdeg = s_fl_oyaw;
        h.flags     = s_fl_flags;                 /* bit1 = 巡航记录 */

        h.crc_r = s_fl_crc;                          /* 写页时已累进算好 */
        h.crc_h = rt_crc16(0xFFFFu, (const uint8 *)&h, 34u);   /* 覆盖 [0,34) = magic..crc_r */

        memset((void *)flash_union_buffer, 0xFF, 2048u);
        memcpy((void *)flash_union_buffer, &h, sizeof(h));

        flash_erase_page(0u, s_fl_base);
        (void)flash_write_page_from_buffer(0u, s_fl_base, (uint32)FLASH_PAGE_LENGTH);

        /* 读回校验表头：写失败多为掉电或擦除未完 */
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

        /* 更新槽索引：写本槽摘要，只有路表（s_fl_setact = 1）才把 active 指到本槽 */
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

/* 遍历某槽记录区：do_write = 0 只算 CRC，= 1 展开到 s_stage[]
   一次页读处理 INS_RT_REC_PER_PAGE 条；返回 1 = 记录区 CRC 与槽头一致。 */
static uint8 flash_walk_records (uint32 base, uint32 n, uint16 want_crc, uint8 do_write)
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

        if (0u != do_write)
        {
            for (k = 0; k < left; k ++)
            {
                rt_rec_t r;

                memcpy(&r, (const uint8 *)flash_union_buffer + k * INS_RT_REC_SIZE,
                       INS_RT_REC_SIZE);

                s_stage[done + k].x          = (float)r.x_mm / 1000.0f;
                s_stage[done + k].y          = (float)r.y_mm / 1000.0f;
                s_stage[done + k].yaw        = (float)r.yaw_cdeg / 100.0f;
                s_stage[done + k].v          = (float)r.v_cmps / 100.0f;
                s_stage[done + k].steer_cdeg = r.steer_cdeg;
                s_stage[done + k].flags      = r.flags;
            }
        }

        done += left;
    }

    return (uint8)((crc_r == want_crc) ? 1u : 0u);
}

uint8 ins_route_flash_load_slot (uint8 slot)
{
    rt_head_t h;
    uint32    n;
    uint32    base;

    if (route_pb_active() || s_running || s_fl_busy) { return 0u; }

    if (slot >= (uint8)INS_RT_SLOT_COUNT) { return 0u; }
    base = INS_RT_SLOT_BASE(slot);

    flash_init();
    flash_read_page_to_buffer(0u, base, (uint32)FLASH_PAGE_LENGTH);
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if (RT_MAGIC != h.magic)                       { return 0u; }   /* 从没存过 */
    if ((uint16)RT_VER != h.ver)                   { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != h.rec_len)      { return 0u; }
    /* 拒绝记录类槽头（LOG/STEP）：记录第 4 字段是舵机 duty，不能当路表载入 */
    if (0u != (h.flags & (uint16)(INS_ROUTE_HDR_FL_LOG | INS_ROUTE_HDR_FL_STEP))) { return 0u; }
    if (0u == h.count)                             { return 0u; }
    if (h.count > INS_RT_FLASH_CAPACITY)           { return 0u; }
    if (h.count > (uint32)INS_ROUTE_MAX_POINTS)    { return 0u; }
    if ((h.ds_mm < (uint32)(INS_ROUTE_DS_MIN * 1000.0f)) ||
        (h.ds_mm > (uint32)(INS_ROUTE_DS_MAX * 1000.0f))) { return 0u; }
    if (rt_crc16(0xFFFFu, (const uint8 *)&h, 34u) != h.crc_h) { return 0u; }

    n = h.count;

    /* 两遍装载：第一遍只验 CRC，通过后展开；失败时 s_wp[] 不变 */
    if (0u == flash_walk_records(base, n, h.crc_r, 0u)) { return 0u; }
    if (0u == flash_walk_records(base, n, h.crc_r, 1u)) { return 0u; }
    memcpy(s_wp, s_stage, (size_t)n * sizeof(s_wp[0]));
    ins_route_ds = (float)h.ds_mm / 1000.0f;
    s_revision ++;

    s_head.origin_x   = (float)h.origin_x_mm / 1000.0f;
    s_head.origin_y   = (float)h.origin_y_mm / 1000.0f;
    s_head.origin_yaw = (float)h.origin_yaw_cdeg / 100.0f;
    s_head.count      = (uint16)n;
    s_head.version    = 1u;
    s_head.total_dist = (float)h.lap_mm / 1000.0f;

    s_running = 0u;
    s_flags   = 0u;
    if (0u != (h.flags & 1u)) { s_flags |= INS_ROUTE_ST_FULL; }

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
    return ins_route_flash_load_slot(s_active_slot);
}


/* ---- 从内存镜像载入路表 ----
 *   与 flash_load 同一字节格式：36 B 头 + 16 B/点 + 两个 CRC16-MODBUS，来源为连续内存。 */
uint8 ins_route_load_image (const uint8 *img, uint32 len)
{
    uint32 magic, count, ds_mm, lap_mm, need, i;
    uint16 ver, rec_len, crc_r, crc_h, flags;
    int32  ox, oy;
    int16  oyaw;

    /* 回放中禁止改写 s_wp[]：回放的点序倒置是原地对调 */
    if (route_pb_active() || s_running || s_fl_busy) { return 0u; }

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

    /* ---- 与 flash_load 一致的准入条件 ---- */
    if (RT_MAGIC != magic)                        { return 0u; }
    if ((uint16)RT_VER != ver)                    { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != rec_len)       { return 0u; }
    if (0u == count)                              { return 0u; }
    if (count > (uint32)INS_ROUTE_MAX_POINTS)     { return 0u; }
    if ((ds_mm < (uint32)(INS_ROUTE_DS_MIN * 1000.0f)) ||
        (ds_mm > (uint32)(INS_ROUTE_DS_MAX * 1000.0f))) { return 0u; }
    /* 记录类镜像（LOG/STEP）不是路表 */
    if (0u != (flags & (uint16)(INS_ROUTE_HDR_FL_LOG | INS_ROUTE_HDR_FL_STEP))) { return 0u; }
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

    /* 统计带打角 / 关键点的点数，回放前菜单可见 */
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

    s_fl_loaded = 0u;               /* 非 Flash 来源：上位机提示为内建/下发的表 */
    ins_route_ds = (float)ds_mm / 1000.0f;
    s_revision ++;

    return 1u;
}

/* ---- 多槽接口 ---- */
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
    if (route_pb_active() || s_running || s_fl_busy) { return 1u; }

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

uint8 ins_route_slot_select (uint8 slot, uint8 persist)
{
    if (0u == ins_route_flash_load_slot(slot)) { return 0u; }
    return (uint8)(ins_route_slot_set_active(slot, persist) == 0u);
}

/* 槽摘要：索引有效就用索引，否则现读该槽头页 */
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

/* 从内存镜像写进指定槽：先校验并展开进 RAM，再分步落盘
   返回 0 = 已武装；1 = 槽越界；2 = 镜像不过；3 = 无法武装 */
uint8 ins_route_slot_load_image (uint8 slot, const uint8 *img, uint32 len)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT)     { return 1u; }
    if (0u == ins_route_load_image(img, len)) { return 2u; }

    ins_route_flash_save_slot(slot);
    if (0u == s_fl_busy)                      { return 3u; }
    return 0u;
}

/* ---- 按镜像偏移读字节，串口上传（车 → PC）用 ----
 *   镜像 = 36 B 头 + N×16 B 记录：off < 36 取头页，否则按 rec=(off-36)/16 换算记录页。
 *   只在页号变化时重读；不做跨调用页缓存（flash_union_buffer 全局共享）。 */
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

    /* --- 头 36 B，已在首页中 --- */
    if (pos < 36u)
    {
        uint32 take = 36u - pos;

        if (take > (uint32)n) { take = (uint32)n; }
        memcpy(dst, (const uint8 *)flash_union_buffer + pos, take);
        done = (uint16)take;
        pos += take;
    }

    while (done < n)
    {
        uint32 rec      = (pos - 36u) / (uint32)INS_RT_REC_SIZE;
        uint32 in_rec   = (pos - 36u) % (uint32)INS_RT_REC_SIZE;
        uint32 page     = rec / (uint32)INS_RT_REC_PER_PAGE;
        uint32 in_page  = (rec % (uint32)INS_RT_REC_PER_PAGE) * (uint32)INS_RT_REC_SIZE + in_rec;

        /* 本页剩余字节从 pos 算起（含 in_rec），否则跨页会多拷并跳过 */
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

/* ---- 巡航记录（实际轨迹）----
 *   巡迹中只在 RAM 里攒（2048 × 16 B = 32 KB），停巡后由主循环分步写进 INS_ROUTE_LOG_SLOT。 */
void ins_route_log_reset (void)
{
    uint16 period_ms = (uint16)((ins_log_ms < 1.0f) ? 1.0f : (ins_log_ms + 0.5f));

    if (period_ms < 1u)    { period_ms = 1u; }
    if (period_ms > 1000u) { period_ms = 1000u; }

    ins_route_rec_begin((uint16)INS_ROUTE_HDR_FL_LOG, period_ms);
}

/* 通用 16 B 记录器：巡航记录与阶跃测试共用 s_log[]，flags 区分解释 */
void ins_route_rec_begin (uint16 flags, uint16 sample_ms)
{
    s_log_flags  = flags;
    s_log_per_ms = (0u == sample_ms) ? 1u : sample_ms;
    /* 阶跃测试进 STEP_SLOT，其余（巡航记录）进 LOG_SLOT，两类互不覆盖 */
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

/* 5 ms 调用一次，由 route_cruise_tick 在循迹路径上调；只写 RAM，不碰 Flash，不阻塞 */
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

    /* 采样周期夹到 1..1000 ms，参数写成 0 会变成每拍都记 */
    period_ms = s_log_per_ms;
    if (0u == period_ms)   { period_ms = 1u; }
    if (period_ms > 1000u) { period_ms = 1000u; }

    s_log_acc_ms = (uint16)(s_log_acc_ms + 5u);
    if (s_log_acc_ms < period_ms) { return; }
    s_log_acc_ms = 0u;

    r = &s_log[s_log_n];

    r->x_mm       = (int32)(x * 1000.0f);
    r->y_mm       = (int32)(y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(yaw_deg);                 /* 绝对航向，不是相对起点 */
    r->v_cmps     = (int16)rt_clampf(servo_duty, -32000.0f, 32000.0f);  /* 这一格存舵机 PWM duty */
    r->steer_cdeg = 0;                                     /* 记录类不用 */
    r->flags      = 0u;

    s_log_n ++;

    if (s_log_n >= (uint32)INS_ROUTE_LOG_MAX) { s_log_full = 1u; }
}

/* 落盘分两步：中断只置请求位，由主循环武装（武装需 flash_init） */
void ins_route_log_finish (void)
{
    if (s_log_n < 10u) { return; }        /* 太短不写，也不覆盖上一次的记录 */
    s_log_arm = 1u;
}

/* 同 ins_route_log_finish，供非巡航记录（阶跃测试）调用 */
void ins_route_rec_end (void)
{
    ins_route_log_finish();
}

void ins_route_log_poll (void)
{
    if (0u == s_log_arm)    { return; }
    if (0u != s_fl_busy)    { return; }   /* 写盘器忙则下一圈再试 */

    s_log_arm    = 0u;
    s_log_saving = 1u;
    flash_arm_log(s_log_slot);
}

/* 立即落盘（菜单/上位机强制保存用），不要从中断调用 */
void ins_route_log_save_now (void)
{
    if (0u != s_fl_busy) { return; }
    if (s_log_n < 10u)   { return; }

    s_log_arm    = 0u;
    s_log_saving = 1u;
    flash_arm_log(s_log_slot);
}
