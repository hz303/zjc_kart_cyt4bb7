/*********************************************************************************************************************
 * C 车惯导 + 路表 - 路表录制（米制路线图）
 *
 *   点结构 waypoint{x, y, yaw, v}，坐标转到录制起点车体系（x 向右、y 向前），
 *   yaw 取相对起点的增量；单槽 2048 点 × 16 B，采样只按距离，点距为运行时参数。
 ********************************************************************************************************************/

#ifndef _INS_ROUTE_H
#define _INS_ROUTE_H

#include "zf_common_headfile.h"
#include "flash_map.h"          /* 页号/记录长度唯一权威（FM_SLOT_*）*/

/* ---- 配置 ---- */
#define INS_ROUTE_MAX_POINTS    (INS_RT_SLOT_CAPACITY)      /* = 2048，单槽容量 */
#define INS_ROUTE_DS_DEFAULT    (0.10f)         /* 米：默认点距 */
#define INS_ROUTE_DS_MIN        (0.02f)
#define INS_ROUTE_DS_MAX        (0.20f)
#define INS_ROUTE_YAW_THRESH    (2.0f)          /* 度：转角触发阈值，默认不启用 */

/* 运行时点距，参数 ID 39 rt_ds 绑定；录制中改它只影响之后的点 */
extern volatile float   ins_route_ds;

/* ---- 数据结构 ---- */
typedef struct
{
    float x;            /* 米，录制起点车体系：x 向右 */
    float y;            /* 米，                y 向前 */
    float yaw;          /* 度，相对录制起点（不是绝对航向）*/
    float v;            /* m/s，录制瞬间车速 */
    int16 steer_cdeg;   /* 当拍舵机目标角，0.01° */
    uint16 flags;       /* 逐点标记，见 INS_WP_FL_* */
} ins_wp_t;

/* 逐点 flags（16 B 记录最后一个字段）：bit0 是打角有效位，不能用打角值是否为 0 代替 */
#define INS_WP_FL_STEER     (0x0001u)   /* 本点 steer_cdeg 有效            */
#define INS_WP_FL_KEY       (0x0002u)   /* 关键点（人开车时打出来的）      */
#define INS_WP_FL_REV       (0x0004u)   /* 录制时为倒车段（v < 0）          */
#define INS_WP_FL_TRAJ      (0x0008u)   /* 本份是实测轨迹（不是教学表）      */

typedef struct
{
    float   origin_x;       /* 录制起点的 odom 坐标（米）  */
    float   origin_y;
    float   origin_yaw;     /* 录制起点的航向（度）        */
    uint16  count;          /* 已录点数                    */
    uint16  version;        /* 格式版本（改布局必须加 1）  */
    float   total_dist;     /* 录制期间总里程（米）        */
} ins_route_head_t;

/* ---- 状态位（上报到遥测 ins_flags）---- */
#define INS_ROUTE_ST_RUNNING    (0x01u)     /* 正在录制 */
#define INS_ROUTE_ST_FULL       (0x02u)     /* 录满了，已停录 */

/* ---- 接口 ---- */
void    ins_route_init      (void);

/* 开始/停止录制。start 会重新捕获起点位姿并清空缓冲。 */
void    ins_route_start     (void);
void    ins_route_stop      (void);

/* 主循环轮询取点；阈值按距离，换拍率不改路径形状 */
void    ins_route_poll      (void);

void    ins_route_clear     (void);         /* 只清缓冲，不含起点 */

uint8   ins_route_is_running(void);
uint8   ins_route_flags     (void);
uint16  ins_route_get_count (void);
float   ins_route_get_total_dist (void);
uint32  ins_route_revision (void);

/* ---- 关键点 / 打角列接口：逐点记当拍舵机目标角，mark_key 只标记最后一个已录点 ---- */
uint8   ins_route_mark_key   (void);        /* 1 = 标上了；0 = 没在录/点太少 */
uint16  ins_route_key_count  (void);        /* 本份记录里的关键点数 */
uint16  ins_route_steer_count(void);        /* 本份记录里带有效打角的点数 */
/* 1 = 之后录的点打实测轨迹标记；0 = 教学表（默认）*/
void    ins_route_set_traj   (uint8 on);

const ins_wp_t          *ins_route_get_points (void);
const ins_route_head_t  *ins_route_get_head   (void);

/* ---- 倒车录表 / 倒序回放：rev_count 统计打 REV 的点数（录制时 v<0），
 *   reverse_points 是旧 ABI 的拒绝入口，永远返回 0，回放不改原始表 ---- */
uint16  ins_route_rev_count      (void);
uint8   ins_route_reverse_points (void);

/* ---- Flash 持久化：多槽布局（页号唯一权威是 flash_map.h 的 FM_SLOT_*）----
 *   page 0 参数页（app_params，不可写）｜page 1 槽索引（active_slot + 摘要 + CRC）
 *   page 2..86 槽 0~4，每槽 17 页 = 1 头页 + 16 记录页；page 87..95 空闲
 *   槽 0 手推/人开车录制（带打角，灌表协议禁写）｜槽 1 电子·正式
 *   槽 2 电子·调试｜槽 3 实测轨迹 A｜槽 4 实测轨迹 B
 *   每槽 16 × 128 = 2048 条，16 B/条；50 mm 点距 ⇒ 102 m。
 *
 *   记录格式（16 B，与上位机 dump 帧逐字节相同）：
 *     int32 x_mm | int32 y_mm | int16 yaw_cdeg | int16 v_cmps
 *     int16 steer_cdeg | uint16 flags
 *   steer_cdeg 是当拍实际施加的舵机目标角，回放前馈用；无打角时 flags.bit0 = 0。
 *
 *   擦写一页 2 KB 是阻塞操作，连写 17 页会顶穿心跳门 200 ms，
 *   故拆成武装 + 主循环每圈写一页（ins_route_flash_poll）。 */
#define INS_RT_SLOT_COUNT       (FM_SLOT_COUNT)         /* 5 个槽（0~4）                    */
#define INS_RT_SLOT_PAGES       (FM_SLOT_PAGES)         /* 17 = 1 头页 + 16 记录页          */
#define INS_RT_FLASH_REC_PAGES  (FM_SLOT_REC_PAGES)     /* 16 */
#define INS_RT_REC_SIZE         (FM_REC_SIZE)           /* 16 B                             */
#define INS_RT_REC_PER_PAGE     (FM_REC_PER_PAGE)       /* 128 */
#define INS_RT_SLOT_CAPACITY    (FM_SLOT_CAPACITY)      /* = 2048 条                        */
#define INS_RT_FLASH_CAPACITY   (INS_RT_SLOT_CAPACITY)  /* 兼容旧名 */
#define INS_RT_INDEX_PAGE       (FM_SLOT_INDEX_PAGE)    /* = 1                              */
#define INS_RT_FLASH_HEAD_PAGE  (FM_SLOT_BASE)          /* 兼容旧名 = 槽 0 头页             */
#define INS_RT_FLASH_REC_FIRST  (FM_SLOT_BASE + 1u)     /* 兼容旧名 = 槽 0 首记录页         */
#define INS_RT_SLOT_BASE(k)     (FM_SLOT_BASE_PAGE(k))
/* 一张表镜像的最大字节（36 B 头 + 记录）—— 下发通道用它做上限 */
#define INS_RT_SLOT_MAX_BYTES   (36u + (uint32)INS_RT_SLOT_CAPACITY * (uint32)INS_RT_REC_SIZE)

/* 上电时从 Flash 恢复，必须在 5 ms 中断打开之前调；返回 1 = 成功，0 = 无表/损坏 */
uint8   ins_route_flash_load        (void);

/* ---- 多槽接口 ---- */
/* 上电时从槽索引读回 active_slot（不载表；索引不存在时默认 0） */
void    ins_route_slot_init         (void);
uint8   ins_route_slot_active       (void);     /* 当前选中的槽（0..INS_RT_SLOT_COUNT-1）*/
/* 选槽。persist=1 时写索引页（掉电保持）。返回 0 = 成功 */
uint8   ins_route_slot_set_active   (uint8 slot, uint8 persist);
/* 先校验并载入，成功后才发布 active 槽。返回 1 = 成功 */
uint8   ins_route_slot_select       (uint8 slot, uint8 persist);
/* 载入指定槽到 RAM（1 = 成功） */
uint8   ins_route_flash_load_slot   (uint8 slot);
/* 把当前 RAM 里的表写进指定槽（分步；slot=0 即槽 0） */
void    ins_route_flash_save_slot   (uint8 slot);
/* 从内存镜像写入指定槽：先校验、展开进 RAM，再分步落盘。返回 0 = 已武装 */
uint8   ins_route_slot_load_image   (uint8 slot, const uint8 *img, uint32 len);

/* 槽摘要（给 PC 列表用）。valid=1 表示该槽有可用表 */
typedef struct
{
    uint8   valid;
    uint8   slot;
    uint16  tag;            /* 0 = 未标记，预留 */
    uint32  count;
    uint32  lap_mm;
    uint32  ds_mm;
    uint32  crc_r;
} ins_route_slot_info_t;
uint8   ins_route_slot_info         (uint8 slot, ins_route_slot_info_t *out);

/* 按镜像偏移从某槽读字节，串口上传用（36 B 头 + N×16 B 记录）；返回实读字节数，0 = 失败 */
uint16  ins_route_slot_read_bytes   (uint8 slot, uint32 off, uint8 *dst, uint16 n);

/* 分步存盘：start 武装 → poll 每圈写一页 → 全部写完自动收尾 */
void    ins_route_flash_save_start  (void);     /* 写槽 0 */
void    ins_route_flash_poll        (void);     /* 主循环每圈调一次，空闲时立刻返回 */
uint8   ins_route_flash_busy        (void);
uint8   ins_route_flash_last_rc     (void);     /* 0 = 未做；1 = 成功；2 = 页写失败；3 = 表空 */
uint8   ins_route_flash_progress    (void);     /* 0~100（含表头那一步）*/
uint8   ins_route_flash_last_slot   (void);     /* 最近一次写/读的槽号，菜单显示用 */

/* 表来自 Flash 的标记，菜单/上位机据此区分上次存的表与刚录的表 */
uint8   ins_route_flash_loaded      (void);

/* 把 [first, first+max_n) 的记录按 16 B 打包写进 dst，返回实际条数；dst ≥ max_n*16 B */
uint16  ins_route_pack_records      (uint16 first, uint8 *dst, uint16 max_n);

/* ---- 内建（理论）路表 ----
 *   把 tools/route_to_carbin.py 生成的理论表编进固件，上电时补进指定槽。
 *   语义：槽是空的才写入，在 ins_route_flash_load() 之前调用，不覆盖用户表；
 *   写槽会把该槽设为 active，故最后写下的那张成为上电默认表。
 *   加一张表：生成 .c 拷进本目录、加进 CM7_0 工程、注册表加一行并改 cnt。
 *   槽位：槽 0 手推｜槽 1 电子·正式｜槽 2 电子·调试｜槽 3 轨迹 A｜槽 4 轨迹 B */
#define INS_ROUTE_BUILTIN_EN            (1)
#define INS_ROUTE_BUILTIN_SLOT_CIRCLE   (FM_SLOT_MAP_DEBUG)     /* R=500 mm 圆环 */
#define INS_ROUTE_BUILTIN_SLOT_TRACK    (FM_SLOT_MAP_RACE)      /* 正式赛道理论中线 */

/* 一条内建表记录。len 用指针：生成物里是 const uint32，不能直接写进静态初始化式。 */
typedef struct
{
    uint8         slot;     /* 补进哪一槽 */
    const char   *name;     /* 日志用 */
    const uint8  *img;      /* 车端镜像（36 B 头 + N×16 B 记录）*/
    const uint32 *len;      /* -> 生成物的 <cname>_len */
} ins_route_builtin_t;

extern const ins_route_builtin_t ins_route_builtin_tbl[];
extern const uint32              ins_route_builtin_cnt;

/* 从内存镜像载入（与 ins_route_flash_load 同一套字节格式；也给以后的下发通道用） */
uint8   ins_route_load_image        (const uint8 *img, uint32 len);

/* ---- 巡航记录（实际轨迹）----
 *   把 Cruise 跑完后的实际轨迹与舵机指令存进一个槽，复用路表记录布局：
 *   第 4 字段（v_cmps）改存舵机 PWM duty，槽头 flags bit1 标记为记录类。
 *   记录 16 B：x_mm | y_mm | yaw_cdeg | servo_duty | 0 | 0；yaw 是绝对航向；
 *   车速由相邻两点反推（v ≈ Δpos / Δt）；槽头 ds_mm 存采样周期 ms，lap_mm 存总时长 ms。
 *   巡迹中只写 RAM（2048 × 16 B = 32 KB），停巡后由主循环分步落盘。 */
#define INS_ROUTE_LOG_SLOT      (FM_SLOT_TRAJ_A)        /* 巡航/实测轨迹写进槽 3 */
#define INS_ROUTE_LOG_MAX       (INS_RT_FLASH_CAPACITY) /* 2048 条 = 单槽容量 */

#define INS_ROUTE_HDR_FL_FULL   (0x0001u)   /* 槽头 flags bit0：录/记满了 */
#define INS_ROUTE_HDR_FL_LOG    (0x0002u)   /* 槽头 flags bit1：巡航记录，第 4 字段 = 舵机 duty */
#define INS_ROUTE_HDR_FL_STEP   (0x0004u)   /* 槽头 flags bit2：转向阶跃测试（见 steer_step.h）*/

/* ---- 通用 16 B 记录器：巡航记录与阶跃测试共用 s_log[2048] 与同一条落槽通道，
 *   只有这 16 字节的解释不同，靠槽头 flags 的 bit1/bit2 区分 ---- */
void    ins_route_rec_begin (uint16 flags, uint16 sample_ms);   /* 开始一份记录（清计数、定类型）*/
void    ins_route_rec_push  (const uint8 *rec16);               /* 中断里调：只写 RAM，不阻塞 */
void    ins_route_rec_end   (void);                             /* 收尾：只置请求落盘位 */
uint32  ins_route_rec_count (void);
uint8   ins_route_rec_saving(void);

/* 阶跃测试写进槽 4，与轨迹记录二选一：同时用会互相覆盖 */
#define INS_ROUTE_STEP_SLOT     (FM_SLOT_TRAJ_B)

/* 采样周期 ms，参数 ID 38 现场可改：20 ms = 50 Hz（默认），2048 条约 41 s */
extern volatile float   ins_log_ms;

void    ins_route_log_reset   (void);       /* 开始一次记录（route_cruise_start 调）*/
void    ins_route_log_finish  (void);       /* 停巡/跑完/中止时调：只置请求位 */
void    ins_route_log_tick    (float x, float y, float yaw_deg, float servo_duty);
uint16  ins_route_log_count   (void);
uint8   ins_route_log_full    (void);
uint8   ins_route_log_saving  (void);       /* 1 = 已排队/正在写进槽 */
void    ins_route_log_poll    (void);       /* 主循环每圈调一次，把排队落实成分步写 */

/* 把当前记录写进 INS_ROUTE_LOG_SLOT（不在中断里调：内部要 flash_init）*/
void    ins_route_log_save_now (void);

#endif
