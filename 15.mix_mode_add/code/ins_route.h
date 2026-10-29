/* ----------------------------------------------------------------------------------------------------------------
 * 路表录制（米制路线图）：点结构、上限宏、Flash 多槽布局与对外接口
 * ---------------------------------------------------------------------------------------------------------------- */

#ifndef _INS_ROUTE_H
#define _INS_ROUTE_H

#include "zf_common_headfile.h"
#include "flash_map.h"          /* 页号与记录长度以 FM_SLOT_* 为准 */

/* ---- 配置 ---- */

#define INS_ROUTE_MAX_POINTS    (INS_RT_SLOT_CAPACITY)      /* 最大点数 = 2048（单槽容量）*/
#define INS_ROUTE_DS_DEFAULT    (0.10f)         /* 米：默认点距 */
#define INS_ROUTE_DS_MIN        (0.02f)         /* 米：点距下限 */
#define INS_ROUTE_DS_MAX        (0.20f)         /* 米：点距上限 */
#define INS_ROUTE_YAW_THRESH    (2.0f)          /* 度：转角触发阈值，默认不启用 */

/* 运行时点距，参数 ID 39 绑定；录制中修改只影响之后的点 */
extern volatile float   ins_route_ds;

/* ---- 数据结构 ---- */

typedef struct
{
    float x;            /* 米，录制起点车体系：x 向右 */
    float y;            /* 米，同系：y 向前 */
    float yaw;          /* 度，相对录制起点的增量，非绝对航向 */
    float v;            /* m/s，车速（取绝对值）*/
    int16 steer_cdeg;   /* 录制瞬间施加的舵机目标角，0.01° */
    uint16 flags;       /* 逐点标记，见 INS_WP_FL_* */
} ins_wp_t;

/* ---- 逐点 flags（16 B 记录最后一个字段）---- */
/* bit0 为打角有效位：直行时打角本就是 0，不能用打角等于 0 代替这一位 */
#define INS_WP_FL_STEER     (0x0001u)   /* 本点 steer_cdeg 有效 */
#define INS_WP_FL_KEY       (0x0002u)   /* 关键点，人开车时按键标出 */
#define INS_WP_FL_REV       (0x0004u)   /* 录制时为倒车段（v < 0）*/
#define INS_WP_FL_TRAJ      (0x0008u)   /* 本份为实测轨迹，非教学表 */

/* 表头：origin_* 为录制起点的 odom 位姿，仅作诊断，回放不用 */
typedef struct
{
    float   origin_x;       /* 起点 odom 坐标，米 */
    float   origin_y;
    float   origin_yaw;     /* 起点航向，度 */
    uint16  count;          /* 已录点数 */
    uint16  version;        /* 格式版本，改布局必须加 1 */
    float   total_dist;     /* 录制总里程，米 */
} ins_route_head_t;

/* ---- 状态位（上报到遥测 ins_flags）---- */

#define INS_ROUTE_ST_RUNNING    (0x01u)
#define INS_ROUTE_ST_FULL       (0x02u)     /* 已录满，点数达上限后停录 */

/* ---- 接口 ---- */

void    ins_route_init      (void);         /* 点距夹到合法区间，清表头与统计 */

/* 开始/停止录制；start 重新捕获起点位姿并清空缓冲，回放或写盘中调用无效 */
void    ins_route_start     (void);
void    ins_route_stop      (void);

/* 主循环轮询：按距离累计决定是否采样，录满后置 FULL 并停录 */
void    ins_route_poll      (void);

void    ins_route_clear     (void);         /* 只清缓冲，不含起点 */

/* 状态查询：是否在录、状态位、点数、总里程、版本号（内容变化即加 1）*/
uint8   ins_route_is_running(void);
uint8   ins_route_flags     (void);
uint16  ins_route_get_count (void);
float   ins_route_get_total_dist (void);
uint32  ins_route_revision (void);

/* ---- 关键点与打角：mark_key 标最后一个已录点，只置位不做 IO ---- */
uint8   ins_route_mark_key   (void);        /* 1 = 标上，0 = 未录制或点数不足 */
uint16  ins_route_key_count  (void);        /* 本表关键点数 */
uint16  ins_route_steer_count(void);        /* 本表有效打角点数 */
void    ins_route_set_traj   (uint8 on);    /* 1 = 之后的点打实测轨迹标记，0 = 教学表 */

/* 点数组与表头只读指针 */
const ins_wp_t          *ins_route_get_points (void);
const ins_route_head_t  *ins_route_get_head   (void);

/* ---- 倒车录表 / 倒序回放：REV 点数过半即判定为倒着录的表 ---- */
uint16  ins_route_rev_count      (void);
/* 旧 ABI 的拒绝入口，恒返回 0；回放不修改原始表 */
uint8   ins_route_reverse_points (void);

/* ---- Flash 多槽布局（页号以 flash_map.h 的 FM_SLOT_* 为准）----
 *   page 0       : 参数页（app_params），不可改写
 *   page 1       : 槽索引（active_slot + 每槽摘要 + CRC）
 *   page 2..86   : 槽 0~4，每槽 17 页 = 1 头页 + 16 记录页
 *       槽 0 手推录制（带打角，灌表协议禁写）  槽 1 正式赛道  槽 2 调试短表
 *       槽 3 实测轨迹 A                       槽 4 实测轨迹 B    page 87..95 空闲
 * 每槽 16 x 128 = 2048 条，50 mm 点距约 102 m；下载通道只能写槽 1~4。
 * 记录格式（packed 16 B，与上位机 dump 帧逐字节相同）：
 *   int32 x_mm | int32 y_mm | int16 yaw_cdeg | int16 v_cmps | int16 steer_cdeg | uint16 flags
 * steer_cdeg 记当拍施加的舵机目标角而非实测角；擦写一页阻塞，故拆成每圈一页。 */
#define INS_RT_SLOT_COUNT       (FM_SLOT_COUNT)         /* 槽数 5（槽 0~4）*/
#define INS_RT_SLOT_PAGES       (FM_SLOT_PAGES)         /* 每槽 17 页 */
#define INS_RT_FLASH_REC_PAGES  (FM_SLOT_REC_PAGES)     /* 每槽记录页 16 */
#define INS_RT_REC_SIZE         (FM_REC_SIZE)           /* 单条记录 16 B */
#define INS_RT_REC_PER_PAGE     (FM_REC_PER_PAGE)       /* 每页 128 条 */
#define INS_RT_SLOT_CAPACITY    (FM_SLOT_CAPACITY)      /* 单槽容量 2048 条 */
#define INS_RT_FLASH_CAPACITY   (INS_RT_SLOT_CAPACITY)  /* 同上，旧名 */
#define INS_RT_INDEX_PAGE       (FM_SLOT_INDEX_PAGE)    /* 槽索引页 = 1 */
#define INS_RT_FLASH_HEAD_PAGE  (FM_SLOT_BASE)          /* 旧名：槽 0 头页 */
#define INS_RT_FLASH_REC_FIRST  (FM_SLOT_BASE + 1u)     /* 旧名：槽 0 首记录页 */
#define INS_RT_SLOT_BASE(k)     (FM_SLOT_BASE_PAGE(k))  /* 槽 k 头页页号 */
#define INS_RT_SLOT_MAX_BYTES   (36u + (uint32)INS_RT_SLOT_CAPACITY * (uint32)INS_RT_REC_SIZE)  /* 镜像上限 */

/* 上电从当前槽恢复，必须在 5 ms 中断使能前调用（内部阻塞）；1 = 成功，0 = 无表或损坏 */
uint8   ins_route_flash_load        (void);

/* ---- 多槽接口 ---- */
void    ins_route_slot_init         (void);     /* 读回 active_slot，不载表；索引缺失取 0 */
uint8   ins_route_slot_active       (void);     /* 当前槽号 0..4 */
uint8   ins_route_slot_set_active   (uint8 slot, uint8 persist);  /* persist = 1 写索引页；0 = 成功 */
uint8   ins_route_slot_select       (uint8 slot, uint8 persist);  /* 先载入再切槽；1 = 成功 */
uint8   ins_route_flash_load_slot   (uint8 slot);   /* 载入指定槽；1 = 成功 */
void    ins_route_flash_save_slot   (uint8 slot);   /* 当前 RAM 表分步写入该槽 */
uint8   ins_route_slot_load_image   (uint8 slot, const uint8 *img, uint32 len);  /* 0 = 已武装 */

/* 槽摘要，供 PC 列槽用；valid = 1 表示该槽有可用表，tag 为自定义标记 */
typedef struct
{
    uint8   valid;
    uint8   slot;
    uint16  tag;
    uint32  count;
    uint32  lap_mm;
    uint32  ds_mm;
    uint32  crc_r;
} ins_route_slot_info_t;
uint8   ins_route_slot_info         (uint8 slot, ins_route_slot_info_t *out);  /* 1 = 有表 */

/* 按镜像偏移读字节，镜像 = 36 B 头 + N x 16 B 记录，逻辑连续。
 * 返回实际字节数；0 = 槽号错、无表、偏移越界或 dst 为空；n 会钳到镜像末尾。 */
uint16  ins_route_slot_read_bytes   (uint8 slot, uint32 off, uint8 *dst, uint16 n);

/* ---- 分步存盘：start 武装，poll 每圈写一页，写完自动收尾 ---- */
void    ins_route_flash_save_start  (void);     /* 写槽 0（手推录制的槽）*/
void    ins_route_flash_poll        (void);     /* 主循环每圈调一次，空闲时立即返回 */
uint8   ins_route_flash_busy        (void);     /* 1 = 正在写 */
uint8   ins_route_flash_last_rc     (void);     /* 0 未做 1 成功 2 页写失败 3 表空 4 槽号错 */
uint8   ins_route_flash_progress    (void);     /* 0~100，含表头那一步 */
uint8   ins_route_flash_last_slot   (void);     /* 最近一次读写的槽号 */
uint8   ins_route_flash_loaded      (void);     /* 1 = 当前 RAM 表来自 Flash */

/* 把 [first, first+max_n) 的记录按 16 B 打包写进 dst，返回实际条数；dst 至少 max_n x 16 B */
uint16  ins_route_pack_records      (uint16 first, uint8 *dst, uint16 max_n);

/* ---- 内建（理论）路表 ----
 * 把 tools/route_to_carbin.py 生成的表编进固件，上电时只补空槽，槽里有表就不写。
 * 新增一张：生成 .c 加入 CM7_0 工程，再在 ins_route_builtin.c 注册表加一行；写槽会设为 active。 */
#define INS_ROUTE_BUILTIN_EN            (1)
#define INS_ROUTE_BUILTIN_SLOT_CIRCLE   (FM_SLOT_MAP_DEBUG)     /* R=500 mm 圆环，调控制层用 */
#define INS_ROUTE_BUILTIN_SLOT_TRACK    (FM_SLOT_MAP_RACE)      /* 正式赛道理论中线 */

/* 一条内建表记录。len 用指针：生成物里是 const uint32，不能直接写进静态初始化式 */
typedef struct
{
    uint8         slot;     /* 补进哪一槽 */
    const char   *name;     /* 日志用名字 */
    const uint8  *img;      /* 车端镜像：36 B 头 + N x 16 B 记录 */
    const uint32 *len;      /* 指向生成物的 <cname>_len */
} ins_route_builtin_t;

extern const ins_route_builtin_t ins_route_builtin_tbl[];
extern const uint32              ins_route_builtin_cnt;

/* 从内存镜像载入，字节格式同 ins_route_flash_load；1 = 成功 */
uint8   ins_route_load_image        (const uint8 *img, uint32 len);

/* ---- 巡航记录（Cruise 实际轨迹）----
 * 与路表共用记录布局和落槽通道，槽头 flags 的 bit1 标记类型。
 * 记录第 4 字段存舵机 PWM duty，yaw 存绝对航向；槽头 ds_mm 存采样周期 ms，lap_mm 存总时长 ms。
 * 等时间间隔采样，车速可由相邻两点反推；巡迹中只攒 RAM，停巡后再排队落盘。 */
#define INS_ROUTE_LOG_SLOT      (FM_SLOT_TRAJ_A)        /* 巡航记录写槽 3 */
#define INS_ROUTE_LOG_MAX       (INS_RT_FLASH_CAPACITY) /* 2048 条 = 单槽容量 */

#define INS_ROUTE_HDR_FL_FULL   (0x0001u)   /* 槽头 bit0：录满 */
#define INS_ROUTE_HDR_FL_LOG    (0x0002u)   /* 槽头 bit1：巡航记录，第 4 字段为舵机 duty */
#define INS_ROUTE_HDR_FL_STEP   (0x0004u)   /* 槽头 bit2：转向阶跃测试记录 */

/* ---- 通用 16 B 记录器：巡航记录与阶跃测试共用 s_log[2048] 与同一条落槽通道 ---- */
void    ins_route_rec_begin (uint16 flags, uint16 sample_ms);   /* 清计数并定类型 */
void    ins_route_rec_push  (const uint8 *rec16);               /* 中断里调，只写 RAM */
void    ins_route_rec_end   (void);                             /* 收尾，只置落盘请求位 */
uint32  ins_route_rec_count (void);                             /* 已记条数 */
uint8   ins_route_rec_saving(void);                             /* 1 = 已排队或正在写 */

#define INS_ROUTE_STEP_SLOT     (FM_SLOT_TRAJ_B)        /* 阶跃测试写槽 4，与轨迹记录二选一 */

extern volatile float   ins_log_ms;         /* 采样周期 ms（参数 ID 38），默认 20 ms = 50 Hz */

void    ins_route_log_reset   (void);       /* 开始一次记录 */
void    ins_route_log_finish  (void);       /* 停巡/跑完/中止：只置请求位 */
void    ins_route_log_tick    (float x, float y, float yaw_deg, float servo_duty);  /* 5 ms 一次，只写 RAM */
uint16  ins_route_log_count   (void);
uint8   ins_route_log_full    (void);       /* 1 = 缓冲已满 */
uint8   ins_route_log_saving  (void);       /* 1 = 已排队或正在写 */
void    ins_route_log_poll    (void);       /* 主循环每圈调一次，把排队落成分步写 */

void    ins_route_log_save_now (void);      /* 立刻写槽，内部要 flash_init，不在中断里调 */

#endif
