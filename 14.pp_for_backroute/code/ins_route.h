/* C 车惯导 + 路表 - 路表录制（米制路线图）
 * `code/Kart_Decision/kart_record.c` / `kart_record.h`（**实测过的阈值，不是拍脑袋**）：
 * waypoint{x, y, yaw, v}；距离阈值 0.05 m；转角阈值 2.0°；容量 1500 点
 * 坐标旋转到"录制起点车体系"（x 向右、y 向前），yaw 取**相对起点的增量**
 * 因为 IMU 的 yaw 一定会漂，而且每次上电的零点都不一样。
 * 如果把"世界系绝对坐标"录进路表，下次上电复现时根本对不上。
 * ⇒ **纪律：只用相对量。** 本文件的 x/y/yaw 已经全部是"相对录制起点"的了，
 * 表头那三个 `origin_x / origin_y / origin_yaw` 只是**录制那一刻的 odom**（诊断用）。
 * 快照 s_o0*，之后全靠增量）。原因：odom 是"上电以来的积分量"，
 * · 烧一次 / 断一次电 → 归零；
 * · 把车抱起来走一段 → 轮子不转，odom 也不动；
 * 拿它当锚点 ⇒ 表只能在同一上电周期、且没人动过车时用，这不现实（现场就是这么炸的：
 * 17:25 那次 origin 差 37.8 m，外环误差 84.7° ⇒ 一开 PUSH 舵机顶死左限位）。
 * ⇒ 现在的语义：**把车摆到表的起点、朝向大致对，再开 Follow。** 谁都不用记 odom。
 * **直接把 `record_running` 清 0 然后 return** —— 没有溢出标志、没有返回码，
 * 和"正常停录"一模一样。现场只能靠"点数恰好停在 1500"这一个迹象判断。
 * 本工程: 加 `INS_ROUTE_ST_FULL` 状态位，并上报到遥测 `ins_flags`。
 * ---- 容量核算 -----------------------------------------------------------------------
 * 1500 点 × 16 B = **24 KB**。CYT4BB7 有 768 KB SRAM，绰绰有余。
 * ① 点距 0.05 → **0.02 m**（循迹要更密），容量提到 **3000 点** = 60 m 计算里程；
 * ② 新增 **Flash 持久化**（`ins_route_flash_*`）—— 断电不丢表，且能被上位机 dump 走。
 * 布局见本文件"Flash 布局"节；RAM 侧 3000×16 B = 48 KB（CM7_0 有 384 KB，够）。
 * ③ 采样**仍然只按距离**（转角触发默认关）—— 见 ins_route.c。
 */

#ifndef _INS_ROUTE_H
#define _INS_ROUTE_H

#include "zf_common_headfile.h"
#include "flash_map.h"

/* 配置
 * 工程 8（C 车）用 0.02 m：那时循迹走"等间距里程索引"，点越密越好。
 * 卡丁改成 **Pure Pursuit**（沿弧长找前视点），点距只要远小于前视距离（0.3~1.5 m）就够，
 * 0.05 m 已经密 6~30 倍。代价/收益：**单槽容量从 40.8 m 变成 102 m**（2040 × 0.05）
 * —— 40.8 m 对卡丁连半圈都不够。
 */
#define INS_ROUTE_MAX_POINTS    (INS_RT_SLOT_CAPACITY)      /* = 2048，见 Flash 多槽布局 */
#define INS_ROUTE_DS_DEFAULT    (0.10f)         /* 米：默认点距 ⇒ 单槽 102 m */
#define INS_ROUTE_DS_MIN        (0.02f)
#define INS_ROUTE_DS_MAX        (0.20f)
#define INS_ROUTE_YAW_THRESH    (2.0f)          /* 度：转弯触发阈值（**默认不启用**） */

extern volatile float   ins_route_ds;

/* 数据结构 */
typedef struct
{
    float x;            /* 米，录制起点车体系：x 向右 */
    float y;            /* 米，                y 向前 */
    float yaw;          /* 度，**相对录制起点**（不是绝对航向） */
    float v;            /* m/s，录制瞬间车速 */
    int16 steer_cdeg;
    uint16 flags;
} ins_wp_t;

/* ---- 逐点 flags（存进 16 B 记录的最后一个字段）----------------------------------------
 * 回放时这一点不参与前馈，只靠 Pure Pursuit。**不要用"打角==0"代替这个位** ——
 * 正中间走直线时打角本来就是 0，那是有效值。
 */
#define INS_WP_FL_STEER     (0x0001u)   /* 本点 steer_cdeg 有效 */
#define INS_WP_FL_KEY       (0x0002u)
#define INS_WP_FL_REV       (0x0004u)   /* 录制时为倒车段（v < 0） */
#define INS_WP_FL_TRAJ      (0x0008u)   /* 本份是**实测轨迹**（不是教学表） */

typedef struct
{
    float   origin_x;       /* 录制起点的 odom 坐标（米） */
    float   origin_y;
    float   origin_yaw;     /* 录制起点的航向（度） */
    uint16  count;          /* 已录点数 */
    uint16  version;        /* 格式版本（改布局必须加 1） */
    float   total_dist;     /* 录制期间总里程（米） */
} ins_route_head_t;

/* 状态位（上报到遥测 ins_flags 的高 8 位里的一位；见 motor_link.h） */
#define INS_ROUTE_ST_RUNNING    (0x01u)
#define INS_ROUTE_ST_FULL       (0x02u)

void    ins_route_init      (void);

/* 开始/停止录制。start 会**重新捕获起点位姿**并清空缓冲。 */
void    ins_route_start     (void);
void    ins_route_stop      (void);

void    ins_route_poll      (void);

void    ins_route_clear     (void);         /* 只清缓冲，不含起点 */

uint8   ins_route_is_running(void);
uint8   ins_route_flags     (void);
uint16  ins_route_get_count (void);
float   ins_route_get_total_dist (void);

/* 录制时**逐点**记下"当拍实际施加的舵机目标角"（`ins_wp_t.steer_cdeg`），
 * 另有 `ins_route_mark_key()` 让人在**关键点**（每个锥桶、每个弯心）按一下，打上
 * `INS_WP_FL_KEY`。回放时关键点可以用来：
 * · 调前视/加前馈权重（关键点附近用打角前馈，其余靠 PP）；
 * · 事后在 PC 上画出来核对"我到底在哪打的点"。
 * 按下去时车已经又走了几厘米，标"最近那个点"才是操作者想的位置。
 */
uint8   ins_route_mark_key   (void);        /* 1 = 标上了；0 = 没在录/点太少 */
uint16  ins_route_key_count  (void);        /* 本份记录里有几个关键点 */
uint16  ins_route_steer_count(void);        /* 本份记录里有几个点带有效打角 */
/* 1 = 之后录的点打"实测轨迹"标记（存进槽 3/4 时用）；0 = 教学表（默认） */
void    ins_route_set_traj   (uint8 on);

const ins_wp_t          *ins_route_get_points (void);
const ins_route_head_t  *ins_route_get_head   (void);

/* `ins_route_rev_count()`     数一数本表有几个点打了 REV（"录制时 v<0"）。倒车录出来的表
 * 过半都是 REV ⇒ 据此判定"这是一张倒车录的表"。
 * `ins_route_reverse_points()` 把 RAM 里的点序**原地倒过来**（x/y/yaw/v/steer/flags 一起换）。
 * 返回 1=倒了；0=没录满/正在录。
 * `route_pb_start()` 的对齐是"**车现在在哪儿，哪儿就是表第 0 点**"（只用增量），
 * 所以点序一倒，语义自动变成"从原末点往原起点走"，别的量一个都不用改。
 * 表里的 yaw（车头方向）会差 180° ⇒ PP 第一拍就满舵。route_pb_start 里硬拦这一条。
 */
uint16  ins_route_rev_count      (void);
uint8   ins_route_reverse_points (void);

/* "串口灌表"（PC 下发一张理论表）与"手推录制"是两件事。旧布局只有一张表，
 * 灌表必然覆盖手推表。现在改成**按槽分**：
 * ---- 布局（**工程 10 已重排**；页号唯一权威是 `flash_map.h`）--------------------------
 * page  2 .. 86 : 槽 0~4（每槽 17 页：1 头页 + 16 记录页）
 * 槽 0 = **手推 / 人开车录制**（带舵机打角，灌表协议禁写）
 * 槽 1 = 电子制图·正式赛道（PC 灌表）
 * 槽 2 = 电子制图·调试（圆环 / 短表）
 * 槽 3 = 实测轨迹 A（跑完自动存，带打角）
 * 槽 4 = 实测轨迹 B
 * page  87..95  : 空闲 9 页
 * ⇒ **下载通道在协议层只允许写 1..4** ⇒ 结构上不可能覆盖手推表。
 * ⇒ 每槽 16 × 128 = **2048 条**；50 mm 点距 ⇒ **102 m**（100 mm ⇒ 204 m）。
 * 一律走 `FM_SLOT_*`。
 * 必须重录或重灌。这是**有意的**：宁可逼你重来，也不让 12 B 的流按 16 B 解成垃圾。
 * ---- 记录格式（packed 16 B，与"上位机 dump 帧"逐字节相同）------------------------
 * int32  x_mm       | int32  y_mm      | int16 yaw_cdeg
 * int16  v_cmps     | int16  steer_cdeg | uint16 flags
 * 而且上位机不必假设浮点格式（float 是 IEEE754 没错，但端序/对齐仍要小心）。
 * int32 mm ⇒ ±2147 km，永远不会溢出。
 * （`δ = ff·steer[k] + PP 修正`，见 `route_pb.c`）。它记的是**当拍实际施加的
 * 舵机目标角**（不是实测角 —— 实测角有随动滞后，拿来当前馈会把滞后也复制一遍）。
 * 擦/写一页 2 KB 是**阻塞**操作（毫秒级）。十七页连着写 ≈ 上百毫秒 ⇒
 * 会顶穿保护层的"心跳门 200 ms / 失联门 600 ms"⇒ 电机被意外掐停。
 * ⇒ 拆成"武装 + 主循环每圈写一页"（`ins_route_flash_poll`），主循环始终在转。
 */
#define INS_RT_SLOT_COUNT       (FM_SLOT_COUNT)         /* 5 个槽（0~4） */
#define INS_RT_SLOT_PAGES       (FM_SLOT_PAGES)         /* 17 = 1 头页 + 16 记录页 */
#define INS_RT_FLASH_REC_PAGES  (FM_SLOT_REC_PAGES)
#define INS_RT_REC_SIZE         (FM_REC_SIZE)
#define INS_RT_REC_PER_PAGE     (FM_REC_PER_PAGE)
#define INS_RT_SLOT_CAPACITY    (FM_SLOT_CAPACITY)      /* = 2048 条 */
#define INS_RT_FLASH_CAPACITY   (INS_RT_SLOT_CAPACITY)  /* 兼容旧名 */
#define INS_RT_INDEX_PAGE       (FM_SLOT_INDEX_PAGE)
#define INS_RT_FLASH_HEAD_PAGE  (FM_SLOT_BASE)          /* 兼容旧名 = 槽 0 头页 */
#define INS_RT_FLASH_REC_FIRST  (FM_SLOT_BASE + 1u)     /* 兼容旧名 = 槽 0 首记录页 */
#define INS_RT_SLOT_BASE(k)     (FM_SLOT_BASE_PAGE(k))
/* 一张表镜像的最大字节（36 B 头 + 记录）—— 下发通道用它做上限 */
#define INS_RT_SLOT_MAX_BYTES   (36u + (uint32)INS_RT_SLOT_CAPACITY * (uint32)INS_RT_REC_SIZE)

/* 上电时从 Flash 恢复（**必须在 5ms 中断打开之前调**：它是阻塞的） */
uint8   ins_route_flash_load        (void);     /* 1 = 载入成功并生效；0 = 无表/损坏 */

/* 上电时从槽索引读回 active_slot（不载表；索引本身不存在时默认 0） */
void    ins_route_slot_init         (void);
uint8   ins_route_slot_active       (void);     /* 当前选中的槽（0..INS_RT_SLOT_COUNT-1） */
/* 选槽。persist=1 时写索引页（掉电保持）。返回 0 = 成功 */
uint8   ins_route_slot_set_active   (uint8 slot, uint8 persist);
/* 载入指定槽到 RAM（1 = 成功）。= ins_route_flash_load 的可选槽版本 */
uint8   ins_route_flash_load_slot   (uint8 slot);
/* 把**当前 RAM 里的表**写进指定槽（分步；slot=0 就是老行为） */
void    ins_route_flash_save_slot   (uint8 slot);
/* 从内存镜像写入指定槽：先校验镜像、展开进 RAM，再分步落盘。返回 0 = 已武装 */
uint8   ins_route_slot_load_image   (uint8 slot, const uint8 *img, uint32 len);

/* 槽摘要（给 PC 列表用）。valid=1 表示该槽有可用表 */
typedef struct
{
    uint8   valid;
    uint8   slot;
    uint16  tag;            /* 0 = 未标记；可用来自定义（预留） */
    uint32  count;
    uint32  lap_mm;
    uint32  ds_mm;
    uint32  crc_r;
} ins_route_slot_info_t;
uint8   ins_route_slot_info         (uint8 slot, ins_route_slot_info_t *out);

/* 给串口上传用（PC 分块拉，车端按需读那一两页）。 返回实际读到的字节数；0 = 槽号错 / 槽里没表 / 偏移越界 / 参数为空。 */
uint16  ins_route_slot_read_bytes   (uint8 slot, uint32 off, uint8 *dst, uint16 n);

/* 分步存盘：start 武装 → poll 每圈写一页 → 全部写完自动收尾 */
void    ins_route_flash_save_start  (void);     /* = 写槽 0（手推录制的家） */
void    ins_route_flash_poll        (void);     /* 主循环每圈调一次（幂等，空闲时立刻返回） */
uint8   ins_route_flash_busy        (void);     /* 1 = 还在写 */
uint8   ins_route_flash_last_rc     (void);     /* 0 = 未做；1 = 成功；2 = 页写失败；3 = 表空 */
uint8   ins_route_flash_progress    (void);     /* 0~100（含表头那一步） */
uint8   ins_route_flash_last_slot   (void);     /* 最近一次写/读的槽号（给菜单显示） */

/* 表来自 Flash 的标记：菜单/上位机据此提示"这是上次存的表，不是刚录的" */
uint8   ins_route_flash_loaded      (void);

/* 这是"路表记录长什么样"的**唯一定义**（Flash 存盘与上位机 dump 都走它） ⇒ 两处布局不可能漂移。dst 至少要 max_n*12 字节。 */
uint16  ins_route_pack_records      (uint16 first, uint8 *dst, uint16 max_n);

/* 把 `tools/route_to_carbin.py` 生成的理论表**编进固件**，用来"上电就把某个槽补上"。
 * 现场意义：调控制层（舵机/循迹）时手上永远有一张**确定正确**的表，不用先推一圈；
 * 正式赛道那张理论中线也编进来，换车/换板后不必依赖 PC 在场。
 * ---- 怎么加/换一张表（全程只碰三个地方）--------------------------------------------
 * python tools/asc_to_route.py <地图.json> --outdir route_out/x
 * python tools/route_to_carbin.py route_out/x/route.json --outdir route_out/x \
 * --cname ins_route_builtin_xxx
 * python tools/route_reroot.py --route route_out/x/route.json \
 * --rec <实际路表 route_dump.csv> --outdir route_out/x/reroot
 * python tools/route_to_carbin.py route_out/x/reroot/route_reroot.json ...
 * ② 把 `ins_route_builtin_xxx.c` 拷到本目录，**加进 CM7_0 的 IAR 工程**（只 CM7_0）。
 * ③ 在 `ins_route_builtin.c`（手写的注册表）里加一行 {槽号, 名字, img, len}。
 * ---- 语义（改过一次，别按旧的记）----------------------------------------------------
 * （`main_cm7_0.c` 的上电块，在 `ins_route_flash_load()` **之前**）。
 * 紧随其后的 flash_load 会把真正的 active 槽重新载入 ⇒ RAM 最终一定对；
 * 放之后就会盖掉用户的表。
 * **只补空槽 ⇒ 永远不会抢用户的表**。
 * ⇒ 表里**最后一个真正写下去的**那张成为上电默认选中的表。
 * ---- 槽位分工（全工程约定；**唯一权威是 flash_map.h 的 FM_SLOT_***）------------------
 * 槽 0 手推录制（人开车 + 打角）｜槽 1 电子·正式｜槽 2 电子·调试
 * ｜槽 3 实测轨迹 A ｜槽 4 实测轨迹 B
 */
#define INS_ROUTE_BUILTIN_EN            (1)
#define INS_ROUTE_BUILTIN_SLOT_CIRCLE   (FM_SLOT_MAP_DEBUG)     /* R=500 mm 圆环：调控制层的靶子 */
#define INS_ROUTE_BUILTIN_SLOT_TRACK    (FM_SLOT_MAP_RACE)      /* 正式赛道理论中线 */

/* 一条内建表记录。`len` 用**指针**：生成物里的是 `const uint32`（不是编译期常量）， 直接写在静态初始化式里在 C 里不合法；指过去就行。 */
typedef struct
{
    uint8         slot;     /* 补进哪一槽 */
    const char   *name;
    const uint8  *img;      /* 车端镜像（36 B 头 + N×**16** B 记录，v2） */
    const uint32 *len;      /* -> 生成物的 <cname>_len */
} ins_route_builtin_t;

extern const ins_route_builtin_t ins_route_builtin_tbl[];
extern const uint32              ins_route_builtin_cnt;

/* 从内存镜像载入（与 ins_route_flash_load 同一套字节格式；也给以后的下发通道用） */
uint8   ins_route_load_image        (const uint8 *img, uint32 len);

/* 目的：`Cruise` 跑完要把**车实际走过的轨迹**连同**舵机指令**一起存进一个槽，
 * 槽头的 `rec_len`、`ins_route_slot_read_bytes()`、`ins_route_flash_load()`
 * 和上位机 `VerifyImage()` 全按 `rec_len` 走。**新造一种记录长度要动四处**，
 * 风险远大于收益 ⇒ 记录类**借用第 4 个字段**（路表里是 `v_cmps` = 录制车速）
 * 来存**舵机 PWM duty**，并在**槽头 flags 的 bit1** 上打标记（`INS_ROUTE_HDR_FL_LOG`）
 * ⇒ 读取方据此改标签。
 * ---- 记录内容（16 B，记录类）--------------------------------------------------------
 * int32 x_mm | int32 y_mm | int16 yaw_cdeg | int16 servo_duty | int16 0 | uint16 0
 * （`ins_log_ms`），`v ≈ Δpos / Δt`。槽头的 `ds_mm` 字段这里存的是**采样周期 ms**，
 * `lap_mm` 存的是**总时长 ms**（都靠 bit1 标记区分语义）。
 * 巡迹中写 Flash 是**阻塞**的（擦一页是毫秒级），会顶穿 5 ms 控制节拍 ⇒
 * 只在 **RAM 里攒**（2048 × 16 B = 32 KB，CM7_0 有 384 KB 可用），
 * `route_cruise_stop()` / 跑完 / 中止时才**排队**落盘，由主循环分步写。
 */
#define INS_ROUTE_LOG_SLOT      (FM_SLOT_TRAJ_A)        /* 巡航/实测轨迹写进哪一槽（槽 3） */
#define INS_ROUTE_LOG_MAX       (INS_RT_FLASH_CAPACITY) /* 2048 条 = 单槽容量 */

#define INS_ROUTE_HDR_FL_FULL   (0x0001u)   /* 槽头 flags bit0：录/记满了 */
#define INS_ROUTE_HDR_FL_LOG    (0x0002u)   /* 槽头 flags bit1：**这份是巡航记录**（第 4 字段 = 舵机 duty） */
#define INS_ROUTE_HDR_FL_STEP   (0x0004u)

/* · 同一块 `s_log[2048]`（32 KB）—— 两者时间上互斥，不需要两块；
 * · 同一个分步写盘器 + 同一条"槽 → CP 上传"读出通道 ⇒ 上位机/工具不需要第二套解析。
 * 唯一区别是**记录里那 16 个字节怎么解释**，用槽头 `flags` 的 bit1/bit2 区分。
 */
void    ins_route_rec_begin (uint16 flags, uint16 sample_ms);   /* 开始一份记录（清计数、定类型） */
void    ins_route_rec_push  (const uint8 *rec16);               /* 中断里调：只写 RAM，绝不阻塞（16 B） */
void    ins_route_rec_end   (void);                             /* 收尾：只置"请求落盘"位 */
uint32  ins_route_rec_count (void);
uint8   ins_route_rec_saving(void);

#define INS_ROUTE_STEP_SLOT     (FM_SLOT_TRAJ_B)

/* 采样周期（ms）。做成参数 ID 38，现场可改：20 ms = 50 Hz（默认）。 */
extern volatile float   ins_log_ms;

void    ins_route_log_reset   (void);       /* 开始一次记录（route_cruise_start 调） */
void    ins_route_log_finish  (void);       /* 停巡/跑完/中止时调：**只置请求位**，主循环去落盘 */
void    ins_route_log_tick    (float x, float y, float yaw_deg, float servo_duty);
uint16  ins_route_log_count   (void);
uint8   ins_route_log_full    (void);
uint8   ins_route_log_saving  (void);       /* 1 = 已排队/正在把记录写进槽（菜单可显示） */
void    ins_route_log_poll    (void);

/* 把当前记录写进 `INS_ROUTE_LOG_SLOT`（**不在中断里调**：内部要 flash_init） */
void    ins_route_log_save_now (void);

#endif
