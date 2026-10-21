/* 工程 10（卡丁）· Flash 页分配约定 —— **唯一权威**
 * ---- 硬件几何（已核实，见 `平台事实速查`）------------------------------------------------
 * CYT4BB7 Work Flash: **192 KB = 96 页 × 2 KB**（`FLASH_PAGE_NUM = 96`）。
 * （库注释明写"仅可填写 0"）。
 * 页 0        ：**参数持久化**（`app_params`，1 页够 —— 载荷 268 B）
 * 页 1        ：**槽索引**（`ins_route` 的 rt_idx_t：active + 每槽摘要 + CRC）
 * 页 2 ~ 86   ：**数据槽池 = 5 槽 × 17 页**
 * 槽 0 ：手推/人开车录制（带舵机打角）
 * 槽 1 ：电子制图 · 正式赛道（PC 灌表）
 * 槽 2 ：电子制图 · 调试（圆环 / 短表）
 * 槽 3 ：实测轨迹 A（跑完自动存，带打角）
 * 槽 4 ：实测轨迹 B
 * 页 87 ~ 95  ：**空闲 9 页**（以后：标定表 / 多圈统计…）
 * ① 跑班数据改由手机 App「卡丁跑班」侧记录（见 `KartPhone/`），车端 `bench_log` 只留
 * ② 记录格式从 12 B 升到 **16 B**（多出 `steer_cdeg` 舵机打角 + `flags`），
 * 见 `ins_route.h`。**手推方案要靠这一列做前馈**，没有它就只能靠 PP 几何凑。
 * ③ 槽位从 4 个升到 **5 个**，且**轨迹与路表共用同一套槽/记录格式**
 * ⇒ 跑完的一趟轨迹可以原样当路表再放一遍（"用真实走过的线跑"）。
 * ④ 每槽 17 页 = 1 头页 + 16 记录页 × **128 条/页** = **2048 条**
 * 把让出来的 32 页里拿 20 页补到槽上（4×4），再加第 5 槽。
 * 点距 0.05 m ⇒ 单槽 **102 m**；0.10 m ⇒ 204 m（参数 `rt_ds`，ID 39）。
 * ---- 记录长度（改它会连带改三处，别只改一处）--------------------------------------------
 * `ins_route.h` 的 `INS_RT_REC_SIZE` / `ins_route.c` 的 `rt_rec_t` /
 * PC 侧 `tools/route_to_carbin.py` 的 `RT_REC_SIZE`
 * ⇒ 统一由 `FM_REC_SIZE` 定义，前两处都引用它，只有 PC 侧是硬编码（靠 PONG 的 rec_size 兜底）。
 */

#ifndef _FLASH_MAP_H
#define _FLASH_MAP_H

#include "zf_common_headfile.h"

/* ---- 总容量 ---- */
#define FM_PAGE_SIZE            (2048u)                         /* 每页字节 */
#define FM_PAGE_TOTAL           (96u)                           /* 总页数(0~95) */

/* ---- 页 0：参数 ---- */
#define FM_PARM_PAGE            (0u)

/* ---- 页 1：槽索引 ---- */
#define FM_SLOT_INDEX_PAGE      (1u)

/* ---- 数据槽池：5 槽 × 17 页 ------------------------------------------------------------ */
#define FM_REC_SIZE             (16u)                           /* 每条记录 16 字节（v2） */
#define FM_REC_PER_PAGE         (FM_PAGE_SIZE / FM_REC_SIZE)    /* = 128 条/页 */

#define FM_SLOT_BASE            (2u)
#define FM_SLOT_COUNT           (5u)
#define FM_SLOT_PAGES           (17u)                           /* 1 头页 + 16 记录页 */
#define FM_SLOT_REC_PAGES       (FM_SLOT_PAGES - 1u)            /* = 16 */
#define FM_SLOT_BASE_PAGE(k)    ((uint32)FM_SLOT_BASE + (uint32)(k) * (uint32)FM_SLOT_PAGES)
#define FM_SLOT_PAGES_TOTAL     (FM_SLOT_COUNT * FM_SLOT_PAGES) /* = 85 页（2~86） */
#define FM_SLOT_CAPACITY        (FM_SLOT_REC_PAGES * FM_REC_PER_PAGE)   /* = 2048 条 */

/* ---- 空闲段（页 87~95）---- */
#define FM_FREE_BASE            (FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL)    /* = 87 */
#define FM_FREE_PAGES           (FM_PAGE_TOTAL - FM_FREE_BASE)

/* ---- 槽号语义（全工程约定；菜单/PC/文档统一用这一份）------------------------------------ */
#define FM_SLOT_MANUAL          (0u)    /* 手推 / 人开车录制（带打角）—— 灌表协议禁写 */
#define FM_SLOT_MAP_RACE        (1u)    /* 电子制图 · 正式赛道 */
#define FM_SLOT_MAP_DEBUG       (2u)    /* 电子制图 · 调试（圆环 / 短表） */
#define FM_SLOT_TRAJ_A          (3u)    /* 实测轨迹 A */
#define FM_SLOT_TRAJ_B          (4u)    /* 实测轨迹 B */
#define FM_SLOT_PROTECTED       (FM_SLOT_MANUAL)  /* 协议层唯一禁写的槽 */

/* 编译期守卫：各段不许越界、不许重叠 */
typedef char fm_param_fit_check[(FM_PARM_PAGE + 1u <= FM_SLOT_INDEX_PAGE) ? 1 : -1];
typedef char fm_index_fit_check[(FM_SLOT_INDEX_PAGE + 1u <= FM_SLOT_BASE) ? 1 : -1];
typedef char fm_slot_fit_check [(FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL <= FM_FREE_BASE) ? 1 : -1];
typedef char fm_free_fit_check [(FM_FREE_BASE + FM_FREE_PAGES <= FM_PAGE_TOTAL) ? 1 : -1];
typedef char fm_rec_fit_check  [(FM_REC_PER_PAGE * FM_REC_SIZE <= FM_PAGE_SIZE) ? 1 : -1];

#endif
