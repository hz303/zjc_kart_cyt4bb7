/*********************************************************************************************************************
 * Flash 页分配约定, 全工程唯一权威
 *
 * 硬件: Work Flash 192 KB = 96 页 × 2 KB; FLASH_PAGE_LENGTH = 512 的单位是 uint32 个数
 *   flash_erase_page / flash_write_page_from_buffer 的 sector_num 无实际作用, 只能填 0
 * 页 0        参数持久化 app_params, 1 页
 * 页 1        槽索引 rt_idx_t, active + 每槽摘要 + CRC
 * 页 2~86     数据槽池 5 槽 × 17 页: 0 手推录制 / 1 正式赛道 / 2 调试 / 3 实测 A / 4 实测 B
 * 页 87~95    空闲 9 页
 * 每槽 17 页 = 1 头页 + 16 记录页 × 128 条 = 2048 条; 点距 0.05 m 单槽 102 m, 0.10 m 则 204 m (rt_ds, ID 39)
 * 新增功能必须在此登记页段, 不得在别处硬编码页号; 页 1 索引记着每槽归属/条数/时间戳/CRC,
 *   读回不依赖页号; 灌表协议 route_link 只允许写槽 1..4, 不会覆盖槽 0
 * 记录长度统一由 FM_REC_SIZE 定义, 另需同步 ins_route.c 的 rt_rec_t 与 PC 侧
 *   tools/route_to_carbin.py 的 RT_REC_SIZE
 ********************************************************************************************************************/

#ifndef _FLASH_MAP_H
#define _FLASH_MAP_H

#include "zf_common_headfile.h"

#define FM_PAGE_SIZE            (2048u)
#define FM_PAGE_TOTAL           (96u)

/* ---- 页 0 参数 ---- */
#define FM_PARM_PAGE            (0u)

/* ---- 页 1 槽索引 ---- */
#define FM_SLOT_INDEX_PAGE      (1u)

/* ---- 数据槽池: 5 槽 × 17 页 ---- */
#define FM_REC_SIZE             (16u)                           /* 每条记录 16 字节 */
#define FM_REC_PER_PAGE         (FM_PAGE_SIZE / FM_REC_SIZE)

#define FM_SLOT_BASE            (2u)
#define FM_SLOT_COUNT           (5u)
#define FM_SLOT_PAGES           (17u)                           /* 1 头页 + 16 记录页 */
#define FM_SLOT_REC_PAGES       (FM_SLOT_PAGES - 1u)
#define FM_SLOT_BASE_PAGE(k)    ((uint32)FM_SLOT_BASE + (uint32)(k) * (uint32)FM_SLOT_PAGES)
#define FM_SLOT_PAGES_TOTAL     (FM_SLOT_COUNT * FM_SLOT_PAGES)
#define FM_SLOT_CAPACITY        (FM_SLOT_REC_PAGES * FM_REC_PER_PAGE)

/* ---- 空闲段, 页 87~95 ---- */
#define FM_FREE_BASE            (FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL)
#define FM_FREE_PAGES           (FM_PAGE_TOTAL - FM_FREE_BASE)

/* ---- 槽号语义, 菜单与 PC 侧统一用这一份 ---- */
#define FM_SLOT_MANUAL          (0u)    /* 手推或人开车录制, 带打角, 灌表协议禁写 */
#define FM_SLOT_MAP_RACE        (1u)    /* 电子制图, 正式赛道 */
#define FM_SLOT_MAP_DEBUG       (2u)    /* 电子制图, 调试用圆环或短表 */
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
