/*********************************************************************************************************************
 * Flash 页分配表：参数页 / 槽索引页 / 5 个数据槽 / 空闲段
 * CYT4BB7 Work Flash 192 KB = 96 页 × 2 KB（FLASH_PAGE_NUM = 96）；FLASH_PAGE_LENGTH = 512 的单位是 uint32 个数, 不是字节
 * flash_erase_page / flash_write_page_from_buffer 的 sector_num 参数无实际作用, 只能填 0
 * 页 0 参数持久化 app_params, 载荷 268 B；页 1 槽索引 rt_idx_t, active + 每槽摘要 + CRC
 * 页 2~86 数据槽池, 5 槽 × 17 页：0 手推 / 人开车录制（带打角）, 1 电子制图·正式赛道, 2 电子制图·调试, 3 实测轨迹 A, 4 实测轨迹 B
 * 页 87~95 空闲 9 页
 * 每槽 1 头页 + 16 记录页, 2048 条；点距 0.05 m 时单槽 102 m, 0.10 m 时 204 m（rt_ds, ID 39）
 * 新增功能须先在此登记页段, 不要硬编码页号；灌表协议 route_link 只允许写槽 1..4, 不会覆盖槽 0
 * 记录长度由 FM_REC_SIZE 定义, 改动需同步 ins_route.c 的 rt_rec_t 与 PC 侧 route_to_carbin.py 的 RT_REC_SIZE
 ********************************************************************************************************************/

#ifndef _FLASH_MAP_H
#define _FLASH_MAP_H

#include "zf_common_headfile.h"

#define FM_PAGE_SIZE            (2048u)                         /* 每页字节 */
#define FM_PAGE_TOTAL           (96u)                           /* 总页数, 0~95 */

#define FM_PARM_PAGE            (0u)
#define FM_SLOT_INDEX_PAGE      (1u)

#define FM_REC_SIZE             (16u)                           /* 每条记录 16 字节 */
#define FM_REC_PER_PAGE         (FM_PAGE_SIZE / FM_REC_SIZE)    /* 128 条/页 */

#define FM_SLOT_BASE            (2u)
#define FM_SLOT_COUNT           (5u)
#define FM_SLOT_PAGES           (17u)                           /* 每槽 1 头页 + 16 记录页 */
#define FM_SLOT_REC_PAGES       (FM_SLOT_PAGES - 1u)            /* 16 记录页 */
#define FM_SLOT_BASE_PAGE(k)    ((uint32)FM_SLOT_BASE + (uint32)(k) * (uint32)FM_SLOT_PAGES)
#define FM_SLOT_PAGES_TOTAL     (FM_SLOT_COUNT * FM_SLOT_PAGES) /* 85 页, 2~86 */
#define FM_SLOT_CAPACITY        (FM_SLOT_REC_PAGES * FM_REC_PER_PAGE)   /* 2048 条 */

#define FM_FREE_BASE            (FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL)    /* 87 */
#define FM_FREE_PAGES           (FM_PAGE_TOTAL - FM_FREE_BASE)          /* 9  */

#define FM_SLOT_MANUAL          (0u)    /* 手推 / 人开车录制, 灌表协议禁写 */
#define FM_SLOT_MAP_RACE        (1u)    /* 电子制图 · 正式赛道 */
#define FM_SLOT_MAP_DEBUG       (2u)    /* 电子制图 · 调试（圆环 / 短表） */
#define FM_SLOT_TRAJ_A          (3u)    /* 实测轨迹 A */
#define FM_SLOT_TRAJ_B          (4u)    /* 实测轨迹 B */
#define FM_SLOT_PROTECTED       (FM_SLOT_MANUAL)  /* 协议层唯一禁写的槽 */

/* 编译期守卫：各段不越界、不重叠 */
typedef char fm_param_fit_check[(FM_PARM_PAGE + 1u <= FM_SLOT_INDEX_PAGE) ? 1 : -1];
typedef char fm_index_fit_check[(FM_SLOT_INDEX_PAGE + 1u <= FM_SLOT_BASE) ? 1 : -1];
typedef char fm_slot_fit_check [(FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL <= FM_FREE_BASE) ? 1 : -1];
typedef char fm_free_fit_check [(FM_FREE_BASE + FM_FREE_PAGES <= FM_PAGE_TOTAL) ? 1 : -1];
typedef char fm_rec_fit_check  [(FM_REC_PER_PAGE * FM_REC_SIZE <= FM_PAGE_SIZE) ? 1 : -1];

#endif
