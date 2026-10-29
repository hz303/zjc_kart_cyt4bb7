/*********************************************************************************************************************
 * 跑班数据记录：纯 RAM 缓冲 + 串口导 CSV
 * 记录格式 12 B/条：int32 t_ms（本次跑班开始以来 ms）+ int16 duty_x10（本拍实际施加的占空比 ×10, 带符号 %）
 *                   + int16 rps_l_x100 / rps_r_x100（左右轮实测转速 ×100 rps）+ int16 seg（小段编号 1~12）
 ********************************************************************************************************************/

#ifndef _BENCH_LOG_H
#define _BENCH_LOG_H

#include "zf_common_headfile.h"

#define BL_IDLE         (0u)
#define BL_REC          (1u)
#define BL_WRITE        (2u)        /* 保留编号, 语义为已冻结 */
#define BL_DONE         (3u)        /* 有数据可导出 */
#define BL_ERR          (4u)        /* 缓冲溢出 / 参数错 */
#define BL_DUMP         (5u)        /* 正在串口导出 CSV */

#define BL_BURST_MS     (1000u)     /* 每小段起始段内密采的时长 ms */
#define BL_STEADY_MS    (100u)      /* 密采结束后每多少 ms 记一点 */

#define BL_CAPACITY     (4096u)     /* RAM 条数, 4096 × 12 B = 48 KB */

#define BL_EXPORT_PER_PAGE  (500u)  /* 主循环每圈最多发多少条, ≈15 ms @115200 */

/* 开始一次记录：清 RAM 计数 */
void    bench_log_begin     (void);

/* 5 ms 中断每拍调用；t_in_seg_ms 为本小段已进行 ms, seg 小段号, duty/rps 为本拍实际值 */
void    bench_log_tick      (uint16 t_in_seg_ms, uint8 seg, float duty, float rps_l, float rps_r);

/* 5 ms 中断每拍调用（在 tick 之后）：只累加本次运行时间 */
void    bench_log_tick_clock(void);

/* 结束本次跑班并冻结条数；complete=1 正常跑完 / 0 被中止 */
void    bench_log_end       (uint8 complete);

/* 主循环每圈调用：分页导出状态机, 每次最多一页 */
void    bench_log_poll      (void);

/* 请求导出 CSV 到调试串口, 由主循环分页发送, 可随时中断 */
void    bench_log_dump      (void);

/* 清空 RAM 记录 */
void    bench_log_clear     (void);

uint8   bench_log_state     (void);
uint16  bench_log_count     (void);     /* 本次 RAM 里的条数 */
uint16  bench_log_stored    (void);     /* 可导出条数, 即上次 end 冻结的条数 */
uint8   bench_log_full      (void);     /* 1 = 缓冲已满, 后续点被丢弃 */

#endif
