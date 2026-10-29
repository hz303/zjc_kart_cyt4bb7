/*********************************************************************************************************************
 * 跑班数据记录, 纯 RAM 加串口导出 CSV
 *
 * 记录格式 12 字节/条: int32 t_ms | int16 duty_x10 | int16 rps_l_x100 | int16 rps_r_x100 | int16 seg
 *   t_ms 本次记录以来的 ms; duty_x10 本拍实际施加的占空比 ×10, 带符号, 单位 %
 *   rps_l_x100 / rps_r_x100 左右轮实测转速 ×100, 单位 rps; seg 小段编号 1~12
 * 记的是实际施加值而非目标值, 安全层可能限幅或钉 0
 * 采样: 每小段前 BL_BURST_MS 内每拍记录, 之后每 BL_STEADY_MS 记一点
 * 存储: 只在 RAM 里攒, 掉电即丢, 需要时整段从调试串口导 CSV, 不写 Flash
 ********************************************************************************************************************/

#ifndef _BENCH_LOG_H
#define _BENCH_LOG_H

#include "zf_common_headfile.h"

/* 记录状态 */
#define BL_IDLE         (0u)
#define BL_REC          (1u)
#define BL_WRITE        (2u)
#define BL_DONE         (3u)
#define BL_ERR          (4u)
#define BL_DUMP         (5u)

/* 采样策略, 5 ms 节拍下, BURST 内密采, 之后按 STEADY 抽点 */
#define BL_BURST_MS     (1000u)
#define BL_STEADY_MS    (100u)

/* RAM 缓冲条数, 4096 × 12 B = 48 KB */
#define BL_CAPACITY     (4096u)

/* 导出节流: 主循环每圈最多发这么多条, 500 × 约 30 B 约 15 ms @115200 */
#define BL_EXPORT_PER_PAGE  (500u)

/* 开始一次记录, 清 RAM 计数, 不碰 Flash */
void    bench_log_begin     (void);

/* 5 ms 中断每拍调用; t_in_seg_ms 本小段已进行 ms, seg 小段号, duty/rps 为本拍实际值 */
void    bench_log_tick      (uint16 t_in_seg_ms, uint8 seg, float duty, float rps_l, float rps_r);

/* 5 ms 中断每拍调用, 在 tick 之后, 只累加运行时间 */
void    bench_log_tick_clock(void);

/* 跑班结束, 冻结条数留在 RAM; complete=1 正常跑完, 0 被中止 */
void    bench_log_end       (uint8 complete);

/* 主循环每圈调一次, 导出状态机, 每次最多一页 */
void    bench_log_poll      (void);

/* 请求导出 CSV 到调试串口, 主循环分页发, 可随时中断 */
void    bench_log_dump      (void);

void    bench_log_clear     (void);

uint8   bench_log_state     (void);
uint16  bench_log_count     (void);     /* 本次 RAM 里的条数 */
uint16  bench_log_stored    (void);     /* 可导出条数, 即上次 bench_log_end 冻结的条数 */
uint8   bench_log_full      (void);     /* 1 = RAM 已满, 后续点被丢弃 */

#endif
