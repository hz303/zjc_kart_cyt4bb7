/* 工程 10（卡丁）· 跑班数据记录（**纯 RAM + 串口导 CSV**）
 * ---- 记录格式（12 字节/条）--------------------------------------------------------------
 * int16  duty_x10      **本拍实际施加的占空比** ×10（带符号，% ；不是目标值！）
 * int16  rps_l_x100    左轮实测转速 ×100 rps
 * int16  rps_r_x100    右轮实测转速 ×100 rps
 * int16  seg           小段编号（1~12，见 spd_bench.c 的排班表）
 * ⇒ **发的是"实际施加"的 duty，不是"我让它给"的 duty** —— 与遥测那条纪律一致：
 * 每个小段开始后的 **前 1000 ms：每拍都记（5 ms 一个点）** —— 抓 τ 量级的瞬态；
 * 之后：**每 100 ms 记一个点** —— 抓稳态（K / 悬崖）。
 * ⇒ 全跑班约 3900 条（≈ 23 小段 × ~215 条），容量 4096，**留了 5% 余量**。
 * 现场跑班数据改由手机 App「卡丁跑班」（`KartPhone/`）记录 ⇒ 车端这一份只做两件事：
 * ① RAM 里攒（`BL_CAPACITY` 条，12 B/条 = 48 KB）；
 * ② 需要时**整段从 RAM 走调试串口导 CSV**（`bench_log_dump`）。
 * ⇒ 掉电即丢。要长期留，用 App 那份；要看单次台架曲线，跑完立刻 dump。
 * 31 页的写入会顶穿 `spd_safety_heartbeat()` 的 200 ms 门；
 * 既然 App 已经能把数据带走，车端没有理由再冒这个风险。
 */

#ifndef _BENCH_LOG_H
#define _BENCH_LOG_H

#include "zf_common_headfile.h"

/* 记录状态 */
#define BL_IDLE         (0u)        /* 没在录，也没有待写数据 */
#define BL_REC          (1u)        /* 正在录（跑班进行中） */
#define BL_WRITE        (2u)        /* 已结束、等待导出（保留编号，语义=已冻结） */
#define BL_DONE         (3u)        /* 有数据可导出 */
#define BL_ERR          (4u)        /* 缓冲溢出 / 参数错 */
#define BL_DUMP         (5u)        /* 正在串口导出 CSV */

/* 采样策略（5ms 节拍下） */
#define BL_BURST_MS     (1000u)     /* 每小段前多少 ms 用 5ms 密采 */
#define BL_STEADY_MS    (100u)      /* 之后每多少 ms 记一点 */

/* RAM 缓冲条数：4096 × 12 B = 48 KB（CM7_0 有 384 KB，够） */
#define BL_CAPACITY     (4096u)

/* 导出节流单位：主循环每圈最多发这么多条（500 × ≈30 B ≈ 15 ms @115200， 够快又不会把主循环 >200 ms 的心跳门顶穿） */
#define BL_EXPORT_PER_PAGE  (500u)

/* 开始一次记录（清 RAM 计数；**不碰 Flash**） */
void    bench_log_begin     (void);

/* t_in_seg_ms = 本小段已进行的 ms；seg = 小段编号；duty/rps 为**本拍实际值** */
void    bench_log_tick      (uint16 t_in_seg_ms, uint8 seg, float duty, float rps_l, float rps_r);

void    bench_log_tick_clock(void);

/* 跑班结束：冻结条数（RAM 里原样留着等导出）。complete=1 正常跑完 / 0 被中止 */
void    bench_log_end       (uint8 complete);

/* 主循环每圈调一次：导出状态机（每次最多一页，绝不长阻塞） */
void    bench_log_poll      (void);

/* 请求导出 CSV 到调试串口（主循环分页发，可随时中断） */
void    bench_log_dump      (void);

/* 清空 RAM 记录 */
void    bench_log_clear     (void);

uint8   bench_log_state     (void);
uint16  bench_log_count     (void);
uint16  bench_log_stored    (void);     /* 可导出的条数（= 上一次 bench_log_end 冻结的条数） */
uint8   bench_log_full      (void);     /* 1 = RAM 缓冲已满（后面的点被丢掉） */

#endif
