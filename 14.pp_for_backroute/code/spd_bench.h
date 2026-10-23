/* 工程 10（卡丁）· 速度环"12 段跑班"自动序列器
 * 来源：`待办-实车标定.md` §5.3 的**手动排班表**（工程 8 里没有固件实现，这是新写的）。
 * 目的：一趟把 **K / u0 / 悬崖(非零稳态下限) / τ / 重复性** 全测出来，用来解速度环超调。
 * 段 | 占空比档            | 每档时长 | 目的
 * 1  | 6 / 8 / 10 %        | 3×2 s    | 确认仍在锁死带（v≡0，车不动）
 * 2  | 11 / 12 / 13 %      | 3×3 s    | 逼近破悬崖点
 * 3  | 13.5 / 14 / 14.5 %  | 3×3 s    | 细分（悬崖分辨率 ±0.5%）
 * 4  | 15 / 15.5 / 16 %    | 3×3 s    | 悬崖上沿
 * 5  | 18 %                | 3 s      | 直线段起点
 * 6  | 21 %                | 3 s      | 直线段
 * 7  | 25 %                | 3 s      | 直线段
 * 8  | 30 %                | 3 s      | 直线段
 * 9  | 35 %                | 3 s      | 端点校验
 * 10  | 25 % ×2             | 2×3 s    | 同文件重复性（判漂移）
 * 稳定的起点。**这两行如果与你原意不符，改 SB_STEPS[] 里对应的两行即可。**
 * ---- 运行前提（都由本模块自己保证，不用你手动设）----------------------------------------
 * · `spd_target_l/r = 0`（开环模式下速度环被绕过，但保持 0 是纪律）
 * · `spd_ol_en = 1`，`spd_ol_duty_l/r = 本小步的占空比`
 * · 5 ms 节拍（`spd_bench_tick()` 在 `pit0_ch2_isr` 里，紧跟 `spd_ctrl_tick()` 之后）
 * ① **遥控不在线（或没合闸）根本不让启动** —— 因为开环要有输出，必须靠遥控的
 * ⇒ 顺带把"车端有人在、手能摸到急停"这件事变成了**硬前提**。
 * ② **跑班全程**每 5 ms 复核这四条，任何一条破了立刻**回 0 占空比 + 退出开环 + 中止记录**：
 * · `RC_Online == 1`           接收机没失控
 * · `RC_Engage == STANDBY`     油门回中、没被别的路径接管（一变 ON/OFF/FAILSAFE 就停）
 * · `RC_Estop == 0`            CH5 没拨急停
 * · `spd_safety_allow(0) != 0` 保护层允许出力
 * ③ **CH5 急停**另有独立的一条硬通路：`remote_ctrl_apply()` → `spd_safety_set_estop(1)`
 * → 保护层把输出**无条件钉 0**（`SAFE_BIT_ESTOP`，本模块绕不过，也不需要绕）。
 * 本模块的检查只是"抢在它前面把开环也撤掉"，让车不会在急停解除后自己接着跑。
 * 避免两处抢同一个档位。
 */

#ifndef _SPD_BENCH_H
#define _SPD_BENCH_H

#include "zf_common_headfile.h"

#define SB_IDLE         (0u)
#define SB_RUN          (1u)        /* 跑班进行中 */
#define SB_WAITLOG      (2u)        /* 跑完了，等 bench_log 落盘 */
#define SB_DONE         (3u)        /* 正常结束且数据已落盘 */
#define SB_ABORT        (4u)        /* 中途被中止（数据仍会落盘，complete=0） */
#define SB_REFUSED      (5u)        /* 启动被拒（见 spd_bench_refuse()） */

#define SB_REF_BUSY         (1u)    /* 已经在跑，或日志正在落盘 */
#define SB_REF_NO_RC        (2u)    /* 遥控接收机不在线（没插 / 失控） */
#define SB_REF_ESTOP        (3u)    /* CH5 急停生效中 */
#define SB_REF_NOT_STANDBY  (4u)    /* 遥控没在 STANDBY：CH6 没合闸 / 油门没回中 */

#define SB_AB_ESTOP         (1u)
#define SB_AB_NO_RC         (2u)
#define SB_AB_NOT_STANDBY   (3u)
#define SB_AB_SAFETY        (4u)
#define SB_AB_MANUAL        (5u)

void    spd_bench_init      (void);

/* 一键启动。返回 0 = 已开始；非 0 = 拒绝码(SB_REF_*) */
uint8   spd_bench_start     (void);

/* 手动中止（菜单用；数据仍会落盘） */
void    spd_bench_abort     (uint8 code);

void    spd_bench_tick      (void);

/* 主循环每圈调一次：收尾（落盘交给 bench_log_poll） */
void    spd_bench_poll      (void);

uint8   spd_bench_state     (void);
uint8   spd_bench_refuse    (void);
uint8   spd_bench_abort_code(void);
uint8   spd_bench_seg       (void);     /* 当前小段编号 1~12；0 = 段间停机 */
uint8   spd_bench_step      (void);     /* 当前小步下标(0 基) */
uint8   spd_bench_nsteps    (void);     /* 总小步数 */

#endif
