/***********************************************************************************************************************
 * 速度环 12 段跑班序列器: 一趟测出 K / u0 / 悬崖 / tau / 重复性
 * ---- 12 段排班: 每小步之间停机 2000 ms(duty = 0); 前提 spd_ol_en = 1, spd_target_l/r = 0 ----
 *   段 1: 6/8/10 % 各 2 s       确认仍在锁死带(v 恒 0)
 *   段 2: 11/12/13 % 各 3 s     逼近破悬崖点
 *   段 3: 13.5/14/14.5 % 各 3 s 细分, 分辨率 0.5%
 *   段 4: 15/15.5/16 % 各 3 s   悬崖上沿
 *   段 5~9: 18/21/25/30/35 % 各 3 s, 段 9 为端点校验
 *   段 10: 25 % 两次, 各 3 s    同一趟内的重复性
 *   段 11: 20 % 1.5 s -> 30 % 3 s   定 tau
 *   段 12: 30 % 1.5 s -> 20 % 3 s   反向校验
 * 安全门: 启动与全程每 5 ms 复核 RC_Online / RC_Engage=STANDBY / RC_Estop=0 / spd_safety_allow 非 0; 破了立刻回 0 并退出开环, CH5 急停另由保护层钉 0
 **********************************************************************************************************************/

#ifndef _SPD_BENCH_H
#define _SPD_BENCH_H

#include "zf_common_headfile.h"

/* 状态 */
#define SB_IDLE         (0u)
#define SB_RUN          (1u)
#define SB_WAITLOG      (2u)        /* 跑完了，等 bench_log 落盘 */
#define SB_DONE         (3u)
#define SB_ABORT        (4u)        /* 中途被中止（数据仍会落盘，complete=0） */
#define SB_REFUSED      (5u)        /* 启动被拒（见 spd_bench_refuse()） */

/* 拒绝码 */
#define SB_REF_BUSY         (1u)    /* 已经在跑，或日志正在落盘 */
#define SB_REF_NO_RC        (2u)    /* 遥控接收机不在线（没插 / 失控） */
#define SB_REF_ESTOP        (3u)    /* CH5 急停生效中 */
#define SB_REF_NOT_STANDBY  (4u)    /* 遥控没在 STANDBY：CH6 没合闸 / 油门没回中 */

/* 中止码 */
#define SB_AB_ESTOP         (1u)
#define SB_AB_NO_RC         (2u)
#define SB_AB_NOT_STANDBY   (3u)
#define SB_AB_SAFETY        (4u)
#define SB_AB_MANUAL        (5u)

void    spd_bench_init      (void);

/* 一键启动: 返回 0 = 已开始, 非 0 = 拒绝码 SB_REF_* */
uint8   spd_bench_start     (void);

/* 手动中止(菜单用), 数据仍会落盘 */
void    spd_bench_abort     (uint8 code);

/* 5 ms 中断里调, 紧跟 spd_ctrl_tick() 之后 */
void    spd_bench_tick      (void);

/* 主循环每圈调一次收尾, 落盘由 bench_log_poll 完成 */
void    spd_bench_poll      (void);

uint8   spd_bench_state     (void);
uint8   spd_bench_refuse    (void);
uint8   spd_bench_abort_code(void);
uint8   spd_bench_seg       (void);     /* 当前小段编号 1~12；0 = 段间停机 */
uint8   spd_bench_step      (void);     /* 当前小步下标(0 基) */
uint8   spd_bench_nsteps    (void);

#endif
