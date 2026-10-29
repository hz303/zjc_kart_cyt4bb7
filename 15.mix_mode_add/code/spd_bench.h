/* 速度环 12 段跑班自动序列器: 一趟测出 K / u0 / 非零稳态下限 / tau / 重复性
 *
 *  占空比序列: 6/8/10 %, 11/12/13 %, 13.5/14/14.5 %, 15/15.5/16 % (每档 2~3 s)
 *              18 %, 21 %, 25 %, 30 %, 35 % (各 3 s), 25 % x2 (2x3 s), 末两段 20->30 / 30->20 % 阶跃各 3 s
 *  每小步之间停机 SB_GAP_MS = 2000 ms(duty = 0); 阶跃段前置电平保持 1500 ms
 *  安全门: 遥控不在线或不在 STANDBY 不许启动; 全程每 5 ms 复核 RC_Online / RC_Engage /
 *          RC_Estop / spd_safety_allow(0), 任一破即回 0 占空比、退出开环并中止记录
 *  运行前提: spd_target_l/r = 0, spd_ol_en = 1, spd_ol_duty_l/r 为当前小步占空比 */

#ifndef _SPD_BENCH_H
#define _SPD_BENCH_H

#include "zf_common_headfile.h"

#define SB_IDLE         (0u)        /* 空闲 */
#define SB_RUN          (1u)        /* 跑班进行中 */
#define SB_WAITLOG      (2u)        /* 跑完了, 等 bench_log 落盘 */
#define SB_DONE         (3u)        /* 正常结束且数据已落盘 */
#define SB_ABORT        (4u)        /* 中途被中止(数据仍落盘, complete=0) */
#define SB_REFUSED      (5u)        /* 启动被拒, 拒绝码由 spd_bench_refuse() 读出 */

/* 拒绝码 */
#define SB_REF_BUSY         (1u)    /* 已在跑, 或日志正在落盘 */
#define SB_REF_NO_RC        (2u)    /* 遥控接收机不在线(没插 / 失控) */
#define SB_REF_ESTOP        (3u)    /* CH5 急停生效中 */
#define SB_REF_NOT_STANDBY  (4u)    /* CH6 未合闸或油门未回中 */

/* 中止码 */
#define SB_AB_ESTOP         (1u)
#define SB_AB_NO_RC         (2u)
#define SB_AB_NOT_STANDBY   (3u)
#define SB_AB_SAFETY        (4u)
#define SB_AB_MANUAL        (5u)

void    spd_bench_init      (void);

/* 一键启动: 返回 0 = 已开始, 非 0 = 拒绝码(SB_REF_*) */
uint8   spd_bench_start     (void);

/* 手动中止(菜单用, 数据仍落盘) */
void    spd_bench_abort     (uint8 code);

/* 5 ms 中断内调用, 紧跟 spd_ctrl_tick() 之后 */
void    spd_bench_tick      (void);

/* 主循环每圈调一次, 做收尾(落盘交给 bench_log_poll) */
void    spd_bench_poll      (void);

uint8   spd_bench_state     (void);
uint8   spd_bench_refuse    (void);
uint8   spd_bench_abort_code(void);
uint8   spd_bench_seg       (void);     /* 当前小段编号 1~12, 0 = 段间停机 */
uint8   spd_bench_step      (void);     /* 当前小步下标(0 基) */
uint8   spd_bench_nsteps    (void);     /* 总小步数 */

#endif
