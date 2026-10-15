#ifndef _STEER_PID_H
#define _STEER_PID_H

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 位置式 PID(输出 = P + I + D), 带积分限幅 + 输出限幅 + 饱和时"撤积分"抗饱和。
 *
 * 移植自 北极星队 Polaris-Kart-TC387 的 code/Kart_Algo/kart_pid.c
 *   (该工程又源于 TopSpeed 越野车久经比赛验证的 PID)。
 * 只去掉原工程的头文件依赖, 算法一字未改。
 *
 * ⚠⚠ 积分【不乘采样周期】。err_sum 是"每拍误差的累加", 所以
 *     Ki 的量纲 = 输出单位 / (误差单位 · 拍)。
 *     ⇒ 换控制周期(5ms -> 10ms)必须重标 Ki, 不能照搬。
 * ⚠ i_max 限的是【积分项 Ki*err_sum 的绝对值】, 不是 err_sum 本身。
 ********************************************************************************************************************/

typedef struct
{
    float   Kp;
    float   Ki;
    float   Kd;

    float   err;                /* 本次误差           */
    float   err_last;           /* 上次误差(算 D 用)  */
    float   err_sum;            /* 误差累加(算 I 用)  */

    float   i_max;              /* 积分项上限         */
    float   out_max;            /* 输出上限(对称)     */

    float   output;             /* 本拍输出           */
    uint8   first_cal;          /* 1 = 还没算过 D(第一次跳过 D) */
} steer_pid_t;

void    steer_pid_init      (steer_pid_t *pid, float kp, float ki, float kd, float i_max, float out_max);
void    steer_pid_update    (steer_pid_t *pid, float err);
void    steer_pid_reset     (steer_pid_t *pid);

#endif
