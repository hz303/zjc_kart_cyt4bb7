#ifndef _STEER_PID_H
#define _STEER_PID_H

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 位置式 PID(输出 = P + I + D), 带积分限幅、输出限幅、饱和时撤积分抗饱和。
 *
 * 积分不乘采样周期: err_sum 是每拍误差的累加, 故 Ki 量纲 = 输出单位/(误差单位·拍),
 * 改控制周期必须重标 Ki。i_max 限的是积分项 Ki*err_sum 的绝对值, 不是 err_sum。
 ********************************************************************************************************************/

typedef struct
{
    float   Kp;
    float   Ki;
    float   Kd;

    float   err;                /* 本次误差 */
    float   err_last;           /* 上次误差, 算 D 用 */
    float   err_sum;            /* 误差累加, 算 I 用 */

    float   i_max;              /* 积分项上限 */
    float   out_max;            /* 输出上限, 对称 */

    float   output;             /* 本拍输出 */
    uint8   first_cal;          /* 1 = 还没算过 D */
} steer_pid_t;

void    steer_pid_init      (steer_pid_t *pid, float kp, float ki, float kd, float i_max, float out_max);
void    steer_pid_update    (steer_pid_t *pid, float err);
void    steer_pid_reset     (steer_pid_t *pid);

#endif
