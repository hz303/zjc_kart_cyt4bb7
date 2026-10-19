/*********************************************************************************************************************
 * 位置式 PID 实现 —— 接口与量纲说明见 steer_pid.h
 * 算法移植自 北极星队 Polaris-Kart-TC387 的 code/Kart_Algo/kart_pid.c(未改算法)
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "steer_pid.h"

void steer_pid_init (steer_pid_t *pid, float kp, float ki, float kd, float i_max, float out_max)
{
    pid->first_cal = 1;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum  = 0.0f;

    pid->i_max   = i_max;
    pid->out_max = out_max;

    pid->output = 0.0f;
}

void steer_pid_update (steer_pid_t *pid, float err)
{
    float p;
    float i;
    float d;

    pid->err = err;

    /* --- 积分累加 --- */
    pid->err_sum += pid->err;

    /* 积分限幅: i_max 限的是"积分项", 除以 Ki 换算成 err_sum 的上限 */
    if (pid->Ki != 0.0f)
    {
        float sum_limit = pid->i_max / pid->Ki;

        if (pid->err_sum > sum_limit)
        {
            pid->err_sum = sum_limit;
        }
        else if (pid->err_sum < -sum_limit)
        {
            pid->err_sum = -sum_limit;
        }
    }

    /* --- P / I / D --- */
    p = pid->err * pid->Kp;
    i = pid->err_sum * pid->Ki;
    d = (pid->err - pid->err_last) * pid->Kd;

    pid->err_last = pid->err;

    if (1 == pid->first_cal)
    {
        pid->output    = p + i;         /* 第一次没有"上次误差", D 无意义 */
        pid->first_cal = 0;
    }
    else
    {
        pid->output = p + i + d;
    }

    /* --- 输出限幅 + 抗积分饱和(饱和就把这一拍的累加撤回) --- */
    if (pid->output > pid->out_max)
    {
        pid->output   = pid->out_max;
        pid->err_sum -= pid->err;
    }
    else if (pid->output < -pid->out_max)
    {
        pid->output   = -pid->out_max;
        pid->err_sum -= pid->err;
    }
}

void steer_pid_reset (steer_pid_t *pid)
{
    pid->first_cal = 1;
    pid->err       = 0.0f;
    pid->err_last  = 0.0f;
    pid->err_sum   = 0.0f;
    pid->output    = 0.0f;
}
