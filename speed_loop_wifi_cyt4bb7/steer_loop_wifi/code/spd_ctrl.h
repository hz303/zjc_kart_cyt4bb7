/* 双电机速度环(WiFi) - 速度环控制器(前馈 + PI, 即 LQI 结构), 双通道
 * ---- 控制律(每路独立) --------------------------------------------------------------
 * u = u_ff(r)  +  k_v*(r - v)  +  k_z*∫(r - v)dt
 * u_ff(r) = r / K  +  u0 * sign(r)        (线性前馈 + 摩擦截距)
 * 符号: u 归一化占空比(带符号, +-1 = +-100%)
 * v 实测转速 rps, r 目标转速 rps
 * K 被控对象稳态增益 rps/单位占空比, u0 摩擦截距(占空比)
 * 把状态取成 [v, z](z = ∫e), 控制律就是 u = u_ff - [k_v, -k_z]*[v, z],
 * 求出来的是**同一个控制器的两组系数**, 结构一模一样。
 * 被控对象: K = 6.46 rps/占空比, u0 = 0.033, tau ~ 24 ms, 反馈等效延迟 ~10 ms
 * 注意: 这组数只对**这台驱动电机**(要装到卡丁后轮上的那台)成立。
 * 另一台转向电机(当舵机用)的 K 约 215, 与本电机参数不能互用。
 * K 会随负载变: 空载 K 最大(稳定性最差工况), 落地带载后 K 变小、u0 变大
 */

#ifndef _SPD_CTRL_H
#define _SPD_CTRL_H

#include "zf_common_headfile.h"

#define SPD_T_S                 (0.005f)        /* 控制周期, 必须与 PIT 的 5ms 一致 */
#define SPD_FF_DEADBAND         (0.05f)         /* |r| 小于它就只加线性前馈(避免目标 0 时输出抖 +-u0) */
#define SPD_Z_MAX               (5.0f)          /* 积分状态限幅(rps*s), 防积分裹死 */

/* 上位机拖滑条是**直接把值写进 spd_target_l**（走参数表, 不经过任何函数），
 * ① 前馈瞬间打满 -> 一开始就冲过目标;
 * SPD_SLEW_DEFAULT: 目标变化率上限 rps/s。0 = 关闭斜坡(恢复原行为)。
 * 20 rps/s ⇒ 0→1.5 rps 用 75ms; 比赛指令本来就是百毫秒级变化的, 够快。
 * SPD_INT_CLR_STEP: 目标跳变超过它就清积分, 防止旧设定值的"悬念"留在积分里。
 */
#define SPD_SLEW_DEFAULT        (20.0f)
#define SPD_INT_CLR_STEP        (0.30f)

#define SPD_K_V_DEFAULT         (0.164f)
#define SPD_K_Z_DEFAULT         (2.32f)
#define SPD_INV_K_DEFAULT       (0.1548f)       /* 1/6.46 */
#define SPD_U0_DEFAULT          (0.033f)
#define SPD_TARGET_MAX_DEFAULT  (3.0f)          /* 目标转速上限 rps(先保守) */

/* 全局参数(上位机通过参数表在线改) */
typedef struct
{
    volatile float  k_v;                /* 比例增益  u/rps */
    volatile float  k_z;                /* 积分增益  u/(rps*s) */
    volatile float  inv_K;              /* 1/K      占空比/rps */
    volatile float  u0;                 /* 摩擦截距(归一化占空比) */
} spd_gains_t;

extern spd_gains_t      spd_g;
extern volatile float   spd_target_l;   /* 目标转速 左轮 rps(上位机下发) */
extern volatile float   spd_target_r;   /* 目标转速 右轮 rps(上位机下发) */
extern volatile float   spd_target_max; /* |target| 上限 rps */
extern volatile float   spd_slew_rps_s; /* 目标变化率上限 rps/s (0=关) */

/* 单路状态(遥测用) */
typedef struct
{
    volatile float  e;          /* 本拍速度误差 rps */
    volatile float  z;          /* 积分状态 */
    volatile float  u_ff;       /* 前馈分量(归一化) */
    volatile float  u_cmd;      /* 限幅前的控制器输出(归一化) */
    volatile float  u;          /* 本拍真正给出去的控制器输出(归一化) */
    volatile float  u_max;      /* 本拍允许的输出上限(由 spd_safety 下压) */
    volatile uint8  sat;        /* 1 = 本拍发生饱和(积分被冻结) */
    volatile uint8  enabled;    /* 1 = 闭环使能 */
    volatile uint32 steps;      /* 已执行的控制拍数(应匀速增长) */
    volatile float  r_app;      /* 本拍实际施加的目标(斜坡后的值) */
} spd_ctrl_st_t;

extern spd_ctrl_st_t    ctl_l;
extern spd_ctrl_st_t    ctl_r;

void    spd_ctrl_init       (void);

/* 5ms 主节拍: 保护判定 -> 两路测速 -> 两路控制 -> 写 PWM/DIR。 由 cm7_0_isr.c 的 pit0_ch2_isr() 调用, 是整个环的唯一入口。 */
void    spd_ctrl_tick       (void);

void    spd_ctrl_reset_int  (void);             /* 清两路积分(标定/换参数后用) */

#endif
