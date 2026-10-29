/*********************************************************************************************************************
 * 双电机速度环(WiFi) - 速度环控制器(前馈 + PI, 即 LQI 结构), 双通道
 *
 * ---- 控制律(每路独立) ----
 *      u = u_ff(r)  +  k_v*(r - v)  +  k_z*积分(r - v)dt
 *      u_ff(r) = r / K  +  u0 * sign(r)          线性前馈 + 摩擦截距
 *
 *      u   归一化占空比, 带符号, +-1 = +-100%
 *      v   实测转速 rps, r 目标转速 rps
 *      K   被控对象稳态增益 rps/单位占空比, u0 摩擦截距(占空比)
 *
 * ---- 默认参数 ----
 *      K = 6.46 rps/占空比, tau 约 24 ms, 反馈等效延迟约 10 ms
 *      k_v = 0.164, k_z = 0.512, T_i = k_v/k_z = 0.32 s
 *      u0 默认 0.057; K、摩擦、时间常数与延迟随负载变, 前馈失配由积分补偿
 ********************************************************************************************************************/

#ifndef _SPD_CTRL_H
#define _SPD_CTRL_H

#include "zf_common_headfile.h"

#define SPD_T_S                 (0.005f)        /* 控制周期 s, 必须与 PIT 的 5ms 一致 */

#define SPD_ZERO_RPS            (0.02f)         /* 速度指令低于它 = 视为没有指令 */
#define SPD_ZERO_V_RPS          (0.10f)         /* 且实测速度也低于它 = 车没在动 */
#define SPD_FF_DEADBAND         (0.05f)         /* |r| 低于它只加线性前馈, 免目标 0 时输出抖 +-u0 */
#define SPD_Z_MAX               (5.0f)          /* 积分状态限幅 rps*s, 防积分饱和 */

/* 目标斜坡 + 大跳变清积分: 上位机直接改写目标, 等于把阶跃灌进环路 */
#define SPD_SLEW_DEFAULT        (20.0f)         /* 目标变化率上限 rps/s, 0 = 关闭斜坡 */
#define SPD_INT_CLR_STEP        (0.30f)         /* 相邻两拍命令跳变超过它才清一次积分 */

#define SPD_K_V_DEFAULT         (0.164f)        /* 比例增益 k_v, u/rps */
#define SPD_K_Z_DEFAULT         (0.512f)        /* 积分增益 k_z, u/(rps*s) */
#define SPD_INV_K_DEFAULT       (0.1548f)       /* 前馈增益 1/K, 占空比/rps */
#define SPD_U0_DEFAULT          (0.057f)        /* 摩擦截距 u0, 归一化占空比 */
#define SPD_TARGET_MAX_DEFAULT  (3.6f)          /* 目标转速上限 rps */

/* ---- 全局参数(上位机通过参数表在线改) ---- */
typedef struct
{
    volatile float  k_v;                /* 比例增益  u/rps                          */
    volatile float  k_z;                /* 积分增益  u/(rps*s)                      */
    volatile float  inv_K;              /* 1/K      占空比/rps                      */
    volatile float  u0;                 /* 摩擦截距(归一化占空比)                   */
} spd_gains_t;

extern spd_gains_t      spd_g;
extern volatile float   spd_target_l;   /* 目标转速 左轮 rps(上位机下发) */
extern volatile float   spd_target_r;   /* 目标转速 右轮 rps(上位机下发) */
extern volatile float   spd_target_max; /* |target| 上限 rps           */
extern volatile float   spd_slew_rps_s; /* 目标变化率上限 rps/s (0=关) */

/* ---- 开环 duty 模式 ----
 *
 *  spd_ol_en > 0.5 时两路直接给占空比, 绕过速度环; 用于开环阶跃标定 K / tau / 纯滞后。
 *      spd_ol_duty_l / _r 单位 %, 带符号, 正 = 车前进
 *  仍过 spd_safety_allow() 门控与占空比上限 u_max(硬顶见 spd_safety.h);
 *  进出开环都清积分与斜坡状态; 模式切换不是无扰切换, 标定前先停稳。
 */
extern volatile float   spd_ol_en;      /* >0.5 = 两路都走开环(直接给 duty, 绕过速度环) */
extern volatile float   spd_ol_duty_l;  /* 左轮开环占空比 %, 带符号(-100~100) */
extern volatile float   spd_ol_duty_r;  /* 右轮开环占空比 %, 带符号(-100~100) */

/* ---- 电子差速: 左右轮目标速度分配 ----
 *
 *  双电机独立驱动, 无机械差速器。按后轴中心速度 v(m/s) 与曲率 k = tan(delta)/L 分配:
 *      v_l = v*(1 - k*W/2)        v_r = v*(1 + k*W/2)
 *  两轮同号且平均恒等于 v, 故里程 ds = (dl+dr)/2 不受差速影响。
 *  用实测舵角而非目标舵角; 内侧最低 15%; 倒车低于门限不分, 防低速抖。
 */
#define SPD_DIFF_W_DEFAULT      (0.605f)    /* 轮距 m, 左右轮心间距 */
#define SPD_DIFF_MAX_DEFAULT    (0.85f)     /* 单侧偏差上限, 内侧最低 15% */
/* 倒车差速门限: 0.3 rps = 0.246 m/s; 前进 0.02 m/s 即分配 */
#define SPD_DIFF_REV_V          (0.246f)

extern volatile float   spd_diff_en;    /* >0.5 = 开电子差速                     */
extern volatile float   spd_diff_w;     /* 轮距 m                                */
extern volatile float   spd_diff_max;   /* 单侧偏差上限, 0.85 = 内侧最低 15%      */

/* 把后轴中心速度 v(m/s) 按实测舵角分成左右轮目标速度(m/s)。
   vl/vr 一定同号且平均等于 v; 静止、以及倒车 |v| < SPD_DIFF_REV_V 时原样返回不分。 */
void spd_diff_alloc (float v_mps, float L_m, float delta_meas_deg, float *vl, float *vr);

/* ---- 单路状态(遥测用) ---- */
typedef struct
{
    volatile float  e;          /* 本拍速度误差 rps                       */
    volatile float  z;          /* 积分状态                               */
    volatile float  u_ff;       /* 前馈分量(归一化)                       */
    volatile float  u_cmd;      /* 限幅前的控制器输出(归一化)             */
    volatile float  u;          /* 本拍真正给出去的控制器输出(归一化)     */
    volatile float  u_max;      /* 本拍允许的输出上限(由 spd_safety 下压) */
    volatile uint8  sat;        /* 1 = 本拍输出被夹或抗饱和冻结积分       */
    volatile uint8  enabled;    /* 1 = 本拍允许输出(开环/闭环均适用)      */
    volatile uint32 steps;      /* 已执行的控制拍数(应匀速增长)           */
    volatile float  r_app;      /* 本拍实际施加的目标(斜坡后的值)         */
    volatile float  r_cmd_prev; /* 上拍限幅后的命令, 仅用于跳变检测       */
} spd_ctrl_st_t;

extern spd_ctrl_st_t    ctl_l;
extern spd_ctrl_st_t    ctl_r;

/* ---- 接口 ---- */
void    spd_ctrl_init       (void);

/* 5ms 主节拍: 保护判定 -> 两路测速 -> 两路控制 -> 写 PWM/DIR。
   由 cm7_0_isr.c 的 pit0_ch2_isr() 调用, 是整个环的唯一入口。 */
void    spd_ctrl_tick       (void);

void    spd_ctrl_reset_int  (void);             /* 清两路积分(标定/换参数后用) */

/* 自由轮: 两后轮输出钉 0, 测速与里程照常更新(推车时航向/位姿仍有效)。
   on=1 时不跑 PI 且积分清零; 只影响两后轮, 不动转向; 退出时由调用方清 0。 */
void    spd_ctrl_set_coast  (uint8 on);
uint8   spd_ctrl_coast      (void);

#endif
