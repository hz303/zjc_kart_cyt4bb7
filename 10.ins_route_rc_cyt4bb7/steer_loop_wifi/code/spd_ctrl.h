/* 双电机速度环(WiFi) - 速度环控制器(前馈 + PI, 即 LQI 结构), 双通道
 * ---- 控制律(每路独立) --------------------------------------------------------------
 * u = u_ff(r)  +  k_v*(r - v)  +  k_z*∫(r - v)dt
 * u_ff(r) = r / K  +  u0 * sign(r)        (线性前馈 + 摩擦截距)
 * 符号: u 归一化占空比(带符号, +-1 = +-100%)
 * v 实测转速 rps, r 目标转速 rps
 * K 被控对象稳态增益 rps/单位占空比, u0 摩擦截距(占空比)
 * 把状态取成 [v, z](z = ∫e), 控制律就是 u = u_ff - [k_v, -k_z]*[v, z],
 * 求出来的是**同一个控制器的两组系数**, 结构一模一样。
 * 被控对象（**台架空载**）: K = 6.46 rps/占空比, u0 = 0.033, tau ~ 24 ms, 反馈等效延迟 ~10 ms
 * （左 7.00 / 右 6.90）, u0 = 5.82% / 5.54% ⇒ 1/K = 0.143~0.145、u0 默认改 **0.057**。
 * 即"K 只差 6%、u0 差了 73%"——落地抬的是**截距**不是斜率，所以前馈的 u0 必须重标，
 * 而 1/K 那 6% 留给积分兜（同 duty 重复性本身就有 4~8%，前馈追不平）。
 * 2.32 的积分太快（T_i = k_v/k_z = 71 ms）⇒ 落地实测定为 **k_z = 0.512**（T_i = 0.32 s），
 * 闭环实测(MM32 台架): 目标 0.5~1.5 rps 跟踪, 稳态误差中位 +-0.01 rps
 * 注意: 这组数只对**这台驱动电机**(要装到卡丁后轮上的那台)成立。
 * 项目里另一台"转向电机"(当舵机用的那个)K 约 215, 差 33 倍, 一个数都不能互抄。
 * K、摩擦、时间常数与延迟都会随负载变; 空载稳定不保证带载不超调。
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
 * SPD_INT_CLR_STEP: 相邻两拍限幅后命令的跳变超过它才清一次积分。
 * 不比较命令与 r_app 的距离, 否则斜坡期间会连续清积分。
 */
#define SPD_SLEW_DEFAULT        (20.0f)
#define SPD_INT_CLR_STEP        (0.30f)

#define SPD_K_V_DEFAULT         (0.164f)
#define SPD_K_Z_DEFAULT         (0.512f)        /* 带载默认 */
#define SPD_INV_K_DEFAULT       (0.1548f)       /* 1/6.46（**台架空载**标定值） 稳态靠积分兜（实测各段稳态误差 ≤2.3%）。 */
/* `u_ff = inv_K·v[rps] + u0`（用 3.0 rps 段验证：0.1452×3 + 0.0619 = 0.4975 vs 实测 0.497）。
 * 0.033 是**空转（无负载）**的摩擦截距；落地后轮胎/减速箱/地面阻力把截距抬了 0.24% duty。
 * 这 0.24% 在 3.0 rps 段只相当于 0.6% 速度，但在 **1.0 rps 段相当于 3.5%** ⇒
 * ---- 纪律 ---------------------------------------------------------------------
 * ⇒ 稳态永远靠积分。这个值错了只影响**超调**和**起步那一脚**。
 * 烧完上电可能仍是旧值，要看串口/参数面板确认。
 */
#define SPD_U0_DEFAULT          (0.057f)
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

/* 给定占空比、看转速怎么爬起来（K / τ / 纯滞后），再从它反算 k_v / k_z / inv_K。
 * 这条路在空转台上做没意义（负载不是同一个），必须在**落地**做。
 * 用法（全部是普通参数，走参数表，不用重烧）：
 * spd_ol_en = 1                 -> 两路都改走开环
 * spd_ol_duty_l / _r = 30.0f    -> 直接给 30% 占空比（带符号，正 = 车前进）
 * spd_ol_en = 0                 -> 交回速度环, 积分清零、目标斜坡从 0 重启
 * 模式切换不是运动中的无扰切换; 标定时应先停稳再切换。
 * 三条**刻意**的约束：
 * ① **仍然过保护门控**：`spd_safety_allow()` 不过就输出 0 —— 开环不是"免死金牌"。
 * ⇒ "WiFi 或遥控至少有一个在场"这条规则照旧管着它。
 * ② **仍然过占空比限幅**：实际输出 = min(|duty|, duty_limit%)。想给到 40% 就先把
 * 「限幅|占空比上限」拖上去（硬顶 SPD_DUTY_HARD_MAX = 60%）。
 * ③ **进开环就把积分清零**、出开环时积分也是干净的 —— 否则"开环跑了一段时间、
 * 误差攒了一堆"再切回闭环，会立刻来一脚猛冲。
 */
extern volatile float   spd_ol_en;      /* >0.5 = 两路都走开环(直接给 duty, 绕过速度环) */
extern volatile float   spd_ol_duty_l;  /* 左轮开环占空比 %, 带符号(-100~100) */
extern volatile float   spd_ol_duty_r;  /* 右轮开环占空比 %, 带符号(-100~100) */

/* 没有机械差速器。转弯时内外轮走过的弧长本来就不同：
 * 满舵 26.8°、R = L/tanδ = 1.30 m、轮距 W = 0.605 m  ⇒  内外差 (W/2)/R = ±23%
 * 实测（wave_20261017_213650.csv 驱动段）左右轮程比 **0.88**，纯滚动应为 0.62
 * ⇒ 里程虚高 **+22%** ⇒ 由里程积分出来的 x/y/yaw 整体偏 ⇒ 回放越跑越偏。
 * ⇒ 差速在这里不是"舒适性配置"，它是**惯导里程正确性的前提**。
 * 分配律（后轴中心速度 v，曲率 κ = tanδ / L）：
 * v_l = v·(1 − κ·W/2)        v_r = v·(1 + κ·W/2)
 * 两边都是"同一个 v 乘一个系数" ⇒ **平均速度恒等于 v** ⇒
 * `ins_odom` 的 `ds = (dl+dr)/2` 不受差速影响（这一点是它能和现有里程/路表共存的关键）。
 * ① **用实测舵角，不用目标舵角**。转向机构有滞后/旷量，用目标会让后轮提前差速，
 * ③ **内侧不许停转**。停转的轮子编码器读 0、速度环积分一直堆，出弯来一脚。
 * 上限 0.85 ⇒ 内侧最低 15%。
 * 所以退化成"单 PID 控左右平均 + duty 前馈偏置"，且 MAX_RATIO 只有 12%（满舵要 21.6%
 * ⇒ 补不满），默认是关的。我们双电机独立、每轮一套编码器，正确做法是在**目标速度**上
 * 分配（闭环下达），而不是在 duty 上打偏置（开环）。
 */
#define SPD_DIFF_W_DEFAULT      (0.605f)    /* 轮距：左右轮**心**间距（实测 60.5 cm） */
#define SPD_DIFF_MAX_DEFAULT    (0.85f)     /* 单侧偏差上限：内侧最低 15% */

extern volatile float   spd_diff_en;    /* >0.5 = 开电子差速 */
extern volatile float   spd_diff_w;     /* 轮距 m */
extern volatile float   spd_diff_max;   /* 单侧偏差上限（0.85 ⇒ 内侧 ≥ 15%） */

/* 把后轴中心速度 v(m/s) 按实测舵角分成左右轮目标速度(m/s)。 */
void spd_diff_alloc (float v_mps, float L_m, float delta_meas_deg, float *vl, float *vr);

/* 单路状态(遥测用) */
typedef struct
{
    volatile float  e;          /* 本拍速度误差 rps */
    volatile float  z;          /* 积分状态 */
    volatile float  u_ff;       /* 前馈分量(归一化) */
    volatile float  u_cmd;      /* 限幅前的控制器输出(归一化) */
    volatile float  u;          /* 本拍真正给出去的控制器输出(归一化) */
    volatile float  u_max;      /* 本拍允许的输出上限(由 spd_safety 下压) */
    volatile uint8  sat;        /* 1 = 本拍输出被夹或抗饱和冻结积分 */
    volatile uint8  enabled;    /* 1 = 本拍允许输出(开环/闭环均适用) */
    volatile uint32 steps;      /* 已执行的控制拍数(应匀速增长) */
    volatile float  r_app;      /* 本拍实际施加的目标(斜坡后的值) */
    volatile float  r_cmd_prev; /* 上拍限幅后的命令, 仅用于跳变检测 */
} spd_ctrl_st_t;

extern spd_ctrl_st_t    ctl_l;
extern spd_ctrl_st_t    ctl_r;

void    spd_ctrl_init       (void);

/* 5ms 主节拍: 保护判定 -> 两路测速 -> 两路控制 -> 写 PWM/DIR。 由 cm7_0_isr.c 的 pit0_ch2_isr() 调用, 是整个环的唯一入口。 */
void    spd_ctrl_tick       (void);

void    spd_ctrl_reset_int  (void);             /* 清两路积分(标定/换参数后用) */

#endif
