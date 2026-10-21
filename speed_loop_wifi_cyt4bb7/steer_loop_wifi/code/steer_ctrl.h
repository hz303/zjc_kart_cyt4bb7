#ifndef _STEER_CTRL_H
#define _STEER_CTRL_H

#include "zf_common_headfile.h"
#include "steer_abs.h"

/* 转向控制 —— 位置环(纯 P + 静摩擦前馈)
 * → 有刷电机断电不回中, 必须靠"转角位置环"把它按在目标角度上。**这是刚需, 不是可选项。**
 * u = kp * err  +  u0 * sign(err)
 * ‾‾‾‾‾‾‾‾‾‾‾   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 * 细调(比例)     克服静摩擦与减速箱阻力(前馈)
 * err 先过【死区】(±0.2 度), 死区内 err=0 ⇒ u0 也不加 ⇒ 静止不抖。
 * 但直流电机 + 减速箱的静摩擦是 Coulomb 型的【常数非线性】, 让积分去逼近它有两个副作用:
 * ① 启动延迟 —— 积分要爬好几拍才够力, 方向盘"先愣一下再动";
 * ② 一旦动起来, 静摩擦突降到动摩擦, 那一堆积分值全变成过冲, 要靠反向积分慢慢退。
 * 前馈直接把这部分出力按符号给出去, 积分就不需要了 ⇒ 结构更简单、响应更干脆。
 * ② 目标软限位 (LIMIT_DEG): 每拍夹一次, 别让控制器主动把方向盘顶向机械硬限位;
 * ③ 越限保护: 实测已超软限位、且误差还指向限位方向 ⇒ 本拍 err 置 0, 不再加力;
 * ④ 失能时【只在"使能->失能"那一拍写 0】, 之后不再每拍写 —— 否则会和手动开环输出互相打架。
 * 再把 steer_ctrl_set_head_enable(1) 即可, 不用改结构。
 * (project/code/Param_Storage.c 的 DEFAULT_PARAMS 末尾两项: 300.0f, 1600.0f)。
 * 两边现在是**同一套硬件** —— 他把后轮正交编码器也改成了脉冲+方向、并采用了我们的引脚,
 * 所以这组值可以直接用, 不必从 0 开始试。
 */

/* --- 位置环默认参数 ---- 要动必须先在上位机把「转向|内环使能」打 1。 */
#define STEER_KP_DEFAULT            (300.0f)
#define STEER_U0_DEFAULT            (1600.0f)
#define STEER_U_MAX_DEFAULT         (3500.0f)   /* 输出限幅   单位 duty      (与 Motor_Set 刻度一致: ±10000 = ±100%) */
#define STEER_DEADBAND_DEG_DEFAULT  (0.2f)      /* 误差死区   单位 度 */
#define STEER_SLEW_DEG_S_DEFAULT    (40.0f)     /* 目标变化率上限 单位 度/秒 */
#define STEER_LIMIT_DEG_DEFAULT     (STEER_ABS_SOFT_LIMIT_DEG)  /* 目标软限位 单位 度 */

/* --- 外环(航向环)默认: 预留, 本工程默认关 --- */
#define STEER_HEAD_KP_DEFAULT       (15.0f)

/* 控制周期(秒): 必须与调用它的节拍一致(转向环跑在 5ms 中断里) */
#define STEER_T_S                   (0.005f)

/* 输出符号: 目标转角为正(向右)时, 转向电机该往哪边给 duty。
 * 我们: duty = (+1)·u  → motor_drv_set(ch2) → mot_inv_s=1 时再取反 → 合计 −u
 * (两边的角度极性都是 STEER_ABS_DIR_SIGN = −1, 所以只需要对齐这一处输出符号。)
 */
#define STEER_LOOP_SIGN             (+1)

/* 本工程转向电机在 motor_drv 里的通道号 */
#define MOTOR_CH_STEER              (2)

/* IMU 航向(度, -180~180)。本工程没有 IMU → 默认 0 且外环关闭; 将来接上 IMU 后由 IMU 模块每拍写这个变量即可打开外环。 */
extern volatile float steer_yaw_deg;

/* 上位机「转向|内环使能」开关写的是这个变量(不是 angle_enable 本身)。 由 steer_ctrl_update() 每拍按边沿同步过去。 */
extern volatile float steer_en_cmd;

typedef struct
{
    /* --- 可调参数(参数表/Flash 直接写这些字段) --- */
    float   kp;                 /* 比例增益 duty/度 */
    float   u0;                 /* 静摩擦前馈 duty */
    float   u_max;              /* 输出限幅 duty */
    float   deadband_deg;       /* 误差死区 度 */
    float   slew_deg_s;         /* 目标变化率上限 度/秒(0 = 关) */
    float   limit_deg;          /* 目标软限位 度 */

    /* --- 外环(航向) --- */
    float   head_kp;

    /* --- 运行状态 --- */
    uint8   angle_enable;       /* 内环使能(=1 才驱动转向电机) */
    uint8   head_enable;        /* 外环使能(=1 时目标转角由航向环给) */
    uint8   was_enabled;        /* 上一拍是否使能(失能时只写一次 0) */

    float   target_delta;       /* 内环目标转角(度, 向右为正; 上位机写) */
    float   applied_delta;      /* 斜坡之后实际施加的目标(度) */
    float   meas_delta;         /* 内环实测转角(度) */
    float   err;                /* 本拍误差(度, 已过死区与越限保护) */
    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */
    int16   output_duty;        /* 本拍转向电机输出(-10000~10000) */

    float   target_yaw;         /* 外环目标航向(度) */
    float   meas_yaw;           /* 外环实测航向(度) */
} steer_ctrl_t;

extern steer_ctrl_t steer;

void    steer_ctrl_init         (void);
void    steer_ctrl_update       (void);         /* 5ms 每拍: 必须排在 steer_abs_update() 之后 */

void    steer_ctrl_set_angle_enable (uint8 en);
void    steer_ctrl_set_head_enable  (uint8 en);
void    steer_ctrl_set_target_delta (float deg);
void    steer_ctrl_set_target_yaw   (float yaw);

void    steer_ctrl_set_angle_pid    (float kp);
void    steer_ctrl_set_u0           (float u0);
void    steer_ctrl_set_angle_outmax (float outmax);

float   steer_ctrl_get_target_delta (void);
float   steer_ctrl_get_meas_delta   (void);
int16   steer_ctrl_get_output       (void);
float   steer_ctrl_get_target_yaw   (void);

#endif
