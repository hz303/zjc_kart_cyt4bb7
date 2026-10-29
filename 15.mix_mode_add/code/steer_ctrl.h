#ifndef _STEER_CTRL_H
#define _STEER_CTRL_H

#include "zf_common_headfile.h"
#include "steer_abs.h"

/*********************************************************************************************************************
 * 转向位置环(P + I + 静摩擦前馈 + 起步滞环) —— 有刷直流电机 + 绝对编码器, 断电不回中, 必须靠位置环按住转角。
 *
 * 控制律: u = kp*err(死区外) + ki*i_sum + u0*sign(err)(死区外), 合成后按 u_max 限幅
 *   err 永远是真值, 不清零; 三项各自决定本拍参不参与:
 *     死区 |err| < deadband(0.15 度) -> Kp 项为 0, 积分照常累加
 *     前馈 三态滞环: |err| > brk(0.25) 开 / |err| <= deadband 关 / 带内保持上一拍
 *     越限保护 block -> 三项全 0
 *   积分双门限: |err| < i_db(0.10) 不累加; |err| > izone(5 度) 则 i_sum 直接清 0;
 *     其间累加, u_i = ki*i_sum 再按 i_max(1600 duty) 限幅
 *   单位: 方向盘度, 向右为正, 标定见 steer_abs.h
 *   目标每拍夹软限位(limit_deg, 最多 27 度); 实测越限且误差仍指向限位 -> 本拍 不出力
 *   失能只在 使能->失能 那一拍写 0, 之后不再每拍写, 避免与手动开环输出打架
 *   遥测发 applied_delta(实际施加的目标), 不是 target_delta
 *   外环航向预留: 本工程无 IMU 默认关, 写 steer_yaw_deg 再置 head_enable 即可
 ********************************************************************************************************************/

/* ---- 位置环默认参数, 上电全不使能, 方向盘不会 自己动 ---- */
#define STEER_KP_DEFAULT            (400.0f)    /* 比例增益 duty/度 */
#define STEER_U0_DEFAULT            (1700.0f)   /* 静摩擦前馈 duty, 启动阈值约 1300~1600 */

/* 积分增益 duty/(度·拍); 0 = 关积分。5 ms 一拍, 1 度恒误差每秒补 ki*200 duty,
   静差消除时间 t = kp/(200*ki), 与误差大小无关 */
#define STEER_KI_DEFAULT            (25.0f)

/* 积分项上限 duty, 累加值上限 = i_max/ki; 默认 1600 >= 破冰力, 积分可把舵机顶出死区 */
#define STEER_IMAX_DEFAULT          (1600.0f)

/* 前馈开门门限 度, 三态滞环: |err| > brk 开 / |err| <= deadband 关 / 带内保持;
   间隔 brk-db 须大于 噪声振幅(约 7 个计数) */
#define STEER_BRK_DEG_DEFAULT       (0.25f)

/* 积分清零 门限 度: |err| 超过它 i_sum 直接清 0(不是冻结), 大阶跃期间积分不参与 */
#define STEER_IZONE_DEG_DEFAULT     (5.0f)

/* 输出限幅 duty, 与 Motor_Set 刻度一致(±10000 = ±100%); 可在线上位机 ID22 里改 */
#define STEER_U_MAX_DEFAULT         (9000.0f)

/* 误差死区 度: |err| < db 只让 Kp 项为 0, 积分下界是 i_db, 前馈由 brk/db 决定, 三者 不绑死 */
#define STEER_DEADBAND_DEG_DEFAULT  (0.15f)     /* 误差死区 度 */

/* 积分累加下界 度: |err| < 它这一拍不累加, 只挡 纯噪声(编码器 1 计数 = 0.02 度) */
#define STEER_I_DB_DEG_DEFAULT      (0.10f)

/* 目标变化率上限 度/秒, 0 = 不限速率(默认); 滑条 steer_slew 可在线改 */
#define STEER_SLEW_DEG_S_DEFAULT    (0.0f)
#define STEER_LIMIT_DEG_DEFAULT     (STEER_ABS_LIMIT_DEG_HARD)  /* 目标软限位 度 = 27, 权威值见 steer_abs.h */
#define STEER_LIMIT_DEG_HARD        (STEER_ABS_LIMIT_DEG_HARD)  /* 硬上限, 参数怎么改都越不过 */

/* ---- 外环航向 ----
 * 兜底增益 度/度, 约等于 2L/ld 在 ld=0.6 m 时的值; 回放时由路线层每拍按 2L/ld 覆盖。
 * 符号: delta = -kp*(yaw_tgt - yaw), yaw 正 = 左转、delta 正 = 右转, 故取负。 */
#define STEER_HEAD_KP_DEFAULT       (2.20f)     /* 航向环 兜底增益 度/度 */

/* 控制周期 秒, 必须与调用节拍一致(转向环跑在 5ms 中断里) */
#define STEER_T_S                   (0.005f)

/* 输出符号: 目标转角为正(向右)时电机的 duty 方向; 本工程取 +1, 方向交给 参数表 minv_s 在线调 */
#define STEER_LOOP_SIGN             (+1)

/* 转向电机在 motor_drv 里的 通道号 */
#define MOTOR_CH_STEER              (2)

/* ---- 主动保护 ----
 * 转向无机械回中, 位置环符号给反会变成 正反馈 一路顶到硬限位堵转, 故设两道保护:
 *   ① 反馈失效 STEER_ENC_*: 锁存 + 输出钉 0, 须 angle_enable 走一次 0->1 才清;
 *      判据用指令侧 |输出| + 角度读数连续落在 全 0 / 全 1 特征值
 *   ② 顶死/无进展 STEER_SAT_*: 只置 告警位, 不钉输出; 要求输出顶到限幅且 |err| 无进展 */
#define STEER_TICK_MS               (5u)        /* 与 steer_ctrl_update() 的 调用节拍 一致 */
#define STEER_ENC_INVALID_LOW       (3)         /* 原始读数 <= 它 视为 无效(掉线/悬空) */
#define STEER_ENC_INVALID_HIGH      (4092)      /* 原始读数 >= 它 视为 无效 */
#define STEER_ENC_LOST_OUT          (800.0f)    /* |输出| 超过它才算 确实在出力 */
#define STEER_ENC_LOST_MS           (200u)      /* 累计多久 坏样 就锁存 */

/* 自动复位 默认关: 0 = 锁存后只能靠 steer_en 走 0->1 人工清; 下面三项当前 不参与 */
#define STEER_ENC_AR_ENABLE         (0)
#define STEER_ENC_AR_MS             (100u)      /* 链路连续干净这么久才复位(20 拍) */
#define STEER_ENC_AR_MAX            (3u)        /* 一个窗口内最多自动复位次数 */
#define STEER_ENC_AR_WIN_MS         (3000u)     /* 窗口长度 */

#define STEER_SAT_OUT_RATIO         (0.95f)     /* 输出 >= u_max 的这个比例即算 顶到限幅 */
#define STEER_SAT_ERR_DEG           (3.0f)      /* 顶到限幅时误差还大于它才观察 有无进展 */
#define STEER_SAT_CHK_MS            (100u)      /* 进展检查 窗口, 每这么久比一次 |err| */
#define STEER_SAT_PROGRESS_DEG      (0.5f)      /* 一个窗口内 |err| 至少缩小这么多才算 有进展 */
#define STEER_SAT_TRIP_MS           (600u)      /* 连续 无进展 多久 => 置告警 */
#define STEER_SAT_WARN_HOLD_MS      (2000u)     /* 告警 粘滞 时间, 条件消失后再保持这么久 */

/* steer_ctrl_fault() 返回位: bit0 会钉输出, bit1/bit2 只报告 */
#define STEER_FLT_ENC               (0x01u)     /* 锁存故障: 反馈失效, 输出钉 0 */
#define STEER_FLT_SAT               (0x02u)     /* 告警: 输出顶死/0.6s 无进展, 不影响输出 */
#define STEER_FLT_LINK              (0x04u)     /* 瞬态: 本拍读数被防抖拒掉, 不锁存不钉输出 */

/* IMU 航向 度(-180~180); 本工程无 IMU, 默认 0 且外环 关闭 */
extern volatile float steer_yaw_deg;

/* 上位机「转向|内环使能」写这个变量; 由 steer_ctrl_update() 每拍按边沿同步到 angle_enable,
   以保留 0->1 那一拍 目标 = 当前实测角 的初始化保护 */
extern volatile float steer_en_cmd;

typedef struct
{
    /* 可调参数(参数表/Flash 直接写这些字段) */
    float   kp;                 /* 比例增益 duty/度 */
    float   u0;                 /* 静摩擦前馈 duty */
    float   u_max;              /* 输出限幅 duty */
    float   deadband_deg;       /* 误差死区 度 */
    float   slew_deg_s;         /* 目标变化率上限 度/秒, 0 = 关 */
    float   limit_deg;          /* 目标软限位 度 */
    float   ki;                 /* 积分增益 duty/(度·拍), 0 = 关 */
    float   i_max;              /* 积分项 上限 duty, 累加值上限 = i_max/ki */
    float   brk_deg;            /* 前馈开门 门限 度(三态滞环上门限) */
    float   izone_deg;          /* 积分清零 门限 度 */
    float   i_db_deg;           /* 积分累加下界 度 */

    /* 外环(航向) */
    float   head_kp;

    /* 运行状态 */
    uint8   angle_enable;       /* 内环使能, 1 = 驱动转向电机 */
    uint8   head_enable;        /* 外环使能, 1 = 目标转角由航向环给 */
    uint8   was_enabled;        /* 上一拍 是否使能(失能时只写一次 0) */

    float   target_delta;       /* 命令目标转角 度(向右为正), 上位机/遥控写这个 */
    float   applied_delta;      /* 实际施加的目标 度 = 软限位夹紧 + 可选斜坡, 遥测发它 */
    float   meas_delta;         /* 实测转角 度 */
    float   err;                /* 本拍误差 度, 恒为真值(越限保护除外) */
    float   u_ff;               /* 前馈分量 duty = u0*u_dir */
    int8    u_dir;              /* 三态 -1/0/+1: 0 = 不给, ±1 = 给满 u0, 符号取 sign(err) */
    float   i_sum;              /* 积分累加值 度·拍; 死区内冻结不清, 即保持力矩的记忆 */
    float   ki_prev;            /* 上一拍的 ki, 供边沿检测做无扰切换, 须与 ki 同步初值 */
    int16   output_duty;        /* 本拍输出 -10000~10000 */

    float   target_yaw;         /* 目标航向 度 */
    float   meas_yaw;           /* 实测航向 度 */

    /* 保护状态 */
    uint8   fault;              /* 锁存故障位 STEER_FLT_ENC, 非 0 时输出钉 0 */
    uint32  enc_bad_ms;         /* 坏样累计时间, 坏样 +5ms / 好样清零 */

    /* 自动复位状态(当前未启用) */
    uint32  ar_ms;              /* 连续干净累计时间 */
    uint32  ar_win_ms;          /* 窗口剩余时间, 到 0 才允许 ar_tries 归零 */
    uint8   ar_tries;           /* 本 窗口内已自动复位次数 */
    uint32  recover_cnt;        /* 累计自动复位次数, 只增 */
    uint32  sat_ms;             /* 顶到限幅且无进展的累计时间 */
    uint32  sat_chk_ms;         /* 进展检查计时 */
    float   err_ref;            /* 上次检查窗口的 |err| 基准 */
    uint8   sat_warn;           /* 告警位, 顶死/无进展, 粘滞 SAT_WARN_HOLD_MS */
    uint32  sat_warn_ms;        /* 告警 粘滞剩余时间 */

    /* 遥控急停(CH5): 直接失能 + 钉 0 */
    uint8   estop;              /* 1 = 急停生效, 不自动恢复 */
} steer_ctrl_t;

extern steer_ctrl_t steer;

void    steer_ctrl_init         (void);
void    steer_ctrl_update       (void);         /* 5ms 一拍, 必须排在 steer_abs_update() 之后 */

void    steer_ctrl_set_angle_enable (uint8 en);
void    steer_ctrl_set_head_enable  (uint8 en);
void    steer_ctrl_set_target_delta (float deg);
void    steer_ctrl_set_target_yaw   (float yaw);
void    steer_ctrl_set_head_kp      (float kp);     /* 航向环增益, Pure Pursuit 每拍按 2L/ld 设 */

void    steer_ctrl_set_angle_pid    (float kp);
void    steer_ctrl_set_angle_i      (float ki, float i_max);   /* 改 ki/i_max 并按 I 守恒 重标 i_sum; 0,0 = 关积分 */
void    steer_ctrl_set_u0           (float u0);
void    steer_ctrl_set_angle_outmax (float outmax);

/* 门限关系钳位, 保证 0 < i_db < izone 且 brk >= deadband; 只在 参数下发 / Flash 加载后 / 上电初值调用 */
void    steer_ctrl_sanitize_thr     (void);

float   steer_ctrl_get_target_delta (void);
float   steer_ctrl_get_meas_delta   (void);
int16   steer_ctrl_get_output       (void);
float   steer_ctrl_get_target_yaw   (void);

/* 保护状态位, 车端会 <<1 拼进 steer_status, 顺序别改:
 *   bit0 = 锁存故障 STEER_FLT_ENC, 输出已钉 0, 只能由 angle_enable 0->1 清除
 *   bit1 = 只报警 STEER_FLT_SAT, 不影响输出, 条件消失后自清(粘滞 2 s)
 *   bit2 = 瞬态 STEER_FLT_LINK, 本拍读数被防抖拒掉 */
uint8   steer_ctrl_fault            (void);
void    steer_ctrl_clear_fault      (void);

/* 实际生效限位 度 = min(steer.limit_deg, 27); 任何往目标写角度的代码都要用它夹 */
float   steer_ctrl_eff_limit        (void);

/* 遥控急停(CH5): 全局生效, 内环强制失能 + 输出钉 0, 撤销后须重新使能才会动 */
void    steer_ctrl_set_estop        (uint8 on);
uint8   steer_ctrl_estop            (void);

#endif
