#ifndef _STEER_CTRL_H
#define _STEER_CTRL_H

#include "zf_common_headfile.h"
#include "steer_abs.h"

/*---------------------------------------------------------------------------------------------------------------------
 * 转向控制 —— 位置环(P + I + 静摩擦前馈 + 起步滞环)
 * 转向执行器为有刷直流电机 + 绝对编码器, 断电不回中, 必须用位置环把转角压在目标角度上。
 * 控制律: u = kp*err + ki*i_sum + u0*sign(err)
 * err 恒为真实误差, 三项各自判断本拍是否参与:
 *   死区 |err| < db(0.15)  -> Kp 项置 0, 积分照常累加
 *   前馈三态滞环           -> |err| > brk(0.25) 给满, <= db 给 0, 带内保持
 *   越限保护               -> 三项全 0
 * 积分双门限: |err| < i_db(0.10) 不累加, 只挡噪声; |err| > izone(5 度) 时 i_sum 清 0;
 *   其间累加, u_i = ki*i_sum 按 i_max(1600) 限幅; 输出 u 最后按 u_max(9000) 限幅。
 * 单位统一为方向盘度, 向右为正; 标定换算见 steer_abs.h。
 * 保护: 目标软限位每拍夹紧; 已超软限位且误差仍指向限位时本拍 err 置 0;
 *   失能时只在使能转失能那一拍写 0, 避免与手动开环输出冲突。
 * applied_delta 为实际施加的目标, 遥测发它而不是 target_delta。
 * 外环(航向环)预留, 默认关闭(无 IMU); 接上 IMU 后写 steer_yaw_deg 并置 head_enable。
 * 上电默认不使能, 需在上位机置转向内环使能。
 ---------------------------------------------------------------------------------------------------------------------*/

/* --- 位置环默认参数 ----
 * 上电 steer_en 默认 0, 方向盘不动; 需在上位机置转向内环使能。 */
#define STEER_KP_DEFAULT            (400.0f)    /* 比例增益 duty/度 */
#define STEER_U0_DEFAULT            (1700.0f)   /* 静摩擦前馈 duty, 起动阈值 1300~1600 */

/* 积分项: 死区内 i_sum 冻结不清, 输出保持 ki*i_sum, 消除静摩擦造成的静止角散布。
 * ki 单位 duty/(度·拍), 5 ms 一拍; ki = 0 时只剩 P + 前馈。
 * i_max 是积分项上限 duty, 累加值上限 = i_max/ki。
 * 约束 i_max + kp*deadband < 破冰力, 否则积分能把舵机顶出死区形成慢爬极限环。 */
#define STEER_KI_DEFAULT            (25.0f)     /* 积分增益 duty/(度·拍); 0 = 关 */
#define STEER_IMAX_DEFAULT          (1600.0f)   /* 积分项上限 duty, 破冰力 1300~1600 */

/* 起步滞环: 前馈三态, 输出在死区边界上不再有 u0 台阶。
 * brk 必须 > deadband, 间隔需大于噪声振幅, 否则慢变目标下前馈逐拍翻转。
 * 死区内的保持属预载, 用于压住间隙与静摩擦, 不足以推动舵机。 */
#define STEER_BRK_DEG_DEFAULT       (0.25f)     /* 前馈开门门限 度 */

/* 积分分离门限: |err| 超过它时 i_sum 直接清 0, 只在小误差区积分, 追赶交给 P + 前馈。 */
#define STEER_IZONE_DEG_DEFAULT     (5.0f)      /* 积分清零门限 度 */

/* 输出限幅: 9000 duty = 90% 占空比, 与 Motor_Set 刻度一致(±10000 = ±100%)。
 * 饱和门槛 |err| = (u_max - u0)/kp, 9000 时约 18 度, 几乎全行程在线性区。
 * 长时间堵转(机械卡死)会发热; 只有反馈失效会钉输出。 */
#define STEER_U_MAX_DEFAULT         (9000.0f)   /* 输出限幅 duty */

/* 误差死区 度: |err| < db 时 Kp 项置 0, 积分与前馈各有独立门限。
 * 三处默认值必须同源: 本宏 / motor_link.h ID21 / 上位机 Defaults.cs。
 * 关系约束见 steer_ctrl_sanitize_thr(): 0 < i_db < izone 且 brk >= db + 0.02。 */
#define STEER_DEADBAND_DEG_DEFAULT  (0.15f)     /* 误差死区 度 */

/* 积分累加下界 度: |err| 小于它不累加, 挡机械抖动与噪声(编码器 1 计数 = 0.02 度)。 */
#define STEER_I_DB_DEG_DEFAULT      (0.10f)     /* 积分累加下界 度 */

/* 目标斜坡: 目标变化率上限, 0 = 关。
 * 本环无积分器, 阶跃不需要防积分饱和; 斜坡只把输出上升沿摊平, 增加纯滞后。
 * 无斜坡时到位附近靠背隙与摩擦自然停住, 顶到软限位更快, 首次上电需复核限位。 */
#define STEER_SLEW_DEG_S_DEFAULT    (0.0f)      /* 目标变化率上限 度/秒, 0 = 关 */
#define STEER_LIMIT_DEG_DEFAULT     (STEER_ABS_LIMIT_DEG_HARD)  /* 目标软限位 度 = 27, 权威值见 steer_abs.h */
#define STEER_LIMIT_DEG_HARD        (STEER_ABS_LIMIT_DEG_HARD)  /* 硬上限 度, 参数无法越过 */

/* --- 外环(航向环) ---
 * head_kp 为兜底值, 正常由 route_pb 每拍按 2L/ld 覆盖, ld 随速度变。
 * 符号: delta = -kp*(yaw_tgt - yaw); yaw 正 = 左转, delta 正 = 右转, 必须取负。 */
#define STEER_HEAD_KP_DEFAULT       (2.20f)     /* 度/度, 约 2L/ld 在 ld = 0.6 m 时的值 */

/* 控制周期 秒, 必须与调用节拍一致(转向环跑在 5 ms 中断) */
#define STEER_T_S                   (0.005f)

/* 输出符号: 目标转角为正(向右)时转向电机的 duty 方向。
 * 取 +1 时方向统一由参数表 minv_s 在线调整, 角度极性由 STEER_ABS_DIR_SIGN 负责。 */
#define STEER_LOOP_SIGN             (+1)

/* 转向电机在 motor_drv 中的通道号 */
#define MOTOR_CH_STEER              (2)

/*===================================================================================================================
 * 转向主动保护
 * 转向为有刷直流电机 + 减速箱 + 机械限位, 无机械回中; 位置环符号给反即成正反馈, 会顶到限位堵转。
 * 两道保护, 只有反馈失效会失能, 顶死 / 无进展只报警。
 * ① 反馈失效 STEER_ENC_*: 锁存并钉输出 0, 需 angle_enable 走一次 0->1 才清;
 *    判据用指令侧 |输出|, 出力状态下角度读数连续 STEER_ENC_LOST_MS 落在全 0 / 全 1 特征值。
 * ② 顶死 / 无进展 STEER_SAT_*: 只置告警位, 不钉输出; 判据为输出 >= u_max 的
 *    STEER_SAT_OUT_RATIO 且 |err| > STEER_SAT_ERR_DEG, 连续 STEER_SAT_TRIP_MS 内
 *    |err| 缩小不足 STEER_SAT_PROGRESS_DEG; 告警粘滞 STEER_SAT_WARN_HOLD_MS, 见 steer_status bit2。
 * ===================================================================================================================*/
#define STEER_TICK_MS               (5u)        /* 与 steer_ctrl_update() 调用节拍一致 */
#define STEER_ENC_INVALID_LOW       (3)         /* 原始读数 <= 它视为无效(掉线/悬空) */
#define STEER_ENC_INVALID_HIGH      (4092)      /* 原始读数 >= 它视为无效 */
#define STEER_ENC_LOST_OUT          (800.0f)    /* |输出| 超过它才算在出力 */
#define STEER_ENC_LOST_MS           (200u)      /* 坏样累计时间超它则锁存(漏水积分, 见 .c 的 enc_check) */

/* 反馈失效的有界自动复位, 已停用: 自动复位会放大锁存与复活之间的极限环。
 * 现行为锁存后不自动复活, 只能靠 steer_en 走一次 0->1 人工清除。
 * 以下宏仅作判读保留, steer_ctrl_enc_check() 不再使用。 */
#define STEER_ENC_AR_ENABLE         (0)         /* 0 = 锁存后只能人工清 */
#define STEER_ENC_AR_MS             (100u)      /* 链路连续干净这么久才复位(20 拍) */
#define STEER_ENC_AR_MAX            (3u)        /* 一个窗口内最多自动复位次数 */
#define STEER_ENC_AR_WIN_MS         (3000u)     /* 复位窗口长度 ms */

#define STEER_SAT_OUT_RATIO         (0.95f)     /* 输出 >= u_max 的此比例即算顶到限幅 */
#define STEER_SAT_ERR_DEG           (3.0f)      /* 顶到限幅时 |err| 大于它才进入观察 */
#define STEER_SAT_CHK_MS            (100u)      /* 进展检查窗口 ms, 每窗口比一次 |err| */
#define STEER_SAT_PROGRESS_DEG      (0.5f)      /* 一个窗口内 |err| 至少缩小这么多算有进展 */
#define STEER_SAT_TRIP_MS           (600u)      /* 连续无进展多久置告警 */
#define STEER_SAT_WARN_HOLD_MS      (2000u)     /* 告警粘滞时间 ms, 条件消失后再保持 */

/* steer_ctrl_fault() 返回位, 供遥测/打印; bit0 会钉输出, bit1/bit2 只报告 */
#define STEER_FLT_ENC               (0x01u)     /* 锁存故障: 反馈失效(读数坏/丢样), 输出钉 0 */
#define STEER_FLT_SAT               (0x02u)     /* 告警不钉输出: 输出顶死/无进展 0.6 s 不收敛 */
#define STEER_FLT_LINK              (0x04u)     /* 瞬态不锁存不钉输出: 本拍读数被防抖拒掉,
                                                   先于 STEER_FLT_ENC 出现, 提示电机干扰编码器 */

/* IMU 航向 度(-180~180)。本工程无 IMU, 默认 0 且外环关闭; 接入后由 IMU 模块每拍写入。 */
extern volatile float steer_yaw_deg;

/* 上位机转向内环使能写入此变量, 由 steer_ctrl_update() 每拍按边沿同步到 angle_enable。
 * 不直接指向 angle_enable: 使能上升沿需用当前实测转角初始化目标, 否则一使能就是大阶跃。 */
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
    float   ki;                 /* 积分增益 duty/(度·拍); 0 = 关 */
    float   i_max;              /* 积分项上限 duty, 累加值上限 = i_max/ki */
    float   brk_deg;            /* 已停用, 不参与控制 */
    float   izone_deg;          /* 积分清零门限 度, |err| 超过它 i_sum = 0 */
    float   i_db_deg;           /* 积分累加下界 度, |err| 小于它不累加 */

    /* --- 外环(航向) --- */
    float   head_kp;

    /* --- 运行状态 --- */
    uint8   angle_enable;       /* 内环使能(=1 才驱动转向电机) */
    uint8   head_enable;        /* 外环使能(=1 时目标转角由航向环给) */
    uint8   was_enabled;        /* 上一拍是否使能(失能时只写一次 0) */

    float   target_delta;       /* 内环目标转角 度, 向右为正; 上位机/遥控写入 */
    float   applied_delta;      /* 实际施加的目标 度 = 软限位夹紧 + 可选斜坡; 遥测发它,
                                   slew = 0 时与 target_delta 相等 */
    float   meas_delta;         /* 内环实测转角 度 */
    float   err;                /* 本拍误差 度; 死区不清零, 越限保护会清零 */
    float   u_ff;               /* 本拍静摩擦前馈 duty = u0 * u_dir, 带滞环 */
    int8    u_dir;              /* 前馈开关兼方向 -1/0/+1; 符号恒等于 sign(err), 滞环只决定给不给 */
    float   i_sum;              /* 积分累加值 度·拍; 死区内冻结不清, 即保持力矩的记忆 */
    float   ki_prev;            /* 上一拍 ki, 用于边沿检测并按 I 守恒重标 i_sum; 必须与 ki 同步初值 */
    int16   output_duty;        /* 本拍转向电机输出(-10000~10000) */

    float   target_yaw;         /* 外环目标航向 度 */
    float   meas_yaw;           /* 外环实测航向 度 */

    /* --- 保护状态 --- */
    uint8   fault;              /* 锁存故障位 STEER_FLT_ENC, 置位后输出钉 0; 0 = 正常 */
    uint32  enc_bad_ms;         /* 坏样累计时间 ms(漏水积分: 坏样 +5, 好样 -5) */

    /* --- 反馈失效自动复位, 已停用 --- */
    uint32  ar_ms;              /* 链路连续干净的累计时间 */
    uint32  ar_win_ms;          /* 窗口剩余时间, 到 0 才允许 ar_tries 归零 */
    uint8   ar_tries;           /* 本窗口内已自动复位次数 */
    uint32  recover_cnt;        /* 累计自动复位次数, 只增 */
    uint32  sat_ms;             /* 输出顶到限幅且误差无进展的累计时间 */
    uint32  sat_chk_ms;         /* 进展检查计时, 每 STEER_SAT_CHK_MS 比一次 |err| */
    float   err_ref;            /* 上次检查窗口的 |err| 基准 */
    uint8   sat_warn;           /* 告警位, 不钉输出; 顶死/无进展, 粘滞 STEER_SAT_WARN_HOLD_MS */
    uint32  sat_warn_ms;        /* 告警粘滞剩余时间 */

    /* --- 遥控急停(CH5): 直接失能并钉 0 --- */
    uint8   estop;              /* 1 = 急停生效, 不自动恢复, 撤销 estop 才放开 */
} steer_ctrl_t;

extern steer_ctrl_t steer;

void    steer_ctrl_init         (void);
void    steer_ctrl_update       (void);         /* 5 ms 每拍, 必须排在 steer_abs_update() 之后 */

void    steer_ctrl_set_angle_enable (uint8 en);
void    steer_ctrl_set_head_enable  (uint8 en);
void    steer_ctrl_set_target_delta (float deg);
void    steer_ctrl_set_target_yaw   (float yaw);
void    steer_ctrl_set_head_kp      (float kp);     /* 航向环增益, 由 Pure Pursuit 每拍按 2L/ld 设置 */

void    steer_ctrl_set_angle_pid    (float kp);
void    steer_ctrl_set_angle_i      (float ki, float i_max);   /* 改 ki/i_max, 按 I 守恒重标 i_sum; 0,0 = 关积分并清零 */
void    steer_ctrl_set_u0           (float u0);
void    steer_ctrl_set_angle_outmax (float outmax);

/* 门限关系钳位: 保证 0 < db < brk < izone, 滞环带与积分区均非空。
 * 只在参数下发、Flash 加载后、上电初值调用, 不在控制拍内调用。 */
void    steer_ctrl_sanitize_thr     (void);

float   steer_ctrl_get_target_delta (void);
float   steer_ctrl_get_meas_delta   (void);
int16   steer_ctrl_get_output       (void);
float   steer_ctrl_get_target_yaw   (void);

/* 保护状态位, 供遥测/打印; 车端左移 1 位拼入 steer_status 的 bit1/bit2, 顺序不可改:
 *   bit0 = STEER_FLT_ENC 锁存故障, 输出已钉 0, 需 angle_enable 走一次 0->1 清除;
 *   bit1 = STEER_FLT_SAT 只报警, 不影响输出, 条件消失后粘滞 2 s 自清。 */
uint8   steer_ctrl_fault            (void);
void    steer_ctrl_clear_fault      (void);

/* 实际生效限位 度 = min(steer.limit_deg, STEER_LIMIT_DEG_HARD), 最多 27。
 * 任何写 target 的代码都需用它夹紧。 */
float   steer_ctrl_eff_limit        (void);

/* 遥控急停(CH5): 内环强制失能并钉输出 0, 对遥控与上位机均生效; 撤销后需重新使能。 */
void    steer_ctrl_set_estop        (uint8 on);
uint8   steer_ctrl_estop            (void);

#endif
