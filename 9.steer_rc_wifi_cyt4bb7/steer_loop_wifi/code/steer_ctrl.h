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
 * ① 目标软限位 (LIMIT_DEG): 每拍夹一次, 别让控制器主动把方向盘顶向机械硬限位;
 * ② 越限保护: 实测已超软限位、且误差还指向限位方向 ⇒ 本拍 err 置 0, 不再加力;
 * ③ 失能时【只在"使能->失能"那一拍写 0】, 之后不再每拍写 —— 否则会和手动开环输出互相打架。
 * STEER_SLEW_DEG_S_DEFAULT 的注解（纯 P + 前馈不需要它，它只贡献纯滞后）。
 * 遥测发的是它。理由与增益回读完全一样 —— **别让图上的名字骗人**：
 * 你会把"斜坡造成的滞后"算到"机械/增益"头上。
 * 再把 steer_ctrl_set_head_enable(1) 即可, 不用改结构。
 * (project/code/Param_Storage.c 的 DEFAULT_PARAMS 末尾两项: 300.0f, 1600.0f)。
 * 两边现在是**同一套硬件** —— 他把后轮正交编码器也改成了脉冲+方向、并采用了我们的引脚,
 * 所以这组值可以直接用, 不必从 0 开始试。
 */

/* --- 位置环默认参数 ---- 要动必须先在上位机把「转向|内环使能」打 1。 */
#define STEER_KP_DEFAULT            (300.0f)
#define STEER_U0_DEFAULT            (1600.0f)
/* 现在前提已经变了：
 * ① 极性已按工程文档 §4.2 台架验证过，且有"反馈失效锁存"（唯一的钉输出故障）兜底；
 * ② 目标被 ±27° 软限位夹死 + 越限保护（实测超限且误差仍指向限位 ⇒ err 置 0）⇒ 不会持续堵转；
 * 档位              饱和所需 |err|              含义
 * 9000（新）   (9000−1600)/300 ≈ **24.7°**  几乎全行程都在线性区，只有顶到 27° 附近才饱和
 * ⇒ 到位速度由"限幅"改由 **Kp 本身**决定（`u = 300·err`：err = 12° 就是 52% duty）。
 * ⇒ 也就是**从"限速巡航"变成"全程按误差比例给力"**：前段明显更快，也明显更冲。
 * ① 电流 / 发热：90% duty 是有刷电机 + DRV8701 能给的极限档，长时间堵转（机械卡死）会烫。
 * —— 只有"反馈失效"会钉输出；"顶死/无进展"**只告警不动作**（见下面 STEER_SAT_*）。
 * ② `STEER_SAT_*` 告警里"顶到限幅"的判据是**相对 u_max 的 95%**（= 8550）⇒ 需要 |err| ≥ 23.2°
 * 才算，比 3500 时代（|err| ≥ 5.8°）难触发得多。抓"极性给反 / 卡死"仍然够用
 * （那两种的误差会一路涨到软限位 27°），但**它不再是"环在满力顶"的灵敏指示器**。
 * 要更灵敏就把 `STEER_SAT_OUT_RATIO` 换成**绝对 duty 门槛**（例：`output_duty >= 4000`），
 * 别只盯 0.95 这个相对值 —— 它随 u_max 一起漂。
 * 改默认值则三处一起改：本宏 + `motor_link.h` ID22 默认值 + 上位机 `Defaults.cs`。
 */
#define STEER_U_MAX_DEFAULT         (9000.0f)   /* 输出限幅   单位 duty      (与 Motor_Set 刻度一致: ±10000 = ±100%) */
#define STEER_DEADBAND_DEG_DEFAULT  (0.2f)      /* 误差死区   单位 度 */

/* 目标斜坡默认 0（关闭）: 无斜坡响应更快且无震荡; 参数 steer_slew 可在线调回。
 * 首次上电按 02-转向标定卡.md 复核。 */
#define STEER_SLEW_DEG_S_DEFAULT    (0.0f)
#define STEER_LIMIT_DEG_DEFAULT     (STEER_ABS_LIMIT_DEG_HARD)  /* 目标软限位 单位 度 = 27（权威值见 steer_abs.h） */
#define STEER_LIMIT_DEG_HARD        (STEER_ABS_LIMIT_DEG_HARD)

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

/* 背景：本车转向是"有刷直流电机 + 减速箱 + 机械限位"，**没有任何机械回中**。
 * 位置环只要符号给反，就不再是负反馈而是正反馈：误差越大越出力，电机一路顶到
 * ① 反馈失效  `STEER_ENC_*`   —— **锁存 + 输出钉 0**（必须 angle_enable 走一次 0->1 才清）
 * 不能拿角度当条件），在出力状态下角度读数连续 STEER_ENC_LOST_MS 落在
 * 全 0 / 全 1 特征值 -> 锁存停机。
 * 判据：输出顶到 u_max 的 STEER_SAT_OUT_RATIO 以上、|err| 还大于 STEER_SAT_ERR_DEG，
 * 而且**误差完全不再收敛** —— 连续 STEER_SAT_TRIP_MS 内 |err| 连
 * STEER_SAT_PROGRESS_DEG 都没缩小 -> 置 sat_warn（粘滞 STEER_SAT_WARN_HOLD_MS）。
 * ±27 是**保护限位**、机械极限还有余量，所以"满转到 27"本身不会堵转、不会烧电机；
 * 而实测这条会在"打到行程末端"时触发 —— 连杆到头那一段机械增益差又涩，方向盘爬得慢，
 * |err| 每 100ms 缩不到 0.5 度就被判成"无进展" ⇒ **用户正常打到最大角就被失能**，
 * 比不保护还碍事。
 * ⇒ 失能去掉；保留"告警"是因为它正是今天定位问题的第一手线索（遥测 steer_status bit2）。
 * "极性给反会烧 DRV8701"这件事改由**台架验证**兜住 —— 见工程文档 §4.2。
 * 那种情况该改的是 kp/u0，不是保护。
 */
#define STEER_TICK_MS               (5u)        /* 与 steer_ctrl_update() 的调用节拍一致 */
#define STEER_ENC_INVALID_LOW       (3)         /* 原始读数 <= 它 视为无效（掉线/悬空） */
#define STEER_ENC_INVALID_HIGH      (4092)      /* 原始读数 >= 它 视为无效 */
#define STEER_ENC_LOST_OUT          (800.0f)    /* |输出| 超过它才算"确实在出力" */
#define STEER_ENC_LOST_MS           (200u)      /* 连续多久读到无效值就锁存 */
#define STEER_SAT_OUT_RATIO         (0.95f)     /* 输出 >= u_max 的这个比例 => 算"顶到限幅" */
#define STEER_SAT_ERR_DEG           (3.0f)      /* 顶到限幅时误差还大于它 => 观察期内看有没有进展 */
#define STEER_SAT_CHK_MS            (100u)      /* 进展检查窗口: 每这么久比一次 |err| */
#define STEER_SAT_PROGRESS_DEG      (0.5f)      /* 一个窗口内 |err| 至少缩小这么多才算"有进展" */
#define STEER_SAT_TRIP_MS           (600u)      /* 连续"无进展"多久 => 锁存停机 */
#define STEER_SAT_WARN_HOLD_MS      (2000u)     /* 告警粘滞时间: 条件消失后再保持这么久(遥测抓得住) */

#define STEER_FLT_ENC               (0x01u)
#define STEER_FLT_SAT               (0x02u)

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

    float   target_delta;       /* 内环目标转角(度, 向右为正; 上位机/遥控写这个 = **命令**) */
    float   applied_delta;      /* 这才是环路的真输入，遥测/调试口发的就是它**（见文件顶部注解）。 `steer_slew = 0` 时与 target_delta 相等。 */
    float   meas_delta;         /* 内环实测转角(度) */
    float   err;                /* 本拍误差(度, 已过死区与越限保护) */
    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */
    int16   output_duty;        /* 本拍转向电机输出(-10000~10000) */

    float   target_yaw;         /* 外环目标航向(度) */
    float   meas_yaw;           /* 外环实测航向(度) */

    /* --- 保护状态（见本文件顶部 STEER_ENC_* / STEER_SAT_*）--- */
    uint8   fault;
    uint32  enc_bad_ms;         /* 在出力、且角度读数无效的累计时间 */
    uint32  sat_ms;             /* 输出顶到限幅 + 误差无进展的累计时间 */
    uint32  sat_chk_ms;         /* 进展检查计时（每 STEER_SAT_CHK_MS 比一次 |err|） */
    float   err_ref;            /* 上次检查窗口的 |err| 基准 */
    uint8   sat_warn;
    uint32  sat_warn_ms;        /* 告警粘滞剩余时间 */

    uint8   estop;              /* 1 = 急停生效（不自动恢复，只有 estop 撤销才放开） */
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

/* bit0 = STEER_FLT_ENC **锁存故障** —— 输出已钉 0，唯一清除途径 =
 * steer_ctrl_set_angle_enable(0) 再 (1)；
 * bit1 = STEER_FLT_SAT **只报警** —— 顶死/无进展告警，**不影响输出**，条件消失后自清（粘滞 2 s）。
 */
uint8   steer_ctrl_fault            (void);
void    steer_ctrl_clear_fault      (void);

/* 任何往 target 里写角度的代码都要用它来夹**（遥控、上位机、以后的外环都一样）。 */
float   steer_ctrl_eff_limit        (void);

/* 急停属于"车要停"，不属于"控制权归谁"。撤销 estop 后需要重新使能才会再动。 */
void    steer_ctrl_set_estop        (uint8 on);
uint8   steer_ctrl_estop            (void);

#endif
