#ifndef _STEER_CTRL_H
#define _STEER_CTRL_H

#include "zf_common_headfile.h"
#include "steer_abs.h"

/*********************************************************************************************************************
 * 转向控制 —— 位置环(纯 P + 静摩擦前馈)
 *
 * 硬件事实(与队友一致): 转向是【有刷直流电机 + 绝对编码器】, 不是会自动回中的 RC 舵机。
 *   → 有刷电机断电不回中, 必须靠"转角位置环"把它按在目标角度上。**这是刚需, 不是可选项。**
 *
 * ★★ 控制律(2026-10-07 换, 来自队友 YINGYYUN/Karting-Project 的 Servo_Crtl):
 *
 *        u = kp * err  +  u0 * sign(err)
 *        ‾‾‾‾‾‾‾‾‾‾‾   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *          细调(比例)     克服静摩擦与减速箱阻力(前馈)
 *
 *   err 先过【死区】(±0.2 度), 死区内 err=0 ⇒ u0 也不加 ⇒ 静止不抖。
 *
 *   为什么不是 PI: 本工程旧版用 Kp/Ki 的 PI 去"顶"静摩擦(靠积分爬到启动阈值)。
 *   但直流电机 + 减速箱的静摩擦是 Coulomb 型的【常数非线性】, 让积分去逼近它有两个副作用:
 *     ① 启动延迟 —— 积分要爬好几拍才够力, 方向盘"先愣一下再动";
 *     ② 一旦动起来, 静摩擦突降到动摩擦, 那一堆积分值全变成过冲, 要靠反向积分慢慢退。
 *   前馈直接把这部分出力按符号给出去, 积分就不需要了 ⇒ 结构更简单、响应更干脆。
 *   (北极星队那台用的是 PI; 队友这台实测改成 P+前馈后跑通 —— 本工程跟队友。)
 *
 * ★ 单位统一成【方向盘度】(向右为正), 标定值见 steer_abs.h。
 *   旧版用的是"编码器计数"(4096 计数 = 编码器轴一圈), 与本车机构的几何对不上;
 *   换成度以后, 参数含义直观, 也能直接对上队友标出来的数。
 *
 * ★ 其它几件"照着队友抄"的保护:
 *   ① 目标软限位 (LIMIT_DEG): 每拍夹一次, 别让控制器主动把方向盘顶向机械硬限位;
 *   ② 越限保护: 实测已超软限位、且误差还指向限位方向 ⇒ 本拍 err 置 0, 不再加力;
 *   ③ 失能时【只在"使能->失能"那一拍写 0】, 之后不再每拍写 —— 否则会和手动开环输出互相打架。
 *   ⚠ 原来还有一件"目标斜坡 (SLEW_DEG_S)"，**2026-10-14 照队友实测改成默认关** —— 见下面
 *     STEER_SLEW_DEG_S_DEFAULT 的注解（纯 P + 前馈不需要它，它只贡献纯滞后）。
 *   ⭐ 另外照队友把"**实际施加的目标**"单独导出去：`applied_delta`（≠ 上位机写的 `target_delta`），
 *     遥测发的是它。理由与增益回读完全一样 —— **别让图上的名字骗人**：
 *     斜坡开着时若只发"上位机写的目标"，波形上是一条干净的阶跃，
 *     你会把"斜坡造成的滞后"算到"机械/增益"头上。
 *
 * ★ 外环(航向环 head): 预留, 本工程默认关(没有 IMU)。接上 IMU 后把航向写进 steer_yaw_deg、
 *   再把 steer_ctrl_set_head_enable(1) 即可, 不用改结构。
 *
 * ❗ 增益来源(2026-10-07 更新): 下面这组 Kp/U0 是【队友在实车上验证过的值】
 *   (project/code/Param_Storage.c 的 DEFAULT_PARAMS 末尾两项: 300.0f, 1600.0f)。
 *   两边现在是**同一套硬件** —— 他把后轮正交编码器也改成了脉冲+方向、并采用了我们的引脚,
 *   所以这组值可以直接用, 不必从 0 开始试。
 *   ⚠ 仍建议首次上电按 02-转向标定卡.md 的 §6 复核一遍(先只给 U0 看能不能推得动, 再加 Kp)。
 ********************************************************************************************************************/

/* --- 位置环默认参数 ----
 * ★ 上电仍然【全不使能】(steer_en 默认 0) ⇒ 方向盘不会自己动;
 *   要动必须先在上位机把「转向|内环使能」打 1。 */
#define STEER_KP_DEFAULT            (300.0f)    /* 比例增益   单位 duty/度   (队友实车验证) */
#define STEER_U0_DEFAULT            (1600.0f)   /* 静摩擦前馈 单位 duty      (队友实车验证) */
/* ★★ 输出限幅：2026-10-15 由 3500(35%) 提到 **9000(90%)**（用户拍板）。
 *
 *  为什么原来卡在 3500：早期为了"转向极性给反 → 全速顶限位 → 堵转烧 DRV8701"留安全余量。
 *  现在前提已经变了：
 *    ① 极性已按工程文档 §4.2 台架验证过，且有"反馈失效锁存"（唯一的钉输出故障）兜底；
 *    ② 目标被 ±27° 软限位夹死 + 越限保护（实测超限且误差仍指向限位 ⇒ err 置 0）⇒ 不会持续堵转；
 *    ③ 目标斜坡已默认关 ⇒ 大角度阶跃本来就要求"满力最快到位"，而 35% 限幅一直在压它的到位速度。
 *
 *  ⚠⚠ **饱和门槛跟着变，这才是这条改动的真正影响**（`Kp = 300`、`u0 = 1600`）：
 *
 *        档位              饱和所需 |err|              含义
 *        3500（旧）   (3500−1600)/300 ≈ **6.3°**   超过 6.3° 就恒出 35% —— 大阶跃一半路程被限幅压着
 *        9000（新）   (9000−1600)/300 ≈ **24.7°**  几乎全行程都在线性区，只有顶到 27° 附近才饱和
 *
 *      ⇒ 到位速度由"限幅"改由 **Kp 本身**决定（`u = 300·err`：err = 12° 就是 52% duty）。
 *      ⇒ 也就是**从"限速巡航"变成"全程按误差比例给力"**：前段明显更快，也明显更冲。
 *
 *  ⚠ 副作用两条，心里有数：
 *    ① 电流 / 发热：90% duty 是有刷电机 + DRV8701 能给的极限档，长时间堵转（机械卡死）会烫。
 *       —— 只有"反馈失效"会钉输出；"顶死/无进展"**只告警不动作**（见下面 STEER_SAT_*）。
 *    ② `STEER_SAT_*` 告警里"顶到限幅"的判据是**相对 u_max 的 95%**（= 8550）⇒ 需要 |err| ≥ 23.2°
 *       才算，比 3500 时代（|err| ≥ 5.8°）难触发得多。抓"极性给反 / 卡死"仍然够用
 *       （那两种的误差会一路涨到软限位 27°），但**它不再是"环在满力顶"的灵敏指示器**。
 *       要更灵敏就把 `STEER_SAT_OUT_RATIO` 换成**绝对 duty 门槛**（例：`output_duty >= 4000`），
 *       别只盯 0.95 这个相对值 —— 它随 u_max 一起漂。
 *
 *  ⚠ **想撤回**：上位机 ID22 `steer_umax` 拖回 3500 即可（在线、不用重烧）；
 *    改默认值则三处一起改：本宏 + `motor_link.h` ID22 默认值 + 上位机 `Defaults.cs`。 */
#define STEER_U_MAX_DEFAULT         (9000.0f)   /* 输出限幅   单位 duty      (与 Motor_Set 刻度一致: ±10000 = ±100%) */
#define STEER_DEADBAND_DEG_DEFAULT  (0.2f)      /* 误差死区   单位 度 */

/* ★★ 目标斜坡：默认【关】(0 = 不限速率)。
 *
 *  2026-10-14 队友(YINGYYUN/Karting-Project)在 commit 4ce6c93
 *  "feat:2026.10.14.22:03:删除了舵机的目标斜坡" 里把斜坡整个删掉，实测结论是：
 *    **没有斜坡反而响应更好，而且没有出现震荡。**
 *
 *  为什么这里有斜坡是多余的（他的原话 + 我们的结构印证）：
 *    斜坡是为了"别把上位机写的阶跃一次性灌进环里"。但本环是 **纯 P + 静摩擦前馈**，
 *    **没有积分器** ⇒ 阶跃不需要"防积分裹死"；而误差本身已被 **±27° 软限位** 卡住，
 *    最大也就 27°，Kp·err = 8100 直接顶到 u_max —— 这正是"最快到位"想要的效果。
 *    加了斜坡只剩一个代价：**误差被一点点喂进来，输出的上升沿被摊平**，纯滞后。
 *
 *  ⚠ 我们这条斜坡比他的更狠：他是 **120 度/秒**（10 ms 拍 ⇒ 1.2 度/拍），
 *    我们是 **40 度/秒**（5 ms 拍 ⇒ 0.2 度/拍）—— 同样 0→27° 的阶跃，
 *    他 225 ms 走完，我们要 **675 ms**。这也是"角度环响应延迟"的一大半。
 *  ⇒ 所以照他的结论把默认改成 0。**滑条还在**（`steer_slew`），想研究斜坡随时拖回去，
 *    不用重烧 —— 但**默认上车就是"没有斜坡"**。
 *
 *  ⚠ 代价要认：没有斜坡就没有"到位前的减速"，到位附近靠背隙/摩擦自然停住；
 *    而且顶到 ±27 软限位会比以前更快更狠 ⇒ **首次上电务必按 02-转向标定卡.md 复核一遍**。 */
#define STEER_SLEW_DEG_S_DEFAULT    (0.0f)      /* 目标变化率上限 单位 度/秒 (0 = 关，照队友实测) */
#define STEER_LIMIT_DEG_DEFAULT     (STEER_ABS_LIMIT_DEG_HARD)  /* 目标软限位 单位 度 = 27（权威值见 steer_abs.h） */
#define STEER_LIMIT_DEG_HARD        (STEER_ABS_LIMIT_DEG_HARD)  /* ★ 硬上限: 参数怎么改都越不过它 */

/* --- 外环(航向环)默认: 预留, 本工程默认关 --- */
#define STEER_HEAD_KP_DEFAULT       (15.0f)     /* 度/度 */

/* 控制周期(秒): 必须与调用它的节拍一致(转向环跑在 5ms 中断里) */
#define STEER_T_S                   (0.005f)

/* 输出符号: 目标转角为正(向右)时, 转向电机该往哪边给 duty。
 * ★ 本工程设 +1: 方向统一交给参数表的 minv_s 在线调(不用重烧)。
 *   ⚠ 要让整条链等效于队友的 SERVO_MOTOR_SIGN = -1, **minv_s 必须默认取 1**:
 *       我们: duty = (+1)·u  → motor_drv_set(ch2) → mot_inv_s=1 时再取反 → 合计 −u
 *       队友: Motor_Set(MOTOR_SERVO, −u)                             → 合计 −u   ✔ 一致
 *   (两边的角度极性都是 STEER_ABS_DIR_SIGN = −1, 所以只需要对齐这一处输出符号。) */
#define STEER_LOOP_SIGN             (+1)

/* 本工程转向电机在 motor_drv 里的通道号 */
#define MOTOR_CH_STEER              (2)

/*===================================================================================================================
 *  ★★ 转向主动保护（2026-10-13 新增，专治"转向极性给反 -> 全速顶限位 -> 堵转烧驱动"）
 *
 *  背景：本车转向是"有刷直流电机 + 减速箱 + 机械限位"，**没有任何机械回中**。
 *        位置环只要符号给反，就不再是负反馈而是正反馈：误差越大越出力，电机一路顶到
 *        机械硬限位堵转，DRV8701 持续满流 -> 发热烧毁。**已经烧掉一个驱动。**
 *        （队友的 Servo_Crtl 也吃过同样的亏，他在 2026-10-08 的 commit 里补了"反馈失效锁存"。）
 *
 *  两道保护 —— ★ **只有"反馈失效"会失能；"顶死/无进展"只报警**：
 *
 *   ① 反馈失效  `STEER_ENC_*`   —— **锁存 + 输出钉 0**（必须 angle_enable 走一次 0->1 才清）
 *      判据完全照队友 Servo_Crtl：**前提用"指令侧"的 |输出|**（反馈坏了就没有角度可读，
 *      不能拿角度当条件），在出力状态下角度读数连续 STEER_ENC_LOST_MS 落在
 *      全 0 / 全 1 特征值 -> 锁存停机。
 *
 *   ② 顶死 / 无进展  `STEER_SAT_*` —— ★ **只置告警位、不钉输出**（2026-10-13 改）
 *      判据：输出顶到 u_max 的 STEER_SAT_OUT_RATIO 以上、|err| 还大于 STEER_SAT_ERR_DEG，
 *      而且**误差完全不再收敛** —— 连续 STEER_SAT_TRIP_MS 内 |err| 连
 *      STEER_SAT_PROGRESS_DEG 都没缩小 -> 置 sat_warn（粘滞 STEER_SAT_WARN_HOLD_MS）。
 *
 *      ★★ 为什么从"锁存失能"降级成"只报警"（用户 2026-10-13 现场要求）：
 *        ±27 是**保护限位**、机械极限还有余量，所以"满转到 27"本身不会堵转、不会烧电机；
 *        而实测这条会在"打到行程末端"时触发 —— 连杆到头那一段机械增益差又涩，方向盘爬得慢，
 *        |err| 每 100ms 缩不到 0.5 度就被判成"无进展" ⇒ **用户正常打到最大角就被失能**，
 *        比不保护还碍事。
 *        ⇒ 失能去掉；保留"告警"是因为它正是今天定位问题的第一手线索（遥测 steer_status bit2）。
 *          "极性给反会烧 DRV8701"这件事改由**台架验证**兜住 —— 见工程文档 §4.2。
 *
 *      ⚠ 别把门槛改成"只要饱和就报警"：任何大角度阶跃都会饱和几百毫秒，
 *        那样告警会变成常亮噪声（合法的 0->25 度阶跃属于这种：|err| 在缩小）。
 *      ⚠ 也别指望它抓"力矩不够、慢吞吞往目标爬"：那是有进展，不触发；
 *        那种情况该改的是 kp/u0，不是保护。
 * ===================================================================================================================*/
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

/* steer_ctrl_fault() 返回的位（供遥测/打印）；★ bit0 会钉输出、bit1 只报警 */
#define STEER_FLT_ENC               (0x01u)     /* ★锁存故障: 反馈失效（读数掉线）-> 输出钉 0 */
#define STEER_FLT_SAT               (0x02u)     /* ★告警(不钉输出): 输出顶死/无进展 0.6s 不收敛 */

/* IMU 航向(度, -180~180)。本工程没有 IMU → 默认 0 且外环关闭;
   将来接上 IMU 后由 IMU 模块每拍写这个变量即可打开外环。 */
extern volatile float steer_yaw_deg;

/* 上位机「转向|内环使能」开关写的是这个变量(不是 angle_enable 本身)。
 * 为什么不让参数表直接指 angle_enable: 那样就绕过了 set_angle_enable() 里
 * "0->1 那一拍用当前实测转角初始化目标"的保护, 一使能就是一次大阶跃(必超调)。
 * 由 steer_ctrl_update() 每拍按边沿同步过去。 */
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
    float   applied_delta;      /* ★ 实际施加的目标(度) = 软限位夹紧 + (可选)斜坡之后。
                                    **这才是环路的真输入，遥测/调试口发的就是它**（见文件顶部注解）。
                                    `steer_slew = 0` 时与 target_delta 相等。 */
    float   meas_delta;         /* 内环实测转角(度) */
    float   err;                /* 本拍误差(度, 已过死区与越限保护) */
    float   u_ff;               /* 本拍静摩擦前馈分量(duty) */
    int16   output_duty;        /* 本拍转向电机输出(-10000~10000) */

    float   target_yaw;         /* 外环目标航向(度) */
    float   meas_yaw;           /* 外环实测航向(度) */

    /* --- 保护状态（见本文件顶部 STEER_ENC_* / STEER_SAT_*）--- */
    uint8   fault;              /* ★锁存故障位: STEER_FLT_ENC（置位后输出被钉 0）, 0 = 正常 */
    uint32  enc_bad_ms;         /* 在出力、且角度读数无效的累计时间 */
    uint32  sat_ms;             /* 输出顶到限幅 + 误差无进展的累计时间 */
    uint32  sat_chk_ms;         /* 进展检查计时（每 STEER_SAT_CHK_MS 比一次 |err|） */
    float   err_ref;            /* 上次检查窗口的 |err| 基准 */
    uint8   sat_warn;           /* ★告警位(不钉输出): 顶死/无进展, 粘滞 STEER_SAT_WARN_HOLD_MS */
    uint32  sat_warn_ms;        /* 告警粘滞剩余时间 */

    /* --- ★ 遥控急停（CH5）：与"谁在开车"无关，直接失能 + 钉 0 --- */
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

/* ★ 保护状态位，供遥测/打印（车端会把它 <<1 拼成 steer_status 的 bit1/bit2，别改顺序）：
     bit0 = STEER_FLT_ENC **锁存故障** —— 输出已钉 0，唯一清除途径 =
            steer_ctrl_set_angle_enable(0) 再 (1)；
     bit1 = STEER_FLT_SAT **只报警** —— 顶死/无进展告警，**不影响输出**，条件消失后自清（粘滞 2 s）。 */
uint8   steer_ctrl_fault            (void);
void    steer_ctrl_clear_fault      (void);

/* ★ 实际生效的限位(度) = min(参数 steer.limit_deg, STEER_LIMIT_DEG_HARD) = 最多 27。
   **任何往 target 里写角度的代码都要用它来夹**（遥控、上位机、以后的外环都一样）。 */
float   steer_ctrl_eff_limit        (void);

/* ★★ 遥控急停（CH5）：内环强制失能 + 输出钉 0。
   ⚠ 是**全局**的：即使此刻是上位机在写 steer_en_cmd / steer_tgt，急停照样把它按停 ——
     急停属于"车要停"，不属于"控制权归谁"。撤销 estop 后需要重新使能才会再动。 */
void    steer_ctrl_set_estop        (uint8 on);
uint8   steer_ctrl_estop            (void);

#endif
