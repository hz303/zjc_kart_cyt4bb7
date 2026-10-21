/* 数据流(5ms 一拍, 紧跟 steer_abs_update() 之后):
 * [外环开] 目标航向 − steer_yaw_deg (wrap±180) → head_kp → 目标转角(夹软限位)
 * [内环]   目标转角 →(斜坡)→ 手夹软限位 → 误差 → 越限保护 → 死区 → kp*err + u0*sign(err) → 限幅 → duty
 */

#include "zf_common_headfile.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "motor_drv.h"

volatile float  steer_yaw_deg = 0.0f;
steer_ctrl_t    steer = {0};
volatile float  steer_en_cmd = 0.0f;

/* 航向误差归一到 −180~180, 保证走最近的方向纠偏(别绕远路) */
static float steer_wrap180 (float err)
{
    while (err >  180.0f) { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }
    return err;
}

/* 参数 `steer.limit_deg` 是"软限位"滑条，但**无论上位机把它拖到多少，都越不过
 * STEER_LIMIT_DEG_HARD(=27，权威值在 steer_abs.h)**。
 * 遥控请求、上位机 s/p 指令、目标斜坡、越限保护，全部走这个函数 ⇒ 限位只有一处。
 */
float steer_ctrl_eff_limit (void)
{
    float lim = steer.limit_deg;

    if (lim > STEER_LIMIT_DEG_HARD) { lim = STEER_LIMIT_DEG_HARD; }
    if (lim < 0.0f)                 { lim = 0.0f; }

    return lim;
}

/* 目标转角软限幅: 夹在**实际生效限位**内, 别顶机械硬限位 */
static float steer_clamp_deg (float deg)
{
    float lim = steer_ctrl_eff_limit();

    if (deg >  lim) { return  lim; }
    if (deg < -lim) { return -lim; }
    return deg;
}

void steer_ctrl_init (void)
{
    steer.kp           = STEER_KP_DEFAULT;
    steer.u0           = STEER_U0_DEFAULT;
    steer.u_max        = STEER_U_MAX_DEFAULT;
    steer.deadband_deg = STEER_DEADBAND_DEG_DEFAULT;
    steer.slew_deg_s   = STEER_SLEW_DEG_S_DEFAULT;
    steer.limit_deg    = STEER_LIMIT_DEG_DEFAULT;
    steer.head_kp      = STEER_HEAD_KP_DEFAULT;

    steer.angle_enable = 0;
    steer.head_enable  = 0;
    steer.was_enabled  = 0;

    steer.target_delta  = 0.0f;
    steer.applied_delta = 0.0f;
    steer.meas_delta    = 0.0f;
    steer.err           = 0.0f;
    steer.u_ff          = 0.0f;
    steer.output_duty   = 0;
    steer.target_yaw    = 0.0f;
    steer.meas_yaw      = 0.0f;

    steer.fault         = 0;
    steer.enc_bad_ms    = 0;
    steer.sat_ms        = 0;
    steer.sat_chk_ms    = 0;
    steer.err_ref       = 0.0f;
    steer.sat_warn      = 0;
    steer.sat_warn_ms   = 0;
    steer.estop         = 0;
}

/* 浮点绝对值: 自己写, 不引 math.h(IAR 默认库配置下少一个链接依赖少一个坑) */
static float steer_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 前提用"指令侧"的 |输出|: 反馈一坏, 误差不再收敛、输出会顶到限幅, 很快超过
 * STEER_ENC_LOST_OUT。所以"在出力 + 角度连续读无效"是可靠判据。
 * 必须在门控之前调用; 用到的 output_duty 是上一拍真正给出去的值。
 */
static void steer_ctrl_enc_check (void)
{
    float o = (float)steer.output_duty;

    if (0 != (steer.fault & STEER_FLT_ENC))
    {
        return;                                 /* 已锁存, 等 0->1 使能边沿清 */
    }

    /* 前提不成立(没使能 / 出力还没到阈值) -> 只清计时, 不判 */
    if ((0 == steer.angle_enable) ||
        ((o < STEER_ENC_LOST_OUT) && (o > -STEER_ENC_LOST_OUT)))
    {
        steer.enc_bad_ms = 0;
        return;
    }

    if ((steer_abs_get_raw() <= STEER_ENC_INVALID_LOW) ||
        (steer_abs_get_raw() >= STEER_ENC_INVALID_HIGH))
    {
        steer.enc_bad_ms += STEER_TICK_MS;
    }
    else
    {
        steer.enc_bad_ms = 0;
    }

    if (steer.enc_bad_ms >= STEER_ENC_LOST_MS)
    {
        steer.enc_bad_ms = 0;
        steer.fault     |= STEER_FLT_ENC;
    }
}

void steer_ctrl_update (void)
{
    float r;
    float err;
    float u;

    /* 0) 上位机使能开关(参数表 -> steer_en_cmd)按【边沿】同步到内部状态。 这样 0->1 那一拍"用当前实测角初始化目标"的保护仍然有效。 */
    {
        uint8 en_cmd = (steer_en_cmd > 0.5f) ? 1 : 0;

        if (en_cmd != steer.angle_enable)
        {
            steer_ctrl_set_angle_enable(en_cmd);
        }
    }

    /* 角度换算是测量量, 与使能无关, 始终更新(供显示与上层使用) */
    steer.meas_delta = steer_abs_get_deg();

    steer_ctrl_enc_check();

    /* 1) 外环(航向): 开了才覆写 target_delta; 没开则沿用上位机直给的目标转角 */
    if (steer.head_enable && steer.angle_enable)
    {
        steer.meas_yaw     = steer_yaw_deg;
        steer.target_delta = steer_clamp_deg(steer.head_kp * steer_wrap180(steer.target_yaw - steer.meas_yaw));
    }

    /* 2) 目标软限位: 上位机是直接写 target_delta 的(走参数表), 这里每拍补一次 */
    steer.target_delta = steer_clamp_deg(steer.target_delta);

    /* ① 让"**实际施加的目标**"始终对外可见（遥测发的是 `applied_delta`，见 motor_link.h 的
     * CH_STEER_TARGET 注解）—— 与是否使能无关。**否则失能时它会读成 0，
     * 滑条上写着 20°、波形上却是 0，现场只会当成"指令没下去"。**
     * ② 斜坡默认已经是 **关**（`STEER_SLEW_DEG_S_DEFAULT = 0`）⇒ 这一段的实际作用
     * 才是"最快到位"。加了斜坡只剩纯滞后 —— 我们这条还是 40 度/秒（0→27° 要 675 ms）。
     */
    r = steer.target_delta;
    if (steer.slew_deg_s > 0.0f)
    {
        float lim = steer.slew_deg_s * STEER_T_S;

        if (r > steer.applied_delta + lim) { r = steer.applied_delta + lim; }
        if (r < steer.applied_delta - lim) { r = steer.applied_delta - lim; }
    }
    steer.applied_delta = r;

    /* 4) 门控不通过: 清状态, 并【只在"使能->失能"那一拍】写一次 0。
     * 失能后不再每拍写电机, 否则会和手动开环输出互相打架。
     * 三种不通过: 总使能关 / 主动保护锁存(STEER_FLT_ENC 或 STEER_FLT_SAT) / **遥控急停**。
     * 而且 `steer_ctrl_set_angle_enable()` 的 0->1 上升沿本来就会把它重置成实测角。
     */
    if ((0 == steer.angle_enable) || (0 != steer.fault) || (0 != steer.estop))
    {
        if (steer.was_enabled)
        {
            motor_drv_set(MOTOR_CH_STEER, 0.0f);
        }
        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;
    }

    /* 5) 误差 = 实际施加目标 - 实测角 */
    err = r - steer.meas_delta;

    /* 6) 越限保护: 实测已超出【实际生效限位】、且误差仍指向限位方向 -> 不再出力。 保证即使因外部原因压到限位, 电机也不会继续加力去顶限位结构。 */
    {
        float lim = steer_ctrl_eff_limit();

        if (((steer.meas_delta >  lim) && (err > 0.0f)) ||
            ((steer.meas_delta < -lim) && (err < 0.0f)))
        {
            err = 0.0f;
        }
    }

    /* 7) 误差死区: 抑制静止抖动 */
    if ((err < steer.deadband_deg) && (err > -steer.deadband_deg))
    {
        err = 0.0f;
    }

    /* 8) 位置环 + 静摩擦前馈 + 输出限幅
     * u = kp * err   : 细调
     * u0 * sign(err) : 克服静摩擦与减速箱阻力
     * err 已过死区与越限保护, 等于 0 时 u0 也不加, 静止不会抖
     */
    u = steer.kp * err;

    steer.u_ff = 0.0f;
    if      (err > 0.0f) { steer.u_ff =  steer.u0; }
    else if (err < 0.0f) { steer.u_ff = -steer.u0; }
    u += steer.u_ff;

    /* 8) 顶死 / 无进展告警（判据与理由见 steer_ctrl.h 的 STEER_SAT_*）
     * 满转到 27 不会烧；而这条在行程末端会误报，失能比不保护还碍事）。
     * 判据 = 输出顶到限幅 **且误差无进展**：
     * · 饱和中 -> 每 STEER_SAT_CHK_MS 比一次 |err|，比上次缩小了 STEER_SAT_PROGRESS_DEG 以上
     * · |err| 连 0.5 度都不缩 => 记"无进展"，累计到 STEER_SAT_TRIP_MS 就置告警（粘滞 2 s）。
     * [i] 判在"限幅之前", 看的才是控制器真正想给出的力。
     * [i] 已经压在【实际生效限位】±1 度以内时**不记账** —— 那一段机械上本来就到头了。
     */
    {
        float ae      = steer_fabs(err);
        float lim_now = steer_ctrl_eff_limit();
        uint8 at_lim  = (steer_fabs(steer.meas_delta) >= (lim_now - 1.0f)) ? 1u : 0u;
        uint8 saturated = ((steer.u_max > 0.0f) &&
                           (steer_fabs(u) >= (STEER_SAT_OUT_RATIO * steer.u_max)) &&
                           (0 == at_lim)) ? 1u : 0u;

        if ((0 == saturated) || (ae < STEER_SAT_ERR_DEG))
        {
            /* 没饱和 / 误差本来就小 -> 正常工作区, 计数全清 */
            steer.sat_ms     = 0;
            steer.sat_chk_ms = 0;
            steer.err_ref    = ae;
        }
        else
        {
            steer.sat_chk_ms += STEER_TICK_MS;

            if (steer.sat_chk_ms >= STEER_SAT_CHK_MS)
            {
                steer.sat_chk_ms = 0;

                if (ae <= (steer.err_ref - STEER_SAT_PROGRESS_DEG))
                {
                    steer.sat_ms = 0;                       /* 有进展 -> 清零 */
                }
                else
                {
                    steer.sat_ms += STEER_SAT_CHK_MS;       /* 无进展 -> 累计 */
                }

                steer.err_ref = ae;
            }
        }

        if (steer.sat_ms >= STEER_SAT_TRIP_MS)
        {
            steer.sat_ms      = 0;
            steer.sat_warn    = 1;
            steer.sat_warn_ms = STEER_SAT_WARN_HOLD_MS;
        }
        else if (0 != steer.sat_warn)
        {
            /* 粘滞：条件消失后再挂 2 s，让 50 Hz 的遥测/上位机一定抓得住 */
            if (steer.sat_warn_ms > STEER_TICK_MS) { steer.sat_warn_ms -= STEER_TICK_MS; }
            else                                   { steer.sat_warn_ms = 0; steer.sat_warn = 0; }
        }
    }

    if (u >  steer.u_max) { u =  steer.u_max; }
    if (u < -steer.u_max) { u = -steer.u_max; }

    /* 本拍刚触发保护 -> 立刻把输出钉 0, 不等下一拍 */
    if (0 != steer.fault)
    {
        steer.err         = err;
        steer.u_ff        = 0.0f;
        steer.output_duty = 0;
        steer.was_enabled = 1;
        motor_drv_set(MOTOR_CH_STEER, 0.0f);
        return;
    }

    steer.err         = err;
    steer.output_duty = (int16)(STEER_LOOP_SIGN * u);
    steer.was_enabled = 1;

    motor_drv_set(MOTOR_CH_STEER, (float)steer.output_duty / (float)PWM_DUTY_MAX);
}

/* =========================== 在线设置接口 =========================== */
void steer_ctrl_set_angle_enable (uint8 en)
{
    if (en && (0 == steer.angle_enable))
    {
        /* 0->1 上升沿: 用当前实测转角作为目标, 并让斜坡从实测处起步,
         * 消除大初始误差防超调。在这里做是安全的 —— 调用点已保证 steer_abs_update() 先跑过。
         */
        steer.target_delta  = steer_abs_get_deg();
        steer.applied_delta = steer.target_delta;

        /* 顺序很重要: 必须在"取当前实测角"之后清, 否则目标会被钉在故障前的旧值上。 */
        steer_ctrl_clear_fault();
    }

    steer.angle_enable = en ? 1 : 0;

    if (0 == steer.angle_enable)
    {
        steer.head_enable = 0;              /* 关内环连带关外环, 防外环空转 */
    }
}

/* =========================== 主动保护接口 =========================== */
uint8 steer_ctrl_fault (void)
{
    /* 车端遥测把它 <<1 变成 steer_status 的 bit1/bit2 —— 上位机的解读按 0x01/0x02 走，别改顺序。 */
    return (uint8)(steer.fault | ((0 != steer.sat_warn) ? STEER_FLT_SAT : 0u));
}

void steer_ctrl_clear_fault (void)
{
    steer.fault       = 0;
    steer.enc_bad_ms  = 0;
    steer.sat_ms      = 0;
    steer.sat_chk_ms  = 0;
    steer.err_ref     = 0.0f;
    steer.sat_warn    = 0;                  /* 告警本来就会自清, 复位时一起清干净 */
    steer.sat_warn_ms = 0;
}

/* =========================== 遥控急停接口 ===========================
 * 只置一个标志位，真正的"失能 + 钉 0"由 steer_ctrl_update() 的门控完成
 * （那样才只有一条收口，不会出现"这里也写 0、那里也写 0"的互相打架）。
 */
void steer_ctrl_set_estop (uint8 on)
{
    steer.estop = on ? 1u : 0u;
}

uint8 steer_ctrl_estop (void)
{
    return steer.estop;
}

void steer_ctrl_set_head_enable (uint8 en)
{
    if (en && (0 == steer.head_enable))
    {
        steer.angle_enable = 1;             /* 开外环必须先有内环托底 */
        steer.target_yaw   = steer_yaw_deg; /* 用当前航向初始化目标 */
        steer.head_enable  = 1;
    }
    else if (0 == en)
    {
        steer.head_enable = 0;
    }
}

void steer_ctrl_set_target_delta (float deg)
{
    steer.target_delta = steer_clamp_deg(deg);
}

void steer_ctrl_set_target_yaw (float yaw)
{
    steer.target_yaw = yaw;
}

void steer_ctrl_set_angle_pid (float kp)
{
    steer.kp = kp;
}

void steer_ctrl_set_u0 (float u0)
{
    steer.u0 = u0;
}

void steer_ctrl_set_angle_outmax (float outmax)
{
    /* 不清状态: 行驶中调它不该让方向盘抖一拍 */
    steer.u_max = outmax;
}

/* =========================== 取值接口(给遥测/调试) =========================== */
float steer_ctrl_get_target_delta (void) { return steer.target_delta; }
float steer_ctrl_get_meas_delta   (void) { return steer.meas_delta; }
int16 steer_ctrl_get_output       (void) { return steer.output_duty; }
float steer_ctrl_get_target_yaw   (void) { return steer.target_yaw; }
