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

/* 目标转角软限幅: 夹在软限位内, 别顶机械硬限位 */
static float steer_clamp_deg (float deg)
{
    if (deg >  steer.limit_deg) { return  steer.limit_deg; }
    if (deg < -steer.limit_deg) { return -steer.limit_deg; }
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

    /* 1) 外环(航向): 开了才覆写 target_delta; 没开则沿用上位机直给的目标转角 */
    if (steer.head_enable && steer.angle_enable)
    {
        steer.meas_yaw     = steer_yaw_deg;
        steer.target_delta = steer_clamp_deg(steer.head_kp * steer_wrap180(steer.target_yaw - steer.meas_yaw));
    }

    /* 2) 目标软限位: 上位机是直接写 target_delta 的(走参数表), 这里每拍补一次 */
    steer.target_delta = steer_clamp_deg(steer.target_delta);

    /* 3) 门控不通过: 清状态, 并【只在"使能->失能"那一拍】写一次 0。 失能后不再每拍写电机, 否则会和手动开环输出互相打架。 */
    if (0 == steer.angle_enable)
    {
        if (steer.was_enabled)
        {
            motor_drv_set(MOTOR_CH_STEER, 0.0f);
        }
        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.output_duty   = 0;
        steer.applied_delta = 0.0f;
        steer.was_enabled   = 0;
        return;
    }

    r = steer.target_delta;
    if (steer.slew_deg_s > 0.0f)
    {
        float lim = steer.slew_deg_s * STEER_T_S;

        if (r > steer.applied_delta + lim) { r = steer.applied_delta + lim; }
        if (r < steer.applied_delta - lim) { r = steer.applied_delta - lim; }
    }
    steer.applied_delta = r;

    err = r - steer.meas_delta;

    /* 5) 越限保护: 实测已超出软限位、且误差仍指向限位方向 -> 不再出力。 保证即使因外部原因压到限位, 电机也不会继续加力去顶限位结构。 */
    if (((steer.meas_delta >  steer.limit_deg) && (err > 0.0f)) ||
        ((steer.meas_delta < -steer.limit_deg) && (err < 0.0f)))
    {
        err = 0.0f;
    }

    /* 6) 误差死区: 抑制静止抖动 */
    if ((err < steer.deadband_deg) && (err > -steer.deadband_deg))
    {
        err = 0.0f;
    }

    /* 7) 位置环 + 静摩擦前馈 + 输出限幅
     * u = kp * err   : 细调
     * u0 * sign(err) : 克服静摩擦与减速箱阻力
     * err 已过死区与越限保护, 等于 0 时 u0 也不加, 静止不会抖
     */
    u = steer.kp * err;

    steer.u_ff = 0.0f;
    if      (err > 0.0f) { steer.u_ff =  steer.u0; }
    else if (err < 0.0f) { steer.u_ff = -steer.u0; }
    u += steer.u_ff;

    if (u >  steer.u_max) { u =  steer.u_max; }
    if (u < -steer.u_max) { u = -steer.u_max; }

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
    }

    steer.angle_enable = en ? 1 : 0;

    if (0 == steer.angle_enable)
    {
        steer.head_enable = 0;              /* 关内环连带关外环, 防外环空转 */
    }
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
