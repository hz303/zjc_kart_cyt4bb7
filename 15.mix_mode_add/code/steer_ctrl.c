/*********************************************************************************************************************
 * 转向位置环实现 —— 控制律、单位、参数含义见 steer_ctrl.h
 * 输出 duty 按 PWM_DUTY_MAX 归一化 后交 motor_drv_set(ch, ±1)
 *
 * 数据流(5ms 一拍, 紧跟 steer_abs_update()):
 *   外环开: 目标航向 - steer_yaw_deg(折 ±180) -> head_kp -> 目标转角(夹软限位)
 *   内环:   目标转角 ->(斜坡)-> 夹软限位 -> 误差 -> 越限保护 -> 死区(只影响积分/前馈)
 *            -> 三态滞环(决定前馈给不给) -> kp*err + ki*i_sum + u0*u_dir -> 限幅 -> duty
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "motor_drv.h"

/* route_pb.c 的 目标归属查询; 不 include route_pb.h(会与 steer_ctrl.h 成环) */
uint8 route_pb_owns_target (void);

volatile float  steer_yaw_deg = 0.0f;
steer_ctrl_t    steer = {0};
volatile float  steer_en_cmd = 0.0f;

/* 航向误差归一 -180~180, 走最近方向 纠偏; 带次数上限与 NaN 保护, 病态输入夹到 ±180 */
static float steer_wrap180 (float err)
{
    uint8 guard = 0u;

    if (!(err == err))                                     { return 0.0f; }   /* NaN */
    while ((err >  180.0f) && (guard < 64u)) { err -= 360.0f; guard ++; }
    guard = 0u;
    while ((err < -180.0f) && (guard < 64u)) { err += 360.0f; guard ++; }
    if (err >  180.0f) { err =  180.0f; }                /* 超 64 圈 = 数值病态, 直接夹住 */
    if (err < -180.0f) { err = -180.0f; }
    return err;
}

/* 积分重标, 改 ki 不扰动输出: i_sum 存的是 累计误差, 直接改 ki 会突变积分输出,
   故按 I 守恒换算 i_sum_new = ki_old*i_sum_old/ki_new, 再按新上限夹一次(上限优先);
   ki_new = 0 则清 0 等于关积分。参数表直写 ki 与 set_angle_i 共用本函数。 */
static void steer_i_rescale (float ki_old, float ki_new, float imax_new)
{
    float i_out = ki_old * steer.i_sum;          /* 改之前的积分输出(duty) */
    float isum  = 0.0f;

    steer.ki    = ki_new;
    steer.i_max = imax_new;

    if (ki_new > 1.0e-6f)
    {
        float ilim = imax_new / ki_new;

        isum = i_out / ki_new;
        if (isum >  ilim) { isum =  ilim; }
        if (isum < -ilim) { isum = -ilim; }
    }

    steer.i_sum   = isum;
    steer.ki_prev = ki_new;                      /* 同步快照, update() 下一拍不会再重标 */
}

/* 门限关系钳位: 保证 0 < i_db < izone(需 积分窗口非空)、deadband > 0、brk >= deadband。
   三个值来自滑条、参数表恢复、Flash 旧 blob、改宏, 任一路都能破坏关系;
   只在参数下发、Flash 加载后、上电初值调用, 不在控制拍里钳。 */
void steer_ctrl_sanitize_thr (void)
{
    if (!(steer.deadband_deg >= 0.01f))                 { steer.deadband_deg = 0.01f; }
    if (!(steer.i_db_deg >= 0.0f))                      { steer.i_db_deg = 0.0f; }
    if (!(steer.i_db_deg < steer.izone_deg))            { steer.i_db_deg = steer.izone_deg * 0.01f; }
    if (!(steer.izone_deg > steer.i_db_deg))            { steer.izone_deg = steer.i_db_deg + 1.0f; }
    if (!(steer.brk_deg >= steer.deadband_deg))         { steer.brk_deg = steer.deadband_deg; }
}

/* 实际生效限位 度 = min(steer.limit_deg, 27); 遥控、上位机 s/p 指令、斜坡、越限保护全走这里,
   限位只有一处; 不可信值(NaN/Inf/负)一律退回硬上限。 */
float steer_ctrl_eff_limit (void)
{
    float lim = steer.limit_deg;

    if (!(lim <= STEER_LIMIT_DEG_HARD)) { return STEER_LIMIT_DEG_HARD; }
    if (!(lim >= 0.0f))                 { return 0.0f; }

    return lim;
}

/* 目标转角 夹在实际生效限位内, 不顶机械硬限位 */
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
    steer.ki           = STEER_KI_DEFAULT;
    steer.i_max        = STEER_IMAX_DEFAULT;
    steer.ki_prev      = STEER_KI_DEFAULT;      /* 无扰切换的上一拍快照, 必须与 ki 同初值 */
    steer.brk_deg      = STEER_BRK_DEG_DEFAULT;
    steer.izone_deg    = STEER_IZONE_DEG_DEFAULT;
    steer.i_db_deg     = STEER_I_DB_DEG_DEFAULT;

    steer.angle_enable = 0;
    steer.head_enable  = 0;
    steer.was_enabled  = 0;

    steer.target_delta  = 0.0f;
    steer.applied_delta = 0.0f;
    steer.meas_delta    = 0.0f;
    steer.err           = 0.0f;
    steer.u_ff          = 0.0f;
    steer.i_sum         = 0.0f;
    steer.u_dir         = 0;
    steer.output_duty   = 0;
    steer.target_yaw    = 0.0f;
    steer.meas_yaw      = 0.0f;

    steer.fault         = 0;
    steer.enc_bad_ms    = 0;
    steer.ar_ms         = 0;
    steer.ar_win_ms     = 0;
    steer.ar_tries      = 0;
    steer.recover_cnt   = 0;
    steer.sat_ms        = 0;
    steer.sat_chk_ms    = 0;
    steer.err_ref       = 0.0f;
    steer.sat_warn      = 0;
    steer.sat_warn_ms   = 0;
    steer.estop         = 0;

    /* 上电先收敛门限关系, 参数被改坏时 当场收住 */
    steer_ctrl_sanitize_thr();
}

/* 浮点绝对值, 不引 math.h */
static float steer_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/* ① 反馈失效判定, 必须在门控之前调用; 前提用 指令侧 |输出|: 反馈坏了就没有角度可读 */
static void steer_ctrl_enc_check (void)
{
    float  o = (float)steer.output_duty;
    uint16 raw_live = steer_abs_get_raw_live();

    /* 已锁存不再判, 只能由 steer_ctrl_clear_fault() 解除(steer_en 走一次 0->1) */
    if (0 != (steer.fault & STEER_FLT_ENC))
    {
        return;
    }

    /* 前提不成立(未使能 / 出力未到阈值) -> 只清计时 */
    if ((0 == steer.angle_enable) ||
        ((o < STEER_ENC_LOST_OUT) && (o > -STEER_ENC_LOST_OUT)))
    {
        steer.enc_bad_ms = 0;
        return;
    }

    /* 用未滤波的实读值判掉线: <=3 / >=4092 即 整帧全 0 / 全 1 */
    if ((raw_live <= STEER_ENC_INVALID_LOW) ||
        (raw_live >= STEER_ENC_INVALID_HIGH))
    {
        steer.enc_bad_ms += STEER_TICK_MS;
    }
    else
    {
        steer.enc_bad_ms = 0;
    }

    /* 连续无效满 STEER_ENC_LOST_MS(200ms) -> 锁存, 门控把输出 钉 0 */
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
    float u_p    = 0.0f;                        /* 本拍 Kp 项输出(死区/越限时置 0死区/越限时置 0) */
    float u_i    = 0.0f;                        /* 本拍积分项输出(已按 i_max 限幅) */
    float u_ff   = 0.0f;                        /* 本拍前馈项输出(死区/越限时置 0) */
    float i_add  = 0.0f;                        /* 本拍累加进积分的量(0 = 不累加) */
    float ae;                                   /* |err|, 本段算一次到处用 */
    uint8 block  = 0u;                          /* 1 = 越限保护生效, 三项都不出力 */

    /* 0) 上位机使能开关 按边沿同步 到内部状态, 保留 0->1 那一拍的目标初始化 */
    {
        uint8 en_cmd = (steer_en_cmd > 0.5f) ? 1 : 0;

        if (en_cmd != steer.angle_enable)
        {
            steer_ctrl_set_angle_enable(en_cmd);
        }
    }

    /* 角度换算是 测量量, 与使能无关, 始终更新(供显示与上层使用) */
    steer.meas_delta = steer_abs_get_deg();

    /* 0b) 反馈失效判定 */
    steer_ctrl_enc_check();

    /* 1) 外环: 开了才覆写 target_delta; 回放期间 目标归路线层所有, 不覆写
     *    delta = -kp*(yaw_tgt - yaw): yaw 正 = 左转、delta 正 = 右转, 必须取负 */
    if (steer.head_enable && steer.angle_enable && (0u == route_pb_owns_target()))
    {
        steer.meas_yaw     = steer_yaw_deg;
        steer.target_delta = steer_clamp_deg(-steer.head_kp * steer_wrap180(steer.target_yaw - steer.meas_yaw));
    }

    /* 2) 目标软限位: 上位机直写 target_delta, 这里每拍 补一次夹紧 */
    steer.target_delta = steer_clamp_deg(steer.target_delta);

    /* 3) 夹软限位 + 可选斜坡, 放在门控之前: 让 applied_delta 始终 对外可见(遥测发它) */
    r = steer.target_delta;
    if (steer.slew_deg_s > 0.0f)
    {
        float lim = steer.slew_deg_s * STEER_T_S;

        if (r > steer.applied_delta + lim) { r = steer.applied_delta + lim; }
        if (r < steer.applied_delta - lim) { r = steer.applied_delta - lim; }
    }
    steer.applied_delta = r;

    /* 4) 门控不通过: 清状态, 只在使能->失能那一拍写一次 0, 避免与手动开环输出打架
     *    不通过 = 总使能关 / 保护锁存 / 遥控急停 */
    if ((0 == steer.angle_enable) || (0 != steer.fault) || (0 != steer.estop))
    {
        if (steer.was_enabled)
        {
            motor_drv_set(MOTOR_CH_STEER, 0.0f);
        }
        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.i_sum         = 0.0f;         /* 失能/保护/急停 清积分记忆, 下次使能从 0 学 */
        steer.u_dir         = 0;            /* 前馈方向 一并清, 免得下次首拍带旧方向 */
        steer.ki_prev       = steer.ki;     /* 门控期间 同步 ki 快照, 使 6b) 只在真变时触发 */
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;
    }

    /* 5) 误差 = 实际施加目标 - 实测角 */
    err = r - steer.meas_delta;

    /* 6) 越限保护: 实测已超生效限位且误差仍指向限位 -> 本拍 三项都不出力;
     *    err 保留真值, 只是三项各不参与 */
    {
        float lim = steer_ctrl_eff_limit();

        block = (((steer.meas_delta >  lim) && (err > 0.0f)) ||
                 ((steer.meas_delta < -lim) && (err < 0.0f))) ? 1u : 0u;
    }

    /* 6b) ki 无扰切换: 参数表直写 steer.ki, 只能靠 边沿检测 补上(见 steer_i_rescale) */
    if (steer.ki != steer.ki_prev)
    {
        steer_i_rescale(steer.ki_prev, steer.ki, steer.i_max);
    }

    /* 7) 三项参与判定: 先看越限, 再按滞环定前馈, 死区只撤 Kp, 积分另有下界 i_db */
    ae = steer_fabs(err);

    if (0u != block)
    {
        u_p         = 0.0f;
        u_ff        = 0.0f;
        i_add       = 0.0f;
        steer.u_dir = 0;
    }
    else
    {
        /* 前馈三态滞环: |err| > brk 开门 / <= db 关门 / 带内保持上一拍的门 */
        {
            int8 s = (err >= 0.0f) ? 1 : -1;        /* 本拍方向, 0 归正侧 */

            if (ae > steer.brk_deg)
            {
                steer.u_dir = s;                    /* 开门 */
            }
            else if (ae <= steer.deadband_deg)
            {
                steer.u_dir = 0;                    /* 关门(含 err 恰为 0) */
            }
            else if (0 != steer.u_dir)
            {
                steer.u_dir = s;                    /* 带内: 门保持开, 方向取当拍 */
            }
            /* 带内且门是关的 -> 继续关着, 这就是滞环 */
        }

        /* Kp: 死区内 本拍不参与 */
        u_p = (ae < steer.deadband_deg) ? 0.0f : (steer.kp * err);

        /* 前馈由上面的 滞环 决定给不给 */
        u_ff = (0 != steer.u_dir) ? (steer.u0 * (float)steer.u_dir) : 0.0f;

        /* 积分下界: 只挡 纯噪声(编码器 1 计数 ≈ 0.02 度), 不能设 0 */
        i_add = (ae < steer.i_db_deg) ? 0.0f : err;
    }

    /* 8) 积分项: |err| > izone 清零(不是冻结), u_i 按 i_max 限幅, 撞限反算 i_sum
     *    ki = 0 时 u_i 恒 0, 顺带清 i_sum, 等价于 关掉积分 */
    if (steer.ki > 1.0e-6f)
    {
        if (ae > steer.izone_deg)
        {
            steer.i_sum = 0.0f;
        }
        else
        {
            steer.i_sum += i_add;
        }

        u_i = steer.ki * steer.i_sum;

        if (u_i > steer.i_max)
        {
            u_i         =  steer.i_max;
            steer.i_sum =  steer.i_max / steer.ki;
        }
        else if (u_i < -steer.i_max)
        {
            u_i         = -steer.i_max;
            steer.i_sum = -steer.i_max / steer.ki;
        }
    }
    else
    {
        steer.i_sum = 0.0f;
        u_i         = 0.0f;
    }

    /* 9) 合成, u_max 只在 三项加起来 之后判一次 */
    steer.u_ff = u_ff;
    u          = u_p + u_i + u_ff;

    /* 9b) 顶死/无进展告警: 只置告警位, 不钉输出; 判在限幅之前, 看的是控制器想给出的力
     *     饱和中每 STEER_SAT_CHK_MS 比一次 |err|, 连续 STEER_SAT_TRIP_MS 不缩小才置位(粘滞 2 s) */
    {
        float ae      = steer_fabs(err);
        float lim_now = steer_ctrl_eff_limit();
        uint8 at_lim  = (steer_fabs(steer.meas_delta) >= (lim_now - 1.0f)) ? 1u : 0u;
        uint8 saturated = ((steer.u_max > 0.0f) &&
                           (steer_fabs(u) >= (STEER_SAT_OUT_RATIO * steer.u_max)) &&
                           (0 == at_lim)) ? 1u : 0u;

        if ((0 == saturated) || (ae < STEER_SAT_ERR_DEG))
        {
            /* 没饱和或误差本来就小 -> 正常工作区, 计数全清 */
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
            steer.sat_ms      = 0;                          /* 只报警, 不碰 fault、不钉输出 */
            steer.sat_warn    = 1;
            steer.sat_warn_ms = STEER_SAT_WARN_HOLD_MS;
        }
        else if (0 != steer.sat_warn)
        {
            /* 粘滞: 条件消失后再挂 2 s, 让 50 Hz 遥测抓得住 */
            if (steer.sat_warn_ms > STEER_TICK_MS) { steer.sat_warn_ms -= STEER_TICK_MS; }
            else                                   { steer.sat_warn_ms = 0; steer.sat_warn = 0; }
        }
    }

    /* 10) 输出限幅 */
    if (u >  steer.u_max) { u =  steer.u_max; }
    if (u < -steer.u_max) { u = -steer.u_max; }

    /* 本拍刚触发保护 -> 立即钉 0, 不等下一拍 */
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
        /* 0->1 上升沿: 目标取当前实测角(夹进软限位), 斜坡也从实测处起步, 消除大初始误差 */
        steer.target_delta  = steer_clamp_deg(steer_abs_get_deg());
        steer.applied_delta = steer.target_delta;

        /* 清保护锁存与计时, 人工复位 途径; 必须在取实测角之后清, 否则目标钉在旧值上 */
        steer_ctrl_clear_fault();

        /* 清积分记忆: 上升沿视为 重新开始, 上一轮的保持力矩不带过来 */
        steer.i_sum = 0.0f;
        steer.u_dir = 0;
    }

    steer.angle_enable = en ? 1 : 0;

    /* 必须同时写 steer_en_cmd, 否则 update() 第 0 步下一拍就把它 按回 0 */
    steer_en_cmd = en ? 1.0f : 0.0f;

    if (0 == steer.angle_enable)
    {
        steer.head_enable = 0;              /* 关内环连带关外环, 防外环空转 */
    }
}

/* =========================== 主动保护接口 =========================== */
uint8 steer_ctrl_fault (void)
{
    /* bit0 锁存故障(钉输出) / bit1 顶死告警(只报告) / bit2 瞬态丢样, 顺序与上位机解读一致 */
    uint8 f = steer.fault;

    if (0 != steer.sat_warn)        { f |= STEER_FLT_SAT;  }
    if (0 != steer_abs_rejecting()) { f |= STEER_FLT_LINK; }

    return f;
}

void steer_ctrl_clear_fault (void)
{
    steer.fault       = 0;
    steer.enc_bad_ms  = 0;
    steer.ar_ms       = 0;
    steer.ar_tries    = 0;
    steer.sat_ms      = 0;
    steer.sat_chk_ms  = 0;
    steer.err_ref     = 0.0f;
    steer.sat_warn    = 0;                  /* 告警本来会自清, 复位时一并清干净 */
    steer.sat_warn_ms = 0;
    /* 故意不清 ar_win_ms 与 recover_cnt: 前者记窗口内已自动复位次数, 清掉就 绕过了限次 */
}

/* =========================== 遥控急停接口 ===========================
   只置标志位, 失能与钉 0 由 steer_ctrl_update() 的 门控收口, 避免多处写 0 打架 */
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
        /* 开外环须先有内环托底; 走 正规入口 拿 0->1 的初始化, 并同步 steer_en_cmd */
        if (0 == steer.angle_enable) { steer_ctrl_set_angle_enable(1u); }
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

/* 航向环增益在线设置: ld 随速度变, 等价增益必须运行时算, 固定常数 就是 换了个环 */
void steer_ctrl_set_head_kp (float kp)
{
    if (kp < 0.0f) { kp = 0.0f; }
    steer.head_kp = kp;
}

void steer_ctrl_set_angle_i (float ki, float i_max)
{
    if (ki < 0.0f)      { ki = 0.0f; }
    if (i_max < 0.0f)   { i_max = 0.0f; }

    /* 与 update() 的 6b) 同一原语, 两条入口行为必须一致 */
    steer_i_rescale(steer.ki, ki, i_max);
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
    /* 不清状态: 行驶中调它 不该让方向盘抖一拍 */
    steer.u_max = outmax;
}

/* =========================== 取值接口(给遥测/调试) =========================== */
float steer_ctrl_get_target_delta (void) { return steer.target_delta; }
float steer_ctrl_get_meas_delta   (void) { return steer.meas_delta; }
int16 steer_ctrl_get_output       (void) { return steer.output_duty; }
float steer_ctrl_get_target_yaw   (void) { return steer.target_yaw; }
