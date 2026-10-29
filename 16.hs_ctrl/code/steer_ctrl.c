/*********************************************************************************************************************
 * 转向位置环实现 —— 控制律、单位与参数含义见 steer_ctrl.h
 *
 * 数据流(5 ms 一拍, 紧跟 steer_abs_update() 之后):
 * [外环开] 目标航向 − steer_yaw_deg(归一到 ±180) → head_kp → 目标转角(夹软限位)
 * [内环]   目标转角 → 斜坡 → 夹软限位 → 误差 → 越限保护 → 死区(只影响 Kp/前馈)
 * → 三态滞环(前馈给不给, 方向取 sign(err)) → kp*err + ki*i_sum + u0*u_dir
 * → 限幅(u_max) → duty
 * 输出经 motor_drv_set(ch, 归一化 ±1) 下发, 即 duty/10000。
 *********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "motor_drv.h"

/* route_pb.c 的状态查询原型; 不 include route_pb.h, 避免头文件互相包含成环 */
uint8 route_pb_owns_target (void);

volatile float  steer_yaw_deg = 0.0f;
steer_ctrl_t    steer = {0};
volatile float  steer_en_cmd = 0.0f;

/* 航向误差归一到 −180~180, 保证走最近方向纠偏 */
static float steer_wrap180 (float err)
{
    /* 减法环绕加次数上限: 病态输入(inf/NaN/极大值)只会被夹到 ±180, 不会死循环卡住 ISR */
    uint8 guard = 0u;

    if (!(err == err))                                     { return 0.0f; }   /* NaN */
    while ((err >  180.0f) && (guard < 64u)) { err -= 360.0f; guard ++; }
    guard = 0u;
    while ((err < -180.0f) && (guard < 64u)) { err += 360.0f; guard ++; }
    if (err >  180.0f) { err =  180.0f; }                /* 超过 64 圈视为病态, 直接夹住 */
    if (err < -180.0f) { err = -180.0f; }
    return err;
}

/* 积分重标: 参数表改 ki 时按 I 守恒换 i_sum, 使改增益不扰动当前积分输出。
 * i_sum_new = ki_old * i_sum_old / ki_new, 再按新上限夹一次(上限优先, 夹后不再守恒)。
 * ki_new <= 0 时 i_sum 清 0, 等同关闭积分。
 * 由 update() 的 ki 边沿检测与 steer_ctrl_set_angle_i() 共用。 */
static void steer_i_rescale (float ki_old, float ki_new, float imax_new)
{
    float i_out = ki_old * steer.i_sum;          /* 改之前实际在出的 I(duty) */
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
    steer.ki_prev = ki_new;                      /* 同步快照, update() 下一拍不再重标 */
}

/* 门限关系钳位: 保证 0 < deadband, 0 < i_db < izone, brk >= deadband。
 * 参数可能来自上位机滑条、恢复默认、Flash 旧值或代码宏, 任一路都可能破坏关系。
 * 由 MOTOR_CMD_SET_PARAM、app_param_init()、steer_ctrl_init() 调用, 不在控制拍内调用。 */
void steer_ctrl_sanitize_thr (void)
{
    /* 保留关系: 0 < i_db < izone, 0 < db; brk 不参与控制, 只保证不小于 db */
    if (!(steer.deadband_deg >= 0.01f))                 { steer.deadband_deg = 0.01f; }
    if (!(steer.i_db_deg >= 0.0f))                      { steer.i_db_deg = 0.0f; }
    if (!(steer.i_db_deg < steer.izone_deg))            { steer.i_db_deg = steer.izone_deg * 0.01f; }
    if (!(steer.izone_deg > steer.i_db_deg))            { steer.izone_deg = steer.i_db_deg + 1.0f; }
    if (!(steer.brk_deg >= steer.deadband_deg))         { steer.brk_deg = steer.deadband_deg; }
}

/* 实际生效限位 度, 全系统唯一出口: min(steer.limit_deg, STEER_LIMIT_DEG_HARD)。
 * 遥控请求、上位机指令、目标斜坡、越限保护都走此处。 */
float steer_ctrl_eff_limit (void)
{
    float lim = steer.limit_deg;

    /* NaN/Inf 防护: 比较对 NaN 全为假, 不可信值一律退回硬上限, 否则限位会整体失效 */
    if (!(lim <= STEER_LIMIT_DEG_HARD)) { return STEER_LIMIT_DEG_HARD; }
    if (!(lim >= 0.0f))                 { return 0.0f; }

    return lim;
}

/* 目标转角软限幅, 夹在实际生效限位内 */
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
    steer.ki_prev      = STEER_KI_DEFAULT;      /* ki 快照, 必须与 ki 同步初值 */
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

    /* 上电先自检门限关系, 宏被改坏时当场收敛 */
    steer_ctrl_sanitize_thr();
}

/* 浮点绝对值, 不引 math.h */
static float steer_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/*===================================================================================================================
 *  ① 反馈失效判定
 *     前提用指令侧 |输出|: 反馈一坏误差不再收敛, 输出会顶到限幅并超过 STEER_ENC_LOST_OUT,
 *     因此在出力且角度连续读无效是可靠判据。
 *     必须在门控之前调用, 用到的 output_duty 是上一拍真正给出去的值。
 *=================================================================================================================*/
static void steer_ctrl_enc_check (void)
{
    float  o = (float)steer.output_duty;
    uint16 raw_live = steer_abs_get_raw_live();

    /* 已锁存则不再判定, 解除途径只有 steer_ctrl_clear_fault()(由 steer_en 走一次 0->1 触发) */
    if (0 != (steer.fault & STEER_FLT_ENC))
    {
        return;
    }

    /* 前提不成立(未使能 / 输出未达阈值) -> 只清计时, 不判定 */
    if ((0 == steer.angle_enable) ||
        ((o < STEER_ENC_LOST_OUT) && (o > -STEER_ENC_LOST_OUT)))
    {
        steer.enc_bad_ms = 0;
        return;
    }

    /* 用未滤波的实读值判定掉线: <= 3 或 >= 4092 视为整帧全 0 / 全 1; 好样直接清零 */
    if ((raw_live <= STEER_ENC_INVALID_LOW) ||
        (raw_live >= STEER_ENC_INVALID_HIGH))
    {
        steer.enc_bad_ms += STEER_TICK_MS;
    }
    else
    {
        steer.enc_bad_ms = 0;
    }

    /* 连续无效满 STEER_ENC_LOST_MS -> 锁存, 门控把输出钉 0 */
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
    float u_p    = 0.0f;                        /* 本拍 Kp 项输出, 死区/越限时置 0 */
    float u_i    = 0.0f;                        /* 本拍积分项输出, 已按 i_max 限幅 */
    float u_ff   = 0.0f;                        /* 本拍静摩擦前馈项输出 */
    float i_add  = 0.0f;                        /* 本拍累加进积分的量, 0 = 不累加 */
    float ae;                                   /* |err| */
    uint8 block  = 0u;                          /* 1 = 越限保护生效, 本拍三项都不出力 */

    /* 0) 使能开关按边沿同步到内部状态, 保证 0->1 那一拍用实测角初始化目标 */
    {
        uint8 en_cmd = (steer_en_cmd > 0.5f) ? 1 : 0;

        if (en_cmd != steer.angle_enable)
        {
            steer_ctrl_set_angle_enable(en_cmd);
        }
    }

    /* 角度换算是测量量, 与使能无关, 始终更新 */
    steer.meas_delta = steer_abs_get_deg();

    /* 0b) 反馈失效判定, 用本拍原始读数与上一拍输出, 先于门控 */
    steer_ctrl_enc_check();

    /* 1) 外环(航向)开启时覆写 target_delta, 否则使用上位机直给的目标转角。
     *    符号取负: delta 正 = 右转, yaw 正 = 左转, 故 delta = -kp*(yaw_tgt - yaw)。
     *    回放进行中目标转角归路线层, 航向环不得覆写。 */
    if (steer.head_enable && steer.angle_enable && (0u == route_pb_owns_target()))
    {
        steer.meas_yaw     = steer_yaw_deg;
        steer.target_delta = steer_clamp_deg(-steer.head_kp * steer_wrap180(steer.target_yaw - steer.meas_yaw));
    }

    /* 2) 目标软限位: 上位机直接写 target_delta, 这里每拍补夹一次 */
    steer.target_delta = steer_clamp_deg(steer.target_delta);

    /* 3) 目标处理: 软限位夹紧 + 可选变化率限制, 放在门控之前, 使 applied_delta 始终对外可见。
     *    斜坡默认关(STEER_SLEW_DEG_S_DEFAULT = 0), 实际作用只剩软限位夹紧。 */
    r = steer.target_delta;
    if (steer.slew_deg_s > 0.0f)
    {
        float lim = steer.slew_deg_s * STEER_T_S;

        if (r > steer.applied_delta + lim) { r = steer.applied_delta + lim; }
        if (r < steer.applied_delta - lim) { r = steer.applied_delta - lim; }
    }
    steer.applied_delta = r;

    /* 4) 门控不通过: 清状态, 并且只在使能转失能那一拍写一次 0, 之后不再每拍写电机。
     *    三种不通过: 总使能关 / 保护锁存 / 遥控急停。applied_delta 不清, 它是对外可见量。 */
    if ((0 == steer.angle_enable) || (0 != steer.fault) || (0 != steer.estop))
    {
        if (steer.was_enabled)
        {
            motor_drv_set(MOTOR_CH_STEER, 0.0f);
        }
        steer.err           = 0.0f;
        steer.u_ff          = 0.0f;
        steer.i_sum         = 0.0f;         /* 失能/保护/急停: 清积分记忆, 下次使能从 0 学 */
        steer.u_dir         = 0;            /* 前馈方向一并清 */
        steer.ki_prev       = steer.ki;     /* 门控期间同步 ki 快照, 使边沿检测只在真变时触发 */
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;
    }

    /* 5) 误差 = 实际施加目标 - 实测角 */
    err = r - steer.meas_delta;

    /* 6) 越限保护: 实测已超出实际生效限位且误差仍指向限位方向 -> 本拍三项都不出力。
     *    只置 block 标志, err 保持真值。 */
    {
        float lim = steer_ctrl_eff_limit();

        block = (((steer.meas_delta >  lim) && (err > 0.0f)) ||
                 ((steer.meas_delta < -lim) && (err < 0.0f))) ? 1u : 0u;
    }

    /* 6b) ki 无扰切换: 参数表直接写 steer.ki, 不经过 set_angle_i, 只能靠边沿检测补上 */
    if (steer.ki != steer.ki_prev)
    {
        steer_i_rescale(steer.ki_prev, steer.ki, steer.i_max);
    }

    /* 7) 三项参与判定: 死区只撤 Kp 与前馈, 积分另有更小的下界 i_db;
     *    大误差时积分清零; err 始终为真值。 */
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
        /* 前馈门控用三态滞环: |err| > brk 开门, |err| <= db 关门, 带内保持上一拍状态。
         * 方向每拍取 sign(err), 带内也不取上一拍方向。
         * 间隔 brk - db 需大于噪声振幅(1 计数 = 0.02 度)。 */
        {
            int8 s = (err >= 0.0f) ? 1 : -1;        /* 本拍方向, 0 归 + 侧 */

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
                steer.u_dir = s;                    /* 带内: 门保持开, 方向重新取 */
            }
            /* 带内且门关 -> 继续关, 即滞环的作用 */
        }

        /* Kp: 死区内本拍不参与 */
        u_p = (ae < steer.deadband_deg) ? 0.0f : (steer.kp * err);

        /* 前馈由上面的滞环决定给不给 */
        u_ff = (0 != steer.u_dir) ? (steer.u0 * (float)steer.u_dir) : 0.0f;

        /* 积分累加门限: 只挡纯噪声(编码器 1 计数约 0.02 度), 不能设 0 */
        i_add = (ae < steer.i_db_deg) ? 0.0f : err;
    }

    /* 8) 积分: |err| > izone 时清零(大阶跃天然抗饱和), 否则累加 i_add。
     *    限幅在积分输出空间判, 撞限后反算 i_sum; ki = 0 时清 i_sum。 */
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

    /* 9) 合成三项后按 u_max 限幅一次 */
    steer.u_ff = u_ff;
    u          = u_p + u_i + u_ff;

    /* 顶死 / 无进展告警: 输出顶到限幅, 且连续 STEER_SAT_TRIP_MS 内 |err| 缩小不足
     * STEER_SAT_PROGRESS_DEG 即置 sat_warn, 只报警不钉输出。
     * 判定在限幅之前, 用控制器想给出的力; 已压在限位 ±1 度内不记账。 */
    {
        float ae      = steer_fabs(err);
        float lim_now = steer_ctrl_eff_limit();
        uint8 at_lim  = (steer_fabs(steer.meas_delta) >= (lim_now - 1.0f)) ? 1u : 0u;
        uint8 saturated = ((steer.u_max > 0.0f) &&
                           (steer_fabs(u) >= (STEER_SAT_OUT_RATIO * steer.u_max)) &&
                           (0 == at_lim)) ? 1u : 0u;

        if ((0 == saturated) || (ae < STEER_SAT_ERR_DEG))
        {
            /* 未饱和 / 误差本就小 -> 计数全清 */
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
            steer.sat_ms      = 0;                          /* 只报警, 不改 fault, 不钉输出 */
            steer.sat_warn    = 1;
            steer.sat_warn_ms = STEER_SAT_WARN_HOLD_MS;
        }
        else if (0 != steer.sat_warn)
        {
            /* 粘滞: 条件消失后再挂 2 s, 便于遥测与上位机抓到 */
            if (steer.sat_warn_ms > STEER_TICK_MS) { steer.sat_warn_ms -= STEER_TICK_MS; }
            else                                   { steer.sat_warn_ms = 0; steer.sat_warn = 0; }
        }
    }

    /* 输出限幅: u_max 只在三项合成之后判一次。大阶跃期间 |err| > izone 积分已清零,
     * 不需要额外的抗饱和回滚。 */
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

/* ---- 在线设置接口 ---- */
void steer_ctrl_set_angle_enable (uint8 en)
{
    if (en && (0 == steer.angle_enable))
    {
        /* 0->1 上升沿: 用当前实测角作为目标并夹进软限位, 斜坡也从实测处起步, 消除大初始误差 */
        steer.target_delta  = steer_clamp_deg(steer_abs_get_deg());
        steer.applied_delta = steer.target_delta;

        /* 同时清保护锁存与计时, 这是人工复位途径; 顺序必须在取实测角之后 */
        steer_ctrl_clear_fault();

        /* 清积分记忆: 上升沿视为重新开始, 上一轮保持力矩不带过来 */
        steer.i_sum = 0.0f;
        steer.u_dir = 0;
    }

    steer.angle_enable = en ? 1 : 0;

    /* 必须同时写 steer_en_cmd, 否则 update() 第 0 步下一拍会把 angle_enable 按回 0 */
    steer_en_cmd = en ? 1.0f : 0.0f;

    if (0 == steer.angle_enable)
    {
        steer.head_enable = 0;              /* 关内环连带关外环, 防外环空转 */
    }
}

/* ---- 主动保护接口 ---- */
uint8 steer_ctrl_fault (void)
{
    /* bit0 = 锁存故障(钉输出) / bit1 = 顶死告警(只报警) / bit2 = 本拍读数被防抖拒掉的瞬态。
     * 车端左移 1 位拼入 steer_status, 上位机按 0x01/0x02/0x04 解读, 顺序不可改。 */
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
    steer.sat_warn    = 0;                  /* 告警会自清, 复位时一并清干净 */
    steer.sat_warn_ms = 0;
    /* 不清 ar_win_ms 与 recover_cnt: 前者维持自动复位限次的窗口, 后者为累计发生次数 */
}

/* ---- 遥控急停接口 ----
 * 只置标志位, 失能与钉 0 由 steer_ctrl_update() 的门控统一完成, 保证只有一条收口。 */
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
        /* 开外环先确保内环已使能, 走正规入口以取得上升沿处理并同步 steer_en_cmd */
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

/* 航向环增益在线设置: ld 随速度变, 等价增益 2L/ld 也必须每拍重算 */
void steer_ctrl_set_head_kp (float kp)
{
    if (kp < 0.0f) { kp = 0.0f; }
    steer.head_kp = kp;
}

void steer_ctrl_set_angle_i (float ki, float i_max)
{
    if (ki < 0.0f)      { ki = 0.0f; }
    if (i_max < 0.0f)   { i_max = 0.0f; }

    /* 与 update() 的 6b) 共用 steer_i_rescale, 两条入口行为一致 */
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
    /* 不清状态, 行驶中调整不应让方向盘抖一拍 */
    steer.u_max = outmax;
}

/* ---- 取值接口, 供遥测/调试 ---- */
float steer_ctrl_get_target_delta (void) { return steer.target_delta; }
float steer_ctrl_get_meas_delta   (void) { return steer.meas_delta; }
int16 steer_ctrl_get_output       (void) { return steer.output_duty; }
float steer_ctrl_get_target_yaw   (void) { return steer.target_yaw; }
