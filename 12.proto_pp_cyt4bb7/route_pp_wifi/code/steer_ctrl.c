/* 数据流(5ms 一拍, 紧跟 steer_abs_update() 之后):
 * [外环开] 目标航向 − steer_yaw_deg (wrap±180) → head_kp → 目标转角(夹软限位)
 * [内环]   目标转角 →(斜坡)→ 手夹软限位 → 误差 → 越限保护 → 误差死区(只影响积分/前馈)
 * → 三态滞环(决定前馈给不给, 方向取 sign(err)) → kp*err + ki*i_sum + u0*u_dir
 * → 抗饱和(整拍回滚) → 限幅(u_max) → duty
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
    /* （上游任何一个量炸了：未初始化的 target_yaw、除零、NaN 之后的溢出…），
     * `-= 360` 永远退不出来 => 这个函数所在的 1 kHz ISR / 主循环就地卡死 =>
     * 菜单冻结、按键无效、WiFi·SPI 停发（TX 灯灭 / RX 被从机拉常亮）、电机不动。
     * 加次数上限 + NaN 保护：病态输入只会被夹到 +-180，绝不会再卡住。
     */
    uint8 guard = 0u;

    if (!(err == err))                                     { return 0.0f; }
    while ((err >  180.0f) && (guard < 64u)) { err -= 360.0f; guard ++; }
    guard = 0u;
    while ((err < -180.0f) && (guard < 64u)) { err += 360.0f; guard ++; }
    if (err >  180.0f) { err =  180.0f; }                /* 超过 64 圈 => 数值已病态，直接夹住 */
    if (err < -180.0f) { err = -180.0f; }
    return err;
}

/* 病根：`i_sum` 存的是**累计误差**，不是积分输出。直接改 `ki` 会立刻改变"当前正在出的 I"：
 * i_sum = 8 时   ki 60 → I=480 ｜ ki 120 → I=960
 * （"改完立刻不抖了 / 立刻开始抖"里混进一次人为突变）。
 * ⇒ 按 I 守恒重标：`i_sum_new = (ki_old · i_sum_old) / ki_new`，再按**新**上限夹一次。
 * （旧 I 无从换算，而且 ki=0 那段时间本来就没有保持力矩）。
 * · `steer_ctrl_update()` 的 6b)：参数表 X-Macro 直接写 `steer.ki`，只能靠边沿检测；
 * · `steer_ctrl_set_angle_i()`：代码/上层调用的正规入口。
 */
static void steer_i_rescale (float ki_old, float ki_new, float imax_new)
{
    float i_out = ki_old * steer.i_sum;          /* 改之前实际在出的 I（duty） */
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
    steer.ki_prev = ki_new;                      /* 同步快照：update() 下一拍不会再重标一次 */
}

/* 0 < deadband < brk ≤ izone
 * · `db = 0`      ⇒ |err| 永远 > db：积分区从 0 开始，静止也在积；
 * · `db >= brk`   ⇒ 滞环带为空 ⇒ 退回"单门限逐拍翻向"（抖动的原始结构）；
 * · `izone <= brk`⇒ 积分区为空 ⇒ 积分永不累加（静差消不掉，静止角又开始随机）。
 * ① 上位机滑条（一个个单独拖）；② `MOTOR_CMD_LOAD` 恢复表里的默认值；
 * ③ Flash 里存着的旧 blob（那时 ID21 默认就是 0.2，和 brk 相等）；
 * ④ 代码里改宏。
 * ⇒ 一次实现，三处调用：`MOTOR_CMD_SET_PARAM`（下发处）、`app_param_init()`（加载后）、
 * `steer_ctrl_init()`（上电初值）。
 */
void steer_ctrl_sanitize_thr (void)
{
    /* 0 < i_db < izone                       —— 积分窗口非空（否则永远不累加 or 永远清零）
     * 0 < db                                —— 死区必须 > 0（Kp/U0 的门限）
     */
    if (!(steer.deadband_deg >= 0.01f))                 { steer.deadband_deg = 0.01f; }
    if (!(steer.i_db_deg >= 0.0f))                      { steer.i_db_deg = 0.0f; }
    if (!(steer.i_db_deg < steer.izone_deg))            { steer.i_db_deg = steer.izone_deg * 0.01f; }
    if (!(steer.izone_deg > steer.i_db_deg))            { steer.izone_deg = steer.i_db_deg + 1.0f; }
    if (!(steer.brk_deg >= steer.deadband_deg))         { steer.brk_deg = steer.deadband_deg; }
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
    steer.ki           = STEER_KI_DEFAULT;
    steer.i_max        = STEER_IMAX_DEFAULT;
    steer.ki_prev      = STEER_KI_DEFAULT;
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

    /* 默认值本来就是 0.10 / 0.20 / 4.0，这一句是空操作 —— 它的价值在"以后有人改宏"。 */
    steer_ctrl_sanitize_thr();
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
    float  o = (float)steer.output_duty;
    uint16 raw_live;

    /* 现场原始现象（第一次跑起来抓到的完整时间线）：
     * reset → 一切正常（转盘控方向、油门控速度）→ 油门打满跑一小段 →
     * **方向盘左右晃两下 → 猛向右打死 → 此后转盘再也控不动方向盘**。
     * 旧逻辑的死结：锁存后**唯一**清除途径是 `steer_en` 走一次 0->1；而遥控在 ON/STANDBY 里
     * 恒写 `steer_en_cmd = 1`，永远不会有边沿 ⇒ 保护**只进不出** ⇒ 方向盘彻底死掉，只能 reset。
     * ⇒ 现在：链路连续干净 STEER_ENC_AR_MS 就自动复位（限次数，防"坏了就反复复活"）。
     */
    if (0 != (steer.fault & STEER_FLT_ENC))
    {
        if (0 == STEER_ENC_AR_ENABLE)
        {
            return;                             /* 关了 = 老行为(等人工 0->1) */
        }

        /* 窗口计时：窗口内次数用超了就一直等 —— 等窗口走完才允许重新试 */
        if (steer.ar_win_ms >= STEER_TICK_MS) { steer.ar_win_ms -= STEER_TICK_MS; }
        else                                  { steer.ar_win_ms = 0; steer.ar_tries = 0; }

        if (0 != steer_abs_link_ok()) { steer.ar_ms += STEER_TICK_MS; }
        else                          { steer.ar_ms = 0; }

        if ((steer.ar_ms >= STEER_ENC_AR_MS) && (steer.ar_tries < STEER_ENC_AR_MAX))
        {
            steer.ar_ms     = 0;
            steer.ar_tries ++;
            steer.ar_win_ms = STEER_ENC_AR_WIN_MS;
            steer.recover_cnt ++;

            steer.fault        &= (uint8)(~STEER_FLT_ENC);
            steer.enc_bad_ms    = 0;
            steer.err_ref       = 0.0f;
            steer.target_delta  = steer.meas_delta;
            steer.applied_delta = steer.meas_delta;
        }
        return;
    }

    /* 前提不成立(没使能 / 出力还没到阈值) -> 只清计时, 不判 */
    if ((0 == steer.angle_enable) ||
        ((o < STEER_ENC_LOST_OUT) && (o > -STEER_ENC_LOST_OUT)))
    {
        steer.enc_bad_ms = 0;
        return;
    }

    /* ================= 2) 本拍算不算"坏样" =================
     * ① 读数越界（<=3 / >=4092）：总线彻底死（全 0 / 全 1）—— 用**未滤波**的实读值判，
     * 因为防抖层会把坏样替换成上一个好值，看滤波后的值就永远发现不了掉线。
     * ② 防抖层本拍拒掉了一条（跳变不合理）—— 它比 ① 灵敏得多，是**最早的预警**。
     */
    raw_live = steer_abs_get_raw_live();

    if ((raw_live <= STEER_ENC_INVALID_LOW) ||
        (raw_live >= STEER_ENC_INVALID_HIGH) ||
        (0 != steer_abs_rejecting()))
    {
        /* 旧写法会把**偶发**毛刺攒成一串、在电机还没真正失稳时就误判成持续失效； 漏水积分只认"坏样占绝大多数"，才是真的链路坏了 —— 而偶发抖动交给第 ① 层防抖吃掉。 */
        steer.enc_bad_ms += STEER_TICK_MS;
    }
    else if (steer.enc_bad_ms >= STEER_TICK_MS)
    {
        steer.enc_bad_ms -= STEER_TICK_MS;
    }
    else
    {
        steer.enc_bad_ms = 0;
    }

    if (steer.enc_bad_ms >= STEER_ENC_LOST_MS)
    {
        steer.enc_bad_ms = 0;
        steer.ar_ms      = 0;                   /* 自动复位的"干净计时"从 0 开始 */
        steer.fault     |= STEER_FLT_ENC;
    }
}

void steer_ctrl_update (void)
{
    float r;
    float err;
    float u;
    float u_p    = 0.0f;                        /* 本拍 Kp 项输出（死区/越限时置 0） */
    float u_i    = 0.0f;                        /* 本拍积分项输出（已按 i_max 限幅） */
    float u_ff   = 0.0f;                        /* 本拍静摩擦前馈项输出（死区/越限时置 0） */
    float i_add  = 0.0f;                        /* 本拍累加进积分的量（0 = 本拍不累加） */
    float ae;                                   /* |err|，本段算一次到处用 */
    uint8 block  = 0u;                          /* 1 = 越限保护生效，本拍三项都不出力 */

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

    /* 1) 外环(航向): 开了才覆写 target_delta; 没开则沿用上位机直给的目标转角
     * 本工程的约定：`steer_delta > 0` = **右转**（实测），`yaw` 正 = **左转**
     * （`ins_odom` 里 `x += -sin(yaw)·ds` ⇒ yaw 增大车头往 −x 偏）。
     * ⇒ "要往右转"时目标航向比当前**小**，航向误差为负，而 δ 要为正
     * ⇒ `δ = −kp·(yaw_tgt − yaw)`。
     * 这就是"head_kp=15 谁开谁振"的真正原因（不是增益大，是**方向反**）。
     */
    if (steer.head_enable && steer.angle_enable)
    {
        steer.meas_yaw     = steer_yaw_deg;
        steer.target_delta = steer_clamp_deg(-steer.head_kp * steer_wrap180(steer.target_yaw - steer.meas_yaw));
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
        steer.i_sum         = 0.0f;         /* 失能/保护/急停: 清积分记忆，下次使能从 0 学 */
        steer.u_dir         = 0;            /* 前馈方向也清（否则下次使能第一拍会带旧方向） */
        steer.ki_prev       = steer.ki;     /* 第一拍会白跑一次重标（结果仍是 0，所以无害， 但让 6b) 的边沿检测语义保持"只在真变时触发"） */
        steer.output_duty   = 0;
        steer.was_enabled   = 0;
        return;
    }

    /* 5) 误差 = 实际施加目标 - 实测角 */
    err = r - steer.meas_delta;

    /* 6) 越限保护：实测已超出【实际生效限位】、且误差仍指向限位方向 -> 本拍不出力
     * 保证即使因外部原因压到限位，电机也不会继续加力去顶限位结构。
     * 抹 0 的写法会让 `steer.err`（遥测/调试）显示成假值 —— "真的到位了"和"被死区改成了 0"
     * 在 CSV 里长得一模一样。改成"三项各自不参与"，err 始终是真值。
     */
    {
        float lim = steer_ctrl_eff_limit();

        block = (((steer.meas_delta >  lim) && (err > 0.0f)) ||
                 ((steer.meas_delta < -lim) && (err < 0.0f))) ? 1u : 0u;
    }

    /* 参数表是**直接写 `steer.ki`** 的（`motor_link.h` 的 X-Macro 把 `&steer.ki` 绑进滑条），
     * 根本不经过 `steer_ctrl_set_angle_i()` ⇒ 这里只能靠**边沿检测**补上无扰切换。
     */
    if (steer.ki != steer.ki_prev)
    {
        steer_i_rescale(steer.ki_prev, steer.ki, steer.i_max);
    }

    /* 我们之前是"三态滞环 + 死区内冻结积分"：`|err|>brk(0.20)` 给满前馈、
     * `<db(0.10)` 撤前馈、中间保持；积分只在 `db<|err|<izone` 累加。
     * 为了治"死区边界上 u0 的 1500 duty 台阶"，加了滞环 = **一个门限干了两件事**
     * （既是 Kp/U0 的门限，又是积分的下界），调"更准"就必须调小 db，
     * 而 db 一小台阶又跑回静止点上 —— 参数互相绑死。
     * ---- 他这一版的做法 ---------------------------------------------------------
     * · **死区只撤 Kp 与 U0，积分照常累加**（下界是另一个更小的门限 i_db）
     * 而不是像我们那样"进死区那一刻的积分值当预载，误差停在 ±db 内"。
     * 不需要额外的抗饱和回滚。
     * · err 永远真值；三项各自"这一拍参不参与"。
     * 参数 `steer_brk`(ID61) 从这一版起**不再参与控制**（留作对照/回退）。
     */
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
        /* （舵机要一直克服摩擦，稳态误差就在那儿），于是**每穿一次 db 就翻一次**
         * ⇒ 输出在 `I` 与 `u0+I` 之间跳 u0（实测 1850）+，25 Hz，听得到"嗡嗡"。
         * err=+0.100 -> u=-1407 ｜ err=+0.160 -> u=+518（差 1925 ≈ u0）
         * err=+0.060 -> u=-1406 ｜ err=+0.180 -> u=+542
         * ---- 滞环只管"给不给"，方向永远跟 err --------------------------------------
         * `|err| <= db`   -> 门关
         * 中间（brk/db 之间）-> **保持上一拍的开/关** ⇒ 慢变时门"开住"，输出连续、不再逐拍翻
         */
        {
            int8 s = (err >= 0.0f) ? 1 : -1;        /* 本拍方向，0 归"+"侧 */

            if (ae > steer.brk_deg)
            {
                steer.u_dir = s;
            }
            else if (ae <= steer.deadband_deg)
            {
                steer.u_dir = 0;                    /* 关门（含 err 恰为 0） */
            }
            else if (0 != steer.u_dir)
            {
                steer.u_dir = s;                    /* 带内：门保持开，但方向不沿用上一拍 */
            }
            /* 带内且门是关的 -> 继续关着（这就是滞环的作用） */
        }

        u_p = (ae < steer.deadband_deg) ? 0.0f : (steer.kp * err);

        /* 前馈：由上面的**滞环**决定给不给，不再看 db 单门限 */
        u_ff = (0 != steer.u_dir) ? (steer.u0 * (float)steer.u_dir) : 0.0f;

        /* 积分累加门限：比死区小一个量级，只挡纯噪声（编码器 1 计数 ≈ 0.02°，机械振动就在这个量级） */
        i_add = (ae < steer.i_db_deg) ? 0.0f : err;
    }

    /* 8) 积分项：**超带清零**（不是冻结）+ 在"积分输出"空间限幅
     * · 否则累加 `i_add`；
     * · 限幅用 u_i 判（`i_max` 就是"积分最多出多大劲"），撞限后反算 `i_sum` 保持一致。
     * 所以 ki=0 时下面这条分支顺带把 i_sum 也清掉（等价于"关掉积分"）。
     */
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

    steer.u_ff = u_ff;
    u          = u_p + u_i + u_ff;

    /* 9) 顶死 / 无进展告警（判据与理由见 steer_ctrl.h 的 STEER_SAT_*）
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

    /* 现在这条路径不存在了，留着反而是**两套抗饱和互相打架**（会掩盖参数错）。 如果以后要回到带滞环的版本，`i_prev` 回滚那段在 git 历史里（提交信息会写明）。 */
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

        /* 否则遥控恒写 steer_en=1、永远没有边沿 ⇒ 方向盘打死一次就彻底死掉。见 enc_check()。
         * 顺序很重要: 必须在"取当前实测角"之后清, 否则目标会被钉在故障前的旧值上。
         */
        steer_ctrl_clear_fault();

        /* （标定时靠 `内环关 -> 内环开` 给每段一个干净的起点；想测"稳态保持"就别关。） */
        steer.i_sum = 0.0f;
        steer.u_dir = 0;
    }

    steer.angle_enable = en ? 1 : 0;

    /* 原因：`steer_ctrl_update()` 第 0 步每拍做"参数 -> 内部状态"的同步：
     * en_cmd = (steer_en_cmd > 0.5)
     * if (en_cmd != steer.angle_enable) steer_ctrl_set_angle_enable(en_cmd);
     * 如果这里只置 `angle_enable`、而 `steer_en_cmd` 还是 0（默认值），
     * 那么**下一拍 5 ms** 就会被按回 0 —— 现场看到的就是"使能了但舵机一动不动"。
     * （工程 11 的航向外环第一次被真正打开时才暴露：以前一直是上位机在写这个参数。）
     * ⇒ 两个量从此永远同源：谁调这个函数，参数表上看到的 ID23 也跟着变。
     */
    steer_en_cmd = en ? 1.0f : 0.0f;

    if (0 == steer.angle_enable)
    {
        steer.head_enable = 0;              /* 关内环连带关外环, 防外环空转 */
    }
}

/* =========================== 主动保护接口 =========================== */
uint8 steer_ctrl_fault (void)
{
    /* / bit2 = **瞬态**"本拍编码器读数被防抖拒掉"(不锁存、不钉输出，是最早的预警)。
     * 车端遥测把它 <<1 变成 steer_status 的 bit1/bit2/bit3 —— 上位机的解读按 0x01/0x02/0x04 走，别改顺序。
     */
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
    steer.sat_warn    = 0;                  /* 告警本来就会自清, 复位时一起清干净 */
    steer.sat_warn_ms = 0;
    /* 人工清一次故障就把它抹掉的话，"编码器真坏了 -> 人狂按使能 -> 反复复活打死"就绕过了限次，
     * 限次就白做了。窗口自己会走完（STEER_ENC_AR_WIN_MS）。
     * `recover_cnt` 也不清 —— 它是"到底发生过没有"的累计证据。
     */
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
        /* 开外环必须先有内环托底。走正规入口而不是直接置位：
         * ① 拿到 0→1 上升沿的"目标=实测角 / 清保护 / 清积分"；
         * ② 顺带把参数量 `steer_en_cmd` 同步成 1（否则 update() 第 0 步下一拍就把它关回去）。
         */
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

/* 固定在头文件里一个常数，前视一变这个环就变成"另一个环"了（工程 10 的 head_kp=15 就是这个问题）。 */
void steer_ctrl_set_head_kp (float kp)
{
    if (kp < 0.0f) { kp = 0.0f; }
    steer.head_kp = kp;
}

void steer_ctrl_set_angle_i (float ki, float i_max)
{
    if (ki < 0.0f)      { ki = 0.0f; }
    if (i_max < 0.0f)   { i_max = 0.0f; }

    /* 与 update() 的 6b) 是**同一个原语**（见 steer_i_rescale 的注释）。 */
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
    /* 不清状态: 行驶中调它不该让方向盘抖一拍 */
    steer.u_max = outmax;
}

/* =========================== 取值接口(给遥测/调试) =========================== */
float steer_ctrl_get_target_delta (void) { return steer.target_delta; }
float steer_ctrl_get_meas_delta   (void) { return steer.meas_delta; }
int16 steer_ctrl_get_output       (void) { return steer.output_duty; }
float steer_ctrl_get_target_yaw   (void) { return steer.target_yaw; }
