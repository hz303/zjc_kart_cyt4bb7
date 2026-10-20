/*********************************************************************************************************************
 * 遥控（HotRc 枪式遥控器 / 逐飞 UART 接收机 + SBUS）实现
 * 通道分配、标定值、急停、接管状态机、与 WiFi 的授权关系，全部见 remote_ctrl.h
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "remote_ctrl.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "motor_drv.h"
#include "ins_odom.h"           /* ins_m_per_rev：rps <-> m/s */
#include "route_pb.h"           /* pb_l：与回放共用同一个有效轴距 */
#include "proto.h"              /* ★ 驱动令牌 / 急停的唯一入口 */


/*===================================================================================================================
 *  对外数据
 *=================================================================================================================*/
volatile uint16  RC_Raw_Ch1     = 0;
volatile uint16  RC_Raw_Ch2     = 0;
volatile uint16  RC_Raw_Ch3     = 0;
volatile uint16  RC_Raw_Ch4     = 0;
volatile uint16  RC_Raw_Ch5     = 0;
volatile uint16  RC_Raw_Ch6     = 0;
volatile float   RC_Steer_Deg   = 0.0f;
volatile float   RC_Steer_Norm  = 0.0f;
volatile uint8   RC_Ch3_Pct     = 100u;
volatile uint8   RC_Hdg_Mode    = 0u;
volatile float   RC_Hdg_Step    = 1.0f;
volatile float   RC_Hdg_Off     = 0.0f;
volatile float   RC_Cmd_Yaw     = 0.0f;
volatile float   RC_Throttle    = 0.0f;
volatile float   RC_Scale       = RC_SCALE_MAX;
volatile uint8   RC_Estop       = 0;
volatile uint8   RC_Arm         = 0;
volatile uint8   RC_Online      = 0;
volatile uint8   RC_Engage      = RC_ENGAGE_OFF;
volatile uint32  RC_Frames      = 0;
volatile float   RC_Target_Rps  = 0.0f;

volatile float   rc_arm_en      = 1.0f;                     /* 1 = 允许遥控接管 */
volatile float   rc_thr_max     = RC_THR_MAX_DEFAULT;       /* 满杆目标 rps     */

/*===================================================================================================================
 *  内部工具
 *=================================================================================================================*/

/*----------------------------------------------------------
 * 原始值 -> 归一化(-1.0 ~ +1.0)
 * 中位两侧量程不对称（方向盘 776:824），所以两侧分别归一化，
 * 这样两端都能吃满 ±1.0，不会有一边提前截断。**照抄队友的算法**。
 *----------------------------------------------------------*/
static float rc_norm (uint16 raw, int mid, int lo, int hi)
{
    float t;

    if ((int)raw >= mid)
    {
        if (hi <= mid) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(hi - mid);
    }
    else
    {
        if (mid <= lo) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(mid - lo);
    }

    if (t >  1.0f) { t =  1.0f; }
    if (t < -1.0f) { t = -1.0f; }

    return t;
}

/* 两态开关：以中位(992)为界。activeHigh = 1 时"高档 = 1" */
static uint8 rc_pick2 (uint16 v, uint8 active_high)
{
    uint8 hi = ((int)v > RC_SW_VAL_MID) ? 1u : 0u;

    return (0 != active_high) ? hi : (uint8)(1u - hi);
}

static float rc_fabs (float v)
{
    return (v < 0.0f) ? -v : v;
}

/*----------------------------------------------------------
 * CH4 油门行程旋钮 -> 速度缩放
 *   旋钮低 -> RC_SCALE_MIN，中 -> 两者中点，高 -> RC_SCALE_MAX。
 *   用 rc_norm 拿到 -1~+1 再线性映射到 [MIN, MAX]，两端对称。
 *   ⚠ CH3（方向行程）**用户明确说先不做**，所以这里不读它；
 *     但遥测/打印里预留了位置，将来要做时在这里加一个 rc_steer_travel() 即可。
 *----------------------------------------------------------*/
static float rc_thr_travel_scale (void)
{
    float n = rc_norm(RC_Raw_Ch4, RC_SW_VAL_MID, RC_SW_VAL_LOW, RC_SW_VAL_HIGH);
    float s = RC_SCALE_MIN + ((n + 1.0f) * 0.5f) * (RC_SCALE_MAX - RC_SCALE_MIN);

    if (s < RC_SCALE_MIN) { s = RC_SCALE_MIN; }
    if (s > RC_SCALE_MAX) { s = RC_SCALE_MAX; }

    return s;
}

/*----------------------------------------------------------
 * 按步长量化（0.2 / 0.5 / 1.0 deg）
 *   量化是为了让"同一个摇杆位置"每次都给出**完全一样**的目标角，
 *   否则手一抖 20.003 与 20.000 会来回跳，看起来像外环在抖。
 *----------------------------------------------------------*/
static float rc_quantize (float v, float step)
{
    if (step < 1.0e-4f) { return v; }
    return ((float)(int)((v / step) + ((v >= 0.0f) ? 0.5f : -0.5f))) * step;
}

/*----------------------------------------------------------
 * 转向指令：两种模式二选一（见 remote_ctrl.h 三·五）
 *   RC_Hdg_Mode = 1 ⇒ CH1 给**目标航向**（基准 + 偏移）
 *   RC_Hdg_Mode = 0 ⇒ CH1 给**舵机转角**（原行为）
 *
 *   基准 s_base_yaw：进模式取一次；之后**每次松手（CH1 回中）再取一次**
 *   ⇒ 推一下走一段、松手停在新的航向上，可以连做台阶。
 *----------------------------------------------------------*/
static float s_base_yaw = 0.0f;
static uint8 s_ht_on    = 0u;
static float s_prev_n   = 0.0f;

static void rc_leave_heading (void)
{
    if (0u != s_ht_on)
    {
        s_ht_on = 0u;
        steer_ctrl_set_head_enable(0u);                            /* 先关外环 */
        steer_ctrl_set_target_delta(steer_ctrl_get_meas_delta());  /* 目标对齐实测，别甩头 */
        RC_Hdg_Off = 0.0f;
        printf("[RC] heading mode OFF: CH1 -> steer angle\r\n");
    }
}

static void rc_apply_steer (void)
{
    if (0u != RC_Hdg_Mode)
    {
        if (0u == s_ht_on)
        {
            s_ht_on    = 1u;
            s_base_yaw = steer_yaw_deg;                 /* 基准 = 进模式时的航向 */
            s_prev_n   = 0.0f;
            printf("[RC] heading mode ON: CH3=%u%% step=%u.%u deg, CH1 = +/-%u deg\r\n",
                   (unsigned)RC_Ch3_Pct,
                   (unsigned)((int)(RC_Hdg_Step * 10.0f) / 10),
                   (unsigned)((int)(RC_Hdg_Step * 10.0f) % 10),
                   (unsigned)(int)RC_HDG_MAX_DEG);
        }

        /* 松手那一拍：把当前航向记成新基准（推一次 = 一个台阶） */
        if ((0.0f == RC_Steer_Norm) && (0.0f != s_prev_n))
        {
            s_base_yaw = steer_yaw_deg;
        }

        /* ⚠⚠ 符号：CH1 右推 = RC_Steer_Norm > 0 = **右转**（δ > 0）；
           而本工程 `yaw` 正 = **左转**（x += -sin(yaw)*ds）。
           ⇒ 右推必须让目标航向**变小**，所以取负号。写正号 = "推右往左拐"。 */
        RC_Hdg_Off = rc_quantize(-RC_Steer_Norm * RC_HDG_MAX_DEG, RC_Hdg_Step);

        if (RC_Hdg_Off >  RC_HDG_MAX_DEG) { RC_Hdg_Off =  RC_HDG_MAX_DEG; }
        if (RC_Hdg_Off < -RC_HDG_MAX_DEG) { RC_Hdg_Off = -RC_HDG_MAX_DEG; }

        RC_Cmd_Yaw = s_base_yaw + RC_Hdg_Off;

        /* 增益每拍写一次：route_pb 会按 2L/ld 改它，不写就会被带跑 */
        steer_ctrl_set_head_kp(RC_HDG_KP);
        steer_ctrl_set_head_enable(1u);
        steer_ctrl_set_target_yaw(RC_Cmd_Yaw);

        s_prev_n = RC_Steer_Norm;
    }
    else
    {
        rc_leave_heading();
        steer.target_delta = RC_Steer_Deg;
    }
}

/*----------------------------------------------------------
 * 油门归一化值 -> 后轮目标 rps（带死区 + 速度缩放 + 上限夹紧）
 *   队友是 ±1.0 线性映射到固定常数；本工程多了"CH4 缩放"与 spd_target_max 夹紧。
 *----------------------------------------------------------*/
static float rc_throttle_to_target (void)
{
    float t;
    float lim;

    if (rc_fabs(RC_Throttle) <= 0.0001f) { return 0.0f; }

    t   = RC_Throttle * rc_thr_max * RC_Scale;
    lim = spd_target_max;

    if (lim > 0.0f)
    {
        if (t >  lim) { t =  lim; }
        if (t < -lim) { t = -lim; }
    }

    return t;
}

/*===================================================================================================================
 *  接口实现
 *=================================================================================================================*/

/*----------------------------------------------------------
 * 初始化：清状态 + 起接收机
 * 接收机：UART_4 = SCB2，RX = P14.0，SBUS 100 kbps，帧间隔计时用 TC_TIME2_CH2
 *        （以上引脚/通道都在 zf_device_uart_receiver.h 里，逐飞库自带，不在这里重定义）
 * ⚠ 必须在 5ms PIT 起来之前调用（接收机会注册帧间隔计数器）
 *----------------------------------------------------------*/
void remote_ctrl_init (void)
{
    /* 先整流解除，避免初始化期间的残留状态把电机放开 */
    remote_ctrl_release();

    RC_Raw_Ch1 = 0;
    RC_Raw_Ch2 = 0;
    RC_Raw_Ch4 = 0;
    RC_Raw_Ch5 = 0;
    RC_Raw_Ch6 = 0;
    RC_Scale   = RC_SCALE_MAX;
    RC_Estop   = 0;
    RC_Arm     = 0;
    RC_Online  = 0;
    RC_Frames  = 0;

    uart_receiver_init();

    /* 顺便清一次接收机快照，避免上电第一圈读到栈上的旧值 */
    uart_receiver.state      = 0;
    uart_receiver.finsh_flag = 0;
    {
        uint8 i;
        for (i = 0; i < UART_RECEVIER_CHANNEL_NUM; i ++)
        {
            uart_receiver.channel[i] = 0;
        }
    }
}

/*----------------------------------------------------------
 * 从接收机快照换算一次（本模块不主动碰驱动，只读 uart_receiver）
 *----------------------------------------------------------*/
void remote_ctrl_update (void)
{
    int thr;

    RC_Raw_Ch1 = uart_receiver.channel[RC_CH_STEER];
    RC_Raw_Ch2 = uart_receiver.channel[RC_CH_THROTTLE];
    RC_Raw_Ch3 = uart_receiver.channel[RC_CH_STEER_TRAVEL];
    RC_Raw_Ch4 = uart_receiver.channel[RC_CH_THR_TRAVEL];
    RC_Raw_Ch5 = uart_receiver.channel[RC_CH_ESTOP];
    RC_Raw_Ch6 = uart_receiver.channel[RC_CH_ARM];
    RC_Online  = uart_receiver.state;

    /* 收帧计数：FRAMES 一直不涨 => 接收机没在发 / 帧格式对不上 / 线接错 */
    if (0 != uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;
        RC_Frames ++;
    }

    /* 两个二值开关（极性由 RC_*_ACTIVE_HIGH 定） */
    RC_Estop = rc_pick2(RC_Raw_Ch5, RC_ESTOP_ACTIVE_HIGH);
    RC_Arm   = rc_pick2(RC_Raw_Ch6, RC_ARM_ACTIVE_HIGH);

    /* 方向盘：**先过中位死区**（±RC_STEER_DEADBAND），再归一化到 -1~+1 乘【实际生效限位】。
       ⚠ 用 steer_ctrl_eff_limit() 而不是 steer.limit_deg：那是**全系统唯一出口**，
         已经保证"参数怎么改都越不过 ±27"（见 steer_abs.h / steer_ctrl.h）。 */
    {
        int d = (int)RC_Raw_Ch1 - RC_STEER_MID;

        if ((d <= RC_STEER_DEADBAND) && (d >= -RC_STEER_DEADBAND))
        {
            /* 死区内 = 方向盘回中：不跟着手上/电位器的零点漂移走。
               （没有这一段时，静止时方向盘会被 ±1~2 个计数的抖动顶到零点几度） */
            RC_Steer_Deg  = 0.0f;
            RC_Steer_Norm = 0.0f;
        }
        else
        {
            RC_Steer_Norm = (float)RC_STEER_DIR_SIGN
                          * rc_norm(RC_Raw_Ch1, RC_STEER_MID, RC_STEER_MIN, RC_STEER_MAX);
            RC_Steer_Deg  = RC_Steer_Norm * steer_ctrl_eff_limit();
        }
    }

    /* 油门：同样 ±RC_THR_DEADBAND 死区，把静止抖动压成 0（队友实测中位在 988/990/992 之间跳） */
    thr = (int)RC_Raw_Ch2 - RC_THR_MID;
    if ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND))
    {
        RC_Throttle = 0.0f;
    }
    else
    {
        RC_Throttle = rc_norm(RC_Raw_Ch2, RC_THR_MID, RC_THR_MIN, RC_THR_MAX);
    }

    /* CH4 旋钮 -> 速度缩放 */
    RC_Scale = rc_thr_travel_scale();

    /* CH3 旋钮 -> 航向微调档位 / 模式（见 remote_ctrl.h 三·五）
       ⚠ 没接 CH3（原始值≈0）时当成"舵机模式"：否则默认 0% 会把 CH1 偷偷变成航向控制，
         现场完全看不出"方向盘为什么不跟手"。 */
    {
        float pct;

        if (RC_Raw_Ch3 < RC_CH3_VALID_MIN)
        {
            pct = 100.0f;                                   /* 没信号 -> 舵机模式 */
        }
        else
        {
            pct = (rc_norm(RC_Raw_Ch3, RC_SW_VAL_MID, RC_SW_VAL_LOW, RC_SW_VAL_HIGH) + 1.0f) * 50.0f;
        }

        if (pct <   0.0f) { pct =   0.0f; }
        if (pct > 100.0f) { pct = 100.0f; }

        RC_Ch3_Pct = (uint8)pct;

        if (pct <= RC_HDG_CH3_PCT_MAX)
        {
            RC_Hdg_Mode = 1u;
            RC_Hdg_Step = (pct < 15.0f) ? RC_HDG_STEP_FINE
                        : ((pct < 25.0f) ? RC_HDG_STEP_MED : RC_HDG_STEP_COARSE);
        }
        else
        {
            RC_Hdg_Mode = 0u;
            RC_Hdg_Step = RC_HDG_STEP_COARSE;
        }
    }
}

/*----------------------------------------------------------
 * 接管状态机
 *   优先级（从高到低）：
 *     1) 接收机失控（ST=0，含"从来没收到过帧"）  -> FAILSAFE（锁存）
 *     2) 急停生效 / 急停未回位                  -> ESTOP（锁存）
 *     3) 使能开关收回 / 上位机禁了遥控            -> OFF
 *     4) 合闸（CH6 合闸 + 在线 + 授权）           -> OFF 先 STANDBY；油门回中才升 ON
 *   ⚠ FAILSAFE 与 ESTOP 都是**锁存**的，且都要求"先把开关收回去"才有机会恢复 ——
 *     失控/急停过就一定是哪里不对，不该让它自动恢复。
 *----------------------------------------------------------*/
static void rc_state_machine (void)
{
    uint8 armed;

    /* 1) 接收机报告失控（或从来没收到过帧 -> state 恒 0）*/
    if (0 == RC_Online)
    {
        RC_Engage = RC_ENGAGE_FAILSAFE;
        return;
    }

    /* 2) 急停：生效就锁存；已经锁存时只有"CH5 回到不生效位置"才放行 */
    if (0 != RC_Estop)
    {
        RC_Engage = RC_ENGAGE_ESTOP;
        return;
    }

    /* 急停开关已回位但状态还停在 ESTOP -> 降回 OFF，等重新合闸 */
    if (RC_ENGAGE_ESTOP == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    armed = ((rc_arm_en > 0.5f) && (0 != RC_Arm)) ? 1u : 0u;

    /* 3) 使能开关收回 / 上位机禁了遥控：无条件放开，并清 FAILSAFE 锁存 */
    if (0 == armed)
    {
        RC_Engage = RC_ENGAGE_OFF;
        return;
    }

    /* 4) 合闸中，但还处于 FAILSAFE 锁存 -> 保持挂起，等上面的 3) 放开一次 */
    if (RC_ENGAGE_FAILSAFE == RC_Engage)
    {
        return;
    }

    /* 5) 刚合闸：先进 STANDBY（双环使能、目标 0），让人看清楚方向盘回中 */
    if (RC_ENGAGE_OFF == RC_Engage)
    {
        RC_Engage = RC_ENGAGE_STANDBY;
        return;
    }

    /* 6) STANDBY -> ON：必须油门在中位（防止合闸瞬间全油门窜出去） */
    if ((RC_ENGAGE_STANDBY == RC_Engage) && (rc_fabs(RC_Throttle) <= RC_ARM_THR_NEUTRAL))
    {
        RC_Engage = RC_ENGAGE_ON;
    }
    /* ON 保持 */
}

/*----------------------------------------------------------
 * 跑状态机 + 写目标 + 推急停/本地接管档位
 *
 * ★★ 授权关系（见 remote_ctrl.h 的长注释）：
 *     · 遥控 ON          -> spd_safety_set_local(1)：把"上位机在场"那三道门换成"本地授权"
 *     · 退出 / 失控 / 急停 -> spd_safety_set_local(0)：完整五道门原样恢复
 *   => 两条路合起来 = 「WiFi 在场 OR 遥控在场」，两个都不在才真的不动。
 *   [!] 心跳门与限幅门**不参与**替换，永远生效。
 *
 * ★★ 目标量（spd_target_l / spd_target_r / steer.target_delta）**只在"遥控真的在开车"时写**：
 *     · ON                 每拍写 —— 遥控就是当前驾驶员（全工程唯一一处每拍写目标的地方）；
 *     · 进 STANDBY 那一拍、以及从 ON/STANDBY **退出**那一拍 —— 写 0 一次
 *       （"停住 + 方向盘回中"的语义，管住"摇杆还推着但接收机突然没了"）；
 *     · 其余任何时刻 —— **一个字都不许写**。
 *
 *   [!][!] 这条是硬约束：这三个量**上位机也在写**（参数 target_l / target_r / steer_tgt
 *        就是按地址直接下发到这三个变量的）。遥控模块每 5ms 无条件压 0，等于把上位机
 *        那三个滑条全废掉，而且**不报任何错**：
 *          2026-10-13 现场症状 —— 上位机把"左轮转速"拖到 2.85，车端遥测 target_l 恒 0。
 *          根因：接收机没插时 RC_Online 恒 0 => 状态机恒为 FAILSAFE => 旧代码的
 *          ESTOP/FAILSAFE/OFF 三个分支每拍 spd_target_l = 0，刚写进去的值下一拍就被清掉。
 *        语义上也要分清：**"遥控没在开车" != "车必须停"**。车要停有急停门（硬钉输出）
 *        和占空比限幅门，不该靠"霸占目标量"来实现。
 *----------------------------------------------------------*/
void remote_ctrl_apply (void)
{
    static uint8 prev_engage = 0xFFu;       /* 上一拍状态(0xFF = 本函数还没跑过) */
    uint8        held;                      /* 上一拍是不是"遥控持有控制权" */

    rc_state_machine();

    /* 急停是**全局**的：不管此刻谁在开车，先把两个环硬停掉。
       放在 switch 之前 -> 即使 ESTOP 之后状态机立刻降回 OFF，本拍也仍然先把急停推住。
       ★ 2026-10-20：改走 `proto_estop()` —— 它除了推两个环，还会**剥夺驱动令牌**：
         急停期间谁都不许继续持有方向盘（笔记 §7.6：急停不属于权限，但令牌必须一起收掉）。 */
    proto_estop(PROTO_SRC_RC, RC_Estop);


    held = ((RC_ENGAGE_ON == prev_engage) || (RC_ENGAGE_STANDBY == prev_engage)) ? 1u : 0u;

    switch (RC_Engage)
    {
        case RC_ENGAGE_ON:
            /* ★ 驱动令牌：遥控 ON = 抢占（人在这儿、看得见车，优先级最高）。
               每圈 acquire 一次就是续租；抢占成功时 proto.c 会给上位机发一条 EVENT。 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 遥控接管：双环使能 + 遥控给目标 */
            spd_safety_set_local(1);
            steer_en_cmd = 1.0f;
            RC_Target_Rps = rc_throttle_to_target();

            /* ★ CH3 ≤ 50% 时 CH1 改为给**目标航向**（±20 度），否则还是舵机角。
               见 rc_apply_steer() 与 remote_ctrl.h 三·五。 */
            rc_apply_steer();

            /* ★ 电子差速（与回放同一条律，见 `工程10-控制层-航向追随与电子差速.md`）
                  κ = tan(实测δ)/L，v_l = v(1−κW/2)、v_r = v(1+κW/2)
               ⚠⚠ **单位**：`spd_target_*` 是 rps，而 `spd_diff_alloc` 吃 m/s
                  ⇒ 进来乘 `m_per_rev`、出去除回去。漏一边就是 1.8 倍误差（cpr 不是 1）。
               ⚠ 用**实测**舵角（不是目标角）：目标角会让后轮在舵机到位之前就提前差速。
               ⚠ 这句话**不改变授权语义**：仍然只在"遥控真的在开车"时写目标；
                 STANDBY/OFF 的"不写"分支原样保留（见本文件前面的长注释）。 */
            {
                float mpr   = (ins_m_per_rev > 1e-4f) ? ins_m_per_rev : 0.8193f;
                float v_mps = RC_Target_Rps * mpr;
                float vl    = v_mps;
                float vr    = v_mps;

                spd_diff_alloc(v_mps, pb_l, steer_ctrl_get_meas_delta(), &vl, &vr);

                spd_target_l = vl / mpr;
                spd_target_r = vr / mpr;
            }
            break;

        case RC_ENGAGE_STANDBY:
            /* ★ 合闸就算"我在看着这台车"：STANDBY 也占住令牌，
               免得上位机在等油门回中的这段时间把目标写进来（两个人抢方向盘）。 */
            (void)proto_drv_acquire(PROTO_SRC_RC);

            /* 待命：双环使能（方向盘会主动回中），目标**只在入口那一拍**压 0。
               ⚠ 2026-10-20 起 STANDBY **不再**"给上位机让路"：CH6 已合闸 ⇒ 遥控就是驾驶员，
                 驱动令牌归遥控，上位机的 P_CMD 写入会被 NAK(busy)。要交还控制权，
                 把 CH6 收回（-> OFF，本函数会释放令牌）即可。 */

            RC_Target_Rps = 0.0f;
            steer_en_cmd = 1.0f;
            rc_leave_heading();                 /* 待命 = 方向盘回中，不保持航向 */
            if (RC_ENGAGE_STANDBY != prev_engage)
            {
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;

        default:
            /* OFF / ESTOP / FAILSAFE：
               1) 本地档位放掉 —— 控制权交还上位机（急停那一路已经被硬钉住）；
               2) 若是从"遥控持有控制权"退出来的，在**这一拍**把目标压 0
                  （摇杆还推着但接收机没了，必须停下来），同时转向失能（方向盘自由）；
               3) 之后**不再碰**这三个量 —— 否则上位机的滑条就全废了。 */
            spd_safety_set_local(0);
            (void)proto_drv_release(PROTO_SRC_RC);      /* 退出即释放驱动权，上位机/菜单可接管 */
            RC_Target_Rps = 0.0f;
            if (0 != held)
            {
                rc_leave_heading();
                steer_en_cmd = 0.0f;
                steer.target_delta = 0.0f;
                spd_target_l = 0.0f;
                spd_target_r = 0.0f;
            }
            break;
    }

    prev_engage = RC_Engage;
}

/*----------------------------------------------------------
 * 立刻解除接管（上电 / 停机路径调用）
 * ⚠ 会把**急停也放开** —— 这是唯一能放开急停的地方（正常运行时只有 CH5 回位能放）。
 *   上电路径先 release 一次是刻意的：保证上电初态是"全部放开"。
 *----------------------------------------------------------*/
void remote_ctrl_release (void)
{
    RC_Engage     = RC_ENGAGE_OFF;
    RC_Target_Rps = 0.0f;

    proto_estop(PROTO_SRC_RC, 0u);      /* 唯一能清急停的地方：上电 / 停机路径 */
    proto_drv_release(PROTO_SRC_RC);    /* 上电初态 = 没有任何设备持有驱动权 */
    spd_safety_set_local(0);

    steer_en_cmd = 0.0f;
    steer.target_delta = 0.0f;
    spd_target_l = 0.0f;
    spd_target_r = 0.0f;
}

uint8 remote_ctrl_estop (void)
{
    return RC_Estop;
}
