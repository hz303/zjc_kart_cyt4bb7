/*********************************************************************************************************************
* CYT4BB Opensource Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
 * 转向位置环 + 双后轮速度环；遥控（逐飞 UART 接收机 SBUS）独立于 WiFi，
 * 插不插 WiFi 都能用发射机完整操控（见 remote_ctrl.h）。
 * 上位机只下发"目标转速 rps"，闭环完全在车端跑，无开环占空比通道。
*
 * ---- 硬件连接 -----------------------------------------------------------------------
*   左后轮:  PWM -> P09_0 (TCPWM_CH24)    DIR -> P09_1
*   右后轮:  PWM -> P10_2 (TCPWM_CH30)    DIR -> P10_3
*   转向  :  PWM -> P05_0 (TCPWM_CH09)    DIR -> P05_1
 *   (引脚一变, minv_l / minv_r / minv_s 三个极性必须重新核, 判据见 motor_drv.h 末尾)
*   左后轮编码器: PULSE -> P17_3 (TC_CH58 CH1)   DIR -> P17_4 (TC_CH58 CH2)  [脉冲+方向]
*   右后轮编码器: PULSE -> P19_2 (TC_CH27 CH1)   DIR -> P19_3 (TC_CH27 CH2)  [脉冲+方向]
*   转向绝对角编码器: SPI_4(SCB5)  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3
*   WiFi-SPI 模块: 见 motor_link.h（SPI_0 + INT P02_4 + RST P23_0）
*   遥控接收机  : UART_4 = SCB2，RX = P14_0（接接收机的 TX），TX = P14_1（占位，悬空）
*                 SBUS 100 kbps，6 通道；帧间隔计时占 TC_TIME2_CH2。见 zf_device_uart_receiver.h
*
 * ---- 三个必须先做的台架标定（顺序不能颠倒）--------------------------------------
 *   第零步 硬件极性（**上电前一定要做, 这是唯一会烧东西的一步**）:
*           车架空 -> 上位机把「转向|内环使能」打开 -> 先只给 steer_u0 看方向盘能不能被推动,
*           再给一个很小的目标角(比如 +3 度), 看方向盘是"朝目标去"还是"背着目标跑"。
*           背着跑 = mot_inv_s 给反了 => **立刻停手**, 把「转向|电机极性」改成 0 再试。
 *           (正反馈会一路顶住机械限位堵转, 烧毁 DRV8701。)
*           后轮同理但没风险: 给小的正目标看轮子是不是朝"车前进"方向转, 反了改 minv_l/minv_r。
*   第零步b 遥控: 先只用 Debug 看 CH1/CH2 是否在中位 992 附近, 再拨 CH5 看 eng 是否 0->1->2。
*           详细判读见 motor_print_status() 里"RC 一行"那段注释。

*   第一步 电机方向: 关掉所有保护（或者就在 5.speed_loop 里做），给一个小的正目标，
*                    确认电机朝"车前进"方向转；反了就把 minv_l/minv_r 之一取反。
*   第二步 测速正负: 判据是"正命令 => 该侧 v 为正"。反了改 inv_l/inv_r。
*   第三步 每转计数: 清零 pos -> 单方向手转【电机轴】整 10 圈 -> cpr = pos 增量 / 10。
*                    本车实测为 1858（1024 线 x 49/27 齿轮）。
*   以上三个参数都能在上位机滑条上在线改，不用重烧。
********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "motor_link.h"
#include "motor_drv.h"
#include "spd_enc.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "remote_ctrl.h"
#include "ins_imu.h"
#include "ins_odom.h"

/* ---- 工程 10 新增：屏幕菜单 + 12 段跑班 ---- */
#include "flash_map.h"
#include "app_params.h"
#include "bench_log.h"
#include "spd_bench.h"
#include "ui_core.h"
#include "ui_menu.h"
#include "route_app.h"          /* 路表(P1): 应用层 */
#include "route_pb.h"            /* 路表(P2): Pure Pursuit 回放 */

/* 内部状态 */
static uint32   g_print_ms = 0;        /* 上次打印状态的时刻 */
static uint8    g_zero_last    = 0;   /* 惯导|位姿归零 命令的边沿记忆(0->1 只触发一次) */
static uint32   g_imu_retry_ms  = 0;   /* IMU 掉线后的重试节拍(1Hz) */

/* 遥测通道填充（顺序即协议，见 motor_link.h 第四节） */
static void motor_fill_telemetry (void)
{
    uint32 age;

    /* 落地标定要长时间高密度采数据，34/37 通道一帧把带宽吃满；16 通道一帧只有 85 字节，
     * 同样的链路能把遥测周期压到 5~10 ms。
     * [!] 这里**直接 return**：标定档的通道号(CT_*)与全量档(CH_*)完全不是一回事，
     * 绝不能"填一半再叠一半"，否则上位机看到的是两份数据混在一起。
     * [!] 切档时上位机的"档案"下拉框也要跟着切到「标定·精简」，否则通道名错位。
     */
    if (spd_tel_sel > 0.5f)
    {
        age = motor_link_rx_age_ms();
        if (age > 60000u)
        {
            age = 60000u;
        }

        motor_link_ch_set(CT_TARGET_L,    spd_target_l);
        motor_link_ch_set(CT_V_L,         enc_l.rps);
        motor_link_ch_set(CT_U_L,         ctl_l.u * 100.0f);
        motor_link_ch_set(CT_TARGET_R,    spd_target_r);
        motor_link_ch_set(CT_V_R,         enc_r.rps);
        motor_link_ch_set(CT_U_R,         ctl_r.u * 100.0f);
        /* 会把"斜坡造成的滞后"算到机械/增益头上。见 motor_link.h 的 CH_STEER_TARGET 注解。 */
        motor_link_ch_set(CT_STEER_TGT,   steer.applied_delta);
        motor_link_ch_set(CT_STEER_DELTA, steer.meas_delta);
        motor_link_ch_set(CT_STEER_U,     (float)steer.output_duty);
        motor_link_ch_set(CT_ONLINE,      (float)motor_link_online());
        motor_link_ch_set(CT_ENABLE,      (float)motor_link_enable_bits());
        motor_link_ch_set(CT_SAFETY,      (float)spd_safety_flags());
        motor_link_ch_set(CT_STEER_EN,    (float)steer.angle_enable);
        motor_link_ch_set(CT_STEER_ST,
                          (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));
        motor_link_ch_set(CT_OL_EN,       (spd_ol_en > 0.5f) ? 1.0f : 0.0f);
        motor_link_ch_set(CT_RESV0,       0.0f);
        /* "滑条上的数"和"车上真正跑的数"不是一回事，这两个通道就是唯一权威。 */
        motor_link_ch_set(CT_K_V,         spd_g.k_v);
        motor_link_ch_set(CT_K_Z,         spd_g.k_z);
        return;
    }

    age = motor_link_rx_age_ms();
    if (age > 60000u)
    {
        age = 60000u;                  /* rx_age 用 0xFFFFFFFF 表示"从没收到过"，截一下 */
    }

    motor_link_ch_set(CH_TARGET_L,    spd_target_l);
    motor_link_ch_set(CH_V_L,         enc_l.rps);
    motor_link_ch_set(CH_U_L,         ctl_l.u * 100.0f);
    motor_link_ch_set(CH_TARGET_R,    spd_target_r);
    motor_link_ch_set(CH_V_R,         enc_r.rps);
    motor_link_ch_set(CH_U_R,         ctl_r.u * 100.0f);
    motor_link_ch_set(CH_UFF_L,       ctl_l.u_ff * 100.0f);
    motor_link_ch_set(CH_UFF_R,       ctl_r.u_ff * 100.0f);
    motor_link_ch_set(CH_Z_L,         ctl_l.z);
    motor_link_ch_set(CH_Z_R,         ctl_r.z);
    motor_link_ch_set(CH_POS_L,       (float)enc_l.pos);
    motor_link_ch_set(CH_POS_R,       (float)enc_r.pos);
    motor_link_ch_set(CH_ONLINE,      (float)motor_link_online());
    motor_link_ch_set(CH_RX_AGE_MS,   (float)age);
    motor_link_ch_set(CH_ENABLE_BITS, (float)motor_link_enable_bits());
    motor_link_ch_set(CH_SAFETY,      (float)spd_safety_flags());

    /* --- 转向(16~20) --- */
    motor_link_ch_set(CH_STEER_TARGET, steer.applied_delta);
    motor_link_ch_set(CH_STEER_DELTA,  steer.meas_delta);
    motor_link_ch_set(CH_STEER_U,      (float)steer.output_duty);
    motor_link_ch_set(CH_STEER_EN,     (float)steer.angle_enable);
    /* bit0   = 绝对编码器上电自检通过
     * bit1   = 0x02 **反馈失效（锁存）** —— 输出已钉 0；会自动复位(限次数), 也能人工 steer_en 0->1
     * bit2   = 0x04 顶死/无进展**告警**（不钉输出, 粘滞 2s）
     * bit3   = 0x08 **本拍编码器读数被防抖拒掉**（瞬态、不锁存）—— "电机干扰编码器"最早的预警。
     * [!] 判读顺序: bit3 闪 -> 丢样计数在涨 -> 才是 bit1。别一看到 bit1 就去改机械。
     */
    motor_link_ch_set(CH_STEER_STATUS,
                      (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));

    /* + 接收机帧数。rc_estop/rc_steer_deg/rc_thr/rc_scale/rc_tgt_rps/ch5/ch6 全删:
     * 它们要么是别的通道的中间量(steer_tgt/target_l/rps), 要么只在一次性标定时看。 ---
     */
    motor_link_ch_set(CH_RC_ENGAGE,    (float)RC_Engage);
    motor_link_ch_set(CH_RC_CH1,       (float)RC_Raw_Ch1);
    motor_link_ch_set(CH_RC_CH2,       (float)RC_Raw_Ch2);
    motor_link_ch_set(CH_RC_FRAMES,    (float)RC_Frames);

    motor_link_ch_set(CH_K_V,          spd_g.k_v);
    motor_link_ch_set(CH_K_Z,          spd_g.k_z);

    /* [!] 必须走**原子快照**: ins_imu_snapshot() / ins_odom_snapshot()。
     * 直接读全局变量会在"中断正在更新"时拿到撕裂值(半新半旧), 上位机画出来的轨迹
     * 会出现**锯齿/回跳** —— 而且只在车动的时候出现, 静止测不出来。
     */
    {
        ins_imu_snap_t  imu;
        ins_odom_snap_t odom;
        uint8           f = 0u;

        ins_imu_snapshot(&imu);
        ins_odom_snapshot(&odom);

        motor_link_ch_set(CH_INS_YAW,  odom.yaw);
        motor_link_ch_set(CH_INS_GZ,   imu.gyro_z_dps);
        motor_link_ch_set(CH_INS_X,    odom.x);
        motor_link_ch_set(CH_INS_Y,    odom.y);
        motor_link_ch_set(CH_INS_DIST, odom.dist_abs);

        /* 状态位图(8 位, 与工程 8 同一套顺序, 上位机/文档别改) */
        if (ins_imu_ready())                 { f |= 0x01u; }   /* IMU 就绪(零偏标定完) */
        if (imu.flags & INS_ST_MAG_CAL_DONE) { f |= 0x02u; }   /* 磁力计已标定 */
        if (odom.flags & ODOM_ST_STILL)      { f |= 0x04u; }   /* 判定静止(ZUPT 窗口) */
        if (odom.flags & ODOM_ST_SLIP)       { f |= 0x08u; }   /* 本拍判打滑(只报警) */
        if (imu.flags & INS_ST_IMU_LINK_BAD) { f |= 0x40u; }   /* IMU 通信坏 */
        if (imu.flags & INS_ST_WARN)         { f |= 0x80u; }   /* 其它告警 */
        motor_link_ch_set(CH_INS_FLAGS, (float)f);
    }

    motor_link_ch_set(CH_RT_CNT,   (float)route_app_count());
    motor_link_ch_set(CH_RT_ST,    (float)route_app_status());
    motor_link_ch_set(CH_PB_ELAT,  route_pb_elat());
}

/* 定点打印小工具 刻意不用 %f：IAR 的 printf 在默认库配置下不一定支持浮点格式化， 不值得为了一行日志去赌链接器的 formatter 配置。 */
/* 打 2 位小数：返回整数部分，小数部分写进 *fp（-1.23 -> i=-1, f=23） */
static int pf2 (float v, int *fp)
{
    int32 t = (int32)(v * 100.0f);
    int   i;
    int   f;

    if (t < 0)
    {
        i = (int)((-t) / 100);
        f = (int)((-t) % 100);
        if (i != 0) { i = -i; }
        else if (f != 0) { f = -f; }
    }
    else
    {
        i = (int)(t / 100);
        f = (int)(t % 100);
    }
    *fp = f;
    return i;
}

/* 打 2 位小数的辅助宏：printf 里直接写 (int) 与 *fp 两个参数 */
/* 每 1 秒打印一次状态 这也是"出了事能自检"的兜底手段：只要这行在跳，就说明主循环活着、时钟活着、串口活着。 */
static void motor_print_status (void)
{
    uint32 now;
    uint32 trip;
    int    i1, i2, i3, i4;
    int    f1, f2, f3, f4;

    now = motor_link_now_ms();
    if ((uint32)(now - g_print_ms) < 1000u)
    {
        return;
    }
    g_print_ms = now;

    /* 先把"新发生的保护事件"取出来打印一次（只报一次，不刷屏） */
    trip = spd_safety_trip_take();
    if (0 != trip)
    {
        printf("[!!] SAFETY EVENT 0x%04X  (0x04=lost 0x08=no heartbeat 0x10=cfg)  output forced 0\r\n",
               (unsigned)trip);
    }

    i1 = pf2(spd_target_l, &f1);
    i2 = pf2(enc_l.rps,    &f2);
    i3 = pf2(spd_target_r, &f3);
    i4 = pf2(enc_r.rps,    &f4);

    /* int  = WiFi-SPI 模块的 INT(P02_4) 电平: 1=模块空闲可收发, 0=模块忙
     * (发送前会看这一位, 为 0 就整帧丢掉 -> 一个字节都不上 SPI)
     * tx   = 上电以来真正交给模块的遥测帧数, 应匀速增长(50/s)
     * int=1 但 tx 不涨 -> 驱动内部 mutex 卡 BUSY;
     * tx 在涨但上位机没数 -> WiFi/Socket(看 err=) 或目标 IP 不对
     */
    /* dL = 本拍(5ms)计数增量   fL = 真换向次数累计   gL = 读数毛刺丢弃次数累计
     * 若 fL 每秒涨几十上百 -> DIR 线在抖; 若 gL 在涨 -> 库内读 DIR 与我们读的不一致
     */
    printf("[%6lu s] err=%u online=%u en=0x%04X rx_age=%lums safe=0x%04X int=%u tx=%lu | "
           "L tgt=%d.%02d v=%d.%02d | R tgt=%d.%02d v=%d.%02d | cpr=%lu/%lu "
           "| dL=%d fL=%lu gL=%lu\r\n",
           (unsigned long)(now / 1000u),
           (unsigned)motor_link_init_err(),
           (unsigned)motor_link_online(),
           (unsigned)motor_link_enable_bits(),
           (unsigned long)motor_link_rx_age_ms(),
           (unsigned)spd_safety_flags(),
           (unsigned)gpio_get_level(WIFI_SPI_INT_PIN),
           (unsigned long)motor_link_tx_count(),
           i1, f1, i2, f2,
           i3, f3, i4, f4,
           (unsigned long)enc_l.cpr,
           (unsigned long)enc_r.cpr,
           (int)enc_l.delta,
           (unsigned long)enc_l.flip,
           (unsigned long)enc_l.glitch);

    /* 转向一行: 目标/实测转角、输出 duty、绝对编码器原始角与自检结果。 排障第一现场 —— 不动方向盘时 raw 应该稳定、delta 应该接近 0。 */
    {
        /* 转角单位已改成"方向盘度", 用 pf2 打两位小数; 误差也打出来, 调 kp/u0 时最直观。 */
        int st_t, st_tf, st_d, st_df, st_e, st_ef;

        st_t  = pf2(steer.target_delta, &st_tf);
        st_d  = pf2(steer.meas_delta,   &st_df);
        st_e  = pf2(steer.err,          &st_ef);

        /* raw  = 环里**真正用**的值（防抖之后）
         * rl   = 本拍 SPI **实读**值（没防抖）—— raw 与 rl 差得远 = 防抖正在拒样，链路在变差
         * bad  = 上电以来被拒样数（只增）—— **只要它在涨，就说明电机在干扰编码器**
         * ar   = 自动复位次数（只增）—— >0 = 真的发生过"锁存->自动复位"
         * flt  = 状态位: 0x01 反馈失效(锁存,会钉输出) / 0x02 顶死告警 / 0x04 **本拍在丢样**(瞬态)
         * -> 现场判读顺序: 先看 flt&0x04 闪不闪(预警) -> 再看 bad 涨不涨 -> 最后才看 0x01
         */
        printf("        STEER tgt=%d.%02d delta=%d.%02d err=%d.%02d duty=%d raw=%u rl=%u en=%u "
               "flt=0x%02X sc=%u bad=%u ar=%lu\r\n",
               st_t, st_tf,
               st_d, st_df,
               st_e, st_ef,
               (int)steer.output_duty,
               (unsigned)steer_abs_get_raw(),
               (unsigned)steer_abs_get_raw_live(),
               (unsigned)steer.angle_enable,
               (unsigned)steer_ctrl_fault(),
               (unsigned)steer_abs_self_check_ok(),
               (unsigned)steer_abs_bad_cnt(),
               (unsigned long)steer.recover_cnt);
    }

    /* 遥控一行: 接管状态 / 两个开关 / 指令量 / 收帧数 / 遥测档。
     * FRM 不涨             -> 接收机没在发 / 线接错 / 波特率不对(应 SBUS 100k)
     * CH1/CH2 不是 992 附近 -> 摇杆微调没回中, 或接收机型号不是 HotRc(量程不同, 要重标)
     * CH5/CH6 不是 192/1792 -> 开关通道接错, 或极性宏 RC_*_ACTIVE_HIGH 给反
     * ARM=0 但 CH6 已合闸    -> 上位机把 rc_arm_en 写成 0 了
     * ENG 恒 0             -> CH6 没收/收反, 或 rc_arm_en = 0
     * ENG 恒 3(FAILSAFE)   -> 接收机报失控(ST=0): 先看发射机与接收机对码/供电
     * ENG 恒 4(ESTOP)      -> 急停开关 CH5 在"生效"位置, 或者极性宏给反了
     * ENG=2 但车不动        -> 看 SAFE 位图(SAFE_BIT_LOCAL 没亮 = 本地档位没推上;
     * SAFE_BIT_ESTOP=0x80 亮 = 急停还按着)
     */
    {
        int rt, rf, ra, rd, rs, rsf;

        rt = pf2(RC_Throttle * 1000.0f, &rf);
        ra = pf2(RC_Steer_Deg, &rd);
        rs = pf2(RC_Scale * 1000.0f, &rsf);

        printf("        RC  eng=%u st=%u arm=%u estop=%u ch5=%u ch6=%u deg=%d.%02d thr=%d scale=%d frm=%lu local=%u tel=%u/%ums\r\n",
               (unsigned)RC_Engage,
               (unsigned)RC_Online,
               (unsigned)RC_Arm,
               (unsigned)RC_Estop,
               (unsigned)RC_Raw_Ch5,
               (unsigned)RC_Raw_Ch6,
               ra, rd,
               rt, rs,
               (unsigned long)RC_Frames,
               (unsigned)spd_safety_local(),
               (unsigned)motor_link_ch_count(),
               (unsigned)motor_link_tx_period_ms());
    }

    /* 惯导一行: 航向 / 陀螺Z / 位姿 / 里程 / 状态位 / 零偏 / 静止统计。
     * 判读顺序(按这个查, 别跳):
     * 横幅 imu_init 非 0        -> IMU 没认出来, 先查 P15 接线与型号, **别去调姿态**
     * yaw 静止时一直在走        -> 看 bz(在线零偏)那一列; bz 稳而 yaw 仍走 = dt/符号问题
     * gz 静止时 |值| > 0.2 dps  -> 零偏没收敛, 或 ZUPT 没生效(看 st=)
     * x/y 推车方向不对          -> 查 INS_YAW_SIGN 与编码器 inv_l/inv_r 极性
     * dist 与卷尺差 k 倍        -> 就是 ins_mpm 错, 按"推 2.00 m"重标(见 ins_odom.h)
     * fl 的 bit6(0x40) 亮       -> IMU 通信坏; bit0(0x01) 不亮 = 零偏还没标定完
     */
    {
        int   yr, yf, gr, gf, xr, xf, dr, df, sr, sf, br, bf;
        float bx, by, bz;

        ins_diag_get_bias(&bx, &by, &bz);   /* 在线零偏, **raw LSB 域**(1 LSB = 0.0699 dps) */

        yr = pf2(ins_imu_get_yaw(),        &yf);
        gr = pf2(ins_imu_get_gyro_z_dps(), &gf);
        xr = pf2(ins_odom_get_x(),         &xf);
        dr = pf2(ins_odom_get_y(),         &df);
        sr = pf2(ins_odom_get_dist(),      &sf);
        br = pf2(bz,                       &bf);

        printf("        INS  yaw=%d.%02d gz=%d.%02d x=%d.%02d y=%d.%02d dist=%d.%02d fl=0x%02X st=%u bz=%d.%02d(LSB) en=%u mpm=%u frz=%lu/%lu\r\n",
               yr, yf, gr, gf, xr, xf, dr, df, sr, sf,
               (unsigned)ins_odom_flags(),
               (unsigned)((ins_imu_flags() & INS_ST_STATIC) ? 1u : 0u),
               br, bf,
               (unsigned)((ins_odom_enable > 0.5f) ? 1u : 0u),
               (unsigned)(int)(ins_m_per_rev * 10000.0f),
               (unsigned long)ins_imu_zupt_frz_ticks(),
               (unsigned long)(ins_imu_zupt_frz_deg() * 100.0f));
    }
}

int main (void)
{
    uint8   ret;

    clock_init(SYSTEM_CLOCK_250M);      /* 时钟配置及系统初始化<务必保留> */
    debug_init();                       /* 调试串口信息初始化 */

    /* 最底层的"我还活着"标记：只要串口参数对，上电第一秒就能看到这一行。 把"时钟/引脚/波特率对不对"和"软件逻辑跑没跑"分开，排障时能省很多事。 */
    printf("[##] UART OK  steer + dual motor speed loop (WiFi)\r\n");

    /* ----------------------------- 执行器：先钉在 0 ----------------------------- */
    motor_drv_init();                   /* 三路 PWM/DIR(左后/右后/转向)都输出 0，且这条路走通了 */

    /* ----------------------------- 编码器 ----------------------------- */
    spd_enc_init();

    /* --------------------- 转向绝对编码器 + 转向位置环 ---------------------
     * 转向是"直流电机 + 绝对编码器"(不是会自动回中的 RC 舵机), 必须靠位置环把它按住。
     * 注意1: steer_abs_init() 里带一次 SPI 自检(最多约 100ms), 失败不会卡住, 只是"角度不可信"。
     * 注意2: 默认【全不使能】—— 要动方向盘得先标定(见 01-上电标定卡.md 转向一节),
     * 然后上位机把"转向|内环使能"打 1。
     */
    steer_abs_init();
    steer_ctrl_init();

    /* ----------------------------- 保护与控制器 ----------------------------- */
    spd_safety_init();                  /* 内部会跑一遍配置自检 */
    spd_ctrl_init();

    /* -------------------------- 惯导: IMU + 航位推算 (工程 10 新增) --------------------------
     * [!] ins_imu_init() 是**阻塞**的: 上电陀螺零偏标定采 1000 点 x 5ms = **5 秒**,
     * 这 5 秒车必须静止(被碰会判"极差超限"并重来, 最多 5 次 -> 最长约 25 秒)。
     * 所以必须放在 pit_ms_init() **之前** —— 开了中断再等, 中断里的 ins_imu_update()
     * 会拿着没标定好的零偏开始积分, yaw 从第一拍就是错的。
     * [!] 卡丁的 IMU 从来没初始化过(**工程 9 一次都没调过**): 第一次烧录请盯串口横幅,
     * 看它认出的器件 ID。逐飞主板 IMU660RA / IMU660RC / IMU963RA 引脚完全相同
     * (SPI_2 / P15_0~3), 所以"认出来了"这件事本身就要用 ID 证实 —— 963RA 的 0x0F 应为 0x6A。
     */
    ins_odom_init();
    ret = ins_imu_init();
    {
        uint8 id0f = 0u, id00 = 0u, idv = 0u;
        ins_imu_dev_id_get(&id0f, &id00, &idv);
        printf("[##] INS    imu_init=%u id(0F)=0x%02X id(00)=0x%02X (963RA 期望 0x6A; 0x00/0xFF=没接上)\r\n",
               (unsigned)ret, (unsigned)id0f, (unsigned)id00);
    }
    {
        float m  = ins_m_per_rev;
        int   mi = (int)m;
        int   mf = (int)((m - (float)mi) * 10000.0f);
        printf("[##] INS    mpm=%d.%04d m/rev (卡丁 cpr=1858 -> 这就是后轮周长; 占位值, 推 2m 必重标)\r\n",
               mi, mf);
    }

    /* ------------------------ 遥控接收机(UART_4 / P14.0) ------------------------
     * 逐飞 UART 接收机(SBUS, 100 kbps, 6 通道)。
     * [!] 必须在 pit_ms_init(PIT_CH2, 5) 之前 —— 它内部要用 TC_TIME2_CH2 做帧间隔计时。
     * [!] 遥控【不依赖 WiFi】: 接管成立时由 remote_ctrl_apply() 调 spd_safety_set_local(1),
     */
    remote_ctrl_init();

    if (0 != spd_safety_cfg_bad())
    {
        printf("[##] CONFIG BAD 0x%02X - motor stays off\r\n", (unsigned)spd_safety_cfg_bad());
    }

    /* ------------------------- 5ms 时基：速度环 + 链路 ------------------------- */
    /* 参数持久化：从 Work Flash 页 0 读回上次 SAVE 的参数。
     * 必须在各模块 init 之后、开 5ms 中断之前 —— 跑起来就是你上次存的增益。
     * [!] 命令/危险量（target / ol_en / ol_duty / steer_en / ins_zero）**不恢复**，
     * 上电永远是安全值，见 app_params.c 的 ap_is_cmd()。
     */
    app_param_init();

    /* 路表：读槽索引 -> 补空槽的内建表 -> 载入 active 槽。
     * [!] 必须在这里（开 5ms 中断之前）：内部会阻塞读 Flash。
     * [!] 内建表只补空槽，且必须在 flash_load 之前（见 ins_route.h 的语义）。
     */
    route_app_init();
    route_pb_init();

    /* 12 段跑班：初始化状态（开环一律关掉，不会自己跑） */
    spd_bench_init();

    pit_ms_init(PIT_CH2, 5);
    interrupt_global_enable(0);         /* pit0_ch2_isr 里跑 spd_ctrl_tick() */

    /* 先给模块一点上电稳定时间：WiFi 模块刚上电时 SPI 还没准备好， 紧接着就 init 容易读到失败（原厂例子是"上电后按键触发"，天然有几秒间隔）。 */
    system_delay_ms(500);

    /* --------------------------- WiFi-SPI 链路 ---------------------------
     * 这个函数会阻塞：WiFi 最长等约 10 秒、Socket 最长等数十秒。
     * 失败也不会卡死在这里（返回非 0），主循环照样跑，只是保护层永远不允许出力。
     */
    ret = motor_link_init();
    if (0 != ret)
    {
        printf("[##] LINK INIT FAIL ret=%u (1=wifi 2=socket)\r\n", (unsigned)ret);
    }
    else
    {
        printf("[##] LINK OK  ssid:%s  ->%s:%s\r\n", MOTOR_WIFI_SSID, MOTOR_HOST_IP, MOTOR_HOST_PORT);
    }

    g_print_ms = motor_link_now_ms();

    /* --------------------------- 屏幕 + 按键（菜单） ---------------------------
     * 屏幕: 逐飞 IPS200 的 **SPI 版**（硬件 SPI_1），6 根脚由 zf_device_ips200.h 定死：
     * SCL P12_2 / SDA(MOSI) P12_1 / CS P12_3 / RST P22_4 / DC P22_3 / BLK P11_0
     * 本工程别的模块**不占**这 6 根：转向绝对编码器走 SPI_4 的 P07_x、
     * WiFi 走 SPI_0 的 P02_x、板载 IMU 走 SPI_2 的 P15_x。改屏幕接线只需改那 6 个宏。
     * 按键: 主板自带 4 键 P20_3/P20_2/P20_0/P20_1（K3/K4 已按工程 8 对调）。
     * [!] 必须在 motor_link_init() 之后：菜单用 motor_link_now_ms() 做 10 ms 分频，
     * 而 key_init(10) 的周期必须与它一致（见 ui_menu.h 头注释）。
     * [!] 屏幕 SPI 是毫秒级开销（清一屏约 40 ms）=> 只能在主循环里画，
     * 绝不能放进 5 ms 中断，否则控制节拍直接被顶穿。
     * 屏幕连背光和复位都不会拉，看起来像"接错线"，其实一行代码都没执行。
     */
    ui_init();
    ui_puts_row(0, 0, "Kart10 INS+Route");
    printf("[##] UI init done: IPS200 SPI_1 (SCL P12_2 / SDA P12_1 / CS P12_3 / DC P22_3 / RST P22_4 / BLK P11_0)\r\n");
    ui_menu_init();

    while (true)
    {
        /* tel_sel=0 -> 全量 37 通道；=1 -> 标定精简 16 通道。 [!] 通道数一变，帧头 COUNT 跟着变，上位机的档案下拉框要同步切。 */
        motor_link_set_ch_count((spd_tel_sel > 0.5f) ? MOTOR_CH_COUNT_CALIB : MOTOR_CH_COUNT);
        motor_link_set_tx_period_ms((uint16)spd_tel_period);

        /* 1) 链路任务：收上位机命令 + 发遥测。
         * 必须在循环里每圈都调到 —— WiFi 的收发都在这里推进，
         * 而且要放在任何"可能失败的分支"之外，否则失败路径会把它停掉，
         * 表现就是"上位机再也收不到数据"却查不出原因。
         */
        motor_link_task();

        /* 2) 主循环心跳：给保护层证明"主循环没卡死"。 超过 200ms 没心跳，保护层直接把两路输出钉 0。 */
        spd_safety_heartbeat();

        /* 放主循环(不是中断)是刻意的: 它写的 steer_en_cmd / spd_target_* 本来就属于
         * "慢速决策层", 真正的 5ms 控制仍在 pit0_ch2_isr 里。
         * [!] 绝不能把遥控做成 while(1) 页面 —— 那样主循环心跳就断了,
         */
        remote_ctrl_update();
        remote_ctrl_apply();

        /* 2d) 工程 10 新增：12 段跑班收尾 + 日志落盘 + 屏幕菜单。
         * 放在心跳之后：菜单刷屏最差 ~40 ms，不能把心跳饿死（200 ms 门）。
         * bench_log_poll() 每圈**最多写一页 Flash**，不长阻塞。
         */
        spd_bench_poll();
        bench_log_poll();
        route_app_poll();       /* 路表: 录制采样 + 串口灌表 + 每圈一页落盘 */
        route_pb_poll();        /* 路表: 回放状态机（不给速度时只算目标航向） */
        ui_menu_task();

        /* 2c) 惯导的慢动作: 位姿归零 + IMU 掉线重试。
         * 放主循环的理由与遥控一样 —— 它们是"慢速决策", 不是 5ms 控制。
         * [!] 归零必须【位姿 + 航向】一起归: 只归一个, 后面所有相对量(路表/回放)
         * 就整体错位了, 而且**看着仍然像一条轨迹**。
         * [!] IMU 重试只在链路坏时做: 那时中断里的 ins_imu_update() 不碰 SPI_2,
         * 主循环里重新 init 才没有竞争(ins_imu_try_recover 内部就是这么判的)。
         */
        {
            uint8 nz = (ins_zero_cmd > 0.5f) ? 1u : 0u;

            if (nz && (!g_zero_last))
            {
                ins_odom_reset();
                ins_imu_zero_yaw();
                printf("[##] INS    zero (x/y/yaw = 0)\r\n");
            }
            g_zero_last = nz;

            if (0 == ins_imu_link_ok())
            {
                uint32 tnow = motor_link_now_ms();
                if ((uint32)(tnow - g_imu_retry_ms) >= 1000u)
                {
                    g_imu_retry_ms = tnow;
                    (void)ins_imu_try_recover();
                }
            }
        }

        /* 3) 转向环已挪到 pit0_ch2_isr(5ms 中断)里跑, 见 cm7_0_isr.c —— 这里不再调用。
         * 挪走的理由: 主循环周期被 WiFi 收发拖成几毫秒的抖动, 而目标斜坡(度/秒)
         * 与误差死区都依赖真实周期; 放中断才能保证严格 5ms。
         * 千万别两处都调, 否则同一拍跑两遍, 斜坡会走两倍速。
         */

        /* 4) 填遥测 + 打印状态 */
        motor_fill_telemetry();
        motor_print_status();

        system_delay_ms(1);
    }
}
