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
* 本工程: 卡丁车整车固件(转向位置环 + 双后轮速度环 + SBUS 遥控 + WiFi 上位机链路)
*
* ---- 硬件连接 ----
*   左后轮:  PWM -> P09_0 (TCPWM_CH24)    DIR -> P09_1
*   右后轮:  PWM -> P10_2 (TCPWM_CH30)    DIR -> P10_3
*   转向  :  PWM -> P05_0 (TCPWM_CH09)    DIR -> P05_1
*   左后轮编码器: PULSE -> P17_3 (TC_CH58 CH1)   DIR -> P17_4 (TC_CH58 CH2)  [脉冲+方向]
*   右后轮编码器: PULSE -> P19_2 (TC_CH27 CH1)   DIR -> P19_3 (TC_CH27 CH2)  [脉冲+方向]
*   转向绝对角编码器: SPI_4(SCB5)  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3
*   WiFi-SPI 模块: 见 motor_link.h（SPI_0 + INT P02_4 + RST P23_0）
*   遥控接收机  : UART_4 = SCB2, RX = P14_0（接接收机的 TX）, TX = P14_1（占位, 悬空）
*                 SBUS 100 kbps, 6 通道; 帧间隔计时占 TC_TIME2_CH2。见 zf_device_uart_receiver.h
*   改引脚后 minv_l / minv_r / minv_s 三个极性必须重新核对
*
* ---- 台架标定（顺序不能颠倒）----
*   1 极性: 车架空, 给 steer_u0 看方向盘能否被推动, 再给 +3 度小目标确认转向跟随方向;
*           反向则 mot_inv_s 取反, 否则正反馈堵转顶住限位会损坏 DRV8701。后轮改 minv_l/minv_r。
*   2 方向与测速: 正目标应使电机朝车前进方向转, 且该侧 v 为正(正命令 => 该侧 v 为正); 反了改极性宏。
*   3 每转计数: 清零 pos, 单方向手转电机轴 10 圈, cpr = pos 增量 / 10; 本车 1858(1024 线 x 49/27)。
*   4 遥控: Debug 看 CH1/CH2 在中位 992 附近, 拨 CH5 看 eng 是否 0->1->2。参数可在上位机滑条在线改。
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

/* 屏幕菜单 + 12 段跑班 + 路表 + 协议 */
#include "flash_map.h"
#include "app_params.h"
#include "bench_log.h"
#include "spd_bench.h"
#include "ui_core.h"
#include "ui_menu.h"
#include "route_app.h"          /* 路表应用层 */
#include "route_pb.h"
#include "proto.h"              /* 协议层/权限层, 唯一写入口 */
#include "head_test.h"          /* 航向测试 */

/* ---- 内部状态 ---- */
static uint32   g_print_ms = 0;        /* 上次打印状态的时刻 */
static uint8    g_zero_last    = 0;   /* 位姿归零命令边沿记忆(0->1 触发一次) */
static uint32   g_imu_retry_ms  = 0;   /* IMU 掉线后的重试节拍(1Hz) */

/* ---- 遥测通道填充(顺序即协议, 见 motor_link.h) ---- */
static void motor_fill_telemetry (void)
{
    uint32 age;

    /* 标定精简档: 参数 tel_sel(>0.5) 切入, 16 通道 85 字节/帧; 通道号 CT_* 与全量档 CH_* 不同, 直接 return 不叠加 */
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
        /* 发实际施加的目标(软限位夹紧 + 斜坡之后), 非上位机写的命令 */
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
        /* 增益回读: 控制器本拍真正乘上去的 k_v / k_z */
        motor_link_ch_set(CT_K_V,         spd_g.k_v);
        motor_link_ch_set(CT_K_Z,         spd_g.k_z);
        return;
    }

    age = motor_link_rx_age_ms();
    if (age > 60000u)
    {
        age = 60000u;                  /* rx_age 用 0xFFFFFFFF 表示从没收到过，截一下 */
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
    motor_link_ch_set(CH_STEER_TARGET, steer.applied_delta);   /* 实际施加目标 */
    motor_link_ch_set(CH_STEER_DELTA,  steer.meas_delta);
    motor_link_ch_set(CH_STEER_U,      (float)steer.output_duty);
    motor_link_ch_set(CH_STEER_EN,     (float)steer.angle_enable);
    /* bit0=绝对编码器上电自检通过   bit1=0x02 反馈失效(锁存), 输出钉 0, 可自动复位或 steer_en 0->1
       bit2=0x04 顶死/无进展告警, 不钉输出, 粘滞 2s   bit3=0x08 本拍读数被防抖拒掉, 瞬态不锁存 */
    motor_link_ch_set(CH_STEER_STATUS,
                      (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));

    /* 遥控(21~24): 接管状态 + CH1/CH2 原始值 + 收帧数 */
    motor_link_ch_set(CH_RC_ENGAGE,    (float)RC_Engage);
    motor_link_ch_set(CH_RC_CH1,       (float)RC_Raw_Ch1);
    motor_link_ch_set(CH_RC_CH2,       (float)RC_Raw_Ch2);
    motor_link_ch_set(CH_RC_FRAMES,    (float)RC_Frames);

    /* 增益回读(25~26): k_v / k_z */
    motor_link_ch_set(CH_K_V,          spd_g.k_v);
    motor_link_ch_set(CH_K_Z,          spd_g.k_z);

    /* 惯导(39~44): 必须走原子快照, 直接读全局量会在中断更新时拿到撕裂值 */
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

        /* 状态位图(8 位, 顺序固定, 上位机/文档别改) */
        if (ins_imu_ready())                 { f |= 0x01u; }   /* IMU 就绪(零偏标定完)  */
        if (imu.flags & INS_ST_MAG_CAL_DONE) { f |= 0x02u; }   /* 磁力计已标定          */
        if (odom.flags & ODOM_ST_STILL)      { f |= 0x04u; }   /* 判定静止(ZUPT 窗口)   */
        if (odom.flags & ODOM_ST_SLIP)       { f |= 0x08u; }   /* 本拍判打滑(只报警)    */
        if (imu.flags & INS_ST_IMU_LINK_BAD) { f |= 0x40u; }   /* IMU 通信坏            */
        if (imu.flags & INS_ST_WARN)         { f |= 0x80u; }   /* 其它告警              */
        motor_link_ch_set(CH_INS_FLAGS, (float)f);
    }

    /* --- 路表/回放(45~47) --- */
    motor_link_ch_set(CH_RT_CNT,   (float)route_app_count());
    motor_link_ch_set(CH_RT_ST,    (float)route_app_status());
    motor_link_ch_set(CH_PB_ELAT,  route_pb_elat());
    /* CH36 = 航向外环的实际输入, 按优先级: 航向测试 > 航向环(PP/遥控) > 当前航向 */
    motor_link_ch_set(CH_TAR_YAW, (0u != head_test_active()) ? head_test_tar_yaw()
                                : ((0u != steer.head_enable) ? steer_ctrl_get_target_yaw()
                                                             : ins_odom_get_yaw()));
    motor_link_ch_set(CH_HT_SEG,      head_test_seg());              /* 0关/1..n段/254完 */

    /* 诊断三通道(38~40):
       tick_ms   = 5ms 时基(60 s 应涨约 60000)
       ins_frz   = ZUPT 冻结期间丢弃的转角(度, 累计只增)
       ins_static= IMU 判静止(冻结生效条件, 非编码器静止) */
    motor_link_ch_set(CH_TICK_MS,    (float)motor_link_now_ms());
    motor_link_ch_set(CH_INS_FRZ,    ins_imu_zupt_frz_deg());
    motor_link_ch_set(CH_INS_STATIC, (float)((0u != (ins_imu_flags() & INS_ST_STATIC)) ? 1u : 0u));

    /* 回放诊断(41~43): kref->0 且 best 平稳 = 表内平段; 跳变 = 投影窗 */
    motor_link_ch_set(CH_PB_BEST, route_pb_get_best());
    motor_link_ch_set(CH_PB_FI,   route_pb_get_fi());
    motor_link_ch_set(CH_PB_KREF, route_pb_get_kref());
    motor_link_ch_set(CH_PB_WHY,  route_pb_get_why_code());
}

/* ---- 定点打印小工具: 不用 %f, IAR 默认库不保证支持浮点格式化 ---- */
/* 打 2 位小数: 返回整数部分, 小数部分写进 *fp (-1.23 -> i=-1, f=23) */
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

/* ---- 每 1 秒打印一次状态 ---- */
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

    /* 取新发生的保护事件, 只报一次 */
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

    /* int = 模块 INT(P02_4) 电平: 1=可收发, 0=忙(整帧丢弃)
       tx  = 上电以来交给模块的遥测帧数, 应 50/s 匀速增长
       dL = 本拍(5ms)计数增量  fL = 换向次数累计  gL = 毛刺丢弃累计 */
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

    /* 航向自检: yaw 为累积值(不折叠), 转一圈 +360; rej = 单拍转角超限被丢弃的拍数 */
    printf("[##] IMU yaw=%ld deg  rej=%u\r\n",
           (long)ins_imu_get_yaw(), (unsigned)ins_imu_yaw_rej());

    /* 转向行: 目标/实测转角、输出 duty、绝对编码器原始角与自检结果 */
    {
        /* 转角单位: 方向盘度; 误差一并打印 */
        int st_t, st_tf, st_d, st_df, st_e, st_ef;

        st_t  = pf2(steer.target_delta, &st_tf);
        st_d  = pf2(steer.meas_delta,   &st_df);
        st_e  = pf2(steer.err,          &st_ef);

        /* raw = 环里真正用的值(防抖之后)      rl  = 本拍 SPI 实读值(未防抖)
           bad = 上电以来被拒样数(只增)        ar  = 自动复位次数(只增)
           flt = 状态位: 0x01 反馈失效(锁存) / 0x02 顶死告警 / 0x04 本拍丢样(瞬态) */
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

    /* 遥控行: 接管 eng / 开关 arm/estop / 指令量 / 收帧数 frm / 遥测档 */
    {
        int rt, rf, ra, rd, rs, rsf;

        rt = pf2(RC_Throttle * 1000.0f, &rf);
        ra = pf2(RC_Steer_Deg, &rd);
        rs = pf2(RC_Scale * 1000.0f, &rsf);

        printf("        RC  eng=%u st=%u arm=%u estop=%u ch5=%u ch6=%u deg=%d.%02d thr=%d scale=%d mid=%u/%u frm=%lu local=%u tel=%u/%ums\r\n",
               (unsigned)RC_Engage,
               (unsigned)RC_Online,
               (unsigned)RC_Arm,
               (unsigned)RC_Estop,
               (unsigned)RC_Raw_Ch5,
               (unsigned)RC_Raw_Ch6,
               ra, rd,
               rt, rs,
               (unsigned)RC_Steer_Mid, (unsigned)RC_Thr_Mid,
               (unsigned long)RC_Frames,
               (unsigned)spd_safety_local(),
               (unsigned)motor_link_ch_count(),
               (unsigned)motor_link_tx_period_ms());

        /* CH3 行: c3=CH3 原始值  thr=判高低自适应阈值  cal=已见过开关两端
           mode=HDG(CH1 控航向) / SRV(CH1 控舵机角)  off=航向偏移  ty=目标航向  yw=实测航向 */
        printf("        RC3 c3=%u thr=%u cal=%u mode=%s off=%d ty=%d yw=%d\r\n",
               (unsigned)RC_Raw_Ch3,
               (unsigned)RC_Hdg_Thr,
               (unsigned)RC_Hdg_Cal,
               (0u != RC_Hdg_Mode) ? "HDG" : "SRV",
               (int)RC_Hdg_Off,
               (int)RC_Cmd_Yaw,
               (int)steer_yaw_deg);
    }

    /* 惯导行: yaw 航向 / gz 陀螺Z / x,y 位姿 / dist 里程 / fl 状态位 / bz 在线零偏 */
    {
        int   yr, yf, gr, gf, xr, xf, dr, df, sr, sf, br, bf;
        float bx, by, bz;

        ins_diag_get_bias(&bx, &by, &bz);   /* 在线零偏, raw LSB 域(1 LSB = 0.0699 dps) */

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

    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化

    /* 上电首行横幅: 确认时钟/引脚/波特率正确, 把硬件与软件问题分开 */
    printf("[##] UART OK  steer + dual motor speed loop (WiFi)\r\n");

    /* --- 执行器: 上电先钉 0 --- */
    motor_drv_init();                   /* 三路 PWM/DIR(左后/右后/转向)都输出 0 */

    /* --- 编码器 --- */
    spd_enc_init();

    /* --- 转向绝对编码器 + 转向位置环: 直流电机 + 绝对编码器, 不会自动回中, 靠位置环保持 ---
       steer_abs_init() 内含一次 SPI 自检(约 100ms), 失败不阻塞, 仅角度不可信; 默认全不使能,
       需上位机把转向|内环使能打 1。 */
    steer_abs_init();
    steer_ctrl_init();

    /* --- 保护与控制器 --- */
    spd_safety_init();                  /* 内部会跑一遍配置自检 */
    spd_ctrl_init();

    /* --- 协议层: WiFi / 菜单 / 遥控 三条写入路径统一到 proto_exec() ---
       必须放在 remote_ctrl_init() 之前: 它会调 proto_estop(), 急停的唯一入口。
       主循环每圈调 proto_tick() 检查驱动令牌租期。不占引脚、不阻塞。 */
    proto_init();

    /* --- 惯导: IMU + 航位推算 ---
       ins_imu_init() 阻塞约 5 秒(1000 点 x 5ms 陀螺零偏标定), 期间车必须静止, 被碰会重来;
       必须在 pit_ms_init() 之前调用, 否则中断用未标定零偏积分。横幅打印器件 ID 与 0x0F: 963RA 应为 0x6A。 */
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

    /* --- 遥控接收机(UART_4 / P14.0): 逐飞 UART 接收机, SBUS 100 kbps, 6 通道 ---
       必须在 pit_ms_init(PIT_CH2, 5) 之前(内部用 TC_TIME2_CH2 计时); 接管成立时 remote_ctrl_apply()
       调 spd_safety_set_local(1) 换掉保护层的上位机在场门。 */
    remote_ctrl_init();

    if (0 != spd_safety_cfg_bad())
    {
        printf("[##] CONFIG BAD 0x%02X - motor stays off\r\n", (unsigned)spd_safety_cfg_bad());
    }

    /* --- 5ms 时基: 速度环 + 链路 --- */
    /* 参数持久化: 从 Work Flash 页 0 读回上次 SAVE 的参数; 必须在各模块 init 之后、开 5ms 中断之前。
       命令/危险量(target / ol_en / ol_duty / steer_en / ins_zero)不恢复, 上电为安全值。 */
    app_param_init();

    /* 路表: 读槽索引 -> 补空槽的内建表 -> 载入 active 槽; 必须在开 5ms 中断之前(阻塞读 Flash)。 */
    route_app_init();
    route_pb_init();

    /* 12 段跑班: 初始化状态, 开环一律关掉 */
    spd_bench_init();

    pit_ms_init(PIT_CH2, 5);
    interrupt_global_enable(0);         /* pit0_ch2_isr 里跑 spd_ctrl_tick() */

    /* 等 WiFi 模块上电稳定: 刚上电 SPI 未就绪, 紧接着 init 容易失败 */
    system_delay_ms(500);

    /* --- WiFi-SPI 链路 ---
       阻塞: WiFi 最长约 10 秒、Socket 最长数十秒; 失败返回非 0, 主循环照跑, 保护层不允许出力。 */
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

    /* --- 屏幕 + 按键(菜单) ---
       屏幕: 逐飞 IPS200 SPI 版(硬件 SPI_1), SCL P12_2 / SDA(MOSI) P12_1 / CS P12_3 / RST P22_4 / DC P22_3 / BLK P11_0;
             其余模块不占这 6 根: 转向编码器 SPI_4 (P07_x)、WiFi SPI_0 (P02_x)、IMU SPI_2 (P15_x)。按键: 4 键 P20_3/P20_2/P20_0/P20_1。
       必须在 motor_link_init() 之后: 菜单用 motor_link_now_ms() 做 10ms 分频, key_init(10) 周期须一致。
       屏幕 SPI 为毫秒级开销(清屏约 40ms), 只能在主循环画, 放进 5ms 中断会顶穿节拍; ui_init() 漏调则全黑屏。 */
    ui_init();
    ui_puts_row(0, 0, "Kart10 INS+Route");
    printf("[##] UI init done: IPS200 SPI_1 (SCL P12_2 / SDA P12_1 / CS P12_3 / DC P22_3 / RST P22_4 / BLK P11_0)\r\n");
    ui_menu_init();

    while (true)
    {
        /* 0) 遥测档切换: tel_sel=0 全量 37 通道, =1 标定精简 16 通道; 通道数变则帧头 COUNT 变 */
        motor_link_set_ch_count((spd_tel_sel > 0.5f) ? MOTOR_CH_COUNT_CALIB : MOTOR_CH_COUNT);
        motor_link_set_tx_period_ms((uint16)spd_tel_period);

        /* 1) 链路任务: 收上位机命令 + 发遥测; 每圈必须调到, 且不能放在可能失败的分支里 */
        motor_link_task();

        /* 2) 主循环心跳: 超过 200ms 无心跳, 保护层把两路输出钉 0 */
        spd_safety_heartbeat();

        /* 2b) 遥控: 读接收机快照 -> 换算 -> 接管状态机 -> 写目标 + 推本地档位。
               慢速决策放主循环, 5ms 控制仍在 pit0_ch2_isr; 不能做成阻塞页面, 否则心跳断、200ms 后全钉 0。 */
        remote_ctrl_update();
        remote_ctrl_apply();

        /* 2c) 协议层心跳: 检查驱动令牌租期, 到期释放; 只有链路真断才清零速度目标。放在 route_pb_poll() 之前 */
        proto_tick();

        /* 航向测试在 5ms 中断里跑, 不能从那里碰协议层(会抢 owner/租期); 由这里替它续 SPI 驱动权 */
        if (0u != head_test_active()) { (void)proto_drv_acquire(PROTO_SRC_SPI); }

        /* 2d) 12 段跑班收尾 + 日志落盘 + 屏幕菜单; 放在心跳之后, bench_log_poll() 每圈最多写一页 Flash */
        spd_bench_poll();
        bench_log_poll();
        route_app_poll();       /* 路表: 录制采样 + 串口灌表 + 每圈一页落盘 */
        route_pb_poll();        /* 路表: 回放状态机（不给速度时只算目标航向）*/
        ui_menu_task();

        /* 2e) 惯导慢动作: 位姿归零(x/y/yaw 必须一起归) + IMU 掉线重试(仅链路坏时做, 无 SPI 竞争) */
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

        /* 3) 转向环在 pit0_ch2_isr(5ms 中断)里跑, 这里不再调用; 两处都调会让目标斜坡走两倍速 */

        /* 4) 填遥测 + 打印状态 */
        motor_fill_telemetry();
        motor_print_status();

        system_delay_ms(1);
    }
}
