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
* 本工程: 卡丁车整车固件(转向位置环 + 双后轮速度环 + SBUS 遥控 + WiFi 上位机 + 路表回放)
*
* ---- 硬件连接 ----
*   左后轮:  PWM -> P09_0 (TCPWM_CH24)    DIR -> P09_1
*   右后轮:  PWM -> P10_2 (TCPWM_CH30)    DIR -> P10_3
*   转向  :  PWM -> P05_0 (TCPWM_CH09)    DIR -> P05_1
*   左后轮编码器: PULSE -> P17_3 (TC_CH58 CH1)   DIR -> P17_4 (TC_CH58 CH2)  [脉冲+方向]
*   右后轮编码器: PULSE -> P19_2 (TC_CH27 CH1)   DIR -> P19_3 (TC_CH27 CH2)  [脉冲+方向]
*   转向绝对角编码器: SPI_4(SCB5)  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3
*   WiFi-SPI 模块 : 见 motor_link.h (SPI_0 + INT P02_4 + RST P23_0)
*   遥控接收机    : UART_4 = SCB2, RX = P14_0 (接接收机 TX), TX = P14_1 (占位, 悬空)
*                   SBUS 100 kbps, 6 通道; 帧间隔计时占 TC_TIME2_CH2, 见 zf_device_uart_receiver.h
*   屏幕          : IPS200 SPI_1, SCL P12_2 / SDA P12_1 / CS P12_3 / RST P22_4 / DC P22_3 / BLK P11_0
*   按键          : 板载 4 键 P20_3 / P20_2 / P20_0 / P20_1
*
* ---- 台架标定(按顺序做) ----
*   1 硬件极性: 车架空, 给 steer_u0 看方向盘能否推动, 再给 +3 度看是否朝目标转, 反向改 minv_s, 后轮同理改 minv_l/minv_r。
*   2 测速正负: 正命令须得该侧 v 为正, 反向改 inv_l / inv_r。
*   3 每转计数: 清零 pos, 手转电机轴 10 圈, cpr = pos 增量 / 10 (本车 1024 线 x 49/27 = 1858)。
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

/* ---- 屏幕菜单 + 12 段跑班 ---- */
#include "flash_map.h"
#include "app_params.h"
#include "bench_log.h"
#include "spd_bench.h"
#include "ui_core.h"
#include "ui_menu.h"
#include "route_app.h"          /* 路表(P1): 应用层 */
#include "route_pb.h"
#include "proto.h"              /* 协议层/权限层, 唯一写入口 */
#include "head_test.h"            /* 航向测试 */

static uint32   g_print_ms = 0;        /* 上次打印状态的时刻 */
static uint8    g_zero_last    = 0;   /* 位姿归零命令的边沿记忆, 0->1 只触发一次 */
static uint32   g_imu_retry_ms  = 0;   /* IMU 掉线后的重试节拍, 1 Hz */

/* ---- 遥测通道填充 ---- */
static void motor_fill_telemetry (void)
{
    uint32 age;

    /* 巡线档 route: 22 通道, tel_sel = 0, 默认档; 须走原子快照, 三档通道号互不为子集, 填完直接 return */
    if (spd_tel_sel < 0.5f)
    {
        ins_imu_snap_t  imu;
        ins_odom_snap_t odom;

        ins_imu_snapshot(&imu);
        ins_odom_snapshot(&odom);

        /* --- 两条速度环: 目标 / 实测 / 输出 --- */
        motor_link_ch_set(RCH_U_L,            ctl_l.u * 100.0f);
        motor_link_ch_set(RCH_V_L,            enc_l.rps);
        motor_link_ch_set(RCH_TARGET_L,       spd_target_l);
        motor_link_ch_set(RCH_U_R,            ctl_r.u * 100.0f);
        motor_link_ch_set(RCH_V_R,            enc_r.rps);
        motor_link_ch_set(RCH_TARGET_R,       spd_target_r);

        /* --- 转向: 实际施加 / 实测 / 输出 --- */
        motor_link_ch_set(RCH_STEER_TARGET,   steer.applied_delta);
        motor_link_ch_set(RCH_STEER_DELTA,    steer.meas_delta);
        motor_link_ch_set(RCH_STEER_U,        (float)steer.output_duty);

        motor_link_ch_set(RCH_K_V,            spd_g.k_v);
        motor_link_ch_set(RCH_K_Z,            spd_g.k_z);

        /* --- 惯导: 航向 / 角速度 / 路表横向误差 --- */
        motor_link_ch_set(RCH_INS_YAW,        odom.yaw);
        motor_link_ch_set(RCH_TAR_YAW,        (0u != head_test_active()) ? head_test_tar_yaw()
                                            : ((0u != steer.head_enable) ? steer_ctrl_get_target_yaw()
                                                                         : ins_odom_get_yaw()));
        motor_link_ch_set(RCH_INS_GZ,         imu.gyro_z_dps);
        motor_link_ch_set(RCH_PB_ELAT,        route_pb_elat());

        motor_link_ch_set(RCH_RC_ENGAGE,      (float)RC_Engage);
        motor_link_ch_set(RCH_STEER_EN,       (float)steer.angle_enable);
        motor_link_ch_set(RCH_STEER_STATUS,
                          (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));
        motor_link_ch_set(RCH_RT_CNT,         (float)route_app_count());
        motor_link_ch_set(RCH_RT_ST,          (float)route_app_status());
        motor_link_ch_set(RCH_TICK_MS,        (float)motor_link_now_ms());
        motor_link_ch_set(RCH_INS_STATIC,
                          (float)((0u != (ins_imu_flags() & INS_ST_STATIC)) ? 1u : 0u));
        return;
    }

    /* 标定档 calib: 18 通道, tel_sel = 1, 一帧 93 字节, 适合长时间高密度采样; CT_* 独立编号 */
    if (spd_tel_sel < 1.5f)
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
        /* 发实际施加的目标(软限位夹紧与斜坡之后), 不是上位机写的命令 */
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

    /* 全量档 full: 45 通道, tel_sel = 2, 一帧 201 B, 200 Hz 下约 40 kB/s; CH_* 独立编号, 填完直接 return */
    if (spd_tel_sel < 2.5f)
    {
        age = motor_link_rx_age_ms();
        if (age > 60000u)
        {
            age = 60000u;                  /* rx_age 用 0xFFFFFFFF 表示从没收到过, 截到 60 s */
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
        motor_link_ch_set(CH_STEER_TARGET, steer.applied_delta);   /* 实际施加的目标 */
        motor_link_ch_set(CH_STEER_DELTA,  steer.meas_delta);
        motor_link_ch_set(CH_STEER_U,      (float)steer.output_duty);
        motor_link_ch_set(CH_STEER_EN,     (float)steer.angle_enable);
        /* bit0 = 绝对编码器自检通过   bit1 = 0x02 反馈失效(锁存, 输出钉 0, 自动复位限次数)
           bit2 = 0x04 顶死/无进展告警(不钉输出, 粘滞 2 s)   bit3 = 0x08 本拍读数被防抖拒掉(瞬态) */
        motor_link_ch_set(CH_STEER_STATUS,
                          (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));

        /* --- 遥控(21~24): 接管状态 / 摇杆原始值 / 接收机帧数 --- */
        motor_link_ch_set(CH_RC_ENGAGE,    (float)RC_Engage);
        motor_link_ch_set(CH_RC_CH1,       (float)RC_Raw_Ch1);
        motor_link_ch_set(CH_RC_CH2,       (float)RC_Raw_Ch2);
        motor_link_ch_set(CH_RC_FRAMES,    (float)RC_Frames);

        /* --- 增益回读(25~26) --- */
        motor_link_ch_set(CH_K_V,          spd_g.k_v);
        motor_link_ch_set(CH_K_Z,          spd_g.k_z);

        /* --- 惯导(39~44): 必须走原子快照, 直接读全局量会拿到撕裂值 --- */
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

            /* 状态位图 8 位, 顺序固定, 上位机侧不可改 */
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
        /* CH36 = 航向外环的实际输入, 优先级: 航向测试 > 航向环(PP/遥控) > 当前航向 */
        motor_link_ch_set(CH_TAR_YAW, (0u != head_test_active()) ? head_test_tar_yaw()
                                    : ((0u != steer.head_enable) ? steer_ctrl_get_target_yaw()
                                                                 : ins_odom_get_yaw()));
        motor_link_ch_set(CH_HT_SEG,      head_test_seg());              /* 0关/1..n段/254完 */

        /* --- 诊断三件套(38~40): tick_ms 时基(60 s 应涨约 60000, 偏小 = 中断被饿死) /
           ins_frz ZUPT 冻结期丢弃的转角(度, 累计只读) / ins_static IMU 判静止标志 --- */
        motor_link_ch_set(CH_TICK_MS,    (float)motor_link_now_ms());
        motor_link_ch_set(CH_INS_FRZ,    ins_imu_zupt_frz_deg());
        motor_link_ch_set(CH_INS_STATIC, (float)((0u != (ins_imu_flags() & INS_ST_STATIC)) ? 1u : 0u));

        /* --- 回放诊断(41~43): kref 归 0 而 best 正常 = 表内平段, best 跳变 = 投影窗口切换 --- */
        motor_link_ch_set(CH_PB_BEST, route_pb_get_best());
        motor_link_ch_set(CH_PB_FI,   route_pb_get_fi());
        motor_link_ch_set(CH_PB_KREF, route_pb_get_kref());
        motor_link_ch_set(CH_PB_WHY,  route_pb_get_why_code());
        return;
    }

    /* 采集档 train: 16 通道, tel_sel = 3。舵角/舵角速率/中心速度/左右轮速/yaw_rate/目标曲率/占空比,
       不含 x/y/dist/yaw; 一帧 85 B, TRCH_* 独立编号, 填完直接 return。 */
    if (spd_tel_sel >= 2.5f)
    {
        ins_imu_snap_t imu;

        ins_imu_snapshot(&imu);

        motor_link_ch_set(TRCH_STEER_TARGET,  steer.applied_delta);
        motor_link_ch_set(TRCH_STEER_DELTA,   steer.meas_delta);
        motor_link_ch_set(TRCH_STEER_U,       (float)steer.output_duty);
        motor_link_ch_set(TRCH_STEER_STATUS,
                          (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));

        motor_link_ch_set(TRCH_TARGET_L,      spd_target_l);
        motor_link_ch_set(TRCH_V_L,           enc_l.rps);
        motor_link_ch_set(TRCH_U_L,           ctl_l.u * 100.0f);
        motor_link_ch_set(TRCH_TARGET_R,      spd_target_r);
        motor_link_ch_set(TRCH_V_R,           enc_r.rps);
        motor_link_ch_set(TRCH_U_R,           ctl_r.u * 100.0f);

        motor_link_ch_set(TRCH_INS_GZ,        imu.gyro_z_dps);
        motor_link_ch_set(TRCH_INS_STATIC,
                          (float)((0u != (ins_imu_flags() & INS_ST_STATIC)) ? 1u : 0u));

        motor_link_ch_set(TRCH_ENABLE_BITS,   (float)motor_link_enable_bits());
        motor_link_ch_set(TRCH_SAFETY,        (float)spd_safety_flags());
        motor_link_ch_set(TRCH_RC_ENGAGE,     (float)RC_Engage);
        motor_link_ch_set(TRCH_TICK_MS,       (float)motor_link_now_ms());
        return;
    }

}

/* ---- 定点打印: 不用 %f, IAR 默认库不保证浮点格式化; pf2 返回整数部分, 小数写进 *fp ---- */
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

/* ---- 每 1 秒打印一次状态: 这行在跳即说明主循环、时钟、串口都活着 ---- */
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

    /* int = INT(P02_4) 电平, 1=空闲 0=忙(整帧丢弃); tx = 交给模块的帧数(应 50/s)。
       int=0 恒不变 = 查 INT 线或供电; int=1 而 tx 不涨 = 驱动 mutex 卡 BUSY; tx 涨而无数据 = WiFi/Socket。
       dL = 本拍计数增量, 应约 cpr*v*0.005 (1.4 rps -> 11~12); fL/gL = 累计换向/毛刺丢弃,
       fL 每秒涨几十以上 = DIR 线抖动; gL 在涨 = 库内读 DIR 与本模块不一致。 */
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

    /* 航向自检: yaw 为累积值, 转一圈应 +360; rej = 被单拍转角闸门丢掉的拍数, 非 0 先查 SPI_2 与供电 */
    printf("[##] IMU yaw=%ld deg  rej=%u\r\n",
           (long)ins_imu_get_yaw(), (unsigned)ins_imu_yaw_rej());

    /* 转向一行: 转角单位方向盘度; 不动方向盘时 raw 应稳定, delta 应近 0 */
    {
        int st_t, st_tf, st_d, st_df, st_e, st_ef;

        st_t  = pf2(steer.target_delta, &st_tf);
        st_d  = pf2(steer.meas_delta,   &st_df);
        st_e  = pf2(steer.err,          &st_ef);

        /* raw = 环里真正用的值(防抖后)   rl = 本拍 SPI 实读值   bad = 累计拒样   ar = 累计自动复位
           flt = 0x01 反馈失效(锁存, 钉输出) / 0x02 顶死告警 / 0x04 本拍丢样(瞬态) */
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

    /* 遥控一行, 不接管时按序查: FRM 不涨 = 接收机没发/线错/波特率非 SBUS 100k;
       CH1/CH2 不在 992 附近 = 摇杆未回中或量程不同; CH5/CH6 不在 192/1792 = 通道接错或极性宏反;
       ENG 恒 0 = CH6 未合闸或 rc_arm_en=0; ENG=3 = 失控, ENG=4 = 急停, ENG=2 车不动 = 查 SAFE */
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

        /* CH3 原始值: c3 不动 = 通道未接或未映射; bad 位图 bit0 CH1 / bit1 CH2 掉线 */
        printf("        RC3 c3=%u bad=%u yw=%d\r\n",
               (unsigned)RC_Raw_Ch3, (unsigned)RC_Stick_Bad,
               (int)steer_yaw_deg);
    }

    /* 惯导一行: imu_init 非 0 = IMU 未识别, 查 P15 接线与型号; yaw 静止仍走 = 看 bz; gz 静止超
       0.2 dps = 零偏未收敛或 ZUPT 未生效; x/y 反向 = INS_YAW_SIGN 与 inv_*; dist 差 k 倍 = ins_mpm 错 */
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

    clock_init(SYSTEM_CLOCK_250M);      /* 时钟配置及系统初始化, 务必保留 */
    debug_init();                       /* 调试串口信息初始化 */

    printf("[##] UART OK  steer + dual motor speed loop (WiFi)\r\n");

    motor_drv_init();                   /* 执行器: 三路 PWM/DIR(左后/右后/转向)上电均输出 0 */

    spd_enc_init();                     /* 编码器: 左右后轮脉冲 + 方向 */

    /* ---- 转向绝对编码器 + 转向位置环: 直流电机 + 绝对编码器, 靠位置环保持角度 ----
       steer_abs_init() 内含 SPI 自检(最长约 100 ms), 失败不阻塞只是角度不可信;
       默认全不使能, 需先标定再把转向内环使能置 1。 */
    steer_abs_init();
    steer_ctrl_init();

    spd_safety_init();                  /* 保护层: 内部跑一遍配置自检 */
    spd_ctrl_init();                    /* 速度环 */

    /* ---- 协议层: WiFi / 菜单 / 遥控 三条写入路径统一到 proto_exec(); 每圈调 proto_tick() 续租 ----
       须在 remote_ctrl_init() 之前, 后者会调 proto_estop()。 */
    proto_init();

    /* ---- 惯导: IMU + 航位推算; ins_imu_init() 阻塞约 5 s(1000 点 x 5 ms 零偏标定) ----
       期间车须静止, 极差超限重来, 最多 5 次; 须在 pit_ms_init() 之前调用, 否则中断里的
       ins_imu_update() 会用未标定零偏积分。963RA 与 IMU660RA/RC 引脚相同, 其 0x0F 应为 0x6A。 */
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

    /* ---- 遥控接收机 UART_4 / P14_0: SBUS 100 kbps, 6 通道 ----
       须在 pit_ms_init(PIT_CH2, 5) 之前(内部用 TC_TIME2_CH2 计时帧间隔); 接管时由
       remote_ctrl_apply() 调 spd_safety_set_local(1), 遥控不依赖 WiFi。 */
    remote_ctrl_init();

    if (0 != spd_safety_cfg_bad())
    {
        printf("[##] CONFIG BAD 0x%02X - motor stays off\r\n", (unsigned)spd_safety_cfg_bad());
    }

    /* 参数持久化: 从 Work Flash 页 0 读回上次 SAVE 的参数; 须在模块 init 之后、开 5 ms 中断之前。
       命令与危险量(target / ol_en / ol_duty / steer_en / ins_zero)不恢复, 上电为安全值。 */
    app_param_init();

    /* 路表: 读槽索引 -> 补空槽内建表 -> 载入 active 槽; 须在开 5 ms 中断之前(阻塞读 Flash),
       内建表只补空槽且须在 flash_load 之前。 */
    route_app_init();
    route_pb_init();

    spd_bench_init();                   /* 12 段跑班: 初始化, 开环默认关闭 */

    pit_ms_init(PIT_CH2, 5);            /* 5 ms 时基: 速度环 + 链路 */
    interrupt_global_enable(0);         /* pit0_ch2_isr 里跑 spd_ctrl_tick() */

    system_delay_ms(500);               /* 等 WiFi 模块上电稳定, 否则紧接着的 init 容易失败 */

    /* ---- WiFi-SPI 链路: 阻塞, WiFi 最长约 10 s, Socket 最长数十秒 ----
       失败返回非 0, 主循环照跑, 只是保护层永远不允许出力。 */
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

    /* ---- 屏幕 + 按键(菜单); 逐飞 IPS200 SPI 版(硬件 SPI_1), 6 脚由 zf_device_ips200.h 定死 ----
       SCL P12_2 / SDA(MOSI) P12_1 / CS P12_3 / RST P22_4 / DC P22_3 / BLK P11_0
       其它模块不占这 6 脚: 转向编码器 SPI_4(P07_x)、WiFi SPI_0(P02_x)、IMU SPI_2(P15_x)。
       按键 P20_3 / P20_2 / P20_0 / P20_1; 须在 motor_link_init() 之后, key_init(10) 与
       motor_link_now_ms() 的 10 ms 分频一致; 刷新毫秒级(清屏约 40 ms), 只能在主循环画,
       ui_init() 与 ui_menu_init() 缺一即黑屏。 */
    ui_init();
    ui_puts_row(0, 0, "Kart10 INS+Route");
    printf("[##] UI init done: IPS200 SPI_1 (SCL P12_2 / SDA P12_1 / CS P12_3 / DC P22_3 / RST P22_4 / BLK P11_0)\r\n");
    ui_menu_init();

    while (true)
    {
        /* 0) 遥测档案: tel_sel 0=巡线 22 / 1=标定 18 / 2=全量 45 / 3=采集 16; 上位机档案须同步切 */
        motor_link_set_ch_count((spd_tel_sel < 0.5f) ? MOTOR_CH_COUNT_ROUTE :
                                ((spd_tel_sel < 1.5f) ? MOTOR_CH_COUNT_CALIB :
                                 ((spd_tel_sel < 2.5f) ? MOTOR_CH_COUNT_FULL : MOTOR_CH_COUNT_TRAIN)));
        motor_link_set_tx_period_ms((uint16)spd_tel_period);

        /* 1) 链路任务: 收上位机命令 + 发遥测。每圈必调, 且不可放在可能失败的分支里 */
        motor_link_task();

        /* 2) 主循环心跳: 超过 200 ms 无心跳, 保护层把两路输出钉 0 */
        spd_safety_heartbeat();

        /* 2b) 遥控: 收快照 -> 换算 -> 接管状态机 -> 写目标与本地档位; 属慢速决策层,
               5 ms 控制仍在 pit0_ch2_isr。不可做成阻塞页面, 否则心跳断、输出被钉 0。 */
        remote_ctrl_update();
        remote_ctrl_apply();

        /* 2c) 协议层心跳: 检查驱动令牌租期, 到期释放; 须在 route_pb_poll() 之前跑 */
        proto_tick();

        /* 航向测试跑在 5 ms 中断里, 不可从那里碰协议层; 此处替它续 SPI 驱动权 */
        if (0u != head_test_active()) { (void)proto_drv_acquire(PROTO_SRC_SPI); }

        /* 2d) 跑班收尾 + 日志落盘 + 菜单; 放在心跳之后(刷屏最差约 40 ms), 日志每圈最多一页 Flash */
        spd_bench_poll();
        bench_log_poll();
        route_app_poll();       /* 路表: 录制采样 + 串口灌表 + 每圈一页落盘 */
        route_pb_poll();        /* 路表: 回放状态机, 不给速度时只算目标航向 */
        ui_menu_task();

        /* 2e) 惯导慢动作: 位姿归零 + IMU 掉线重试(1 Hz)。归零须位姿与航向一起归,
               否则路表与回放的相对量整体错位; 重试只在链路坏时做, 那时中断不碰 SPI_2。 */
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

        /* 3) 转向环在 pit0_ch2_isr(5 ms 中断)里跑, 此处不再调用; 两处都调会使目标斜坡走两倍速 */

        /* 4) 填遥测 + 打印状态 */
        motor_fill_telemetry();
        motor_print_status();

        system_delay_ms(1);
    }
}
