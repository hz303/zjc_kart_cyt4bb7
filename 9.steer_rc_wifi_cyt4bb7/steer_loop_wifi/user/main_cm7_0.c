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
* 本工程 = 工程 7(steer_loop_wifi) + **遥控(工程 9 新增)**。
*   基座 = 4.motor_wifi_verify_cyt4bb7（WiFi-SPI 链路 + 上位机 KartHost 通信，已验证跑通）
*   控制 = 转向位置环 + 双后轮速度环（与工程 7 同）
*   新增 = 逐飞 UART 接收机(SBUS) + remote_ctrl：**遥控独立于 WiFi**，
*          插不插 WiFi 都能用发射机完整操控（说明见 remote_ctrl.h）
*
* 与基座工程的差别：**取消了开环占空比通道**。上位机只下发"目标转速 rps"，
* 闭环完全在车端跑。开环标定请用另一个工程 5.speed_loop_vofa_cyt4bb7（串口 + VOFA+）。
*
* ---- 硬件连接（★ 2026-10-13 按队友 Karting-Project commit ca55f2b 更新到【最新线束】）----
*   左后轮:  PWM -> P09_0 (TCPWM_CH24)    DIR -> P09_1
*   右后轮:  PWM -> P10_2 (TCPWM_CH30)    DIR -> P10_3
*   转向  :  PWM -> P05_0 (TCPWM_CH09)    DIR -> P05_1
*   (工程 7 用的那一版是 P05_2/P05_3 + P05_1/P05_0 + P10_2/P10_3，**已作废**；
*    引脚一变, minv_l / minv_r / minv_s 三个极性必须重新核, 判据见 motor_drv.h 末尾)
*   左后轮编码器: PULSE -> P17_3 (TC_CH58 CH1)   DIR -> P17_4 (TC_CH58 CH2)  [脉冲+方向]  ← 没变
*   右后轮编码器: PULSE -> P19_2 (TC_CH27 CH1)   DIR -> P19_3 (TC_CH27 CH2)  [脉冲+方向]  ← 没变
*   转向绝对角编码器: SPI_4(SCB5)  CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3  ← 没变
*   WiFi-SPI 模块: 见 motor_link.h（SPI_0 + INT P02_4 + RST P23_0）
*   遥控接收机  : UART_4 = SCB2，RX = P14_0（接接收机的 TX），TX = P14_1（占位，悬空）
*                 SBUS 100 kbps，6 通道；帧间隔计时占 TC_TIME2_CH2。见 zf_device_uart_receiver.h
*
* ---- ★ 三个必须先做的台架标定（顺序不能颠倒）--------------------------------------
*   第零步 硬件极性（**换脚之后一定要重做, 这是唯一会烧东西的一步**）:
*           车架空 -> 上位机把「转向|内环使能」打开 -> 先只给 steer_u0 看方向盘能不能被推动,
*           再给一个很小的目标角(比如 +3 度), 看方向盘是"朝目标去"还是"背着目标跑"。
*           背着跑 = mot_inv_s 给反了 => **立刻停手**, 把「转向|电机极性」改成 0 再试。
*           (后果: 正反馈会一路顶住机械限位堵转, 把 DRV8701 烧掉 —— 已经烧过一个。)
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

/*===================================================================================================================
 *  内部状态
 *=================================================================================================================*/
static uint32   g_print_ms = 0;        /* 上次打印状态的时刻 */

/*===================================================================================================================
 *  遥测通道填充（顺序即协议，见 motor_link.h 第四节）
 *=================================================================================================================*/
static void motor_fill_telemetry (void)
{
    uint32 age;

    /* ★★★ 标定精简档案（16 通道）—— 由参数 `tel_sel`（>0.5）切进来，见 motor_link.h 第五节。
       落地标定要长时间高密度采数据，34/37 通道一帧把带宽吃满；16 通道一帧只有 85 字节，
       同样的链路能把遥测周期压到 5~10 ms。
       [!] 这里**直接 return**：标定档的通道号(CT_*)与全量档(CH_*)完全不是一回事，
         绝不能"填一半再叠一半"，否则上位机看到的是两份数据混在一起。
       [!] 切档时上位机的"档案"下拉框也要跟着切到「标定·精简」，否则通道名错位。 */
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
        /* ★ 发【实际施加】的目标（软限位夹紧 + 斜坡之后），不是上位机写的命令 ——
           理由同增益回读：斜坡开着时若发命令，波形上是一条干净阶跃，
           会把"斜坡造成的滞后"算到机械/增益头上。见 motor_link.h 的 CH_STEER_TARGET 注解。 */
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
        /* ★ 增益回读: 发的是控制器本拍真正乘上去的那两个数（见 motor_link.h 的 CH_K_V 注解）——
           "滑条上的数"和"车上真正跑的数"不是一回事，这两个通道就是唯一权威。 */
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
    motor_link_ch_set(CH_DLT_L,       (float)enc_l.delta);
    motor_link_ch_set(CH_DLT_R,       (float)enc_r.delta);
    motor_link_ch_set(CH_FLIP_L,      (float)enc_l.flip);
    motor_link_ch_set(CH_FLIP_R,      (float)enc_r.flip);
    motor_link_ch_set(CH_ONLINE,      (float)motor_link_online());
    motor_link_ch_set(CH_RX_AGE_MS,   (float)age);
    motor_link_ch_set(CH_ENABLE_BITS, (float)motor_link_enable_bits());
    motor_link_ch_set(CH_SAFETY,      (float)spd_safety_flags());

    /* --- 转向(20~25) --- */
    motor_link_ch_set(CH_STEER_TARGET, steer.applied_delta);   /* ★ 实际施加目标，非上位机写的命令 */
    motor_link_ch_set(CH_STEER_DELTA,  steer.meas_delta);
    motor_link_ch_set(CH_STEER_U,      (float)steer.output_duty);
    motor_link_ch_set(CH_STEER_RAW,    (float)steer_abs_get_raw());
    motor_link_ch_set(CH_STEER_EN,     (float)steer.angle_enable);
    /* bit0 = 绝对编码器自检通过; bit1/bit2 = steer_ctrl 的主动保护锁存
       (0x02 反馈失效 / 0x04 顶死或反转) —— 方向给反时这一位会亮, 是现场第一判据。 */
    motor_link_ch_set(CH_STEER_STATUS,
                      (float)(steer_abs_self_check_ok() | ((uint16)steer_ctrl_fault() << 1)));

    /* --- ★ 遥控(26~36): 2026-10-13 新增。通道含义见 remote_ctrl.h 的通道分配表 --- */
    motor_link_ch_set(CH_RC_ENGAGE,    (float)RC_Engage);
    motor_link_ch_set(CH_RC_ESTOP,     (float)RC_Estop);
    motor_link_ch_set(CH_RC_STEER_DEG, RC_Steer_Deg);
    motor_link_ch_set(CH_RC_THR,       RC_Throttle * 1000.0f);
    motor_link_ch_set(CH_RC_SCALE,     RC_Scale * 1000.0f);
    motor_link_ch_set(CH_RC_TGT_RPS,   RC_Target_Rps);
    motor_link_ch_set(CH_RC_CH1,       (float)RC_Raw_Ch1);
    motor_link_ch_set(CH_RC_CH2,       (float)RC_Raw_Ch2);
    motor_link_ch_set(CH_RC_CH5,       (float)RC_Raw_Ch5);
    motor_link_ch_set(CH_RC_CH6,       (float)RC_Raw_Ch6);
    motor_link_ch_set(CH_RC_FRAMES,    (float)RC_Frames);

    /* --- ★★ 增益回读(37~38): 2026-10-14 新增。理由见 motor_link.h 的 CH_K_V 注解 --- */
    motor_link_ch_set(CH_K_V,          spd_g.k_v);
    motor_link_ch_set(CH_K_Z,          spd_g.k_z);
}

/*===================================================================================================================
 *  定点打印小工具
 *
 *  刻意不用 %f：IAR 的 printf 在默认库配置下不一定支持浮点格式化，
 *  不值得为了一行日志去赌链接器的 formatter 配置。
 *=================================================================================================================*/
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
/*===================================================================================================================
 *  每 1 秒打印一次状态
 *
 *  这也是"出了事能自检"的兜底手段：只要这行在跳，就说明主循环活着、时钟活着、串口活着。
 *=================================================================================================================*/
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
              (发送前会看这一位, 为 0 就整帧丢掉 -> 一个字节都不上 SPI)
       tx   = 上电以来真正交给模块的遥测帧数, 应匀速增长(50/s)
       ★ 判读: int=0 一直不变 -> 查 INT 这根线/模块供电;
               int=1 但 tx 不涨 -> 驱动内部 mutex 卡 BUSY;
               tx 在涨但上位机没数 -> WiFi/Socket(看 err=) 或目标 IP 不对 */
    /* dL = 本拍(5ms)计数增量   fL = 真换向次数累计   gL = 读数毛刺丢弃次数累计
       ★ dL 应该稳定在 cpr*v*0.005 附近(1.4rps -> 11~12); fL/gL 每秒只该 +0~1
         若 fL 每秒涨几十上百 -> DIR 线在抖; 若 gL 在涨 -> 库内读 DIR 与我们读的不一致 */
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

    /* 转向一行: 目标/实测转角、输出 duty、绝对编码器原始角与自检结果。
       排障第一现场 —— 不动方向盘时 raw 应该稳定、delta 应该接近 0。 */
    {
        /* 转角单位已改成"方向盘度", 用 pf2 打两位小数; 误差也打出来, 调 kp/u0 时最直观。 */
        int st_t, st_tf, st_d, st_df, st_e, st_ef;

        st_t  = pf2(steer.target_delta, &st_tf);
        st_d  = pf2(steer.meas_delta,   &st_df);
        st_e  = pf2(steer.err,          &st_ef);

        printf("        STEER tgt=%d.%02d delta=%d.%02d err=%d.%02d duty=%d raw=%u en=%u flt=0x%02X sc=%u\r\n",
               st_t, st_tf,
               st_d, st_df,
               st_e, st_ef,
               (int)steer.output_duty,
               (unsigned)steer_abs_get_raw(),
               (unsigned)steer.angle_enable,
               (unsigned)steer_ctrl_fault(),
               (unsigned)steer_abs_self_check_ok());
    }

    /* 遥控一行: 接管状态 / 两个开关 / 指令量 / 收帧数 / 遥测档。
       ★ 判读(不接管时按这个顺序查):
           FRM 不涨             -> 接收机没在发 / 线接错 / 波特率不对(应 SBUS 100k)
           CH1/CH2 不是 992 附近 -> 摇杆微调没回中, 或接收机型号不是 HotRc(量程不同, 要重标)
           CH5/CH6 不是 192/1792 -> 开关通道接错, 或极性宏 RC_*_ACTIVE_HIGH 给反
           ARM=0 但 CH6 已合闸    -> 上位机把 rc_arm_en 写成 0 了
           ENG 恒 0             -> CH6 没收/收反, 或 rc_arm_en = 0
           ENG 恒 3(FAILSAFE)   -> 接收机报失控(ST=0): 先看发射机与接收机对码/供电
           ENG 恒 4(ESTOP)      -> 急停开关 CH5 在"生效"位置, 或者极性宏给反了
           ENG=2 但车不动        -> 看 SAFE 位图(SAFE_BIT_LOCAL 没亮 = 本地档位没推上;
                                    SAFE_BIT_ESTOP=0x80 亮 = 急停还按着) */
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
}

int main (void)
{
    uint8   ret;

    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化

    /* 最底层的"我还活着"标记：只要串口参数对，上电第一秒就能看到这一行。
       把"时钟/引脚/波特率对不对"和"软件逻辑跑没跑"分开，排障时能省很多事。 */
    printf("[##] UART OK  steer + dual motor speed loop (WiFi)\r\n");

    /*----------------------------- 执行器：先钉在 0 -----------------------------*/
    motor_drv_init();                   /* 三路 PWM/DIR(左后/右后/转向)都输出 0，且这条路走通了 */

    /*----------------------------- 编码器 -----------------------------*/
    spd_enc_init();

    /*--------------------- 转向绝对编码器 + 转向位置环 ---------------------
       转向是"直流电机 + 绝对编码器"(不是会自动回中的 RC 舵机), 必须靠位置环把它按住。
       引脚按队友的飞线方案: SPI_4(SCB5) CLK P07_2 / MOSI P07_1 / MISO P07_0 / CS P07_3。
       注意1: steer_abs_init() 里带一次 SPI 自检(最多约 100ms), 失败不会卡住, 只是"角度不可信"。
       注意2: 默认【全不使能】—— 要动方向盘得先标定(见 01-上电标定卡.md 转向一节),
              然后上位机把"转向|内环使能"打 1。 */
    steer_abs_init();
    steer_ctrl_init();

    /*----------------------------- 保护与控制器 -----------------------------*/
    spd_safety_init();                  /* 内部会跑一遍配置自检 */
    spd_ctrl_init();

    /*------------------------ 遥控接收机(UART_4 / P14.0) ------------------------
       逐飞 UART 接收机(SBUS, 100 kbps, 6 通道)。
       [!] 必须在 pit_ms_init(PIT_CH2, 5) 之前 —— 它内部要用 TC_TIME2_CH2 做帧间隔计时。
       [!] 遥控【不依赖 WiFi】: 接管成立时由 remote_ctrl_apply() 调 spd_safety_set_local(1),
         把保护层那三道"上位机在场"的门换掉。见 remote_ctrl.h。 */
    remote_ctrl_init();

    if (0 != spd_safety_cfg_bad())
    {
        printf("[##] CONFIG BAD 0x%02X - motor stays off\r\n", (unsigned)spd_safety_cfg_bad());
    }

    /*------------------------- 5ms 时基：速度环 + 链路 -------------------------*/
    pit_ms_init(PIT_CH2, 5);
    interrupt_global_enable(0);         /* pit0_ch2_isr 里跑 spd_ctrl_tick() */

    /* 先给模块一点上电稳定时间：WiFi 模块刚上电时 SPI 还没准备好，
       紧接着就 init 容易读到失败（原厂例子是"上电后按键触发"，天然有几秒间隔）。 */
    system_delay_ms(500);

    /*--------------------------- WiFi-SPI 链路 ---------------------------
       这个函数会阻塞：WiFi 最长等约 10 秒、Socket 最长等数十秒。
       失败也不会卡死在这里（返回非 0），主循环照样跑，只是保护层永远不允许出力。 */
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

    while (true)
    {
        /* 0) ★ 标定档案切换：把参数 tel_sel / tel_period 推给链路层（不用重烧）。
              tel_sel=0 -> 全量 37 通道；=1 -> 标定精简 16 通道。
              [!] 通道数一变，帧头 COUNT 跟着变，上位机的档案下拉框要同步切。 */
        motor_link_set_ch_count((spd_tel_sel > 0.5f) ? MOTOR_CH_COUNT_CALIB : MOTOR_CH_COUNT);
        motor_link_set_tx_period_ms((uint16)spd_tel_period);

        /* 1) 链路任务：收上位机命令 + 发遥测。
              必须在循环里每圈都调到 —— WiFi 的收发都在这里推进，
              而且要放在任何"可能失败的分支"之外，否则失败路径会把它停掉，
              表现就是"上位机再也收不到数据"却查不出原因。 */
        motor_link_task();

        /* 2) 主循环心跳：给保护层证明"主循环没卡死"。
              超过 200ms 没心跳，保护层直接把两路输出钉 0。 */
        spd_safety_heartbeat();

        /* 2b) ★ 遥控: 读接收机快照 -> 换算 -> 跑接管状态机 -> 写目标 + 推 spd_safety 本地档位。
               放主循环(不是中断)是刻意的: 它写的 steer_en_cmd / spd_target_* 本来就属于
               "慢速决策层", 真正的 5ms 控制仍在 pit0_ch2_isr 里。
               [!] 绝不能把遥控做成 while(1) 页面 —— 那样主循环心跳就断了,
                 保护层会在 200ms 后把电机全部钉 0(这正是队友那份工程的坑)。 */
        remote_ctrl_update();
        remote_ctrl_apply();

        /* 3) 转向环已挪到 pit0_ch2_isr(5ms 中断)里跑, 见 cm7_0_isr.c —— 这里不再调用。
              挪走的理由: 主循环周期被 WiFi 收发拖成几毫秒的抖动, 而目标斜坡(度/秒)
              与误差死区都依赖真实周期; 放中断才能保证严格 5ms。
              千万别两处都调, 否则同一拍跑两遍, 斜坡会走两倍速。 */

        /* 4) 填遥测 + 打印状态 */
        motor_fill_telemetry();
        motor_print_status();

        system_delay_ms(1);
    }
}
