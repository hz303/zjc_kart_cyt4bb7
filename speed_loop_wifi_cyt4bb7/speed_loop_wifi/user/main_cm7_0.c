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
* 本工程由 Example/Motherboard_Demo/E3_motor 组合裁剪而来：
*   基座 = 4.motor_wifi_verify_cyt4bb7（WiFi-SPI 链路 + 上位机 KartHost 通信，已验证跑通）
*   控制 = 单电机速度环的 LQI 结构 + 保护，扩成**左右双电机闭环**
*
* 与基座工程的差别：**取消了开环占空比通道**。上位机只下发"目标转速 rps"，
* 闭环完全在车端跑。开环标定请用另一个工程 5.speed_loop_vofa_cyt4bb7（串口 + VOFA+）。
*
* ---- 硬件连接 -----------------------------------------------------------------------
*   左电机:  PWM -> P10_2 (TCPWM_CH30)    DIR -> P10_3
*   右电机:  PWM -> P05_2 (TCPWM_CH11)    DIR -> P05_3
*   左编码器: PULSE -> P17_3 (TC_CH58 CH1)   DIR -> P17_4 (TC_CH58 CH2)
*   右编码器: PULSE -> P19_2 (TC_CH27 CH1)   DIR -> P19_3 (TC_CH27 CH2)
*   WiFi-SPI 模块: 见 motor_link.h（SPI_0 + INT P02_4 + RST P23_0）
*
* ---- 两个必须先做的台架标定（顺序不能颠倒）-----------------------------------------
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
}

int main (void)
{
    uint8   ret;

    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化

    /* 最底层的"我还活着"标记：只要串口参数对，上电第一秒就能看到这一行。
       把"时钟/引脚/波特率对不对"和"软件逻辑跑没跑"分开，排障时能省很多事。 */
    printf("[##] UART OK  dual motor speed loop (WiFi)\r\n");

    /*----------------------------- 执行器：先钉在 0 -----------------------------*/
    motor_drv_init();                   /* 两路 PWM/DIR 都输出 0，且这条路走通了 */

    /*----------------------------- 编码器 -----------------------------*/
    spd_enc_init();

    /*----------------------------- 保护与控制器 -----------------------------*/
    spd_safety_init();                  /* 内部会跑一遍配置自检 */
    spd_ctrl_init();

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
        /* 1) 链路任务：收上位机命令 + 发遥测。
              必须在循环里每圈都调到 —— WiFi 的收发都在这里推进，
              而且要放在任何"可能失败的分支"之外，否则失败路径会把它停掉，
              表现就是"上位机再也收不到数据"却查不出原因。 */
        motor_link_task();

        /* 2) 主循环心跳：给保护层证明"主循环没卡死"。
              超过 200ms 没心跳，保护层直接把两路输出钉 0。 */
        spd_safety_heartbeat();

        /* 3) 填遥测 + 打印状态 */
        motor_fill_telemetry();
        motor_print_status();

        system_delay_ms(1);
    }
}
