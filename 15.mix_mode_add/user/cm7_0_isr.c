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
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cm7_0_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
* 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
* 2025-2-4      pudding            新增两个串口接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor_link.h"
#include "steer_abs.h"
#include "steer_ctrl.h"
#include "ins_imu.h"
#include "ins_odom.h"
#include "spd_bench.h"
#include "head_test.h"
#include "route_link.h"      /* route_link_uart_irq() */

/* ---- PIT 中断服务函数 ---- */
void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH0);

}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);

}

void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH2);

    motor_link_tick_5ms();                 // WiFi-SPI 链路的 5ms 时基

    /* 推车自由轮: 每拍按用户意图决定, 不能依赖 route_pb_poll() 的状态机 —— 那里每个安全门
       (无表/惯导关/遥控失控)都会提前返回, 速度目标被压成 0, 车就推不动。必须在 spd_ctrl_tick() 之前调。 */
    route_pb_sync_coast();

    spd_ctrl_tick();                       // 速度环 5ms 节拍: 保护 + 测速 + 控制 + 输出

    /* 转向位置环 5ms 节拍: 先刷新绝对角(SPI_4/SCB5, 与 WiFi 的 SPI_0 是两条独立硬件), 再跑位置环
       (纯 P + 静摩擦前馈)。顺序不能反, 否则内环吃到上一拍角度; 放中断才能保证严格 5ms 周期。 */
    steer_abs_update();

    /* 航向外环反馈: 用上一拍 odom 航向(本拍 yaw 稍后才更新, 5ms 滞后可忽略);
       必须用 odom 而非 imu 的 yaw —— 路表/回放都在 odom 坐标系, 两处要同源。 */
    steer_yaw_deg = ins_odom_get_yaw();

    steer_ctrl_update();

    /* 惯导两拍(顺序是硬约束): 先读 IMU(SPI_2 / P15_0~3) 出本拍 yaw, 再用本拍编码器 delta 积分 x/y。
       必须在 spd_ctrl_tick() 之后(那里 spd_enc_update() 才更新 delta); IMU 未接时内部直接 return, 不阻塞中断。 */
    ins_imu_update();
    ins_odom_update();

    /* 航向测试: 要读本拍 yaw 与里程量 上升时间/段内里程, 故放在上面两拍之后;
       只写 target_yaw / spd_target_*, 出力仍在 steer_ctrl_update() 与 spd_ctrl_tick()。 */
    head_test_tick();

    /* 12 段跑班: 写 spd_ol_duty_*, 必须在 spd_ctrl_tick() 之后才变成 PWM;
       每 5ms 复核四道安全门(遥控在线/STANDBY/CH5 急停/保护层), 破了立即回 0 duty 并退出开环。 */
    spd_bench_tick();


}

void pit0_ch10_isr()                    // 定时器通道 10 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH10);

}

void pit0_ch11_isr()                    // 定时器通道 11 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()                    // 定时器通道 12 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()                    // 定时器通道 14 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()                    // 定时器通道 15 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()                    // 定时器通道 16 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()                    // 定时器通道 17 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()                    // 定时器通道 18 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()                    // 定时器通道 19 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()                    // 定时器通道 20 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()                    // 定时器通道 21 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}

/* ---- 串口中断 ---- */
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {

#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        route_link_uart_irq();           /* 串口灌表(route_link) 的接收口 */
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去

    }
    else                                 // 串口0发送中断
    {

    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {

        wireless_module_uart_handler();  // 无线模块统一回调函数

    }
    else                                // 串口1发送中断
    {

    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {

        gnss_uart_callback();            // GPS模块回调函数

    }
    else                                // 串口2发送中断
    {

    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {

    }
    else                                // 串口3发送中断
    {

    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收机回调函数

    }
    else                                // 串口4发送中断
    {

    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {

    }
    else                                // 串口5发送中断
    {

    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {

    }
    else                                // 串口6发送中断
    {

    }
}

/* ---- 外部中断 ---- */
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数
{

}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

    }
    if(exti_flag_get(P01_1))
    {

    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数
{
    if(exti_flag_get(P02_0))
    {


    }
    if(exti_flag_get(P02_4))
    {


    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数
{

}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数
{

}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数
{

}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数
{
    if(exti_flag_get(IMU660RC_INT2_PIN))
    {
        imu660rc_callback();
    }
}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数
{

}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数
{

}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数
{

}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数
{

}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数
{

}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数
{

}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数
{

}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数
{

}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数
{

}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数
{

}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数
{

}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数
{

}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数
{

}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数
{

}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数
{

}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数
{

}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数
{

}
