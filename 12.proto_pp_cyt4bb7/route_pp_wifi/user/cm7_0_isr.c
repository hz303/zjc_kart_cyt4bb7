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



// **************************** PIT中断函数 ****************************
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
    
    // PUSH free-wheel must be decided every tick from the USER INTENT, not from
    // route_pb_poll() (main loop, many early-return safety gates). If it depended on
    // the state machine, any gate (no table / odom off / RC failsafe) would leave the
    // speed loop holding target=0 -> the car cannot be pushed.
    // Call it BEFORE spd_ctrl_tick() so it takes effect in the same tick.
    route_pb_sync_coast();

    spd_ctrl_tick();                       // 速度环 5ms 节拍: 保护 + 测速 + 控制 + 输出

    // 转向位置环 5ms 节拍: 先刷新绝对角(SPI_4/SCB5, 与 WiFi 的 SPI_0 是两条独立硬件, 不冲突),
    // 再跑位置环(纯 P + 静摩擦前馈, 见 steer_ctrl.c)。顺序不能反, 否则内环吃到的是上一拍的角度。
    // 放在中断里是为了让控制周期严格等于 5ms —— 主循环周期会被 WiFi 收发拖出几毫秒抖动,
    // 而目标斜坡(度/秒)与误差死区都依赖真实周期。
    steer_abs_update();

    /* ★★ 2026-10-20 修：航向外环的**实测输入**一直没人写。
       steer_ctrl.c 的航向环用 steer_yaw_deg 当反馈（δ = -kp*(target_yaw - steer_yaw_deg)），
       而该变量从工程 9 起恒 0（头文件写着"接上 IMU 后由 IMU 模块每拍写" —— 一直没接）。
       => pb_smode=0（默认）和航向测试里的"航向环"实际是**开环打角**：目标航向给出一个固定舵角，
         完全没有用到车真正朝哪。
       这里用**上一拍**的航位推算航向：本拍 yaw 要等下面 ins_odom_update() 才更新，
       5 ms 滞后对航向环可忽略。
       [!] 用 odom.yaw 而不是 imu yaw：路表/回放都在 odom 坐标系里，两处必须同源。 */
    steer_yaw_deg = ins_odom_get_yaw();

    steer_ctrl_update();

    // ---- 工程 10: 惯导两拍（顺序是硬约束，别调换）----
    //  (1) ins_imu_update(): 读 IMU(SPI_2 / P15_0~3) -> Mahony -> 累积 yaw（本拍的新 yaw）
    //  (2) ins_odom_update(): 用【本拍】spd_enc 的 delta + (1) 的新 yaw 积分出 (x,y)
    //  !! 必须在 spd_ctrl_tick() 之后 —— 那里面的 spd_enc_update() 才是本拍的 delta。
    //  !! 也必须在 ins_imu_update() 之后 —— 用的是同一拍的 yaw，不是上一拍的。
    //  !! IMU 没接好时 ins_imu_update() 内部直接 return（完全不碰 SPI_2），
    //     所以「IMU 掉了」不会把 5ms 中断拖住 —— 这是中断里访问外部器件的唯一硬约束。
    ins_imu_update();
    ins_odom_update();

    // ---- 工程 11: 航向测试（App 两个测试的段序列与计时都在这里）----
    //  放在 ins_odom_update() 之后: 它要读本拍的 yaw / 里程来量"上升时间 + 段内里程"。
    //  它只写 target_yaw / spd_target_*，真正的出力仍在 steer_ctrl_update() / spd_ctrl_tick()。
    //  [顺序说明] 写在 steer_ctrl_update() 之后不是笔误: 目标是给下一拍用的，5ms 滞后对
    //     "上升时间"这种几十~几百毫秒的量可忽略; 插到它前面会打乱"先采角度再算环"的顺序。
    head_test_tick();

    // ---- 工程 10: 12 段跑班（一键跑班）----
    //  必须在 spd_ctrl_tick() 之后：开环占空比是本函数写进 spd_ol_duty_* 的，
    //  而 spd_ctrl_tick 才是真正把它变成 PWM 的地方。
    //  内部每 5ms 复核四道安全门（遥控在线 / STANDBY / CH5 急停 / 保护层），
    //  任一条破了立刻回 0 duty + 退出开环 + 中止记录。没在跑班时它直接 return。
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
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        route_link_uart_irq();           /* 2026-10-21: 串口灌表(route_link)的接收口 */   /* 2026-10-21: 串口灌表(route_link) 的接收口；debug 控制台输入没人用，见工程 8 同款接法 */
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
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
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
// **************************** 外部中断函数 ****************************