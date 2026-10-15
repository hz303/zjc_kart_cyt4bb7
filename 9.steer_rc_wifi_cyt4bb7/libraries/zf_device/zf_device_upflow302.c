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
* 文件名称          zf_device_upflow302
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-4-28       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   TX                查看 zf_device_upflow302.h 中 UP_FLOW_302_RX_PIN 宏定义
*                   RX                查看 zf_device_upflow302.h 中 UP_FLOW_302_TX_PIN 宏定义
*                   GND               电源地
*                   5V                5V电源
*                   ------------------------------------
********************************************************************************************************************/
#include "zf_device_upflow302.h"

uint8   upflow302_receiver_data[UP_FLOW_302_DATA_LEN]  = {0};    // UP-FLOW-302 光流模块 接收原始数据

uint8  upflow302_finsh_flag = 0;                                    
uint8  upflow302_state_flag = 1;                                    
uint16  upflow302_response_time = 0;

upflow302_receive_struct upflow302_receive;    

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UP-FLOW-302 光流模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     upflow302_receiver_callback();
// 备注信息     该函数在 ISR 文件 指定的串口接收中断函数内调用
//-------------------------------------------------------------------------------------------------------------------
void upflow302_receive_callback(void)
{
    static vuint8 length  = 0;
    uint8  parity_bit_sum = 0;
    uint8  parity_bit     = 0;
    uint8  receive_data   = 0;
    
    if(uart_query_byte(UP_FLOW_302_UART_INDEX, &receive_data))
    {
        upflow302_receiver_data[length++] = receive_data;

        if((1 == length) && (UP_FLOW_302_FRAME_STAR != upflow302_receiver_data[0])) // 起始位判断
        {
            length =  0;
        }                                           

        if(UP_FLOW_302_DATA_LEN <= length)                                      // 数据长度判断
        {
            parity_bit = upflow302_receiver_data[12];
            
            upflow302_receiver_data[12] = 0;

            for(int  i = 2; i < 12; i ++)
            {
                parity_bit_sum ^= upflow302_receiver_data[i];
            }

            if (parity_bit_sum == parity_bit)                                   // 和校验判断
            {
                upflow302_finsh_flag = 1;
                upflow302_state_flag = 1;
                upflow302_response_time = 0;
                upflow302_receiver_data[12]= parity_bit;
                // 将接收到的数据拷贝到结构体中
                memcpy((uint8*)&upflow302_receive, (uint8*)upflow302_receiver_data, sizeof(upflow302_receiver_data));
            }
            else
            {
                upflow302_finsh_flag = 0;
            }
            
            parity_bit_sum = 0;
            
            length = 0;
        }
    }
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口接收机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     upflow302_receiver_init();
//-------------------------------------------------------------------------------------------------------------------
void upflow302_receive_init(void)
{
    uart_init(UP_FLOW_302_UART_INDEX, UP_FLOW_302_BAUDRATE, UP_FLOW_302_RX_PIN, UP_FLOW_302_TX_PIN);
    uart_rx_interrupt(UP_FLOW_302_UART_INDEX, 1);
}
