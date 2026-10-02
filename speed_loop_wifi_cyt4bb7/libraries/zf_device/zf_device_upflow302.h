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
#ifndef _zf_device_upflow302_h
#define _zf_device_upflow302_h

#include "zf_common_headfile.h"


#define UP_FLOW_302_UART_INDEX            (UART_2)              // 定义 UP-FLOW-302 光流模块 使用的串口
#define UP_FLOW_302_BAUDRATE              (19200)               // 指定 UP-FLOW-302 光流模块 串口所使用的的串口波特率 (用户不可修改)
#define UP_FLOW_302_TX_PIN                (UART2_RX_P10_0)      // UP-FLOW-302 光流模块 的 TX 引脚 连接单片机的 RX 引脚 
#define UP_FLOW_302_RX_PIN                (UART2_TX_P10_1)      // UP-FLOW-302 光流模块 的 RX 引脚 连接单片机的 TX 引脚 


#define UP_FLOW_302_DATA_LEN              ( 14   )              // UP-FLOW-302 光流模块 的 帧长
#define UP_FLOW_302_FRAME_STAR            ( 0XFE )              // UP-FLOW-302 光流模块 的 帧头信息
#define UP_FLOW_302_DEVICE_ID             ( 0X0A )              // UP-FLOW-302 光流模块 的 帧尾信息
                                                                
#define UP_FLOW_302_TIMEOUT_COUNT         ( 0x00FF )            // UP-FLOW-302 光流模块 的 超时计数


typedef struct
{
    uint8 head;                                                 // 帧头
    uint8 device_id;                                            // 设备id
        
    int16 upflow302_x;                                          // 光流_x
    int16 upflow302_y;                                          // 光流_y
    int16 upflow302_us;                                         // 光流_时间差
    int16 upflow302_us_a;                                       // 预留位
        
    uint8  upflow302_valid;                                     // 光流_状态 0不可用  245  可用
    uint8  upflow302_version;                                   // 光流_版本号
        
    uint8 sum_check;                                            // 和校验
    uint8 sum_end;                                              // 和校验

}upflow302_receive_struct;


extern upflow302_receive_struct upflow302_receive;              // UP-FLOW-302 光流模块 通道数据与状态

extern uint8   upflow302_receiver_data[UP_FLOW_302_DATA_LEN];   // UP-FLOW-302 光流模块 接收原始数据
extern uint8   upflow302_finsh_flag;
extern uint8   upflow302_state_flag;                            // UP-FLOW-302 光流模块 状态(1表示正常，否则表示无效)
extern uint16  upflow302_response_time;

extern int16 upflow302_x,upflow302_y;


void upflow302_receive_callback(void);

void upflow302_receive_init(void);

#endif
