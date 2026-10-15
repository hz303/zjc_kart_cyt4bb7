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
* 文件名称          zf_device_pmw3901
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-03-09       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   SCLK               查看 zf_device_pmw3901.h 中 PMW3901_SCLK_PIN 宏定义
*                   MOSI               查看 zf_device_pmw3901.h 中 PMW3901_MOSI_PIN 宏定义
*                   MISO               查看 zf_device_pmw3901.h 中 PMW3901_MISO_PIN 宏定义
*                   NCS                查看 zf_device_pmw3901.h 中 PMW3901_CS_PIN 宏定义
*                   MOTION             查看 zf_device_pmw3901.h 中 PMW3901_MOTION_PIN 宏定义（可选）
*                   NRESET             查看 zf_device_pmw3901.h 中 PMW3901_RESET_PIN 宏定义（可选）
*                   VCC                5V电源
*                   GND                电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_device_pmw3901.h"

static uint8 pmw3901_init_state = 0;                                            // 模块完成初始化标志位

int16 pmw3901_delta_x   = 0, pmw3901_delta_y   = 0;                             // 偏移速度数据变量
int32 pmw3901_delta_x_i = 0, pmw3901_delta_y_i = 0;                             // 偏移距离数据变量

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PMW3901 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     pmw3901_write_register(PMW3901_POWER_UP_RESET, 0x5A);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void pmw3901_write_register(uint8 reg, uint8 data)
{
    reg |= 0x80u;

    PMW3901_CS(0);
    spi_write_8bit_register(PMW3901_SPI, reg, data);
    PMW3901_CS(1);
    
    if(pmw3901_init_state == 0)                                                 // 若模块未初始化完成 需要连续发送/读取数据 增加延时间隔                                      
    {
        system_delay_us(45); 
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PMW3901 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           读取到的数据
// 使用示例     pmw3901_read_register(PMW3901_PRODUCT_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 pmw3901_read_register(uint8 reg)
{
    uint8 data;
    reg &= ~0x80u;

    PMW3901_CS(0);
    spi_read_8bit_registers(PMW3901_SPI, reg, &data, 1);
    PMW3901_CS(1);
    
    if(pmw3901_init_state == 0)                                                 // 若模块未初始化完成 需要连续发送/读取数据 增加延时间隔                                      
    {
        system_delay_us(45); 
    }
    
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PMW3901 自检
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     pmw3901_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 pmw3901_self_check(void)
{
    uint8 id = pmw3901_read_register(PMW3901_PRODUCT_ID);
    if (id != 0x49)
    {
        zf_log(0, "PMW3901 self check failed!!!");
        return 1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 PMW3901 运动数据
// 参数说明     void
// 返回参数     void
// 使用示例     pmw3901_get_motion();
// 备注信息     执行后直接查看 pmw3901_delta_x 和 pmw3901_delta_y  
//              请务必注意以下使用事项：
//              1.该函数调用周期至少 20 毫秒
//              2.测量距离至少大于 5 厘米
//              3.务必保持光线充足 否则无法正常采集到光流数据
//-------------------------------------------------------------------------------------------------------------------
void pmw3901_get_motion(void)
{
    uint8 buf[6];
    PMW3901_CS(0);
    spi_read_8bit_registers(PMW3901_SPI, PMW3901_MOTION_BURST , buf, 6);
    PMW3901_CS(1);


    pmw3901_delta_x = (int16)((buf[3] << 8) | buf[2]);                          // 解析运动数据
    pmw3901_delta_y = (int16)((buf[5] << 8) | buf[4]);
    
    pmw3901_delta_x_i += pmw3901_delta_x;                                       // 积分获取距离数据
    pmw3901_delta_y_i += pmw3901_delta_y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 PMW3901
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     pmw3901_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 pmw3901_init(void)
{
    uint8 inti_state = 1;
    
    pmw3901_init_state = 0;
    
    // 初始化 SPI 和 GPIO
    spi_init(PMW3901_SPI, SPI_MODE0, PMW3901_SPI_SPEED, PMW3901_SCLK_PIN, PMW3901_MOSI_PIN, PMW3901_MISO_PIN, SPI_CS_NULL);
    gpio_init(PMW3901_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    system_delay_ms(5);
    pmw3901_write_register(0x3A,0x5A);        
    system_delay_ms(10);

    pmw3901_read_register(0x02);
    pmw3901_read_register(0x03);
    pmw3901_read_register(0x04);
    pmw3901_read_register(0x05);
    pmw3901_read_register(0x06);
    
    if(pmw3901_self_check() == 0)                                               // 自检
    {   
        pmw3901_write_register( 0x7F, 0x00);
        pmw3901_write_register( 0x61, 0xAD);
        pmw3901_write_register( 0x7F, 0x03);
        pmw3901_write_register( 0x40, 0x00);
        pmw3901_write_register( 0x7F, 0x05);
        pmw3901_write_register( 0x41, 0xB3);
        pmw3901_write_register( 0x43, 0xF1);
        pmw3901_write_register( 0x45, 0x14);
        pmw3901_write_register( 0x5B, 0x32);
        pmw3901_write_register( 0x5F, 0x34);
        pmw3901_write_register( 0x7B, 0x08);
        pmw3901_write_register( 0x7F, 0x06);
        pmw3901_write_register( 0x44, 0x1B);
        pmw3901_write_register( 0x40, 0xBF);
        pmw3901_write_register( 0x4E, 0x3F);
        pmw3901_write_register( 0x7F, 0x08);
        pmw3901_write_register( 0x65, 0x20);
        pmw3901_write_register( 0x6A, 0x18);
        pmw3901_write_register( 0x7F, 0x09);
        pmw3901_write_register( 0x4F, 0xAF);
        pmw3901_write_register( 0x5F, 0x40);
        pmw3901_write_register( 0x48, 0x80);
        pmw3901_write_register( 0x49, 0x80);
        pmw3901_write_register( 0x57, 0x77);
        pmw3901_write_register( 0x60, 0x78);
        pmw3901_write_register( 0x61, 0x78);
        pmw3901_write_register( 0x62, 0x08);
        pmw3901_write_register( 0x63, 0x50);
        pmw3901_write_register( 0x7F, 0x0A);
        pmw3901_write_register( 0x45, 0x60);
        pmw3901_write_register( 0x7F, 0x00);
        pmw3901_write_register( 0x4D, 0x11);
        pmw3901_write_register( 0x55, 0x80);
        pmw3901_write_register( 0x74, 0x1F);
        pmw3901_write_register( 0x75, 0x1F);
        pmw3901_write_register( 0x4A, 0x78);
        pmw3901_write_register( 0x4B, 0x78);
        pmw3901_write_register( 0x44, 0x08);
        pmw3901_write_register( 0x45, 0x50);
        pmw3901_write_register( 0x64, 0xFF);
        pmw3901_write_register( 0x65, 0x1F);
        pmw3901_write_register( 0x7F, 0x14);
        pmw3901_write_register( 0x65, 0x67);
        pmw3901_write_register( 0x66, 0x08);
        pmw3901_write_register( 0x63, 0x70);
        pmw3901_write_register( 0x7F, 0x15);
        pmw3901_write_register( 0x48, 0x48);
        pmw3901_write_register( 0x7F, 0x07);
        pmw3901_write_register( 0x41, 0x0D);
        pmw3901_write_register( 0x43, 0x14);
        pmw3901_write_register( 0x4B, 0x0E);
        pmw3901_write_register( 0x45, 0x0F);
        pmw3901_write_register( 0x44, 0x42);
        pmw3901_write_register( 0x4C, 0x80);
        pmw3901_write_register( 0x7F, 0x10);
        pmw3901_write_register( 0x5B, 0x02);
        pmw3901_write_register( 0x7F, 0x07);
        pmw3901_write_register( 0x40, 0x41);
        pmw3901_write_register( 0x70, 0x00);

        system_delay_ms(10); 

        pmw3901_write_register( 0x32, 0x44);
        pmw3901_write_register( 0x7F, 0x07);
        pmw3901_write_register( 0x40, 0x40);
        pmw3901_write_register( 0x7F, 0x06);
        pmw3901_write_register( 0x62, 0xF0);
        pmw3901_write_register( 0x63, 0x00);
        pmw3901_write_register( 0x7F, 0x0D);
        pmw3901_write_register( 0x48, 0xC0);
        pmw3901_write_register( 0x6F, 0xD5);
        pmw3901_write_register( 0x7F, 0x00);
        pmw3901_write_register( 0x5B, 0xA0);
        pmw3901_write_register( 0x4E, 0xA8);
        pmw3901_write_register( 0x5A, 0x50);
        pmw3901_write_register( 0x40, 0x80);
        pmw3901_write_register( 0x7F, 0x00);
        pmw3901_write_register( 0x5A, 0x10);
        pmw3901_write_register( 0x54, 0x00);

        system_delay_ms(10); 

        pmw3901_write_register( 0x7F, 0x0E);
        pmw3901_write_register( 0x72, 0x0F);
        pmw3901_write_register( 0x7F, 0x00);

        system_delay_ms(10); 
        pmw3901_write_register( PMW3901_OBSERVATION, 0x00);
        system_delay_ms(20); 
        
        if(pmw3901_read_register(PMW3901_OBSERVATION) == 0xBF)                 //  检查配置是否成功
        {
            inti_state = 0;
            pmw3901_init_state = 1;
        }
        else
        {
            zf_log(0, "PMW3901 init config failed!!!");
        }
    }

    return inti_state;
}
