/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "isr_config.h"
#include "zf_common_headfile.h"
#include "image.h"
#include "control.h"
#include "pid.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。

// *************************** 例程硬件连接说明 ***********************
// 核心板正常供电即可 无需额外连接


// *************************** 例程测试说明 **************************
// 1.核心板烧录完成本例程，完成上电
// 2.可以看到核心板上两个 LED 按照同的频率闪烁
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// *************************** 特别说明 **************************
// 本例程为多核使用例程 LED1由核心0控制，并且每延时200毫秒就翻转一次LED显示状态
//               LED2由核心1控制，并且每延时300毫秒就翻转一次LED显示状态

// **************************** 代码区域 ****************************

#define ENCODER_DIR_L                     (TIM5_ENCODER)                         // 左带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_L               (TIM5_ENCODER_CH1_P10_3)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_L                 (TIM5_ENCODER_CH2_P10_1)               // DIR 对应的引脚

#define ENCODER_DIR_R                     (TIM6_ENCODER)                         // 左带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_R               (TIM6_ENCODER_CH1_P20_3)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_R                 (TIM6_ENCODER_CH2_P20_0)               // DIR 对应的引脚

#define ENCODER_PIT                       (CCU60_CH0 )

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    mt9v03x_init();
    tft180_init();
    // icm20602_init();
    wireless_uart_init();
    pwm_init(ATOM0_CH4_P02_4, 50, 0);
    pwm_init(ATOM0_CH5_P02_5, 50, 0);
    pwm_init(ATOM0_CH6_P02_6, 50, 0);
    pwm_init(ATOM0_CH7_P02_7, 50, 0);

    encoder_dir_init(ENCODER_DIR_L, ENCODER_DIR_PULSE_L, ENCODER_DIR_DIR_L);
    encoder_dir_init(ENCODER_DIR_R, ENCODER_DIR_PULSE_R, ENCODER_DIR_DIR_R);

//    pit_ms_init(ENCODER_PIT, 50);
    // 此处编写用户代码 例如外设初始化代码等

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        // Camera();
        tft180_show_gray_image(0 ,0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);
        // 此处编写需要循环执行的代码
    }
}

// IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
// {
//    interrupt_global_enable(0);                     // 开启中断嵌套
//    pit_clear_flag(CCU60_CH0);

//    get_motor_speed();

// }


#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
