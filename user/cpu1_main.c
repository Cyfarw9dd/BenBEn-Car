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

#pragma section all "cpu1_dsram"


// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************
// buzzer p33.10
#define PIT_CCU60_ms            1                                               // pit定时器1中断进入周期
#define BUZZER                  ATOM2_CH0_P33_10                                // 蜂鸣器引脚
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P14_0
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P14_1
#define REDLIGHT                101                                             // 定义红灯标志位
#define NOREDLIGHT              10                                              // 定义红灯丢失位
#define YELLOWLIGHT             201                                             // 定义黄灯标志位
#define NOYELLOWLIGHT           20                                              // 定义黄灯丢失位
#define LED1                    (P20_9)

unsigned char outflag = 0;                                                      // 出库标志位
extern short speed1, speed2;                                                    // 左右编码器值
extern S_FLOAT_XYZ GYRO_REAL, REAL_ACC;
unsigned char receive_array[9];
void dl1a_movement(void);
void trafficlights_movement(void);
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    outflag = 1;
    mt9v03x_init();
    tft180_init();
    imu660ra_init();
    wireless_uart_init();
    PID_int();

    pwm_init(ATOM0_CH0_P21_2, 17 * 1000, 0);
    pwm_init(ATOM0_CH1_P21_3, 17 * 1000, 0);
    pwm_init(ATOM0_CH2_P21_4, 17 * 1000, 0);
    pwm_init(ATOM0_CH3_P21_5, 17 * 1000, 0);

    encoder_dir_init(ENCODER_DIR_L, ENCODER_DIR_PULSE_L, ENCODER_DIR_DIR_L);
    encoder_dir_init(ENCODER_DIR_R, ENCODER_DIR_PULSE_R, ENCODER_DIR_DIR_R);

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
    gpio_init(TOGGLE1, GPI, GPIO_HIGH, GPI_PULL_UP);        // 初始化拨码开关1
    gpio_init(TOGGLE2, GPI, GPIO_HIGH, GPI_PULL_UP);        // 初始化拨码开关2
    pwm_init(BUZZER, 1500, 0);                                // 初始化编码器
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);  
    while(1)
    {
        if(dl1a_init())
            gpio_toggle_level(LED1);                                            
        else
            break;
        system_delay_ms(1000);                                                  
    }
    // 此处编写用户代码 例如外设初始化代码等
    // tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_set_font(TFT180_6X8_FONT);
    pit_ms_init(CCU60_CH0, PIT_CCU60_ms);
    
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        // 控制部分采用时间片轮询法，或许可以把速度环另开一个中断
        // pwm_set_duty(BUZZER, 8000);
        TaskProcess();
        image_process();
        tft180_show_gray_image(0, 0, &bin_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);
        dl1a_movement();
        // trafficlights_movement();
        // 此处编写需要循环执行的代码
    }
}

void dl1a_movement(void)
{
    dl1a_get_distance();
    if(dl1a_finsh_flag == 1)
    {
        dl1a_finsh_flag = 0;
        if(dl1a_distance_mm < 400)
        {        
            pit_disable(CCU60_CH0); // 关闭定时器中断
            motor_ctrl(-1800, -1800);//识别倒车   8.1V
            system_delay_ms(80);                
            motor_ctrl(0,0);
            system_delay_ms(200);
            motor_ctrl(-1800, 1800);//左转
            system_delay_ms(420);
            motor_ctrl(0,0);
            system_delay_ms(200);
            motor_ctrl(1800, 1800);//直行
            system_delay_ms(500);
            motor_ctrl(0,0);
            system_delay_ms(200);
            motor_ctrl(1800, -1800);//右转
            system_delay_ms(740);
            motor_ctrl(0,0);
            system_delay_ms(200);                                 
            motor_ctrl(1800, 1800);//直行
            system_delay_ms(600);
            motor_ctrl(0,0);
            system_delay_ms(200);
            // motor_ctrl(-1800, 1800);//左转
            // system_delay_ms(380);
            // motor_ctrl(0,0);
            // system_delay_ms(100);
            get_motor_speed();      // 重新采集一次编码器数据，不然编码器数据会保留在中断关闭前的那一刻，会有影响
            pit_enable(CCU60_CH0);  // 恢复定时器中断
        }
    }
}

void trafficlights_movement(void)
{
    
    unsigned char receive_array;
    uart_query_byte(UART_INDEX, &receive_array);
    // tft180_show_int(0, 0, receive_array + 1, 5);
    // tft180_show_string(0, 0, receive_array);
    if (receive_array + 1 == REDLIGHT)      // 如果识别到红灯
    {
        // tft180_clear();
        // tft180_show_string(0, 0, "check it");
        aim_speed = 0;                      // 期望速度0，刹车
        system_delay_ms(20);                // 一定延时
        pit_disable(CCU60_CH0);             // 关闭PIT中断
    }
    else if (receive_array + 2== YELLOWLIGHT)
    {
        aim_speed = 50;                      // 期望速度0，刹车
        system_delay_ms(20);
        system_delay_ms(20);                // 一定延时
        pit_disable(CCU60_CH0);             // 关闭PIT中断
    }
    else
    {
        aim_speed = 100;
        get_motor_speed();
        pit_enable(CCU60_CH0);
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
