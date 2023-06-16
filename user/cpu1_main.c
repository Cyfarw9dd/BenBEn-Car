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
// buzzer p33.1
#define PIT_CCU60_ms 5
unsigned char outflag = 0;      // 出库标志位

extern short speed1, speed2;
extern S_FLOAT_XYZ GYRO_REAL, REAL_ACC;
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

    // 此处编写用户代码 例如外设初始化代码等
    // tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_set_font(TFT180_6X8_FONT);
    pit_ms_init(CCU60_CH0, 1);
    
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        // while (outflag)
        // {
        //     motor_ctrl(3000, 2000);
        //     system_delay_ms(500);
        //     outflag = 0;
        // }
        // List_Switch();
        // cal_curvature(&(MyRoad_Characteristics.Curve_Err));
        // 以下为常用的测试代码

        // get_motor_speed();
        // motor_ctrl(3000, 3000);   // (0, 3000)向右转，(3000, 0)向左转
        // tft180_show_gray_image(0 ,0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);
        // tft180_show_int(0, 90, GYRO_REAL.Z, 5);
        // tft180_show

        gyroOffsetInit();
        // Camera();
		// sendimg_binary_CHK(&bin_image[0], MT9V03X_W, MT9V03X_H, image_thereshold, 25);
        // tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);
        put_float(0, real_real_speed);
        tft180_show_string(0, 30, "BlackPoints_Nums");         tft180_show_float(60, 30, real_real_speed, 5, 2);
        // tft180_show_string(0, 50 , "TurnNei_I");        tft180_show_float(60, 50, Turn_NeiPID.Ki, 5, 2);
        // tft180_show_string(0, 70, "TurnNei_D");        tft180_show_float(60, 70, Turn_NeiPID.Kd, 5, 2);
        // tft180_show_string(0, 110, "Turn_P");         tft180_show_float(45, 110, TurnPID.Kp, 5, 2);
        tft180_show_string(0, 110, "Turn_I");        tft180_show_float(45, 110, TurnPID.Ki, 5, 2);
        tft180_show_string(0, 130, "Turn_d");        tft180_show_float(45, 130, TurnPID.Kd, 5, 2);
        // tft180_show_float(0, 90, Centerline_Err, 5, 2);         tft180_show_float(60, 90, Prospect_Err, 5, 2);
        TaskProcess();
        image_process();
        tft180_show_float(0, 90, speed1, 5, 2);         tft180_show_float(60, 90, speed2, 5, 2); 
        Deal_Road_Characteristics(&bin_image[0], &MyRoad_Characteristics);
        Hightlight_Lines(&bin_image[0]);                 
        if(Key1 == onepress){
			Key1 = nopress;
			Turn_NeiPID.Kp += 0.1;
			// system_delay_ms(300);
		}
		if(Key2 == onepress){
            Key2 = nopress;
			TurnPID.Kp += 10;
			// system_delay_ms(300);
		}
		if(Key3 == onepress){
            Key3 = nopress;
			TurnPID.Kd += 1;
			// system_delay_ms(300);
		}
        // 此处编写需要循环执行的代码
    }
}



#pragma section all restore
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
