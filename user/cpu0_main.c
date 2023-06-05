/*********************************************************************************************************************
  CPU1速度比CPU0快，更适合做运算
  定义全局变量的时候要想清楚，跨核访问变量比较占用资源
  可以把图像处理放在CPU1

->元素判断是否放在1核
->如果元素判断放在0核，那么图像数组就得跨核访问，所需要的时间应该会很长

  初步决定将摄像头部分代码放在核1，电磁相关代码放在核0，屏幕和上位机待定
  默认情况下全局变量都是保存在dsram1里的，导致核0中跑的代码对RAM的读取有一定的延迟
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

/*
	@Cyfarw9dd
	根据逐飞的变量和代码存储例程，以表明cpu1比cpu0更适合做运算，所以故将图像处理，屏幕显示等函数放在cpu1里调用
	cpu0则存放着电磁的代码，需要注意的是，在默认状态下，全局变量会存放在cpu1中，而核与核之间的跨核访问是比较占用时间和资源的
	所以应将电磁相关的全局变量通过语句调用存放在cpu0的dsram中
*/

// *************************** 代码区域 ****************************
extern short speed1, speed2;
extern S_FLOAT_XYZ GYRO_REAL, REAL_ACC;

int core0_main(void)
{
	clock_init();                   // 获取时钟频率<务必保留>
	debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

	// 初始化8路运放
	adc_init(ADC0_CH0_A0, ADC_8BIT);
    adc_init(ADC0_CH1_A1, ADC_8BIT);
    adc_init(ADC0_CH2_A2, ADC_8BIT);
    adc_init(ADC0_CH3_A3, ADC_8BIT);
    adc_init(ADC0_CH4_A4, ADC_8BIT);
    adc_init(ADC0_CH5_A5, ADC_8BIT);
    adc_init(ADC0_CH6_A6, ADC_8BIT);
    adc_init(ADC0_CH7_A7, ADC_8BIT);

	// pit_ms_init(CCU61_CH0, 1);
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
	while (TRUE)
	{
		// 此处编写需要循环执行的代码
		// gyroOffsetInit();
		// ADC_TaskProcess();
		// tft180_show_string(0, 0, "left out"); 		tft180_show_int(60, 0, Left_Adc, 5);
		// tft180_show_string(0, 30, "left in"); 		tft180_show_int(60, 30, Left_Shu_Adc, 5);
		// tft180_show_string(0, 60, "right in"); 		tft180_show_int(60, 60, Right_Adc, 5);
		// tft180_show_string(0, 90, "right out"); 		tft180_show_int(60, 90, Right_Shu_Adc, 5);
		// tft180_show_float(0, 100, GYRO_REAL.Z, 5, 2);

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
