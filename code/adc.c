#include "zf_common_headfile.h"
#include "adc.h"
#include "math.h"

unsigned char adc_value[4];                 //储存电感采集值原始值    4个电感 

int16 AD_V[4];                      //储存电感采集值归一化值中间变量 （无需关心，请勿删除）
unsigned char adc_max[4]={255,255,255,255}; //电感采值最大值 需要自己采集
unsigned char adc_min[4]={1,1,1,1};        //电感采值最小值
unsigned char Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;//电感值
int8 NM=4;                          //电感个数

//环道参数
unsigned short annulus_s     = 0;           //环岛积分距离
unsigned short annulus_z     = 0;           //环岛第积分打角
unsigned short annulus_t = 0;

struct ROAD_TYPE road_type = {0};
int16 obstacle_Current_Dir[]={
	                            30,31,32,33,34,35,36,37,38,39,
	                            40,41,42,43,44,45,46,47,48,49,
	                            -69,-68,-67,-66,-65,-64,-63,-62,-61,-60,
	                            -59,-58,-57,-56,-55,-54,-53,-52,-51,-50,
	                            -49,-48,-47,-46,-45,-44,-43,-42,-41,-40,
                              -39,-38,-37,-36,-35,-34,-33,-32,-31,-30,
                             };
/***当前位置*************/
float Current_Dir = 0;

int16 ADC_PWM=0;
unsigned char flag_obstacle=0;
unsigned short obstacle_time=0;
unsigned char temp=0;			
	 
/***************************电感采集通道初始化****************************
函数：  void ADC_int(void)  
功能：  电感采值进行初始化
参数：  void
说明：  电感采集初始化
返回值；无
日期：  2020-10-27
作者：  heqiu
************************************************************************/
// void ADC_int(void)
// {
//     adc_init(Left_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.0为ADC功能
//     adc_init(LeftXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.1为ADC功能
//     adc_init(RightXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.5为ADC功能
//     adc_init(Right_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.6为ADC功能 
// }

/***************************中值滤波函数*********************************
函数：unsigned short adc_mid(adc_channel_enum adcn,ADCCH_enum ch)  
功能： 3次电感采值进行中值滤波
参数： adcn        选择ADC通道       resolution      分辨率
说明： 8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；k(unsigned char)中间那个值
日期：  2020-10-27
作者：  heqiu 
************************************************************************/
unsigned short adc_mid(adc_channel_enum adcn, adc_resolution_enum ch)
{
	unsigned short i,j,k,tmp;
	i=adc_convert(adcn);
	j=adc_convert(adcn);
	k=adc_convert(adcn);
	if(i>j)
	{
		tmp=i,i=j,j=tmp;
	}
	if(k>j)
	{
		tmp=j;
	}
	else if(k>i)
	{
		tmp=k;
	}
	else
	{
		tmp=i;
	}
	return(tmp);
}

/***************************均值滤波函数****************************
函数：  unsigned short adc_ave(adc_channel_enum adcn,ADCCH_enum ch,unsigned char N) 
功能：  中值滤波后的5个电感值求平均值
参数：  adcn        选择ADC通道         
说明：  该函数调用中值滤波函数，即电感值是中位置
返回值；tmp
日期：  2020-10-27
作者：  heqiu 
示例：  adc_ave(ADC_P10, ADC_8BIT)-->ADC通道为P-10，分辨率为8bit 
*******************************************************************/
unsigned short adc_ave(adc_channel_enum adcn, adc_resolution_enum ch,unsigned char N)
{
	unsigned short tmp=0;
	unsigned char i;
	for(i=0;i<N;i++)
	{
	  tmp+=adc_mid(adcn,ch);
	}
	tmp=tmp/N;
	return(tmp);
}
/***************************电感采值************************************
函数：  void ADC_Collect()   
功能：  电感采值
参数：  void
说明：  8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；void
日期：  2020-10-27
作者：  heqiu   
***********************************************************************/
void ADC_Collect()
{
	adc_value[0] = adc_ave(ADC0_CH7_A7, ADC_8BIT, 5);     //左横电感 ADC0_CH7_A7
	adc_value[1] = adc_ave(ADC0_CH3_A3, ADC_8BIT, 5);  //左竖电感 ADC0_CH3_A3
	adc_value[2] = adc_ave(ADC0_CH0_A0, ADC_8BIT, 5); //右竖电感 ADC0_CH1_A1
    adc_value[3] = adc_ave(ADC0_CH1_A1, ADC_8BIT, 5);    //右横电感 ADC0_CH3_A3
}
/*********************************电感采值********************************
函数：  void Data_current_analyze()   
功能：  电感采值原始值归一化（0~100）
参数：  void
说明：  归一化处理
返回值；void
日期：  2023-3-17
作者：  heqiu              
*************************************************************************/
void Data_current_analyze()
{
    unsigned char i;
    for(i=0;i < NM; i++)              
    {
        AD_V[i] = ((adc_value[i]-adc_min[i])*100)/adc_max[i];          
    if( AD_V[i]<=0)
    {
        AD_V[i]=0;
    }
    else if(AD_V[i]>=100)
    {
        AD_V[i]=100;
    }
    }
    Left_Adc = AD_V[0];       //左电感最终值
    Left_Shu_Adc = AD_V[1];   //左竖电感最终值
    Right_Shu_Adc = AD_V[2];  //右竖电感最终值
    Right_Adc = AD_V[3];	    //右电感最终值	
}

/*********************************差比和函数**********************************
函数：  float Cha_bi_he(int16 data1, int16 data2,int16 x)
功能：  差比和求赛道偏差
参数：  int16 data1, int16 data2,int16 x
说明：  差比和求赛道偏差
返回值；result
日期：  2023-3-23
作者：  heqiu              
****************************************************************************/
float Cha_bi_he(int16 data1, int16 data2,int16 x)
{
    int16 cha;
    int16 he;
    float result;

    cha = data1-data2;
    he = data1+data2+1;
    result = (cha*x)/(1.0*he);

    return result;
}
// /*****************************************舵机初始化*************************************
// 函数：  void init_PWM(void)
// 参数：  无
// 说明：  分母10000，使用，如需修改引脚修改对应宏定义即可
//         pwm_init(PWM0_P00, 100, 5000);     //初始化PWM0  使用引脚P0.0  输出PWM频率100HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
// 				PWM_DUTY_MAX在zf_pwm.h文件中 默认为10000
// *注意，先调节舵机，如果舵机为SD05，则频率为200hz ,如果舵机为S3010,频率则为50hz
// *频率确定后，先把占空比分母，即PWM_DUTY_MAX确定，一般无需修改了
// *然后就开始调节舵机了，调占空比的分子，即调用的函数的最后那个参数，根据经验算一下，大概是1/20的占空比，然后往左往右慢慢试
// *计算公式：中值占空比大概是7.5% （和频率精度都有关系） 20ms(1.5ms高电平)
// 返回值：无  
// ***************************************************************************************/
// void init_Steer_PWM(void)
// {
//   	pwm_init(Steer_Pin, 50, Steer_Duty_Midle);     //初始化舵机  输出PWM频率50HZ，并设置中值
// }

// /************************************舵机转向控制输出**********************************
// 函数：  void Steering_Control_Out(int16 duty)
// 功能：  舵机转向控制  
// 参数：  无
// 说明：  舵机转向控制    注意调好舵机中值后，左右极限也调出来，要修改上面的宏定义
// 返回值：无 
// **************************************************************************************/
// void Steering_Control_Out(int16 duty)
// {
//    duty = Steer_Duty_Midle + duty ;//在舵机中值的基础上进行偏移
//    if (duty >= Steer_Duty_Max) 
// 	 {
// 		 duty = Steer_Duty_Max;
// 	 }
//    else if(duty <= Steer_Duty_Min) 
// 	 {
// 		 duty = Steer_Duty_Min;
// 	 }
//    pwm_duty(Steer_Pin, duty);
// }
/*****************************************出界保护函数*************************************
函数：  void Out_protect() 
参数：  无
说明：  防止车冲出赛道后撞坏东西,检测出赛道后中断失能，电机停转，放回赛道中断使能继续跑

*注意：！！！平时调试时可以打开，加了避障处理后需要关闭此函数，不然有可能无法实现避障功能！！！
返回值：无  
作者：  heqiu 
******************************************************************************************/
void Out_protect(void)
{
	if(Left_Adc<OUTSIDE&&Right_Adc<OUTSIDE)
	{
		pit_disable(CCU60_CH1);//关闭总中断
		motor_ctrl(0, 0);
	}
	else
	{
		pit_enable(CCU60_CH1);
	}
}
/*****************************************判断赛道类型*************************************
函数：  void Road_type_judge(void)
参数：  无
说明：  赛道类型判断--环岛--弯道--直道--

*注意： 环岛和弯道判断的阈值可能不准，需要自己测量！！！每辆车的运放电路，赛道电磁线，电感精度等等
        都会影响到环岛处理，所以这里的阈值不能保证每一辆车都能进环，关于怎么调电感和处理圆环的方法
        如果购买了视频讲解将会在视频讲中详细讲解！！！
返回值：无  
作者：  heqiu 
******************************************************************************************/
void Road_type_judge(void)
{	 
	  //环岛判断
	  if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT)
	  {
			road_type.annulus        = 1;
			road_type.straight       = 0;
			road_type.bend           = 0;  
		}
//		//弯道判断（不准确暂时不用）
//	  else if((Left_Adc > 85 && Right_Shu_Adc < 50 && Left_Shu_Adc>40)||(Left_Adc <50 && Right_Adc > 85 && Right_Shu_Adc>40))
//		{     
//			road_type.annulus        = 0;
//			road_type.straight       = 1;
//			road_type.bend           = 0;
//		}
		//直道判断：以上类型均不满足则为直道
		else
		{   
			road_type.annulus        = 0;
		  road_type.straight       = 1;
		  road_type.bend           = 0;	  
		}
}
/*****************************************环岛处理***************************************
函数：  void Annulus_handle(void)
参数：  无
说明：  环岛处理函数

*注意：用两个竖电感引导进环
返回值：无  
作者：  heqiu 
******************************************************************************************/
void Annulus_handle(void)
{
	  //左环判断
		if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_left==0&&(Left_Shu_Adc>30))	 
		{		  
			road_type.in_annulus_left = 1;
	
		}
		//右环判断
	  else if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_right==0&&(Right_Shu_Adc>30))
		{
			road_type.in_annulus_right = 1;
	
		}
		//左环处理
		if(road_type.in_annulus_left == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_left==1&&road_type.on_annulus_left==0)
		  {
			  road_type.on_annulus_left = 1;
		
		  }
		  if(road_type.on_annulus_left==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)
		  {			
				road_type.out_annulus = 1;
				
		  }
		}
		//右环处理
	  else if(road_type.in_annulus_right == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_right==1&&road_type.on_annulus_right==0)
		  {
			  road_type.on_annulus_right = 1;
		
		  }
		  if(road_type.on_annulus_right==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)  
		  {			
				road_type.out_annulus = 1;		
		
		  }
		}
		//出环处理
		if(annulus_t>DISTANCE_ANNULUS_T)
		{				
				road_type.annulus          = 0;
				road_type.in_annulus_left  = 0;
			  road_type.in_annulus_right = 0;
				road_type.on_annulus_left  = 0;
			  road_type.on_annulus_right = 0;
				road_type.out_annulus      = 0;
				annulus_s             		 = 0;
			  annulus_z                  = 0;
			  annulus_t                  = 0;
		}
}
// /*************************************环岛辅助函数*************************************
// 函数：  void Annulus_assist(void)
// 参数：  无
// 说明：  过环岛三角区积分，进环积分，出环积分等

// *注意： 积分值会会随采样时间的不同而改变，需要自己用手推车去测量用屏幕显示看并记录去修改
// 返回值：无  
// 作者：  heqiu 
// ******************************************************************************************/
// void Annulus_assist(void)
// {
// 	 if(road_type.annulus==1&&road_type.in_annulus_left==0&&road_type.in_annulus_right==0)
//    {
//         annulus_s += fabs(real_speed*0.1);  
//    }
// 	 if((road_type.in_annulus_left==1 ||road_type.in_annulus_right==1) && road_type.on_annulus_left==0)
//    {
//         annulus_z += fabs(icm20602_gyro_z*0.01);
//    }
// 	 if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && road_type.out_annulus==1)
//    {
//         annulus_t=annulus_t+10;
//    }
// }
// /*************************************避障检测函数*************************************
// 函数：  void obstacle_avoidance(void)
// 参数：  无
// 说明：  TFO避障模块检测，使用软件模拟IIC通信，理论上任何引脚都可以使用，但是要注意不能引脚
//         复用。
// *注意： TOF模块离障碍物越远数值越大，越近数值越小
// 返回值：无  
// 作者：  heqiu 
// ******************************************************************************************/
// void obstacle_avoidance(void)
// {
// 	dl1a_get_distance();                                       //距离测量
// 	if(dl1a_finsh_flag==1&&dl1a_distance_mm<SET_DLLA_DISTANCE) //测量距离小于设定值标志位成立
// 	{
// 		dl1a_finsh_flag=0;
// 		flag_obstacle=1;
// 	}
// }
/*************************根据赛道类型选择不同的方向偏差计算方法*************************
函数：  int16 Direction_error(void)
功能：  根据赛道类型选择不同的方向偏差
参数：  无
说明：  根据赛道类型选择不同的方向偏差--直道--弯道--环岛处理切换不同差比和
返回值：error--返回赛道偏差
作者：  heqiu 
****************************************************************************************/
int16 Direction_error(void)
{
    int16 error = 0;

    //直道方向偏差计算
    if(road_type.straight==1)
    {
			  error = Cha_bi_he(Left_Adc+Left_Shu_Adc,Right_Adc+Right_Shu_Adc,40); 
    }
    //弯道方向偏差计算
    else if(road_type.bend)
    {
			  error = Cha_bi_he(Left_Adc+Left_Shu_Adc,Right_Adc+Right_Shu_Adc,200);  
    }
    // //环岛方向偏差计算
    // else if(road_type.annulus==1)
    // {
	// 		  //准备入环岛方向偏差计算
    //     if(road_type.in_annulus_left==0 && road_type.in_annulus_right==0 && road_type.on_annulus_left==0 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
	// 			{
    //         error = Cha_bi_he(Left_Adc, Right_Adc, 10);
	// 			}
    //     //入左环岛方向偏差计算
    //     if(road_type.in_annulus_left ==1 && road_type.on_annulus_left==0 && road_type.out_annulus==0)
	// 			{
	// 			    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,60);
	// 			}
	// 			//入右环岛方向偏差计算
	// 		  if(road_type.in_annulus_right ==1 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
	// 			{
	// 			    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,90);
					
	// 			}
    //     //出环岛方向偏差计算
    //     //if((road_type.in_annulus_left||road_type.in_annulus_right) && (road_type.on_annulus_left||road_type.on_annulus_right)&&road_type.out_annulus)
	// 			    //error = Cha_bi_he(Left_Adc,Right_Adc,30);
    // }
		//避障误差处理
	//   if(flag_obstacle==1)
	// 	{
	// 		
	// 			 aim_speed=100;
	// 			 error = obstacle_Current_Dir[temp];
	// 			 temp++;
	// 			 if(temp==40)
	// 			 {
	// 				 temp=0;
	// 				 flag_obstacle=0;
	// 				 obstacle_time=0;
	// 			   aim_speed=400;
	// 			   BUZZ_OFF;
	// 			 }
	// 	}
    return error;
}

/**********************************电磁所有总处理***************************************
函数：  void Get_deviation(void)
功能：  电磁所有总处理
参数：  无
说明：  放中断调用此函数即可
返回值：无
作者：  heqiu 
****************************************************************************************/
void Get_deviation(void)
{
	ADC_Collect();           //电感原始值采值
	Data_current_analyze();  //电感值归一化函数
	// Road_type_judge();       //赛道类型判断---直道---弯道---环岛
	// Annulus_handle();        //环岛处理
	//obstacle_avoidance();    //障碍物检测
	Current_Dir = Cha_bi_he(Left_Adc+Left_Shu_Adc,Right_Adc+Right_Shu_Adc,40); //获得赛道偏差
	Out_protect();         //出界保护
}



