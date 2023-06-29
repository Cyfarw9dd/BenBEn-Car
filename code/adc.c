#include "zf_common_headfile.h"

// 1 4 6 8 电感采集路道
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

/***************************均值滤波函数*****************************/
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
/***************************电感采值************************************/
void ADC_Collect()
{
	adc_value[0] = (unsigned char)adc_ave(ADC0_CH7_A7, ADC_8BIT, 5);     
	adc_value[1] = (unsigned char)adc_ave(ADC0_CH3_A3, ADC_8BIT, 5);  
	adc_value[2] = (unsigned char)adc_ave(ADC0_CH2_A2, ADC_8BIT, 5); 
    adc_value[3] = (unsigned char)adc_ave(ADC0_CH1_A1, ADC_8BIT, 5);    
}
/*********************************电感采值********************************/
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
    Left_Adc = (unsigned char)AD_V[0];       //左电感最终值
    Left_Shu_Adc = (unsigned char)AD_V[1];   //左竖电感最终值
    Right_Shu_Adc = (unsigned char)AD_V[2];  //右竖电感最终值
    Right_Adc = (unsigned char)AD_V[3];	    //右电感最终值	
}

/*********************************差比和函数**********************************/
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
/*****************************************出界保护函数************************************/
void Out_protect(void)
{
	if(Left_Adc<2&&Right_Adc<2)
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



