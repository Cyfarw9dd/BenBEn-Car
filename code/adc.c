#include "zf_common_headfile.h"

// 1 4 6 8 电感采集路道
unsigned char adc_value[4];                 

int16 AD_V[4];                      
unsigned char adc_max[4]={255,255,255,255}; 
unsigned char adc_min[4]={1,1,1,1};        
unsigned char Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;
int8 NM=4;                          

//环道参数
unsigned short annulus_s     = 0;           
unsigned short annulus_z     = 0;           
unsigned short annulus_t 	 = 0;

struct ROAD_TYPE road_type = {0};
int16 obstacle_Current_Dir[]={
								30,31,32,33,34,35,36,37,38,39,
								40,41,42,43,44,45,46,47,48,49,
								-69,-68,-67,-66,-65,-64,-63,-62,-61,-60,
								-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,
								-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,
								-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,
                             };

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
	adc_value[1] = (unsigned char)adc_ave(ADC0_CH5_A5, ADC_8BIT, 5);  
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
		if( AD_V[i]<=0)			AD_V[i]=0;
		else if(AD_V[i]>=100)	AD_V[i]=100;
    }
    Left_Adc 		= (unsigned char)AD_V[0];       
    Left_Shu_Adc 	= (unsigned char)AD_V[1];   
    Right_Shu_Adc 	= (unsigned char)AD_V[2];  
    Right_Adc 		= (unsigned char)AD_V[3];	    
}


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

void Road_type_judge(void)
{	 
	//环岛判断
	if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT)
	{
		road_type.annulus        = 1;
		road_type.straight       = 0;
		road_type.bend           = 0;  
	}
	else
	{   
		road_type.annulus        = 0;
		road_type.straight       = 1;
		road_type.bend           = 0;	  
	}
}

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
		annulus_s             	   = 0;
		annulus_z                  = 0;
		annulus_t                  = 0;
	}
}

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
    return error;
}

void Get_deviation(void)
{
	ADC_Collect();           //电感原始值采值
	Data_current_analyze();  //电感值归一化函数
	Current_Dir = Cha_bi_he(Left_Adc+Left_Shu_Adc,Right_Adc+Right_Shu_Adc,30); //获得赛道偏差
}



