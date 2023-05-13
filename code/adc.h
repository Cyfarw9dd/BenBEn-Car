

// #ifndef CODE_ADC_H_
// #define CODE_ADC_H_

// #include "zf_common_headfile.h"

// //环岛检测电感阈值
// #define IN_ANNULUS_H_LIMIT        160
// //出环检测电感阈值  
// #define OUT_ANNULUS_S_LIMIT       160
// //环岛积分距离
// #define DISTANCE_ANNULUS_S        400    
// //环岛打角积分
// #define DISTANCE_ANNULUS_Z        250
// //环岛定时积分
// #define DISTANCE_ANNULUS_T        500    
// //出界判断
// #define OUTSIDE                   2
// //避障模块设定距离(根据实际情况修改)
// #define SET_DLLA_DISTANCE         400

// //舵机相关值
// #define Steer_Duty_Max            990
// #define Steer_Duty_Midle          850 //舵机中值
// #define Steer_Duty_Min            710

// //赛道类型判断
// struct ROAD_TYPE
// {
//      int8 straight;               //直道
// 	   int8 bend;                   //弯道
//      int8 annulus;                //环岛
//      int8 in_annulus_left;        //入左环岛
// 	   int8 in_annulus_right;       //入右环道
//      int8 on_annulus_left;        //在左环岛
// 	   int8 on_annulus_right;       //在右环岛
//      int8 out_annulus;            //出环岛
// 	   int8 in_park;                //入库
// };
// extern struct ROAD_TYPE road_type;

// //变量声明
// extern uint8 adc_value[4];  
// extern int16 AD_V[4]; 
// extern uint8 adc_max[4];  
// extern uint8 adc_min[4];  
// extern uint8 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;
// extern int8 NM;          
// extern uint16 annulus_s;         
// extern uint16 annulus_t;
// extern uint16 annulus_z;     
// extern float Current_Dir;         
// extern uint8 flag_obstacle;
// extern uint16 obstacle_time;
// extern int16 ADC_PWM;            
// extern uint8 temp;

// //函数声明
// void ADC_int(void);                   
// void ADC_Collect(void);               
// void Data_current_analyze(void);      
// float Cha_bi_he(int16 data1, int16 data2,int16 x); 
// void Road_type_judge(void);    
// void Annulus_handle(void);      
// int16 Direction_error(void);    
// void init_Steer_PWM(void);      
// void Steering_Control_Out(int16 duty);  
// void Out_protect(void);                  
// void Get_deviation(void);
// void Annulus_assist(void);
// void obstacle_avoidance(void);



// #endif /* CODE_ADC_H_ */
