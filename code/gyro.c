#include "gyro.h"
#include "image.h"
#include "math.h"
#include "elements.h"
#include "zf_device_icm20602.h"

_steepest_st steepest_gz;
_steepest_st steepest_gy;
_sensor_st sensor;
S_INT16_XYZ GYRO, MPU_ACC;
S_FLOAT_XYZ GYRO_REAL, REAL_ACC;
int steepest_gz_arr[2];
int steepest_gy_arr[2];
float Real_Gyro_Z = 0;
float Real_Gyro_Y = 0;

// 零点漂移
float GyroX_Zero = 10;
float GyroY_Zero = 10;
float GyroZ_Zero = 10;


// 加速度计返回的加速度不是一般意义的加速度
// 只用加速度计就能得到车身的俯仰角，且是对地的绝对角度
// 但因为加速度计的不稳定，叠加在重力测量信号上使输出信号不能很好的反映车模的倾角



void steepest_descend(int32 arr[],unsigned char len,_steepest_st *steepest,unsigned char step_num,int32 in){
    unsigned char updw = 1;//0 dw,1up
    short i;
    unsigned char step_cnt=0;
    unsigned char step_slope_factor=1;
    unsigned char on = 1;
    char pn = 1;
    //float last = 0;
    float step = 0;
    int32 start_point = 0;
    int32 pow_sum = 0;

    steepest->lst_out = steepest->now_out;

    if( ++(steepest->cnt) >= len )
    {
        (steepest->cnt) = 0; //now
    }

    //last = arr[ (steepest->cnt) ];

    arr[ (steepest->cnt) ] = in;

    step = (float)(in - steepest->lst_out)/step_num ;

    if(absolute(step)<1)
    {
        if(absolute(step)*step_num<2)
        {
            step = 0;
        }
        else
        {
          step = (step > 0) ? 1 : -1;
        }
    }

    start_point = steepest->lst_out;
    do
    {
        //start_point = steepest->lst_out;
        for(i=0;i<len;i++)
        {
//          j = steepest->cnt + i + 1;
//          if( j >= len )
//          {
//              j = j - len; 
//          }
            pow_sum += my_pow(arr[i] - start_point );

            //start_point += pn *(step_slope_factor *step/len);
        }

        if(pow_sum - steepest->lst_pow_sum > 0)
        {
            if(updw==0)
            {
                on = 0;
            }
            updw = 1;
            pn = (pn == 1 )? -1:1;

        }
        else
        {
            updw = 0; 
            if(step_slope_factor<step_num)
            {
                step_slope_factor++;
            }
        }

        steepest->lst_pow_sum = pow_sum;
        pow_sum = 0;
        start_point += pn *step;

        if(++step_cnt > step_num)
        {
            on = 0;
        }
            //////
            if(step_slope_factor>=2)
            {
                on = 0;

            }
            //////

    }
    while(on == 1);

    steepest->now_out = start_point ;//0.5f *(start_point + steepest->lst_out);//

    steepest->now_velocity_xdt = steepest->now_out - steepest->lst_out;
}


void Data_steepest(void){
    steepest_descend(steepest_gz_arr, 2, &steepest_gz, 2, (int32)GYRO.Z);
    steepest_descend(steepest_gy_arr, 2, &steepest_gy , 2, (int32) GYRO.Y);

    sensor.Gyro_deg.Z = steepest_gz.now_out * 0.06404;
    sensor.Gyro_deg.Y = steepest_gy.now_out * 0.06404;
}



//extern int16 aacx,aacy,aacz;
//extern int16 gyrox,gyroy,gyroz;
// extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
// extern int16 icm_acc_x,icm_acc_y,icm_acc_z;
float Accel_x;
float Accel_y;
float Accel_z;
float Gyro_x;
float Gyro_y;
float Gyro_z;
float Angle_x_temp;
float Angle_y_temp;
float Angle_X_Final;
float Angle_Y_Final;


void Angle_Calcu(void)
{
    float accx,accy,accz;
    // get_icm20602_accdata_spi();
    // get_icm20602_gyro_spi();
    Get_IcmData();
//    get_accdata();
//    get_gyro();
//    Accel_x = aacx;
//    Accel_y = aacy;
//    Accel_z = aacz;
//    Gyro_x  = gyrox;
//    Gyro_y  = gyroy;
//    Gyro_z  = gyroz;

    Accel_x = MPU_ACC.X;
    Accel_y = MPU_ACC.Y;
    Accel_z = MPU_ACC.Z;
    Gyro_x  = GYRO.X;
    Gyro_y  = GYRO.Y;
    Gyro_z  = GYRO.Z;
    if(Accel_x<32764) accx=Accel_x/16384;
    else              accx=1-(Accel_x-49152)/16384;
    if(Accel_y<32764) accy=Accel_y/16384;
    else              accy=1-(Accel_y-49152)/16384;
    if(Accel_z<32764) accz=Accel_z/16384;
    else              accz=(Accel_z-49152)/16384;

    Angle_x_temp=(atan(accy/accz))*180/3.14;
    Angle_y_temp=(atan(accx/accz))*180/3.14;

    if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
    if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
    if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
    if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;


    if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
    if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
    if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
    if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
    if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
    if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;


    Kalman_Filter_X(Angle_x_temp,Gyro_x);
    Kalman_Filter_Y(Angle_y_temp,Gyro_y);
}



float Q_angle = 0.1;
float Q_gyro  = 0.3;
float R_angle = 0.5;
float dt      = 0.008;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float P[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro)
{

    Angle_X_Final += (Gyro - Q_bias) * dt;


    P[0]= Q_angle - PP[0][1] - PP[1][0];
    P[1]= -PP[1][1];
    P[2]= -PP[1][1];
    P[3]= Q_gyro;
    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_X_Final;


    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;


    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;


    Angle_X_Final += K_0 * Angle_err;
    Q_bias        += K_1 * Angle_err;
    Gyro_x         = Gyro - Q_bias;
}


void Kalman_Filter_Y(float Accel,float Gyro)
{
    Angle_Y_Final += (Gyro - Q_bias) * dt;
    P[0]=Q_angle - PP[0][1] - PP[1][0];
    P[1]=-PP[1][1];
    P[2]=-PP[1][1];
    P[3]=Q_gyro;
    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_Y_Final;
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
    Angle_Y_Final   += K_0 * Angle_err;
    Q_bias  += K_1 * Angle_err;
    Gyro_y   = Gyro - Q_bias;
}


// 获取icm20602的数据
// 获取角速度计，加速度计的原始数据并赋值，需在程序的开始的时候实时运行
void Get_IcmData(void){
    imu660ra_get_acc();
    imu660ra_get_gyro();
    // Data_steepest();                    
    
    GYRO.X = imu660ra_gyro_x;
    GYRO.Y = imu660ra_gyro_y;
    GYRO.Z = imu660ra_gyro_z;
    MPU_ACC.X = imu660ra_acc_x;
	MPU_ACC.Y = imu660ra_acc_y;
	MPU_ACC.Z = imu660ra_acc_z;
    // add
    // GYRO_REAL.X = imu660ra_gyro_x;
    // GYRO_REAL.Y = imu660ra_gyro_y;
    // GYRO_REAL.Z = imu660ra_gyro_z;
    // REAL_ACC.X = imu660ra_acc_x;
    // REAL_ACC.Y = imu660ra_acc_y;
    // REAL_ACC.Z = imu660ra_acc_z;


    Real_Gyro_Z = sensor.Gyro_deg.Z;
    Real_Gyro_Y = sensor.Gyro_deg.Y;
}

// “陀螺仪的使用方式其实很简单”
// 转角值 = 图像or电感计算出的偏差值 * KP + (本次偏差 - 上次偏差) * KD + 陀螺仪数值 * GKD

// 入口滤波，算数平均
// void Anglefiltering(void){
    
// }

// 互补滤波角度计算
// void AngleGet(void){
//     int Angle_acc, Angle_ratio;
//     float dt = 0.0001249;   //Gy 2ms时间积分系数
//     double angle_ratio;     //加速度比值
//     // Anglefiltering();       //入口滤波，算数平均

//     //以下为加速度计取反正切得到角度

//     angle_ratio = ((double)REAL_ACC.X) / (REAL_ACC.Z + 0.1);
//     Angle_acc = (float)atan(angle_ratio) * 57.29578049;  //加速度计得到的角
//     if(Angle_acc > 89)
//         Angle_acc = 89;
//     if(Angle_acc < -89)
//         Angle_acc = -89;
    
//     //以下为角速度计积分，同融合加速度，得到角度

//     float GY = (float)(GYRO_REAL.Y);
//     GY = GYRO_REAL.Y - GyroY_Zero;      //去零漂之后的陀螺仪采集值

//     float Angle = (float)(Angle-(float)(GY * dt));
//     Angle = Angle + (Angle_acc-Angle)*0.001; 
//     //相当于Angle = Angle*(1-0.00105) + Angle_acc*0.
// }   


// 陀螺仪滤除零漂

void gyroOffsetInit(void){
    GYRO_REAL.X = 0;
    GYRO_REAL.Y = 0;
    GYRO_REAL.Z = 0;
    for(unsigned char i = 0; i < 200; i++){
        Get_IcmData();
        GYRO_REAL.X += GYRO.X;
        GYRO_REAL.Y += GYRO.Y;
        GYRO_REAL.Z += GYRO.Z;
        // system_delay_us(50);
    }
    GYRO_REAL.X /= 200;
    GYRO_REAL.Y /= 200;
    GYRO_REAL.Z /= 200;
}
