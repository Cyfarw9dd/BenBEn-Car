#include "gyro.h"
#include "image.h"
#include "math.h"
#include "elements.h"
#include "zf_device_icm20602.h"

_steepest_st steepest_gz;
_steepest_st steepest_gy;
_sensor_st sensor;
S_INT16_XYZ GYRO, MPU_ACC;
int steepest_gz_arr[2];
int steepest_gy_arr[2];
float Real_Gyro_Z = 0;
float Real_Gyro_Y = 0;


void Get_IcmData(void){
    icm20602_get_acc();
    icm20602_get_gyro();
    Data_steepest();                    
    
    GYRO.X = icm20602_gyro_x;
    GYRO.Y = icm20602_gyro_y;
    GYRO.Z = icm20602_gyro_z;
    MPU_ACC.X = icm20602_acc_x;
	MPU_ACC.Y = icm20602_acc_y;
	MPU_ACC.Z = icm20602_acc_z;
    Real_Gyro_Z = sensor.Gyro_deg.Z;
    Real_Gyro_Y = sensor.Gyro_deg.Y;
}


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
 
