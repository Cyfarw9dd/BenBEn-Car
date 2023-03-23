#ifndef CODE_GYRO_H_
#define CODE_GYRO_H_

#include "zf_common_headfile.h"

extern float Real_Gyro_Z;
extern float Real_Gyro_Y;

extern float Angle_X_Final;
extern float Angle_Y_Final;
#define my_pow(a) ((a)*(a))
//extern S_INT16_XYZ GYRO, MPU_ACC;

typedef struct
{
    unsigned char  cnt;

    int lst_pow_sum;

    int now_out;
    int lst_out;
    int now_velocity_xdt;
} _steepest_st;


typedef struct
{
    short X;
    short Y;
    short Z;
}S_INT16_XYZ;

typedef struct
{
    float X;
    float Y;
    float Z;
}S_FLOAT_XYZ;

typedef struct
{
    unsigned char acc_CALIBRATE;
    unsigned char gyr_CALIBRATE;
    unsigned char vec3d_CALIBRATE;

    S_INT16_XYZ Acc_I16;
    S_INT16_XYZ Gyro_I16;

    S_FLOAT_XYZ Acc;
    S_FLOAT_XYZ Acc_mmss;
    S_FLOAT_XYZ Gyro;
    S_FLOAT_XYZ Gyro_deg;
}_sensor_st;//__attribute__((packed))

void steepest_descend(int arr[], unsigned char len, _steepest_st *steepest, unsigned char step_num, int in);

void Data_steepest(void);

unsigned short Get_fitter_distance(unsigned short real_distance);

void Angle_Calcu(void);

void Kalman_Filter_X(float Accel,float Gyro);

void Kalman_Filter_Y(float Accel,float Gyro);

void Get_IcmData(void);

void AngleGet(void);

#endif /* CODE_GYRO_H_ */
