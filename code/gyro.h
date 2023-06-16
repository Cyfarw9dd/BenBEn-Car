#ifndef CODE_GYRO_H_
#define CODE_GYRO_H_

#include "zf_common_headfile.h"

extern float Real_Gyro_Z;
extern float Real_Gyro_Y;

extern float Angle_X_Final;
extern float Angle_Y_Final;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;


typedef struct {
    float pitch;    //俯仰角
    float roll;     //偏航角
    float yaw;       //翻滚角
} euler_param_t;


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

float fast_sqrt(float x);

void gyroOffsetInit(void);

void ICM_getValues(void); 

void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void ICM_getEulerianAngles(void);

#endif /* CODE_GYRO_H_ */
