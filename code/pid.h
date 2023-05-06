#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

#define KP 0  
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5

typedef struct
{
    float SumError;
    int LastError;
    int PrevError;
    int LastData;
}PID;

typedef struct 
{
    float SumError;

    float Kp;
    float Ki;
    float Kd;

    float LastError;
    float LLastError;
}MyPID;


typedef struct
{
    float Set;             //设定值
    float Actual;          //实际值
    float err;             //当前误差
    float err_last;        //上一次误差
    float last_derivative; //上次误差与上上次误差之差

    float Kp,Ki,Kd,Kout;
    float voltage;         //计算值
    float integral;        //误差积分值
}PID_2;

extern PID Lmotor_PID, Rmotor_PID, Direct_PID, Prospect_PID, Bottom_PID, Turn_PID, Motor_PID;
extern PID_2 pid;
extern PID Steer_PID;
extern float Lmotor[3], Rmotor[3];
extern float Direct[4], Prospect[4], Bottom[4], Motor[4], Prospect_ForPCrossing[4], Bottom_ForPCrossing[4];
extern float Turn[5][4];
extern float Steer[3];



int PlacePID_Control(PID *sprt, float *PID, int NowPiont, int SetPoint);

int PID_Realize(PID *sptr, float *PID, int NowData, int Point);

int16 PID_Increase(PID *sptr, float *PID, int NowData, int Point);

int PD_control(float *PID);

void PID_Parameter_Init(PID *sptr);

int16 range_protect(int16 duty, int16 min, int16 max);

int16 PID_Increase_for_ProspectErr(PID *sptr, float *PID);

int16 PID_Increase_for_BottomErr(PID *sptr, float *PID);

int16 PID_Realize_for_ProspectErr(PID *sptr, float *PID);

int16 PID_Realize_for_BottomErr(PID *sptr, float *PID);

void PID_Init(void);

int calculate_pid(float a);

int16 LocP_DCalc(MyPID*sptr, int16 Setpoint, int16 Turepoint);

int16 IncPIDCalc(MyPID *sptr,int16 Setpoint,int16 Turepoint);

#endif /* CODE_PID_H_ */
