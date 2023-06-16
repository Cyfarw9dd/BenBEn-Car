#include "zf_common_headfile.h"

PID Lmotor_PID,
    Rmotor_PID,
    Turn_PID,
    Direct_PID,
    Prospect_PID,
    Bottom_PID,
    Motor_PID;

PID_2 pid;
PID Steer_PID;

float Lmotor[3] = {100, 0, 2};
float Rmotor[3] = {100, 0, 2};
float Direct[4] = {30, 0, 0, -15};

float Prospect[4] = {0, 0, 0, 0};

float Bottom[4] = {0, 0, 0, 0};
float Motor[4]   = {50, 4.9, 3, 50};
float Prospect_ForPCrossing[4] = {40, 10, 0, 0};
float Bottom_ForPCrossing[4] = {-30, 0, 0, 0};
float Turn[5][4]  = {{8000, 380 ,  20,  400},


                    { 22,  17 ,  14,  400},             //160
                    { 23,  19 ,  15,  400},             //190
                    { 23,  19 ,  15,  400},             //220
                    { 24,  20 ,  16,  400}};            //240

float Steer[3] = {-0.5, 0, 0};


int PlacePID_Control(PID *sprt, float *PID, int NowPiont, int SetPoint)
{

    int iError,
          Actual;
    float Kp;

    iError = SetPoint - NowPiont;
    Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];

    Actual = Kp * iError
           + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);
    sprt->LastError = iError;

    Actual = range_protect(Actual, -3000, 3000);

    return Actual;
}


int PID_Realize(PID *sptr, float *PID, int NowData, int Point)
{

    int iError,
        Realize;

    iError = Point - NowData;
    sptr->SumError += PID[KI] * iError;
    if (sptr->SumError >= PID[KT])
    {
        sptr->SumError = PID[KT];
    }
    else if (sptr->SumError <= -PID[KT])
    {
        sptr->SumError = -PID[KT];
    }

    Realize = PID[KP] * iError
            + sptr->SumError
            + PID[KD] * (iError - sptr->LastError);
    sptr->PrevError = sptr->LastError;
    sptr->LastError = iError;
    sptr->LastData  = NowData;

    return Realize;
}

short PID_Increase(PID *sptr, float *PID, int NowData, int Point)
{

    int iError,
        Increase;

    iError = Point - NowData;

    Increase =  PID[KP] * (iError - sptr->LastError)
              + PID[KI] * iError
              + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);

    sptr->PrevError = sptr->LastError;
    sptr->LastError = iError;
    sptr->LastData  = NowData;

    return Increase;
}

int PD_control(float *PID){
    int Direct_Parameter;
    Direct_Parameter = PID[KP] * Prospect_Err + PID[KT] * Bottom_Err; //+ PID[KD] * Actual_gyro_z
    return Direct_Parameter;
}


void PID_Parameter_Init(PID *sptr)
{
    sptr->SumError  = 0;
    sptr->LastError = 0;    //Error[-1]
    sptr->PrevError = 0;    //Error[-2]
    sptr->LastData  = 0;
}


short range_protect(short duty, short min, short max)
{
    if (duty >= max)
        return max;
    if (duty <= min)
        return min;
    else
        return duty;
}


void PID_Init(void){
    pid.err=0;
    pid.err_last=0;

    pid.Kp=20;
    pid.Ki=0;
    pid.Kd=10;
    pid.Kout=0;
    pid.voltage=0;
    pid.integral=0;
}

int calculate_pid(float a){
    int t;

    pid.Set      = 0;
    pid.Actual   = a;
    pid.err      = pid.Set - pid.Actual;


    pid.integral = pid.integral + pid.err;

    pid.voltage = pid.Kp*pid.err
                + pid.Ki*pid.integral/1000
                + pid.Kd*(pid.err-pid.err_last)
                + pid.Kout;

    pid.err_last = pid.err;

    t = MOTOR_EXPECTATION + pid.voltage;

    /*if(t<0)             t=-t;
    if(t>9999)      t=9998;*/

    return t;
}



/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：位置式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float 
************************************************/
int16 LocP_DCalc(MyPID*sptr, int16 Setpoint, int16 Turepoint)
{
    int16 iError, dError;
    int16 output;

    iError = Setpoint - Turepoint;  //偏差
    sptr->SumError += iError;            //积分(采样时间很短时，用一阶差分代替一阶微分，用累加代替积分)
    dError = (int16)(iError - (sptr->LastError));     //微分
    sptr->LastError = iError;
    if(sptr->SumError > 2500) 
        sptr->SumError = 2500;   //积分限幅
    if(sptr->SumError < -2500) 
        sptr->SumError=-2500;
    output = (int16)(sptr->Kp * iError  //比例项
          +(sptr->Ki * sptr->SumError)//积分项
          +sptr->Kd * dError);        //微分项
    return(output);
}
/************************************************
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：增量式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
int16 IncPIDCalc(MyPID *sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError, iIncpid;
    //当前误差
    iError = Setpoint - Turepoint;      //偏差
 
    //增量计算
    iIncpid = (int16)(sptr->Kp * (iError - sptr->LastError)
            + sptr->Ki * iError
            + sptr->Kd * (iError - 2 * sptr->LastError + sptr->LLastError));
    //储存误差，用于下次计算

    sptr->LLastError = sptr->LastError;

    sptr->LastError = iError;

    return(iIncpid);
}