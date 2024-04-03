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

/*

注意 Attention

有部分PID函数是很久之前的，我在现在的代码中并没有调用。直接看现在调用的就好
上方的一些参数设置也是，有部分是之前留下的，现在并没有用到

*/
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

    return (iIncpid);
}


//---------------------->PID参数<----------------------//


// 正常摄像头循迹用pid
void normalpid_params(void)
{
    SpeedPID.Kp = 20; 
    SpeedPID.Ki = 2;    
    SpeedPID.Kd = 1;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    Turn_NeiPID.Kp = 2.8; 
    Turn_NeiPID.Ki = 0;
    Turn_NeiPID.Kd = 0;
}

// 电磁专用pid
void adcpid_params(void)
{
    ADC_SpeedPID.Kp = 20;
    ADC_SpeedPID.Ki = 2;
    ADC_SpeedPID.Kd = 1;

    ADC_TurnPID.Kp = 180; 
    ADC_TurnPID.Ki = 0;
    ADC_TurnPID.Kd = 0;

    ADC_TURNNeiPID.Kp = 2.8;
    ADC_TURNNeiPID.Ki = 0;
    ADC_TURNNeiPID.Kd = 0;
}

// 加速bangbang pid
void speeduppid_params(void)
{
    SpeedPID.Kp = 10; 
    SpeedPID.Ki = 2;    
    SpeedPID.Kd = 12;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;     

    Turn_NeiPID.Kp = 3.6; 
    Turn_NeiPID.Ki = 0;
    Turn_NeiPID.Kd = 0; 
}

// 停车pid
void stoppid_params(void)
{
    // 快速停车， P超调
    SpeedPID.Kp = 10; 
    SpeedPID.Ki = 2;    
    SpeedPID.Kd = 12;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    Turn_NeiPID.Kp = 2.8; 
    Turn_NeiPID.Ki = 0;
    Turn_NeiPID.Kd = 0;
}

// 角度环pid参数
void anglepid_params(void)
{
    SpeedPID.Kp = 10; 
    SpeedPID.Ki = 2;    
    SpeedPID.Kd = 12;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    Angle_PID.Kp = 1.95; 
    Angle_PID.Ki = 0;
    Angle_PID.Kd = 2; 
}

void gostraighpid_params(void)
{
    SpeedPID.Kp = 10; 
    SpeedPID.Ki = 2;    
    SpeedPID.Kd = 12;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    Turn_NeiPID.Kp = 3.8; 
    Turn_NeiPID.Ki = 0;
    Turn_NeiPID.Kd = 0;   
}