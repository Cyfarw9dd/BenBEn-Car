#include "zf_common_headfile.h"

extern S_FLOAT_XYZ GYRO_REAL, REAL_ACC;
int16 aim_speed = 0;         // 目标速度 
int16 real_speed = 0;        // 左右轮平均速度 
float real_real_speed = 0;   // 左右轮平均速度换算成实际速度   
int16 left_speed = 0;        // 左轮速度
int16 right_speed = 0;       // 右轮速度
MyPID SpeedPID = {0};
MyPID L_SpeedPID ={0};
MyPID R_SpeedPID ={0};
MyPID TurnPID ={0};
MyPID Turn_NeiPID ={0};

int Steer_pwm;
int All_PWM_left, All_PWM_right;
int Prospect_err;
int Speed_pwm_all;
int Centerline_Err;

static TASK_COMPONENTS TaskComps[] =
{
    {0,  2,  2, Motor_output_control},          // 角速度内环和D车速度环2ms
    {0, 10, 10, Trailing_control},              // 转向外环10ms
    {0,  2,  2, Speed_control},                   // C车速度环20ms
    // {0,  1,  1, KeyScan},                       // 按键扫描函数  
};

void PID_int(void)
{
	SpeedPID.Kp=12.0;     //1.0 //速度环PID参数（D车用，速度环2ms）
	SpeedPID.Ki=2;     //0.7
	SpeedPID.Kd=16;
	
	L_SpeedPID.Kp=0.5;   //左轮速度环PID参数（C车用，C车建议左右轮PID分开调,速度环20ms）
	L_SpeedPID.Ki=0.8;
	L_SpeedPID.Kd=0;
	
	R_SpeedPID.Kp=0.4;   //右速度环PID参数（C车用，C车建议左右轮PID分开调,速度环20ms）
	R_SpeedPID.Ki=0.9;
	R_SpeedPID.Kd=0;
	
	TurnPID.Kp=100;       //转向环PID参数 （C车只用调这个，不用串级转向）
	TurnPID.Ki=0;
	TurnPID.Kd=0;
	
	Turn_NeiPID.Kp=4.8;  //转向内环PID参数（D车用） 4.89
	Turn_NeiPID.Ki=0;
	Turn_NeiPID.Kd=0;
}

/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
* attention      : ***在定时器中断中调用此函数即可***
**************************************************************************************/
void TaskRemarks(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(TaskComps) / sizeof(TASK_COMPONENTS); i++)          // 逐个任务时间处理
    {
        if (TaskComps[i].Timer)          // 时间不为0
        {
           TaskComps[i].Timer--;         // 减去一个节拍
           if (TaskComps[i].Timer == 0)       // 时间减完了
           {
                TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，从新下一次
                TaskComps[i].Run = 1;           // 任务可以运行
           }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理|判断什么时候该执行那一个任务
* EntryParameter : None
* ReturnValue    : None
* * attention      : ***放在mian的while(1)即可***
**************************************************************************************/
void TaskProcess(void)
{
    unsigned char i; 
	  for (i = 0; i < sizeof(TaskComps) / sizeof(TASK_COMPONENTS); i++)           // 逐个任务时间处理
    {
        if (TaskComps[i].Run)           // 时间不为0
        {
            TaskComps[i].TaskHook();       // 运行任务
            TaskComps[i].Run = 0;          // 标志清0
        }
    }
}

/****************************角速度内环和D车速度环**************************************
函数：  void Motor_output_control()
参数：  void
说明：  角速度内环和D车速度环(D车/三轮车才会用)
返回值：void
作者：  heqiu
***************************************************************************************/
void Motor_output_control()
{
    //icm20602_get_gyro();   //获取陀螺仪角速度值
    imu660ra_get_gyro();
    Steer_pwm = LocP_DCalc(&Turn_NeiPID, GYRO_REAL.Z, Prospect_err);   //转向内环PWM	 icm20602_gyro_z
    Steer_pwm = range_protect(Steer_pwm, -6000, 6000);                //转向内环PWM限幅
	  
    All_PWM_left = Speed_pwm_all - Steer_pwm;                         //左电机所有PWM输出 Speed_pwm_all Steer_pwm
    All_PWM_right = Speed_pwm_all + Steer_pwm;                        //右电机所有PWM输出
	
    motor_ctrl(All_PWM_left, All_PWM_right);                          //动力输出
}

/****************************转向环（D车转向外环）**************************************
函数：  void Trailing_control()
参数：  void
说明：  转向环（D车转向外环）（C车转向环）
返回值：void
作者：  heqiu
***************************************************************************************/
void Trailing_control()
{
    // Get_deviation();  //电磁采集并获取赛道偏差
    // Annulus_assist(); //环岛辅助函数
    cal_curvature(&(MyRoad_Characteristics.Curve_Err));
    Centerline_Err = Cal_centerline();
    Prospect_err = LocP_DCalc(&TurnPID, Centerline_Err, 0);//位置式PD控制转向
    //ADC_PWM = PlacePID_Control(&TurnPID,Current_Dir,0);//动态位置式PID控制 (用于转向控制)
    //Steering_Control_Out(ADC_PWM);//(C车用控制舵机转向)
}

/****************************速度环（C车用）**************************************
函数：  void Speed_control()
参数：  void
说明：  速度环（C车用）
返回值：void
作者：  heqiu
***************************************************************************************/
void Speed_control()
{
    // timed_task();           //出库定时打开干簧管等
    get_motor_speed();      //编码器测量
    real_speed = (speed1 + speed2) / 2;
    real_real_speed = speed1 * 0.0432f;      // 0.0432f
    //Speed_pwm_left+=real_speed;
    //	  if(Speed_pwm_left>20000)
    //		{
    //			DisableGlobalIRQ();//关闭总中断
    //		  go_motor(0,0);
    //		}
    aim_speed = 400;      //目标速度

    //Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed ,real_speed); //D车速度环（位置式）
    Speed_pwm_all += IncPIDCalc(&SpeedPID, aim_speed, real_speed);//D车速度环（增量式）

    //Speed_pwm_left += IncPIDCalc(&L_SpeedPID,aim_speed , left_speed); //C车左轮速度环（位置式）
    //Speed_pwm_right += IncPIDCalc(&R_SpeedPID, aim_speed, right_speed); //C车右轮速度环（位置式）
    //go_motor(Speed_pwm_left,Speed_pwm_rig .ht);                         //动力输出
}