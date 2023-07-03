#include "zf_common_headfile.h"

// extern icm_param_t imu_data;
// extern euler_param_t eulerAngle;
extern S_FLOAT_XYZ GyroOffset;
int16 aim_speed;       // 目标速度
int16 real_speed = 0;      // 左右轮平均速度
float real_real_speed = 0; // 左右轮平均速度换算成实际速度
int16 left_speed = 0;      // 左轮速度
int16 right_speed = 0;     // 右轮速度
MyPID SpeedPID = {0};
MyPID L_SpeedPID = {0};
MyPID R_SpeedPID = {0};
MyPID TurnPID = {0};
MyPID Turn_NeiPID = {0};
MyPID ADC_TurnPID = {0};

short Steer_pwm;
short All_PWM_left, All_PWM_right;
short Prospect_err;
short Speed_pwm_all;
int Centerline_Err;

static TASK_COMPONENTS TaskComps[] =
{
    {0, 2, 2, Motor_output_control}, // 角速度内环和D车速度环2ms
    {0, 10, 10, Trailing_control},   // 转向外环10ms
    {0, 10, 10, Speed_control},        // C车速度环20ms
};

static TASK_COMPONENTS TaskCollect[] =
{
    {0, 2, 2, gyroOffsetInit},          // 陀螺仪数据采集
    {0, 10, 10, get_motor_speed},         // 编码器数据采集
};

void PID_int(void)
{
    SpeedPID.Kp = 10; // 1.0 //速度环PID参数（D车用，速度环2ms）
    SpeedPID.Ki = 2;    // 0.7
    SpeedPID.Kd = 12;

    L_SpeedPID.Kp = 0.5; // 左轮速度环PID参数（C车用，C车建议左右轮PID分开调,速度环20ms）
    L_SpeedPID.Ki = 0.8;
    L_SpeedPID.Kd = 0;

    R_SpeedPID.Kp = 0.4; // 右速度环PID参数（C车用，C车建议左右轮PID分开调,速度环20ms）
    R_SpeedPID.Ki = 0.9;
    R_SpeedPID.Kd = 0;

    TurnPID.Kp = 135; 
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    ADC_TurnPID.Kp = 100; // 电磁转向环PID参数
    ADC_TurnPID.Ki = 0;
    ADC_TurnPID.Kd = 0;

    Turn_NeiPID.Kp = 2.8; //  4.89
    Turn_NeiPID.Ki = 0;
    Turn_NeiPID.Kd = 0;
}

void TaskRemarks(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(TaskComps) / sizeof(TASK_COMPONENTS); i++) // 逐个任务时间处理
    {
        if (TaskComps[i].Timer) // 时间不为0
        {
            TaskComps[i].Timer--;        // 减去一个节拍
            if (TaskComps[i].Timer == 0) // 时间减完了
            {
                TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，从新下一次
                TaskComps[i].Run = 1;                      // 任务可以运行
            }
        }
    }
}


void TaskProcess(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(TaskComps) / sizeof(TASK_COMPONENTS); i++) // 逐个任务时间处理
    {
        if (TaskComps[i].Run) // 时间不为0
        {
            TaskComps[i].TaskHook(); // 运行任务
            TaskComps[i].Run = 0;    // 标志清0
        }
    }
}

void Motor_output_control()
{
    if (track_mode == NORMAL)
    {
        gyroOffsetInit();
        Steer_pwm = LocP_DCalc(&Turn_NeiPID, (short)GyroOffset.Z, Prospect_err); // 转向内环PWM	 Prospect_err
        Steer_pwm = range_protect(Steer_pwm, -6000, 6000);               // 转向内环PWM限幅

        All_PWM_left = Speed_pwm_all - Steer_pwm;  // 左电机所有PWM输出 Speed_pwm_all Steer_pwm
        All_PWM_right = Speed_pwm_all + Steer_pwm; // 右电机所有PWM输出

        motor_ctrl(All_PWM_left, All_PWM_right); // 动力输出
    }
    if (track_mode == ADC)
    {
        gyroOffsetInit();
        Steer_pwm = LocP_DCalc(&Turn_NeiPID, (short)GyroOffset.Z, ADC_PWM); // 转向内环PWM	 icm20602_gyro_z
        Steer_pwm = range_protect(Steer_pwm, -6000, 6000);          // 转向内环PWM限幅

        All_PWM_left = Speed_pwm_all - Steer_pwm;  // 左电机所有PWM输出
        All_PWM_right = Speed_pwm_all + Steer_pwm; // 右电机所有PWM输出

        motor_ctrl(All_PWM_left, All_PWM_right); // 动力输出
    }
    if (track_mode == GO_STRAIGHT)
    {
        // gyroOffsetInit();
        Steer_pwm = LocP_DCalc(&Turn_NeiPID, (short)GyroOffset.Z, Prospect_err); // 转向内环PWM	 Prospect_err
        Steer_pwm = range_protect(Steer_pwm, -6000, 6000);               // 转向内环PWM限幅

        All_PWM_left = Speed_pwm_all - 0;  // 左电机所有PWM输出 Speed_pwm_all Steer_pwm
        All_PWM_right = Speed_pwm_all + 0; // 右电机所有PWM输出

        motor_ctrl(All_PWM_left, All_PWM_right); // 动力输出
    }
    if (track_mode == TURN)
    {
        // gyroOffsetInit();
        Prospect_err = 300;
        Steer_pwm = LocP_DCalc(&Turn_NeiPID, (short)GyroOffset.Z, Prospect_err); // 转向内环PWM	 Prospect_err
        Steer_pwm = range_protect(Steer_pwm, -6000, 6000);               // 转向内环PWM限幅

        All_PWM_left = 0 - Steer_pwm;  // 左电机所有PWM输出 Speed_pwm_all Steer_pwm
        All_PWM_right = 0 + Steer_pwm; // 右电机所有PWM输出

        motor_ctrl(All_PWM_left, All_PWM_right); // 动力输出
        track_mode = NORMAL;
    }
    if (track_mode == OBSTACLE)
        return;
}


void Trailing_control()
{
    if (track_mode == NORMAL)
    {
        Get_deviation();
        Centerline_Err = Cal_centerline(); 
        // track_decision();
        Prospect_err = LocP_DCalc(&TurnPID, (short)Centerline_Err, 0); // 位置式PD控制转向
    }
    if (track_mode == ADC)
    {
        Get_deviation(); // 电磁采集并获取赛道偏差
        ADC_PWM = LocP_DCalc(&ADC_TurnPID, Current_Dir, 0); // 位置式PD控制转向
        // ADC_PWM = -ADC_PWM;
    }
    if (track_mode == OBSTACLE)
        return;
}


void Speed_control()
{
    if (track_mode == NORMAL || track_mode == GO_STRAIGHT || track_mode == OBSTACLE)
    {
        get_motor_speed(); // 编码器测量
        real_speed = (speed1 + speed2) / 2;
        real_real_speed = speed1 * 0.0432f; // 0.0432f
        Speed_pwm_all += IncPIDCalc(&SpeedPID, aim_speed, real_speed); // D车速度环（增量式）
        range_protect(Speed_pwm_all, -6000, 6000); 
    }
    if (track_mode == ADC)
    {
        get_motor_speed();      //编码器测量
        real_speed = (speed1 + speed2) / 2;
        real_real_speed = speed1 * 0.0432f; // 0.0432f
        Speed_pwm_all += IncPIDCalc(&SpeedPID, aim_speed, real_speed); // D车速度环（增量式） 
        range_protect(Speed_pwm_all, -6000, 6000); 
    }
}


void TaskCollectedRemarks(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(TaskCollect) / sizeof(TASK_COMPONENTS); i++) // 逐个任务时间处理
    {
        if (TaskCollect[i].Timer) // 时间不为0
        {
            TaskCollect[i].Timer--;        // 减去一个节拍
            if (TaskCollect[i].Timer == 0) // 时间减完了
            {
                TaskCollect[i].Timer = TaskCollect[i].ItvTime; // 恢复计时器值，从新下一次
                TaskCollect[i].Run = 1;                      // 任务可以运行
            }
        }
    }
}

void TaskCollectedProcess(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(TaskCollect) / sizeof(TASK_COMPONENTS); i++) // 逐个任务时间处理
    {
        if (TaskCollect[i].Run) // 时间不为0
        {
            TaskCollect[i].TaskHook(); // 运行任务
            TaskCollect[i].Run = 0;    // 标志清0
        }
    }
}
