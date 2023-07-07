#ifndef CODE_FUSE_H_
#define CODE_FUSE_H_

#include "zf_common_headfile.h"

extern  MyPID SpeedPID, 
        L_SpeedPID, 
        R_SpeedPID,
        TurnPID,
        ADC_TurnPID,
        Turn_NeiPID,
        ADC_SpeedPID,
        ADC_TURNNeiPID,
        Angle_PID;
        
extern short All_PWM_left, All_PWM_right;
extern short Speed_pwm_all;
extern int16 real_speed;
extern float real_real_speed;
extern int Centerline_Err;
extern int16 aim_speed;
extern short Steer_pwm;
extern short ADC_Speed_pwm;
extern short Prospect_err;
extern short ADC_Speed_left, ADC_Speed_right;

extern float theta;
extern float aim_theta;

// 时间片轮询任务结构体定义
typedef struct
{
    unsigned char Run;
    unsigned char Timer;
    unsigned char ItvTime;
    void (*TaskHook)(void);
}TASK_COMPONENTS;

void TaskRemarks(void);

void TaskProcess(void);

void Motor_output_control(void);

void Trailing_control(void);

void Speed_control(void);

void TaskCollectedRemarks(void);

void TaskCollectedProcess(void);


#endif /* CODE_FUSE_H_ */
