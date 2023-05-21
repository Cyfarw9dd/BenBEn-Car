#ifndef CODE_FUSE_H_
#define CODE_FUSE_H_

#include "zf_common_headfile.h"

extern  MyPID SpeedPID, 
        L_SpeedPID, 
        R_SpeedPID,
        TurnPID,
        Turn_NeiPID;
        
extern int All_PWM_left, All_PWM_right;
extern int16 real_speed;
extern int16 aim_speed;
extern float real_real_speed;
extern int Centerline_Err;



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


#endif /* CODE_FUSE_H_ */
