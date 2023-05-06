#include "cycle.h"
#include "control.h"
#include "pid.h"
#include "image.h"
#include "gyro.h"


short Left_Increase = 0;
short Right_Increase = 0;
short LDuty = 0, RDuty = 0, motor1 = 0, motor2 = 0;
int Direct_Last = 0;
short LMotor_Duty = 0;
short RMotor_Duty = 0;
unsigned char Speed_Parameter = 0;
float CurrentVoltage, PresentVoltage, TempVoltage;

int num = 0;
unsigned char data_array[4];
unsigned char truth;


void Speed_cycle(short expect){
    get_motor_speed();
    short Expect = 0;
    Expect = expect;
    Left_Increase = PID_Increase(&Lmotor_PID, Lmotor, speed1, Expect);
    Right_Increase = PID_Increase(&Rmotor_PID, Rmotor, speed2, Expect);

    LDuty = LDuty + Left_Increase;
    RDuty = RDuty + Right_Increase;
    tft180_show_int(0, 3, Left_Increase, 5);
    tft180_show_int(0, 4, Right_Increase, 5);

    motor_ctrl(LDuty, RDuty);
}


int Speed_deal(void){
    //get_motor_speed();
    int speed_now = (speed1 + speed2) / 2;
    int speed_set = 100;
    int Theory_Duty = PID_Realize(&Motor_PID, Motor, speed_now, speed_set);
    return Theory_Duty;
}


void Turn_cycle_ver2(short theory_duty){ //1600
    // cal_curvature();
    short Prospect_Parameter = PID_Realize_for_ProspectErr(&Prospect_PID, Prospect);
    short Bottom_Parameter = PID_Realize_for_BottomErr(&Bottom_PID, Bottom);
    short Union_result = Prospect_Parameter + Bottom_Parameter;
    // short Direct_Parameter = -PID_Realize(&Direct_PID, Direct, (int32)(Real_Gyro_Z * 10),(int32)Union_result);

    Direct_Last = Direct_Last * 0.2 + Prospect_Parameter * 0.8;
    LMotor_Duty = theory_duty + Direct_Last * 12;
    RMotor_Duty = theory_duty - Direct_Last * 12;
    motor_ctrl(LMotor_Duty, RMotor_Duty);
}



void Turn_cycle_ver3(short Theory_Duty){
    unsigned char x;
    unsigned char y;
    x = (centerline[119] + centerline[119 - 1] + centerline[119 - 2] + centerline[119 -3] + centerline[119 - 4]) / 5;
    y = 119 + 118 + 117 + 116 + 115;
    int Direct_Parameter = calculate_pid((one_curvature(x, y))*100);
    Direct_Last = Direct_Last * 0.2 + Direct_Parameter * 0.8;
    LMotor_Duty = Theory_Duty + Direct_Last * 10000;
    RMotor_Duty = Theory_Duty - Direct_Last * 10000;

    motor_ctrl(LMotor_Duty, RMotor_Duty);
}



void Turn_cycle_ForPCrossing(short theory_duty){ //1600
    // cal_curvature();
    short Prospect_Parameter = PID_Realize_for_ProspectErr(&Prospect_PID, Prospect);
    short Bottom_Parameter = PID_Realize_for_BottomErr(&Bottom_PID, Bottom);
    short Union_result = Prospect_Parameter + Bottom_Parameter;
    // short Direct_Parameter = -PID_Realize(&Direct_PID, Direct, (int32)(Real_Gyro_Z * 10),(int32)Union_result);

    Direct_Last = Direct_Last * 0.2 + Union_result * 0.8;
    LMotor_Duty = theory_duty - Direct_Last * 4;
    RMotor_Duty = theory_duty + Direct_Last * 4;

    motor_ctrl(LMotor_Duty, RMotor_Duty);
}



// 陀螺仪向右转的时候Z轴数据为正，向左转的时候Z轴速度为负，初步判断KD值为负值

extern S_FLOAT_XYZ GYRO_REAL, REAL_ACC;
float TKD = 0; // 13
float TGKD = 0; //0.11
float TKP = 1;
short Err, Last_Err;
short Direct_Duty;
void Turn_cycle(short Theory_Duty){
    #if 1
    short Prospect_Parameter = PID_Realize_for_ProspectErr(&Prospect_PID, Prospect);
    short Bottom_Parameter = PID_Realize_for_BottomErr(&Bottom_PID, Bottom);
    short Now_Err = Prospect_Parameter + Bottom_Parameter;
    Direct_Duty = Now_Err + (Now_Err - Last_Err) * TKD + GYRO_REAL.Z * TGKD;
    #else
    short Now_Err = Cal_centerline();
    Direct_Duty = Now_Err * TKP + (Now_Err - Last_Err) * TKD + GYRO_REAL.Z * TGKD;
    #endif
    LMotor_Duty = Theory_Duty - Direct_Duty * 7;    // 7
    RMotor_Duty = Theory_Duty + Direct_Duty * 7;
    Last_Err = Now_Err;
    // ***** //
    // LMotor_Duty = range_protect(LMotor_Duty, MOTOR_MIN, MOTOR_MAX);
    // RMotor_Duty = range_protect(RMotor_Duty, MOTOR_MIN, MOTOR_MAX);
    motor_ctrl(LMotor_Duty, RMotor_Duty);    
}
