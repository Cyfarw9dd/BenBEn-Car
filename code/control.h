#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

#define ENCODER_DIR_L                     (TIM2_ENCODER)                         // 左带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_L               (TIM2_ENCODER_CH2_P33_6)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_L                 (TIM2_ENCODER_CH1_P33_7)               // DIR 对应的引脚

#define ENCODER_DIR_R                     (TIM6_ENCODER)                         // 右带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_R               (TIM6_ENCODER_CH1_P20_3)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_R                 (TIM6_ENCODER_CH2_P20_0)               // DIR 对应的引脚

#define ENCODER_PIT                        (CCU60_CH0 )

#define PWM_CH1                            (ATOM0_CH0_P21_2)                     // PWM引脚1  
#define PWM_CH2                            (ATOM0_CH2_P21_4)                     // PWM引脚2
#define PWM_CH3                            (ATOM0_CH3_P21_5)                     // PWM引脚3
#define PWM_CH4                            (ATOM0_CH1_P21_3)                     // PWM引脚4   

#define MOTOR_MAX 6000                                                           // 电机限幅最大值
#define MOTOR_MIN 0                                                              // 电机限幅最小值
#define MOTOR_EXPECTATION 3000                                                   // 电机理论输出期望

#define BUZZER ATOM2_CH0_P33_10                                                  // 定义蜂鸣器引脚

// #define CURVE_ERR 50
enum motor_mode
{
    MODE_NORMAL = 0,
    MODE_TURN,
};

extern short speed1, speed2;

void motor_ctrl(short Lmotor, short Rmotor);

void get_motor_speed(void);

void Quick_Break(void);

void Buzzer(void);

#endif /* CODE_CONTROL_H_ */


