#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

#define ENCODER_DIR_L                     (TIM5_ENCODER)                         // 左带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_L               (TIM5_ENCODER_CH1_P10_3)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_L                 (TIM5_ENCODER_CH2_P10_1)               // DIR 对应的引脚

#define ENCODER_DIR_R                     (TIM6_ENCODER)                         // 左带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_R               (TIM6_ENCODER_CH1_P20_3)               // PULSE 对应的引脚
#define ENCODER_DIR_DIR_R                 (TIM6_ENCODER_CH2_P20_0)               // DIR 对应的引脚

#define ENCODER_PIT                       (CCU60_CH0 )

#define PWM_CH1                            (ATOM0_CH4_P02_4)                     // PWM引脚1
#define PWM_CH2                            (ATOM0_CH5_P02_5)                     // PWM引脚2
#define PWM_CH3                            (ATOM0_CH6_P02_6)                     // PWM引脚3
#define PWM_CH4                            (ATOM0_CH7_P02_7)                     // PWM引脚4

#define MOTOR_MAX 4000                                                           // 电机限幅最大值
#define MOTOR_MIN 0                                                              // 电机限幅最小值
#define MOTOR_EXPECTATION 3000                                                   // 电机理论输出期望

extern short speed1, speed2;

void motor_ctrl(short Lmotor, short Rmotor);

void get_motor_speed(void);

void Quick_Break(void);

#endif /* CODE_CONTROL_H_ */
