#include "zf_common_headfile.h"
#include "control.h"
#include "pid.h"
#include "cycle.h"

short DUTY_MAX = 4000;  // ????
short DUTY_MIN = 0;     // ????
short speed1 = 0, speed2 = 0; // ???1?2

// ??????
void motor_ctrl(short Lmotor, short Rmotor){
    if(Lmotor >= 0){
        Lmotor = range_protect(Lmotor, DUTY_MIN, DUTY_MAX);
        pwm_set_duty(ATOM0_CH4_P02_4, Lmotor); 
        pwm_set_duty(ATOM0_CH5_P02_5, 0);
    }
    else{
        Lmotor = range_protect(-Lmotor, DUTY_MIN, DUTY_MAX);
        pwm_set_duty(ATOM0_CH4_P02_4, 0);      
        pwm_set_duty(ATOM0_CH5_P02_5, Lmotor);
    }
    if(Rmotor >= 0){
        Rmotor = range_protect(Rmotor, DUTY_MIN, DUTY_MAX);
        pwm_set_duty(ATOM0_CH6_P02_6, Rmotor); 
        pwm_set_duty(ATOM0_CH7_P02_7, 0);
    }
    else{
        Rmotor = range_protect(-Rmotor, DUTY_MIN, DUTY_MAX);
        pwm_set_duty(ATOM0_CH6_P02_6, 0);      
        pwm_set_duty(ATOM0_CH7_P02_7, Rmotor);
    }
}

// ???????
/*
    ?????1 -> TIM5_ENCODER
    ?????2 -> TIM6_ENCODER
    ???1?? -> TIM5_ENCODER_CH1_P10_3 || TIM5_ENCODER_CH2_P10_1
    ???2?? -> TIM6_ENCODER_CH1_P20_3 || TIM6_ENCODER_CH2_P20_0
*/
void get_motor_speed(void){
    speed1 = encoder_get_count(TIM5_ENCODER);
    speed2 = -encoder_get_count(TIM6_ENCODER);

    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
    tft180_show_int(0, 66, speed1, 5);
    tft180_show_int(0, 86, speed2, 5);
    // systick_delay_ms(STM0, 100);
}


// ????
void Quick_Break(void)
{
    short speed_now = (speed1 + speed2) / 2;
    if(speed_now > 50)
        motor_ctrl(-2000,-2000);
    else
        motor_ctrl(0,0);
}
