#include "zf_common_headfile.h"



short speed1 = 0, speed2 = 0; // 定义编码器值获取变量
int buzzer_cnt = 0;
int buzzer_flag = 0;

float speed_array[2];

// 电机控制
void motor_ctrl(short Lmotor, short Rmotor){
    //     if(Lmotor >= 0){
    //     Lmotor = range_protect(Lmotor, MOTOR_MIN, MOTOR_MAX);
    //     pwm_set_duty(PWM_CH1, Lmotor);      
    //     pwm_set_duty(PWM_CH2, 0);
    // }
    // else{
    //     Lmotor = range_protect(-Lmotor, MOTOR_MIN, MOTOR_MAX);
    //     pwm_set_duty(PWM_CH1, 0); 
    //     pwm_set_duty(PWM_CH2, Lmotor);
    // }
    // if(Rmotor >= 0){
    //     Rmotor = range_protect(Rmotor, MOTOR_MIN, MOTOR_MAX);
    //     pwm_set_duty(PWM_CH3, Rmotor);      
    //     pwm_set_duty(PWM_CH4, 0);
    // }
    // else{
    //     Rmotor = range_protect(-Rmotor, MOTOR_MIN, MOTOR_MAX);
    //     pwm_set_duty(PWM_CH3, 0); 
    //     pwm_set_duty(PWM_CH4, Rmotor);
    // }
    
    if(Lmotor >= 0){
        Lmotor = Lmotor <= MOTOR_MAX ? Lmotor : MOTOR_MAX;
        // Lmotor = range_protect(Lmotor, MOTOR_MIN, MOTOR_MAX);
        pwm_set_duty(PWM_CH1, Lmotor); 
        pwm_set_duty(PWM_CH2, 0);
    }
    else{
        Lmotor = Lmotor >= -MOTOR_MAX ? (-Lmotor) : MOTOR_MAX;
        // Lmotor = range_protect(-Lmotor, MOTOR_MIN, MOTOR_MAX);
        pwm_set_duty(PWM_CH1, 0);      
        pwm_set_duty(PWM_CH2, Lmotor);
    }
    if(Rmotor >= 0){
        Rmotor = Rmotor <= MOTOR_MAX ? Rmotor : MOTOR_MAX;
        // Rmotor = range_protect(Rmotor, MOTOR_MIN, MOTOR_MAX);
        pwm_set_duty(PWM_CH3, Rmotor); 
        pwm_set_duty(PWM_CH4, 0);
    }
    else{
        Rmotor = Rmotor >= -MOTOR_MAX ? (-Rmotor) : MOTOR_MAX;
        // Rmotor = range_protect(-Rmotor, MOTOR_MIN, MOTOR_MAX);
        pwm_set_duty(PWM_CH3, 0);      
        pwm_set_duty(PWM_CH4, Rmotor);
    }
    // tft180_show_float(0, 90, speed1, 5, 2);         tft180_show_float(60, 90, speed2, 5, 2);
}

// 获取编码器数据
/*
    编码器通道1 -> TIM5_ENCODER
    编码器通道2 -> TIM6_ENCODER
    编码器1引脚 -> TIM5_ENCODER_CH1_P10_3 || TIM5_ENCODER_CH2_P10_1
    编码器2引脚 -> TIM6_ENCODER_CH1_P20_3 || TIM6_ENCODER_CH2_P20_0
*/
void get_motor_speed(void){
    speed1 = -encoder_get_count(ENCODER_DIR_L);
    speed2 = encoder_get_count(ENCODER_DIR_R);
    real_speed = (speed1 + speed2) / 2;
    left_distance += speed1;
    right_distance +=speed2;
    barrier_turning_distance = (left_distance + right_distance) / 2;
    // put_int32(0, right_distance);
    // put_int32(0, real_speed);
    
    encoder_clear_count(ENCODER_DIR_L);
    encoder_clear_count(ENCODER_DIR_R);
}


// 快速停车
void Quick_Break(void)
{
    short speed_now = (speed1 + speed2) / 2;
    if(speed_now > 50)
        motor_ctrl(-2000,-2000);
    else
        motor_ctrl(0,0);
}

void Buzzer(void)
{
    // 蜂鸣器相应函数，计数在中断中执行
    pwm_set_duty(BUZZER, 7000);
    // system_delay_ms(1000);
    // pwm_set_duty(BUZZER, 0);
    // system_delay_ms(1000);
}

// 速度决策将运行在速度环中，计算结果串在转向内环中

// void Speed_Deal(void)
// {
//     if ()
// }