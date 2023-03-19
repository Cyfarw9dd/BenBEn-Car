#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

extern short speed1, speed2;
extern short DUTY_MIN, DUTY_MAX;

void motor_ctrl(short Lmotor, short Rmotor);

void get_motor_speed(void);

void Quick_Break(void);

#endif /* CODE_CONTROL_H_ */
