#ifndef CODE_LIST_H_
#define CODE_LIST_H_

#include "zf_common_headfile.h"

/* 
    按键标志位计时程序写在中断里，每隔一段时间中断标志位自增，定时器计数计时
    按键扫描写在
*/
#define KEY1 P22_0
#define KEY2 P22_2
#define KEY3 P22_1
#define KEY4 P22_3
#define TOGGLE1 P33_12
#define TOGGLE2 P33_13



typedef enum{
    nopress = 0,
    onepress,
    holdpress,
}KeySatateEnum;


extern KeySatateEnum Key1;
extern KeySatateEnum Key2;
extern KeySatateEnum Key3;
extern KeySatateEnum Key4;



void KeyParams_Init(void);

void KeyScan();

#endif /* CODE_LIST_H_ */
