#ifndef CODE_LIST_H_
#define CODE_LIST_H_

#include "zf_common_headfile.h"

#define KEY1 P22_0
#define KEY2 P22_2
#define KEY3 P22_1
#define KEY4 P22_3

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

void KeyScan(void);

#endif /* CODE_LIST_H_ */
