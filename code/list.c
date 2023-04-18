#include "list.h"

void KeyParams_Init(void)
{
    Key1 = nopress;
    Key2 = nopress;
    Key3 = nopress;
    Key4 = nopress;
}


// 一般来说同一时间只会按下一个按键，所以多个按键共用同一个按键标志位也是可以的
void KeyScan(void)
{
    static unsigned char KeyPressNum = 0;
    static unsigned short KeyPressTime = 0;
    static unsigned char SomeKeyPress_Flag = 0; // 0松开 1按下 2消抖 3长按
    #define AlwaysPressTime 1200
    #define debouncing 5

    if(SomeKeyPress_Flag == 0 && !gpio_get_level(KEY1))
    {
        SomeKeyPress_Flag = 1;
    }
    if(SomeKeyPress_Flag > 0)
    {
        KeyPressTime++;
        // 5ms消抖
        if(SomeKeyPress_Flag == 1 && KeyPressTime >= debouncing)
        {
            SomeKeyPress_Flag = 2;
            if(gpio_get_level(KEY1))
            {
                KeyPressNum = 1;
            }
        }
        // 短按
        if(!gpio_get_level(KEY1) && KeyPressTime < AlwaysPressTime && SomeKeyPress_Flag == 2)
        {
            SomeKeyPress_Flag = 0;
            if(KeyPressNum == 1)
            {
                Key1 = onepress;
            }
        }
        // 长按
        if(KeyPressTime >= AlwaysPressTime && SomeKeyPress_Flag == 2)
        {
            if(KeyPressNum == 1)
            {
                Key1 = holdpress;
            }
            if(!gpio_get_level(KEY1))
            {
                SomeKeyPress_Flag = 0;
                KeyPressTime = 0;
                Key1 = nopress;
            }
        }
    }
}



