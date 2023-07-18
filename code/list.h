#ifndef CODE_LIST_H_
#define CODE_LIST_H_

#include "zf_common_headfile.h"

extern char Key1_last_status;
extern char Key2_last_status;
extern char Key3_last_status;
extern char Key4_last_status;
extern char Key1_status;
extern char Key2_status;
extern char Key3_status;
extern char Key4_status;

extern char Key1_flag;
extern char Key2_flag;
extern char Key3_flag;
extern char Key4_flag;

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

void MyKeyScan(void);

void Key_flag_clear(void);

int have_sub_list(int ListIndex);

int show_sub_list(int parent_index, int highlight_col);

void List_Switch(void);

void show_gray_image(void);

void show_binary_image(void);

void show_boundaries(void);

void tuning(void);

void show_params(void);

void show_roadtraits(void);

void show_corners(void);

void show_sample_boundaries(void);

void show_barrier_params(void);

void debug_mode(void);

void show_inflectionpoint(void);

void show_breakroad(void);

void show_straight(void);

void show_zebra(void);

#endif /* CODE_LIST_H_ */
