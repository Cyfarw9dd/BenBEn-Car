#ifndef CODE_ROAD_H_
#define CODE_ROAD_H_

#include "zf_common_headfile.h"
extern unsigned char lnum;
extern unsigned char rnum;
extern unsigned char bend_flag;
typedef enum{
    Straight = 0,
    Bend,
}Road_Type;

typedef struct{
    unsigned char pointflag;
    int trait_cnt;                  // 计时器
    float turn_value;               // 转角值
}Trait_smachine;

typedef struct{
    unsigned char startrow;
    unsigned char endrow;
    unsigned char row[3];   // 前两个元素存储上下拐点
    unsigned char col[3];
    unsigned char find;
}Trackline;

extern Trait_smachine Crossing;
extern Trait_smachine RoundAbout;
extern Trait_smachine Barrier;
extern Trait_smachine BreakRoad;
extern Trait_smachine Slope;
extern Trait_smachine Startline;
extern Trackline checkline_l;
extern Trackline checkline_r;

// 寻找角点
void Find_corners(void);

void Lostline_count(unsigned char LlineF[120], unsigned char RlineF[120], unsigned char *lcnt, unsigned char *rcnt);
// 寻找断点
void Track_line_l(Trackline *checkline);

void Track_line_r(Trackline *checkline);
// 元素处理总函数，协调元素
void Traits_process(void);
// 十字路口
void Crossing_process(void);
// 环岛
void RoundAbout_process(void);
// 障碍
void Barrier_process(void);
// 断路
void BreakRoad_process(Trait_smachine *road_smachine);
// 坡道
void Slope_process(void);
// 判断弯道和直道
void Startline_process(Trait_smachine *road_smachine, unsigned char (*binary_array)[188]);
// 点集三角滤波
void blur_points(float pts_in[][2], int num, int kernel);

int clip(int x, int low, int up);

float fclip(float x, float low, float up);

void Find_corners(void);

#endif /* CODE_ROAD_H_ */
