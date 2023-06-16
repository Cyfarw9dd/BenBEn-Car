#ifndef CODE_ROAD_H_
#define CODE_ROAD_H_

#include "zf_common_headfile.h"

typedef enum{
    Straight = 0,
    Bend,
}Road_Type;

typedef struct{
    unsigned char entering_pflag;   // 进入元素前标志位
    unsigned char being_pflag;      // 进入元素后标志位
    unsigned char outing_pflag;     // 出元素标志位
    int trait_cnt;                  // 计时器
    float turn_value;               // 转角值
}Trait_smachine;

extern Trait_smachine Crossing;
extern Trait_smachine RoundAbout;
extern Trait_smachine Barrier;
extern Trait_smachine BreakRoad;
extern Trait_smachine Slope;

// 寻找角点
void Find_corners(void);

// 元素处理总函数，协调元素
void Traits_process(void);
// 十字路口
void Crossing_process(void);
// 环岛
void RoundAbout_process(void);
// 障碍
void Barrier_process(void);
// 断路
void BreakRoad_process(void);
// 坡道
void Slope_process(void);
// 判断弯道和直道
void Road_(void);

#endif /* CODE_ROAD_H_ */
