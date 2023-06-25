#ifndef CODE_ROAD_H_
#define CODE_ROAD_H_

#include "zf_common_headfile.h"

// 边线等距采样
extern unsigned short points_ls[MT9V03X_H * 3][2];
extern unsigned short points_rs[MT9V03X_H * 3][2];
extern int points_lnum, points_rnum;
// 左右边线局部角度变化率
extern float rpts0a[MT9V03X_H * 3];
extern float rpts1a[MT9V03X_H * 3];
extern int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
extern float rpts0an[MT9V03X_H * 3];
extern float rpts1an[MT9V03X_H * 3];
extern int rpts0an_num, rpts1an_num;
// Y角点
extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
extern bool Ypt0_found, Ypt1_found;

// L角点
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

// 长直道
extern bool is_straight0, is_straight1;

// 弯道
extern bool is_bend0, is_bend1;

extern unsigned char lnum;
extern unsigned char rnum;
extern unsigned char bend_flag;
typedef enum{
    Straight = 0,
    Bend,
}Road_Type;

typedef struct{
    unsigned char status;
    unsigned char pointflag;
    unsigned char cpt0_row, cpt0_col;
    unsigned char cpt1_row, cpt1_col;
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
// 八邻域边界等距采样, 每隔三个点取一个点
void sample_border(float *(points)[2], int num1, float *(points_s)[2], int *num2, int dist_point);
// 边线等距采样
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
// 角度变化率非极大抑制
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
// 寻找角点
void find_corners(void);

#endif /* CODE_ROAD_H_ */
