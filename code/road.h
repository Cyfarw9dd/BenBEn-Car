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

// 左右边线跳变列
extern int ldcptc[CLIP_IMAGE_H];
extern int rdcptc[CLIP_IMAGE_H];
// up
extern int lucptc[CLIP_IMAGE_H];
extern int rucptc[CLIP_IMAGE_H];

// 上拐点开始扫描列
#define Upcpt_Row 45
// 跳变列非极大极小值抑制
// 左线非极大值抑制，右线非极小值抑制
extern int ldown[CLIP_IMAGE_H];
extern int rdown[CLIP_IMAGE_H];
extern int lupon[CLIP_IMAGE_H];
extern int rupon[CLIP_IMAGE_H];

extern int Xpt0, Xpt1;
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
// 循迹模式
extern int track_mode;

extern int Lupon_id, Rupon_id;
extern bool Lupon_found, Rupon_found;

extern int Ldown_id, Rdown_id;
extern bool Ldown_found, Rdown_found;

extern float clip_ctline_k1;
extern float clip_ctline_k2;
extern bool straight_flag;
extern float kerr;
extern int straight_frame_flag;
extern int bend_frame_flag;
extern unsigned char lnum;
extern unsigned char rnum;
extern bool bend_flag;
extern int track_mode;

enum track_mode{
    NORMAL = 20,
    FARLINE,
    CLOSELINE,
    OBSTACLE,
    ADC,
    TURN,
    GARAGE_STOP,
    SPEED_UP,
    SLOW_DOWN,
    BEND,
};

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

void find_inflectionpoint(void);
// 寻找下拐点
void Downpoint_check(short clip_lfline[], short clip_rtline[], int ldcptc[], int rdcptc[]);
// 寻找上拐点
void Uponpoint_check(short clip_lfline[], short clip_rtline[], int lucptc[], int rucptc[]);

void find_changepoint(void);

void Track_line_l(Trackline *checkline);

void Track_line_r(Trackline *checkline);
// 元素处理总函数，协调元素
void Traits_process(void);

// 点集三角滤波
void blur_points(float pts_in[][2], int num, int kernel);

int clip(int x, int low, int up);

float fclip(float x, float low, float up);
// 八邻域边界等距采样, 每隔三个点取一个点
void sample_border(float *(points)[2], int num1, float *(points_s)[2], int *num2, int dist_point);
// 边线等距采样
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
// 左线角度变化率
void left_local_angle_points(int num, int dist);
// 右线角度变化率
void right_local_angle_points(int num, int dist);
// 左线非极大值抑制
void lnms_angle(int num, int kernel);
// 右线非极大值抑制
void rnms_angle(int num, int kernel);
// 寻找角点
void find_corners(void);

void roll_out(void);
// 左线非极大值抑制
void maximum(int num, int kernel, int *input, int *output);
// 右线非极小值抑制
void minimum(int num, int kernel, int *input, int *output);

float regression( int startline1, int endline1, int startline2, int endline2);

void Straight_process(void);

#endif /* CODE_ROAD_H_ */