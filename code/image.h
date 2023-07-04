#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"
extern int position_x;
extern int position_y;
extern int position1_x;
extern int position1_y;
extern int ipts0_num;
extern int ipts1_num;

#define TopRow  0
#define BORDER_CLIP_ROW 75

//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度
#define CLIP_IMAGE_H 80

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值

#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

extern unsigned char centerline[120], leftline[120], rightline[120];
extern unsigned char image_deal[MT9V03X_H][MT9V03X_W];
extern unsigned char  Prospect_Err;
extern unsigned char  Bottom_Err;
extern unsigned char  further, middle, near;
extern unsigned char Left_RoadWidth[120], Right_RoadWidth[120];
extern int BlackPoints_Nums;
// extern short image_threshold;

extern unsigned short data_stastics_l;//统计左边找到点的个数
extern unsigned short data_stastics_r;//统计右边找到点的个数


extern unsigned char bin_image[image_h][MT9V03X_W];  //图像数组
extern unsigned char l_border[image_h];//左线数组
extern unsigned char r_border[image_h];//右线数组
extern unsigned char center_line[image_h];//中线数组
extern unsigned char left_lost_line; // 左边丢线数目
extern unsigned char right_lost_line;  // 右边丢线数目
extern unsigned char image_thereshold;
extern int centerline_k;
extern int ipts0[MT9V03X_H][2];
extern int ipts1[MT9V03X_H][2];
extern int ipts0_num, ipts1_num;
extern float Block_size;


extern unsigned short points_l[(unsigned short)USE_num][2];
extern unsigned short points_r[(unsigned short)USE_num][2];
extern unsigned short cutpoints_l[(unsigned short)USE_num][2];
extern unsigned short cutpoints_r[(unsigned short)USE_num][2];
extern unsigned char clip_image[CLIP_IMAGE_H][MT9V03X_W];
extern unsigned char clip_bin_image[CLIP_IMAGE_H][MT9V03X_W];
extern unsigned char clip_image_thereshold;
extern short clip_lfline[CLIP_IMAGE_H];
extern short clip_rtline[CLIP_IMAGE_H];
extern short clip_ctline[CLIP_IMAGE_H];




typedef struct
{
    unsigned char Left_RoadWidth[120];      // 左半边赛道宽度
    unsigned char Right_RoadWidth[120];     // 右半边赛道宽度
    int Left_Curve;         // 左线弧度
    int Right_Curve;        // 右线弧度
    int Curve_Err;          // 中线弧度偏差
} Road_Charac;

typedef struct image {
    uint8_t *data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} image_t;

typedef struct fimage {
    float *data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} fimage_t;


extern Road_Charac MyRoad_Charac;

void Deal_Road_Characteristics(unsigned char (*binary_array)[188], Road_Charac *);

void Hightlight_Lines(unsigned char (*binary_array)[188]);

void sobel(unsigned char (*imageIn)[188], unsigned char (*imageOut)[188], unsigned char Threshold);

short Cal_BlackPoints(unsigned char (*binary_array)[188], unsigned char Start_Row, unsigned char End_Row);

int my_abs(int value);

short limit_a_b(short x, int a, int b);

void Get_image(unsigned char(*mt9v03x_image)[image_w]);

void my_get_image(unsigned char (*mt9v03x_image)[188], unsigned char (*clip_image)[188]);

unsigned char OtsuThreshold(unsigned char *image, unsigned short col, unsigned short row);

void turn_to_bin(void);

void myturn_to_binary(unsigned char (*clip_image)[188], unsigned char (*clip_bin_image)[188]);

unsigned char get_start_point(unsigned char start_row);

unsigned char my_getstart_point(unsigned char start_row, unsigned char (*clip_bin_image)[188]);

void search_l_r(unsigned short break_flag, unsigned char(*image)[image_w], unsigned short *l_stastic, unsigned short *r_stastic, unsigned char l_start_x, unsigned char l_start_y, unsigned char r_start_x, unsigned char r_start_y, unsigned char*hightest);

void get_left(unsigned short total_L);

void my_get_left(unsigned short total_L);

void get_right(unsigned short total_R);

void my_get_right(unsigned short total_R);

void image_filter(unsigned char(*bin_image)[image_w]); //形态学滤波，简单来说就是膨胀和腐蚀的思想;

void image_draw_rectan(unsigned char(*image)[image_w]);

// void image_process(void);

void clip_imageprocess(void);

int Cal_centerline(void);

void LocalThresholding(void);

void findline_lefthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num);

void findline_righthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num);

void my_process_image(void);

unsigned char Gray_Search_Line(unsigned char(*img)[188],unsigned char i1,unsigned char j1,unsigned char i2,unsigned char j2,unsigned char thres);

void highlight_Lcorners(void);

void cut_borderline(void);

void cut_get_left(unsigned short total_L);

void cut_get_right(unsigned short total_R);

#endif /* CODE_IMAGE_H_ */
