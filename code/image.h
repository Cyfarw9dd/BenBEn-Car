#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

extern int position_x;
extern int position_y;
extern int position1_x;
extern int position1_y;
extern int ipts0_num;
extern int ipts1_num;

#define USER_SIZE_H 120
#define USER_SIZE_W 188

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色
#define TopRow  0


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

extern unsigned char centerline[120], leftline[120], rightline[120];
extern unsigned char image_deal[MT9V03X_H][MT9V03X_W];
extern unsigned char  Prospect_Err;
extern unsigned char  Bottom_Err;
extern unsigned char  further, middle, near;
extern unsigned char Left_RoadWidth[120], Right_RoadWidth[120];
extern int BlackPoints_Nums;
// extern short image_threshold;


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

extern void image_process(void);   //直接在中断或循环里调用此程序就可以循环执行了


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

int regression(int startline,int endline);

short Cal_BlackPoints(unsigned char (*binary_array)[188], unsigned char Start_Row, unsigned char End_Row);

int my_abs(int value);

short limit_a_b(short x, int a, int b);

void Get_image(unsigned char(*mt9v03x_image)[image_w]);

unsigned char OtsuThreshold(unsigned char *image, unsigned short col, unsigned short row);

void turn_to_bin(void);

unsigned char get_start_point(unsigned char start_row);

void search_l_r(unsigned short break_flag, unsigned char(*image)[image_w], unsigned short *l_stastic, unsigned short *r_stastic, unsigned char l_start_x, unsigned char l_start_y, unsigned char r_start_x, unsigned char r_start_y, unsigned char*hightest);

void get_left(unsigned short total_L);

void get_right(unsigned short total_R);

void get_centerline_ver2(void);

void image_filter(unsigned char(*bin_image)[image_w]); //形态学滤波，简单来说就是膨胀和腐蚀的思想;

void image_draw_rectan(unsigned char(*image)[image_w]);

// void image_process(void);

int Cal_centerline(void);

void LocalThresholding(void);

void findline_lefthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num);

void findline_righthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num);

void my_process_image(void);

unsigned char Gray_Search_Line(unsigned char(*img)[188],unsigned char i1,unsigned char j1,unsigned char i2,unsigned char j2,unsigned char thres);

#endif /* CODE_IMAGE_H_ */
