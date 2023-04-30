#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define USER_SIZE_H 120
#define USER_SIZE_W 188

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色


//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值

extern unsigned char centerline[120], leftline[120], rightline[120];
extern unsigned char image_deal[MT9V03X_H][MT9V03X_W];
extern int Prospect_Err;
extern int Bottom_Err;
extern int further, middle, near;
extern unsigned char Left_RoadWidth[120], Right_RoadWidth[120];
extern short image_threshold;


extern unsigned char original_image[image_h][image_w];
extern unsigned char bin_image[image_h][image_w];  //图像数组
extern unsigned char l_border[image_h];//左线数组
extern unsigned char r_border[image_h];//右线数组
extern unsigned char center_line[image_h];//中线数组

extern void image_process(void);   //直接在中断或循环里调用此程序就可以循环执行了


void Camera(void);

unsigned char otsuThreshold(unsigned char *image, unsigned short col, unsigned short row);

void Searching_for_boundaries(unsigned char (*binary_array)[188]);

void Deal_Road_Characteristics(unsigned char (*binary_array)[188]);

void Hightlight_Lines(unsigned char (*binary_array)[188]);

void lcd_binaryzation032_zoom(unsigned char *p, unsigned char *q, unsigned short width, unsigned short height, unsigned char threshold);

void Four_neighbourhood_Filter(unsigned char (*binary_array)[188]);

float one_curvature(int x1, int y1);

void cal_curvature(void);

void sobel(unsigned char (*imageIn)[188], unsigned char (*imageOut)[188], unsigned char Threshold);

short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W]);

void regression(int type, int startline, int endline);

short Cal_BlackPoints(unsigned char (*binary_array)[188], unsigned char Start_Row, unsigned char End_Row);

void Eight_neighborhood(unsigned char flag);


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

#endif /* CODE_IMAGE_H_ */
