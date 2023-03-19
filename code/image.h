#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define USER_SIZE_H 120
#define USER_SIZE_W 188

extern unsigned char centerline[120], leftline[120], rightline[120];
extern unsigned char image_deal[MT9V03X_H][MT9V03X_W];
extern int Prospect_Err;
extern int Bottom_Err;
extern int further, middle, near;
extern unsigned char Left_RoadWidth[120], Right_RoadWidth[120];
extern short image_threshold;


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



#endif /* CODE_IMAGE_H_ */
