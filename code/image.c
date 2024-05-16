#include "zf_common_headfile.h"
// 将一下变量存储在数据ram中
#pragma section all "cpu1_dsram"

#define Eightboundary 1
#define AT                  AT_IMAGE
#define AT_CLIP(img, x, y)  AT_IMAGE((img), clip((x), 0, (img)->width-1), clip((y), 0, (img)->height-1));
#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.width}
#define STATIC_THRESHOLD    70

unsigned short points_l[(unsigned short)USE_num][2] = { {  0 } };//左线
unsigned short points_r[(unsigned short)USE_num][2] = { {  0 } };//右线
unsigned short cutpoints_l[(unsigned short)USE_num][2] = {{0}};
unsigned short cutpoints_r[(unsigned short)USE_num][2] = {{0}};
unsigned short dir_r[(unsigned short)USE_num] = { 0 };//用来存储右边生长方向
unsigned short dir_l[(unsigned short)USE_num] = { 0 };//用来存储左边生长方向
unsigned short data_stastics_l = 0;//统计左边找到点的个数
unsigned short data_stastics_r = 0;//统计右边找到点的个数
unsigned char hightest = 0;//最高点
unsigned char start_point_l[2] = { 0 };//左边起点的x，y值
unsigned char start_point_r[2] = { 0 };//右边起点的x，y值

image_t img_raw = DEF_IMAGE(NULL, MT9V03X_W, MT9V03X_H);


unsigned char centerline[120];                  //定义中线数组，中线数组的横坐标等于（左线横坐标+右线横坐标）/ 2
unsigned char leftline[120];                    //定义左边线数组
unsigned char rightline[120];                   //定义右边线数组
unsigned char image_deal[MT9V03X_H][MT9V03X_W]; //声明一个二维数组，用于存放二值化后的图像，其中，下标MT9V03X_H，MT9V03X_W表示总钻风图像的高和宽
unsigned char Left_RoadWidth[120];              //定义左半边赛道宽度，即中线到左边线的距离
unsigned char Right_RoadWidth[120];             //定义右半边赛道宽度
// 裁剪后图像，分辨率为80*188
unsigned char clip_image[CLIP_IMAGE_H][MT9V03X_W];
unsigned char clip_bin_image[CLIP_IMAGE_H][MT9V03X_W];
// 裁剪后图像边线
// unsigned char -> char
short clip_lfline[CLIP_IMAGE_H];
short clip_rtline[CLIP_IMAGE_H];
short clip_ctline[CLIP_IMAGE_H];
// 裁剪后图像分割阈值
unsigned char clip_image_thereshold; 

unsigned char original_image[image_h][MT9V03X_W];
unsigned char bin_image[image_h][image_w];//图像数组
unsigned char image_thereshold;//图像分割阈值


int BlackPoints_Nums = 0;
// 迷宫寻线边线数组
int ipts0[MT9V03X_H][2];
int ipts1[MT9V03X_H][2];
// 迷宫边线数目
int ipts0_num, ipts1_num;
int *ipts0_nump = &ipts0_num;
int *ipts1_nump = &ipts1_num;
// 追踪用坐标
int position_x;
int position_y;
int position1_x;
int position1_y;
// 定义局部阈值方块大小
float Block_size = 5;

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int8 dir_front[4][2]= {{0,  -1},
                                {1,  0},
                                {0,  1},
                                {-1, 0}};
const int8 dir_frontleft[4][2] = {{-1, -1},
                                    {1,  -1},
                                    {1,  1},
                                    {-1, 1}};
const int8 dir_frontright[4][2] = {{1,  -1},
                                     {1,  1},
                                     {-1, 1},
                                     {-1, -1}};


/*这两个变量用于计算中线的偏差程度，并将计算出的结果交给转向环计算*/

unsigned char Prospect_Err = 0;                   //定义前瞻偏差，前瞻偏差的取值为实际中线上三个等距的点分别对理想中线做的差
unsigned char Bottom_Err = 0;                     //定义车身横向偏差，即摄像头拍到图像的最底端一行所处的中线值对理想中线做的差
unsigned char further, middle, near;              //图像中的远点，中点和近点

unsigned char left_lost_line;
unsigned char right_lost_line;
int centerline_k = 0;


Road_Charac MyRoad_Charac;    // 图像特征处理结构体
#pragma section all restore

void Deal_Road_Characteristics(unsigned char (*binary_array)[188], Road_Charac *rsptr){
    // for(unsigned char i = BottomRow; i > 0; i--){
    //     center_line[i] = (l_border[i] + r_border[i]) / 2;
    //     rsptr->Left_RoadWidth[i] = (unsigned char)absolute(93 - l_border[i]);
    //     rsptr->Right_RoadWidth[i] = (unsigned char)absolute(r_border[i] - 93);
    // }
    for (int i = CLIP_IMAGE_H - 1; i > 0; i--)
        clip_ctline[i] = (clip_lfline[i] + clip_rtline[i]) / 2;
    // 最小二乘法拟合中线，扫描平放下方的一块矩形区域，(187, 0)->(137, 187)
    // for (int i = BottomRow; i > BottomRow - 50; i--)
    // {
    //     center_line[i] = (int)(centerline_k * (i - BottomRow)) + 0;
    // }
}

void Hightlight_Lines(unsigned char (*binary_array)[188])
{
    // int height1 = 0;
    // int height2 = 0;
    // for (int i = 0; i < 359; i++)
    // {
    //     if (height1 != points_l[i][1])
    //         height1++;
    //     if (height1 > 75)
    //         break;
    //     tft180_draw_point(points_l[i][0] / 1.5, points_l[i][1] / 1.5, RGB565_WHITE);
        
    // }
    // for (int i = 0; i < 359; i++)
    // {;
    //     if (height2 != points_r[i][1])
    //         height2++;
    //     if (height2 > 75)
    //         break;
    //     tft180_draw_point(points_r[i][0] / 1.5, points_r[i][1] / 1.5, RGB565_WHITE);
        
    // }
    for (int i = 0; i < 359; i++)
    {
        tft180_draw_point(points_l[i][0] / 1.5, points_l[i][1] / 1.5, RGB565_WHITE);
        tft180_draw_point(points_r[i][0] / 1.5, points_r[i][1] / 1.5, RGB565_WHITE);
    }
}

/*
    索贝尔算子
*/
void sobel(unsigned char (*imageIn)[188], unsigned char (*imageOut)[188], unsigned char Threshold)
{
    /** 卷积核大小 */
    //short KERNEL_SIZE = 3;
    short xStart = 1;
    short xEnd = MT9V03X_W - 2;
    short yStart = 1;
    short yEnd = MT9V03X_H - 1;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            if (temp[0] > Threshold)
            {
                imageOut[i][j] = 255;
            }
            else
            {
                imageOut[i][j] = 0;
            }
        }
    }
}


short Cal_BlackPoints(unsigned char (*binary_array)[188], unsigned char Start_Row, unsigned char End_Row){
    short blackpoint;
    blackpoint = 0;
    for(unsigned char i = Start_Row; i > End_Row; i--){
        for(unsigned char j = 53; j < 133; i++){          // 一共扫描80列的黑点
            if(binary_array[i][j] == 0){
                blackpoint++;
            }
        }
    }
    return blackpoint;
}

// 以下为新的八邻域移植代码
// 可用的八邻域处理方法
#pragma section all "cpu1_psram"
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = (short)a;
    if(x>b) x = (short)b;
    return x;
}

int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}



void my_get_image(unsigned char (*mt9v03x_image)[188], unsigned char (*clip_image)[188])
{
    int height = CLIP_IMAGE_H - 1;
    for (int i = BottomRow; i > BottomRow - 80; i--)
    {
        for (int j = StartCoL; j < EndCoL; j++)
        {
            clip_image[height][j] = mt9v03x_image[i][j];
        }
        height--;
    }
}


unsigned char OtsuThreshold(unsigned char *image, unsigned short col, unsigned short row)
{
#define GrayScale 256
    unsigned short Image_Width  = col;
    unsigned short Image_Height = row;
    int X; unsigned short Y;
    unsigned char* data = image;
    int HistGram[GrayScale] = {0};
	
	unsigned int Amount = 0;
    unsigned int PixelBack = 0;
    unsigned int PixelIntegralBack = 0;
    unsigned int PixelIntegral = 0;
    int PixelIntegralFore = 0;
    int PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    unsigned char MinValue=0, MaxValue=0;
    unsigned char Threshold = 0;
	
	
    for (Y = Image_Height; Y > 0; Y--) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (unsigned char)Y;
          }
    }
   return Threshold;
}

void myturn_to_binary(unsigned char (*clip_image)[188], unsigned char (*clip_bin_image)[188])
{
    // 可以采用固定阈值
    clip_image_thereshold = OtsuThreshold(clip_image[0], CLIP_IMAGE_H, MT9V03X_W);  // STATIC_THRESHOLD
    for (int i = 0; i < CLIP_IMAGE_H; i++)
    {
        for (int j = StartCoL; j < EndCoL; j++)
        {
            if (clip_image[i][j] > clip_image_thereshold)   clip_bin_image[i][j] = white_pixel;
            else    clip_bin_image[i][j] = black_pixel;
        }
    }
}


unsigned char my_getstart_point(unsigned char start_row, unsigned char (*clip_bin_image)[188])
{
    unsigned char i = 0,l_found = 0,r_found = 0;

	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (clip_bin_image[start_row][i] == 255 && clip_bin_image[start_row][i - 1] == 0)
		{
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (clip_bin_image[start_row][i] == 255 && clip_bin_image[start_row][i + 1] == 0)
		{
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

void search_l_r(unsigned short break_flag, unsigned char(*image)[image_w], unsigned short *l_stastic, unsigned short *r_stastic, unsigned char l_start_x, unsigned char l_start_y, unsigned char r_start_x, unsigned char r_start_y, unsigned char *hightest)
{

	unsigned char i = 0, j = 0;

	//左边变量
	unsigned char search_filds_l[8][2] = { {  0 } };
	unsigned char index_l = 0;
	unsigned char temp_l[8][2] = { {  0 } };
	unsigned char center_point_l[2] = {  0 };
	unsigned short l_data_statics;
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };


	//右边变量
	unsigned char search_filds_r[8][2] = { {  0 } };
	unsigned char center_point_r[2] = { 0 };
	unsigned char index_r = 0;
	unsigned char temp_r[8][2] = { {  0 } };
	unsigned short r_data_statics;

	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };

	l_data_statics = *l_stastic;    // 统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

    //开启邻域循环
	while (break_flag--)
	{

		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一
		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}
		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}
			if (index_l)
			{
				//更新坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1])
					{
						center_point_l[0] = temp_l[j][0];//x
						center_point_l[1] = temp_l[j][1];//y
					}
				}
			}

		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			center_point_l[0] = (unsigned char) points_l[l_data_statics - 1][0];//x
			center_point_l[1] = (unsigned char) points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
			}
			if (index_r)
			{
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];
						center_point_r[1] = temp_r[j][1];
					}
				}

			}
		}


	}
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}

void my_get_left(unsigned short total_L)
{
    int	h = CLIP_IMAGE_H - 2;
	for (int j = 0; j < total_L; j++)
	{
		//printf("%d\n", j);
		if (points_l[j][1] == h)
		{
			clip_lfline[h] = points_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}



void my_get_right(unsigned short total_R)
{
    int h = CLIP_IMAGE_H - 2;
	for (int j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			clip_rtline[h] = points_r[j][0] - 1;
		}
		else continue;
		h--;
		if (h == 0)break;
	}    
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5   // 此参数可根据自己的需求调节
#define threshold_min	255*2
void image_filter(unsigned char(*bin_image)[image_w])
{
	unsigned short i, j;
	unsigned int num = 0;


	for (i = 1; i < CLIP_IMAGE_H - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;

			}

		}
	}

}
void image_draw_rectan(unsigned char(*image)[image_w])
{

	unsigned char i = 0;
	for (i = 0; i < CLIP_IMAGE_H; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[image_h-1][i] = 0;

	}
}

void clip_imageprocess(void)
{
    if (mt9v03x_finish_flag)
    {      

        unsigned char hightest = 0;     // 定义循环结束的最高行，试试40
        my_get_image(&mt9v03x_image[0], &clip_image[0]);
        myturn_to_binary(&clip_image[0], &clip_bin_image[0]);
        image_filter(&clip_bin_image[0]);        // 滤波，但是很占资源
        image_draw_rectan(&clip_bin_image[0]);   // 画框，让边界找到边框上
        //清零
        data_stastics_l = 0;
        data_stastics_r = 0;

        if (my_getstart_point(CLIP_IMAGE_H - 2, &clip_bin_image[0]))  // 把起点限定的高一点
        {
            search_l_r((unsigned short)USE_num, &clip_bin_image[0], &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
            // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
            my_get_left(data_stastics_l);
            my_get_right(data_stastics_r);
        }

        // 计算中线
        for (int i = CLIP_IMAGE_H - 1; i > 0; i--)
            clip_ctline[i] = (clip_lfline[i] + clip_rtline[i]) / 2;
 
        find_inflectionpoint();        // 寻找拐点   

        straight_frame_flag++;
        bend_frame_flag++;

    }
}


// 计算中线偏移量，并根据所选的循迹模式来执行不同的决策
int Cal_centerline(void)
{
    int ratio_sum = 0;
    int centerline_err_sum = 0;
    unsigned char centerline_ratio[] = 
    {
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
        22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
        25, 25, 25, 25, 25, 25, 25, 25, 25, 25,
        30, 30, 30, 30, 30, 30, 30, 50, 50, 50,
    };
    unsigned char farline_ratio[] = 
    {
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    };
    unsigned char closeline_ratio[] = 
    {
        25, 25, 25, 25, 25, 18, 18, 18, 15, 15,
    };
    if (track_mode == NORMAL)
    {
        for (int i = sizeof(centerline_ratio) / sizeof(centerline_ratio[0]); i > 0; i--)
        {
            ratio_sum += centerline_ratio[i];
            centerline_err_sum += (clip_ctline[(CLIP_IMAGE_H - 1) - i] - 93) * centerline_ratio[i]; ;
        }
        return centerline_err_sum / ratio_sum;
    }
    else if (track_mode == FARLINE)
    {
        for (int i = sizeof(farline_ratio) / sizeof(farline_ratio[0]); i > 0; i--)
        {
            ratio_sum += farline_ratio[i];
            centerline_err_sum += (clip_ctline[(CLIP_IMAGE_H - 1 - 50) - i] - 93) * farline_ratio[i]; ;
        }
        return centerline_err_sum / ratio_sum;
    }
    else if (track_mode == CLOSELINE)
    {
        for (int i = sizeof(closeline_ratio) / sizeof(closeline_ratio[0]); i > 0; i--)
        {
            ratio_sum += closeline_ratio[i];
            centerline_err_sum += (clip_ctline[(CLIP_IMAGE_H - 1 - 50) - i] - 93) * closeline_ratio[i]; ;
        }
        return centerline_err_sum / ratio_sum;
    }
    else 
        return 0;
}


// 左手迷宫巡线
void findline_lefthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num)
{

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
//     while (step < *num && half < x && x < width - half - 1 && half < y && y < height - half - 1 && turn < 4) {

//         int local_thres = 0;
//         for (int8 dy = -half; dy <= half; dy++) {
//             for (int8 dx = -half; dx <= half; dx++) {
//                 local_thres +=img[y + dy][x + dx];
//             }
//         }
//         local_thres /= block_size * block_size;
//         local_thres -= clip_value;//自适应二值的阈值

// //        unsigned char current_value = img[y][x];
//         unsigned char front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
//         unsigned char frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
//         if (front_value < local_thres) {
//             dir = (dir + 1) % 4;
//             turn++;
//         } else if (frontleft_value < local_thres) {
//             x += dir_front[dir][0];
//             y += dir_front[dir][1];
//             pts[step][0] = x;
//             pts[step][1] = y;
//             step++;
//             turn = 0;
//         } else {
//             x += dir_frontleft[dir][0];
//             y += dir_frontleft[dir][1];
//             dir = (dir + 3) % 4;
//             pts[step][0] = x;
//             pts[step][1] = y;
//             step++;
//             turn = 0;
//         }
//     }

   while (step <*num  && half < y && y < height - half - 1 && turn < 4) {

       if(half < x && x < width - half - 1)
       {
           int local_thres = 0;
           for (int8 dy = -half; dy <= half; dy++) {
               for (int8 dx = -half; dx <= half; dx++) {
                   local_thres +=img[y + dy][x + dx];
               }
           }
           local_thres /= block_size * block_size;
           local_thres -= clip_value;//自适应二值的阈值

//            unsigned char current_value = img[y][x];
           unsigned char front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
           unsigned char frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
           if (front_value < local_thres) {
               dir = (dir + 1) % 4;
               turn++;
           } else if (frontleft_value < local_thres) {
               x += dir_front[dir][0];
               y += dir_front[dir][1];
               pts[step][0] = x;
               pts[step][1] = y;
               step++;
               turn = 0;
           } else {
               x += dir_frontleft[dir][0];
               y += dir_frontleft[dir][1];
               dir = (dir + 3) % 4;
               pts[step][0] = x;
               pts[step][1] = y;
               step++;
               turn = 0;
           }
       }
       else {

               if(x==half)
                   x++;
               else if(x==width - half - 1)
                   x--;
           while(!Gray_Search_Line(img,y,x,y-1,x,10))
           {
               step++;
               y--;
               pts[step][0] = x;
               pts[step][1] = y;

           }

       }
      }
    *num = step;
}


// 右手迷宫巡线
void findline_righthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
//     while (step < *num && half < x && x < width - half - 1 && half < y && y < height - half - 1 && turn < 4) {

//         int16 local_thres = 0;
//         for (int8 dy = -half; dy <= half; dy++) {
//             for (int8 dx = -half; dx <= half; dx++) {
//                 local_thres += img[y + dy][x + dx];
//             }
//         }
//         local_thres /= block_size * block_size;
//         local_thres -= clip_value;

// //        unsigned char current_value =  img[y ][x ];
//         unsigned char front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
//         unsigned char frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
//         if (front_value < local_thres) {
//             dir = (dir + 3) % 4;
//             turn++;
//         } else if (frontright_value < local_thres) {
//             x += dir_front[dir][0];
//             y += dir_front[dir][1];
//             pts[step][0] = x;
//             pts[step][1] = y;
//             step++;
//             turn = 0;
//         } else {
//             x += dir_frontright[dir][0];
//             y += dir_frontright[dir][1];
//             dir = (dir + 1) % 4;
//             pts[step][0] = x;
//             pts[step][1] = y;
//             step++;
//             turn = 0;
//         }
//     }

while (step <*num && half < y && y < height - half - 1 && turn < 4) {

   if(half < x && x < width - half - 1)
   {
       int local_thres = 0;
       for (int dy = -half; dy <= half; dy++) {
           for (int dx = -half; dx <= half; dx++) {
               local_thres += img[y + dy][x + dx];
           }
       }
       local_thres /= block_size * block_size;
       local_thres -= clip_value;

//        unsigned char current_value =  img[y ][x ];
       unsigned char front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
       unsigned char frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
       if (front_value < local_thres) {
           dir = (dir + 3) % 4;
           turn++;
       } else if (frontright_value < local_thres) {
           x += dir_front[dir][0];
           y += dir_front[dir][1];
           pts[step][0] = x;
           pts[step][1] = y;
           step++;
           turn = 0;
       } else {
           x += dir_frontright[dir][0];
           y += dir_frontright[dir][1];
           dir = (dir + 1) % 4;
           pts[step][0] = x;
           pts[step][1] = y;
           step++;
           turn = 0;
       }
   }else
   {

       if(x==half)
          x++;
      else if(x==width - half - 1)
          x--;
       while(!Gray_Search_Line(img,y,x,y-1,x,10))
       {
           step++;
           y--;
           pts[step][0] = x;
           pts[step][1] = y;

       }

   }

   }
    *num = step;
}

void my_process_image(void) 
{
    // 原图找左右边线
    int x1 = MT9V03X_W / 2 - 20, y1 = BottomRow - 30;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    for (; x1 > StartCoL; x1--){
        if(abs(((mt9v03x_image[y1][x1] - mt9v03x_image[y1][x1 + 1]) * 100) / (mt9v03x_image[y1][x1] + mt9v03x_image[y1][x1 + 1])) > 28)
        {
            findline_lefthand_adaptive(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, Block_size, 2, x1 + 1, y1, ipts0, ipts0_nump);
            position_x = x1 + 1;
            position_y = y1;
            break;
        }
        else ipts0_num = 0;
    }
    int x2 = MT9V03X_W / 2 + 20, y2 = BottomRow - 30;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    for (; x2 < EndCoL; x2++)
    { 
        if(abs(((mt9v03x_image[y2][x2] - mt9v03x_image[y2][x2 - 1]) * 100) / (mt9v03x_image[y2][x2] + mt9v03x_image[y2][x2 - 1])) > 28)
        {
            findline_righthand_adaptive(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, Block_size, 2, x2 - 1, y2, ipts1, ipts1_nump);
            position1_x = x2 - 1;
            position1_y = y2;
            break;
        }
        else ipts1_num = 0;
    }
    
}


//两点像素做差比和比较
unsigned char Gray_Search_Line(unsigned char(*img)[188],unsigned char i1,unsigned char j1,unsigned char i2,unsigned char j2,unsigned char thres)
{
    int16 pixel_sum=img[i1][j1]+img[i2][j2];
    int16 pixel_deff=(img[i1][j1]>img[i2][j2]) ?
                     (img[i1][j1]-img[i2][j2]) :
                     (img[i2][j2]-img[i1][j1]) ;
    if(thres*pixel_sum<=pixel_deff*100)           //存在边界
    {
      return j2;                                  //返回列坐标
    }else{
      return 0;
    }
}


void highlight_Lcorners(void)
{
    // 显示左L角点
    clip_bin_image[points_l[Xpt0][1]][points_l[Xpt0][0]] = 120;
    clip_bin_image[points_l[Xpt0][1]][points_l[Xpt0][0] + 1] = 120;
    clip_bin_image[points_l[Xpt0][1]][points_l[Xpt0][0] - 1] = 120;
    clip_bin_image[points_l[Xpt0][1] + 1][points_l[Xpt0][0]] = 120;
    clip_bin_image[points_l[Xpt0][1] + 1][points_l[Xpt0][0] + 1] = 120;
    clip_bin_image[points_l[Xpt0][1] + 1][points_l[Xpt0][0] - 1] = 120;
    clip_bin_image[points_l[Xpt0][1] - 1][points_l[Xpt0][0]] = 120;
    clip_bin_image[points_l[Xpt0][1] - 1][points_l[Xpt0][0] + 1] = 120;
    clip_bin_image[points_l[Xpt0][1] - 1][points_l[Xpt0][0] - 1] = 120;
    // 显示右L角点
    clip_bin_image[points_l[Xpt1][1]][points_l[Xpt1][0]] = 120;
    clip_bin_image[points_l[Xpt1][1]][points_l[Xpt1][0] + 1] = 120;
    clip_bin_image[points_l[Xpt1][1]][points_l[Xpt1][0] - 1] = 120;
    clip_bin_image[points_l[Xpt1][1] + 1][points_l[Xpt1][0]] = 120;
    clip_bin_image[points_l[Xpt1][1] + 1][points_l[Xpt1][0] + 1] = 120;
    clip_bin_image[points_l[Xpt1][1] + 1][points_l[Xpt1][0] - 1] = 120;
    clip_bin_image[points_l[Xpt1][1] - 1][points_l[Xpt1][0]] = 120;
    clip_bin_image[points_l[Xpt1][1] - 1][points_l[Xpt1][0] + 1] = 120;
    clip_bin_image[points_l[Xpt1][1] - 1][points_l[Xpt1][0] - 1] = 120;
}


void cut_borderline(void)
{
    int height_l = 0;
    int height_r = 0;
    for (int i = 0; i < 359; i++)
    {
        if (height_l != points_l[i][1])
            height_l++;
        if (height_l > BORDER_CLIP_ROW)
            break;
        cutpoints_l[i][0] = points_l[i][0];
        cutpoints_l[i][1] = points_l[i][1];
    }
    for (int i = 0; i < 359; i++)
    {
        if (height_r != points_r[i][1])
            height_r++;
        if (height_r > BORDER_CLIP_ROW)
            break;
        cutpoints_r[i][0] = points_r[i][0];
        cutpoints_r[i][1] = points_r[i][1];
    }
}

void cut_get_left(unsigned short total_L)
{
    int	h = CLIP_IMAGE_H - 2;
	for (int j = 0; j < total_L; j++)
	{
		if (cutpoints_l[j][1] == h)
		{
			clip_lfline[h] = cutpoints_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}

void cut_get_right(unsigned short total_R)
{
    int h = CLIP_IMAGE_H - 2;
	for (int j = 0; j < total_R; j++)
	{
		if (cutpoints_r[j][1] == h)
		{
			clip_rtline[h] = cutpoints_r[j][0] - 1;
		}
		else continue;
		h--;
		if (h == 0)break;
	}    
}

#pragma section all restore