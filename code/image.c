#include "zf_common_headfile.h"

#define Eightboundary 1
#define AT                  AT_IMAGE
#define AT_CLIP(img, x, y)  AT_IMAGE((img), clip((x), 0, (img)->width-1), clip((y), 0, (img)->height-1));
#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.width}

image_t img_raw = DEF_IMAGE(NULL, MT9V03X_W, MT9V03X_H);


unsigned char centerline[120];                  //定义中线数组，中线数组的横坐标等于（左线横坐标+右线横坐标）/ 2
unsigned char leftline[120];                    //定义左边线数组
unsigned char rightline[120];                   //定义右边线数组
unsigned char image_deal[MT9V03X_H][MT9V03X_W]; //声明一个二维数组，用于存放二值化后的图像，其中，下标MT9V03X_H，MT9V03X_W表示总钻风图像的高和宽
unsigned char Left_RoadWidth[120];              //定义左半边赛道宽度，即中线到左边线的距离
unsigned char Right_RoadWidth[120];             //定义右半边赛道宽度

unsigned char original_image[image_h][MT9V03X_W];
unsigned char bin_image[image_h][image_w];//图像数组
unsigned char image_thereshold;//图像分割阈值


int BlackPoints_Nums = 0;
int ipts0[MT9V03X_H][2];
int ipts1[MT9V03X_H][2];
int ipts0_num, ipts1_num;
float block_size = 5;


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


void Deal_Road_Characteristics(unsigned char (*binary_array)[188], Road_Charac *rsptr){
    for(unsigned char i = BottomRow; i > 0; i--){
        center_line[i] = (l_border[i] + r_border[i]) / 2;
        rsptr->Left_RoadWidth[i] = (unsigned char)absolute(93 - l_border[i]);
        rsptr->Right_RoadWidth[i] = (unsigned char)absolute(r_border[i] - 93);
    }
    // 最小二乘法拟合中线，扫描平放下方的一块矩形区域，(187, 0)->(137, 187)
    // for (int i = BottomRow; i > BottomRow - 50; i--)
    // {
    //     center_line[i] = (int)(centerline_k * (i - BottomRow)) + 0;
    // }
}

void Hightlight_Lines(unsigned char (*binary_array)[188]){
    // for(unsigned char i = BottomRow; i > 0; i--){
    //     tft180_draw_point((l_border[i] + 5) / 2, i / 2, RGB565_BLUE);
    //     tft180_draw_point(center_line[i] / 2, i / 2, RGB565_RED);
    //     tft180_draw_point((r_border[i] - 5)  / 2, i / 2, RGB565_GREEN);
    // }
	for (int i = BottomRow; i > 0; i--)
	{
		tft180_draw_point(ipts0[i][0] / 2, ipts0[i][1] / 2, RGB565_BLUE);
		tft180_draw_point(ipts1[i][0] / 2, ipts1[i][1] / 2, RGB565_GREEN);
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

/************************************线性回归计算中线斜率************************************/
// y = Ax+B
int regression(int startline,int endline)
{
    int i = 0, SumX = 0, SumY = 0, SumLines = 0; 
    float SumUp = 0, SumDown = 0, avrX = 0, avrY=0;
    int A, B;
    SumLines = endline - startline;   // startline 为开始行， //endline 结束行 //SumLines

    for(i=startline;i<endline;i++)     
    { 
    SumX+=i;       
    SumY+=center_line[i];    //这里Middle_black为存放中线的数组
    }         
    avrX=(float)SumX/SumLines;     //X的平均值
    avrY=(float)SumY/SumLines;     //Y的平均值       
    SumUp=0;      
    SumDown=0;  
    for(i=startline;i<endline;i++)   
    {       
    SumUp+=(center_line[i]-avrY)*(i-avrX);    
    SumDown+=(i-avrX)*(i-avrX);    
    }    
    if(SumDown==0) 
    B=0;  
    else 
    B=(int)(SumUp/SumDown);       
    A=(SumY-B*SumX)/SumLines;  //截距
    return B;  //返回斜率
}


/*
    函数名称：黑点计数
    函数参数：二值化后图像的原始二维数组，开始计算的行数，结束计算的行数
    调用实例：Cal_BlackPoints(&image_deal[0], 119, 0);
*/
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

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
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

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}



//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(unsigned char(*mt9v03x_image)[image_w])
{
#define use_num		1	//1就是不压缩，2就是压缩一倍	
	unsigned char i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
			line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
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
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
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
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
unsigned char bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
{
  unsigned char i,j;
 image_thereshold = OtsuThreshold(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


/*
函数名称：void get_start_point(unsigned char start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
unsigned char start_point_l[2] = { 0 };//左边起点的x，y值
unsigned char start_point_r[2] = { 0 };//右边起点的x，y值
unsigned char get_start_point(unsigned char start_row)
{
	unsigned char i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
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

/*
函数名称：void search_l_r(unsigned short break_flag, unsigned char(*image)[image_w],unsigned short *l_stastic, unsigned short *r_stastic,
							unsigned char l_start_x, unsigned char l_start_y, unsigned char r_start_x, unsigned char r_start_y,unsigned char*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[image_w]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r((unsigned short)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
unsigned short points_l[(unsigned short)USE_num][2] = { {  0 } };//左线
unsigned short points_r[(unsigned short)USE_num][2] = { {  0 } };//右线
unsigned short dir_r[(unsigned short)USE_num] = { 0 };//用来存储右边生长方向
unsigned short dir_l[(unsigned short)USE_num] = { 0 };//用来存储左边生长方向
unsigned short data_stastics_l = 0;//统计左边找到点的个数
unsigned short data_stastics_r = 0;//统计右边找到点的个数
unsigned char hightest = 0;//最高点
void search_l_r(unsigned short break_flag, unsigned char(*image)[image_w], unsigned short *l_stastic, unsigned short *r_stastic, unsigned char l_start_x, unsigned char l_start_y, unsigned char r_start_x, unsigned char r_start_y, unsigned char*hightest)
{

	unsigned char i = 0, j = 0;

	//左边变量
	unsigned char search_filds_l[8][2] = { {  0 } };
	unsigned char index_l = 0;
	unsigned char temp_l[8][2] = { {  0 } };
	unsigned char center_point_l[2] = {  0 };
	unsigned short l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	unsigned char search_filds_r[8][2] = { {  0 } };
	unsigned char center_point_r[2] = { 0 };//中心坐标点
	unsigned char index_r = 0;//索引下标
	unsigned char temp_r[8][2] = { {  0 } };
	unsigned short r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

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
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
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
			//printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			// printf("\n如果左边比右边高了，左边等待右边\n");	
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
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
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void get_left(unsigned short total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
unsigned char l_border[image_h];//左线数组
unsigned char r_border[image_h];//右线数组
unsigned char center_line[image_h];//中线数组
void get_left(unsigned short total_L)
{
	unsigned char i = 0;
	unsigned short j = 0;
	unsigned char h = 0;
	//初始化
	for (i = 0;i<image_h;i++)
	{
		l_border[i] = border_min;
	}
	h = image_h - 2;
	//左边
	for (j = 0; j < total_L; j++)
	{
		//printf("%d\n", j);
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}
/*
函数名称：void get_right(unsigned short total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(unsigned short total_R)
{
	unsigned char i = 0;
	unsigned short j = 0;
	unsigned char h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h - 2;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}

void get_centerline_ver2(void){
    for(unsigned char i = 119; i > 0; i--){
        centerline[i] = (r_border[i] + l_border[i]) / 2;
    }
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节
void image_filter(unsigned char(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	unsigned short i, j;
	unsigned int num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//黑

			}

		}
	}

}

/*
函数名称：void image_draw_rectan(unsigned char(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：unsigned char(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(unsigned char(*image)[image_w])
{

	unsigned char i = 0;
	for (i = 0; i < image_h; i++)
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

/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
*/
void image_process(void)
{
unsigned short i;
unsigned char hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
/*这是离线调试用的*/
Get_image(&mt9v03x_image[0]);
turn_to_bin();
/*提取赛道边界*/
image_filter(&bin_image[0]);//滤波
image_draw_rectan(&bin_image[0]);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
	// printf("正在开始八领域\n");
	search_l_r((unsigned short)USE_num, &bin_image[0], &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	// printf("八邻域已结束\n");
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
	//处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}
centerline_k = regression(BottomRow - 50, BottomRow);

}

int Cal_centerline(void)
{
    int ratio_sum = 0;
    int centerline_err_sum;
    unsigned char centerline_ratio[] = 
    {
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
        22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
        25, 25, 25, 25, 25, 25, 25, 25, 25, 25,
        30, 30, 30, 30, 30, 30, 30, 50, 50, 50,
    };
    for (int i = sizeof(centerline_ratio) / sizeof(unsigned char); i > 0; i--)
    {
        ratio_sum += centerline_ratio[i];
        centerline_err_sum += (center_line[MT9V03X_H - i] - 93) * centerline_ratio[i]; ;
    }
    return centerline_err_sum / ratio_sum;
}

void Cal_lostline(void)
{
    for (unsigned char i = 119; i > 0; i--)
    {
        if (l_border[i] == 0)
        {
            left_lost_line++;
        }
        if (r_border[i] == 187)
        {
            right_lost_line++;
        }
    }
}


void LocalThresholding(void)
{
    // unsigned char *image_ptr[188];
    unsigned char thres;    // 定义局部二值化阈值  
    for (int current_row = BottomRow; current_row > 3;)     // 自下而上遍历行数，到倒数第四行截止
    {   
        if (current_row < 3)                                // 限制条件
            break;
        for (int current_col = StartCoL; current_col < (187 - 4);)      // 从左到右遍历列数，到倒数第四列截止
        {
            if (current_col > (187 - 4))    
                break;
            // 将4x4范围内的图像矩阵存储在数组中
            unsigned char image_data[16] = 
            {
                mt9v03x_image[current_row][current_col], mt9v03x_image[current_row][current_col + 1], mt9v03x_image[current_row][current_col + 1], mt9v03x_image[current_row][current_col + 3],
                mt9v03x_image[current_row - 1][current_col], mt9v03x_image[current_row - 1][current_col + 1], mt9v03x_image[current_row - 1][current_col + 1], mt9v03x_image[current_row - 1][current_col + 3],
                mt9v03x_image[current_row - 2][current_col], mt9v03x_image[current_row - 2][current_col + 1], mt9v03x_image[current_row - 2][current_col + 1], mt9v03x_image[current_row - 2][current_col + 3],
                mt9v03x_image[current_row - 3][current_col], mt9v03x_image[current_row - 3][current_col + 1], mt9v03x_image[current_row - 3][current_col + 1], mt9v03x_image[current_row - 3][current_col + 3]
            };
            // 运行大津法
            thres = OtsuThreshold(image_data, current_col, current_row);
            // unsigned char temp[16];
            for (int m = 0; m < 16; m++)
            {
                if (image_data > thres)     image_data[m] = 255;
                if (image_data < thres)     image_data[m] = 0;
            }
            for (int row = current_row; row > current_row - 4; row--)
            {
                for (int col = current_col; col < current_col + 4; col++)
                {
                    image_deal[row][col] = image_data[row + col];
                }
            }
            // image_deal
            current_col -= 4;
        }
        current_row -= 4;
    }
}

// 左手迷宫巡线
void findline_lefthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num)
{

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step <*num && half < x && x < width - half - 1 && half < y && y < height - half - 1 && turn < 4) {

        int local_thres = 0;
        for (int8 dy = -half; dy <= half; dy++) {
            for (int8 dx = -half; dx <= half; dx++) {
                local_thres +=img[y + dy][x + dx];
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;//自适应二值的阈值

//        unsigned char current_value = img[y][x];
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

//    while (step <*num  && half < y && y < height - half - 1 && turn < 4) {
//
//        if(half < x && x < width - half - 1)
//        {
//            int local_thres = 0;
//            for (int8 dy = -half; dy <= half; dy++) {
//                for (int8 dx = -half; dx <= half; dx++) {
//                    local_thres +=img[y + dy][x + dx];
//                }
//            }
//            local_thres /= block_size * block_size;
//            local_thres -= clip_value;//自适应二值的阈值
//
////            unsigned char current_value = img[y][x];
//            unsigned char front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
//            unsigned char frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
//            if (front_value < local_thres) {
//                dir = (dir + 1) % 4;
//                turn++;
//            } else if (frontleft_value < local_thres) {
//                x += dir_front[dir][0];
//                y += dir_front[dir][1];
//                pts[step][0] = x;
//                pts[step][1] = y;
//                step++;
//                turn = 0;
//            } else {
//                x += dir_frontleft[dir][0];
//                y += dir_frontleft[dir][1];
//                dir = (dir + 3) % 4;
//                pts[step][0] = x;
//                pts[step][1] = y;
//                step++;
//                turn = 0;
//            }
//        }
//        else {
//
//                if(x==half)
//                    x++;
//                else if(x==width - half - 1)
//                    x--;
//            while(!Gray_Search_Line(img,y,x,y-1,x,10))
//            {
//                step++;
//                y--;
//                pts[step][0] = x;
//                pts[step][1] = y;
//
//            }
//
//        }
//       }
    *num = step;
}


// 右手迷宫巡线
void findline_righthand_adaptive(unsigned char(*img)[188],unsigned char width,unsigned char height, unsigned char block_size, unsigned char clip_value, unsigned char x, unsigned char y, unsigned char (*pts)[2], unsigned char *num)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step <*num && half < x && x < width - half - 1 && half < y && y < height - half - 1 && turn < 4) {

        int16 local_thres = 0;
        for (int8 dy = -half; dy <= half; dy++) {
            for (int8 dx = -half; dx <= half; dx++) {
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
    }

//while (step <*num && half < y && y < height - half - 1 && turn < 4) {
//
//    if(half < x && x < width - half - 1)
//    {
//        int local_thres = 0;
//        for (int dy = -half; dy <= half; dy++) {
//            for (int dx = -half; dx <= half; dx++) {
//                local_thres += img[y + dy][x + dx];
//            }
//        }
//        local_thres /= block_size * block_size;
//        local_thres -= clip_value;
//
////        unsigned char current_value =  img[y ][x ];
//        unsigned char front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
//        unsigned char frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
//        if (front_value < local_thres) {
//            dir = (dir + 3) % 4;
//            turn++;
//        } else if (frontright_value < local_thres) {
//            x += dir_front[dir][0];
//            y += dir_front[dir][1];
//            pts[step][0] = x;
//            pts[step][1] = y;
//            step++;
//            turn = 0;
//        } else {
//            x += dir_frontright[dir][0];
//            y += dir_frontright[dir][1];
//            dir = (dir + 1) % 4;
//            pts[step][0] = x;
//            pts[step][1] = y;
//            step++;
//            turn = 0;
//        }
//    }else
//    {
//
//        if(x==half)
//           x++;
//       else if(x==width - half - 1)
//           x--;
//        while(!Gray_Search_Line(img,y,x,y-1,x,10))
//        {
//            step++;
//            y--;
//            pts[step][0] = x;
//            pts[step][1] = y;
//
//        }
//
//    }
//
//    }
    *num = step;
}

void my_process_image(void) 
{
    // 原图找左右边线
    int x1 = img_raw.width / 2 - 30, y1 = BottomRow - 5;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    // for (; x1 > 0; x1--) if (mt9v03x_image[x1][y1] < 70) break;
    // if (mt9v03x_image[x1][y1] >= 70)
	for (; x1 > 0; x1--)
		if (bin_image[y1][x1] == 0 && bin_image[y1][x1 + 1] == 255)
        	findline_lefthand_adaptive(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, block_size, 0, x1, y1, ipts0, &ipts0_num);
		//;
    else ipts0_num = 0;
    int x2 = img_raw.width / 2 + 30, y2 = BottomRow - 5;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    // for (; x2 < img_raw.width - 1; x2++) if (mt9v03x_image[x2][y2] < 70) break;
    // if (mt9v03x_image[x2][y2] >= 70)
	for (; x2 < img_raw.width; x2++)
		if (bin_image[y2][x2] == 0 && bin_image[y2][x2 - 1] == 255)
		//;
        	findline_righthand_adaptive(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, block_size, 0, x2, y2, ipts1, &ipts1_num);
    else ipts1_num = 0;
}

// void new(void)
// {
// 	for (int i = 119; i > 0; i--)
// 	{
// 		ipts0[i][0] = 5;
// 		ipts0[i][1] = i;
// 		ipts1[i][0] = 187;
// 		ipts1[i][1] = i;
// 	}
// }