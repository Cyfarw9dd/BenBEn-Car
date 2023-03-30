/*
    处理总钻风图像时需要注意一点，总钻风原始图像大小是120 * 188的。如果用XY平面坐标系表示原始图像的话，总钻风图像的坐标原点
    位于左上方，其中，x轴正方向为竖直向下，y轴正方向为水平向右。
    换言之，X坐标表示的是图像的行数，Y坐标表示图像中的列数。

    mt9v03x_image[120][188]为总钻风摄像头采集到的原始灰度图像

    在灰度图像中，每个像素点是有其各自的灰度值的，这些灰度值可能为30，可能为90，数字越大，颜色越浅，数字越小，颜色越深。且灰
    度值的范围为0~255。所以，灰度值为0会被表达为黑色，为255的话即为白色。
    0~255恰好为无符号查尔变量(unsigned char)所表达的值的范围，在ADS里写作unsigned char，所以有关于图像处理，图像数组的变量都会将其声
    明为unsigned char类型。


    //-----------------------------------------------------------------//
    这里给出一个数据类型的参考表
    unsigned char           unsigned char
    int16                   short
    int                   int
    左边是ADS里的惯用的写法，但是并不符合C标准，右边是C语言中的写法，但在ADS里编程时，更多的还是用左边的这种形式
*/
#include "zf_common_headfile.h"
#include "image.h"
#include "gyro.h"
#include "elements.h"
#include "cycle.h"
#include "pid.h"
#include "control.h"

unsigned char centerline[120];                  //定义中线数组，中线数组的横坐标等于（左线横坐标+右线横坐标）/ 2
unsigned char leftline[120];                    //定义左边线数组
unsigned char rightline[120];                   //定义右边线数组
unsigned char image_deal[MT9V03X_H][MT9V03X_W]; //声明一个二维数组，用于存放二值化后的图像，其中，下标MT9V03X_H，MT9V03X_W表示总钻风图像的高和宽
unsigned char image_deal2[MT9V03X_H][MT9V03X_W];
unsigned char Left_RoadWidth[120];              //定义左半边赛道宽度，即中线到左边线的距离
unsigned char Right_RoadWidth[120];             //定义右半边赛道宽度
short image_threshold = 0;                      //定义图像处理阈值

/*这两个变量用于计算中线的偏差程度，并将计算出的结果交给转向环计算*/

int Prospect_Err = 0;                   //定义前瞻偏差，前瞻偏差的取值为实际中线上三个等距的点分别对理想中线做的差
int Bottom_Err = 0;                     //定义车身横向偏差，即摄像头拍到图像的最底端一行所处的中线值对理想中线做的差
int further, middle, near;              //图像中的远点，中点和近点

/*
    摄像头运行的主体函数，大津法，扫线等都整合在里面运行
*/
void Camera(void){
    if(mt9v03x_finish_flag){                              //mt9v03x_finish_flag为图像处理结束的标志位，在逐飞库中有着详细定义
        image_threshold = GetOSTU(mt9v03x_image[0]);      //通过大津法来得到原始灰度图像的阈值
        lcd_binaryzation032_zoom(mt9v03x_image[0], image_deal[0], MT9V03X_W , MT9V03X_H, image_threshold); //将二值化后的图像存放到image_deal[120][188]里
        // Get_IcmData();                                    //获取陀螺仪数据

        Searching_for_boundaries(&image_deal[0]);         //寻找赛道边界 
        Deal_Road_Characteristics(&image_deal[0]);        //处理赛道特征，如计算左右半边赛道宽度等       
        Turn_cycle_ver2(1500);                            
        // Pokemon_Go();                                     //元素判断
        Hightlight_Lines(&image_deal[0]);                 //高亮左右边界以及中线
        tft180_show_gray_image(0, 0, &image_deal[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);

        mt9v03x_finish_flag = 0;                          //标志位归0，一定要归0！不归0的话图像只处理起始帧
    }
}


/*
    函数名称：大津法获取阈值函数
    函数功能：通过计算原始灰度图像中的灰度直方图来自动计算出当前灰度图像中的阈值
    大津法的具体实现过程可以不用特别了解
*/
unsigned char otsuThreshold(unsigned char *image, unsigned short col, unsigned short row)
{
    #define GrayScale 256
    unsigned short width = col;
    unsigned short height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    unsigned char threshold = 0;
    unsigned char* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
        }
    }

    //计算每个像素在整幅图像中的比例
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (unsigned char)i;
        }
    }

    return threshold;
}

/*
    图像扫描，即扫线函数，接受一个参数，这个参数即为二值化后的图像的指针
    通过从图像中间像左右两边依次遍历寻找到黑色边界
    并将这些黑色边界的横坐标分别存放到边界数组中
    中线坐标即用边线的计算出来
*/
void Searching_for_boundaries(unsigned char (*binary_array)[188]){
    unsigned char Row = 0;
    unsigned char Left_Col = 0;
    unsigned char Right_Col = 0;
    for(Row=119;Row>0;Row--){
        for(Left_Col=93;Left_Col<188;Left_Col++){
            if(Left_Col == EndCoL){
                rightline[Row] = EndCoL;
                break;
            }
            if((binary_array[Row][Left_Col-2]==255)&&(binary_array[Row][Left_Col-1]==0)&&(binary_array[Row][Left_Col]==0)){
                rightline[Row]=Left_Col;
                break;
            }
        }

        for(Right_Col=93;Right_Col>1;Right_Col--){
            if(Right_Col == StartCoL){
                leftline[Row] = StartCoL;
                break;
            }
            if((binary_array[Row][Right_Col]==0)&&(binary_array[Row][Right_Col+1]==0)&&(binary_array[Row][Right_Col+2]==255)){
                leftline[Row]=Right_Col;
                break;
            }
        }
    }
}


void Deal_Road_Characteristics(unsigned char (*binary_array)[188]){
    for(unsigned char i = BottomRow; i > 0; i--){
        centerline[i] = (rightline[i] + leftline[i]) / 2;
        Left_RoadWidth[i] = absolute(93 - leftline[i]);
        Right_RoadWidth[i] = absolute(rightline[i] - 93);
    }
}

void Hightlight_Lines(unsigned char (*binary_array)[188]){
    for(unsigned char i = BottomRow; i > 0; i--){
        binary_array[i][centerline[i]] = 120;
        binary_array[i][rightline[i]] = 120;
        binary_array[i][leftline[i]] = 120;
    }
}

/*
    二值化函数
    *p          原始图像数组指针
    *q          空图像数组指针
    width       原始图像的宽
    height      原始图像的高
    threshold   阈值，这里为大津法计算出的图像阈值
*/

void lcd_binaryzation032_zoom(unsigned char *p, unsigned char *q, unsigned short width, unsigned short height, unsigned char threshold)//传入一个处理数组的指针，然后进行修改数组内容
{
    unsigned i,j;
    unsigned short temp = 0;

    for(j=0;j<height;j++)
    {
        for(i=0;i<width;i++)
        {
            temp = *p;
            if(temp>threshold)
                *q = 255;
            else
                *q  = 0;
            ++q;
            ++p;
        }
    }
}

/*
    四邻域滤波
*/
void Four_neighbourhood_Filter(unsigned char (*binary_array)[188]){
    short row; //行
    short col; //列

    for (col = 1; col < MT9V03X_H - 1; col++)
    {
        for (row = 1; row < MT9V03X_W - 1; row++)
        {
            if ((binary_array[col][row] == 0)
                    && (binary_array[col - 1][row] + binary_array[col + 1][row] + binary_array[col][row + 1] + binary_array[col][row - 1] > 550))
            {
                binary_array[col][row] = 255;
            }
            else if ((binary_array[col][row] == 1)
                    && (binary_array[col - 1][row] + binary_array[col + 1][row] + binary_array[col][row + 1] + binary_array[col][row - 1] < 550))
            {
                binary_array[col][row] = 0;
            }
        }
    }
}

float one_curvature(int x1, int y1) // one_curvature(centerline[30], 30)
{
    float K;
    int l = x1 - 63;
    int h = y1 - 59;
    K = (float)2 * l / (l * l + h * h);
    return K;
}

/*
    函数名称：计算中线偏差
    返回值：无
    参数：无
    调用过程：
    自定义前瞻行数，默认是15，前瞻行数值越大，代表着参与偏差计算的行数就越多，前瞻行数值越小，参与偏差计算的行数就越小，即反应
    就更加灵敏。自定义远点，中点，近点，实际的中线偏差由这三个点结合实际情况横向求差而得。并且，对这三个点各自取了临近的两个点
    相加并求均值，尽可能地过滤掉异常数据。
*/

void cal_curvature(void){
    int prospect = 5;            // 摄像头高度为20cm，自定义前瞻行数15 || 当前摄像头高度为10cm，将前瞻行数更改为5

    near = (centerline[119] + centerline[119 - 1] + centerline[119 - 2]) / 3;
    middle = (centerline[119 - prospect] + centerline[119 - prospect - 1] + centerline[119 - prospect - 2]) / 3;
    further = (centerline[119 - prospect * 2] + centerline[119 - prospect * 2 - 1] + centerline[119 - prospect * 2 - 2]) / 3;

    if(further < middle && middle < near){
        Prospect_Err = ((middle - further)  + (near - middle)) / 2;
    }
    else if(further < middle && middle >= near){
        Prospect_Err = near - middle;
    }
    else if(further >= middle && middle < near){
        Prospect_Err = near - middle;
    }
    else{
        Prospect_Err = ((middle - further) + (near - middle)) / 2;
    }
    Bottom_Err = near - 94;
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

/*
    船新搬运过来的大津法
    速度应该比逐飞写的要快一些，目前用的是这个
*/
short GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < MT9V03X_H; j++)
    {
        for (i = 0; i < MT9V03X_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}

void regression(int type, int startline, int endline){
    int i = 0;
    int sumlines = endline - startline;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    float parameterA, parameterB;
    if (type == 0)      //拟合中线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += centerline[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x的平均值
            averageY = sumY / sumlines;     //y的平均值
        }
        else
        {
            averageX = 0;     //x的平均值
            averageY = 0;     //y的平均值
        }
        for (i = startline; i < endline; i++)
        {
            sumUp += (centerline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 1)//拟合左线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += leftline[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            //SetText("lefetline"+i+" " +lefetline[i] + " averageY" +" "+ averageY);
            sumUp += (leftline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)//拟合右线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += rightline[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            sumUp += (rightline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
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
        for(unsigned char j = 0; j < 186; i++){
            if(binary_array[i][j] == 0){
                blackpoint++;
            }
        }
    }
    return blackpoint;
}


/************************************************************
【函数名称】Eight_neighborhood
【功    能】八邻域算法求边界
【参    数】寻种子方式
【返 回 值】无
【实    例】Eight_neighborhood(0);
【注意事项】
    1*该算法需要你传入的二值化数组，是经过二值化之后的数组，白色为1  黑色为0
    2*该算法输出的图像需要你自己定义一个与二值化数组尺寸相同的二维数组
    3*下面的宏定义必须换成自己实际的图像数组尺寸和二维数组名
    4*记得声明该函数
************************************************************/

//这里换上你二值化之后的图像数组
//#define User_Image    image
//这里是八邻域输出数组需要自己定义一个与原图像相同尺寸的二维数组
//#define Edge_arr  image
#define User_Image image_deal
#define Edge_arr image_deal2

void Eight_neighborhood(unsigned char flag){
    unsigned char i,j;
    //核子中心坐标  起始方向初始为6
    unsigned char core_x,core_y,start;
    //方向
    char direction;
    unsigned length = 0;
    //清空边界数据
    for(i=0;i<USER_SIZE_H;i++)
        for(j=0;j<USER_SIZE_W;j++)
            Edge_arr[i][j]=0;
    if(flag==0)//从里向外找种子
    {
        start = 6;
        //如果中间为白
        if(User_Image[USER_SIZE_H-1][USER_SIZE_W/2]==255)
        {
            for(i=USER_SIZE_W/2;i>=1;i--)
            {
                if(User_Image[USER_SIZE_H-1][i-1]==0||i==0)
                {//将左下第一个边界点作为种子
                    core_x = i;
                    core_y = USER_SIZE_H-1;
                    break;
                }
            }
        }//如果中间为黑则去两边找
        else if(User_Image[USER_SIZE_H-1][USER_SIZE_W/2]==0)
        {
            if(User_Image[USER_SIZE_H-1][5]==255)
                for(i=5;i>=1;i--)
                {
                    if(User_Image[USER_SIZE_H-1][i-1]==0||i==0)
                    {//将左下第一个边界点作为种子
                        core_x = i;
                        core_y = USER_SIZE_H-1;
                        break;
                    }
                }
            else if(User_Image[USER_SIZE_H-1][USER_SIZE_W-5]==255)
                for(i=USER_SIZE_W-5;i>=1;i--)
                {
                    if(User_Image[USER_SIZE_H-1][i-1]==0||i==0)
                    {//将左下第一个边界点作为种子
                        core_x = i;
                        core_y = USER_SIZE_H-1;
                        break;
                    }
                }
            else//否则将视为无效图像不做处理
                return;
        }
    }
    else if(flag==1)
    {
        start = 6;
        for(i=0;i<USER_SIZE_W;i++)
        {
            if(User_Image[USER_SIZE_H-1][i]==255||i==USER_SIZE_W-1)
            {//将左下第一个边界点作为种子
                core_x = i;
                core_y = USER_SIZE_H-1;
                break;
            }
        }
    }
    else if(flag==2)
    {
        start = 2;
        //如果中间为白
        if(User_Image[USER_SIZE_H-1][USER_SIZE_W/2]==255)
        {
            for(i=USER_SIZE_W/2;i<USER_SIZE_W;i++)
            {
                if(User_Image[USER_SIZE_H-1][i+1]==0||i==USER_SIZE_W-1)
                {
                    //将右下第一个边界点作为种子
                    core_x = i;
                    core_y = USER_SIZE_H-1;
                    break;
                }
            }
        }//如果中间为黑则去两边找
        else if(User_Image[USER_SIZE_H-1][USER_SIZE_W/2]==0)
        {
            if(User_Image[USER_SIZE_H-1][5]==255)
                for(i=5;i<USER_SIZE_W;i++)
                {
                    if(User_Image[USER_SIZE_H-1][i+1]==0||i==USER_SIZE_W-1)
                    {//将右下第一个边界点作为种子
                        core_x = i;
                        core_y = USER_SIZE_H-1;
                        break;
                    }
                }
            else if(User_Image[USER_SIZE_H-1][USER_SIZE_W-5]==255)
                for(i=USER_SIZE_W-5;i<USER_SIZE_W;i++)
                {
                    if(User_Image[USER_SIZE_H-1][i+1]==0||i==USER_SIZE_W-1)
                    {//将右下第一个边界点作为种子
                        core_x = i;
                        core_y = USER_SIZE_H-1;
                        break;
                    }
                }
            else//否则将视为无效图像不做处理
                return;
        }
    }
    else if(flag==3)
    {
        start = 2;
        for(i=USER_SIZE_W-1;i>=0;i--)
        {
            if(User_Image[USER_SIZE_H-1][i]==255||i==0)
            {//将右下第一个边界点作为种子
                core_x = i;
                core_y = USER_SIZE_H-1;
                break;
            }
        }
    }
    //寻找边缘
  while(1)
    {
        direction = start;
        Edge_arr[core_y][core_x]=255;
        if(flag == 0||flag == 1)
        {
            for(i=0;i<8;i++)
            {
                if(direction == 0) {if(core_y!=0)                                                                               {if(User_Image[core_y-1][core_x]==255)      {core_y--;                      start=6;    break;}}}
                if(direction == 1) {if(core_y!=0&&core_x!=USER_SIZE_W-1)                                {if(User_Image[core_y-1][core_x+1]==255)    {core_y--; core_x++;    start=6;    break;}}}
                if(direction == 2) {if(core_x!=USER_SIZE_W-1)                                                       {if(User_Image[core_y][core_x+1]==255)      {core_x++;                      start=0;    break;}}}
                if(direction == 3) {if(core_y!=USER_SIZE_H-1&&core_x!=USER_SIZE_W-1)        {if(User_Image[core_y+1][core_x+1]==255)    {core_y++; core_x++;    start=0;    break;}}}
                if(direction == 4) {if(core_y!=USER_SIZE_H-1)                                                       {if(User_Image[core_y+1][core_x]==255)      {core_y++;                      start=2;    break;}}}
                if(direction == 5) {if(core_y!=USER_SIZE_H-1&&core_x!=0)                                {if(User_Image[core_y+1][core_x-1]==255)    {core_y++; core_x--;    start=2;    break;}}}
                if(direction == 6) {if(core_x!=0)                                                                               {if(User_Image[core_y][core_x-1]==255)      {core_x--;                      start=4;    break;}}}
                if(direction == 7) {if(core_y!=0&&core_x!=0)                                                        {if(User_Image[core_y-1][core_x-1]==255)    {core_y--; core_x--;    start=4;    break;}}}
                direction++;    if(direction>7) direction=0;
            }
        }
        else if(flag == 2||flag == 3)
        {
            for(i=0;i<8;i++)
            {
                if(direction == 0) {if(core_y!=0)                                                                               {if(User_Image[core_y-1][core_x]==255)      {core_y--;                      start=2;    break;}}}
                if(direction == 1) {if(core_y!=0&&core_x!=USER_SIZE_W-1)                                {if(User_Image[core_y-1][core_x+1]==255)    {core_y--; core_x++;    start=4;    break;}}}
                if(direction == 2) {if(core_x!=USER_SIZE_W-1)                                                       {if(User_Image[core_y][core_x+1]==1)        {core_x++;                      start=4;    break;}}}
                if(direction == 3) {if(core_y!=USER_SIZE_H-1&&core_x!=USER_SIZE_W-1)        {if(User_Image[core_y+1][core_x+1]==255)    {core_y++; core_x++;    start=6;    break;}}}
                if(direction == 4) {if(core_y!=USER_SIZE_H-1)                                                       {if(User_Image[core_y+1][core_x]==255)      {core_y++;                      start=6;    break;}}}
                if(direction == 5) {if(core_y!=USER_SIZE_H-1&&core_x!=0)                                {if(User_Image[core_y+1][core_x-1]==255)    {core_y++; core_x--;    start=0;    break;}}}
                if(direction == 6) {if(core_x!=0)                                                                               {if(User_Image[core_y][core_x-1]==255)      {core_x--;                      start=0;    break;}}}
                if(direction == 7) {if(core_y!=0&&core_x!=0)                                                        {if(User_Image[core_y-1][core_x-1]==255)    {core_y--; core_x--;    start=2;    break;}}}
                direction--;    if(direction==-1)   direction=7;
            }
        }
        if(core_y==USER_SIZE_H-1&&length>5)
        {
            Edge_arr[core_y][core_x]=255;
            break;
        }
        length++;
    }
    if(flag==0&&length<80)
    {
        Eight_neighborhood(1);
    }
    if(flag==1&&length<80)
    {
        Eight_neighborhood(2);
    }
    if(flag==2&&length<80)
    {
        Eight_neighborhood(3);
    }
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
    if(x<a) x = a;
    if(x>b) x = b;
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


/*变量声明*/
unsigned char original_image[image_h][image_w];
unsigned char image_thereshold;//图像分割阈值
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
			printf("\n如果左边比右边高了，左边等待右边\n");	
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
Get_image(mt9v03x_image[0]);
turn_to_bin();
/*提取赛道边界*/
image_filter(bin_image);//滤波
image_draw_rectan(bin_image);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
	printf("正在开始八领域\n");
	search_l_r((unsigned short)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	printf("八邻域已结束\n");
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
	//处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}


//显示图像   改成你自己的就行 等后期足够自信了，显示关掉，显示屏挺占资源的
tft180_show_gray_image(0, 0, &bin_image[0], image_w, image_h, image_w / 1.75, image_h / 1.75, 0);

	//根据最终循环次数画出边界点
	for (i = 0; i < data_stastics_l; i++)
	{
		tft180_draw_point(points_l[i][0]+2, points_l[i][1], uesr_BLUE);//显示起点
	}
	for (i = 0; i < data_stastics_r; i++)
	{
		tft180_draw_point(points_r[i][0]-2, points_r[i][1], uesr_RED);//显示起点
	}

	for (i = hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
		//求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
		//当然也有多组边线的找法，但是个人感觉很繁琐，不建议
		tft180_draw_point(center_line[i], i, uesr_GREEN);//显示起点 显示中线	
		tft180_draw_point(l_border[i], i, uesr_GREEN);//显示起点 显示左边线
		tft180_draw_point(r_border[i], i, uesr_GREEN);//显示起点 显示右边线
	}


}





/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/


