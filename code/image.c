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
    int32                   int
    左边是ADS里的惯用的写法，但是并不符合C标准，右边是C语言中的写法，但在ADS里编程时，更多的还是用左边的这种形式
*/

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
        Turn_cycle_ver2(1800);                            //转向环，传入1800的pwm
        // Pokemon_Go();                                     //元素判断
        Hightlight_Lines(&image_deal[0]);                 //高亮左右边界以及中线
        tft180_show_gray_image(0, 0, &image_deal[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 1.5, MT9V03X_H / 1.5, 0);

        // 问题大概率出在了屏幕显示二值化图像的这个地方，也有可能是合宙的tft屏的原因

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
    int prospect = 15;            //自定义前瞻行数 15

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


