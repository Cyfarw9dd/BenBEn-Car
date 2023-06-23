#include "zf_common_headfile.h"

// Y角点
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// 长直道
bool is_straight0, is_straight1;

unsigned char bend_flag;
// 左右线丢线数目
unsigned char lnum = 0;
unsigned char rnum = 0;
unsigned char *lnump = &lnum;
unsigned char *rnump = &rnum;
unsigned char lflinef[120];
unsigned char rtlinef[120];

Trait_smachine Crossing;
Trait_smachine RoundAbout;
Trait_smachine Barrier;
Trait_smachine BreakRoad;
Trait_smachine Slope;
Trait_smachine Startline;
Trackline checkline_l;
Trackline checkline_r;

void Traits_process(void)
{
    Departure();
    Lostline_count(lflinef, rtlinef, lnump, rnump);
    // Track_line_l(&checkline_l);
    // Track_line_r(&checkline_r);
    BreakRoad_process(&BreakRoad);
    Startline_process(&Startline, &bin_image[0]);
    // if (BreakRoad.pointflag == 1)
    // {
    //     pit_disable(CCU60_CH0);     // 关闭摄像头中断
    //     pit_enable(CCU60_CH1);      // 打开电磁中断
    // }
    // if (BreakRoad.pointflag == 0)
    // {
    //     pit_disable(CCU60_CH1);
    //     pit_enable(CCU60_CH0);
    // }
    // if (Startline.pointflag == 1)
    // {
    //     // Buzzer();
    //     // 当检测到斑马线的时候，关中断停车
    //     while (1)
    //     {
    //         pit_disable(CCU60_CH1);
    //         pit_disable(CCU60_CH0);
    //     }
    // }
}

void Lostline_count(unsigned char LlineF[120], unsigned char RlineF[120], unsigned char *lcnt, unsigned char *rcnt)
{
    lnum = 0;
    rnum = 0;
    for (int i = BottomRow; i > BottomRow - 80; i--)
    {
        if (l_border[i] == 2)
        {
            *lcnt++;
            lnum = *lcnt;
            LlineF[i] = 0;   
        }
        else LlineF[i] = 1;
        if (r_border[i] == 185)
        {
            *rcnt++;
            rnum = *rcnt;
            RlineF[i] = 0;
        }
        else RlineF[i] = 1;
    }
}
/*遍历左右线，将找到所有满足的点都在屏幕上高亮出来*/
// 遍历左线数组，找到断开行
void Track_line_l(Trackline *checkline)
{
    // 初始化
    for (int i = 0; i < sizeof(checkline->row) / sizeof(unsigned char); i++)
    {
        checkline->row[i] = 0;
        checkline->col[i] = 0;
    }
    checkline->startrow = BottomRow ;
    checkline->endrow = BottomRow - 80;
    checkline->find = 0;
    // 判断弯道
    if (lnum < 50 && rnum > 70)   bend_flag = 1;
    if (rnum < 50 && lnum > 70)   bend_flag = 1;
    // 如果不是弯道则开始查找
    if (1)
    {
        for (int i = checkline->startrow; i > checkline->endrow; i--)
        {   
            // 找下拐点
            if (lflinef[i] == 1 && lflinef[i + 1] == 1 && lflinef[i + 2] == 1 && lflinef[i + 6] == 0 && lflinef[i + 7] == 0 & lflinef[i + 8] == 0)
            {   
                // 记录坐标
                checkline->row[0] = i + 2;
                checkline->col[0] = l_border[i + 2];
            }
            // 找上拐点
            if (lflinef[i] == 0 && lflinef[i + 1] == 0 && lflinef[i + 2] == 0 && lflinef[i + 3] == 1 && lflinef[i + 4] == 1 && lflinef[i + 5] == 1)
            {
                // 记录坐标
                checkline->row[1] = i + 2;
                checkline->col[1] = l_border[i + 2];
            }
        }
    }
}

// 遍历右线数组，查找断开行
void Track_line_r(Trackline *checkline)
{
    // 初始化
    for (int i = 0; i < sizeof(checkline->row) / sizeof(unsigned char); i++)
    {
        checkline->row[i] = 0;
        checkline->col[i] = 0;
    }
    checkline->startrow = BottomRow;
    checkline->endrow = BottomRow - 80;
    checkline->find = 0;
    // 判断弯道
    if (lnum < 50 && rnum > 70)   bend_flag = 1;
    if (rnum < 50 && lnum > 70)   bend_flag = 1;
    // 如果不是弯道则开始查找
    if (1)
    {
        for (int i = checkline->startrow; i > checkline->endrow; i--)
        {   
            // 找下拐点
            if (rtlinef[i] == 1 && rtlinef[i + 1] == 1 && rtlinef[i + 2] == 1 && rtlinef[i + 6] == 0 && rtlinef[i + 7] == 0 & rtlinef[i + 8] == 0)
            {   
                // 记录坐标
                checkline->row[0] = i + 2;
                checkline->col[0] = r_border[i + 2];
            }
            // 找上拐点
            if (rtlinef[i] == 0 && rtlinef[i + 1] == 0 && rtlinef[i + 2] == 0 && rtlinef[i + 3] == 1 && rtlinef[i + 4] == 1 && rtlinef[i + 5] == 1)
            {
                // 记录坐标
                checkline->row[1] = i + 2;
                checkline->col[1] = r_border[i + 2];
            }
        }
    }
}

void BreakRoad_process(Trait_smachine *road_smachine)
{
    blackpoints = 0;
    for (int row = 119; row > 117; row --)
    {
        for (int left_col = 93; left_col > 0; left_col -= 3)
        {
            if (mt9v03x_image[row][left_col + 3] < image_thereshold)
            {
                blackpoints++;
            }
        }
        for (int right_col = 0; right_col < 187; right_col += 3)
        {
            if (mt9v03x_image[row][right_col - 3] < image_thereshold)
            {
                blackpoints++;
            }
        }
    }
    if (blackpoints > 70)   road_smachine->pointflag = 1;
    else                    road_smachine->pointflag = 0;
}

void Startline_process(Trait_smachine *road_smachine, unsigned char (*binary_array)[188])
{
    unsigned char times = 0;
    for (unsigned char i = BottomRow - 18; i >= BottomRow - 23; i--)
    {
        unsigned char black_blocks = 0;
        unsigned char cursor = 0;
        for(unsigned char j = 0; j <= 186; j++)
        {
            if(binary_array[i][j] == 0)
            {
                if(cursor >= 20)
                {

                }
                else
                {
                    cursor++;
                }
            }
            else
            {
                if(cursor >= 4 && cursor <= 8)
                {
                    black_blocks++;
                    cursor = 0;
                }
                else
                {
                    cursor = 0;
                }
            }
        }
        if(black_blocks >= 6 && black_blocks <= 12) times++;
    }
    if(times >= 3 /*&& times <= 5*/)
    {
        road_smachine->pointflag = 1;
    }
    else
    {
        road_smachine->pointflag = 0;
    }
}

// 点集三角滤波
void blur_points(float pts_in[][2], int num, int kernel)
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        // pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_in[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_in[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_in[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_in[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}

int clip(int x, int low, int up) 
{
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) 
{
    return x > up ? up : x < low ? low : x;
}


void Find_corners(void)
{

}