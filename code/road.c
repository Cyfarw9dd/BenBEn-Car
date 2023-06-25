#include "zf_common_headfile.h"
// 八邻域左右边线 points_l, points_r
// 0代表左，1为右

// 边线等距采样
unsigned short points_ls[MT9V03X_H * 3][2];
unsigned short points_rs[MT9V03X_H * 3][2];
int points_lnum, points_rnum;
// 左右边线局部角度变化率
float rpts0a[MT9V03X_H * 3];
float rpts1a[MT9V03X_H * 3];
int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[MT9V03X_H * 3];
float rpts1an[MT9V03X_H * 3];
int rpts0an_num, rpts1an_num;

// Y角点
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// 长直道
bool is_straight0, is_straight1;

// 弯道
bool is_bend0, is_bend1;

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

// num1 = points_num;
// num2 = points_num;
// 八邻域边界等距采样, 每隔三个点取一个点
void sample_border(float *(points)[2], int num1, float *(points_s)[2], int *num2, int dist_point)
{
    int len = 0;
    for (int i = 0; i < num1 && len < *num2; i++)
    {
        len++;
        if (i % dist_point == 0)
        {
            points_s[len][0] = points[i][0];
            points_s[len][1] = points[i][1];
        }
    }
}

// 边线等距采样
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);

    }
    *num2 = len;    
}
// 角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}
// 非极大值抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

// 寻找角点
void find_corners(void)
{
    // 识别Y,L拐点
    Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;      // 初始化角点标志位
    is_straight0 = data_stastics_l > 1. / 0.1;   // 左右边界直线判断？                  
    is_straight1 = data_stastics_r > 1. / 0.1;
    for (int i = 0; i < data_stastics_l; i++) {
        if (rpts0an[i] == 0) continue;
        int im1 = clip(i - (int) round(10), 0, data_stastics_l - 1);   // round(angle_dist / sample_dist)
        int ip1 = clip(i + (int) round(10), 0, data_stastics_l - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

        //Y角点阈值
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / 0.1) {     // sample_dist
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        //L角点阈值
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / 0.1) {    // sample_dist
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //长直道阈值
        if (conf > 5. / 180. * PI && i < 1.0 / 0.1) is_straight0 = false;
        if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false) break;
    }
    for (int i = 0; i < data_stastics_r; i++) {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - (int) round(10), 0, data_stastics_r - 1);
        int ip1 = clip(i + (int) round(10), 0, data_stastics_r - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / 0.1) {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / 0.1) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        if (conf > 5. / 180. * PI && i < 1.0 / 0.1) is_straight1 = false;

        if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false) break;
    }
    // // Y点二次检查,依据两角点距离及角点后张开特性
    // if (Ypt0_found && Ypt1_found) {
    //     float dx = points_l[Ypt0_rpts0s_id][0] - points_r[Ypt1_rpts1s_id][0];
    //     float dy = points_l[Ypt0_rpts0s_id][1] - points_r[Ypt1_rpts1s_id][1];
    //     float dn = sqrtf(dx * dx + dy * dy);
    //     if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
    //         float dwx = points_l[clip(Ypt0_rpts0s_id + 50, 0, data_stastics_l - 1)][0] -
    //                     points_r[clip(Ypt1_rpts1s_id + 50, 0, data_stastics_r - 1)][0];
    //         float dwy = points_l[clip(Ypt0_rpts0s_id + 50, 0, data_stastics_l - 1)][1] -
    //                     points_r[clip(Ypt1_rpts1s_id + 50, 0, data_stastics_r - 1)][1];
    //         float dwn = sqrtf(dwx * dwx + dwy * dwy);
    //         if (!(dwn > 0.7 * pixel_per_meter &&
    //               points_l[clip(Ypt0_rpts0s_id + 50, 0, data_stastics_l - 1)][0] < points_l[Ypt0_rpts0s_id][0] &&
    //               points_r[clip(Ypt1_rpts1s_id + 50, 0, data_stastics_r - 1)][0] > points_r[Ypt1_rpts1s_id][0])) {
    //             Ypt0_found = Ypt1_found = false;
    //         }
    //     } else {
    //         Ypt0_found = Ypt1_found = false;
    //     }
    // }
    // // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
    // if (Lpt0_found && Lpt1_found) {
    //     float dx = points_l[Lpt0_rpts0s_id][0] - points_r[Lpt1_rpts1s_id][0];
    //     float dy = points_l[Lpt0_rpts0s_id][1] - points_r[Lpt1_rpts1s_id][1];
    //     float dn = sqrtf(dx * dx + dy * dy);
    //     if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
    //         float dwx = points_l[clip(Lpt0_rpts0s_id + 50, 0, data_stastics_l - 1)][0] -
    //                     points_r[clip(Lpt1_rpts1s_id + 50, 0, data_stastics_r - 1)][0];
    //         float dwy = points_l[clip(Lpt0_rpts0s_id + 50, 0, data_stastics_l - 1)][1] -
    //                     points_r[clip(Lpt1_rpts1s_id + 50, 0, data_stastics_r - 1)][1];
    //         float dwn = sqrtf(dwx * dwx + dwy * dwy);
    //         if (!(dwn > 0.7 * pixel_per_meter &&
    //                 points_l[clip(Lpt0_rpts0s_id + 50, 0, data_stastics_l - 1)][0] < points_l[Lpt0_rpts0s_id][0] &&
    //                 points_r[clip(Lpt1_rpts1s_id + 50, 0, data_stastics_r - 1)][0] > points_r[Lpt1_rpts1s_id][0])) {
    //             Lpt0_found = Lpt1_found = false;
    //         }
    //     } else {
    //         Lpt0_found = Lpt1_found = false;
    //     }
    // }

}

