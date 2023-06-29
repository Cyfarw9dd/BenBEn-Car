#include "zf_common_headfile.h"
// 八邻域左右边线 points_l, points_r
// 0代表左，1为右
#pragma section all "cpu1_dsram"
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

// 左右边线跳变列
int lcptc[CLIP_IMAGE_H];
int rcptc[CLIP_IMAGE_H];

// 跳变列非极大极小值抑制
// 左线非极大值抑制，右线非极小值抑制
int lmax[CLIP_IMAGE_H];
int rmin[CLIP_IMAGE_H];

// 随便什么角点
int Xpt0, Xpt1;
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
// 循迹模式
int track_mode;

unsigned char bend_flag;
// 左右线丢线数目
unsigned char lnum = 0;
unsigned char rnum = 0;
unsigned char lflinef[CLIP_IMAGE_H];
unsigned char rtlinef[CLIP_IMAGE_H];

Trait_smachine Crossing;
Trait_smachine RoundAbout;
Trait_smachine Barrier;
Trait_smachine BreakRoad;
Trait_smachine Slope;
Trait_smachine Startline;
Trackline checkline_l;
Trackline checkline_r;

unsigned char outflag = 0;      // 出库标志位
#pragma section all restore


#pragma section all "cpu1_psram"
void Traits_process(void)
{
    aim_speed = 280;
    // 默认情况下正常循迹
    track_mode = NORMAL;
    // roll_out();  // 出库打死
    if (!Departure_PointFlag)
        Departure();
    Barrier_process(&Barrier);
    BreakRoad_process(&BreakRoad);
    Startline_process(&Startline, &bin_image[0]);
}

void find_inflectionpoint(void)
{
    // 遍历左右线
    Lostline_check(clip_lfline, clip_rtline, lcptc, rcptc);
    // 非极大值&非极小值抑制
    lmaximum(sizeof(lcptc) / sizeof(lcptc[0]), 5, lcptc);
    rminimum(sizeof(rcptc) / sizeof(rcptc[0]), 5, rcptc);
    // 最终坐标应该为[0]位元素
}

void Lostline_check(int clip_lfline[], int clip_rtline[], int lcptc[], int rcptc[])
{
    // 遍历左线
    int m, n;
    for (int i = CLIP_IMAGE_H - 1; i > TopRow; i--)
    {
        m++;
        // 扫描拐点上下五行
        if (clip_lfline[clip(i + 5, TopRow, CLIP_IMAGE_H - 1)] > 5 && clip_lfline[clip(i + 4, TopRow, CLIP_IMAGE_H - 1)] > 5
         && clip_lfline[clip(i + 3, TopRow, CLIP_IMAGE_H - 1)] > 5 && clip_lfline[clip(i + 2, TopRow, CLIP_IMAGE_H - 1)] > 5
         && clip_lfline[clip(i + 1, TopRow, CLIP_IMAGE_H - 1)] > 5 && clip_lfline[clip(i, TopRow, CLIP_IMAGE_H - 1)] > 5
         && clip_lfline[clip(i - 1, TopRow, CLIP_IMAGE_H - 1)] < 3 && clip_lfline[clip(i - 2, TopRow, CLIP_IMAGE_H - 1)] < 3
         && clip_lfline[clip(i - 3, TopRow, CLIP_IMAGE_H - 1)] < 3 && clip_lfline[clip(i - 4, TopRow, CLIP_IMAGE_H - 1)] < 3
         && clip_lfline[clip(i - 4, TopRow, CLIP_IMAGE_H - 1)] < 3 && clip_lfline[clip(i - 5, TopRow, CLIP_IMAGE_H - 1)] < 3)
        {
            lcptc[m] = i;
        }
    }
    // 遍历右线
    for (int i = CLIP_IMAGE_H - 1; i > TopRow; i--)
    {
        n++;
        // 扫描拐点上下五行
        if (clip_rtline[clip(i + 5, TopRow, CLIP_IMAGE_H - 1)] > 180 && clip_rtline[clip(i + 4, TopRow, CLIP_IMAGE_H - 1)] > 180
         && clip_rtline[clip(i + 3, TopRow, CLIP_IMAGE_H - 1)] > 180 && clip_rtline[clip(i + 2, TopRow, CLIP_IMAGE_H - 1)] > 180
         && clip_rtline[clip(i + 1, TopRow, CLIP_IMAGE_H - 1)] > 180 && clip_rtline[clip(i, TopRow, CLIP_IMAGE_H - 1)] > 180
         && clip_rtline[clip(i - 1, TopRow, CLIP_IMAGE_H - 1)] < 183 && clip_rtline[clip(i - 2, TopRow, CLIP_IMAGE_H - 1)] < 183
         && clip_rtline[clip(i - 3, TopRow, CLIP_IMAGE_H - 1)] < 183 && clip_rtline[clip(i - 4, TopRow, CLIP_IMAGE_H - 1)] < 183
         && clip_rtline[clip(i - 4, TopRow, CLIP_IMAGE_H - 1)] < 183 && clip_rtline[clip(i - 5, TopRow, CLIP_IMAGE_H - 1)] < 183)
        {
            rcptc[n] = i;
        }
    }
}

#pragma section all restore
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
                checkline->col[0] = clip_lfline[i + 2];
            }
            // 找上拐点
            if (lflinef[i] == 0 && lflinef[i + 1] == 0 && lflinef[i + 2] == 0 && lflinef[i + 3] == 1 && lflinef[i + 4] == 1 && lflinef[i + 5] == 1)
            {
                // 记录坐标
                checkline->row[1] = i + 2;
                checkline->col[1] = clip_lfline[i + 2];
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
                checkline->col[0] = clip_rtline[i + 2];
            }
            // 找上拐点
            if (rtlinef[i] == 0 && rtlinef[i + 1] == 0 && rtlinef[i + 2] == 0 && rtlinef[i + 3] == 1 && rtlinef[i + 4] == 1 && rtlinef[i + 5] == 1)
            {
                // 记录坐标
                checkline->row[1] = i + 2;
                checkline->col[1] = clip_rtline[i + 2];
            }
        }
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
    for (int i = 0; i < num1 - dist_point; i += dist_point)
    {
        if (len > *num2 - dist_point)
            return;
        else{
            points_s[len++][0] = points[i][0];
            points_s[len++][1] = points[i][1];
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

#pragma section all "cpu1_psram"
// 左线角度变化率
void left_local_angle_points(int num, int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            rpts0a[i] = 0;
            continue;
        }
        float dx1 = (float)cutpoints_l[i][0] - (float)cutpoints_l[clip(i - dist, 0, num - 1)][0];
        float dy1 = (float)cutpoints_l[i][1] - (float)cutpoints_l[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = (float)cutpoints_l[clip(i + dist, 0, num - 1)][0] - (float)cutpoints_l[i][0];
        float dy2 = (float)cutpoints_l[clip(i + dist, 0, num - 1)][1] - (float)cutpoints_l[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        rpts0a[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 右线角度变化率
void right_local_angle_points(int num, int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            rpts1a[i] = 0;
            continue;
        }
        float dx1 = (float)cutpoints_r[i][0] - (float)cutpoints_r[clip(i - dist, 0, num - 1)][0];
        float dy1 = (float)cutpoints_r[i][1] - (float)cutpoints_r[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = (float)cutpoints_r[clip(i + dist, 0, num - 1)][0] - (float)cutpoints_r[i][0];
        float dy2 = (float)cutpoints_r[clip(i + dist, 0, num - 1)][1] - (float)cutpoints_r[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        rpts1a[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 左线非极大值抑制
void lnms_angle(int num, int kernel)
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        rpts0an[i] = rpts0a[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(rpts0a[clip(i + j, 0, num - 1)]) > fabs(rpts0an[i])) {
                rpts0an[i] = 0;
                break;
            }
        }
    }
}
// 右线非极大值抑制
void rnms_angle(int num, int kernel)
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        rpts1an[i] = rpts1a[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(rpts1a[clip(i + j, 0, num - 1)]) > fabs(rpts1an[i])) {
                rpts1an[i] = 0;
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
    // is_straight0 = data_stastics_l > 1. / 0.1;   // 左右边界直线判断？                  
    // is_straight1 = data_stastics_r > 1. / 0.1;
    for (int i = 0; i < data_stastics_l; i++) {
        if (rpts0an[i] == 0) continue;
        int im1 = clip(i - (int) round(10), 0, data_stastics_l - 1);   // round(angle_dist / sample_dist)
        int ip1 = clip(i + (int) round(10), 0, data_stastics_l - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

        // 随便什么角点
        Xpt0 = i;
        //Y角点阈值
        if (Ypt0_found == false && 20. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / 0.1) {     // sample_dist
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
        Xpt1 = i;
        if (Ypt1_found == false && 20. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / 0.1) {
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


void roll_out(void)
{
        #if 0
        while (outflag)
        {
            motor_ctrl(3000, 2000);
            system_delay_ms(500);
            outflag = 0;
        }
        #endif
        #if 1
        while (outflag)
        {
            motor_ctrl(1800, 3500);
            system_delay_ms(500);
            outflag = 0;
        }
        #endif 
}

// 左线非极大值抑制
void lmaximum(int num, int kernel, int input[])
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        rpts0an[i] = rpts0a[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(rpts0a[clip(i + j, 0, num - 1)]) > fabs(rpts0an[i])) {
                rpts0an[i] = 0;
                break;
            }
        }
    }
}
// 右线非极小值抑制
void rminimum(int num, int kernel, int input[])
{
    // zf_assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        rpts0an[i] = rpts0a[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(rpts0a[clip(i + j, 0, num - 1)]) < fabs(rpts0an[i])) {
                rpts0an[i] = 0;
                break;
            }
        }
    }
}

#pragma section all restore