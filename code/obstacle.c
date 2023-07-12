#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"
#define DISTANCE0   8000
#define DISTANCE1   4800
#define ANGLE0      -35
#define ANGLE1      60
int barrier_turning_distance = 0;
int left_distance = 0; 
int right_distance = 0;
int turn_flag = 0;
int enter_flag = 0;
int turn_err = 0;

#pragma section all restore

#pragma section all "cpu1_psram"
void Barrier_process(Trait_smachine *road_smh)
{
    if (road_smh->trait_cnt < 1)
        {
        // 获取tof测距信息
        dl1a_get_distance();
        // // 没遇到障碍，且障碍转向时
        // if (dl1a_distance_mm > 1200 && road_smh->status != BARRIER_TURN)
        // {
        //     road_smh->status = BARRIER_NONE;
        //     track_mode = NORMAL;
        //     road_smh->pointflag= 0;
        // }
        // 识别到障碍了
        if (dl1a_distance_mm < 1100 && road_smh->status == BARRIER_NONE && road_smh->pointflag == 0)
        {
            for (int i = BottomRow; i > TopRow; i--)
            {
                if (clip_bin_image[clip(i + 5, TopRow, BottomRow)][93] == white_pixel && clip_bin_image[clip(i + 4, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i + 3, TopRow, BottomRow)][93] == black_pixel
                &&  clip_bin_image[clip(i + 2, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i + 1, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i, TopRow, BottomRow)][93] == black_pixel)
                {
                    road_smh->status = BARRIER_FOUND;
                    track_mode = SLOW_DOWN;
                    road_smh->pointflag = 1;
                }
            }
        }
        if (road_smh->status == BARRIER_FOUND && road_smh->pointflag == 1 && dl1a_distance_mm < 900)
        {
            road_smh->status = BARRIER_TURN;
            road_smh->pointflag = 2;
            turn_flag = 0;
        }
        if (road_smh->status == BARRIER_TURN)
        {
            run_off(&Startline);
            // runoff执行结束后直接清零标志位和状态位
        }
        // if (road_smh->status == BARRIER_TURN && road_smh->pointflag == 2 && turn_flag == 5)
        // {
        //     track_mode = NORMAL;
        //     road_smh->pointflag = 0;
        //     road_smh->status = BARRIER_NONE;
        // }
    }
}


// 状态机反复直行runoff函数，标记标志位进入下一标志位
void run_off(Trait_smachine *road_smh)
{
    // turnflag = 0
    // 初始化角度位置，以当前姿态为0°角
    if (turn_flag == 0)
    {
        theta = 0;
        turn_flag = 1;
        // 初始化角度环模式
        track_mode = TURN;
    }
    if (turn_flag == 1)
    {
        aim_theta = ANGLE0;
        if (theta <= aim_theta + 5 && turn_flag == 1)
            turn_flag = 2;
    }
    if (turn_flag == 2)
    {
        // 清零距离及分量
        right_distance = 0;
        left_distance = 0;
        turn_flag = 3;
        track_mode = GO_STRAIGHT;
    }
    if (turn_flag == 3 && left_distance >= DISTANCE0 && right_distance >= DISTANCE0)
    {
        turn_flag = 4;
    }
    if (turn_flag == 4)
    {
        // 将当前姿态设置为0°
        track_mode = TURN;
        theta = 0;
        turn_flag = 5;
    }
    if (turn_flag == 5)
    {
        aim_theta = ANGLE1;
        if (theta >= aim_theta - 5)
            turn_flag = 6;
    }
    if (turn_flag == 6)
    {
        // 将距离积分量清理
        left_distance = 0;
        right_distance = 0;
        turn_flag = 7;
        track_mode = GO_STRAIGHT;
    }
    if (turn_flag == 7 && left_distance >= DISTANCE1 && right_distance >= DISTANCE1)
    {
        turn_flag = 8;
    }
    // 避障结束
    // 清零标志位和状态位
    if (turn_flag == 8)
    {
        track_mode = TURN;
        theta = 0;
        turn_flag = 9;
    }
    if (turn_flag == 9)
    {
        aim_theta = -20;
        if (theta < aim_theta)
            turn_flag = 10;
    }
    if (turn_flag == 10)
    {
        road_smh->pointflag = 0;
        // turn_flag = 0;
        track_mode = NORMAL;
        road_smh->status = BARRIER_NONE;
        road_smh->trait_cnt += 1;
    }

    // road_smh->pointflag = 0;
    // // turn_flag = 0;
    // track_mode = NORMAL;
    // road_smh->status = BARRIER_NONE;
}

#pragma section all restore