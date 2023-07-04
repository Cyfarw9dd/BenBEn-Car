#include "zf_common_headfile.h"

#define TURN0       3000
#define TURN1       3000   
#define DISTANCE0   11000
#define DISTANCE1   8000
int barrier_turning_distance = 0;
int left_distance = 0; 
int right_distance = 0;
int turn_flag = 0;
int enter_flag = 0;
int turn_err = 0;

void Barrier_process(Trait_smachine *road_smh)
{
    // 获取tof测距信息
    dl1a_get_distance();

    if (dl1a_distance_mm > 1200 && road_smh)
    {
        track_mode = NORMAL;
        road_smh->pointflag= 0;
    }
    // 识别到障碍了
    if (dl1a_distance_mm < 800 && road_smh->status == BARRIER_NONE && road_smh->pointflag == 0)
    {
        // 这里可能会导致越界
        for (int i = BottomRow; i > TopRow; i--)
        {
            if (clip_bin_image[clip(i + 5, TopRow, BottomRow)][93] == white_pixel && clip_bin_image[clip(i + 4, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i + 3, TopRow, BottomRow)][93] == black_pixel
            &&  clip_bin_image[clip(i + 2, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i + 1, TopRow, BottomRow)][93] == black_pixel && clip_bin_image[clip(i, TopRow, BottomRow)][93] == black_pixel)
            {
                road_smh->status = BARRIER_IN;
                road_smh->pointflag = 1;
            }
        }
    }
    if (road_smh->status == BARRIER_IN && road_smh->pointflag == 1)
    {
        road_smh->status = BARRIER_IN;
        road_smh->pointflag = 2;
        track_mode == TURN;
    }
    if (road_smh->status == BARRIER_IN && road_smh->pointflag == 2)
    {
        road_smh->status = BARRIER_TURN;
        run_off(&Barrier);
    }
    // if (road_smh->status == BARRIER_IN && road_smh->pointflag == 2)
    // {
    //     track_mode = NORMAL;
    //     road_smh->status = BARRIER_NONE;
    //     road_smh->pointflag = 0;
    // }
}


// -> 原地转角
// -> 直行
// -> 原地转角
// -> 直行（屏蔽断路）
void run_off(Trait_smachine *road_smh)
{
    if (road_smh->status == BARRIER_TURN)
    {
        track_mode = TURN;
        if (turn_flag == 0)     right_distance = 0;
        if (turn_flag == 0 && right_distance < TURN0)
        {
            turn_err = 100;
            turn_flag == 1;
            if (right_distance > TURN0)
            {
                turn_flag = 2;
            }
        }
        if (turn_flag == 2)     barrier_turning_distance = 0;
        if (turn_flag == 2 && barrier_turning_distance < DISTANCE0)
        {
            turn_err = 0;
            turn_flag = 3;
            if (barrier_turning_distance > DISTANCE0)
            {
                turn_flag = 4;
            }
        }
        if (turn_flag == 4)     left_distance = 0;
        if (turn_flag == 4 && left_distance < TURN1)
        {
            turn_err = -100;
            turn_flag = 5;
            if (left_distance > TURN1)
            {
                turn_flag = 6;
            }
        }
        if (turn_flag == 6)     barrier_turning_distance = 0;
        if (turn_flag == 6 && barrier_turning_distance < DISTANCE1)
        {
            turn_err = 0;
            turn_flag = 7;
            if (barrier_turning_distance > DISTANCE1)
            {
                turn_flag =  8;
            }
        }
        if (turn_flag == 8)     turn_flag = 0;

    }
}