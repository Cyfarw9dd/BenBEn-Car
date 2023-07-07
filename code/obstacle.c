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
    // 没遇到障碍，且障碍转向时
    if (dl1a_distance_mm > 1200 && road_smh->status != TURN)
    {
        track_mode = NORMAL;
        road_smh->pointflag= 0;
    }
    // 识别到障碍了
    if (dl1a_distance_mm < 800 && road_smh->status == BARRIER_NONE && road_smh->pointflag == 0)
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
    if (road_smh->status == BARRIER_FOUND && road_smh->pointflag == 1 && dl1a_distance_mm < 700)
    {
        road_smh->status = BARRIER_IN;
        road_smh->pointflag = 2;
        track_mode == TURN;
        run_off(&Startline);
    }
    // if (road_smh->status == BARRIER_IN && road_smh->pointflag == 2)
    // {
    //     track_mode = NORMAL;
    //     road_smh->status = BARRIER_NONE;
    //     road_smh->pointflag = 0;
    // }
}


void run_off(Trait_smachine *road_smh)
{

    aim_theta = 110;
    if (theta > aim_theta)
    {
        // aim_theta = 0;
        // test
        track_mode = OBSTACLE;
    }
}