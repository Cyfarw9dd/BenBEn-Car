#include "zf_common_headfile.h"


#define DISTANCE0 11000
#define DISTANCE1 8000
int barrier_turning_distance = 0;
int left_distance = 0; 
int right_distance = 0;

void Barrier_process(Trait_smachine *road_smh)
{
    // 获取tof测距信息
    dl1a_get_distance();

    if (dl1a_distance_mm > 1200)
    {
        track_mode = NORMAL;
        road_smh->status = BARRIER_NONE;
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
        track_mode == OBSTACLE;
        run_off();
    }
    // if (road_smh->status == BARRIER_IN && road_smh->pointflag == 2)
    // {
    //     track_mode = NORMAL;
    //     road_smh->status = BARRIER_NONE;
    //     road_smh->pointflag = 0;
    // }
}

void run_off(void)
{
    // 打角 
    motor_ctrl(3000, 0);
    system_delay_ms(100);
    motor_ctrl(0, 0);
    system_delay_ms(100);
    pit_disable(CCU60_CH0);
}