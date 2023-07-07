#include "zf_common_headfile.h"

#define BLACK_NUM   80
#define ROADFOUND   50
int break_blackpoints;
void BreakRoad_process(Trait_smachine *road_smh, unsigned char (*image)[188])
{
    break_blackpoints = 0;
    // 对底边两行扫描黑点
    for (int row = CLIP_IMAGE_H - 3; row > CLIP_IMAGE_H - 5; row--)
    {
        for (int left_col = 93; left_col > 0; left_col -= 3)
        {
            if (image[row][left_col + 3] < clip_image_thereshold)
            {
                break_blackpoints++;
            }
        }
        for (int right_col = 0; right_col < 187; right_col += 3)
        {
            if (image[row][right_col - 3] < clip_image_thereshold)
            {
                break_blackpoints++;
            }
        }
    }

    if (break_blackpoints > BLACK_NUM && road_smh->status == BREAKROAD_NONE)
    {
        road_smh->pointflag = 1;
        road_smh->status = BREAKROAD_IN;
    }
    if (break_blackpoints > BLACK_NUM && road_smh->status == BREAKROAD_IN && road_smh->pointflag == 1)
    {
        road_smh->pointflag = 2;
        track_mode = ADC;
    }
    if (break_blackpoints < ROADFOUND && road_smh->status == BREAKROAD_IN && road_smh->pointflag == 2)
    {
        road_smh->pointflag = 0;
        road_smh->status = BREAKROAD_NONE;
        track_mode = NORMAL;
    }
}
