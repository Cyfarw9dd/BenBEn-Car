#include "zf_common_headfile.h"


void BreakRoad_process(Trait_smachine *road_smh)
{
    int blackpoints = 0;
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

    if (blackpoints > 70 && road_smh->status == BREAKROAD_NONE)
    {
        road_smh->pointflag = 1;
        road_smh->status = BREAKROAD_IN;
        track_mode = ADC;
    }
    if (blackpoints < 70 && road_smh->status == BREAKROAD_IN)
    {
        road_smh->pointflag = 0;
        road_smh->status = BREAKROAD_NONE;
        track_mode = NORMAL;
    }
}
