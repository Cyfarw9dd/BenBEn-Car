#include "zf_common_headfile.h"

int garage_flag = 0;
int goin_flag = 0;

void Startline_process(Trait_smachine *road_smh, unsigned char (*binary_array)[188])
{
    // if (BreakRoad.trait_cnt >= 1)
    // {
        unsigned char times = 0;
        for (unsigned char i = (CLIP_IMAGE_H - 1) - 3; i >= (CLIP_IMAGE_H - 1) - 38; i--)
        {
            unsigned char black_blocks = 0;
            unsigned char cursor = 0;
            for(unsigned char j = 0; j <= 186; j++)
            {
                if(clip_bin_image[i][j] == 0)
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
            if(black_blocks >= 3 && black_blocks <= 12) times++;
        }
        if(times >= 3 && road_smh->pointflag != 1)     /*&& times <= 5*/
        {
            road_smh->pointflag = 1;
            road_smh->status = ZEBRA_IN;
            garage_flag = 1;    
        }
        if (garage_flag == 1)
        {
            Departure_PointFlag = 0;
            if (!goin_flag)
            {
                track_mode = OBSTACLE;
                motor_ctrl(2000, 3000);
                system_delay_ms(200);
                motor_ctrl(0, 0);
                system_delay_ms(200);
                goin_flag = 1;
            }
            garage_flag = 2;
        }
        if (garage_flag == 2)
        {
            // Departure_PointFlag = 1;
            // track_mode = GARAGE_STOP;
            while (true)
            {
                Departure_PointFlag = 0;
                pit_disable(CCU60_CH0);
            }
        }
    // }
}
