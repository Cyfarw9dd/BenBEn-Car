#include "zf_common_headfile.h"


void Startline_process(Trait_smachine *road_smh, unsigned char (*binary_array)[188])
{
    unsigned char times = 0;
    for (unsigned char i = (CLIP_IMAGE_H - 1) - 18; i >= (CLIP_IMAGE_H - 1) - 23; i--)
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
        road_smh->pointflag = 1;
    }
    else
    {
        road_smh->pointflag = 0;
    }
}
