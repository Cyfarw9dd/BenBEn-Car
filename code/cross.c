#include "zf_common_headfile.h"

void cross_init(Trait_smachine *road_smh)
{
    road_smh->status = CROSS_NONE;
}

void Crossing_process(Trait_smachine *road_smh)
{
    int limit_row = 0;
    bool X_found = Lpt0_found && Lpt1_found;
    if (!X_found)
    {
        road_smh->cpt0_row = 0;
        road_smh->cpt0_col = 0;
        road_smh->cpt1_row = 0;
        road_smh->cpt1_col = 0;
    }
    else 
    {
        road_smh->cpt0_row = (unsigned char) points_l[Lpt0_rpts0s_id][1];
        road_smh->cpt0_col = (unsigned char) points_l[Lpt0_rpts0s_id][0];
        road_smh->cpt1_row = (unsigned char) points_r[Lpt1_rpts1s_id][1];
        road_smh->cpt1_col = (unsigned char) points_r[Lpt1_rpts1s_id][0];
        limit_row = (road_smh->cpt0_row + road_smh->cpt1_row) / 2;
    }
    // 
    if (X_found && road_smh->status == CROSS_NONE && limit_row > Dpt_minrow && limit_row < Dpt_maxrow)
    {
        road_smh->status = CROSS_IN;
        road_smh->pointflag = 1;
    }

    // 
    if (X_found && road_smh->status == CROSS_IN && road_smh->pointflag == 1 && limit_row > Upt_minrow && limit_row < Upt_maxrow)
    {
        road_smh->pointflag = 2;
        road_smh->status = CROSS_IN;
        // 
    }
    if (road_smh->status == CROSS_IN && road_smh->pointflag == 2)
    {
        // 
        if (X_found && road_smh->pointflag == 3 && limit_row > Dpt_minrow && limit_row < Dpt_maxrow)
        {
            road_smh->pointflag = 4;
            road_smh->status = CROSS_IN;
        }
        else 
        {
            road_smh->pointflag = 3;
            road_smh->status = CROSS_IN;
        }
    }
    // 
    if (X_found && road_smh->pointflag == 4 && road_smh->status == CROSS_IN && limit_row > Upt_minrow && limit_row < Upt_maxrow)
    {
        road_smh->pointflag = 5;
        road_smh->status = CROSS_IN;
        // 
    }
    if (!X_found && road_smh->pointflag == 5)
    {
        road_smh->pointflag = 0;
        road_smh->status = CROSS_NONE;
    }
}


