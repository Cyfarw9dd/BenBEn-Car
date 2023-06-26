#include "zf_common_headfile.h"

void cross_init(Trait_smachine *road_smachine)
{
    road_smachine->status = CROSS_NONE;
}

void Crossing_process(Trait_smachine *road_smachine)
{
    int limit_row = 0;
    bool X_found = Lpt0_found && Lpt1_found;
    if (!X_found)
    {
        road_smachine->cpt0_row = 0;
        road_smachine->cpt0_col = 0;
        road_smachine->cpt1_row = 0;
        road_smachine->cpt1_col = 0;
    }
    else 
    {
        road_smachine->cpt0_row = (unsigned char) points_l[Lpt0_rpts0s_id][1];
        road_smachine->cpt0_col = (unsigned char) points_l[Lpt0_rpts0s_id][0];
        road_smachine->cpt1_row = (unsigned char) points_r[Lpt1_rpts1s_id][1];
        road_smachine->cpt1_col = (unsigned char) points_r[Lpt1_rpts1s_id][0];
        limit_row = (road_smachine->cpt0_row + road_smachine->cpt1_row) / 2;
    }
    // ???????????????1
    if (X_found && road_smachine->status == CROSS_NONE && limit_row > Dpt_minrow && limit_row < Dpt_maxrow)
    {
        road_smachine->status = CROSS_IN;
        road_smachine->pointflag = 1;
    }

    // ??????????????????
    if (X_found && road_smachine->status == CROSS_IN && road_smachine->pointflag == 1 && limit_row > Upt_minrow && limit_row < Upt_maxrow)
    {
        road_smachine->pointflag = 2;
        road_smachine->status = CROSS_IN;
        // ??? trackmode = farline ????????????
    }
    if (road_smachine->status == CROSS_IN && road_smachine->pointflag == 2)
    {
        // ?????????????
        if (X_found && road_smachine->pointflag == 3 && limit_row > Dpt_minrow && limit_row < Dpt_maxrow)
        {
            road_smachine->pointflag = 4;
            road_smachine->status = CROSS_IN;
        }
        else 
        {
            road_smachine->pointflag = 3;
            road_smachine->status = CROSS_IN;
        }
    }
    // ????
    if (X_found && road_smachine->pointflag == 4 && road_smachine->status == CROSS_IN && limit_row > Upt_minrow && limit_row < Upt_maxrow)
    {
        road_smachine->pointflag = 5;
        road_smachine->status = CROSS_IN;
        // ???
    }
    if (!X_found && road_smachine->pointflag == 5)
    {
        road_smachine->pointflag = 0;
        road_smachine->status = CROSS_NONE;
    }
}


