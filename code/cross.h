#ifndef CODE_CROSS_H_
#define CODE_CROSS_H_

#include "zf_common_headfile.h"

#define Dpt_minrow 50
#define Dpt_maxrow 70
#define Upt_minrow 40
#define Upt_maxrow 60

enum cross_status{
    CROSS_NONE = 0,
    CROSS_IN,
};

void Crossing_process(Trait_smachine *road_smh);



#endif /* CODE_CROSS_H_ */
