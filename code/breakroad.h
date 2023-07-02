#ifndef CODE_BREAKROAD_H_
#define CODE_BREAKROAD_H_

#include "zf_common_headfile.h"

enum Breakroad_status{
    BREAKROAD_IN = 21,
    BREAKROAD_NONE,
};


void BreakRoad_process(Trait_smachine *road_smh);

#endif /* CODE_BREAKROAD_H_ */
