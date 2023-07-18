#ifndef CODE_OBSTACLE_H_
#define CODE_OBSTACLE_H_

#include "zf_common_headfile.h"

extern int barrier_turning_distance;
extern int left_distance; 
extern int right_distance;
extern int turn_err;
extern int turn_flag;

enum obstacle_status{
    BARRIER_NONE = 10,
    BARRIER_IN,
    BARRIER_FOUND,
    BARRIER_TURN,
};

void Barrier_process(Trait_smachine *road_smh);

void run_off(Trait_smachine *road_smh);

#endif /* CODE_OBSTACLE_H_ */
