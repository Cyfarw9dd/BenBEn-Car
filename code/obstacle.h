#ifndef CODE_OBSTACLE_H_
#define CODE_OBSTACLE_H_

#include "zf_common_headfile.h"

extern int barrier_turning_distance;
extern int left_distance; 
extern int right_distance;


enum obstacle_status{
    BARRIER_NONE = 10,
    BARRIER_IN,
};

void Barrier_process(Trait_smachine *road_smh);

void run_off(void);

#endif /* CODE_OBSTACLE_H_ */
