#ifndef CODE_ELEMENTS_H_
#define CODE_ELEMENTS_H_

#include "zf_common_headfile.h"

#define P_CROSSING 0
#define DIVERGENCE 1
#define ROUNDABOUT 2
#define GARAGE     3

#define cutCOL 187
#define Button_row 119
#define TRIP 0
#define STRAIGHT 1
#define CROSSING_DOWN 2
#define CROSSING_ABOVE 3
#define BottomRow 119
#define CenterLine 0
#define LeftLine 1
#define RightLine 2
#define StartCoL 0
#define EndCoL 187
#define THRESHOLD 50


extern short StartLine_Counter;
extern short Upslope_Counter;
extern short Downslope_Counter;
extern short Rounding_LCounter;
extern short Rounding_RCounter;
extern short RoundAbout_LCounter;
extern short RoundAbout_RCounter;
extern short Departure_cnt;

extern unsigned char Present_RoundAbout_PointFlagL;
extern unsigned char Present_RoundAbout_PointFlagR;

typedef struct{
    unsigned char changepoint_start;
    unsigned char changepoint_end;
    unsigned char changepoint_row;
    unsigned char changepoint_col;
    unsigned char changepoint_flag;
}changepoint;


extern unsigned char StartLine_PointFlag;
extern unsigned char RoundAbout_PointFlag_L;
extern unsigned char RoundAbout_PointFlag_R;
extern unsigned char Upslope_PointFlag;
extern unsigned char Downslope_PointFlag;
extern unsigned char Parking_PointFlag;
extern unsigned char BreakRoad_PointFlag;
extern unsigned char Obstacle_PointFlag;
extern unsigned char Departure_PointFlag;

extern changepoint Parking_L, Parking_R;
extern changepoint RoundAbout_LeftDown, RoundAbout_LeftAbove, RoundAbout_RightDown, RoundAbout_RightAbove;


int absolute(int var);

int minimum(int var1, int var2);

void Findchangepoint_L(changepoint *prt, unsigned char Start, unsigned char End, unsigned char (*binary_array)[188], unsigned char mode);

void Findchangepoint_R(changepoint *prt, unsigned char Start, unsigned char End, unsigned char (*binary_array)[188], unsigned char mode);

unsigned char Leftline_Lost_Sum(unsigned char Start_Row, unsigned char End_row);

unsigned char Rightline_Lost_Sum(unsigned char Start_Row, unsigned char End_row);

void Judging_StartLine(unsigned char (*binary_array)[188]);

void Judging_RoundAbout(unsigned char (*binary_array)[188]);

void Judging_Slope(void);

void Judging_Break_Road(unsigned char (*binary_array)[188]);



#endif /* CODE_ELEMENTS_H_ */
