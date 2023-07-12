/*
    2023.3.18
    18届智能车
    elements -> 环岛 车库 断路 路障 十字 坡道
    移植代码 -> 去除三叉和回环
*/
#include "zf_common_headfile.h"
                         
changepoint Parking_L, Parking_R;                                                                   // 定义车库拐点结构体
changepoint RoundAbout_LeftDown, RoundAbout_LeftAbove, RoundAbout_RightDown, RoundAbout_RightAbove; // 环岛拐点结构体

short StartLine_Counter = 0;    // 斑马线识别计时器
short Upslope_Counter = 0;      // 上坡识别计时器
short Downslope_Counter = 0;    // 下坡识别计时器
short Rounding_LCounter = 0;    // 左环岛识别计时器
short Rounding_RCounter = 0;    // 右环岛识别计时器
short Departure_cnt = 0;        // 发车计数器
int blackpoints = 0;

unsigned char Present_RoundAbout_PointFlagL = 1;    /*********************/
unsigned char Present_RoundAbout_PointFlagR = 1;


/*------各类标志位-------*/
unsigned char StartLine_PointFlag = 0;      // 斑马线标志位
unsigned char RoundAbout_PointFlag_L = 0;   // 左环岛标志位
unsigned char RoundAbout_PointFlag_R = 0;   // 右环岛标志位
unsigned char Upslope_PointFlag = 0;        // 上坡标志位
unsigned char Downslope_PointFlag = 0;      // 下坡标志位
unsigned char Parking_PointFlag = 0;        // 停车标志位
unsigned char BreakRoad_PointFlag = 0;      // 断路标志位
unsigned char Obstacle_PointFlag = 0;       // 障碍标志位
unsigned char Departure_PointFlag = 0;          // 发车标志位


void Judging_Elements(void)
{
    Judging_Break_Road(&bin_image[0]);
    Judging_StartLine(&bin_image[0]);
    // Judging_RoundAbout(&bin_image[0]);
    if (BreakRoad_PointFlag == 1)
    {
        buzzer_flag = 1;
        pit_disable(CCU60_CH0);
        pit_enable(CCU60_CH1);
    }
    else if (BreakRoad_PointFlag == 0)
    {
        buzzer_flag = 0;
        pit_disable(CCU60_CH1);
        pit_enable(CCU60_CH0);
    }
}


/*
    绝对值计算函数
*/
int absolute(int var){
    if(var < 0)
        return -var;
    else
        return var;
}

/*
    函数名称：寻找左拐点
    函数参数：一个changepoint类型的结构体，拐点寻找的起始行，结束行，二值化后的原始数组，不同拐点的模式参数
    为了方便寻找不同类型的拐点，预先设置了不同的模式（mode），一一对应不同类型的拐点。大体上，将拐点分为上下拐点两类，在下拐点又细分
    角拐点和平拐点，比如三岔入口底部左右两侧的角拐点以及十字路口出入口底部左右两侧的平拐点。

    @parameter
    prt -> 结构体变量地址或结构体指针
    Start -> 拐点寻找的起始行
    End -> 拐点寻找的结束行
    (*binary_array)[188] -> 二值化后的数组
    mode -> 拐点寻找的模式，不同的拐点给定了不同的模式
*/
void Findchangepoint_L(changepoint *prt, unsigned char Start, unsigned char End, unsigned char (*binary_array)[188], unsigned char mode){
    // 角拐点
    if(mode == TRIP){
        prt->changepoint_start = Start;     // 拐点寻找的起始行
        prt->changepoint_end = End;         // 拐点寻找的结束行
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for (unsigned char i = prt->changepoint_start; i > prt->changepoint_end; i--){
            if (leftline[i] != 0 && leftline[i - 1] != 0 && leftline[i + 1] != 0
            && leftline[i - 2] != 0
            && leftline[i] < 178 && leftline[i - 1] < 178 && leftline[i + 1] < 178
            && leftline[i - 2] < 178)
            {

                if ((leftline[i] - leftline[i - 1] >= 0) && (leftline[i] - leftline[i - 2] >= 0)
                && (leftline[i] - leftline[i - 3] >= 2)
                && (leftline[i] - leftline[i + 1] >= 0)
                && (leftline[i] - leftline[i + 2] >= 0)
                && (leftline[i] - leftline[i + 3] >= 0))
                {
                    prt->changepoint_row = i;
                    prt->changepoint_col = leftline[i];
                    prt->changepoint_flag = 1;
                    break;
                }
            }
        }
        if(prt->changepoint_flag == 0){
            prt->changepoint_row = 119;
            prt->changepoint_col = 0;
        }
    }
    // 平拐点，但是没用
    if(mode == STRAIGHT){

        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        unsigned char Temp_Left_X, Temp_Left_Y, A;
        A = 0;
        Temp_Left_X = 95;
        for(unsigned char i = 93; i >= 5; i--){
            if(binary_array[Temp_Left_X][i] == 0 && binary_array[Temp_Left_X][i + 1] == 255){
                Temp_Left_Y = i;
                A = 1;
                break;
            }
        }

        if(A){
            Temp_Left_X = Temp_Left_X - 1;
            for(unsigned char i = Temp_Left_X; i > 0; i--){
                for(unsigned char j = Temp_Left_Y; j < 185; j++){
                    if(binary_array[i][j] == 255){
                        prt->changepoint_col = j - 1;
                        break;
                    }
                }
                if(binary_array[i - 1][prt->changepoint_col] == 255){
                    prt->changepoint_row = i ;
                    prt->changepoint_flag = 1;
                    break;
                }
            }
        }
    }
    // 下平拐点
    if(mode == CROSSING_DOWN){
        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for (unsigned char i = prt->changepoint_start; i > prt->changepoint_end; i--){
            if ((leftline[i] - leftline[i - 1] >= 0) && (leftline[i] - leftline[i - 2] >= 0)
                && (leftline[i] - leftline[i - 3] >= 8)
                && (leftline[i] - leftline[i + 1] >= 0)
                && (leftline[i] - leftline[i + 2] >= 0)
                && (leftline[i] - leftline[i + 3] >= 3))
            {
                prt->changepoint_row = i - 1;
                prt->changepoint_col = leftline[i];
                prt->changepoint_flag = 1;
                break;
            }
        }
    }
    // 上平拐点
    if(mode == CROSSING_ABOVE){

        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for(unsigned char i = prt->changepoint_start; i > prt->changepoint_end; i--){
            if(leftline[i] - leftline[i - 1] <= 0 && leftline[i] - leftline[i - 2] <= 0 && leftline[i] - leftline[i - 3] <= -2
            && leftline[i] - leftline[i + 1] >= 0 && leftline[i] - leftline[i + 2] >= 0 && leftline[i] - leftline[i + 3] >= 10){
                prt->changepoint_row = i + 1;
                prt->changepoint_col = leftline[i];
                prt->changepoint_flag = 1;
                break;
            }
        }
    }
}


/*
    函数名称：寻找右拐点
    函数参数：一个changepoint类型的结构体，拐点寻找的起始行，结束行，二值化后的原始数组，不同拐点的模式参数
    为了方便寻找不同类型的拐点，预先设置了不同的模式（mode），一一对应不同类型的拐点。大体上，将拐点分为上下拐点两类，在下拐点又细分
    角拐点和平拐点，比如三岔入口底部左右两侧的角拐点以及十字路口出入口底部左右两侧的平拐点。

    @parameter
    prt -> 结构体变量地址或结构体指针
    Start -> 拐点寻找的起始行
    End -> 拐点寻找的结束行
    (*binary_array)[188] -> 二值化后的数组
    mode -> 拐点寻找的模式，不同的拐点给定了不同的模式
*/
void Findchangepoint_R(changepoint *prt, unsigned char Start, unsigned char End, unsigned char (*binary_array)[188], unsigned char mode){
    // 角拐点
    if(mode == TRIP){
        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for (unsigned char i = prt->changepoint_start; i >= prt->changepoint_end; i--){
            if((rightline[i] != (cutCOL - 1) && rightline[i - 1] != (cutCOL - 1)
            && rightline[i - 2] != (cutCOL - 1)
            && rightline[i - 3] != (cutCOL - 1))
            && (rightline[i] > 10 && rightline[i - 1] > 10
            && rightline[i + 1] > 10 && rightline[i - 2] > 10))
            {
                if ((rightline[i] - rightline[i + 1] <= 0)
                && (rightline[i] - rightline[i + 2]) <= 0
                && (rightline[i] - rightline[i + 3]) <= 0
                && (rightline[i] - rightline[i - 1] <= 0)
                && (rightline[i] - rightline[i - 2] <= 0)
                && (rightline[i] - rightline[i - 3] <= -2))
                {
                    prt->changepoint_row = i;
                    prt->changepoint_col = rightline[i];
                    prt->changepoint_flag = 1;
                    break;
                }
            }
        }
        if(prt->changepoint_flag == 0){
            prt->changepoint_row = 119;
            prt->changepoint_col = 187;
        }
    }

    // 平拐点，但是没用
    if(mode == STRAIGHT){
        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        unsigned char Temp_Right_X, Temp_Right_Y, B;
        Temp_Right_X = 95;
        for(unsigned char i = 93; i < cutCOL; i++){
            if(binary_array[Temp_Right_X][i] == 0 && binary_array[Temp_Right_X][i - 1] == 255){
                Temp_Right_Y = i;
                B = 1;
                break;
            }
        }

        if(B){
            Temp_Right_X = Temp_Right_X - 1;
            for(unsigned char i = Temp_Right_X; i > 0; i--){
                for(unsigned char j = Temp_Right_Y; j > 5; j--){
                    if(binary_array[i][j] == 255){
                        prt->changepoint_col = j + 1;
                        break;
                    }
                }
                if(binary_array[i - 1][prt->changepoint_col] == 255){
                    prt->changepoint_row = i;
                    prt->changepoint_flag = 1;
                    break;
                }
            }
        }
    }

    // 下平拐点
    if(mode == CROSSING_DOWN){
        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for (unsigned char i = prt->changepoint_start; i >= prt->changepoint_end; i--){
            if ((rightline[i] - rightline[i + 1] <= 0)
                && (rightline[i] - rightline[i + 2]) <= 0
                && (rightline[i] - rightline[i + 3]) <= -3
                && (rightline[i] - rightline[i - 1] <= 0)
                && (rightline[i] - rightline[i - 2] <= 0)
                && (rightline[i] - rightline[i - 3] <= -3))
            {
                prt->changepoint_row = i - 1;
                prt->changepoint_col = rightline[i];
                prt->changepoint_flag = 1;
                break;
            }
        }
    }

    // 上平拐点
    if(mode == CROSSING_ABOVE){

        prt->changepoint_start = Start;
        prt->changepoint_end = End;
        prt->changepoint_row = 0;
        prt->changepoint_col = 0;
        prt->changepoint_flag = 0;

        for(unsigned char i = prt->changepoint_start; i >= prt->changepoint_end; i--){
            if(rightline[i] - rightline[i - 1] >= 0 && rightline[i] - rightline[i - 2] >= 0 && rightline[i] - rightline[i - 3] >= 0
            && rightline[i] - rightline[i + 1] <= 0 && rightline[i] - rightline[i + 2] <= 0 && rightline[i] - rightline[i + 3] <= -10){
                prt->changepoint_row = i + 1;
                prt->changepoint_col = rightline[i];
                prt->changepoint_flag = 1;
                break;
            }
        }
    }
}




// 判断起跑线
void Judging_StartLine(unsigned char (*binary_array)[188]){
    Findchangepoint_L(&Parking_L, 70, 40, &image_deal[0], CROSSING_ABOVE);
    Findchangepoint_R(&Parking_R, 70, 40, &image_deal[0], CROSSING_ABOVE);
    unsigned char times = 0;
    for (unsigned char i = BottomRow - 18; i >= BottomRow - 23; i--)
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
        StartLine_PointFlag = 1;
    }
    else
    {
        StartLine_PointFlag = 0;
    }
    
    if(StartLine_PointFlag && Parking_L.changepoint_flag){
        Parking_PointFlag = 1;
        for(unsigned char i = BottomRow; i > 60; i--){
            centerline[i] = 93;
        }
    }
    
    if(StartLine_PointFlag && Parking_R.changepoint_flag && leftline[119] != StartCoL && StartLine_Counter >= 1000){
        Parking_PointFlag = 2;
        float TempK;
        TempK = (float)(Parking_R.changepoint_col - 0) / (float)(Parking_R.changepoint_row - 119);
        for(unsigned char i = BottomRow; i > Parking_R.changepoint_row; i--){
            leftline[i] = (int)(TempK * (i - Parking_R.changepoint_row) + Parking_R.changepoint_col);
            rightline[i] = 187;
        }
    }

}

// 判断环岛
void Judging_RoundAbout(unsigned char (*binary_array)[188]){
    // Judging right roundabout point flag 1st
    for(unsigned char i = BottomRow - 10; i > 60; i--){
        if(leftline[BottomRow] != StartCoL && rightline[BottomRow] != EndCoL && absolute(Right_RoadWidth[i + 10] - Right_RoadWidth[i] > 7)
        && absolute(Right_RoadWidth[i - 10] - Right_RoadWidth[i]) > 35 && RoundAbout_PointFlag_R <= 1 && Rounding_RCounter < 300)
        {
            RoundAbout_PointFlag_R = 1;
            break;
        }
    }
    // Judging right roundabout point flag 2nd
    if(RoundAbout_PointFlag_R > 0 && RoundAbout_PointFlag_R <= 2 && Right_RoadWidth[BottomRow - 10] - Left_RoadWidth[BottomRow - 10] > 5 && Right_RoadWidth[BottomRow - 111] - Left_RoadWidth[BottomRow - 11] > 6
    && Right_RoadWidth[BottomRow - 12] - Left_RoadWidth[BottomRow - 12] > 8 && Right_RoadWidth[BottomRow - 13] - Left_RoadWidth[BottomRow - 13] > 8
    && Right_RoadWidth[BottomRow - 14] - Left_RoadWidth[BottomRow - 14] > 8 && Right_RoadWidth[BottomRow - 15] - Left_RoadWidth[BottomRow-  15] > 8 && RoundAbout_PointFlag_R <= 2 && Rounding_RCounter < 300)
    {
        RoundAbout_PointFlag_R = 2;
    }
    // Judging right roundabout point flag 3rd
    for(unsigned char i = BottomRow - 10; i > 60; i--){
        if(leftline[BottomRow] != StartCoL && rightline[BottomRow] != EndCoL && Right_RoadWidth[i + 10] - Right_RoadWidth[i + 5] > 3 && Right_RoadWidth[i + 5] - Right_RoadWidth[i] > 2
        && Right_RoadWidth[i] - Right_RoadWidth[i + 10] < -5 && Right_RoadWidth[i] - Right_RoadWidth[i - 5] < -2 && Right_RoadWidth[i - 5] - Right_RoadWidth[i - 10] < -3
        && Right_RoadWidth[i - 10] - Right_RoadWidth[i] > 5 && RoundAbout_PointFlag_R >= 2 && RoundAbout_PointFlag_R <= 3 && Rounding_RCounter < 300)
        {
            RoundAbout_PointFlag_R = 3;
            break;
        }
    }
    // Judging right roundabout point flag 4th, Entering the roundabout
    if(RoundAbout_PointFlag_R >= 3 && RoundAbout_PointFlag_R <= 4 && rightline[BottomRow] > 180 && rightline[BottomRow - 1] > 180 && rightline[BottomRow - 2] > 180
    && rightline[BottomRow - 3] > 180 && rightline[BottomRow - 4] > 180 && rightline[BottomRow - 5] > 180 && Rounding_RCounter < 300)
    {
        Findchangepoint_R(&RoundAbout_RightAbove, 119, 40, &image_deal[0], CROSSING_ABOVE);
        if(RoundAbout_RightAbove.changepoint_flag && RoundAbout_RightAbove.changepoint_col > 100){
            RoundAbout_PointFlag_R = 4;
            unsigned char TempPoint_Row, TempPoint_Col;
            TempPoint_Row = BottomRow;
            TempPoint_Col = leftline[BottomRow];
            float TempK;
            TempK = (float)(TempPoint_Col - RoundAbout_RightAbove.changepoint_col) / (float)(TempPoint_Row - RoundAbout_RightAbove.changepoint_row);
            for(unsigned char i = TempPoint_Row; i > RoundAbout_RightAbove.changepoint_row; i--){
                leftline[i] = (int)(TempK * (i - TempPoint_Row) + TempPoint_Col);
            }
        }
    }
    // Judging right roundabout point flag 5th, running in the roundabout
    if(Rounding_RCounter >= 200 && RoundAbout_PointFlag_R >= 4){
        RoundAbout_PointFlag_R = 5;
    }
    // Judging right roundabout point flag 6th, prepare to leave the roundabout
    if(RoundAbout_PointFlag_R >= 5 && RoundAbout_PointFlag_R <= 6 && Rounding_RCounter >= 300){
        Findchangepoint_R(&RoundAbout_LeftDown, 119, 40, &image_deal[0], TRIP);
        if(RoundAbout_LeftDown.changepoint_flag && RoundAbout_LeftDown.changepoint_col < 93 && RoundAbout_LeftDown.changepoint_col >= 20){
            RoundAbout_PointFlag_R = 6;
        }
        if(RoundAbout_PointFlag_R == 6){
            unsigned char TempPoint_Row, TempPoint_Col;
            float TempK;
            TempPoint_Col = 93;
            for(unsigned char i = BottomRow; i > 20; i--){
                if(binary_array[i + 1][TempPoint_Col] == 255 && binary_array[i][TempPoint_Col] == 0 && binary_array[i - 1][TempPoint_Col] == 0){
                    TempPoint_Row = i;
                    break;
                }
            }
            TempK = (float)(TempPoint_Col - RoundAbout_LeftDown.changepoint_col) / (float)(TempPoint_Row - RoundAbout_LeftDown.changepoint_row);
            for(unsigned char i = RoundAbout_LeftDown.changepoint_row; i > TempPoint_Row; i--){
                leftline[i] = (int)(TempK * (i - TempPoint_Row) + TempPoint_Col);
            }
        }
    }


    // Judging the left roundabout point flag 1st
    for(unsigned char i = BottomRow - 10; i > 60; i--){
        if(leftline[BottomRow] != StartCoL && rightline[BottomRow] != EndCoL && absolute(Right_RoadWidth[i + 10] - Left_RoadWidth[i + 10] < 10)
        && absolute(Right_RoadWidth[i - 10] - Left_RoadWidth[i - 10]) > 35 && RoundAbout_PointFlag_L <= 1)
        {
            RoundAbout_PointFlag_L = 1;
            break;
        }
    }
    // Judging the left roundabout point flag 2nd
    if(RoundAbout_PointFlag_L > 0 && RoundAbout_PointFlag_L <= 2 && leftline[BottomRow] > 180 && leftline[BottomRow - 1] > 180 && leftline[BottomRow - 2] > 180
    && leftline[BottomRow - 3] > 180 && leftline[BottomRow - 4] > 180 && leftline[BottomRow - 5] > 180 && RoundAbout_PointFlag_L <= 2)
    {
        RoundAbout_PointFlag_L = 2;
    }
    // Judging the left roundabout point flag 3rd
    for(unsigned char i = BottomRow - 10; i > 60; i--){
        if(rightline[BottomRow] != StartCoL && Left_RoadWidth[i + 10] - Left_RoadWidth[i] < -10 && Left_RoadWidth[i - 10] - Left_RoadWidth[i] < -10
        && RoundAbout_PointFlag_L >= 2 && RoundAbout_PointFlag_L <= 3)
        {
            RoundAbout_PointFlag_L = 3;
            break;
        }
    }
    // Judging the left roundabout point flag 4th, Entering the roundabout
    if(RoundAbout_PointFlag_L >= 3 && RoundAbout_PointFlag_L <= 4 && leftline[BottomRow] > 180 && leftline[BottomRow - 1] > 180 && leftline[BottomRow - 2] > 180
    && leftline[BottomRow - 3] > 180 && leftline[BottomRow - 4] > 180 && leftline[BottomRow - 5] > 180)
    {
        Findchangepoint_L(&RoundAbout_LeftAbove, 119, 40, &image_deal[0], CROSSING_ABOVE);
        if(RoundAbout_LeftAbove.changepoint_flag && RoundAbout_LeftAbove.changepoint_col > 100){
            RoundAbout_PointFlag_L = 4;
            unsigned char TempPoint_Row, TempPoint_Col;
            TempPoint_Row = BottomRow;
            TempPoint_Col = rightline[BottomRow];
            float TempK;
            TempK = (float)(TempPoint_Col - RoundAbout_LeftAbove.changepoint_col) / (float)(TempPoint_Row - RoundAbout_LeftAbove.changepoint_row);
            for(unsigned char i = TempPoint_Row; i > RoundAbout_LeftAbove.changepoint_row; i--){
                rightline[i] = (int)(TempK * (i - TempPoint_Row) + TempPoint_Col);
            }
        }
    }
    // Judging the left roundabout point flag 5th, running in the roundabout
    if(Rounding_LCounter >= 200 && RoundAbout_PointFlag_L >= 4){
        RoundAbout_PointFlag_L = 5;
    }
    // Judging the left roundabout point flag 6th, prepare to leave the roundabout
    if(RoundAbout_PointFlag_L >= 5 && RoundAbout_PointFlag_L <= 6 && Rounding_RCounter <= 300){
        Findchangepoint_R(&RoundAbout_RightDown, 119, 40, &image_deal[0], TRIP);
        if(RoundAbout_RightDown.changepoint_flag && RoundAbout_RightDown.changepoint_col < 93 && RoundAbout_RightDown.changepoint_col >= 20){
            RoundAbout_PointFlag_L = 6;
        }
        if(RoundAbout_PointFlag_L == 6){
            unsigned char TempPoint_Row, TempPoint_Col;
            float TempK;
            TempPoint_Col = 93;
            for(unsigned char i = BottomRow; i > 20; i--){
                if(binary_array[i + 1][TempPoint_Col] == 255 && binary_array[i][TempPoint_Col] == 0 && binary_array[i - 1][TempPoint_Col] == 0){
                    TempPoint_Row = i;
                    break;
                }
            }
            TempK = (float)(TempPoint_Col - RoundAbout_RightDown.changepoint_col) / (float)(TempPoint_Row - RoundAbout_RightDown.changepoint_row);
            for(unsigned char i = RoundAbout_RightDown.changepoint_row; i > TempPoint_Row; i--){
                rightline[i] = (int)(TempK * (i - TempPoint_Row) + TempPoint_Col);
            }
        }
    }
}


void Judging_Slope(void){
    if(Real_Gyro_Y <= -35){
        Upslope_PointFlag = 1;
    }
    if(Real_Gyro_Y >= 35){
        Downslope_PointFlag = 1;
    }
}

//  --------------
// -> start from here 
// 2 * 188 = 376

void Judging_Break_Road(unsigned char (*binary_array)[188])
{
    blackpoints = 0;
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
    if (blackpoints > 70)  BreakRoad_PointFlag = 1;
    else                    BreakRoad_PointFlag = 0;
}

void Departure(void)
{
    while (!Departure_PointFlag)
    {
        pit_disable(CCU60_CH0);
    }
    Departure_PointFlag = 1;
    // if (outflag)
    // {
    //     motor_ctrl(3000, 2000);
    //     system_delay_ms(1000);
    //     motor_ctrl(0, 0);
    //     system_delay_ms(1000);
    //     outflag = 0;
    // }
    pit_enable(CCU60_CH0);
}
