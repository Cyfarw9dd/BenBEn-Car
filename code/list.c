#include "zf_common_headfile.h"

extern icm_param_t imu_data;
extern euler_param_t eulerAngle;

char Key1_last_status;
char Key2_last_status;
char Key3_last_status;
char Key4_last_status;
char Key1_status;
char Key2_status;
char Key3_status;
char Key4_status;

char Key1_flag;
char Key2_flag;
char Key3_flag;
char Key4_flag;


// 菜单的主体结构体，结构体数组
typedef struct 
{
    int ListIndex;             // 当前菜单索引
    char ListName[30];    // 菜单名称
    void (*list_action)(float *param, char name[30]);
    float *param;
}List;

int parent_list_index = 0;
int highlight_col = 0;
//主菜单
List MyList[] = 
{
    {1, "Image Mode", NULL, NULL},              // 图像模式
    {11, "gray image", show_gray_image, NULL},             // 显示灰度图像
    {12, "binary Imange", show_binary_image, NULL},          // 显示二值化图像
    {13, "show boudaries", NULL, NULL},         // 显示图像边界
    {131, "show ori bound", show_boundaries, NULL},
    {132, "show sample", show_sample_boundaries, NULL},
    {14, "show road traits", show_roadtraits, NULL},
    {15, "show corners", show_corners, NULL},
    {16, "show barrier", show_barrier_params, NULL},
    {17, "back to main", NULL, NULL},           // 返回主菜单
    {2, "Tuning", NULL, NULL},
    {21, "Speed Cycle", NULL, NULL},
    {211, "Camera", NULL, NULL},
    {2111, "P", NULL, NULL},
    {2112, "I", NULL, NULL},
    {2113, "D", NULL, NULL},
    {2114, "go back", NULL, NULL},
    {212, "ADC", NULL, NULL},
    {2121, "P", NULL, NULL},
    {2122, "I", NULL, NULL},
    {2123, "D", NULL, NULL},
    {2124, "go back", NULL, NULL},
    {22, "InnerTurn Cycle", NULL, NULL},
    {221, "P", NULL, NULL},
    {222, "I", NULL, NULL},
    {223, "D", NULL, NULL},
    {224, "go back", NULL, NULL},
    {23, "Turn Cycle", NULL, NULL},
    {231, "Camera", NULL, NULL},
    {2311, "P", NULL, NULL},
    {2322, "I", NULL, NULL},
    {2333, "D", NULL, NULL},
    {232, "ADC", NULL, NULL},
    {2321, "P", NULL, NULL},
    {2322, "I", NULL, NULL},
    {2323, "D", NULL, NULL},
    {2324, "go back", NULL, NULL},
    {233, "go back", NULL, NULL},
    {24, "show params", show_params, NULL},
    {25, "debug mode", debug_mode, NULL},
    {26, "go back", NULL, NULL},
    {3, "Departure", NULL, NULL},
};

List *current_list_item;

void MyKeyScan(void)
{
    Key1_last_status = Key1_status;
    Key2_last_status = Key2_status;
    Key3_last_status = Key3_status;
    Key4_last_status = Key4_status;

    Key1_status = gpio_get_level(KEY4);
    Key2_status = gpio_get_level(KEY3);
    Key3_status = gpio_get_level(KEY2);
    Key4_status = gpio_get_level(KEY1);

    if (Key1_status && !Key1_last_status)   Key1_flag = 1;
    if (Key2_status && !Key2_last_status)   Key2_flag = 1;
    if (Key3_status && !Key3_last_status)   Key3_flag = 1;
    if (Key4_status && !Key4_last_status)   Key4_flag = 1;
}

void Key_flag_clear(void)
{
    Key1_flag = 0;
    Key2_flag = 0;
    Key3_flag = 0;
    Key4_flag = 0;
}

int have_sub_list(int ListIndex)     // 查看是否存在子菜单
{
    for (int i = 0; i < sizeof(MyList) / sizeof(List); i++)    // 遍历整个结构体数组
    {
        if (MyList[i].ListIndex / 10 == ListIndex)                  // 查看是否有子菜单
        {
            return true;
        }
    }
    return false;
}

int show_sub_list(int parent_index, int highlight_col)      // 显示子菜单以及当前高亮菜单
{
    int item_index = 0; // 子菜单追踪计数
    for (int i = 0; i < sizeof(MyList) / sizeof(List); i++)        // 遍历整个结构体数组
    {
        if (MyList[i].ListIndex / 10 == parent_index)           
        {
            if (item_index == highlight_col)
            {
                current_list_item = &MyList[i];
                ips200_set_color(RGB565_RED, RGB565_BLACK);
            }
            else
            {
                ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            }
            ips200_show_string(1, 20 * item_index, MyList[i].ListName);
            item_index++;
        }
    }
    return item_index;          // 返回子菜单的个数
}


void List_Switch(void)
{
    int list_item_count = show_sub_list(parent_list_index, highlight_col);
    // parent_list_index = current_list_item->ListIndex / 10;
    int i = have_sub_list(current_list_item->ListIndex);
    ips200_show_int(0, 120, current_list_item->ListIndex, 5);
    ips200_show_int(0, 135, i, 5);
    MyKeyScan();
    if (Key1_flag && highlight_col > 0)    // 按下按键1减少当前行数
    {
        highlight_col--;
    }
    else if (Key2_flag && highlight_col < list_item_count - 1)    // 按下按键2增加
    {
        highlight_col++;
    }
    else if (Key3_flag)
    {
        ips200_clear();     // 清屏
        if (have_sub_list(current_list_item->ListIndex))
        {
            highlight_col = 0;
            parent_list_index = current_list_item->ListIndex;
        }
        else if (strcmp(current_list_item->ListName, "back to main") == 0)   // 如果当前菜单为Back to Main，则返回主菜单
        {
            highlight_col = 0;
            parent_list_index = 0;
        }
        else if (current_list_item->list_action)
        {
            current_list_item->list_action(current_list_item->param, current_list_item->ListName);
        }
        else if (strcmp(current_list_item->ListName, "go back") == 0)   // 如果当前菜单为go back，则返回上一级
        {
            highlight_col = 0;
            parent_list_index = parent_list_index / 10;
        }
        else if (strcmp(current_list_item->ListName, "Departure") == 0)   // 发车模式
        {
            highlight_col = 0;
            Departure_PointFlag = 1;
        }
        else if (strcmp(current_list_item->ListName, "buzzer test") == 0)   
        {
            highlight_col = 0;
            buzzer_flag = 1;
        }
    }
    else if (Key4_flag)
    {
        ips200_clear();     // 清屏
        highlight_col = 0;
        parent_list_index = 0;
    }
    list_item_count = show_sub_list(parent_list_index, highlight_col);
    Key_flag_clear();
}


void show_gray_image(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        ips200_show_gray_image(0, 0, clip_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        ips200_show_string(0, 65, "StartLine_PFlag");       ips200_show_int(100, 65, clip_image_thereshold, 3);
        ips200_show_string(0, 80, "LRoundAbout_PFlag");     ips200_show_int(110, 80, rnum, 2);
        ips200_show_string(0, 95, "RRoundAbout_PFlag");     ips200_show_int(110, 95, RoundAbout_PointFlag_R, 2);
        ips200_show_string(0, 110, "BreakRoad_PFlag");      ips200_show_int(110, 110, BreakRoad_PointFlag, 2);
        ips200_show_string(0, 125, "Obstacle_PFlag");      ips200_show_int(110, 125, Obstacle_PointFlag, 2);
    }
    ips200_clear();
}

void show_binary_image(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        for(unsigned char i = CLIP_IMAGE_H - 1; i > 0; i--){
            clip_bin_image[i][clip_lfline[i] + 3] = 70;
            clip_bin_image[i][clip_rtline[i] - 3] = 70;
            clip_bin_image[i][clip_ctline[i]] = 70;
        }
        ips200_show_gray_image(0, 0, clip_bin_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        ips200_show_string(0, 65, "StartLine_PFlag");       ips200_show_int(110, 65, StartLine_PointFlag, 2);
        ips200_show_string(0, 80, "LRoundAbout_PFlag");     ips200_show_int(110, 80, RoundAbout_PointFlag_L, 2);
        ips200_show_string(0, 95, "RRoundAbout_PFlag");     ips200_show_int(110, 95, RoundAbout_PointFlag_R, 2);
        ips200_show_string(0, 110, "BreakRoad_PFlag");      ips200_show_int(110, 110, BreakRoad_PointFlag, 2);
        ips200_show_string(0, 125, "Obstacle_PFlag");      ips200_show_int(110, 125, Obstacle_PointFlag, 2);
        // ips200_draw_point(checkline_l.col[0] / 2, checkline_l.row[0] / 2, RGB565_BLUE);
        // ips200_draw_point(checkline_l.col[1] / 2, checkline_l.row[1] / 2, RGB565_RED);
        // ips200_draw_point(checkline_r.col[0] / 2, checkline_r.row[0] / 2, RGB565_BLUE);
        // ips200_draw_point(checkline_r.col[1] / 2, checkline_r.row[1] / 2, RGB565_RED);
    }
    ips200_clear();
        
}
void show_boundaries(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        // ips200_show_gray_image(0, 0, bin_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
        Hightlight_Lines(bin_image[0]);
        ips200_show_int(0, 80, data_stastics_l, 3);
        ips200_show_int(0, 90, data_stastics_r, 3);
        ips200_clear();
    }
    ips200_clear();
}

void tuning(void)
{
    ips200_clear();
    Key_flag_clear();
    ips200_show_string(0, 0, "start tuning");
}

void show_params(void)
{
    ips200_clear();
    Key_flag_clear();
    while(!Key3_flag)
    {
        MyKeyScan();  
        ips200_show_string(0, 0, "l_encoder");      ips200_show_int(80, 0, speed1, 5);
        ips200_show_string(0, 20, "r_encoder");     ips200_show_int(80, 20, speed2, 5);
        ips200_show_string(0, 40, "l_distance");    ips200_show_int(80, 40, left_distance, 5);
        ips200_show_string(0, 60, "r_distance");    ips200_show_int(80, 60, right_distance, 5);
        // ips200_show_string(0, 65, "imu_data_x");        
        // ips200_show_string(0, 80, "imu_data_x");        
        // ips200_show_string(0, 100, "imu_data_x");     
        ips200_show_float(0, 70, adc_value[0], 5, 5);
        ips200_show_float(0, 80, adc_value[1], 5, 5);
        ips200_show_float(0, 100, adc_value[2], 5, 5); 
        ips200_show_float(0, 120, adc_value[3], 5, 5); 
    }
    ips200_clear();
}

void show_roadtraits(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        // ips200_show_gray_image(0, 0, clip_bin_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        for (int i = 0; i < CLIP_IMAGE_H; i ++)
        {
            ips200_draw_point(clip_lfline[i], i, RGB565_WHITE);
            ips200_draw_point(clip_rtline[i] , i, RGB565_WHITE);
            ips200_draw_point(clip_ctline[i] , i, RGB565_WHITE);
        }
        
        // for (int i = 0; i < CLIP_IMAGE_H; i ++)
        // {
        //     ips200_draw_point(l_border[i], i, RGB565_BLUE);
        //     ips200_draw_point(r_border[i] , i, RGB565_GREEN);
        //     ips200_draw_point(center_line[i] , i, RGB565_YELLOW);
        // }
        // for(unsigned char i = BottomRow; i > 0; i--){
        //     ips200_draw_point((l_border[i] + 5) / 2, i / 2, RGB565_BLUE);
        //     ips200_draw_point(center_line[i] / 2, i / 2, RGB565_RED);
        //     ips200_draw_point((r_border[i] - 5)  / 2, i / 2, RGB565_GREEN);
        // }
        ips200_clear();
    }
    ips200_clear();
}

void show_corners(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        // 将角点周围的五个点都高亮
        // 显示二值化图像
        ips200_show_gray_image(0, 0, clip_bin_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        // 显示坐标
        ips200_show_int(0, 80, Lpt0_rpts0s_id, 3);
        ips200_show_int(0, 100, Lpt1_rpts1s_id, 3);
        ips200_show_float(0, 120, rpts0an[1] / PI * 180, 3, 3);
        ips200_show_float(0, 140, rpts1an[1] / PI * 180, 3, 3);
        // 显示左L角点
        highlight_Lcorners();
    }
    ips200_clear();
}

void show_sample_boundaries(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag)
    {
        MyKeyScan();
        for (int i = 0; i < 359; i++)
        {
            ips200_draw_point(points_ls[i][0], points_ls[i][1], RGB565_YELLOW);
            ips200_draw_point(points_rs[i][0], points_rs[i][1], RGB565_YELLOW);
        }
        // ips200_show_float(0, 80, points_ls[30][30], 3, 2);
        // ips200_show_int(0, 90, data_stastics_l, 3);
        // ips200_show_int(0, 100, data_stastics_r, 3);
    }    
    ips200_clear();
}

void show_barrier_params(void)
{
    ips200_clear();
    Key_flag_clear();
    barrier_turning_distance = 0;   
    while (!Key3_flag)
    {
        MyKeyScan();
        barrier_turning_distance += (speed1 + speed2) / 2;
        ips200_show_gray_image(0, 0, clip_bin_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        ips200_show_string(0, 80, "barrier ptflag");    ips200_show_int(100, 80, Barrier.pointflag, 3);
        ips200_show_int(0, 90, dl1a_distance_mm, 5);
        ips200_show_string(0, 120, "encoder");          ips200_show_int(70, 120, barrier_turning_distance, 5);
        if (Barrier.status == BARRIER_NONE)
        {
            ips200_show_string(0, 140, "didn't find barrier");
            // ips200_clear();
        }
        if (Barrier.status == BARRIER_IN)
        {
            ips200_show_string(0, 140, "barrier_found      ");
            // ips200_clear();
        }
    }
    ips200_clear();
}

void debug_mode(void)
{
    ips200_clear();
    Key_flag_clear();
    while (!Key3_flag) 
    {
        MyKeyScan();
        for(unsigned char i = CLIP_IMAGE_H - 1; i > 0; i--){
            clip_bin_image[i][clip_lfline[i] + 3] = 70;
            clip_bin_image[i][clip_rtline[i] - 3] = 70;
            clip_bin_image[i][clip_ctline[i]] = 70;
        }
        ips200_show_gray_image(0, 0, clip_bin_image[0], MT9V03X_W, CLIP_IMAGE_H, MT9V03X_W, CLIP_IMAGE_H, 0);
        aim_speed = 0;
    }
    ips200_clear();
}