#include "avoid.h"

osSemaphoreId_t *avoid_sem;

int Should_Avoid(){
    if(fabs(Current_Yaw - 90.0f)<0.001){
        if(current_y + 1 <= 9 && current_y + 1 >= 0 && map[current_x][current_y + 1]){
            return 1;
        }
    }
            
    else if(fabs(Current_Yaw + 90.0f)<0.001){
        if(current_y - 1 <= 9 && current_y - 1 >= 0 && map[current_x][current_y - 1]){
            return 1;
        }
    }

    else if(fabs(Current_Yaw)<0.001){
         if(current_x + 1 <= 9 && current_x + 1 >= 0 && map[current_x + 1][current_y]){
            return 1;
        }
    }
    
    else if(fabs(Current_Yaw - 180.0f)<0.001){
         if(current_x - 1 <= 9 && current_x - 1 >= 0 && map[current_x - 1][current_y]){
            return 1;
        }
    }
    return 0;
}

void avoid_run(){
    int init_yaw = Current_Yaw;
    printf("start to avoid\n");
    int init_tar_x = current_target_x;
    int init_tar_y = current_target_y;
    if(abs(Current_Yaw - 90.0f)<0.001){
        Axis_move_to_target_position(current_x+1,current_y);
        Axis_move_to_target_position(current_x,current_y+3);
        Axis_move_to_target_position(current_x-1,current_y);
    }
        
    else if(abs(Current_Yaw + 90.0f)<0.001){
        Axis_move_to_target_position(current_x+1,current_y);
        Axis_move_to_target_position(current_x,current_y-3);
        Axis_move_to_target_position(current_x-1,current_y);
    }

    else if(abs(Current_Yaw)<0.001){
        Axis_move_to_target_position(current_x,current_y+1);
        Axis_move_to_target_position(current_x+3,current_y);
        Axis_move_to_target_position(current_x,current_y-1);
    }

    else if(abs(Current_Yaw - 180.0f)<0.001){
        Axis_move_to_target_position(current_x,current_y+1);
        Axis_move_to_target_position(current_x-3,current_y);
        Axis_move_to_target_position(current_x,current_y-1);
    }
    printf("start to return\n");
    printf("current_x:%d,current_y:%d,tar_x:%d,tar_y:%d\n",current_x,current_y,init_tar_x,init_tar_y);
    Axis_Path_Planning(map1,(Point){init_tar_x,init_tar_y});

}