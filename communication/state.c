#include "state.h"

uint8_t buffer[256];                //用于存储接收到的数据
uint8_t command[20];                //用于存储解析出的指令

Point mapbuffer[100];          //map数据缓冲区
int obp_num_buffer;             //障碍数量缓冲区

uint8_t command_recieve_flag = 0;   //接收到指令标志位

current_State car_state;            

current_State state_buffer;         //状态缓冲区
uint8_t State_Change_Flag;          //状态切换标志位
//路径规划目标点位标志位
Point tar_buffer;
//巡逻开关缓冲区
uint8_t patrol_switch_buffer;
//巡逻路线缓冲区
uint8_t patrol_num;
Point patrol_buffer[20];

/**
 * @brief 得到命令
 * 
 * @return int 发现指令 返回指令的长度，空包则返回负数
 */
int State_Get_Command(){
    int i = 0;
    while(buffer[i]!=' '){
        command[i] = buffer[i];
        i++;
        if(i>=256)
            return -1;
    }
    command[i+1] = '\0';
    return i;
}

void State_copy_buffer(uint8_t bufin[256]){
    for(int i = 0;i<256;i++){
        buffer[i] = bufin[i];
    }
}

//方向标志
#define Direction_0     0
#define Direction_90    90
#define Direction_180   180
#define Direction_270   270

int remote_con_direction = 0;
int changed_axis = 0;
/**
 * @brief 遥控函数，接收到有没有位移则转向，有位移则运行定点跑函数
 * 
 */
void remote_con_run(){
    if(command_recieve_flag && !strcmp(command,CMD3_remotecon)){
        printf("rmcon:start to move axis:%d,direction:%d\n",changed_axis,remote_con_direction);
        //没有位移
        if(changed_axis == 0){
            Car_Change_Yaw(remote_con_direction);
        }
        //有位移
        else{
            int tar_x = current_x,tar_y = current_y;
            switch (remote_con_direction)
            {
            case Direction_0:
                tar_x+=changed_axis;
                break;
            case Direction_90:
                tar_y+=changed_axis;
                break;
            case Direction_180:
                tar_x-=changed_axis;
                break;
            case Direction_270:
                tar_y-=changed_axis;
                break;
            }
            // Dtof_Flag = 0;
            // // while(Dtof_Flag == 0);
            // if(avoid_flag == 1){
            //     printf("find bariar forward\n");
            //     osSemaphoreRelease(avoid_sem);
            //     osSemaphoreAcquire(avoid_sem,osWaitForever);
            // }
            printf("start to move\n");
            Axis_move_to_target_position(tar_x,tar_y);
        }

        command_recieve_flag = 0;
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
    }
    else{
        return;
    }
}

/**
 * @brief 初始化运行
 * 
 */
void init_run(){
    if(command_recieve_flag){
        if(!strcmp(command,CMD2_map)){
            // 设置当前地图
            printf("init:map_init!\n");
            Axis_set_ob(mapbuffer,obp_num_buffer);
            car_state = State_remote_control;
        }
        else if(!strcmp(command,CMD1_modechang)){
            Axis_reset_map();
            Axis_patrol_reset();
        }
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
        command_recieve_flag = 0;
    }
    else{
        return;
    }
}


void path_plan_run(){
    if(command_recieve_flag){
        if(!strcmp(command,CMD4_path_plan)){
            //检测点是否有效
            printf("pathplan:start");
            if(IsPointAvailable(tar_buffer)){
                Axis_Path_Planning(map,tar_buffer);
                tar_buffer.x = -1;
                tar_buffer.y = -1;
            }
            else{
                tar_buffer.x = -1;
                tar_buffer.y = -1;
                printf("pathplan:error target!\n");
            }
        }
        command_recieve_flag = 0;
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
    }
}

uint8_t patrol_flag = 0;
void patrol_run(){

    //未开启巡逻
    if(command_recieve_flag){
        if(!strcmp(command,CMD5_setpatrolpath)){
            printf("patrol:reset patrol point\n");
            Axis_patrol_reset();
            Axis_set_patrolNode(patrol_buffer,patrol_num);
        }
        command_recieve_flag = 0;
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
    }
    if(patrol_flag){
        Axis_patrol_run();
    }

    
   
}

int copy_info(char* buf,int num){
    int i = num;
    int j = 0;
    while(buffer[i] != ' ' && buffer[i] != '\0'){
        buf[j] = buffer[i];
        i++;
        j++;
    }
    return i;
}
/**
 * @brief 指令解析
 * 
 */
void State_command_deal(){
    int num = State_Get_Command();
    // printf("CurCommand:%s\n",command);
    if(num == -1)
        return;
    //模式切换
    if(!strcmp(command,CMD1_modechang)){
        
        state_buffer = buffer[num+1];
        State_Change_Flag = 1;
        printf("modechange:%c\n",state_buffer);
    }
    //地图指令
    else if(!strcmp(command,CMD2_map)){
        char buf[20] = "\0";
        num = copy_info(buf,num+1);
        obp_num_buffer = atoi(buf);
        memset(buf,'\0',20);
        for(int i = 0;i<obp_num_buffer;i++){
            num = copy_info(buf,num+1);
            mapbuffer[i].x = buf[0] - 48;
            mapbuffer[i].y = buf[1] - 48;
            memset(buf,'\0',20);
            printf("mapcmd:point%d (%d,%d)\n",i,mapbuffer[i].x,mapbuffer[i].y);
        }
    }
    //遥控信息
    else if(!strcmp(command,CMD3_remotecon)){
        //获取方向信息
        char buf[20] = "\0";
        num = copy_info(buf,num+1);
        remote_con_direction = atoi(buf);
        // //获取移动坐标数信息
        memset(buf,'\0',20);
        num = copy_info(buf,num+1);
        changed_axis = atoi(buf);
        printf("remotecon:dir:%d,axis:%d\n",remote_con_direction,changed_axis);
    }
    //路径规划
    else if(!strcmp(command,CMD4_path_plan)){
        //获取x轴
        char buf[20] = "\0";
        num = copy_info(buf,num+1);
        tar_buffer.x = buf[0] - 48;
        tar_buffer.y = buf[1] - 48;
        printf("pathplan:(%d,%d)\n",tar_buffer.x,tar_buffer.y);
    }
    //设置巡逻点
    else if(!strcmp(command,CMD5_setpatrolpath)){
        char buf[20] = "\0";
        num = copy_info(buf,num+1);
        patrol_num = atoi(buf);
        for(int i = 0;i<patrol_num;i++){
            num = copy_info(buf,num+1);
            patrol_buffer[i].x = buf[0] - 48;
            patrol_buffer[i].y = buf[1] - 48;
            memset(buf,'\0',20);
            printf("patrolpath:point%d (%d,%d)\n",i,patrol_buffer[i].x,patrol_buffer[i].y);
            
        }
    }
    else if(!strcmp(command,CMD6_patrolswtich)){
        patrol_switch_buffer = buffer[num+1] - 48;
        patrol_flag = (patrol_switch_buffer && !IsPatrolEmpty)?1:0;
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
        printf("patrol_swtich:%d\n",patrol_flag);
    }
    else{
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
        return;
    }
        

    command_recieve_flag = 1;

}

/**
 * @brief 状态机
 * 
 */
void State_Machine(){
    switch (car_state)
    {
    case State_Init:
        init_run();
        break;
    
    case State_path_plan:
       path_plan_run();
        break;
    
    case State_remote_control:
        remote_con_run();
        break;

    case State_patrol:
        patrol_run();
        break;
    }
    //状态切换
    if(State_Change_Flag){
        car_state = state_buffer;
        State_Change_Flag = 0;
        memset(command,0,sizeof(command));
        memset(buffer,0,sizeof(buffer));
        printf("Current_Mode:%c\n",car_state);
    }

    // printf("--------------------------------\n");
    // printf("CurrentState:%c\n",car_state);
    // printf("State_Change_Flag:%d\n",State_Change_Flag);
    // printf("patrol_flag:%d\n",patrol_flag);
    // printf("Command_rec_flag:%d\n",command_recieve_flag);
    // printf("patrol_begin_flag:%d\n",patrol_begin_flag);
}

void State_Machine_Init(){
    car_state = State_Init;
    // car_state = State_remote_control;
    State_Change_Flag = 0;
    command_recieve_flag = 0;
}
