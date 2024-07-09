#include "coordinate.h"
#include "a_star.h"

#define PI 3.1415926f

//常规速度 单位cm/s
#define NORMAL_SPEED 20
#define SLOW_SPEED 15
#define FAST_SPEED 35

#define IsNode(p) (p.x!=-1&&p.y!=-1)

//当前坐标系
int current_x = 0;
int current_y = 0;
int current_target_x = 0;
int current_target_y = 0;
float Current_Yaw = 0;

float current_vy = 0;
float current_vx = 0;

Point patrol_node[20];
//存储当前地图
int map[ROW][COL];
//b表
int break_flag = 0;

float my_atan2(float y, float x, int infNum)
{
    int i;
    float z = y / x, sum = 0.0f,temp;
    float del = z / infNum;

    for (i = 0; i < infNum;i++) {
        z = i * del;
        temp = 1 / (z * z + 1) * del;
        sum += temp;
    }
    if (x>0)
        return sum;
    else if (y >= 0 && x < 0)
        return sum + PI;
    else if (y < 0 && x < 0)
        return sum - PI;
    else if (y > 0 && x == 0)
        return PI / 2;
    else if (y < 0 && x == 0)
        return -1 * PI / 2;
    else
        return 0;
}

float my_sqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy

    return 1/x;
}

/**
 * @brief 跑点函数 单位坐标位置为2dm
 * 
 * @param target_x 目标坐标 x
 * @param target_y 目标坐标 y
 * 
*/
void Axis_move_to_target_position(int target_x,int target_y){
    
    //获取改变的坐标
    int dx = target_x - current_x;
    int dy = target_y - current_y;

    current_target_x = target_x;
    current_target_y = target_y;
    //若两次坐标没有变换 则退出函数
    if(dx == 0&&dy == 0)
        return;

    //获取航向角+更改航向角  航向角单位为角度
    printf("cac atan\n");
    Current_Yaw = my_atan2(dy,dx,1000)/PI/2*360;
    printf("yaw change\n");
    Car_Change_Yaw(Current_Yaw);
    printf("mysqrt\n");
    float distance = my_sqrt(dx*dx+dy*dy);
    printf("dt\n");
    //计算时间 单位s 
    float dt = distance*20/(NORMAL_SPEED);

    printf("dt:%f,yaw:%f,anref:%f,tar:(%d,%d),cur:(%d,%d)\n",dt,Current_Yaw,angle_con.Ref,target_x,target_y,current_x,current_y);

    //注意输入单位为 cm 和 10ms
    Car_DisMotion((float)(distance*20),(float)(dt*100),target_x,target_y);

    osDelay(50);

    if(car_state == State_patrol && !patrol_flag){
        return;
    }

    current_x = target_x;
    current_y = target_y;
}

/**
 * @brief 重置巡逻数组
 * 
 */
int patrol_empty_flag = 0;
void Axis_patrol_reset(){
    for(int i = 0;i<20;i++){
        patrol_node[i].x = -1;
        patrol_node[i].y = -1;
    }
    patrol_empty_flag = 1;
}

/**
 * @brief 设置巡逻节点
 * 
 * @param patrol 节点数组
 * @param num 节点数量
 */
void Axis_set_patrolNode(Point* patrol,int num){
    for(int i = 0;i<num;i++){
        patrol_node[i] = patrol[i];
    }
    patrol_empty_flag = 0;
}

/**
 * @brief 设置地图
 * 
 * @param map_input 地图数组
 */
void Axis_set_map(int map_input[ROW][COL]){
    for(int i = 0;i<ROW;i++){
        for(int j = 0;j<COL;j++){
            map[i][j] = map_input[i][j];
        }
    }
}

void Axis_set_ob(Point* obp,int num){
    for(int i = 0;i<num;i++){
        map[obp[i].x][obp[i].y] = 1;
    }

}

void Axis_reset_map(){
    for(int i = 0;i<ROW;i++){
        for(int j = 0;j<COL;j++){
            map[i][j] = 0;
        }
    }
}

/**
 * @brief 路径规划，使用a*规划算法从一点移动到另一点
 * 
 * @param map 地图
 * @param target 目标点
 */
void Axis_Path_Planning(int map,Point target){

    if(target.x == current_x && target.y == current_y){
        return;
    }
    a_star_search(map,(Point){current_x,current_y},target);
    int num = find_node((Point){current_x,current_y},target);
    printf("%d\n",num);
    for(int i = num-1;i>=0;i--){
        printf("%d:(%d,%d)\n",i,return_node[i].x,return_node[i].y);
    }
    //返回初始点
    for(int i = num-1;i>=0;i--){
        Axis_move_to_target_position(return_node[i].x,return_node[i].y);
    }
}


/**
 * @brief 巡逻函数
 * 
 */
int patrol_begin_flag = 0;
void Axis_patrol_run(){
    
    int i = 0;
    //初始标志位
    if(!patrol_begin_flag){
        Axis_Path_Planning(map,patrol_node[0]);
        //当发现退出巡线时
        if(car_state == State_patrol && !patrol_flag){
            patrol_begin_flag = 0;
            return;
        }
        patrol_begin_flag = 1;
    }

    while(IsNode(patrol_node[i]) && i<=19){
        Axis_move_to_target_position(patrol_node[i].x,patrol_node[i].y);
        //巡逻过程中发现巡线模式停止,
        if(car_state == State_patrol && !patrol_flag){
            patrol_begin_flag = 0;
            return;
        }
        i++;
    }
    //从现在的位置路径规划移动向目的坐标
    Axis_Path_Planning(map,patrol_node[0]);
}




