#include "car.h"

//将实际单位转换为车辆单位 cm to 车辆单位
#define Car_from_cm(dy) (dy*1.5384f) 
#define my_abs(i)       (i = i<0?-i:i)

//PID
Pos_PID_t Motor_Con_R;
Pos_PID_t Motor_Con_L;
//角度闭环控制
Pos_PID_t angle_con;

//车辆速度控制
Car_Speed Speed;

void car_init(){
    Pos_PID_Init(&Motor_Con_R,1,0.5,0);
    Motor_Con_R.Output_Max = 50;
    Motor_Con_R.Output_Min = -50;
    Motor_Con_R.Value_I_Max = 200;
    Motor_Con_R.Ref = 0;

    Pos_PID_Init(&Motor_Con_L,1,0.5,0);
    Motor_Con_L.Output_Max = 50;
    Motor_Con_L.Output_Min = -50;
    Motor_Con_L.Value_I_Max = 200;
    Motor_Con_L.Ref = 0;

    Pos_PID_Init(&angle_con,-3,0,0);
    angle_con.Output_Max = 30;
    angle_con.Output_Min = -30;
    angle_con.Value_I_Max = 200;
    angle_con.Ref = 0;

    init_wheel_codec();
    init_car_drive();
    car_stop();
    osDelay(100);
    MPU6050Init();
}

/**
 * @brief 将车的速度解算到电机
 * 
*/
void Car_SpeedSolution(){
    Motor_Con_L.Ref = Speed.Speed_y + Speed.Speed_omega;
    Motor_Con_R.Ref = Speed.Speed_y - Speed.Speed_omega;

}

/**
 * @brief 车辆速度控制接口
 * 
*/
void car_run(){
    //角度闭环
    Att_GetYaw();
    int omega = Pos_PID_Controller(&angle_con,yaw);
    Car_Chang_Speed(Speed.Speed_y,omega);
    //速度解算
    Car_SpeedSolution();
    //获取编码器数据
    int left_speed = get_encoder_left();
    int right_speed = get_encoder_right();
    //获取PID输出的值
    int left_pid = Pos_PID_Controller(&Motor_Con_L,left_speed);
    int right_pid = Pos_PID_Controller(&Motor_Con_R,right_speed);
    //输入出到电机
    car_drive(0,left_pid);
    car_drive(1,right_pid);
    // printf("%.1f,%.1f,%.1f\n",angle_con.Ref,yaw,Speed.Speed_omega);
    usleep(40000); // 10ms delay
}

/**
 * @brief 外部改变车速度的接口
 * 
 * @param speed_y
 * @param omega 
 *
*/
void Car_Chang_Speed(int speed_y,int omega){
    Speed.Speed_omega = omega;
    Speed.Speed_y = speed_y;
}

/**
 * @brief 车定点跑函数
 * 
 * @param dy 车辆位移    cm为单位
 * @param dt 需要的时间  10ms为单位
*/

uint8_t IsMultiple(int tar,int mul){
    int a = tar/mul;
    if(a*mul == tar)
        return a;
    return 0;
}

void Car_DisMotion(int dy,int dt,int target_x,int target_y){
    float vy = -Car_from_cm(dy)/(float)dt*100.0f; 

    Car_Chang_Speed((int)vy,Speed.Speed_omega);
    // 延时dt个毫秒
    for(int i = 1;i<=dt;i++){
        int mul = IsMultiple(i,100);
        if(mul){
            if(abs(Current_Yaw - 90.0f)<0.001)
                current_y+=1;
            
            else if(abs(Current_Yaw + 90.0f)<0.001)
                current_y-=1;

            else if(abs(Current_Yaw)<0.001)
                current_x+=1;
            
            else if(abs(Current_Yaw - 180.0f)<0.001)
                current_x-=1;

            printf("Current_x:%d,Current_y:%d\n",current_x,current_y);
            //发现前面有障碍
            if(avoid_flag && (car_state == State_patrol||car_state == State_path_plan)){
                Car_Chang_Speed(0,Speed.Speed_omega);
                printf("Get into avoid_sem:tarx:%d,tary:%d\n",current_target_x,current_target_y);
                osDelay(10);
                osSemaphoreRelease(avoid_sem);
                osSemaphoreAcquire(avoid_sem,osWaitForever);
                return;
            }
            //发现停止 巡逻状态
            if(car_state == State_patrol && !patrol_flag){
                Car_Chang_Speed(0,Speed.Speed_omega);
                return;
            }
        }
        usleep(100);
    }

    Car_Chang_Speed(0,Speed.Speed_omega);
}

/**
 * @brief 改变航向角的函数
*/
void Car_Change_Yaw(float taryaw){

    int i = 0;
    float Ref = angle_con.Ref;
    printf("taryaw:%f,cur:%f\n",taryaw,Ref);
    while(Ref>180){
        Ref -= 360;
        printf("Ref:%f\n",Ref);
        i++;
    }
    while(Ref<-180){
        Ref += 360;
        printf("angle +\n");
        i--;
    }
    //当角度相差无几时，直接退出
    if(abs(taryaw - Ref)<0.00001){
        Ref+=360*i;
        return;
    }

    if(taryaw - Ref <= 180 && taryaw - Ref >= -180)
        Ref = taryaw;
    else if(taryaw - Ref > 180)
        Ref = taryaw - 360;
    else if(taryaw - Ref < -180)
        Ref = taryaw + 360;

   
    angle_con.Ref = Ref + 360*i;
    //修正误差，并且给足时间转向
    yaw += 0.5;
    osDelay(150);
        
}