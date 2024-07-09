#ifndef __CAR_H__
#define __CAR_H__

#include "headfile.h"
#include "PID.h"

/**
 * 车辆速度单位测试数据
 * 速度40--26cm/s 1cm--
 * 速度154--10dm/s
 * 速度77--0.5m/s
*/

typedef struct{
    int Speed_y;
    int Speed_omega;

}Car_Speed;

extern Pos_PID_t Motor_Con_R;
extern Pos_PID_t Motor_Con_L;

extern Pos_PID_t angle_con;
extern Pos_PID_t speed_con;

extern Car_Speed Speed;

void car_init();
void car_run();
void Car_Chang_Speed(int speed_y,int omega);
void Car_DisMotion(int dy,int dt,int target_x,int target_y);
void Car_Change_Yaw(float yaw);


#endif // !__CAR_H__