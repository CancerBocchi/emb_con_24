#ifndef __COORDINATE_H___
#define __COORDINATE_H___

#include "headfile.h"
#include "a_star.h"

extern float current_vy;
extern float current_vx;

extern int current_x;
extern int current_y;
extern float Current_Yaw;

extern int current_target_x;
extern int current_target_y;

extern int patrol_empty_flag;

extern int map[ROW][COL];

extern int patrol_begin_flag;

#define IsPointInAxis(p)    (p.x < ROW && p.x>=0 && p.y < COL && p.y>=0)
#define IsPointAvailable(p) (IsPointInAxis(p) && map[p.x][p.y] == 0)
#define IsPatrolEmpty       (patrol_empty_flag)

void Axis_move_to_target_position(int target_x,int target_y);
void Axis_patrol_run();
float my_atan2(float y, float x, int infNum);
float my_sqrt(float x);
void Axis_patrol_reset();
void Axis_set_patrolNode(Point* patrol,int num);
void Axis_set_map(int map_input[ROW][COL]);
void Axis_Path_Planning(int map,Point target);
void Axis_set_ob(Point* obp,int num);
void Axis_reset_map();


#endif

