#ifndef __STATE_H__
#define __STATE_H__

#include "headfile.h"


#define CMD1_modechang              "modechange"
#define CMD1_info_rmcon             'R'
#define CMD1_info_patrol            'P'
#define CMD1_info_pathplan          'M'

#define CMD2_map                    "map"

#define CMD3_remotecon              "remotecon"
#define CMD4_path_plan              "pathplan"

#define CMD5_setpatrolpath          "patrolpath"
#define CMD6_patrolswtich           "patrolswitch"

/**
 * @brief 一共三种状态
 * 
 * State_remote_control 手机端遥控
 * State_patrol 自动巡逻
 * State_path_plan  手机端控制终点，自动路径规划
 */
typedef enum{
    State_remote_control = 'R',
    State_patrol = 'P',
    State_path_plan = 'M',
    State_Init = 'I',

}current_State;

extern uint8_t buffer[256];

extern uint8_t State_Change_Flag;

extern current_State state_buffer;         //状态缓冲区
extern uint8_t State_Chdnge_Flag;          //状态切换标志位

extern current_State car_state; 

extern uint8_t patrol_flag;         //巡逻标志位

void State_copy_buffer(uint8_t bufin[256]);
void State_command_deal();
void State_Machine();
void State_Machine_Init();

#endif // !__STATE_H__