#ifndef __APP_LIDAR_CAR_H_
#define __APP_LIDAR_CAR_H_

#include "AllHeader.h"


void App_Print_Data(void);
float App_Data_Avg(uint16_t start,uint16_t end);
float App_Data_Min(uint16_t start,uint16_t end);
void Car_Avoid(void);
void Car_Alarm(void);
void Car_Follow(void);
void Car_Patrol(void);

void Get_DIS_Group(void);

void LiDar_Straight(void) ;
void Set_lidartrack_speed(void);
float Distance_Adjust_PID(float Current_Distance,float Target_Distance);//距离调整PID  Distance Adjustment PID


void Set_lidartrack_speed_wall(void);
void LiDar_Straight_wall(void);
float Distance_Adjust_PID_wall(float Current_Distance,float Target_Distance);//沿着墙边走  Walk along the wall
#endif

