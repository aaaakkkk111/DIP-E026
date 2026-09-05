/**
* @par Copyright (C): 2018-2028, Shenzhen Yahboom Tech
* @file         // main.c
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 程序入口 Program entry
* @details      
* @par History  // 修改历史记录列表，每条修改记录应包括修改日期、修改者及
*               // 修改内容简述  Modification history list, each modification record should include the modification date, modifier and a brief description of the modification content
*/ 

#include "AllHeader.h"
#include "intsever.h"
//注意:操作蜂鸣器的时候，要判断是否处于正常电压
//Attention: When operating the buzzer, check if it is at normal voltage

uint8_t GET_Angle_Way=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波  //Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//平衡倾角 平衡陀螺仪 转向陀螺仪 //Balance tilt angle balance gyroscope steering gyroscope
int Motor_Left,Motor_Right;                 	  		//电机PWM变量 //Motor PWM variable
int Temperature;                                		//温度变量 		//Temperature variable
float Acceleration_Z;                           		//Z轴加速度计  //Z-axis accelerometer
int Mid_Angle;                          						//机械中值  //Mechanical median
float Move_X,Move_Z; //Move_X:前进速度  Move_Z：转向速度  //Move_X: Forward speed Move_Z: Steering speed
u8 Stop_Flag = 1; //0:开始 1:停止  //0: Start 1: Stop


enCarState g_newcarstate = enSTOP; 

char showbuf[20]={'\0'};

int main(void)
{	
	Mid_Angle = 0; //根据小车来获取 //Obtain based on the car
	
	
	bsp_init();
	
	MPU6050_EXTI_Init();					//此中断服务函数放到最后 //This interrupt service function is placed last
	
	OLED_Draw_Line("put down key start!", 1, true, true); 

	while(!Key1_State(1));
	Stop_Flag = 0; //开始控制 //Start controlling

	
	OLED_Draw_Line("wave avoid start!", 1, true, true); 
	


	while(1)
	{
		APP_avoid();//避障 Obstacle avoidance
		
		sprintf(showbuf,"dis = %d mm  ",g_distance);
		OLED_Draw_Line(showbuf, 2, false, true); 
		sprintf(showbuf,"angle = %.2f  ",Angle_Balance);
		OLED_Draw_Line(showbuf, 3, false, true); 
	
	}
}


