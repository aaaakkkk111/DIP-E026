#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Normal;//Normal; //模式为正常


//模式选择 用手拧轮子来进行模式切换
//Mode selection: Use the hand to twist the wheel to switch modes
void Mode_select(void)
{
	int16_t mode_cnt = 0;
	OLED_Draw_Line("1.Standard Mode", 1, true, true); 

	while(!Key1_State(1)) 
	{
		mode_cnt +=Read_Encoder(MOTOR_ID_ML);
		mode_cnt +=-Read_Encoder(MOTOR_ID_MR);
		car_mode(mode_cnt);//模式选择 Mode selection
		show_mode_oled();//oled显示模式 oled display mode
		
	}
	
	Set_Mid_Angle();//模式设置好后,设置机械中值 After setting the mode, set the mechanical median value
	Set_angle();//设置跌倒倾角 Set the inclination angle for falls
	Set_control_speed();//设置遥控的速度 Set the speed of the remote control


	Set_PID();//某些模式的需要特殊设置一下平衡pid Some modes require special settings for balancing PID

}


void car_mode(int16_t cnt)
{
    static int16_t cnt_old=0;
    static const Car_Mode supported[3]={Normal,Weight_M,Test_Mode};
    u8 index=0;
    if(myabs(myabs(cnt)-myabs(cnt_old))<=250) return;
    if(mode==Weight_M) index=1; else if(mode==Test_Mode) index=2;
    index=(u8)((index+(cnt<cnt_old?2:1))%3);
    mode=supported[index]; cnt_old=cnt;
}


//根据模式设置机械中值 Set the mechanical median according to the mode
void Set_Mid_Angle(void){Mid_Angle=0;}

//遥控的速度初始化
//Speed initialization of remote control
void Set_control_speed(void){Car_Target_Velocity=30;Car_Turn_Amplitude_speed=36;}

//设置跌倒的倾角
//Set the inclination angle for falls
void Set_angle(void){angle_max=40;}


extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd; //引入立直环、速度环、转向环 //Introduce vertical rings, speed rings, and steering rings
void Set_PID(void)
{
    if(mode==Weight_M){Balance_Kp=9600;Balance_Kd=75;Velocity_Kp=7000;Velocity_Ki=35;Turn_Kp=1400;Turn_Kd=20;}
    else{Balance_Kp=9600;Balance_Kd=48;Velocity_Kp=6200;Velocity_Ki=31;Turn_Kp=1700;Turn_Kd=20;}
}

