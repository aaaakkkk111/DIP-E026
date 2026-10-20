#include "app_ele_tracking.h"


#define Track_ELESpeed 35
void Set_eletrack_speed(void)
{
	if(ele_seat==0)//不在磁场内 Not within the magnetic field
	{
		Move_X = 0;//停车 stop
		return;
	}
	
	Move_X = Track_ELESpeed;       
}



#define ELE_Trun_KP (25) //25  0-25-不加d的值  0-25-value without adding d
#define ELE_Trun_KD (0.1) //0.1 0-2
#define ELE_Minddle  135 //此值需要根据自己的赛道调 This value needs to be adjusted according to one's own track
//130 150:能跑2个方向  Can run in 2 directions

//位置式pid  Positional pid
int Turn_ELE_PD(float gyro)
{
	if(ele_seat==0)//不在磁场内  Not in magnetic field
	{
		return 0;
	}
	
	int ELETurn = 0;
	float err = 0;	  
	
	err=ele_seat-ELE_Minddle;
	
	ELETurn=err*ELE_Trun_KP+gyro*ELE_Trun_KD;
	
	return ELETurn;

}

