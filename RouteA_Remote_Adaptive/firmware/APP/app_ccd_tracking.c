#include "app_ccd_tracking.h"


#define Track_CCDSpeed 30 //25  速度不能太高 The speed cannot be too high
void Set_CCDtrack_speed(void)
{

	Move_X = Track_CCDSpeed;       
}



#define CCD_Trun_KP (11) // 0-25
#define CCD_Trun_KD (0.1) // 0-2
#define CCD_Minddle  64 
 
//位置式pid  Positional pid
int Turn_CCD_PD(float gyro)
{

	int CCDTurn = 0;
	float err = 0;	  
	
	err=CCD_Zhongzhi-CCD_Minddle;
	
	CCDTurn=err*CCD_Trun_KP+gyro*CCD_Trun_KD;
	
	return CCDTurn;

}

