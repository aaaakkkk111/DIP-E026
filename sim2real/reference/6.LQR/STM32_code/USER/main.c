/**********
***Shenzhen Yabo Intelligent Technology Co., Ltd.
***Author: lly
***Revision: First revision
***Time: 20240827
***Note: This LQR program is only suitable for the basic version, and accessories can only be installed with Bluetooth and ultrasonic, and other accessories need to be removed. Otherwise, the balance will be affected
***深圳市亚博智能科技有限公司
***作者:lly
***修订：第一次修订
***时间:20240827
***注意:LQR此程序只适配基础版，配件只能安装蓝牙和超声波，其它配件需要一律拆除。否则影响平衡
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


char showbuf[20]={'\0'};

extern u8 newLineReceived;




int main(void)
{	
	
	bsp_init();
	
	MPU6050_EXTI_Init();					//此中断服务函数放到最后 //This interrupt service function is placed last
	
	OLED_Draw_Line("put down key start!", 1, true, true); 

	while(!Key1_State(1));
	Stop_Flag = 0; //开始控制 //Start controlling

	
	OLED_Draw_Line("start control!", 1, true, true); 
	


	while(1)
	{
		
		if (newLineReceived) //蓝牙遥控服务  //Bluetooth remote control service
		{
			ProtocolCpyData();
			Protocol();
		}
		
		
		sprintf(showbuf,"dis = %dmm    ",g_distance);
		OLED_Draw_Line(showbuf, 2, false, true);
		sprintf(showbuf,"angle = %.2f  ",Angle_Balance);
		OLED_Draw_Line(showbuf, 3, false, true); 
		
//		sprintf(showbuf,"LS = %.2f  ",Velocity_Left);
//		OLED_Draw_Line(showbuf, 3, false, true);
	
	}
}


